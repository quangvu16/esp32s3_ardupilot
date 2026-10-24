import SwiftUI

/// The LD14 scan from above (car in the middle, front up) and the hoverboard_laser
/// behaviours: Avoid, Follow and Warn.
struct LidarView: View {
    @EnvironmentObject var m: AppModel
    @State private var rangeM: Double = 4

    var body: some View {
        _ = m.tick
        let b = m.behaviours
        let cmd = m.lidar.last
        return ScrollView {
            VStack(spacing: 10) {
                ScanCanvas(rangeM: rangeM)
                    .aspectRatio(1, contentMode: .fit)
                    .frame(maxWidth: 520)
                    .onTapGesture { rangeM = rangeM == 2 ? 4 : rangeM == 4 ? 8 : 2 }
                Text(info()).font(.caption).foregroundStyle(.secondary)

                Picker("Behaviour", selection: Binding(get: { b.mode }, set: { m.setLidarMode($0) })) {
                    ForEach(LidarBehaviours.Mode.allCases) { Text($0.rawValue).tag($0) }
                }
                .pickerStyle(.segmented)

                Text(hint(b.mode)).font(.callout).foregroundStyle(.secondary).frame(maxWidth: .infinity, alignment: .leading)

                HStack {
                    Button(m.lidar.running ? "Stop" : "Start") { m.lidar.running ? m.stopLidar() : m.startLidar() }
                        .buttonStyle(.borderedProminent)
                        .tint(m.lidar.running ? .bad : .good)
                    Toggle("Sound", isOn: $m.lidarSound).fixedSize()
                    Spacer()
                }
                Text(statusText(cmd))
                    .font(.callout.monospacedDigit())
                    .foregroundStyle(cmd.alarm ? Color.bad : Color.primary)
                    .frame(maxWidth: .infinity, alignment: .leading)

                settings(b.mode)
            }
            .padding(12)
        }
    }

    private func statusText(_ cmd: LidarCommand) -> String {
        if !m.lidar.running { return "\(m.behaviours.mode.rawValue): not running" }
        let why = cmd.why.prefix(1).uppercased() + String(cmd.why.dropFirst())
        return why + (m.lidar.drives ? String(format: "   thr %+.2f  steer %+.2f", cmd.throttle, cmd.steer) : "")
    }

    private func info() -> String {
        guard let s = m.vehicle.scan else { return "No scan yet: needs firmware with the LD14 scan (PRX1_TYPE 19). Tap the scan to zoom." }
        let n = s.nearest(0, 180, m.behaviours.p.minRange)
        return "\(m.vehicle.scanCount) scans · range \(Int(rangeM)) m" +
            (n.map { String(format: " · nearest %.2f m at %+.0f°", $0.range, $0.angle) } ?? " · nothing in range")
    }

    private func hint(_ mode: LidarBehaviours.Mode) -> String {
        let base: String
        switch mode {
        case .off: base = "Shows the scan only. Choose a behaviour from hoverboard_laser."
        case .avoid: base = "laser_Avoidance: drives forward and steers away from obstacles in the front, left and right sectors."
        case .follow: base = "laser_Tracker: follows the nearest object ahead and keeps the set distance from it."
        case .warn: base = "laser_Warning: beeps when something ahead is too close. Doesn't drive."
        }
        return base + (mode == .avoid || mode == .follow
            ? " Drives with stick overrides (arm, then Acro, Manual or Steering). Touching the Drive stick pauses it; Stop or leaving the app stops the car." : "")
    }

    @ViewBuilder private func settings(_ mode: LidarBehaviours.Mode) -> some View {
        let p = Binding(get: { m.behaviours.p }, set: { m.behaviours.p = $0; m.objectWillChange.send() })
        VStack(alignment: .leading, spacing: 8) {
            switch mode {
            case .avoid:
                SliderRow(label: "Speed (stick)", value: p.linear, range: 0.05...0.6)
                SliderRow(label: "Turn (stick)", value: p.angular, range: 0.2...1)
                SliderRow(label: "Response dist (m)", value: p.responseDist, range: 0.3...3)
                SliderRow(label: "Side sector (°)", value: p.laserAngle, range: 25...90, step: 5, format: "%.0f")
            case .follow:
                SliderRow(label: "Keep distance (m)", value: p.followDist, range: 0.3...3)
                SliderRow(label: "Max speed (stick)", value: p.maxLinear, range: 0.05...0.6)
                SliderRow(label: "Max steering (stick)", value: p.maxAngular, range: 0.2...1)
                SliderRow(label: "Search angle (°)", value: p.trackAngle, range: 10...90, step: 5, format: "%.0f")
            case .warn:
                SliderRow(label: "Warn distance (m)", value: p.warnDist, range: 0.2...3)
                SliderRow(label: "Warn angle (°)", value: p.warnAngle, range: 5...60, step: 5, format: "%.0f")
            case .off:
                EmptyView()
            }
            if mode != .off {
                SliderRow(label: "Ignore closer than (m), the car itself", value: p.minRange, range: 0...1)
                Picker("Vehicle", selection: Binding(get: { m.behaviours.p.skidSteer }, set: { m.behaviours.p.skidSteer = $0; m.saveSettings() })) {
                    Text("RC car").tag(false)
                    Text("Skid steer").tag(true)
                }
                .pickerStyle(.segmented)
            }
        }
    }
}

/// One rotation drawn from above: rings every metre, the behaviour's sectors, the returns.
struct ScanCanvas: View {
    @EnvironmentObject var m: AppModel
    let rangeM: Double

    var body: some View {
        _ = m.tick
        let scan = m.vehicle.scan
        let p = m.behaviours.p
        let mode = m.behaviours.mode
        return Canvas { ctx, size in
            let r = min(size.width, size.height) / 2
            let c = CGPoint(x: size.width / 2, y: size.height / 2)
            let k = r / CGFloat(rangeM)
            ctx.fill(Path(ellipseIn: CGRect(x: c.x - r, y: c.y - r, width: 2 * r, height: 2 * r)), with: .color(.black.opacity(0.85)))
            for ring in 1...Int(rangeM) {
                let rr = CGFloat(ring) * k
                ctx.stroke(Path(ellipseIn: CGRect(x: c.x - rr, y: c.y - rr, width: 2 * rr, height: 2 * rr)), with: .color(.gray.opacity(0.5)), lineWidth: 1)
            }
            // point at a left-positive angle (deg) and range (m): front is up
            func pt(_ a: Float, _ d: Float) -> CGPoint {
                let rad = Double(a) * .pi / 180
                return CGPoint(x: c.x - CGFloat(sin(rad)) * CGFloat(d) * k, y: c.y - CGFloat(cos(rad)) * CGFloat(d) * k)
            }
            func wedge(_ from: Float, _ to: Float, _ d: Float, _ hit: Bool) {
                var w = Path()
                w.move(to: c)
                var a = from
                while a <= to { w.addLine(to: pt(a, d)); a += 2 }
                w.closeSubpath()
                ctx.fill(w, with: .color((hit ? Color.red : Color.blue).opacity(0.25)))
            }
            if let s = scan {
                let (a, rr) = s.points(p.minRange)
                func hit(_ lo: Float, _ hi: Float, _ d: Float) -> Bool {
                    a.indices.contains { a[$0] >= lo && a[$0] <= hi && rr[$0] < d }
                }
                switch mode {
                case .avoid:
                    let d = p.responseDist * 1.5
                    wedge(-20, 20, d, hit(-20, 20, d))
                    wedge(20, p.laserAngle, d, hit(20, p.laserAngle, d))
                    wedge(-p.laserAngle, -20, d, hit(-p.laserAngle, -20, d))
                case .follow:
                    wedge(-p.trackAngle, p.trackAngle, p.followDist, false)
                case .warn:
                    wedge(-p.warnAngle, p.warnAngle, p.warnDist, hit(-p.warnAngle, p.warnAngle, p.warnDist))
                case .off: break
                }
                for i in a.indices where Double(rr[i]) <= rangeM {
                    let q = pt(a[i], rr[i])
                    ctx.fill(Path(ellipseIn: CGRect(x: q.x - 3, y: q.y - 3, width: 6, height: 6)), with: .color(.green))
                }
                if mode == .follow, let t = s.nearest(0, p.trackAngle, p.minRange) {
                    let q = pt(t.angle, t.range)
                    ctx.stroke(Path(ellipseIn: CGRect(x: q.x - 9, y: q.y - 9, width: 18, height: 18)), with: .color(.yellow), lineWidth: 2)
                }
            }
            // the car
            var car = Path()
            car.move(to: CGPoint(x: c.x, y: c.y - 12)); car.addLine(to: CGPoint(x: c.x + 8, y: c.y + 10)); car.addLine(to: CGPoint(x: c.x - 8, y: c.y + 10))
            car.closeSubpath()
            ctx.fill(car, with: .color(.white))
        }
    }
}

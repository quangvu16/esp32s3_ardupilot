import SwiftUI

/// Flight Data: HUD, Arm, mode buttons, the map (tap to drive in Guided, or the
/// chosen tap action) and the drive stick.
struct DataView: View {
    @EnvironmentObject var m: AppModel
    @State private var showDrive = false
    @State private var confirmArm = false
    @State private var tapAction = TapAction.guided

    enum TapAction: String, CaseIterable, Identifiable {
        case guided = "Drive here", waypoint = "Add waypoint", home = "Set home", none = "Nothing"
        var id: String { rawValue }
    }

    var body: some View {
        _ = m.tick
        let v = m.vehicle
        return VStack(spacing: 0) {
            HudView().frame(height: 150)
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 6) {
                    Button {
                        if !m.requireLink() { return }
                        if v.armed { v.arm(false) } else { confirmArm = true }
                    } label: { Text(v.armed ? "Disarm" : "Arm").bold().frame(minWidth: 64) }
                        .buttonStyle(.borderedProminent)
                        .tint(v.armed ? .bad : .good)
                    ForEach(modeButtons(), id: \.0) { mode, name in
                        Button(name) { m.setMode(mode) }
                            .buttonStyle(.bordered)
                            .tint(v.customMode == mode ? .accentColor : .gray)
                    }
                    Button { showDrive.toggle(); if !showDrive { m.stickHeld = false } } label: {
                        Label("Drive", systemImage: "gamecontroller")
                    }.buttonStyle(.bordered).tint(showDrive ? .accentColor : .gray)
                }
                .padding(.horizontal, 10).padding(.vertical, 6)
            }
            ZStack(alignment: .bottom) {
                MapPanel { c in mapTap(c) }
                    .overlay(alignment: .topLeading) {
                        Picker("Tap", selection: $tapAction) {
                            ForEach(TapAction.allCases) { Text("Tap: \($0.rawValue)").tag($0) }
                        }
                        .pickerStyle(.menu)
                        .padding(4)
                        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 8))
                        .padding(8)
                    }
                if showDrive { DrivePanel().padding(10) }
            }
        }
        .confirmationDialog("Arm the motors?", isPresented: $confirmArm, titleVisibility: .visible) {
            Button("Arm") { v.arm(true) }
            Button("Force arm (skip checks)", role: .destructive) { v.arm(true, force: true) }
        } message: { Text("Wheels off the ground for the first test.") }
    }

    private func mapTap(_ c: Coord) {
        switch tapAction {
        case .guided: m.goTo(c)
        case .waypoint: m.plan.append(MissionItem(lat: c.lat, lon: c.lon)); m.show("Waypoint \(m.plan.count) added (Plan tab to write it)")
        case .home: if m.requireLink() { m.vehicle.setHome(lat: c.lat, lon: c.lon); m.show("Home moved") }
        case .none: break
        }
    }
}

/// Artificial horizon with heading, speed, throttle and the waypoint distance.
struct HudView: View {
    @EnvironmentObject var m: AppModel
    var body: some View {
        _ = m.tick
        let v = m.vehicle
        return ZStack {
            Canvas { ctx, size in
                let w = size.width, h = size.height
                let pitchPx = CGFloat(v.pitch) * h / 1.2          // ±0.6 rad over the height
                var t = CGAffineTransform(translationX: w / 2, y: h / 2)
                t = t.rotated(by: CGFloat(-v.roll))
                t = t.translatedBy(x: 0, y: pitchPx)
                let big = max(w, h) * 2
                ctx.fill(Path(CGRect(x: -big, y: -big, width: 2 * big, height: big)).applying(t), with: .color(Color(red: 0.30, green: 0.55, blue: 0.85)))
                ctx.fill(Path(CGRect(x: -big, y: 0, width: 2 * big, height: big)).applying(t), with: .color(Color(red: 0.45, green: 0.33, blue: 0.20)))
                var line = Path(); line.move(to: CGPoint(x: -big, y: 0)); line.addLine(to: CGPoint(x: big, y: 0))
                ctx.stroke(line.applying(t), with: .color(.white), lineWidth: 2)
                // the car's level mark
                var mark = Path()
                mark.move(to: CGPoint(x: w / 2 - 40, y: h / 2)); mark.addLine(to: CGPoint(x: w / 2 - 12, y: h / 2))
                mark.move(to: CGPoint(x: w / 2 + 12, y: h / 2)); mark.addLine(to: CGPoint(x: w / 2 + 40, y: h / 2))
                ctx.stroke(mark, with: .color(.yellow), lineWidth: 3)
            }
            VStack {
                HStack {
                    Text(String(format: "%03ld°", (v.heading + 360) % 360)).font(.title3.monospacedDigit().bold())
                    Spacer()
                    Text(v.modeName).font(.title3.bold())
                }
                Spacer()
                HStack(alignment: .bottom) {
                    VStack(alignment: .leading) {
                        Text(String(format: "%.1f m/s", v.groundSpeed)).font(.headline.monospacedDigit())
                        Text("Throttle \(v.throttle)%").font(.caption)
                    }
                    Spacer()
                    VStack(alignment: .trailing) {
                        if v.customMode == 10 || v.customMode == 15 || v.customMode == 11 {
                            Text("WP \(v.missionSeq) · \(v.wpDist) m").font(.headline.monospacedDigit())
                        }
                        Text(String(format: "hAcc %.2f m · HDOP %.1f", v.gpsHAcc, v.gpsHdop)).font(.caption)
                    }
                }
            }
            .foregroundStyle(.white)
            .shadow(color: .black, radius: 1)
            .padding(10)
        }
        .clipped()
    }
}

/// The drive stick: RC override over MAVLink (MANUAL_CONTROL), with a power limit.
struct DrivePanel: View {
    @EnvironmentObject var m: AppModel
    var body: some View {
        HStack(alignment: .center, spacing: 16) {
            VStack(alignment: .leading, spacing: 6) {
                Text("Stick: RC override (Manual, Acro, Steering…). Needs SYSID_MYGCS = 255.")
                    .font(.caption).foregroundStyle(.secondary)
                SliderRow(label: "Power", value: Binding(get: { m.driveLimit }, set: { m.driveLimit = $0; m.saveSettings() }),
                          range: 0.1...1, step: 0.05, format: "%.2f")
                #if os(macOS)
                Text("Or hold the arrow keys / WASD while this window has the focus.").font(.caption).foregroundStyle(.secondary)
                #endif
            }
            JoystickView()
                .frame(width: 150, height: 150)
        }
        .padding(12)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 14))
    }
}

/// A stick: drag from the centre; x = steering (right +), y = throttle (up +). Lets go to zero.
struct JoystickView: View {
    @EnvironmentObject var m: AppModel
    @State private var knob = CGSize.zero

    var body: some View {
        GeometryReader { g in
            let r = min(g.size.width, g.size.height) / 2
            ZStack {
                Circle().fill(Color.gray.opacity(0.25))
                Circle().stroke(Color.gray.opacity(0.6), lineWidth: 2)
                Path { p in p.move(to: CGPoint(x: r, y: 8)); p.addLine(to: CGPoint(x: r, y: 2 * r - 8))
                    p.move(to: CGPoint(x: 8, y: r)); p.addLine(to: CGPoint(x: 2 * r - 8, y: r)) }
                    .stroke(Color.gray.opacity(0.4), lineWidth: 1)
                Circle().fill(Color.accentColor).frame(width: r * 0.6, height: r * 0.6)
                    .offset(knob)
            }
            .frame(width: 2 * r, height: 2 * r)
            .contentShape(Circle())
            .gesture(DragGesture(minimumDistance: 0)
                .onChanged { d in
                    var x = d.location.x - r, y = d.location.y - r
                    let len = hypot(x, y)
                    if len > r { x *= r / len; y *= r / len }
                    knob = CGSize(width: x, height: y)
                    m.stickX = Float(x / r)
                    m.stickY = Float(-y / r)
                    m.stickHeld = true
                }
                .onEnded { _ in
                    knob = .zero
                    m.stickX = 0; m.stickY = 0
                    m.stickHeld = false
                })
        }
    }
}

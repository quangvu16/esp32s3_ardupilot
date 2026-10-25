import SwiftUI

/// The map image, rebuilt only when the map or the costmap changes.
final class IndoorImageCache {
    private var key = -1
    private(set) var image: CGImage?

    func update(_ s: IndoorNav.Snapshot) {
        guard let g = s.grid else { image = nil; key = -1; return }
        let k = s.gridVersion &* 1_000_003 &+ s.costVersion
        if k == key && image != nil { return }
        key = k
        let px = Viewport.rgba(g, s.cost)
        guard let provider = CGDataProvider(data: Data(px) as CFData) else { return }
        image = CGImage(width: g.width, height: g.height, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: g.width * 4,
                        space: CGColorSpaceCreateDeviceRGB(), bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.noneSkipLast.rawValue),
                        provider: provider, decode: nil, shouldInterpolate: false, intent: .defaultIntent)
    }
}

/// Indoor navigation, Nav2 style: map a room, save it, find the car in it, tap a goal or run a patrol.
struct IndoorView: View {
    @EnvironmentObject var m: AppModel
    @State private var vp = Viewport()
    @State private var fitted = false
    @State private var lastDrag: CGPoint?
    @State private var zoomBase = 1.0
    @State private var poseMode = false
    @State private var poseStart: CGPoint?
    @State private var poseEnd: CGPoint?
    @State private var addMode = false
    @State private var goal: (Double, Double)?
    @State private var askSave = false
    @State private var saveName = "room"
    @State private var showLoad = false
    @State private var showWaypoints = false
    @State private var showSettings = false
    @State private var confirmNew = false
    @State private var cache = IndoorImageCache()

    var body: some View {
        _ = m.tick
        let s = m.indoor.snap
        cache.update(s)
        return VStack(spacing: 6) {
            GeometryReader { geo in
                mapCanvas(s, geo.size)
                    .contentShape(Rectangle())
                    .gesture(dragGesture(geo.size))
                    .simultaneousGesture(MagnificationGesture()
                        .onChanged { z in vp.zoom(Double(z) / zoomBase, Double(geo.size.width) / 2, Double(geo.size.height) / 2); zoomBase = Double(z); vp.follow = false }
                        .onEnded { _ in zoomBase = 1 })
                    .simultaneousGesture(SpatialTapGesture(count: 2).onEnded { _ in vp.follow = true })
                    .simultaneousGesture(SpatialTapGesture().onEnded { e in tap(e.location, geo.size) })
            }
            .frame(minHeight: 260)
            .background(Color(red: 0.85, green: 0.87, blue: 0.85))
            .clipShape(RoundedRectangle(cornerRadius: 8))

            Text(s.status).font(.callout.bold()).frame(maxWidth: .infinity, alignment: .leading).lineLimit(2)
            Text(info(s)).font(.caption).foregroundStyle(.secondary).frame(maxWidth: .infinity, alignment: .leading).lineLimit(2)

            ScrollView(.horizontal, showsIndicators: false) {
                HStack {
                    Button("STOP") { m.indoor.stop() }.buttonStyle(.borderedProminent).tint(.bad)
                    Button(m.vehicle.armed ? "Disarm" : "Arm") { m.vehicle.arm(!m.vehicle.armed) }.buttonStyle(.bordered)
                    Toggle(addMode ? "Tap: add waypoint" : "Tap: go here", isOn: $addMode).toggleStyle(.button)
                    Button("Patrol") { m.indoor.startPatrol() }.buttonStyle(.borderedProminent)
                    Button("Waypoints…") { showWaypoints = true }.buttonStyle(.bordered)
                }
            }
            ScrollView(.horizontal, showsIndicators: false) {
                HStack {
                    Button("Map new") { confirmNew = true }.buttonStyle(.bordered)
                    Button("Save map") { saveName = s.mapName ?? "room"; askSave = true }.buttonStyle(.bordered)
                    Button("Load map") { showLoad = true }.buttonStyle(.bordered)
                    Button("Find me") { m.indoor.findMe() }.buttonStyle(.bordered)
                    Toggle("Set pose", isOn: $poseMode).toggleStyle(.button)
                    Button("Settings…") { showSettings = true }.buttonStyle(.bordered)
                    if m.vehicle.paramValue("PRX1_SCAN_FULL") == 0 {
                        Button("Use 1° scans") { m.vehicle.setParam("PRX1_SCAN_FULL", 1) }.buttonStyle(.bordered)
                    }
                }
            }
        }
        .padding(10)
        .alert("Map a new room?", isPresented: $confirmNew) {
            Button("Start") { m.indoor.startMapping(); fitted = false; vp.follow = true }
            Button("Cancel", role: .cancel) {}
        } message: {
            Text("Put the car where it will always start, then drive it slowly round the room (Manual, or the stick on the Data tab). Save when the walls are complete.")
        }
        .alert("Save map", isPresented: $askSave) {
            TextField("name", text: $saveName)
            Button("Save") {
                let n = saveName.trimmingCharacters(in: .whitespaces).replacingOccurrences(of: "[^A-Za-z0-9_-]", with: "_", options: .regularExpression)
                if !n.isEmpty { m.indoor.saveMap(n) }
            }
            Button("Cancel", role: .cancel) {}
        }
        .alert("Go here?", isPresented: Binding(get: { goal != nil }, set: { if !$0 { goal = nil } })) {
            Button("Go") { if let g = goal { m.indoor.goTo(g.0, g.1) }; goal = nil }
            Button("Cancel", role: .cancel) { goal = nil }
        } message: {
            Text(goal.map { String(format: "Drive to %.1f m N, %.1f m E in Guided mode.", $0.0, $0.1) } ?? "")
        }
        .confirmationDialog("Load map", isPresented: $showLoad) {
            ForEach(m.indoor.maps(), id: \.self) { n in Button(n) { m.indoor.loadMap(n); fitted = false; vp.follow = true } }
            Button("Cancel", role: .cancel) {}
        }
        .sheet(isPresented: $showWaypoints) { WaypointsSheet().environmentObject(m) }
        .sheet(isPresented: $showSettings) { IndoorSettingsSheet().environmentObject(m) }
    }

    private func info(_ s: IndoorNav.Snapshot) -> String {
        var parts = [String]()
        parts.append(s.mapName.map { "Map: \($0)" } ?? (s.mode == .mapping ? "Mapping" : "No map"))
        if s.grid != nil && s.mode == .navigation {
            parts.append(s.locInitialised ? String(format: "pose ±%.2f m, match %.0f%%", s.locSpread, s.locScore * 100) : "pose not set")
        }
        if s.active { parts.append(String(format: "%.2f m/s", s.speedCmd)) }
        if let sc = m.vehicle.scan { parts.append(sc.cm.count >= 360 ? "scan 1°" : "scan \(Int(sc.incrementDeg))°") } else { parts.append("no scan") }
        if m.vehicle.paramValue("PRX1_SLAM") == 0 { parts.append("PRX1_SLAM is off: no indoor position") }
        if poseMode { parts.append("Set pose: drag from the car's place the way it faces") }
        return parts.joined(separator: "  ·  ")
    }

    private func tap(_ p: CGPoint, _ size: CGSize) {
        if poseMode { return }
        var v = vp; v.width = Double(size.width); v.height = Double(size.height)
        let x = v.mapX(Double(p.y)), y = v.mapY(Double(p.x))
        guard m.indoor.snap.grid != nil else { m.show("Load or make a map first"); return }
        if addMode { m.indoor.addWaypoint(x, y); return }
        if !m.vehicle.armed { m.show("Arm the car first"); return }
        goal = (x, y)
    }

    private func dragGesture(_ size: CGSize) -> some Gesture {
        DragGesture(minimumDistance: 4)
            .onChanged { g in
                if poseMode {
                    if poseStart == nil { poseStart = g.startLocation }
                    poseEnd = g.location
                    return
                }
                let last = lastDrag ?? g.startLocation
                vp.cy -= Double(g.location.x - last.x) / vp.scale
                vp.cx += Double(g.location.y - last.y) / vp.scale
                vp.follow = false
                lastDrag = g.location
            }
            .onEnded { g in
                lastDrag = nil
                if poseMode, let ps = poseStart {
                    var v = vp; v.width = Double(size.width); v.height = Double(size.height)
                    let dx = Double(g.location.x - ps.x), dy = Double(g.location.y - ps.y)
                    if hypot(dx, dy) > 12 {
                        m.indoor.setPose(v.mapX(Double(ps.y)), v.mapY(Double(ps.x)), atan2(dx, -dy))
                        poseMode = false
                    }
                }
                poseStart = nil; poseEnd = nil
            }
    }

    private func mapCanvas(_ s: IndoorNav.Snapshot, _ size: CGSize) -> some View {
        var v = vp
        v.width = Double(size.width); v.height = Double(size.height)
        if let g = s.grid, !fitted {
            v.fit(g)
            DispatchQueue.main.async { vp = v; fitted = true }
        }
        if v.follow, let p = s.pose { v.cx = p.x; v.cy = p.y }
        let view = v
        let image = cache.image
        let ps = poseStart, pe = poseEnd
        return Canvas { ctx, _ in
            guard let g = s.grid else {
                ctx.draw(Text("No map: Map new, or Load map").foregroundColor(.secondary), at: CGPoint(x: size.width / 2, y: 24))
                return
            }
            if let img = image {
                let left = view.sx(g.originY), top = view.sy(g.originX + Double(g.height) * g.res)
                let cell = g.res * view.scale
                ctx.draw(Image(decorative: img, scale: 1).interpolation(.none),
                         in: CGRect(x: left, y: top, width: Double(g.width) * cell, height: Double(g.height) * cell))
            }
            for i in stride(from: 0, to: s.particlesX.count, by: 3) {
                let r = CGRect(x: view.sx(s.particlesY[i]) - 1.5, y: view.sy(s.particlesX[i]) - 1.5, width: 3, height: 3)
                ctx.fill(Path(r), with: .color(.blue.opacity(0.4)))
            }
            for i in 0..<min(s.scanX.count, s.scanY.count) {
                let r = CGRect(x: view.sx(s.scanY[i]) - 2, y: view.sy(s.scanX[i]) - 2, width: 4, height: 4)
                ctx.fill(Path(ellipseIn: r), with: .color(.red))
            }
            if s.path.count > 1 {
                var p = Path()
                for (k, q) in s.path.enumerated() {
                    let pt = CGPoint(x: view.sx(q.1), y: view.sy(q.0))
                    if k == 0 { p.move(to: pt) } else { p.addLine(to: pt) }
                }
                ctx.stroke(p, with: .color(.blue), style: StrokeStyle(lineWidth: 3, lineCap: .round, lineJoin: .round))
            }
            for w in s.waypoints {
                let c = CGPoint(x: view.sx(w.y), y: view.sy(w.x))
                ctx.fill(Path(ellipseIn: CGRect(x: c.x - 6, y: c.y - 6, width: 12, height: 12)), with: .color(.orange))
                ctx.draw(Text(w.name).font(.caption.bold()), at: CGPoint(x: c.x + 14, y: c.y - 10))
            }
            if let gl = s.goal {
                let c = CGPoint(x: view.sx(gl.y), y: view.sy(gl.x))
                ctx.stroke(Path(ellipseIn: CGRect(x: c.x - 11, y: c.y - 11, width: 22, height: 22)), with: .color(.green), lineWidth: 3)
            }
            if let p = s.pose {
                let len = max(0.35, 22 / view.scale)
                let pts: [(Double, Double)] = [(len * 0.6, 0), (-len * 0.4, len * 0.3), (-len * 0.4, -len * 0.3)]
                var tri = Path()
                for (k, l) in pts.enumerated() {
                    let wpt = p.apply(l.0, l.1)
                    let pt = CGPoint(x: view.sx(wpt.1), y: view.sy(wpt.0))
                    if k == 0 { tri.move(to: pt) } else { tri.addLine(to: pt) }
                }
                tri.closeSubpath()
                ctx.fill(tri, with: .color(.blue)); ctx.stroke(tri, with: .color(.white), lineWidth: 1.5)
            }
            if let a = ps, let b = pe {
                ctx.stroke(Path(ellipseIn: CGRect(x: a.x - 10, y: a.y - 10, width: 20, height: 20)), with: .color(.purple), lineWidth: 3)
                var l = Path(); l.move(to: a); l.addLine(to: b)
                ctx.stroke(l, with: .color(.purple), lineWidth: 3)
            }
            var bar = Path(); bar.move(to: CGPoint(x: 12, y: size.height - 14)); bar.addLine(to: CGPoint(x: 12 + view.scale, y: size.height - 14))
            ctx.stroke(bar, with: .color(.primary), lineWidth: 2)
            ctx.draw(Text("1 m").font(.caption2), at: CGPoint(x: 24, y: size.height - 24))
        }
    }
}

/// The waypoints of the loaded map, in patrol order.
struct WaypointsSheet: View {
    @EnvironmentObject var m: AppModel
    @Environment(\.dismiss) private var dismiss
    var body: some View {
        _ = m.tick
        let wps = m.indoor.snap.waypoints
        return NavigationStack {
            List {
                if wps.isEmpty { Text("No waypoints: turn on \"Tap: add waypoint\" and tap the map.").foregroundStyle(.secondary) }
                ForEach(wps) { w in
                    HStack {
                        Text(w.name).bold()
                        Text(String(format: "%.1f N, %.1f E", w.x, w.y)).foregroundStyle(.secondary)
                        Spacer()
                        Button("Go") { m.indoor.goToWaypoint(w.name); dismiss() }.buttonStyle(.bordered)
                        Button(role: .destructive) { m.indoor.removeWaypoint(w.name) } label: { Image(systemName: "trash") }
                            .buttonStyle(.borderless)
                    }
                }
            }
            .navigationTitle("Waypoints")
            .toolbar {
                ToolbarItem(placement: .cancellationAction) { Button("Close") { dismiss() } }
                ToolbarItem(placement: .destructiveAction) { Button("Delete all") { m.indoor.clearWaypoints() } }
            }
        }
        .frame(minWidth: 360, minHeight: 320)
    }
}

/// Speed and the car's size for the costmap.
struct IndoorSettingsSheet: View {
    @EnvironmentObject var m: AppModel
    @Environment(\.dismiss) private var dismiss
    @State private var speed = 0.4
    @State private var radius = 0.2
    @State private var inflation = 0.55
    var body: some View {
        NavigationStack {
            Form {
                Stepper(String(format: "Speed %.2f m/s", speed), value: $speed, in: 0.1...1.5, step: 0.05)
                Stepper(String(format: "Car radius %.2f m", radius), value: $radius, in: 0.05...1.0, step: 0.01)
                Stepper(String(format: "Keep away from walls %.2f m", inflation), value: $inflation, in: 0.1...2.0, step: 0.05)
                Text("Car radius: half the car's width plus a little; the path keeps the car's centre this far from walls. New values apply when a map is loaded or saved.")
                    .font(.caption).foregroundStyle(.secondary)
            }
            .navigationTitle("Indoor navigation")
            .toolbar {
                ToolbarItem(placement: .confirmationAction) {
                    Button("OK") { m.setIndoorSettings(speed: speed, radius: radius, inflation: inflation); dismiss() }
                }
                ToolbarItem(placement: .cancellationAction) { Button("Cancel") { dismiss() } }
            }
            .onAppear { speed = m.indoor.settings.speed; radius = m.indoor.settings.robotRadius; inflation = m.indoor.settings.inflation }
        }
        .frame(minWidth: 360, minHeight: 260)
    }
}

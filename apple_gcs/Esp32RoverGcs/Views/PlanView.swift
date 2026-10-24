import SwiftUI
import UniformTypeIdentifiers

/// Flight Plan: waypoints (tap the map to add), read/write, Start Auto, .waypoints
/// files, zig-zag lanes, patrol, and the fence editor.
struct PlanView: View {
    @EnvironmentObject var m: AppModel
    @State private var fenceMode = false
    @State private var pickZig: Character? = nil            // "A" or "B" while picking a zig-zag point
    @State private var editIndex: Int?
    @State private var showZigZag = false
    @State private var showPatrol = false
    @State private var importing = false
    @State private var exporting = false
    @State private var circleRadius: Float = 10
    @State private var askCircle = false
    @State private var enablePlan: (text: String, type: Int, action: Int)?

    var body: some View {
        _ = m.tick
        return VStack(spacing: 0) {
            Picker("", selection: $fenceMode) {
                Text("Waypoints").tag(false)
                Text("Fence").tag(true)
            }
            .pickerStyle(.segmented)
            .padding(.horizontal, 10).padding(.vertical, 6)

            MapPanel(editing: true) { c in tap(c) }
                .overlay(alignment: .bottom) {
                    if let z = pickZig {
                        Text("Tap the map to place point \(String(z))").padding(8)
                            .background(.regularMaterial, in: Capsule()).padding(8)
                    }
                }
                .frame(minHeight: 300)
                .layoutPriority(1)

            if fenceMode { fencePanel } else { waypointPanel }
        }
        .sheet(isPresented: Binding(get: { editIndex != nil }, set: { if !$0 { editIndex = nil } })) {
            if let i = editIndex, i < m.plan.count { ItemEditor(index: i) { editIndex = nil } }
        }
        .sheet(isPresented: $showZigZag) { ZigZagSheet(pick: { p in showZigZag = false; pickZig = p }) }
        .sheet(isPresented: $showPatrol) { PatrolSheet() }
        .fileImporter(isPresented: $importing, allowedContentTypes: [.plainText, .data]) { r in
            if case .success(let url) = r, let s = readPickedFile(url) { m.loadWaypoints(s) }
        }
        .fileExporter(isPresented: $exporting, document: TextFile(m.waypointsText()), contentType: .plainText,
                      defaultFilename: "mission.waypoints") { _ in }
        .alert("Circle radius (m)", isPresented: $askCircle) {
            TextField("Radius", value: $circleRadius, format: .number)
            Button("Stay-in") { m.addZone(.circleIn, radius: circleRadius) }
            Button("Stay-out") { m.addZone(.circleOut, radius: circleRadius) }
            Button("Cancel", role: .cancel) {}
        }
        .alert("Enable the fence", isPresented: Binding(get: { enablePlan != nil }, set: { if !$0 { enablePlan = nil } })) {
            Button("Enable") { if let p = enablePlan { m.enableFence(type: p.type, action: p.action) } }
            Button("Cancel", role: .cancel) {}
        } message: { Text(enablePlan?.text ?? "") }
    }

    private func tap(_ c: Coord) {
        if let z = pickZig {
            if z == "A" { m.zigzag.a = c } else { m.zigzag.b = c }
            pickZig = nil
            showZigZag = true
            return
        }
        m.planTap(c, fenceMode: fenceMode)
    }

    // --- waypoints
    private var waypointPanel: some View {
        VStack(spacing: 6) {
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 6) {
                    Button("Read") { if m.requireLink() { m.vehicle.downloadMission() } }
                    Button("Write") { m.writePlan() }.buttonStyle(.borderedProminent)
                    Button("Start Auto") { m.startAuto() }.tint(.good)
                    Button("Zig zag…") { showZigZag = true }
                    Button("Patrol…") { showPatrol = true }
                    Button("Load") { importing = true }
                    Button("Save") { exporting = true }
                    Button("Undo") { if !m.plan.isEmpty { m.plan.removeLast() } }
                    Button("Clear", role: .destructive) { m.plan.removeAll() }
                }
                .buttonStyle(.bordered)
                .padding(.horizontal, 10)
            }
            HStack {
                Text("Mission speed")
                Slider(value: $m.planSpeed, in: 0...5, step: 0.25)
                Text(m.planSpeed == 0 ? "WP_SPEED" : String(format: "%.2f m/s", m.planSpeed)).monospacedDigit().frame(width: 84)
            }
            .font(.callout)
            .padding(.horizontal, 10)
            List {
                if m.plan.isEmpty {
                    Text("Tap the map to add waypoints.").foregroundStyle(.secondary)
                }
                ForEach(Array(m.plan.enumerated()), id: \.element.id) { i, it in
                    Button { editIndex = i } label: {
                        HStack {
                            Text("\(i + 1)").bold().frame(width: 28)
                            Text(itemLabel(it))
                            Spacer()
                            if it.hasPosition {
                                Text(String(format: "%.6f, %.6f", it.lat, it.lon)).font(.caption.monospacedDigit()).foregroundStyle(.secondary)
                            }
                        }
                    }
                    .buttonStyle(.plain)
                }
                .onDelete { m.plan.remove(atOffsets: $0) }
                .onMove { m.plan.move(fromOffsets: $0, toOffset: $1) }
            }
            .listStyle(.plain)
            .frame(minHeight: 110, maxHeight: 220)
        }
    }

    // --- fence
    private var fencePanel: some View {
        VStack(spacing: 6) {
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 6) {
                    Button("+ Stay-in") { m.addZone(.polygonIn) }.tint(.green)
                    Button("+ Stay-out") { m.addZone(.polygonOut) }.tint(.red)
                    Button("+ Circle") { askCircle = true }
                    Button("+ Return") { m.addZone(.returnPoint) }
                    Button("Read") { if m.requireLink() { m.vehicle.downloadFence() } }
                    Button("Write") { m.writeFence() }.buttonStyle(.borderedProminent)
                    Button("Enable…") { enablePlan = m.fenceEnablePlan() }
                    Button("Clear", role: .destructive) { m.fence.removeAll(); m.fenceZone = nil }
                }
                .buttonStyle(.bordered)
                .padding(.horizontal, 10)
            }
            Text("Storage: \(Fence.storageBytes(m.fence)) of \(Fence.BOARD_STORAGE_BYTES) bytes" +
                 (Fence.problem(m.fence).map { " · \($0)" } ?? ""))
                .font(.caption).foregroundStyle(Fence.problem(m.fence) == nil ? Color.secondary : Color.bad)
            List {
                if m.fence.isEmpty {
                    Text("Add a zone, then tap the map at each corner. Green zones: stay in. Red: keep out.").foregroundStyle(.secondary)
                }
                ForEach(Array(m.fence.enumerated()), id: \.element.id) { i, z in
                    HStack {
                        Image(systemName: m.fenceZone == i ? "pencil.circle.fill" : "circle")
                            .foregroundStyle(z.kind.inclusion ? Color.green : z.kind == .returnPoint ? Color.blue : Color.red)
                        VStack(alignment: .leading) {
                            Text("\(i + 1). \(z.kind.rawValue)")
                            Text(z.kind.polygon ? "\(z.points.count) corners" : z.kind.circle ? String(format: "radius %.0f m", z.radius) : (z.points.isEmpty ? "no position" : "set"))
                                .font(.caption).foregroundStyle(.secondary)
                        }
                        Spacer()
                        if z.kind.polygon || z.kind.circle {
                            Button(z.kind.inclusion ? "→ out" : "→ in") {
                                m.fence[i].kind = switchKind(z.kind)
                            }.buttonStyle(.bordered).font(.caption)
                        }
                        if z.kind.polygon && !z.points.isEmpty {
                            Button("Undo corner") { m.fence[i].points.removeLast() }.buttonStyle(.bordered).font(.caption)
                        }
                    }
                    .contentShape(Rectangle())
                    .onTapGesture { m.fenceZone = i; m.show("Map taps now edit zone \(i + 1)") }
                }
                .onDelete { idx in m.fence.remove(atOffsets: idx); m.fenceZone = nil }
            }
            .listStyle(.plain)
            .frame(minHeight: 110, maxHeight: 220)
        }
    }

    private func switchKind(_ k: FenceZone.Kind) -> FenceZone.Kind {
        switch k {
        case .polygonIn: return .polygonOut
        case .polygonOut: return .polygonIn
        case .circleIn: return .circleOut
        case .circleOut: return .circleIn
        case .returnPoint: return .returnPoint
        }
    }
}

func itemLabel(_ it: MissionItem) -> String {
    switch it.command {
    case Mav.CMD_NAV_WAYPOINT: return it.p1 > 0 ? "Waypoint, wait \(Patrol.fmt(it.p1)) s" : "Waypoint"
    case Mav.CMD_NAV_LOITER_TIME: return "Loiter \(Patrol.fmt(it.p1)) s"
    case Mav.CMD_NAV_RETURN_TO_LAUNCH: return "Return to launch"
    case Mav.CMD_DO_CHANGE_SPEED: return "Speed \(Patrol.fmt(it.p2)) m/s"
    case Mav.CMD_DO_JUMP: return it.p2 < 0 ? "Jump to \(Int(it.p1)), forever" : "Jump to \(Int(it.p1)), \(Int(it.p2)) more times"
    default: return "Command \(it.command)"
    }
}

/// Edits one plan item: its command, wait/loiter time or speed; move or delete it.
struct ItemEditor: View {
    @EnvironmentObject var m: AppModel
    let index: Int
    let done: () -> Void

    var body: some View {
        if index < m.plan.count { editor } else { Text("Deleted").onAppear { done() } }
    }

    private var editor: some View {
        NavigationStack {
            Form {
                Picker("Command", selection: $m.plan[index].command) {
                    Text("Waypoint").tag(Mav.CMD_NAV_WAYPOINT)
                    Text("Loiter time").tag(Mav.CMD_NAV_LOITER_TIME)
                    Text("Return to launch").tag(Mav.CMD_NAV_RETURN_TO_LAUNCH)
                    Text("Change speed").tag(Mav.CMD_DO_CHANGE_SPEED)
                    Text("Jump").tag(Mav.CMD_DO_JUMP)
                }
                switch m.plan[index].command {
                case Mav.CMD_NAV_WAYPOINT: NumberField(label: "Wait", value: $m.plan[index].p1, unit: "s")
                case Mav.CMD_NAV_LOITER_TIME: NumberField(label: "Loiter", value: $m.plan[index].p1, unit: "s")
                case Mav.CMD_DO_CHANGE_SPEED: NumberField(label: "Speed", value: $m.plan[index].p2, unit: "m/s")
                case Mav.CMD_DO_JUMP:
                    NumberField(label: "To item", value: $m.plan[index].p1)
                    NumberField(label: "Repeats (-1 forever)", value: $m.plan[index].p2)
                default: EmptyView()
                }
                if m.plan[index].hasPosition {
                    Text(String(format: "%.7f, %.7f", m.plan[index].lat, m.plan[index].lon)).font(.callout.monospacedDigit())
                }
                Section {
                    Button("Move up") { if index > 0 { m.plan.swapAt(index, index - 1); done() } }.disabled(index == 0)
                    Button("Move down") { if index < m.plan.count - 1 { m.plan.swapAt(index, index + 1); done() } }
                        .disabled(index >= m.plan.count - 1)
                    Button("Delete", role: .destructive) { m.plan.remove(at: index); done() }
                }
            }
            .formStyle(.grouped)
            .navigationTitle("Item \(index + 1)")
            .toolbar { ToolbarItem(placement: .confirmationAction) { Button("Done") { done() } } }
        }
        .frame(minWidth: 320, minHeight: 360)
    }
}

/// Copter's ZigZag as waypoints: A and B, line spacing, number of lines, side, wait, run-out.
struct ZigZagSheet: View {
    @EnvironmentObject var m: AppModel
    @Environment(\.dismiss) private var dismiss
    let pick: (Character) -> Void

    var body: some View {
        NavigationStack {
            Form {
                Section("Points") {
                    HStack {
                        Text("A"); Spacer()
                        Text(m.zigzag.a.map { String(format: "%.6f, %.6f", $0.lat, $0.lon) } ?? "not set").foregroundStyle(.secondary)
                    }
                    HStack {
                        Button("A = car") { if m.vehicle.hasPosition { m.zigzag.a = Coord(lat: m.vehicle.lat, lon: m.vehicle.lon) } }
                        Button("A on the map") { pick("A") }
                    }.buttonStyle(.bordered)
                    HStack {
                        Text("B"); Spacer()
                        Text(m.zigzag.b.map { String(format: "%.6f, %.6f", $0.lat, $0.lon) } ?? "not set").foregroundStyle(.secondary)
                    }
                    HStack {
                        Button("B = car") { if m.vehicle.hasPosition { m.zigzag.b = Coord(lat: m.vehicle.lat, lon: m.vehicle.lon) } }
                        Button("B on the map") { pick("B") }
                    }.buttonStyle(.bordered)
                    if m.zigzag.lengthM > 0 { Text(String(format: "A to B: %.1f m", m.zigzag.lengthM)).foregroundStyle(.secondary) }
                }
                Section("Lanes") {
                    NumberField(label: "Line spacing", value: $m.zigzag.sideDistM, unit: "m")
                    Stepper("Lines: \(m.zigzag.lines)", value: $m.zigzag.lines, in: 1...200)
                    Picker("Side", selection: $m.zigzag.right) {
                        Text("Right of A→B").tag(true)
                        Text("Left of A→B").tag(false)
                    }
                    NumberField(label: "Wait at line ends", value: $m.zigzag.waitS, unit: "s")
                    NumberField(label: "Run-out past the ends", value: $m.zigzag.runOutM, unit: "m")
                }
                if let p = m.zigzag.problem() { Text(p).foregroundStyle(Color.bad) }
                Section {
                    Button("Replace the plan with the lanes") {
                        m.plan = m.zigzag.waypoints()
                        m.show("\(m.plan.count) waypoints: press Write, then Start Auto")
                        dismiss()
                    }.disabled(m.zigzag.problem() != nil)
                }
            }
            .formStyle(.grouped)
            .navigationTitle("Zig zag")
            .toolbar { ToolbarItem(placement: .cancellationAction) { Button("Close") { dismiss() } } }
        }
        .frame(minWidth: 360, minHeight: 520)
    }
}

/// Patrol: loop the plan with a DO_JUMP; set up BendyRuler for the LiDAR.
struct PatrolSheet: View {
    @EnvironmentObject var m: AppModel
    @Environment(\.dismiss) private var dismiss
    @State private var laps = 0
    @State private var wait: Float = 0

    var body: some View {
        let problems = Patrol.bendyRulerProblems { m.vehicle.paramValue($0) }
        return NavigationStack {
            Form {
                Section("Loop") {
                    Stepper(laps == 0 ? "Laps: forever" : "Laps: \(laps)", value: $laps, in: 0...100)
                    NumberField(label: "Wait at the end of each lap", value: $wait, unit: "s")
                    Button("Make the plan a patrol") {
                        if m.plan.filter({ $0.hasPosition }).count < 2 { m.show("Add at least 2 waypoints first"); return }
                        m.plan = Patrol.apply(m.plan, laps: laps, waitS: wait)
                        m.show("Patrol ready: press Write, then Start Auto")
                        dismiss()
                    }
                }
                Section("BendyRuler (steers round what the LiDAR sees)") {
                    if problems.isEmpty { Label("Settings look right", systemImage: "checkmark.circle").foregroundStyle(Color.good) }
                    ForEach(problems, id: \.self) { Text($0).font(.callout) }
                    Button("Set up BendyRuler") { setup() }
                    Text("The first press writes OA_TYPE and reboots; press again after it reconnects and the parameters are refreshed.")
                        .font(.caption).foregroundStyle(.secondary)
                }
            }
            .formStyle(.grouped)
            .navigationTitle("Patrol")
            .toolbar { ToolbarItem(placement: .cancellationAction) { Button("Close") { dismiss() } } }
        }
        .frame(minWidth: 360, minHeight: 480)
        .onAppear { laps = Patrol.laps(m.plan) == 1 ? 0 : Patrol.laps(m.plan) }
    }

    private func setup() {
        guard m.requireLink() else { return }
        if m.vehicle.paramValue("OA_TYPE") == nil { m.show("Read the parameters first (Params → Refresh)"); return }
        let (writes, reboot) = Patrol.setupWrites { m.vehicle.paramValue($0) }
        m.paramWriteOk = 0; m.paramWriteFail = []
        for (k, v) in writes { m.vehicle.setParam(k, v) }
        if reboot {
            m.show("OA_TYPE written; rebooting. Refresh the parameters after it reconnects, then press again.")
            DispatchQueue.main.asyncAfter(deadline: .now() + 2) { m.vehicle.reboot() }
        } else if writes.isEmpty { m.show("BendyRuler is already set up") }
    }
}

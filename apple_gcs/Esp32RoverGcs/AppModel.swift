import Foundation
import SwiftUI
#if os(iOS)
import UIKit
import AudioToolbox
#else
import AppKit
#endif

/// The app's state around the Vehicle: settings, the plan and fence being edited,
/// the LiDAR behaviours, NTRIP, the drive stick and the messages shown to the user.
/// Everything runs on the main thread. SwiftUI redraws at 5 Hz (tick).
final class AppModel: ObservableObject {
    let vehicle = Vehicle()
    let behaviours = LidarBehaviours()
    lazy var lidar = LidarDriver(vehicle: vehicle, behaviours: behaviours)
    /// Indoor navigation (the Indoor tab), Nav2 style.
    lazy var indoor = IndoorNav(vehicle: vehicle)

    @Published var tick = 0
    @Published var banner: String?
    private var bannerUntil = Date.distantPast

    // --- connection settings (kept between launches)
    enum Transport: String, CaseIterable, Identifiable { case tcp = "TCP", udp = "UDP"; var id: String { rawValue } }
    @Published var transport: Transport = .tcp
    @Published var host = "192.168.4.1"
    @Published var port = "5760"

    // --- plan and fence being edited
    @Published var plan = [MissionItem]()
    @Published var planSpeed: Float = 0          // 0 = vehicle's WP_SPEED
    @Published var fence = [FenceZone]()
    @Published var fenceZone: Int? = nil          // zone that map taps add corners to
    @Published var zigzag = ZigZag()
    @Published var guidedTarget: Coord?
    var lapCounter = LapCounter()

    // --- parameters staged for one write
    @Published var staged = [String: Float]()
    var paramWriteOk = 0
    var paramWriteFail = [String]()

    // --- driving with the on-screen stick or the keyboard
    @Published var driveLimit: Float = 0.3
    var stickX: Float = 0, stickY: Float = 0
    var stickHeld = false
    private var stickZeros = 0

    // --- LiDAR behaviour
    @Published var lidarSound = true
    private var lastBeep: Int64 = 0

    // --- calibration
    @Published var accelPos = 0
    @Published var accelText = "Mount the board in the car first. Then 6 positions, or level only."
    @Published var magText = "Rotate the car in every direction until all bars are full."
    @Published var rcCal = false
    var rcMin = [Int](repeating: 3000, count: 8)
    var rcMax = [Int](repeating: 0, count: 8)

    // --- NTRIP
    @Published var ntripHost = ""
    @Published var ntripPort = "2101"
    @Published var ntripMount = ""
    @Published var ntripUser = ""
    @Published var ntripPass = ""
    @Published var ntripCellular = true
    private(set) var ntrip: NtripClient?

    private var fastTimer: Timer?
    private var uiTimer: Timer?
    private let defaults = UserDefaults.standard

    /// The running model, for the macOS app delegate.
    static weak var current: AppModel?

    init() {
        load()
        AppModel.current = self
        vehicle.onEvent = { [weak self] e in self?.onEvent(e) }
        let fast = Timer(timeInterval: 0.1, repeats: true) { [weak self] _ in self?.fastTick() }
        let ui = Timer(timeInterval: 0.2, repeats: true) { [weak self] _ in self?.uiTick() }
        RunLoop.main.add(fast, forMode: .common)
        RunLoop.main.add(ui, forMode: .common)
        fastTimer = fast; uiTimer = ui
        if defaults.object(forKey: "autoconnect") as? Bool ?? true { connect() }
    }

    // =========================================================================
    // Settings
    // =========================================================================

    private func load() {
        transport = Transport(rawValue: defaults.string(forKey: "transport") ?? "") ?? .tcp
        host = defaults.string(forKey: "host") ?? "192.168.4.1"
        port = defaults.string(forKey: "port") ?? "5760"
        driveLimit = defaults.object(forKey: "drive_limit") as? Float ?? 0.3
        ntripHost = defaults.string(forKey: "ntrip_host") ?? ""
        ntripPort = defaults.string(forKey: "ntrip_port") ?? "2101"
        ntripMount = defaults.string(forKey: "ntrip_mount") ?? ""
        ntripUser = defaults.string(forKey: "ntrip_user") ?? ""
        ntripPass = defaults.string(forKey: "ntrip_pass") ?? ""
        ntripCellular = defaults.object(forKey: "ntrip_cellular") as? Bool ?? true
        behaviours.mode = LidarBehaviours.Mode(rawValue: defaults.string(forKey: "lidar_mode") ?? "") ?? .off
        behaviours.p.skidSteer = defaults.bool(forKey: "lidar_skid")
        if let v = defaults.object(forKey: "indoor_speed") as? Double { indoor.settings.speed = v }
        if let v = defaults.object(forKey: "indoor_radius") as? Double { indoor.settings.robotRadius = v }
        if let v = defaults.object(forKey: "indoor_inflation") as? Double { indoor.settings.inflation = v }
    }

    func setIndoorSettings(speed: Double, radius: Double, inflation: Double) {
        indoor.settings.speed = speed; indoor.settings.robotRadius = radius; indoor.settings.inflation = inflation
        defaults.set(speed, forKey: "indoor_speed"); defaults.set(radius, forKey: "indoor_radius")
        defaults.set(inflation, forKey: "indoor_inflation")
    }

    func saveSettings() {
        defaults.set(transport.rawValue, forKey: "transport")
        defaults.set(host, forKey: "host")
        defaults.set(port, forKey: "port")
        defaults.set(driveLimit, forKey: "drive_limit")
        defaults.set(ntripHost, forKey: "ntrip_host")
        defaults.set(ntripPort, forKey: "ntrip_port")
        defaults.set(ntripMount, forKey: "ntrip_mount")
        defaults.set(ntripUser, forKey: "ntrip_user")
        defaults.set(ntripPass, forKey: "ntrip_pass")
        defaults.set(ntripCellular, forKey: "ntrip_cellular")
        defaults.set(behaviours.mode.rawValue, forKey: "lidar_mode")
        defaults.set(behaviours.p.skidSteer, forKey: "lidar_skid")
    }

    // =========================================================================
    // Link
    // =========================================================================

    func connect() {
        saveSettings()
        defaults.set(true, forKey: "autoconnect")
        let h = host.trimmingCharacters(in: .whitespaces)
        let p = UInt16(port) ?? (transport == .tcp ? 5760 : 14550)
        let t = transport
        vehicle.connect { () -> Link in t == .tcp ? TcpLink(host: h, port: p) : UdpLink(host: h, port: p) }
    }

    func disconnect() {
        defaults.set(false, forKey: "autoconnect")
        stopLidar()
        vehicle.disconnect()
    }

    /// True when the vehicle is there; otherwise says so.
    func requireLink() -> Bool {
        if vehicle.connected { return true }
        show("Not connected to the vehicle")
        return false
    }

    // =========================================================================
    // Timers
    // =========================================================================

    private func fastTick() {
        let now = clockMs()
        indoor.feed()
        // the on-screen stick / keyboard: 10 Hz while held, then three zeros
        if stickHeld {
            stickZeros = 3
            vehicle.manualControl(steer: stickX * driveLimit, thr: stickY * driveLimit)
        } else if stickZeros > 0 {
            stickZeros -= 1
            vehicle.manualControl(steer: 0, thr: 0)
        }
        let cmd = lidar.tick(now, paused: stickHeld, limit: driveLimit)
        if lidar.running && cmd.alarm && lidarSound && now - lastBeep > 450 {
            lastBeep = now
            beep()
        }
        if !plan.isEmpty || vehicle.customMode == 10 {
            _ = lapCounter.update(vehicle.missionSeq, inAuto: vehicle.customMode == 10)
        }
        if rcCal {
            for i in 0..<8 where vehicle.rc[i] > 800 && vehicle.rc[i] < 2200 {
                rcMin[i] = min(rcMin[i], vehicle.rc[i]); rcMax[i] = max(rcMax[i], vehicle.rc[i])
            }
        }
    }

    private func uiTick() {
        if banner != nil && Date() > bannerUntil { banner = nil }
        tick &+= 1
    }

    private func beep() {
        #if os(iOS)
        AudioServicesPlaySystemSound(1005)
        UIImpactFeedbackGenerator(style: .heavy).impactOccurred()
        #else
        NSSound.beep()
        #endif
    }

    func show(_ text: String) {
        banner = text
        bannerUntil = Date().addingTimeInterval(3.5)
    }

    // =========================================================================
    // Vehicle events
    // =========================================================================

    private func onEvent(_ e: VehicleEvent) {
        switch e {
        case .text(let sev, let text):
            if sev <= 4 { show(text) }
        case .link(let text):
            show(text)
        case .paramsDone(let got, let total):
            show(got >= total ? "Parameters: \(got)" : "Parameters: \(got) of \(total) (press Refresh to try again)")
        case .paramWritten(let name, let ok, _):
            if ok { paramWriteOk += 1 } else { paramWriteFail.append(name) }
            if vehicle.paramWritesPending == 0 {
                show(paramWriteFail.isEmpty ? "Written: \(paramWriteOk) parameter(s)"
                     : "Written \(paramWriteOk); FAILED: \(paramWriteFail.joined(separator: ", "))")
                paramWriteOk = 0; paramWriteFail = []
            }
        case .missionUploaded(_, let text, _):
            show(text)
        case .missionDownloaded(let items, let text, let type):
            show(text)
            if let items = items {
                if type == Vehicle.MISSION_TYPE_FENCE { fence = Fence.fromItems(items); fenceZone = nil }
                else { plan = items }
            }
        case .commandAck(let cmd, let result):
            let what: String
            switch cmd {
            case Mav.CMD_COMPONENT_ARM_DISARM: what = "Arm/disarm"
            case Mav.CMD_DO_SET_MODE: what = "Mode change"
            case Mav.CMD_MISSION_START: what = "Mission start"
            case Mav.CMD_PREFLIGHT_CALIBRATION: what = "Calibration"
            case Mav.CMD_DO_START_MAG_CAL: what = "Compass calibration"
            case Mav.CMD_PREFLIGHT_REBOOT_SHUTDOWN: what = "Reboot"
            case Mav.CMD_DO_SET_HOME: what = "Set home"
            default: what = "Command \(cmd)"
            }
            if result != 0 { show("\(what): \(Vehicle.cmdResult(result))") }
            else if cmd == Mav.CMD_PREFLIGHT_REBOOT_SHUTDOWN { show("Rebooting") }
        case .accelCalPos(let pos):
            accelPos = pos
            let names = [1: "LEVEL", 2: "on its LEFT side", 3: "on its RIGHT side", 4: "NOSE DOWN", 5: "NOSE UP", 6: "on its BACK"]
            if let n = names[pos] {
                accelText = "Place the car \(n), hold it still, then press Done."
                show("Accel calibration: place the car \(n)")
            } else if pos == 16777215 {
                accelText = "Accelerometer calibrated. Reboot the board."; accelPos = 0; show("Accel calibration done")
            } else if pos == 16777216 {
                accelText = "Calibration FAILED. Try again, holding each position still."; accelPos = 0
            }
        case .magCalReport(let status, let fitness):
            magText = status == 4 ? String(format: "Compass calibrated (fitness %.1f). Reboot the board.", fitness)
                : "Compass calibration failed (status \(status)). Try again away from metal."
        }
    }

    // =========================================================================
    // Actions
    // =========================================================================

    func setMode(_ mode: Int) {
        guard requireLink() else { return }
        if mode != 10 { lapCounter.reset() }
        vehicle.setMode(mode)
    }

    func goTo(_ c: Coord) {
        guard requireLink() else { return }
        if vehicle.customMode != 15 { vehicle.setMode(15) }
        vehicle.goTo(lat: c.lat, lon: c.lon)
        // Guided needs the mode first; send the target again once it has changed.
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.7) { [weak self] in self?.vehicle.goTo(lat: c.lat, lon: c.lon) }
        guidedTarget = c
    }

    func writePlan() {
        guard requireLink() else { return }
        if plan.isEmpty { show("The plan is empty"); return }
        var items = plan
        if planSpeed > 0 {
            items.insert(MissionItem(command: Mav.CMD_DO_CHANGE_SPEED, p1: 1, p2: planSpeed, p3: -1), at: 0)
        }
        vehicle.uploadMission(items)
    }

    func startAuto() {
        guard requireLink() else { return }
        lapCounter.reset()
        vehicle.setMode(10)
        vehicle.startMission()
    }

    func writeFence() {
        guard requireLink() else { return }
        if let p = Fence.problem(fence) { show(p); return }
        let bytes = Fence.storageBytes(fence)
        if bytes > Fence.BOARD_STORAGE_BYTES { show("The fence needs \(bytes) bytes; the board stores \(Fence.BOARD_STORAGE_BYTES)"); return }
        vehicle.uploadFence(Fence.toItems(fence))
    }

    /// The text for the fence's Enable confirmation, or nil (and a message) when the parameters are missing.
    func fenceEnablePlan() -> (text: String, type: Int, action: Int)? {
        guard requireLink() else { return nil }
        guard let en = vehicle.paramValue("FENCE_ENABLE"), let typeF = vehicle.paramValue("FENCE_TYPE") else {
            show("Read the parameters first (Params → Refresh)"); return nil
        }
        let type = Int(typeF)
        let action = vehicle.paramValue("FENCE_ACTION").map { Int($0) }
        let newType = type | 4                   // bit 2: inclusion/exclusion circles + polygons
        let newAction = (action == nil || action == 0) ? 2 : action!
        let text = "FENCE_ENABLE \(Int(en)) → 1\nFENCE_TYPE \(type) → \(newType) (adds polygons and circles)\n" +
            "FENCE_ACTION \(action.map { String($0) } ?? "?") → \(newAction) (\(newAction == 2 ? "Hold" : "unchanged"))\n\n" +
            "The car will not arm while it is outside a stay-in zone or inside a stay-out zone."
        return (text, newType, newAction)
    }

    func enableFence(type: Int, action: Int) {
        vehicle.setParam("FENCE_TYPE", Float(type))
        vehicle.setParam("FENCE_ACTION", Float(action))
        vehicle.setParam("FENCE_ENABLE", 1)
    }

    /// A map tap in the Plan tab: a fence corner, a circle centre, or a waypoint.
    func planTap(_ c: Coord, fenceMode: Bool) {
        if fenceMode {
            guard let zi = fenceZone, zi < fence.count else { show("Add a zone first (+ Stay-in, + Stay-out, + Circle)"); return }
            if fence[zi].kind.polygon { fence[zi].points.append(c) }
            else { fence[zi].points = [c]; fenceZone = nil }
        } else {
            plan.append(MissionItem(lat: c.lat, lon: c.lon))
        }
    }

    func addZone(_ kind: FenceZone.Kind, radius: Float = 0) {
        fence.append(FenceZone(kind: kind, radius: radius))
        fenceZone = fence.count - 1
        show(kind.polygon ? "Tap the map at each corner" : "Tap the map at the centre")
    }

    // --- parameters
    func writeStaged() {
        guard requireLink() else { return }
        paramWriteOk = 0; paramWriteFail = []
        for (k, v) in staged { vehicle.setParam(k, v) }
        show("Writing \(staged.count) parameter(s)")
        staged.removeAll()
    }

    /// Stages the values of a .parm file that differ from the vehicle's.
    func loadParmFile(_ text: String) {
        var n = 0, unknown = 0
        for raw in text.components(separatedBy: .newlines) {
            let line = raw.split(separator: "#", maxSplits: 1, omittingEmptySubsequences: false).first.map(String.init) ?? ""
            let f = line.split(whereSeparator: { $0 == "," || $0 == " " || $0 == "\t" }).map(String.init)
            guard f.count >= 2, let v = Float(f[1]) else { continue }
            let name = f[0].trimmingCharacters(in: .whitespaces)
            guard let cur = vehicle.paramValue(name) else { unknown += 1; continue }
            if abs(cur - v) > max(1e-6, abs(v) * 1e-5) { staged[name] = v; n += 1 }
        }
        show("Staged \(n) changed value(s)" + (unknown > 0 ? "; \(unknown) not on this vehicle (reboot may add them)" : ""))
    }

    func parmFileText() -> String {
        var s = "# \(vehicle.sortedParams.count) parameters, ESP32 Rover GCS\n"
        for p in vehicle.sortedParams { s += "\(p.name),\(Patrol.fmt(p.value))\n" }
        return s
    }

    // --- mission files
    /// Mission Planner / QGC "QGC WPL 110" .waypoints files; line 0 is home.
    func loadWaypoints(_ text: String) {
        let lines = text.components(separatedBy: .newlines).map { $0.trimmingCharacters(in: .whitespaces) }.filter { !$0.isEmpty }
        guard lines.first?.hasPrefix("QGC WPL") == true else { show("Not a QGC WPL / Mission Planner .waypoints file"); return }
        var items = [MissionItem]()
        for l in lines.dropFirst() {
            let f = l.split(whereSeparator: { $0 == " " || $0 == "\t" }).map(String.init)
            if f.count < 12 || f[0] == "0" { continue }
            items.append(MissionItem(command: Int(f[3]) ?? 16, lat: Double(f[8]) ?? 0, lon: Double(f[9]) ?? 0, alt: Float(f[10]) ?? 0,
                                     p1: Float(f[4]) ?? 0, p2: Float(f[5]) ?? 0, p3: Float(f[6]) ?? 0, p4: Float(f[7]) ?? 0,
                                     frame: Int(f[2]) ?? 3, autocontinue: Int(f[11]) ?? 1))
        }
        plan = items
        show("Loaded \(items.count) items")
    }

    func waypointsText() -> String {
        var s = "QGC WPL 110\n"
        let first = plan.first { $0.hasPosition }
        let hl = vehicle.hasHome ? vehicle.homeLat : first?.lat ?? 0
        let ho = vehicle.hasHome ? vehicle.homeLon : first?.lon ?? 0
        s += String(format: "0\t1\t0\t16\t0\t0\t0\t0\t%.8f\t%.8f\t0\t1\n", hl, ho)
        for (i, it) in plan.enumerated() {
            s += "\(i + 1)\t0\t\(it.frame)\t\(it.command)\t\(Patrol.fmt(it.p1))\t\(Patrol.fmt(it.p2))\t\(Patrol.fmt(it.p3))\t\(Patrol.fmt(it.p4))\t" +
                String(format: "%.8f\t%.8f\t", it.lat, it.lon) + "\(Patrol.fmt(it.alt))\t\(it.autocontinue)\n"
        }
        return s
    }

    // --- LiDAR
    func startLidar() {
        if behaviours.mode == .off { show("Choose Avoid, Follow or Warn first"); return }
        if vehicle.scan == nil { show("No LiDAR scan from the vehicle yet"); return }
        if lidar.drives {
            guard requireLink() else { return }
            if ![0, 1, 3].contains(vehicle.customMode) { vehicle.setMode(0); show("Switched to Manual: stick overrides only drive in Manual, Acro or Steering") }
            if !vehicle.armed { show("The car is disarmed: arm it on the Data tab to drive") }
        }
        lidar.start()
        objectWillChange.send()
    }

    func stopLidar() {
        lidar.stop()
        objectWillChange.send()
    }

    func setLidarMode(_ m: LidarBehaviours.Mode) {
        let was = lidar.running
        behaviours.mode = m
        saveSettings()
        if m == .off { stopLidar() } else if was { lidar.start() }
        objectWillChange.send()
    }

    // --- NTRIP
    func startNtrip() {
        stopNtrip()
        saveSettings()
        guard !ntripHost.isEmpty, !ntripMount.isEmpty, let p = UInt16(ntripPort) else { show("Fill in the caster, port and mountpoint"); return }
        let v = vehicle
        let c = NtripClient(host: ntripHost, port: p, mount: ntripMount, user: ntripUser, pass: ntripPass, overCellular: ntripCellular,
                            position: { v.hasPosition ? Coord(lat: v.lat, lon: v.lon) : nil },
                            onFrame: { v.injectRtcm($0) })
        ntrip = c
        c.start()
    }

    func stopNtrip() {
        ntrip?.stop()
        ntrip = nil
    }

    // --- radio calibration
    func radioCalToggle() {
        if !rcCal {
            guard requireLink() else { return }
            rcMin = [Int](repeating: 3000, count: 8); rcMax = [Int](repeating: 0, count: 8)
            rcCal = true
            show("Move every stick and switch to both ends, centre the sticks, then Save")
            return
        }
        rcCal = false
        var n = 0
        paramWriteOk = 0; paramWriteFail = []
        for i in 0..<8 where rcMax[i] - rcMin[i] >= 200 {
            let ch = i + 1
            vehicle.setParam("RC\(ch)_MIN", Float(rcMin[i]))
            vehicle.setParam("RC\(ch)_MAX", Float(rcMax[i]))
            // Trim = the centred stick now; switches and the throttle of a car stay at their middle.
            vehicle.setParam("RC\(ch)_TRIM", Float(max(rcMin[i], min(rcMax[i], vehicle.rc[i]))))
            n += 1
        }
        show(n == 0 ? "No channel moved: nothing saved" : "Writing min/max/trim for \(n) channels")
    }

    /// Leaving the app (background, window closed): stop anything that drives.
    func stopDriving() {
        stickHeld = false
        stopLidar()
        indoor.stop()
    }
}

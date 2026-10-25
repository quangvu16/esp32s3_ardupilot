import Foundation

/// Monotonic milliseconds.
func clockMs() -> Int64 { Int64(DispatchTime.now().uptimeNanoseconds / 1_000_000) }

/// One mission item as ArduPilot stores it (lat/lon in degrees, frame relative-alt).
struct MissionItem: Identifiable, Equatable {
    var id = UUID()
    var command: Int = Mav.CMD_NAV_WAYPOINT
    var lat: Double = 0
    var lon: Double = 0
    var alt: Float = 0
    var p1: Float = 0, p2: Float = 0, p3: Float = 0, p4: Float = 0
    var frame: Int = Mav.FRAME_GLOBAL_RELATIVE_ALT_INT
    var autocontinue: Int = 1

    /// Commands whose x/y are a position (drawn on the map).
    var hasPosition: Bool { lat != 0 || lon != 0 }

    func copy() -> MissionItem { var c = self; c.id = UUID(); return c }
}

final class Param: Identifiable {
    let name: String
    var value: Float
    var type: Int
    var index: Int
    var id: String { name }
    init(name: String, value: Float, type: Int, index: Int) {
        self.name = name; self.value = value; self.type = type; self.index = index
    }
}

/// Things the UI shows once (banners, dialogs, logs).
enum VehicleEvent {
    case text(severity: Int, text: String)
    case link(String)
    case paramsDone(received: Int, total: Int)
    case paramWritten(name: String, ok: Bool, value: Float)
    case missionUploaded(ok: Bool, text: String, type: Int)
    case missionDownloaded(items: [MissionItem]?, text: String, type: Int)
    case commandAck(command: Int, result: Int)
    case accelCalPos(Int)
    case magCalReport(status: Int, fitness: Float)
}

struct LogLine: Identifiable {
    let id = UUID()
    let severity: Int
    let text: String
    let time: Date
}

/// The autopilot as seen from the app. Everything runs on the main thread: the
/// links deliver bytes there and a 10 Hz timer drives the protocols.
/// A port of the Android app's core/Vehicle.kt.
final class Vehicle {
    var onEvent: ((VehicleEvent) -> Void)?

    private let enc = MavEncoder()
    private let parser = MavParser()
    private var timer: Timer?

    private(set) var link: Link?
    private var wantLink: (() -> Link)?
    private var reconnectAt: Int64 = 0
    private(set) var linkText = "Not connected"
    /// One failed attempt is reported; the retries every few seconds are not.
    private var failureShown = false

    /// A few words for the status pill (linkText has the details).
    var linkShort: String {
        if connected, let l = link { return l.name }
        if let l = link { return l.open ? "Waiting for heartbeat" : "Connecting…" }
        return wantLink == nil ? "Not connected" : "No link · retrying"
    }

    // --- telemetry
    var sysId = 0
    var compId = 1
    var lastHeartbeat: Int64 = 0
    var vehicleType = 0
    var armed = false
    var customMode = -1
    var systemStatus = 0
    var roll: Float = 0, pitch: Float = 0, yaw: Float = 0, yawRate: Float = 0
    var groundSpeed: Float = 0
    var heading = 0
    var throttle = 0
    var lat = 0.0, lon = 0.0
    var relAlt: Float = 0
    var hasPosition = false
    var gpsFix = 0, gpsSats = 0
    var gpsHdop: Float = 0, gpsHAcc: Float = 0
    var voltage: Float = 0, current: Float = 0
    var batteryPct = -1
    var sensorsPresent: Int64 = 0, sensorsHealth: Int64 = 0, sensorsEnabled: Int64 = 0
    var homeLat = 0.0, homeLon = 0.0
    var hasHome = false
    var missionSeq = 0
    var wpDist = 0
    var navBearing = 0, targetBearing = 0
    var xtrack: Float = 0
    var ekfFlags = 0
    var rc = [Int](repeating: 0, count: 16)
    var rcCount = 0
    var rssi = 0
    var rcTime: Int64 = 0
    /// Proximity sectors (DISTANCE_SENSOR orientation 0-7 = 0°, 45° … 315°), metres, 0 = none.
    var proximity = [Float](repeating: 0, count: 8)
    var proximityTime = [Int64](repeating: 0, count: 8)
    var magCalPct = [0, 0, 0]
    var magCalStatus = [0, 0, 0]
    var rxPackets = 0
    /// Latest full LiDAR scan (OBSTACLE_DISTANCE from the LD14 driver), or nil before the first.
    var scan: LidarScan?
    var scanCount = 0
    /// The car's own pose by autopilot time (LOCAL_POSITION_NED + ATTITUDE), for indoor navigation. Thread-safe.
    let poses = PoseHistory()
    var localX = 0.0, localY = 0.0, localVx = 0.0, localVy = 0.0
    var localTime: Int64 = 0          // clockMs() of the last LOCAL_POSITION_NED, 0 = none
    var vehicleBootMs: Int64 = 0
    /// Ask for position and attitude at 10 Hz (indoor navigation).
    var fastPose = false { didSet { if fastPose != oldValue { lastStreamsAt = 0 } } }
    private var partScan: [Int]?
    private var partScanUs: Int64 = -1
    private var partScanDeg: Float = 0
    var rtcmBytes: Int64 = 0
    /// The car's track, newest last.
    var trail = [(Double, Double)]()

    var messages = [LogLine]()      // newest last

    var connected: Bool { clockMs() - lastHeartbeat < 4000 }
    var modeName: String {
        if let n = Vehicle.ROVER_MODES.first(where: { $0.0 == customMode })?.1 { return n }
        return customMode < 0 ? "—" : "Mode \(customMode)"
    }

    // --- parameters
    private(set) var params = [String: Param]()
    var sortedParams: [Param] { params.values.sorted { $0.name < $1.name } }
    private(set) var paramCount = 0
    private(set) var paramsFetching = false
    private var paramLastRx: Int64 = 0
    private var paramRounds = 0
    private var paramIndices = Set<Int>()
    private final class PendingSet { let name: String; let value: Float; var tries = 0; var sentAt: Int64 = 0
        init(_ n: String, _ v: Float) { name = n; value = v } }
    private var paramSets = [String: PendingSet]()
    private var paramOrder = [String]()
    var paramWritesPending: Int { paramSets.count }

    // --- mission
    private enum Tx { case idle, upload, download }
    private var tx = Tx.idle
    private var txItems = [MissionItem]()
    private var txCount = 0
    private var txNext = 0
    private var txSentAt: Int64 = 0
    private var txTries = 0
    private var txType = Vehicle.MISSION_TYPE_MISSION
    private(set) var missionProgress = ""
    var missionBusy: Bool { tx != .idle }

    // --- commands
    private final class PendingCmd { let cmd: Int; let p: [Float]; var tries: Int; var sentAt: Int64
        init(_ c: Int, _ p: [Float]) { cmd = c; self.p = p; tries = 1; sentAt = clockMs() } }
    private var commands = [Int: PendingCmd]()

    private var lastStreamsAt: Int64 = 0
    private var lastAttitudeAt: Int64 = 0
    private var lastHeartbeatTx: Int64 = 0

    init() {
        let t = Timer(timeInterval: 0.1, repeats: true) { [weak self] _ in self?.tick() }
        RunLoop.main.add(t, forMode: .common)
        timer = t
    }

    // =========================================================================
    // Link
    // =========================================================================

    func connect(_ make: @escaping () -> Link) {
        disconnect()
        wantLink = make
        reconnectAt = 0
        failureShown = false
    }

    func disconnect() {
        wantLink = nil
        link?.close()
        link = nil
        lastHeartbeat = 0
        linkText = "Not connected"
    }

    private func openLink() {
        guard let make = wantLink else { return }
        let l = make()
        linkText = "Connecting to \(l.name)…"
        l.onBytes = { [weak self] d in
            guard let self = self else { return }
            self.parser.feed(d) { self.handle($0) }
        }
        l.onClosed = { [weak self, weak l] why in
            guard let self = self, let l = l, self.link === l else { return }
            let wasConnected = self.connected
            self.link = nil
            self.lastHeartbeat = 0
            self.linkText = (wasConnected ? "Lost \(l.name): " : "\(l.name): ") + why
            // Say it when a working link drops, and once for a link that never came up.
            if wasConnected || !self.failureShown { self.post(.link(self.linkText)) }
            self.failureShown = true
            self.reconnectAt = clockMs() + 2000
        }
        link = l
        l.start()
        lastStreamsAt = 0
    }

    private func send(_ msgId: Int, _ b: Payload) {
        link?.send(enc.frame(msgId, b))
    }

    private func post(_ e: VehicleEvent) { onEvent?(e) }

    private func tick() {
        let now = clockMs()
        if link == nil && wantLink != nil && now >= reconnectAt {
            reconnectAt = now + 3000
            openLink()
        }
        guard let l = link else { return }
        if l.open && linkText.hasPrefix("Connecting") { linkText = "Waiting for heartbeat on \(l.name)" }
        if now - lastHeartbeatTx >= 1000 {
            lastHeartbeatTx = now
            sendHeartbeat()
        }
        if connected {
            linkText = l.name
            // Ask for telemetry once, and again if attitude stops (e.g. after a reboot).
            if now - lastStreamsAt > 15000 || (now - lastAttitudeAt > 3000 && now - lastStreamsAt > 3000) {
                lastStreamsAt = now
                requestStreams()
            }
        }
        tickParams(now)
        tickMission(now)
        tickCommands(now)
    }

    private func sendHeartbeat() {
        var b = Mav.payload(Mav.HEARTBEAT)
        b.put8(4, 6)          // MAV_TYPE_GCS
        b.put8(5, 8)          // MAV_AUTOPILOT_INVALID
        b.put8(7, 4)          // MAV_STATE_ACTIVE
        b.put8(8, 3)
        send(Mav.HEARTBEAT, b)
    }

    /// Stream rates kept low: the ESP32's WiFi is shared with the GPS corrections.
    private func requestStreams() {
        let rates: [(Int, Int)] = [
            (2, 2),   // EXTENDED_STATUS
            (3, 2),   // RC_CHANNELS
            (6, fastPose ? 10 : 3),   // POSITION
            (10, fastPose ? 10 : 8),  // EXTRA1: attitude
            (11, 4),  // EXTRA2: VFR_HUD
            (12, 3),  // EXTRA3: distance sensors, EKF, battery, mag cal
        ]
        for (stream, hz) in rates {
            var b = Mav.payload(Mav.REQUEST_DATA_STREAM)
            b.put16(0, hz)
            b.put8(2, sysId); b.put8(3, compId)
            b.put8(4, stream); b.put8(5, 1)
            send(Mav.REQUEST_DATA_STREAM, b)
        }
    }

    // =========================================================================
    // Receive
    // =========================================================================

    private func handle(_ m: MavPacket) {
        rxPackets += 1
        let p = m.p
        if m.msgId == Mav.HEARTBEAT {
            let type = p.u8(4)
            let ap = p.u8(5)
            // Only the autopilot: not other GCSs (type 6), not peripherals (autopilot INVALID).
            if type == 6 || ap == 8 { return }
            if sysId == 0 || sysId != m.sysId { sysId = m.sysId; compId = m.compId }
            if m.sysId != sysId || m.compId != compId { return }
            let first = !connected
            customMode = Int(p.u32(0))
            vehicleType = type
            armed = p.u8(6) & 0x80 != 0
            systemStatus = p.u8(7)
            lastHeartbeat = clockMs()
            if first { post(.link("Connected to system \(sysId)")); failureShown = false }
            return
        }
        if m.sysId != sysId { return }
        switch m.msgId {
        case Mav.SYS_STATUS:
            sensorsPresent = p.u32(0); sensorsEnabled = p.u32(4); sensorsHealth = p.u32(8)
            voltage = Float(p.u16(14)) / 1000
            current = p.i16(16) == -1 ? 0 : Float(p.i16(16)) / 100
            batteryPct = p.i8(30)
        case Mav.ATTITUDE:
            roll = p.f32(4); pitch = p.f32(8); yaw = p.f32(12); yawRate = p.f32(24)
            lastAttitudeAt = clockMs()
            poses.addYaw(p.u32(0), Double(yaw))
        case Mav.LOCAL_POSITION_NED:
            let t = p.u32(0)
            localX = Double(p.f32(4)); localY = Double(p.f32(8))
            localVx = Double(p.f32(16)); localVy = Double(p.f32(20))
            localTime = clockMs(); vehicleBootMs = t
            poses.addPosition(t, localX, localY)
        case Mav.GLOBAL_POSITION_INT:
            let la = p.i32(4), lo = p.i32(8)
            if la != 0 || lo != 0 {
                lat = Double(la) / 1e7; lon = Double(lo) / 1e7; hasPosition = true
                if let last = trail.last {
                    let dx = (lon - last.1) * 111320 * cos(lat * .pi / 180), dy = (lat - last.0) * 111320
                    if dx * dx + dy * dy > 0.25 { trail.append((lat, lon)) }
                } else { trail.append((lat, lon)) }
                if trail.count > 2000 { trail.removeFirst(500) }
            }
            relAlt = Float(p.i32(16)) / 1000
        case Mav.GPS_RAW_INT:
            gpsFix = p.u8(28); gpsSats = p.u8(29)
            gpsHdop = p.u16(20) == 0xffff ? 99.99 : Float(p.u16(20)) / 100
            gpsHAcc = Float(p.u32(34)) / 1000
        case Mav.VFR_HUD:
            groundSpeed = p.f32(4); heading = p.i16(16); throttle = p.u16(18)
        case Mav.HOME_POSITION:
            homeLat = Double(p.i32(0)) / 1e7; homeLon = Double(p.i32(4)) / 1e7
            hasHome = homeLat != 0 || homeLon != 0
        case Mav.MISSION_CURRENT:
            missionSeq = p.u16(0)
        case Mav.NAV_CONTROLLER_OUTPUT:
            navBearing = p.i16(20); targetBearing = p.i16(22); wpDist = p.u16(24); xtrack = p.f32(16)
        case Mav.RC_CHANNELS:
            for i in 0..<16 { rc[i] = p.u16(4 + 2 * i) }
            rcCount = min(p.u8(40), 16); rssi = p.u8(41); rcTime = clockMs()
        case Mav.EKF_STATUS_REPORT:
            ekfFlags = p.u16(20)
        case Mav.DISTANCE_SENSOR:
            let orient = p.u8(12)
            if orient <= 7 {
                let d = p.u16(8), mx = p.u16(6)
                proximity[orient] = (d == 0 || d >= mx) ? 0 : Float(d) / 100
                proximityTime[orient] = clockMs()
            }
        case Mav.OBSTACLE_DISTANCE:
            let cm = (0..<72).map { p.u16(8 + 2 * $0) }
            let incF = p.f32(158)
            let inc = incF > 0 ? incF : Float(p.u8(157))
            let off = p.f32(162)
            let us = p.u32(0) | (p.u32(4) << 32)
            if inc > 0 && inc * 72 >= 359 {
                scan = LidarScan(cm: cm, incrementDeg: inc, offsetDeg: off,
                                 minCm: p.u16(152), maxCm: p.u16(154), timeMs: clockMs(), vehicleTimeUs: us)
                scanCount += 1
            } else if inc > 0 {
                // a finer scan in parts (PRX_SCAN_FULL: 5 x 72 bins of 1°) with the same time
                let bins = Int((360 / inc).rounded())
                if us != partScanUs || partScan?.count != bins {
                    partScan = [Int](repeating: 0xffff, count: bins); partScanUs = us; partScanDeg = 0
                }
                let first = Int((off / inc).rounded())
                for i in 0..<72 where first + i >= 0 && first + i < bins { partScan![first + i] = cm[i] }
                partScanDeg += 72 * inc
                if partScanDeg >= 359, let full = partScan {
                    scan = LidarScan(cm: full, incrementDeg: inc, offsetDeg: 0,
                                     minCm: p.u16(152), maxCm: p.u16(154), timeMs: clockMs(), vehicleTimeUs: us)
                    scanCount += 1
                    partScanDeg = -1e9
                }
            }
        case Mav.BATTERY_STATUS:
            if p.u8(32) == 0 && p.i8(35) >= 0 { batteryPct = p.i8(35) }
        case Mav.STATUSTEXT:
            let line = LogLine(severity: p.u8(0), text: p.str(1, 50), time: Date())
            messages.append(line)
            if messages.count > 300 { messages.removeFirst(100) }
            post(.text(severity: line.severity, text: line.text))
        case Mav.MAG_CAL_PROGRESS:
            let id = p.u8(12)
            if id < 3 { magCalPct[id] = p.u8(16); magCalStatus[id] = p.u8(14) }
        case Mav.MAG_CAL_REPORT:
            let id = p.u8(40)
            if id < 3 { magCalStatus[id] = p.u8(42); magCalPct[id] = 100 }
            post(.magCalReport(status: p.u8(42), fitness: p.f32(0)))
        case Mav.COMMAND_LONG:
            // The autopilot asks us to pose the vehicle during accel calibration.
            if p.u16(28) == Mav.CMD_ACCELCAL_VEHICLE_POS { post(.accelCalPos(Int(p.f32(0)))) }
        case Mav.COMMAND_ACK:
            let cmd = p.u16(0), res = p.u8(2)
            if res == 5 { return }       // IN_PROGRESS
            if commands.removeValue(forKey: cmd) != nil { post(.commandAck(command: cmd, result: res)) }
        case Mav.PARAM_VALUE:
            onParamValue(p)
        case Mav.MISSION_REQUEST_INT, Mav.MISSION_REQUEST, Mav.MISSION_COUNT, Mav.MISSION_ITEM_INT, Mav.MISSION_ACK:
            onMission(m)
        default:
            break
        }
    }

    // =========================================================================
    // Commands and modes
    // =========================================================================

    func command(_ cmd: Int, _ args: Float...) {
        let pp = (0..<7).map { $0 < args.count ? args[$0] : 0 }
        commands[cmd] = PendingCmd(cmd, pp)
        sendCommandLong(cmd, pp, 0)
    }

    private func sendCommandLong(_ cmd: Int, _ p: [Float], _ confirmation: Int) {
        var b = Mav.payload(Mav.COMMAND_LONG)
        for i in 0..<7 { b.putF(4 * i, p[i]) }
        b.put16(28, cmd)
        b.put8(30, sysId); b.put8(31, compId); b.put8(32, confirmation)
        send(Mav.COMMAND_LONG, b)
    }

    private func tickCommands(_ now: Int64) {
        for (key, c) in commands where now - c.sentAt >= 1500 {
            if c.tries >= 3 {
                commands.removeValue(forKey: key)
                post(.commandAck(command: c.cmd, result: -1))
            } else {
                c.tries += 1; c.sentAt = now
                sendCommandLong(c.cmd, c.p, c.tries - 1)
            }
        }
    }

    func setMode(_ mode: Int) { command(Mav.CMD_DO_SET_MODE, 1, Float(mode)) }
    func arm(_ on: Bool, force: Bool = false) { command(Mav.CMD_COMPONENT_ARM_DISARM, on ? 1 : 0, force ? 21196 : 0) }
    func reboot() { command(Mav.CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1) }
    func setHomeHere() { command(Mav.CMD_DO_SET_HOME, 1) }
    func startMission() { command(Mav.CMD_MISSION_START) }
    func changeSpeed(_ mps: Float) { command(Mav.CMD_DO_CHANGE_SPEED, 1, mps, -1) }

    func setHome(lat: Double, lon: Double) {
        var b = Mav.payload(Mav.COMMAND_INT)
        b.put32(16, Int(lat * 1e7)); b.put32(20, Int(lon * 1e7))
        b.put16(28, Mav.CMD_DO_SET_HOME)
        b.put8(30, sysId); b.put8(31, compId); b.put8(32, Mav.FRAME_GLOBAL_RELATIVE_ALT_INT)
        send(Mav.COMMAND_INT, b)
    }

    /// Speed (m/s, negative reverse) and turn rate (rad/s, clockwise) in Guided mode:
    /// SET_POSITION_TARGET_LOCAL_NED, body frame, velocity + yaw rate (type_mask 1511).
    func sendVelocity(_ v: Double, _ yawRate: Double) {
        var b = Mav.payload(Mav.SET_POSITION_TARGET_LOCAL_NED)
        b.put32(0, Int(vehicleBootMs & 0xffffffff))
        b.putF(16, Float(v))
        b.putF(44, Float(yawRate))
        b.put16(48, 1511)
        b.put8(50, sysId); b.put8(51, compId)
        b.put8(52, Mav.FRAME_BODY_NED)
        send(Mav.SET_POSITION_TARGET_LOCAL_NED, b)
    }

    /// Drive to a point in Guided mode.
    func goTo(lat: Double, lon: Double) {
        var b = Mav.payload(Mav.SET_POSITION_TARGET_GLOBAL_INT)
        b.put32(4, Int(lat * 1e7)); b.put32(8, Int(lon * 1e7))
        b.put16(48, 0x0FF8)       // use position only
        b.put8(50, sysId); b.put8(51, compId)
        b.put8(52, Mav.FRAME_GLOBAL_RELATIVE_ALT_INT)
        send(Mav.SET_POSITION_TARGET_GLOBAL_INT, b)
    }

    func setCurrentWaypoint(_ seq: Int) {
        var b = Mav.payload(Mav.MISSION_SET_CURRENT)
        b.put16(0, seq); b.put8(2, sysId); b.put8(3, compId)
        send(Mav.MISSION_SET_CURRENT, b)
    }

    /// Steering and throttle, -1..1 each. Rover maps MANUAL_CONTROL y to steering and
    /// z to throttle (an RC override that times out after RC_OVERRIDE_TIME). Needs SYSID_MYGCS = 255.
    func manualControl(steer: Float, thr: Float) {
        var b = Mav.payload(Mav.MANUAL_CONTROL)
        b.put16(0, Int(Int16.max))                         // x unused
        b.put16(2, Int(max(-1, min(1, steer)) * 1000))
        b.put16(4, Int(max(-1, min(1, thr)) * 1000))
        b.put16(6, Int(Int16.max))                         // r unused
        b.put8(10, sysId)
        send(Mav.MANUAL_CONTROL, b)
    }

    // --- calibration
    func startAccelCal() { command(Mav.CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 1) }
    func accelCalLevelOnly() { command(Mav.CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 2) }
    func accelCalPosition(_ pos: Int) {
        // Answers the vehicle's request; ArduPilot does not ack it, so send it untracked.
        sendCommandLong(Mav.CMD_ACCELCAL_VEHICLE_POS, [Float(pos), 0, 0, 0, 0, 0, 0], 0)
    }
    func startMagCal() {
        magCalPct = [0, 0, 0]; magCalStatus = [0, 0, 0]
        command(Mav.CMD_DO_START_MAG_CAL, 0, 1 /* retry */, 1 /* autosave */, 0, 0)
    }
    func acceptMagCal() { command(Mav.CMD_DO_ACCEPT_MAG_CAL) }
    func cancelMagCal() { command(Mav.CMD_DO_CANCEL_MAG_CAL) }

    // --- RTK corrections
    private var rtcmSeq = 0

    /// Sends one RTCM3 frame as GPS_RTCM_DATA, in up to 4 fragments of 180 bytes.
    func injectRtcm(_ frame: [UInt8]) {
        if frame.count > 4 * 180 { return }
        rtcmBytes += Int64(frame.count)
        let seq = rtcmSeq; rtcmSeq = (rtcmSeq + 1) & 0x1f
        if frame.count <= 180 { sendRtcm(seq << 3, frame, 0, frame.count); return }
        var off = 0, frag = 0
        while off < frame.count {
            let n = min(180, frame.count - off)
            sendRtcm(1 | (frag << 1) | (seq << 3), frame, off, n)
            off += n; frag += 1
        }
        // A frame that is an exact multiple of 180 must end with an empty fragment.
        if frame.count % 180 == 0 && frag < 4 { sendRtcm(1 | (frag << 1) | (seq << 3), frame, 0, 0) }
    }

    private func sendRtcm(_ flags: Int, _ data: [UInt8], _ off: Int, _ n: Int) {
        var b = Mav.payload(Mav.GPS_RTCM_DATA)
        b.put8(0, flags); b.put8(1, n)
        for i in 0..<n { b.bytes[2 + i] = data[off + i] }
        send(Mav.GPS_RTCM_DATA, b)
    }

    // =========================================================================
    // Parameters
    // =========================================================================

    func fetchParams() {
        params.removeAll(); paramIndices.removeAll(); paramCount = 0
        paramsFetching = true; paramRounds = 0
        paramLastRx = clockMs()
        var b = Mav.payload(Mav.PARAM_REQUEST_LIST)
        b.put8(0, sysId); b.put8(1, compId)
        send(Mav.PARAM_REQUEST_LIST, b)
    }

    func paramValue(_ name: String) -> Float? { params[name]?.value }

    /// Queues a write; the result arrives as .paramWritten.
    func setParam(_ name: String, _ value: Float) {
        if paramSets[name] == nil { paramOrder.append(name) }
        paramSets[name] = PendingSet(name, value)
    }

    private func sendParamSet(_ name: String, _ value: Float) {
        var b = Mav.payload(Mav.PARAM_SET)
        b.putF(0, value)
        b.put8(4, sysId); b.put8(5, compId)
        b.putString(6, name, max: 16)
        b.put8(22, params[name]?.type ?? 9)
        send(Mav.PARAM_SET, b)
    }

    private func requestParam(_ index: Int) {
        var b = Mav.payload(Mav.PARAM_REQUEST_READ)
        b.put16(0, index)
        b.put8(2, sysId); b.put8(3, compId)
        send(Mav.PARAM_REQUEST_READ, b)
    }

    private func onParamValue(_ p: Payload) {
        let name = p.str(8, 16)
        let value = p.f32(0)
        let count = p.u16(4)
        let index = p.u16(6)
        if let existing = params[name] {
            existing.value = value
            if index != 0xffff { existing.index = index }
        } else {
            params[name] = Param(name: name, value: value, type: p.u8(24), index: index)
        }
        if index != 0xffff {
            paramCount = count
            paramIndices.insert(index)
            paramLastRx = clockMs()
            if paramsFetching && paramIndices.count >= count {
                paramsFetching = false
                post(.paramsDone(received: paramIndices.count, total: count))
            }
        }
        if let s = paramSets[name], s.sentAt != 0 {
            paramSets.removeValue(forKey: name); paramOrder.removeAll { $0 == name }
            let ok = abs(value - s.value) <= max(1e-6, abs(s.value) * 1e-5)
            post(.paramWritten(name: name, ok: ok, value: value))
        }
    }

    private func tickParams(_ now: Int64) {
        if paramsFetching && now - paramLastRx > 1500 {
            if paramCount == 0 || paramRounds >= 8 {
                paramsFetching = false
                post(.paramsDone(received: paramIndices.count, total: paramCount))
            } else {
                // Ask again for the ones lost on the way, a few at a time.
                paramRounds += 1
                paramLastRx = now
                var asked = 0
                for i in 0..<paramCount where !paramIndices.contains(i) {
                    requestParam(i); asked += 1
                    if asked >= 20 { break }
                }
            }
        }
        // Writes: up to 4 in flight, each retried twice.
        var inFlight = 0
        for name in paramOrder {
            guard let s = paramSets[name] else { continue }
            if s.sentAt != 0 && now - s.sentAt < 1200 { inFlight += 1; continue }
            if s.tries >= 3 {
                paramSets.removeValue(forKey: name); paramOrder.removeAll { $0 == name }
                post(.paramWritten(name: name, ok: false, value: params[name]?.value ?? .nan))
                continue
            }
            if inFlight >= 4 { break }
            s.tries += 1; s.sentAt = now; inFlight += 1
            sendParamSet(s.name, s.value)
        }
    }

    // =========================================================================
    // Mission and fence transfers: the same protocol, told apart by mission_type.
    // Waypoint lists start with home (item 0, as ArduPilot stores it); fences don't.
    // =========================================================================

    func uploadMission(_ items: [MissionItem]) {
        if tx != .idle { return }
        let home = MissionItem(
            lat: hasHome ? homeLat : hasPosition ? lat : (items.first?.lat ?? 0),
            lon: hasHome ? homeLon : hasPosition ? lon : (items.first?.lon ?? 0),
            frame: Mav.FRAME_GLOBAL)
        startUpload(Vehicle.MISSION_TYPE_MISSION, [home] + items)
    }

    /// Fence items as Fence.toItems makes them; an empty list clears the fence.
    func uploadFence(_ items: [MissionItem]) {
        if tx != .idle { return }
        startUpload(Vehicle.MISSION_TYPE_FENCE, items)
    }

    func downloadMission() { startDownload(Vehicle.MISSION_TYPE_MISSION) }
    func downloadFence() { startDownload(Vehicle.MISSION_TYPE_FENCE) }

    private func startUpload(_ type: Int, _ items: [MissionItem]) {
        txItems = items
        txType = type
        tx = .upload; txTries = 0
        missionProgress = "Uploading…"
        sendMissionCount()
    }

    private func startDownload(_ type: Int) {
        if tx != .idle { return }
        txType = type
        tx = .download; txTries = 0; txCount = -1; txNext = 0
        txItems = []
        missionProgress = "Downloading…"
        sendMissionRequestList()
    }

    func clearMission() {
        var b = Mav.payload(Mav.MISSION_CLEAR_ALL)
        b.put8(0, sysId); b.put8(1, compId); b.put8(2, Vehicle.MISSION_TYPE_MISSION)
        send(Mav.MISSION_CLEAR_ALL, b)
    }

    private func sendMissionCount() {
        var b = Mav.payload(Mav.MISSION_COUNT)
        b.put16(0, txItems.count); b.put8(2, sysId); b.put8(3, compId)
        b.put8(4, txType)
        send(Mav.MISSION_COUNT, b)
        txSentAt = clockMs()
    }

    private func sendMissionRequestList() {
        var b = Mav.payload(Mav.MISSION_REQUEST_LIST)
        b.put8(0, sysId); b.put8(1, compId); b.put8(2, txType)
        send(Mav.MISSION_REQUEST_LIST, b)
        txSentAt = clockMs()
    }

    private func sendMissionRequest(_ seq: Int) {
        var b = Mav.payload(Mav.MISSION_REQUEST_INT)
        b.put16(0, seq); b.put8(2, sysId); b.put8(3, compId)
        b.put8(4, txType)
        send(Mav.MISSION_REQUEST_INT, b)
        txSentAt = clockMs()
    }

    private func sendMissionItem(_ seq: Int) {
        let it = txItems[seq]
        var b = Mav.payload(Mav.MISSION_ITEM_INT)
        b.putF(0, it.p1); b.putF(4, it.p2); b.putF(8, it.p3); b.putF(12, it.p4)
        b.put32(16, Int((it.lat * 1e7).rounded())); b.put32(20, Int((it.lon * 1e7).rounded())); b.putF(24, it.alt)
        b.put16(28, seq); b.put16(30, it.command)
        b.put8(32, sysId); b.put8(33, compId)
        b.put8(34, it.frame); b.put8(35, (seq == 0 && txType == Vehicle.MISSION_TYPE_MISSION) ? 1 : 0)
        b.put8(36, it.autocontinue); b.put8(37, txType)
        send(Mav.MISSION_ITEM_INT, b)
        txSentAt = clockMs()
    }

    private func sendMissionAck(_ result: Int) {
        var b = Mav.payload(Mav.MISSION_ACK)
        b.put8(0, sysId); b.put8(1, compId); b.put8(2, result); b.put8(3, txType)
        send(Mav.MISSION_ACK, b)
    }

    private func onMission(_ m: MavPacket) {
        if tx == .idle { return }
        let p = m.p
        // mission_type sits after the fields every version has; ignore the other list's traffic.
        let type: Int
        switch m.msgId {
        case Mav.MISSION_REQUEST_INT, Mav.MISSION_REQUEST, Mav.MISSION_COUNT: type = p.u8(4)
        case Mav.MISSION_ITEM_INT: type = p.u8(37)
        case Mav.MISSION_ACK: type = p.u8(3)
        default: return
        }
        if type != txType { return }
        let fence = txType == Vehicle.MISSION_TYPE_FENCE
        let what = fence ? "Fence" : "Mission"
        switch tx {
        case .upload:
            if m.msgId == Mav.MISSION_REQUEST_INT || m.msgId == Mav.MISSION_REQUEST {
                let seq = p.u16(0)
                if seq < txItems.count {
                    txTries = 0
                    missionProgress = "Uploading \(seq + 1)/\(txItems.count)"
                    sendMissionItem(seq)
                }
            } else if m.msgId == Mav.MISSION_ACK {
                let result = p.u8(2)
                tx = .idle
                missionProgress = ""
                let n = fence ? txItems.count : txItems.count - 1
                post(.missionUploaded(ok: result == 0,
                                      text: result == 0 ? "\(what) uploaded: \(n) items" : "\(what) upload refused: \(Vehicle.missionResult(result))",
                                      type: txType))
            }
        case .download:
            if m.msgId == Mav.MISSION_COUNT {
                if txCount >= 0 { return }
                txCount = p.u16(0); txTries = 0
                if txCount == 0 {
                    tx = .idle; missionProgress = ""
                    sendMissionAck(0)
                    post(.missionDownloaded(items: [], text: "No \(what.lowercased()) on the vehicle", type: txType))
                } else { sendMissionRequest(0) }
            } else if m.msgId == Mav.MISSION_ITEM_INT {
                let seq = p.u16(28)
                if seq != txNext { return }
                txItems.append(MissionItem(
                    command: p.u16(30), lat: Double(p.i32(16)) / 1e7, lon: Double(p.i32(20)) / 1e7, alt: p.f32(24),
                    p1: p.f32(0), p2: p.f32(4), p3: p.f32(8), p4: p.f32(12),
                    frame: p.u8(34), autocontinue: p.u8(36)))
                txNext += 1; txTries = 0
                missionProgress = "Downloading \(txNext)/\(txCount)"
                if txNext < txCount { sendMissionRequest(txNext) } else {
                    tx = .idle; missionProgress = ""
                    sendMissionAck(0)
                    // A mission's item 0 is home: report the rest.
                    let items = fence ? txItems : Array(txItems.dropFirst())
                    post(.missionDownloaded(items: items, text: "\(what) downloaded: \(items.count) items", type: txType))
                }
            }
        case .idle:
            break
        }
    }

    private func tickMission(_ now: Int64) {
        if tx == .idle || now - txSentAt < 1500 { return }
        txTries += 1
        if txTries > 5 {
            let up = tx == .upload
            tx = .idle; missionProgress = ""
            let what = txType == Vehicle.MISSION_TYPE_FENCE ? "Fence" : "Mission"
            if up { post(.missionUploaded(ok: false, text: "\(what) upload timed out", type: txType)) }
            else { post(.missionDownloaded(items: nil, text: "\(what) download timed out", type: txType)) }
            return
        }
        switch tx {
        case .upload: sendMissionCount()
        case .download: if txCount < 0 { sendMissionRequestList() } else { sendMissionRequest(txNext) }
        case .idle: break
        }
    }

    static func missionResult(_ t: Int) -> String {
        switch t {
        case 1: return "error (see the Log tab)"
        case 2: return "unsupported frame"
        case 3: return "unsupported command"
        case 4: return "no space"
        case 5: return "invalid"
        case 6...12: return "invalid param\(t - 5)"
        case 13: return "invalid sequence"
        case 14: return "denied"
        case 15: return "cancelled"
        default: return "code \(t)"
        }
    }

    func close() {
        disconnect()
        timer?.invalidate()
    }

    static let MISSION_TYPE_MISSION = 0
    static let MISSION_TYPE_FENCE = 1

    static let ROVER_MODES: [(Int, String)] = [
        (0, "Manual"), (1, "Acro"), (3, "Steering"), (4, "Hold"), (5, "Loiter"),
        (6, "Follow"), (7, "Simple"), (8, "Dock"), (9, "Circle"), (10, "Auto"),
        (11, "RTL"), (12, "SmartRTL"), (15, "Guided"), (16, "Initialising"),
    ]
    static let GPS_FIX = ["No GPS", "No fix", "2D fix", "3D fix", "DGPS", "RTK Float", "RTK Fixed", "Static", "PPP"]
    static let SEVERITY = ["EMERG", "ALERT", "CRIT", "ERROR", "WARN", "NOTICE", "INFO", "DEBUG"]

    static func cmdResult(_ r: Int) -> String {
        switch r {
        case 0: return "accepted"
        case 1: return "temporarily rejected"
        case 2: return "denied"
        case 3: return "unsupported"
        case 4: return "failed"
        case 5: return "in progress"
        case 6: return "cancelled"
        case -1: return "no answer"
        default: return "result \(r)"
        }
    }
}

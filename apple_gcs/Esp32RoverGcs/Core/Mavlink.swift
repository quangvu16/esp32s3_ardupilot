import Foundation

/// The bit of MAVLink this app needs: v1/v2 framing, the X.25 CRC, and the
/// messages it reads or sends. Field offsets, lengths and CRC extras are those of
/// ArduPilot 4.6's generated headers; payloads are little-endian, fields sorted by
/// size. A port of the Android app's core/Mavlink.kt.
enum Mav {
    static let HEARTBEAT = 0
    static let SYS_STATUS = 1
    static let PARAM_REQUEST_READ = 20
    static let PARAM_REQUEST_LIST = 21
    static let PARAM_VALUE = 22
    static let PARAM_SET = 23
    static let GPS_RAW_INT = 24
    static let ATTITUDE = 30
    static let LOCAL_POSITION_NED = 32
    static let GLOBAL_POSITION_INT = 33
    static let MISSION_REQUEST = 40
    static let MISSION_SET_CURRENT = 41
    static let MISSION_CURRENT = 42
    static let MISSION_REQUEST_LIST = 43
    static let MISSION_COUNT = 44
    static let MISSION_CLEAR_ALL = 45
    static let MISSION_ACK = 47
    static let MISSION_REQUEST_INT = 51
    static let NAV_CONTROLLER_OUTPUT = 62
    static let RC_CHANNELS = 65
    static let REQUEST_DATA_STREAM = 66
    static let MANUAL_CONTROL = 69
    static let RC_CHANNELS_OVERRIDE = 70
    static let MISSION_ITEM_INT = 73
    static let VFR_HUD = 74
    static let COMMAND_INT = 75
    static let COMMAND_LONG = 76
    static let COMMAND_ACK = 77
    static let SET_POSITION_TARGET_LOCAL_NED = 84
    static let SET_POSITION_TARGET_GLOBAL_INT = 86
    static let DISTANCE_SENSOR = 132
    static let BATTERY_STATUS = 147
    static let MAG_CAL_PROGRESS = 191
    static let MAG_CAL_REPORT = 192
    static let EKF_STATUS_REPORT = 193
    static let GPS_RTCM_DATA = 233
    static let HOME_POSITION = 242
    static let OBSTACLE_DISTANCE = 330
    static let STATUSTEXT = 253

    /// message id -> (CRC extra, full payload length). Checked against pymavlink.
    static let INFO: [Int: (crc: UInt8, len: Int)] = [
        HEARTBEAT: (50, 9),
        SYS_STATUS: (124, 31),
        PARAM_REQUEST_READ: (214, 20),
        PARAM_REQUEST_LIST: (159, 2),
        PARAM_VALUE: (220, 25),
        PARAM_SET: (168, 23),
        GPS_RAW_INT: (24, 52),
        ATTITUDE: (39, 28),
        LOCAL_POSITION_NED: (185, 28),
        GLOBAL_POSITION_INT: (104, 28),
        MISSION_REQUEST: (230, 5),
        MISSION_SET_CURRENT: (28, 4),
        MISSION_CURRENT: (28, 6),
        MISSION_REQUEST_LIST: (132, 3),
        MISSION_COUNT: (221, 5),
        MISSION_CLEAR_ALL: (232, 3),
        MISSION_ACK: (153, 4),
        MISSION_REQUEST_INT: (196, 5),
        NAV_CONTROLLER_OUTPUT: (183, 26),
        RC_CHANNELS: (118, 42),
        REQUEST_DATA_STREAM: (148, 6),
        MANUAL_CONTROL: (243, 30),
        RC_CHANNELS_OVERRIDE: (124, 38),
        MISSION_ITEM_INT: (38, 38),
        VFR_HUD: (20, 20),
        COMMAND_INT: (158, 35),
        COMMAND_LONG: (152, 33),
        COMMAND_ACK: (143, 10),
        SET_POSITION_TARGET_LOCAL_NED: (143, 53),
        SET_POSITION_TARGET_GLOBAL_INT: (5, 53),
        DISTANCE_SENSOR: (85, 39),
        BATTERY_STATUS: (154, 54),
        MAG_CAL_PROGRESS: (92, 27),
        MAG_CAL_REPORT: (36, 54),
        EKF_STATUS_REPORT: (71, 26),
        GPS_RTCM_DATA: (35, 182),
        HOME_POSITION: (104, 60),
        OBSTACLE_DISTANCE: (23, 167),
        STATUSTEXT: (83, 54),
    ]

    // MAV_CMD
    static let CMD_NAV_WAYPOINT = 16
    static let CMD_NAV_LOITER_TIME = 19
    static let CMD_NAV_RETURN_TO_LAUNCH = 20
    static let CMD_DO_JUMP = 177
    static let CMD_DO_SET_MODE = 176
    static let CMD_DO_CHANGE_SPEED = 178
    static let CMD_DO_SET_HOME = 179
    static let CMD_PREFLIGHT_CALIBRATION = 241
    static let CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
    static let CMD_MISSION_START = 300
    static let CMD_COMPONENT_ARM_DISARM = 400
    static let CMD_DO_START_MAG_CAL = 42424
    static let CMD_DO_ACCEPT_MAG_CAL = 42425
    static let CMD_DO_CANCEL_MAG_CAL = 42426
    static let CMD_ACCELCAL_VEHICLE_POS = 42429

    static let FRAME_GLOBAL = 0
    static let FRAME_GLOBAL_RELATIVE_ALT = 3
    static let FRAME_GLOBAL_RELATIVE_ALT_INT = 6
    static let FRAME_BODY_NED = 8

    static func crcAccumulate(_ b: UInt8, _ crc: UInt16) -> UInt16 {
        var tmp = b ^ UInt8(truncatingIfNeeded: crc)
        tmp ^= tmp << 4
        let t = UInt16(tmp)
        return (crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)
    }

    static func crc(_ buf: [UInt8], _ from: Int, _ to: Int, extra: UInt8) -> UInt16 {
        var c: UInt16 = 0xffff
        var i = from
        while i < to { c = crcAccumulate(buf[i], c); i += 1 }
        return crcAccumulate(extra, c)
    }

    /// A zeroed payload of the message's full length.
    static func payload(_ msgId: Int) -> Payload { Payload(count: INFO[msgId]!.len) }
}

/// A little-endian payload buffer, read or written by byte offset.
struct Payload {
    var bytes: [UInt8]

    init(count: Int) { bytes = [UInt8](repeating: 0, count: count) }
    init(bytes: [UInt8]) { self.bytes = bytes }

    // --- read
    func u8(_ at: Int) -> Int { Int(bytes[at]) }
    func i8(_ at: Int) -> Int { Int(Int8(bitPattern: bytes[at])) }
    func u16(_ at: Int) -> Int { Int(bytes[at]) | Int(bytes[at + 1]) << 8 }
    func i16(_ at: Int) -> Int { Int(Int16(bitPattern: UInt16(u16(at)))) }
    func u32(_ at: Int) -> Int64 {
        Int64(bytes[at]) | Int64(bytes[at + 1]) << 8 | Int64(bytes[at + 2]) << 16 | Int64(bytes[at + 3]) << 24
    }
    func i32(_ at: Int) -> Int { Int(Int32(bitPattern: UInt32(u32(at)))) }
    func f32(_ at: Int) -> Float { Float(bitPattern: UInt32(u32(at))) }
    func str(_ at: Int, _ max: Int) -> String {
        var out = [UInt8]()
        for i in 0..<max { let c = bytes[at + i]; if c == 0 { break }; out.append(c) }
        return String(decoding: out, as: UTF8.self)
    }

    // --- write
    mutating func put8(_ at: Int, _ v: Int) { bytes[at] = UInt8(truncatingIfNeeded: v) }
    mutating func put16(_ at: Int, _ v: Int) {
        let u = UInt16(truncatingIfNeeded: v)
        bytes[at] = UInt8(u & 0xff); bytes[at + 1] = UInt8(u >> 8)
    }
    mutating func put32(_ at: Int, _ v: Int) {
        let u = UInt32(truncatingIfNeeded: v)
        for k in 0..<4 { bytes[at + k] = UInt8((u >> (8 * UInt32(k))) & 0xff) }
    }
    mutating func putF(_ at: Int, _ v: Float) { put32(at, Int(v.bitPattern)) }
    mutating func putString(_ at: Int, _ s: String, max: Int) {
        let b = Array(s.utf8)
        for i in 0..<min(b.count, max) { bytes[at + i] = b[i] }
    }
}

/// A received message; [p] is zero-padded back to full length (v2 trims trailing zeros).
struct MavPacket {
    let msgId: Int
    let sysId: Int
    let compId: Int
    let p: Payload
}

/// Splits a byte stream into packets. Frames of unknown messages are skipped (no CRC extra to check them).
final class MavParser {
    private var buf = [UInt8]()
    private(set) var crcErrors = 0

    func feed(_ data: Data, _ out: (MavPacket) -> Void) {
        buf.append(contentsOf: data)
        var i = 0
        let len = buf.count
        while i < len {
            let magic = buf[i]
            if magic != 0xFD && magic != 0xFE { i += 1; continue }
            if len - i < 2 { break }
            let plen = Int(buf[i + 1])
            let v2 = magic == 0xFD
            let hdr = v2 ? 10 : 6
            if len - i < hdr { break }
            let signed = v2 && (buf[i + 2] & 0x01) != 0
            let total = hdr + plen + 2 + (signed ? 13 : 0)
            if len - i < total { break }
            let msgId = v2 ? (Int(buf[i + 7]) | Int(buf[i + 8]) << 8 | Int(buf[i + 9]) << 16) : Int(buf[i + 5])
            guard let info = Mav.INFO[msgId] else {
                // Can't check the CRC; trust the length only if the next byte looks like a frame start.
                let next = i + total
                if next >= len || buf[next] == 0xFD || buf[next] == 0xFE { i = next } else { i += 1 }
                continue
            }
            let crc = Mav.crc(buf, i + 1, i + hdr + plen, extra: info.crc)
            let got = UInt16(buf[i + hdr + plen]) | UInt16(buf[i + hdr + plen + 1]) << 8
            if crc != got { crcErrors += 1; i += 1; continue }
            var p = [UInt8](repeating: 0, count: max(info.len, plen))
            for k in 0..<plen { p[k] = buf[i + hdr + k] }
            let sys = Int(buf[i + (v2 ? 5 : 3)])
            let comp = Int(buf[i + (v2 ? 6 : 4)])
            out(MavPacket(msgId: msgId, sysId: sys, compId: comp, p: Payload(bytes: p)))
            i += total
        }
        buf.removeFirst(i)
    }
}

/// Builds MAVLink 2 frames as the GCS (system 255, component 190, like Mission Planner).
final class MavEncoder {
    private let sysId: UInt8
    private let compId: UInt8
    private var seq: UInt8 = 0

    init(sysId: UInt8 = 255, compId: UInt8 = 190) { self.sysId = sysId; self.compId = compId }

    func frame(_ msgId: Int, _ payload: Payload) -> Data {
        let full = payload.bytes
        var n = full.count
        while n > 1 && full[n - 1] == 0 { n -= 1 }      // v2 payload truncation
        var f = [UInt8](repeating: 0, count: 10 + n + 2)
        f[0] = 0xFD
        f[1] = UInt8(n)
        f[4] = seq; seq &+= 1
        f[5] = sysId
        f[6] = compId
        f[7] = UInt8(msgId & 0xff); f[8] = UInt8((msgId >> 8) & 0xff); f[9] = UInt8((msgId >> 16) & 0xff)
        for k in 0..<n { f[10 + k] = full[k] }
        let crc = Mav.crc(f, 1, 10 + n, extra: Mav.INFO[msgId]!.crc)
        f[10 + n] = UInt8(crc & 0xff); f[11 + n] = UInt8(crc >> 8)
        return Data(f)
    }
}

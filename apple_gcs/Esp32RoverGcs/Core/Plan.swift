import Foundation

// Fences, zig-zag lanes and patrols: ports of core/Fence.kt, ZigZag.kt and Patrol.kt.

/// Geofence zones as ArduPilot stores them (AC_PolyFence_loader): inclusion and
/// exclusion polygons and circles, and an optional return point. They travel as
/// mission items of mission_type FENCE, one item per polygon vertex.
struct FenceZone: Identifiable {
    enum Kind: String, CaseIterable, Identifiable {
        case polygonIn = "Stay-in polygon", polygonOut = "Stay-out polygon"
        case circleIn = "Stay-in circle", circleOut = "Stay-out circle"
        case returnPoint = "Return point"
        var id: String { rawValue }
        var polygon: Bool { self == .polygonIn || self == .polygonOut }
        var inclusion: Bool { self == .polygonIn || self == .circleIn }
        var circle: Bool { self == .circleIn || self == .circleOut }
    }

    var id = UUID()
    var kind: Kind
    var points: [Coord] = []
    var radius: Float = 0
}

struct Coord: Equatable { var lat: Double; var lon: Double }

enum Fence {
    static let CMD_RETURN_POINT = 5000
    static let CMD_POLYGON_INCLUSION = 5001
    static let CMD_POLYGON_EXCLUSION = 5002
    static let CMD_CIRCLE_INCLUSION = 5003
    static let CMD_CIRCLE_EXCLUSION = 5004

    /// Fence storage on the esp32s3devkit board (16 KB parameter storage, Rover): 160 + 256 + 256 bytes.
    static let BOARD_STORAGE_BYTES = 672

    static func toItems(_ zones: [FenceZone]) -> [MissionItem] {
        var out = [MissionItem]()
        for z in zones {
            switch z.kind {
            case .polygonIn, .polygonOut:
                let cmd = z.kind == .polygonIn ? CMD_POLYGON_INCLUSION : CMD_POLYGON_EXCLUSION
                for p in z.points { out.append(item(cmd, p, Float(z.points.count))) }
            case .circleIn, .circleOut:
                let cmd = z.kind == .circleIn ? CMD_CIRCLE_INCLUSION : CMD_CIRCLE_EXCLUSION
                if let c = z.points.first { out.append(item(cmd, c, z.radius)) }
            case .returnPoint:
                if let c = z.points.first { out.append(item(CMD_RETURN_POINT, c, 0)) }
            }
        }
        return out
    }

    private static func item(_ cmd: Int, _ p: Coord, _ p1: Float) -> MissionItem {
        MissionItem(command: cmd, lat: p.lat, lon: p.lon, p1: p1, frame: Mav.FRAME_GLOBAL_RELATIVE_ALT_INT)
    }

    /// Groups downloaded items back into zones; consecutive vertices of one polygon carry its vertex count in param1.
    static func fromItems(_ items: [MissionItem]) -> [FenceZone] {
        var zones = [FenceZone]()
        var i = 0
        while i < items.count {
            let it = items[i]
            let c = Coord(lat: it.lat, lon: it.lon)
            switch it.command {
            case CMD_POLYGON_INCLUSION, CMD_POLYGON_EXCLUSION:
                let n = max(1, Int(it.p1))
                var z = FenceZone(kind: it.command == CMD_POLYGON_INCLUSION ? .polygonIn : .polygonOut)
                for k in i..<min(i + n, items.count) { z.points.append(Coord(lat: items[k].lat, lon: items[k].lon)) }
                zones.append(z)
                i += n
                continue
            case CMD_CIRCLE_INCLUSION: zones.append(FenceZone(kind: .circleIn, points: [c], radius: it.p1))
            case CMD_CIRCLE_EXCLUSION: zones.append(FenceZone(kind: .circleOut, points: [c], radius: it.p1))
            case CMD_RETURN_POINT: zones.append(FenceZone(kind: .returnPoint, points: [c]))
            default: break
            }
            i += 1
        }
        return zones
    }

    /// Bytes ArduPilot needs to store these zones (AC_PolyFence_loader::fence_storage_space_required).
    static func storageBytes(_ zones: [FenceZone]) -> Int {
        4 + zones.reduce(0) { sum, z in
            sum + 1 + (z.kind.polygon ? 1 + 8 * z.points.count : z.kind.circle ? 12 : 8)
        }
    }

    /// The first problem ArduPilot would refuse the upload for, or nil.
    static func problem(_ zones: [FenceZone]) -> String? {
        if zones.filter({ $0.kind == .returnPoint }).count > 1 { return "Only one return point is allowed" }
        for (i, z) in zones.enumerated() {
            let name = "\(z.kind.rawValue) \(i + 1)"
            if z.kind.polygon && z.points.count < 3 { return "\(name) needs at least 3 corners (has \(z.points.count))" }
            if z.kind.polygon && z.points.count > 255 { return "\(name) has more than 255 corners" }
            if z.kind.circle && z.radius <= 0 { return "\(name) needs a radius" }
            if z.points.isEmpty { return "\(name) has no position" }
        }
        return nil
    }
}

/// Copter's ZigZag mode as a Rover mission: drive A to B, step sideways, drive back
/// parallel to it, and so on. Rover has no ZigZag mode, so the app writes the lanes as
/// waypoints and the car runs them in Auto. runOutM extends each line past both ends so
/// the car turns outside the area it covers.
struct ZigZag {
    var a: Coord?
    var b: Coord?
    var sideDistM: Float = 2          // ZIGZ_SIDE_DIST
    var right = true                  // ZIGZ_DIRECTION 1 (right) or 3 (left) of A->B
    var lines = 5                     // ZIGZ_LINE_NUM
    var waitS: Float = 0              // ZIGZ_WP_DELAY
    var runOutM: Float = 0

    var lengthM: Double {
        guard let p = a, let q = b else { return 0 }
        return ZigZag.distance(p, q)
    }

    /// Why the lanes can't be made yet, or nil.
    func problem() -> String? {
        if a == nil { return "Set point A" }
        if b == nil { return "Set point B" }
        if lengthM < 1 { return "A and B are less than 1 m apart" }
        if sideDistM <= 0 { return "Line spacing must be more than 0" }
        if lines < 1 { return "At least 1 line" }
        if lines > 200 { return "At most 200 lines" }
        return nil
    }

    /// Line 1 runs A to B, line 2 back from B' to A' one spacing to the side, and so on.
    /// Line 1 starts at A itself (usually where the car stands).
    func waypoints() -> [MissionItem] {
        guard let p = a, let q = b else { return [] }
        let len = ZigZag.distance(p, q)
        let (bx, by) = ZigZag.toLocal(p, q)                 // x east, y north, metres from A
        let ux = bx / len, uy = by / len
        let sx = right ? uy : -uy, sy = right ? -ux : ux     // A->B turned 90° right or left
        var out = [MissionItem]()
        for i in 0..<lines {
            let off = Double(i) * Double(sideDistM)
            let r = Double(runOutM)
            let rs = i == 0 ? 0 : r                          // no run-out behind A on the first line
            let ax = sx * off - ux * rs, ay = sy * off - uy * rs
            let ex = bx + sx * off + ux * r, ey = by + sy * off + uy * r
            let (x1, y1, x2, y2) = i % 2 == 0 ? (ax, ay, ex, ey) : (ex, ey, ax, ay)
            out.append(wp(p, x1, y1, 0))
            out.append(wp(p, x2, y2, waitS))
        }
        return out
    }

    private func wp(_ origin: Coord, _ x: Double, _ y: Double, _ wait: Float) -> MissionItem {
        let c = ZigZag.fromLocal(origin, x, y)
        return MissionItem(lat: c.lat, lon: c.lon, p1: wait)
    }

    static let M_PER_DEG = 111320.0

    static func toLocal(_ o: Coord, _ p: Coord) -> (Double, Double) {
        ((p.lon - o.lon) * M_PER_DEG * cos(o.lat * .pi / 180), (p.lat - o.lat) * M_PER_DEG)
    }

    static func fromLocal(_ o: Coord, _ x: Double, _ y: Double) -> Coord {
        Coord(lat: o.lat + y / M_PER_DEG, lon: o.lon + x / (M_PER_DEG * cos(o.lat * .pi / 180)))
    }

    static func distance(_ p: Coord, _ q: Coord) -> Double {
        let (x, y) = toLocal(p, q)
        return hypot(x, y)
    }
}

/// Patrol: drive the plan round and round in Auto, with ArduPilot's BendyRuler
/// (OA_TYPE 1) steering round whatever the LiDAR sees. The loop is a DO_JUMP at the
/// end of the mission, so the car keeps patrolling even if the app loses the link.
enum Patrol {
    /// BendyRuler settings for a small car with the LD14 (README step 8), tested in SITL.
    static let BENDY_RULER: [(String, Float)] = [
        ("OA_TYPE", 1),            // BendyRuler
        ("OA_MARGIN_MAX", 2),      // m kept from obstacles; above WP_RADIUS
        ("WP_RADIUS", 1),          // m; below OA_MARGIN_MAX, at least TURN_RADIUS
        ("OA_BR_LOOKAHEAD", 5),    // m probed ahead, under the LD14's 8 m
        ("OA_DB_DIST_MAX", 8),     // m, the LD14's range
        ("WP_SPEED", 1),           // m/s
        ("AVOID_MARGIN", 1),       // m; below OA_MARGIN_MAX
    ]

    /// The plan as a patrol: any DO_JUMP already at its end is replaced, the last
    /// positioned waypoint waits waitS at the end of each lap, and a DO_JUMP back to
    /// the first item repeats it. laps 0 = forever.
    static func apply(_ plan: [MissionItem], laps: Int, waitS: Float) -> [MissionItem] {
        var items = plan
        while let l = items.last, l.command == Mav.CMD_DO_JUMP { items.removeLast() }
        if let i = items.lastIndex(where: { $0.command == Mav.CMD_NAV_WAYPOINT && $0.hasPosition }) {
            items[i].p1 = max(0, waitS)
        }
        if laps == 1 { return items }
        // Mission item 1 is the plan's first item (item 0 is home). num_times counts repeats.
        items.append(MissionItem(command: Mav.CMD_DO_JUMP, p1: 1, p2: laps <= 0 ? -1 : Float(laps - 1)))
        return items
    }

    /// Laps a plan drives: 1 without a DO_JUMP at the end, 0 = forever.
    static func laps(_ plan: [MissionItem]) -> Int {
        guard let j = plan.last, j.command == Mav.CMD_DO_JUMP else { return 1 }
        return j.p2 < 0 ? 0 : Int(j.p2) + 1
    }

    /// What stops BendyRuler from working well, given the vehicle's parameters.
    static func bendyRulerProblems(_ param: (String) -> Float?) -> [String] {
        guard let prx = param("PRX1_TYPE") else { return ["Read the parameters first"] }
        var out = [String]()
        if prx == 0 { out.append("No LiDAR: PRX1_TYPE is 0 (19 = LD14)") }
        let oa = param("OA_TYPE")
        let bendy = oa == 1 || oa == 3
        if !bendy { out.append("OA_TYPE is \(oa.map { String(Int($0)) } ?? "?"): BendyRuler is 1 (or 3 with fences)") }
        if bendy && param("OA_BR_LOOKAHEAD") == nil { out.append("Refresh the parameters: the OA_BR_ ones appear after the reboot") }
        for (k, want) in BENDY_RULER where k != "OA_TYPE" {
            if let v = param(k), v != want { out.append("\(k) is \(fmt(v)) (suggested \(fmt(want)))") }
        }
        if let margin = param("OA_MARGIN_MAX"), let wpr = param("WP_RADIUS"), margin <= wpr {
            out.append("OA_MARGIN_MAX (\(fmt(margin))) should be above WP_RADIUS (\(fmt(wpr)))")
        }
        let avoid = Int(param("AVOID_ENABLE") ?? 0)
        if avoid & 2 != 0, let margin = param("OA_MARGIN_MAX"), let am = param("AVOID_MARGIN"), am >= margin {
            out.append("AVOID_MARGIN (\(fmt(am))) stops the car before BendyRuler steers: keep it below OA_MARGIN_MAX")
        }
        return out
    }

    /// What "Set up BendyRuler" writes now, and whether a reboot must follow (the OA_BR_
    /// parameters only exist once the board has rebooted with OA_TYPE 1).
    static func setupWrites(_ param: (String) -> Float?) -> (writes: [(String, Float)], reboot: Bool) {
        let oa = param("OA_TYPE")
        let bendyNow = oa == 1 || oa == 3
        let writes = BENDY_RULER.filter { k, v in
            k == "OA_TYPE" ? !bendyNow : (param(k) != nil && param(k) != v)
        }
        return (writes, !bendyNow)
    }

    static func fmt(_ f: Float) -> String {
        (f.isFinite && abs(f) < 1e15 && f == f.rounded()) ? String(Int(f)) : String(f)
    }
}

/// Counts patrol laps from MISSION_CURRENT: a lap ends when Auto jumps back to an earlier item.
struct LapCounter {
    private(set) var lap = 0
    private var lastSeq = -1

    mutating func reset() { lap = 0; lastSeq = -1 }

    mutating func update(_ seq: Int, inAuto: Bool) -> Int {
        if !inAuto { lastSeq = -1; return lap }
        if lap == 0 { lap = 1 }
        if lastSeq >= 0 && seq < lastSeq { lap += 1 }
        lastSeq = seq
        return lap
    }
}

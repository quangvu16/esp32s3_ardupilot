import Foundation

/*
 Indoor navigation, after ROS 2 Nav2: a saved map, AMCL, a costmap, A*, Regulated
 Pure Pursuit and a small behaviour state machine. A port of the Android/Windows
 core (android_gcs/core/.../nav); the comments there explain the details.

 Frames, as ArduPilot's: x north, y east (m), yaw clockwise from north (rad).
 Robot frame: x forward, y right. "odom" is the car's own frame (EKF / onboard
 LiDAR SLAM, LOCAL_POSITION_NED); "map" the saved map's. AMCL finds map ← odom.
 */

func wrapPi(_ a: Double) -> Double {
    var r = (a + .pi).truncatingRemainder(dividingBy: 2 * .pi)
    if r < 0 { r += 2 * .pi }
    return r - .pi
}

/// A 2D pose or transform.
struct Pose2: Equatable {
    var x = 0.0, y = 0.0, yaw = 0.0

    /// self ⊕ p: p (in this pose's frame) in the outer frame.
    static func + (a: Pose2, p: Pose2) -> Pose2 {
        let c = cos(a.yaw), s = sin(a.yaw)
        return Pose2(x: a.x + c * p.x - s * p.y, y: a.y + s * p.x + c * p.y, yaw: wrapPi(a.yaw + p.yaw))
    }
    func inverse() -> Pose2 {
        let c = cos(yaw), s = sin(yaw)
        return Pose2(x: -(c * x + s * y), y: -(-s * x + c * y), yaw: wrapPi(-yaw))
    }
    func apply(_ px: Double, _ py: Double) -> (Double, Double) {
        let c = cos(yaw), s = sin(yaw)
        return (x + c * px - s * py, y + s * px + c * py)
    }
    func toLocal(_ px: Double, _ py: Double) -> (Double, Double) {
        let c = cos(yaw), s = sin(yaw), dx = px - x, dy = py - y
        return (c * dx + s * dy, -s * dx + c * dy)
    }
    func distance(to o: Pose2) -> Double { hypot(o.x - x, o.y - y) }
}

/// A named goal in the map frame.
struct Waypoint: Equatable, Identifiable {
    var name: String
    var x: Double
    var y: Double
    var id: String { name }
}

/// A small, seedable random source with Gaussian samples (SplitMix64 + Box-Muller).
struct NavRandom {
    private var state: UInt64
    private var spare: Double?
    init(seed: UInt64) { state = seed &+ 0x9E3779B97F4A7C15 }
    mutating func next() -> UInt64 {
        state &+= 0x9E3779B97F4A7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58476D1CE4E5B9
        z = (z ^ (z >> 27)) &* 0x94D049BB133111EB
        return z ^ (z >> 31)
    }
    mutating func uniform() -> Double { Double(next() >> 11) / Double(1 << 53) }
    mutating func gaussian() -> Double {
        if let s = spare { spare = nil; return s }
        var u = 0.0
        repeat { u = uniform() } while u <= 1e-12
        let v = uniform()
        let r = (-2 * log(u)).squareRoot()
        spare = r * sin(2 * .pi * v)
        return r * cos(2 * .pi * v)
    }
}

/// An occupancy grid: width cells east by height cells north, res m a cell, cell
/// (0, 0) at originX north / originY east. Log-odds -127 (free) .. 127 (occupied).
final class OccGrid {
    let width: Int, height: Int
    let res: Double, originX: Double, originY: Double
    var cells: [Int8]

    static let occThreshold: Int8 = 12       // Nav2 map_saver occupied_thresh 0.65, as log-odds x 20
    static let freeThreshold: Int8 = -21     // free_thresh 0.25

    init(width: Int, height: Int, res: Double, originX: Double, originY: Double) {
        self.width = width; self.height = height; self.res = res; self.originX = originX; self.originY = originY
        cells = [Int8](repeating: 0, count: width * height)
    }

    func row(_ x: Double) -> Int { Int(floor((x - originX) / res)) }
    func col(_ y: Double) -> Int { Int(floor((y - originY) / res)) }
    func inside(_ r: Int, _ c: Int) -> Bool { r >= 0 && r < height && c >= 0 && c < width }
    func index(_ r: Int, _ c: Int) -> Int { r * width + c }
    func cellX(_ r: Int) -> Double { originX + (Double(r) + 0.5) * res }
    func cellY(_ c: Int) -> Double { originY + (Double(c) + 0.5) * res }
    func occupied(_ i: Int) -> Bool { cells[i] > OccGrid.occThreshold }
    func free(_ i: Int) -> Bool { cells[i] < OccGrid.freeThreshold }
    func known(_ i: Int) -> Bool { occupied(i) || free(i) }

    func copy() -> OccGrid {
        let g = OccGrid(width: width, height: height, res: res, originX: originX, originY: originY)
        g.cells = cells
        return g
    }
}

/// Maps on disk in the ROS map_server format (PGM + YAML), with <name>.json for the waypoints and patrol.
enum MapFiles {
    struct Saved { let grid: OccGrid; let waypoints: [Waypoint]; let patrol: [String] }

    static func list(_ dir: URL) -> [String] {
        let fs = (try? FileManager.default.contentsOfDirectory(atPath: dir.path)) ?? []
        return fs.filter { $0.hasSuffix(".yaml") }.map { String($0.dropLast(5)) }.sorted()
    }

    static func save(_ dir: URL, _ name: String, _ g: OccGrid, _ wps: [Waypoint], _ patrol: [String]) throws {
        try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        var pgm = Array("P5\n# esp32s3_ardupilot indoor map\n\(g.width) \(g.height)\n255\n".utf8)
        pgm.reserveCapacity(pgm.count + g.width * g.height)
        for r in stride(from: g.height - 1, through: 0, by: -1) {
            for c in 0..<g.width {
                let i = g.index(r, c)
                pgm.append(g.occupied(i) ? 0 : g.free(i) ? 254 : 205)
            }
        }
        try Data(pgm).write(to: dir.appendingPathComponent("\(name).pgm"))
        let yaml = "image: \(name).pgm\nmode: trinary\nresolution: \(fmt(g.res))\n" +
            "origin: [\(fmt(g.originY)), \(fmt(g.originX)), 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
        try yaml.write(to: dir.appendingPathComponent("\(name).yaml"), atomically: true, encoding: .utf8)
        try saveWaypoints(dir, name, wps, patrol)
    }

    static func saveWaypoints(_ dir: URL, _ name: String, _ wps: [Waypoint], _ patrol: [String]) throws {
        var s = "{\n  \"waypoints\": ["
        for (k, w) in wps.enumerated() {
            s += (k == 0 ? "\n" : ",\n") + "    {\"name\": \(quote(w.name)), \"x\": \(fmt(w.x)), \"y\": \(fmt(w.y))}"
        }
        s += "\n  ],\n  \"patrol\": [" + patrol.map { quote($0) }.joined(separator: ", ") + "]\n}\n"
        try s.write(to: dir.appendingPathComponent("\(name).json"), atomically: true, encoding: .utf8)
    }

    static func load(_ dir: URL, _ name: String) throws -> Saved {
        let yaml = try String(contentsOf: dir.appendingPathComponent("\(name).yaml"), encoding: .utf8)
        func field(_ key: String) -> String? {
            for line in yaml.split(separator: "\n") {
                let t = line.trimmingCharacters(in: .whitespaces)
                if t.hasPrefix(key + ":") { return String(t.dropFirst(key.count + 1)).trimmingCharacters(in: .whitespaces) }
            }
            return nil
        }
        guard let image = field("image")?.trimmingCharacters(in: CharacterSet(charactersIn: "\"'")),
              let res = Double(field("resolution") ?? ""),
              let originS = field("origin") else { throw NSError(domain: "map", code: 1, userInfo: [NSLocalizedDescriptionKey: "\(name).yaml is incomplete"]) }
        let origin = originS.trimmingCharacters(in: CharacterSet(charactersIn: "[]")).split(separator: ",")
            .compactMap { Double($0.trimmingCharacters(in: .whitespaces)) }
        guard origin.count >= 2 else { throw NSError(domain: "map", code: 2, userInfo: [NSLocalizedDescriptionKey: "bad origin"]) }
        let negate = field("negate") == "1"
        let occT = Double(field("occupied_thresh") ?? "") ?? 0.65
        let freeT = Double(field("free_thresh") ?? "") ?? 0.25
        let (w, h, px) = try readPgm(dir.appendingPathComponent(image))
        let g = OccGrid(width: w, height: h, res: res, originX: origin[1], originY: origin[0])
        for r in 0..<h {
            for c in 0..<w {
                let v = Double(px[(h - 1 - r) * w + c])
                let p = negate ? v / 255 : (255 - v) / 255
                g.cells[g.index(r, c)] = p > occT ? 100 : p < freeT ? -100 : 0
            }
        }
        let (wps, patrol) = loadWaypoints(dir.appendingPathComponent("\(name).json"))
        return Saved(grid: g, waypoints: wps, patrol: patrol)
    }

    static func delete(_ dir: URL, _ name: String) {
        for ext in ["yaml", "pgm", "json"] { try? FileManager.default.removeItem(at: dir.appendingPathComponent("\(name).\(ext)")) }
    }

    private static func readPgm(_ url: URL) throws -> (Int, Int, [UInt8]) {
        let b = [UInt8](try Data(contentsOf: url))
        var i = 0
        func token() -> String {
            while i < b.count {
                if b[i] == UInt8(ascii: "#") { while i < b.count && b[i] != 10 { i += 1 } }
                else if b[i] == 32 || b[i] == 10 || b[i] == 13 || b[i] == 9 { i += 1 } else { break }
            }
            let s = i
            while i < b.count && !(b[i] == 32 || b[i] == 10 || b[i] == 13 || b[i] == 9) { i += 1 }
            return String(decoding: b[s..<i], as: UTF8.self)
        }
        guard token() == "P5", let w = Int(token()), let h = Int(token()), let mx = Int(token()), mx <= 255 else {
            throw NSError(domain: "map", code: 3, userInfo: [NSLocalizedDescriptionKey: "not an 8-bit binary PGM"])
        }
        i += 1
        guard b.count - i >= w * h else { throw NSError(domain: "map", code: 4, userInfo: [NSLocalizedDescriptionKey: "PGM too short"]) }
        return (w, h, Array(b[i..<(i + w * h)]))
    }

    private static func loadWaypoints(_ url: URL) -> ([Waypoint], [String]) {
        guard let data = try? Data(contentsOf: url),
              let obj = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else { return ([], []) }
        let wps = (obj["waypoints"] as? [[String: Any]] ?? []).compactMap { d -> Waypoint? in
            guard let n = d["name"] as? String else { return nil }
            return Waypoint(name: n, x: (d["x"] as? NSNumber)?.doubleValue ?? 0, y: (d["y"] as? NSNumber)?.doubleValue ?? 0)
        }
        return (wps, obj["patrol"] as? [String] ?? [])
    }

    private static func fmt(_ v: Double) -> String { String(format: "%.4f", locale: Locale(identifier: "en_US_POSIX"), v) }
    private static func quote(_ s: String) -> String {
        "\"" + s.replacingOccurrences(of: "\\", with: "\\\\").replacingOccurrences(of: "\"", with: "\\\"") + "\""
    }
}

/// Builds a map from scans with known poses: log-odds ray tracing.
final class Mapper {
    let grid: OccGrid
    private(set) var scans = 0
    static let occStep = 24, freeStep = 6, freeStepNoReturn = 2, limit = 120

    init(sizeM: Double = 40, res: Double = 0.1, centre: Pose2 = Pose2()) {
        let n = Int(sizeM / res)
        grid = OccGrid(width: n, height: n, res: res, originX: centre.x - Double(n) * res / 2, originY: centre.y - Double(n) * res / 2)
    }

    func add(_ s: OdomScan) {
        for k in 0..<s.count {
            ray(s.ox[k], s.oy[k], s.px[k], s.py[k], s.hit[k] ? Mapper.freeStep : Mapper.freeStepNoReturn, s.hit[k])
        }
        scans += 1
    }

    private func ray(_ x0: Double, _ y0: Double, _ x1: Double, _ y1: Double, _ freeStep: Int, _ hit: Bool) {
        var r0 = grid.row(x0), c0 = grid.col(y0)
        let r1 = grid.row(x1), c1 = grid.col(y1)
        let dr = abs(r1 - r0), dc = abs(c1 - c0)
        let sr = r1 > r0 ? 1 : -1, sc = c1 > c0 ? 1 : -1
        var err = dc - dr
        let steps = max(dr, dc)
        for s in 0...steps {
            if !grid.inside(r0, c0) { return }
            let i = grid.index(r0, c0)
            let last = s == steps
            let v = Int(grid.cells[i])
            if !hit && grid.occupied(i) { return }      // a ray with no return stops at a known wall
            if last && hit {
                grid.cells[i] = Int8(min(v + Mapper.occStep, Mapper.limit))
            } else if !hit || s < steps - 1 {
                grid.cells[i] = Int8(max(v - freeStep, -Mapper.limit))
            }
            if last { return }
            let e2 = 2 * err
            if e2 > -dr { err -= dr; c0 += sc }
            if e2 < dc { err += dc; r0 += sr }
        }
    }
}

/// One LiDAR rotation: angles clockwise from ahead (rad), ranges in m (+inf: nothing within maxRange).
struct ScanFrame {
    let endMs: Int64
    let angles: [Double]
    let ranges: [Double]
    let maxRange: Double
}

/// The car's recent poses in its own frame, by autopilot time. Thread-safe.
final class PoseHistory {
    private var pos = [(t: Int64, x: Double, y: Double)]()
    private var yaws = [(t: Int64, a: Double)]()
    private let lock = NSLock()

    func addPosition(_ t: Int64, _ x: Double, _ y: Double) {
        lock.lock(); defer { lock.unlock() }
        if let l = pos.last, t < l.t { pos.removeAll(); yaws.removeAll() }
        pos.append((t, x, y)); if pos.count > 256 { pos.removeFirst(pos.count - 256) }
    }
    func addYaw(_ t: Int64, _ a: Double) {
        lock.lock(); defer { lock.unlock() }
        if let l = yaws.last, t < l.t { yaws.removeAll() }
        yaws.append((t, a)); if yaws.count > 256 { yaws.removeFirst(yaws.count - 256) }
    }
    func clear() { lock.lock(); pos.removeAll(); yaws.removeAll(); lock.unlock() }

    func latestMs() -> Int64 {
        lock.lock(); defer { lock.unlock() }
        guard let p = pos.last, let a = yaws.last else { return -1 }
        return min(p.t, a.t)
    }

    func at(_ t: Int64, extrapolateMs: Int64 = 250) -> Pose2? {
        lock.lock(); defer { lock.unlock() }
        guard let p = interp(pos.map { $0.t }, t, extrapolateMs), let a = interp(yaws.map { $0.t }, t, extrapolateMs) else { return nil }
        let x = pos[p.0].x + p.2 * (pos[p.1].x - pos[p.0].x)
        let y = pos[p.0].y + p.2 * (pos[p.1].y - pos[p.0].y)
        let yaw = yaws[a.0].a + a.2 * wrapPi(yaws[a.1].a - yaws[a.0].a)
        return Pose2(x: x, y: y, yaw: wrapPi(yaw))
    }

    private func interp(_ ts: [Int64], _ t: Int64, _ extra: Int64) -> (Int, Int, Double)? {
        guard let newest = ts.last, let oldest = ts.first else { return nil }
        if t >= newest { return t - newest <= extra ? (ts.count - 1, ts.count - 1, 0) : nil }
        if t < oldest || newest - t > 3000 { return nil }
        var k = ts.count - 1
        while k > 0 {
            if ts[k - 1] <= t {
                let span = Double(ts[k] - ts[k - 1])
                return (k - 1, k, span > 0 ? Double(t - ts[k - 1]) / span : 0)
            }
            k -= 1
        }
        return nil
    }
}

/// A scan with the car's motion during the rotation taken out, in the odom frame.
struct OdomScan {
    let end: Pose2
    let ox: [Double], oy: [Double], px: [Double], py: [Double]
    let hit: [Bool]
    let maxRange: Double
    var count: Int { px.count }

    func local() -> ([Double], [Double]) {
        let inv = end.inverse()
        var lx = [Double](repeating: 0, count: count), ly = lx
        for i in 0..<count { let p = inv.apply(px[i], py[i]); lx[i] = p.0; ly[i] = p.1 }
        return (lx, ly)
    }

    func transformed(_ t: Pose2) -> OdomScan {
        var ox2 = ox, oy2 = oy, px2 = px, py2 = py
        for i in 0..<count {
            let o = t.apply(ox[i], oy[i]), p = t.apply(px[i], py[i])
            ox2[i] = o.0; oy2[i] = o.1; px2[i] = p.0; py2[i] = p.1
        }
        return OdomScan(end: t + end, ox: ox2, oy: oy2, px: px2, py: py2, hit: hit, maxRange: maxRange)
    }

    static func build(_ scan: ScanFrame, _ poses: PoseHistory, periodS: Double, sensorX: Double, sensorY: Double,
                      minRange: Double = 0.2, clearRange: Double = 3.0) -> OdomScan? {
        guard let end = poses.at(scan.endMs) else { return nil }
        var ox = [Double](), oy = [Double](), px = [Double](), py = [Double](), hit = [Bool]()
        var cachedTen: Int64 = .min
        var pose = end
        for k in 0..<scan.angles.count {
            var r = scan.ranges[k]
            let isHit = r.isFinite
            if isHit && r < minRange { continue }
            if !isHit { r = clearRange }
            let a = scan.angles[k]
            let t = scan.endMs - Int64((1 - a / (2 * .pi)) * periodS * 1000)
            if t / 10 != cachedTen { pose = poses.at(t) ?? end; cachedTen = t / 10 }
            let s = pose.apply(sensorX, sensorY)
            let w = pose.apply(sensorX + r * cos(a), sensorY + r * sin(a))
            ox.append(s.0); oy.append(s.1); px.append(w.0); py.append(w.1); hit.append(isHit)
        }
        return OdomScan(end: end, ox: ox, oy: oy, px: px, py: py, hit: hit, maxRange: scan.maxRange)
    }
}

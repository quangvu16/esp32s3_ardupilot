import Foundation

/// AMCL with the likelihood-field model (port of nav/Localizer.kt): particles move
/// with the car's own odometry and are weighed by how well the scan lands on walls.
/// The result is offset = map ← odom.
final class Localizer {
    struct Params {
        var particles = 600
        var beams = 72
        var sigmaHit = 0.15
        var zHit = 0.5, zRand = 0.5
        var maxDist = 1.0
        var alpha1 = 0.1, alpha2 = 0.1, alpha3 = 0.1, alpha4 = 0.1
        var updateMinD = 0.10, updateMinA = 0.10
        var initSigmaXY = 0.3, initSigmaYaw = 0.2
    }

    let map: OccGrid
    let p: Params
    private var rnd = NavRandom(seed: 1)
    private var x: [Double], y: [Double], t: [Double], w: [Double]
    private var field: [Float]

    private(set) var offset = Pose2()
    private(set) var pose = Pose2()
    private(set) var spreadXY = Double.greatestFiniteMagnitude
    private(set) var spreadYaw = Double.greatestFiniteMagnitude
    private(set) var score = 0.0
    private(set) var initialised = false
    private var lastOdom: Pose2?
    private var since = (d: 0.0, a: 0.0)
    private var forceUpdates = 0

    var confident: Bool { initialised && spreadXY < 0.25 && spreadYaw < 0.15 && score > 0.4 }

    init(map: OccGrid, params: Params = Params()) {
        self.map = map; p = params
        let n = params.particles
        x = [Double](repeating: 0, count: n); y = x; t = x
        w = [Double](repeating: 1.0 / Double(n), count: n)
        field = [Float](repeating: 0, count: map.width * map.height)
        buildField()
    }

    private var n: Int { p.particles }

    func setPose(_ mapPose: Pose2, odom: Pose2, sigmaXY: Double? = nil, sigmaYaw: Double? = nil) {
        let sx = sigmaXY ?? p.initSigmaXY, sa = sigmaYaw ?? p.initSigmaYaw
        for i in 0..<n {
            x[i] = mapPose.x + rnd.gaussian() * sx
            y[i] = mapPose.y + rnd.gaussian() * sx
            t[i] = wrapPi(mapPose.yaw + rnd.gaussian() * sa)
            w[i] = 1.0 / Double(n)
        }
        lastOdom = odom; initialised = true; forceUpdates = 5
        summarise(odom)
    }

    /// Global localisation: coarse search over free cells and headings, refined, then particles at the best 5.
    @discardableResult
    func findGlobally(_ scan: OdomScan) -> Bool {
        let (lx, ly) = scan.local()
        let idx = thin(lx.count, 30, scan.hit)
        if idx.count < 8 { return false }
        let keep = 30, heads = 48
        var cand = [(x: Double, y: Double, yaw: Double, s: Double)](repeating: (0, 0, 0, -1), count: keep)
        for r in stride(from: 0, to: map.height, by: 2) {
            for c in stride(from: 0, to: map.width, by: 2) where map.free(map.index(r, c)) {
                let px = map.cellX(r), py = map.cellY(c)
                for h in 0..<heads {
                    let yaw = -Double.pi + Double(h) * 2 * .pi / Double(heads)
                    let sc = scoreAt(px, py, yaw, lx, ly, idx, 0.3)
                    if sc > cand[keep - 1].s {
                        var k = keep - 1
                        while k > 0 && cand[k - 1].s < sc { cand[k] = cand[k - 1]; k -= 1 }
                        cand[k] = (px, py, yaw, sc)
                    }
                }
            }
        }
        if cand[0].s <= 0 { return false }
        for k in 0..<keep where cand[k].s > 0 {
            var b = cand[k]
            var best = scoreAt(b.x, b.y, b.yaw, lx, ly, idx, p.sigmaHit)
            var step = 0.1, astep = 0.06
            for _ in 0..<30 {
                var improved = false
                for d in [(step, 0.0, 0.0), (-step, 0.0, 0.0), (0.0, step, 0.0), (0.0, -step, 0.0), (0.0, 0.0, astep), (0.0, 0.0, -astep)] {
                    let sc = scoreAt(b.x + d.0, b.y + d.1, b.yaw + d.2, lx, ly, idx, p.sigmaHit)
                    if sc > best { best = sc; b.x += d.0; b.y += d.1; b.yaw += d.2; improved = true }
                }
                if !improved { step /= 2; astep /= 2 }
            }
            b.yaw = wrapPi(b.yaw); b.s = best
            cand[k] = b
        }
        let order = cand.indices.filter { cand[$0].s > 0 }.sorted { cand[$0].s > cand[$1].s }.prefix(5).map { $0 }
        for i in 0..<n {
            let b = cand[order[i % order.count]]
            x[i] = b.x + rnd.gaussian() * 0.1; y[i] = b.y + rnd.gaussian() * 0.1
            t[i] = wrapPi(b.yaw + rnd.gaussian() * 0.05); w[i] = 1.0 / Double(n)
        }
        lastOdom = scan.end; initialised = true; forceUpdates = 10
        summarise(scan.end)
        return true
    }

    /// Move by the odometry, and weigh by the scan when the car has moved enough.
    @discardableResult
    func update(_ scan: OdomScan) -> Bool {
        guard initialised else { return false }
        let odom = scan.end
        let last = lastOdom ?? odom
        let d = last.inverse() + odom
        lastOdom = odom
        let trans = hypot(d.x, d.y), rot = abs(d.yaw)
        if trans > 1e-4 || rot > 1e-4 {
            let sRot = p.alpha1 * rot + p.alpha2 * trans
            let sTrans = p.alpha3 * trans + p.alpha4 * rot
            for i in 0..<n {
                let dx = d.x + rnd.gaussian() * sTrans
                let dy = d.y + rnd.gaussian() * sTrans * 0.5
                let c = cos(t[i]), s = sin(t[i])
                x[i] += c * dx - s * dy; y[i] += s * dx + c * dy
                t[i] = wrapPi(t[i] + d.yaw + rnd.gaussian() * sRot)
            }
        }
        since.d += hypot(d.x, d.y); since.a += rot
        let moved = since.d >= p.updateMinD || since.a >= p.updateMinA
        if !moved && forceUpdates <= 0 { summarise(odom); return false }
        if forceUpdates > 0 { forceUpdates -= 1 }
        since = (0, 0)

        let (lx, ly) = scan.local()
        let idx = thin(lx.count, p.beams, scan.hit)
        if idx.isEmpty { summarise(odom); return false }
        var total = 0.0
        let zr = p.zRand / scan.maxRange
        let k = 1.0 / (2 * p.sigmaHit * p.sigmaHit)
        for i in 0..<n {
            let c = cos(t[i]), s = sin(t[i])
            var q = 1.0
            for j in idx {
                let dist = distAt(x[i] + c * lx[j] - s * ly[j], y[i] + s * lx[j] + c * ly[j])
                let pz = p.zHit * exp(-dist * dist * k) + zr
                q += pz * pz * pz
            }
            w[i] *= q
            total += w[i]
        }
        if total <= 0 || total.isNaN { for i in 0..<n { w[i] = 1.0 / Double(n) } } else { for i in 0..<n { w[i] /= total } }
        var sq = 0.0
        for i in 0..<n { sq += w[i] * w[i] }
        if 1 / sq < Double(n) / 2 { resample() }
        summarise(odom)
        var good = 0
        for j in idx {
            let pt = pose.apply(lx[j], ly[j])
            if distAt(pt.0, pt.1) < 2 * p.sigmaHit { good += 1 }
        }
        score = Double(good) / Double(idx.count)
        return true
    }

    func particles() -> ([Double], [Double]) { (x, y) }

    private func resample() {
        var nx = x, ny = y, nt = t
        let step = 1.0 / Double(n)
        let r = rnd.uniform() * step
        var c = w[0], i = 0
        for m in 0..<n {
            let u = r + Double(m) * step
            while u > c && i < n - 1 { i += 1; c += w[i] }
            nx[m] = x[i]; ny[m] = y[i]; nt[m] = t[i]
        }
        x = nx; y = ny; t = nt
        for m in 0..<n { w[m] = step }
    }

    private func summarise(_ odom: Pose2) {
        var mx = 0.0, my = 0.0, sc = 0.0, ss = 0.0
        for i in 0..<n { mx += w[i] * x[i]; my += w[i] * y[i]; sc += w[i] * cos(t[i]); ss += w[i] * sin(t[i]) }
        var vx = 0.0, vy = 0.0
        for i in 0..<n { vx += w[i] * (x[i] - mx) * (x[i] - mx); vy += w[i] * (y[i] - my) * (y[i] - my) }
        spreadXY = (vx + vy).squareRoot()
        spreadYaw = (-2 * log(min(max(hypot(sc, ss), 1e-9), 1.0))).squareRoot()
        pose = Pose2(x: mx, y: my, yaw: atan2(ss, sc))
        offset = pose + odom.inverse()
    }

    private func scoreAt(_ px: Double, _ py: Double, _ yaw: Double, _ lx: [Double], _ ly: [Double], _ idx: [Int], _ sigma: Double) -> Double {
        let c = cos(yaw), s = sin(yaw), k = 1.0 / (2 * sigma * sigma)
        var q = 0.0
        for j in idx {
            let d = distAt(px + c * lx[j] - s * ly[j], py + s * lx[j] + c * ly[j])
            q += exp(-d * d * k)
        }
        return q
    }

    private func thin(_ size: Int, _ want: Int, _ hit: [Bool]) -> [Int] {
        let hits = (0..<size).filter { hit[$0] }
        if hits.count <= want { return hits }
        return (0..<want).map { hits[$0 * hits.count / want] }
    }

    private func distAt(_ wx: Double, _ wy: Double) -> Double {
        let r = map.row(wx), c = map.col(wy)
        if !map.inside(r, c) { return p.maxDist }
        return Double(field[map.index(r, c)])
    }

    private func buildField() {
        let w = map.width, h = map.height
        for i in 0..<field.count { field[i] = map.occupied(i) ? 0 : 1e9 }
        let a = Float(map.res), b = Float(map.res * 1.4142)
        for r in 0..<h {
            for c in 0..<w {
                let i = r * w + c
                var v = field[i]
                if c > 0 { v = min(v, field[i - 1] + a) }
                if r > 0 {
                    v = min(v, field[i - w] + a)
                    if c > 0 { v = min(v, field[i - w - 1] + b) }
                    if c < w - 1 { v = min(v, field[i - w + 1] + b) }
                }
                field[i] = v
            }
        }
        for r in stride(from: h - 1, through: 0, by: -1) {
            for c in stride(from: w - 1, through: 0, by: -1) {
                let i = r * w + c
                var v = field[i]
                if c < w - 1 { v = min(v, field[i + 1] + a) }
                if r < h - 1 {
                    v = min(v, field[i + w] + a)
                    if c < w - 1 { v = min(v, field[i + w + 1] + b) }
                    if c > 0 { v = min(v, field[i + w - 1] + b) }
                }
                field[i] = v
            }
        }
        let cap = Float(p.maxDist)
        for i in 0..<field.count where field[i] > cap { field[i] = cap }
    }
}

/// Nav2's layered costmap (port of nav/Costmap.kt): static walls, what the LiDAR
/// sees now, and inflation 252·e^(-k(d - r_inscribed)) round both.
final class Costmap {
    struct Params {
        var inscribedRadius = 0.20
        var inflationRadius = 0.55
        var costScaling = 4.0
        var obstacleRange = 3.5
        var raytraceRange = 4.0
        var obstacleKeepMs: Int64 = 8000
        var unknownIsLethal = true
    }
    static let lethal = 254, inscribed = 253, unknown = 255

    let map: OccGrid
    let p: Params
    let width: Int, height: Int
    private var staticCost: [UInt8]
    private var obstacleMs: [Int64]
    private(set) var master: [UInt8]
    private var kernel = [(Int, Int, Int)]()
    private(set) var version = 0

    init(map: OccGrid, params: Params = Params()) {
        self.map = map; p = params; width = map.width; height = map.height
        staticCost = [UInt8](repeating: 0, count: map.width * map.height)
        obstacleMs = [Int64](repeating: 0, count: map.width * map.height)
        master = staticCost
        let cells = Int(ceil(params.inflationRadius / map.res))
        for dr in -cells...cells {
            for dc in -cells...cells {
                let c = costAtDistance(hypot(Double(dr) * map.res, Double(dc) * map.res))
                if c > 0 { kernel.append((dr, dc, c)) }
            }
        }
        for i in 0..<staticCost.count {
            staticCost[i] = map.occupied(i) ? UInt8(Costmap.lethal) : (!map.known(i) && p.unknownIsLethal) ? UInt8(Costmap.unknown) : 0
        }
        for i in 0..<staticCost.count where map.occupied(i) { stamp(&staticCost, i / width, i % width) }
        rebuild(0)
    }

    func costAtDistance(_ d: Double) -> Int {
        if d <= 1e-9 { return Costmap.lethal }
        if d <= p.inscribedRadius { return Costmap.inscribed }
        if d <= p.inflationRadius { return min(252, max(1, Int(252 * exp(-p.costScaling * (d - p.inscribedRadius))))) }
        return 0
    }

    func distanceForCost(_ c: Int) -> Double {
        if c >= Costmap.inscribed { return 0 }
        if c <= 0 { return p.inflationRadius }
        return p.inscribedRadius + log(252.0 / Double(c)) / p.costScaling
    }

    func cost(_ i: Int) -> Int { Int(master[i]) }
    func costAt(_ x: Double, _ y: Double) -> Int {
        let r = map.row(x), c = map.col(y)
        return map.inside(r, c) ? cost(map.index(r, c)) : (p.unknownIsLethal ? Costmap.unknown : 0)
    }
    func blocked(_ x: Double, _ y: Double) -> Bool { costAt(x, y) >= Costmap.inscribed }

    func addScan(_ s: OdomScan, nowMs: Int64) {
        for k in 0..<s.count {
            let dx = s.px[k] - s.ox[k], dy = s.py[k] - s.oy[k]
            let len = hypot(dx, dy)
            if len < 1e-6 { continue }
            let cl = min(len - map.res, p.raytraceRange)
            clear(s.ox[k], s.oy[k], s.ox[k] + dx / len * cl, s.oy[k] + dy / len * cl)
            if s.hit[k] && len <= p.obstacleRange {
                let r = map.row(s.px[k]), c = map.col(s.py[k])
                if map.inside(r, c) && !map.occupied(map.index(r, c)) { obstacleMs[map.index(r, c)] = nowMs }
            }
        }
        rebuild(nowMs)
    }

    func clearObstacles() {
        for i in 0..<obstacleMs.count { obstacleMs[i] = 0 }
        rebuild(0)
    }

    private func clear(_ x0: Double, _ y0: Double, _ x1: Double, _ y1: Double) {
        var r0 = map.row(x0), c0 = map.col(y0)
        let r1 = map.row(x1), c1 = map.col(y1)
        let dr = abs(r1 - r0), dc = abs(c1 - c0)
        let sr = r1 > r0 ? 1 : -1, sc = c1 > c0 ? 1 : -1
        var err = dc - dr
        while true {
            if !map.inside(r0, c0) { return }
            obstacleMs[map.index(r0, c0)] = 0
            if r0 == r1 && c0 == c1 { return }
            let e2 = 2 * err
            if e2 > -dr { err -= dr; c0 += sc }
            if e2 < dc { err += dc; r0 += sr }
        }
    }

    private func rebuild(_ nowMs: Int64) {
        master = staticCost
        for i in 0..<obstacleMs.count {
            let t = obstacleMs[i]
            if t == 0 { continue }
            if nowMs - t > p.obstacleKeepMs { obstacleMs[i] = 0; continue }
            master[i] = UInt8(Costmap.lethal)
            stamp(&master, i / width, i % width)
        }
        version += 1
    }

    private func stamp(_ layer: inout [UInt8], _ r: Int, _ c: Int) {
        for (dr, dc, cost) in kernel {
            let rr = r + dr, cc = c + dc
            if rr < 0 || rr >= height || cc < 0 || cc >= width { continue }
            let i = rr * width + cc
            let old = Int(layer[i])
            if old == Costmap.unknown && cost < Costmap.lethal { continue }
            if cost > old || old == Costmap.unknown { layer[i] = UInt8(cost) }
        }
    }
}

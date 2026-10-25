import Foundation

/// A binary min-heap of (key, index), for A*.
private struct MinHeap {
    private var keys = [Float](), vals = [Int]()
    var isEmpty: Bool { keys.isEmpty }
    mutating func push(_ k: Float, _ v: Int) {
        keys.append(k); vals.append(v)
        var i = keys.count - 1
        while i > 0 {
            let p = (i - 1) / 2
            if keys[p] <= keys[i] { break }
            keys.swapAt(p, i); vals.swapAt(p, i); i = p
        }
    }
    mutating func pop() -> Int {
        let top = vals[0]
        let lastK = keys.removeLast(), lastV = vals.removeLast()
        if !keys.isEmpty {
            keys[0] = lastK; vals[0] = lastV
            var i = 0
            while true {
                let l = 2 * i + 1, r = l + 1
                var m = i
                if l < keys.count && keys[l] < keys[m] { m = l }
                if r < keys.count && keys[r] < keys[m] { m = r }
                if m == i { break }
                keys.swapAt(m, i); vals.swapAt(m, i); i = m
            }
        }
        return top
    }
}

/// Cost-aware A* on the costmap, then a simple smoother (port of nav/Planner.kt).
final class Planner {
    let cm: Costmap
    var travelMult = 2.0
    var goalTolerance = 0.3
    var maxIterations = 400_000
    var spacing = 0.10
    private static let dr = [1, -1, 0, 0, 1, 1, -1, -1]
    private static let dc = [0, 0, 1, -1, 1, -1, 1, -1]
    private static let step: [Float] = [1, 1, 1, 1, 1.4142, 1.4142, 1.4142, 1.4142]

    init(_ cm: Costmap) { self.cm = cm }

    func plan(from start: Pose2, toX gx: Double, toY gy: Double) -> [(Double, Double)]? {
        let map = cm.map, w = map.width, h = map.height
        guard let s = nearestFree(map.row(start.x), map.col(start.y), 0.35),
              let g = nearestFree(map.row(gx), map.col(gy), goalTolerance) else { return nil }
        var gCost = [Float](repeating: .greatestFiniteMagnitude, count: w * h)
        var parent = [Int](repeating: -1, count: w * h)
        var closed = [Bool](repeating: false, count: w * h)
        var open = MinHeap()
        let gr = g / w, gc = g % w
        func heur(_ i: Int) -> Float { Float(hypot(Double(i / w - gr), Double(i % w - gc))) }
        gCost[s] = 0
        open.push(heur(s), s)
        var iters = 0
        while !open.isEmpty && iters < maxIterations {
            iters += 1
            let cur = open.pop()
            if closed[cur] { continue }
            if cur == g { return smooth(cells(parent, g)) }
            closed[cur] = true
            let cr = cur / w, cc = cur % w
            for k in 0..<8 {
                let nr = cr + Planner.dr[k], nc = cc + Planner.dc[k]
                if nr < 0 || nr >= h || nc < 0 || nc >= w { continue }
                let ni = nr * w + nc
                if closed[ni] { continue }
                let c = cm.cost(ni)
                if c >= Costmap.inscribed { continue }
                if k >= 4 && cm.cost(cr * w + nc) >= Costmap.inscribed && cm.cost(nr * w + cc) >= Costmap.inscribed { continue }
                let ng = gCost[cur] + Planner.step[k] * (1 + Float(travelMult * Double(c) / 252))
                if ng < gCost[ni] { gCost[ni] = ng; parent[ni] = cur; open.push(ng + heur(ni), ni) }
            }
        }
        return nil
    }

    func pathBlocked(_ path: [(Double, Double)], from: Int = 0) -> Bool {
        guard from < path.count else { return false }
        for k in from..<path.count where cm.blocked(path[k].0, path[k].1) { return true }
        return false
    }

    private func nearestFree(_ r: Int, _ c: Int, _ radius: Double) -> Int? {
        let map = cm.map
        let cells = Int(radius / map.res)
        var best: Int? = nil, bestD = Double.greatestFiniteMagnitude
        for dr in -cells...cells {
            for dc in -cells...cells {
                let rr = r + dr, cc = c + dc
                if !map.inside(rr, cc) { continue }
                let i = map.index(rr, cc)
                if cm.cost(i) >= Costmap.inscribed { continue }
                let d = hypot(Double(dr), Double(dc))
                if d <= Double(cells) + 1e-9 && d < bestD { bestD = d; best = i }
            }
        }
        return best
    }

    private func cells(_ parent: [Int], _ goal: Int) -> [(Double, Double)] {
        let map = cm.map
        var out = [(Double, Double)]()
        var i = goal
        while i >= 0 { out.append((map.cellX(i / map.width), map.cellY(i % map.width))); i = parent[i] }
        return out.reversed()
    }

    private func smooth(_ path: [(Double, Double)]) -> [(Double, Double)] {
        if path.count < 3 { return path }
        let orig = path
        var p = path
        for _ in 0..<60 {
            for k in 1..<(p.count - 1) {
                let old = p[k]
                var nx = p[k].0 + 0.2 * (orig[k].0 - p[k].0) + 0.3 * (p[k - 1].0 + p[k + 1].0 - 2 * p[k].0)
                var ny = p[k].1 + 0.2 * (orig[k].1 - p[k].1) + 0.3 * (p[k - 1].1 + p[k + 1].1 - 2 * p[k].1)
                if cm.blocked(nx, old.1) { nx = old.0 }
                if cm.blocked(nx, ny) { ny = old.1 }
                p[k] = (nx, ny)
            }
        }
        var out = [p[0]]
        var carry = 0.0
        for k in 1..<p.count {
            let a = p[k - 1], b = p[k]
            let seg = hypot(b.0 - a.0, b.1 - a.1)
            var at = spacing - carry
            while at <= seg {
                let f = at / seg
                out.append((a.0 + f * (b.0 - a.0), a.1 + f * (b.1 - a.1)))
                at += spacing
            }
            carry = max(0, seg - (at - spacing))
        }
        if let l = out.last, abs(l.0 - p.last!.0) + abs(l.1 - p.last!.1) > 1e-6 { out.append(p.last!) }
        return out
    }
}

/// Speed (m/s, negative reverse) and turn rate (rad/s, clockwise), Nav2's cmd_vel.
struct VelCmd {
    var v: Double, yawRate: Double, note: String
    static let stop = VelCmd(v: 0, yawRate: 0, note: "stop")
}

/// Regulated Pure Pursuit for a car (port of nav/Controller.kt).
final class Rpp {
    struct Params {
        var desiredVel = 0.4, lookaheadTime = 1.2, minLookahead = 0.35, maxLookahead = 0.9
        var minTurnRadius = 0.9, regMinRadius = 0.9, regMinSpeed = 0.2
        var costScalingDist = 0.6, costScalingGain = 1.0
        var approachDist = 0.6, minApproachVel = 0.12
        var maxAccel = 0.5, maxDecel = 1.0
        var timeToCollision = 1.5, goalTolerance = 0.25
        var reverseSpeed = 0.25, reverseTurnStart = 1.75, reverseTurnEnd = 0.9
        var dt = 0.1
    }
    let cm: Costmap
    var p: Params
    private var vLast = 0.0
    private var reversing = false
    private var reversedM = 0.0

    init(_ cm: Costmap, _ p: Params = Params()) { self.cm = cm; self.p = p }
    func reset() { vLast = 0; reversing = false; reversedM = 0 }

    func closest(_ pose: Pose2, _ path: [(Double, Double)]) -> Int {
        var best = 0, bd = Double.greatestFiniteMagnitude
        for (i, q) in path.enumerated() {
            let d = hypot(q.0 - pose.x, q.1 - pose.y)
            if d < bd { bd = d; best = i }
        }
        return best
    }

    func remaining(_ pose: Pose2, _ path: [(Double, Double)]) -> Double {
        let k = closest(pose, path)
        var d = hypot(path[k].0 - pose.x, path[k].1 - pose.y)
        if k + 1 < path.count { for i in (k + 1)..<path.count { d += hypot(path[i].0 - path[i - 1].0, path[i].1 - path[i - 1].1) } }
        return d
    }

    func step(_ pose: Pose2, _ path: [(Double, Double)], vNow: Double) -> VelCmd {
        guard let goal = path.last else { return .stop }
        let toGoal = remaining(pose, path)
        if hypot(goal.0 - pose.x, goal.1 - pose.y) < p.goalTolerance && toGoal < p.goalTolerance * 2 {
            vLast = 0; return VelCmd(v: 0, yawRate: 0, note: "arrived")
        }
        let ld = min(max(abs(vNow) * p.lookaheadTime, p.minLookahead), p.maxLookahead)
        let c0 = closest(pose, path)
        var carrot = goal
        var acc = 0.0
        if c0 + 1 < path.count {
            for i in (c0 + 1)..<path.count {
                acc += hypot(path[i].0 - path[i - 1].0, path[i].1 - path[i - 1].1)
                if acc >= ld { carrot = path[i]; break }
            }
        }
        let local = pose.toLocal(carrot.0, carrot.1)
        let bearing = atan2(local.1, local.0)

        if !reversing && abs(bearing) > p.reverseTurnStart && toGoal > p.goalTolerance * 2 { reversing = true; reversedM = 0 }
        if reversing {
            if abs(bearing) < p.reverseTurnEnd || reversedM > 1.5 { reversing = false }
            else {
                let v = -p.reverseSpeed
                let w = (bearing >= 0 ? 1.0 : -1.0) * p.reverseSpeed / p.minTurnRadius
                if arcCollides(pose, v, w) { reversing = false; vLast = 0; return VelCmd(v: 0, yawRate: 0, note: "blocked behind") }
                reversedM += p.reverseSpeed * p.dt
                vLast = v
                return VelCmd(v: v, yawRate: w, note: "3-point turn")
            }
        }

        let d2 = local.0 * local.0 + local.1 * local.1
        let kMax = 1 / p.minTurnRadius
        let kappa = min(max(d2 > 1e-6 ? 2 * local.1 / d2 : 0, -kMax), kMax)
        var v = p.desiredVel
        if abs(kappa) > 1e-6 { let r = 1 / abs(kappa); if r < p.regMinRadius { v *= r / p.regMinRadius } }
        let dObs = cm.distanceForCost(cm.costAt(pose.x, pose.y))
        if dObs < p.costScalingDist { v *= p.costScalingGain * dObs / p.costScalingDist }
        v = max(v, p.regMinSpeed)
        if toGoal < p.approachDist { v = max(p.minApproachVel, v * toGoal / p.approachDist) }
        v = min(v, p.desiredVel)
        v = min(max(v, vLast - p.maxDecel * p.dt), vLast + p.maxAccel * p.dt)
        if v < 0 { v = 0 }
        let vCheck = max(v, p.regMinSpeed)
        if arcCollides(pose, vCheck, vCheck * kappa) { vLast = 0; return VelCmd(v: 0, yawRate: 0, note: "obstacle ahead") }
        vLast = v
        return VelCmd(v: v, yawRate: v * kappa, note: "following")
    }

    func arcCollides(_ pose: Pose2, _ v: Double, _ w: Double) -> Bool {
        let steps = 15, dt = p.timeToCollision / Double(steps)
        var x = pose.x, y = pose.y, yaw = pose.yaw
        let start = cm.costAt(x, y)
        for _ in 0..<steps {
            x += v * cos(yaw) * dt; y += v * sin(yaw) * dt; yaw += w * dt
            let c = cm.costAt(x, y)
            if start < Costmap.inscribed { if c >= Costmap.inscribed { return true } }
            else if c > start { return true }
        }
        return false
    }
}

/// What the navigator needs from the car.
protocol NavDriver: AnyObject {
    func sendVelocity(_ v: Double, _ yawRate: Double)
    func setGuided()
    func setHold()
    var inGuided: Bool { get }
    var armed: Bool { get }
}

/// Nav2's bt_navigator + waypoint_follower as a state machine (port of nav/Navigator.kt).
final class Navigator {
    enum State: String { case idle, following, blocked, recovering, waiting, done, failed }
    struct Params {
        var replanMs: Int64 = 1000, progressDist = 0.5, progressMs: Int64 = 10_000, blockedReplanMs: Int64 = 1500
        var retries = 6, waitMs: Int64 = 5000, backupDist = 0.3, backupSpeed = 0.2, pauseAtWaypointMs: Int64 = 3000
        var staleMs: Int64 = 1000, lostGraceMs: Int64 = 2000
    }
    var p = Params()
    let planner: Planner
    let rpp: Rpp
    weak var drive: NavDriver?
    var onClearObstacles: (() -> Void)?

    private(set) var state = State.idle
    private(set) var status = "Idle"
    private(set) var path = [(Double, Double)]()
    private(set) var goals = [Waypoint]()
    private(set) var goalIndex = 0
    private(set) var loop = false
    private(set) var lastCmd = VelCmd.stop

    private var lastPlanMs: Int64 = 0, failures = 0, blockedSince: Int64 = 0
    private var progressPose: Pose2?, progressMs: Int64 = 0
    private var recoveryStep = 0, recoveryUntil: Int64 = 0, backupStart: Pose2?
    private var waitUntil: Int64 = 0, lostSince: Int64 = 0, guidedAsked: Int64 = 0

    init(planner: Planner, rpp: Rpp, drive: NavDriver) { self.planner = planner; self.rpp = rpp; self.drive = drive }

    var goal: Waypoint? { goalIndex < goals.count ? goals[goalIndex] : nil }
    var active: Bool { state == .following || state == .blocked || state == .recovering || state == .waiting }

    func start(_ list: [Waypoint], loop: Bool, nowMs: Int64) {
        if list.isEmpty { return }
        goals = list; self.loop = loop; goalIndex = 0
        beginGoal(nowMs)
        drive?.setGuided(); guidedAsked = nowMs
    }

    func stop(_ why: String = "Stopped") {
        let was = active
        state = .idle; status = why; path = []
        if was { drive?.sendVelocity(0, 0); drive?.setHold() }
        lastCmd = .stop
    }

    private func beginGoal(_ now: Int64) {
        state = .following; failures = 0; recoveryStep = 0
        path = []; lastPlanMs = 0; blockedSince = 0
        progressPose = nil; progressMs = now
        rpp.reset()
        status = "Going to \(goal?.name ?? "")"
    }

    private func fault(_ why: String) {
        state = .failed; status = why; path = []
        drive?.sendVelocity(0, 0); drive?.setHold(); lastCmd = .stop
    }

    func tick(nowMs now: Int64, pose: Pose2?, confident: Bool, poseAgeMs: Int64, scanAgeMs: Int64, vNow: Double) {
        guard active, let drive = drive else { return }
        guard let pose = pose, poseAgeMs <= p.staleMs else { fault("Stopped: no position from the car"); return }
        if scanAgeMs > p.staleMs { fault("Stopped: no LiDAR scan"); return }
        if !confident {
            if lostSince == 0 { lostSince = now }
            if now - lostSince > p.lostGraceMs { fault("Stopped: lost in the map (set the pose again)"); return }
            send(VelCmd(v: 0, yawRate: 0, note: "localising")); status = "Localising…"; return
        }
        lostSince = 0
        if !drive.armed { fault("Stopped: the car is disarmed"); return }
        if !drive.inGuided {
            if now - guidedAsked < 3000 { send(.stop); return }
            stop("Stopped: the mode was changed"); return
        }
        guard let g = goal else { stop("Done"); return }
        switch state {
        case .waiting:
            send(.stop)
            if now >= waitUntil { nextGoal(now) }
            return
        case .recovering:
            recover(now, pose); return
        default: break
        }
        let due = path.isEmpty || now - lastPlanMs >= p.replanMs || (state == .blocked && now - blockedSince >= p.blockedReplanMs)
        if due {
            lastPlanMs = now
            if let np = planner.plan(from: pose, toX: g.x, toY: g.y) {
                path = np
                if state == .blocked { state = .following; blockedSince = 0 }
            } else if path.isEmpty || planner.pathBlocked(path, from: rpp.closest(pose, path)) {
                startRecovery(now, "no path"); return
            }
        }
        let cmd = rpp.step(pose, path, vNow: vNow)
        switch cmd.note {
        case "arrived": send(.stop); arrived(now); return
        case "obstacle ahead", "blocked behind":
            if state != .blocked { state = .blocked; blockedSince = now; lastPlanMs = 0 }
            status = "Blocked: \(cmd.note), replanning"
        default:
            if state == .blocked { state = .following }
            status = "Going to \(g.name) (\(cmd.note))"
        }
        send(cmd)
        if let pp = progressPose, pose.distance(to: pp) < p.progressDist {
            if now - progressMs > p.progressMs { startRecovery(now, "no progress") }
        } else { progressPose = pose; progressMs = now }
    }

    private func arrived(_ now: Int64) {
        failures = 0
        if goalIndex == goals.count - 1 && !loop {
            state = .done; status = "Arrived at \(goal?.name ?? "")"; path = []; drive?.setHold(); return
        }
        state = .waiting; waitUntil = now + p.pauseAtWaypointMs
        status = "At \(goal?.name ?? ""), waiting"
    }

    private func nextGoal(_ now: Int64) {
        goalIndex += 1
        if goalIndex >= goals.count {
            if !loop { state = .done; status = "Done"; drive?.setHold(); return }
            goalIndex = 0
        }
        beginGoal(now)
    }

    private func startRecovery(_ now: Int64, _ why: String) {
        failures += 1
        if failures > p.retries {
            if goals.count > 1 { status = "Skipped \(goal?.name ?? ""): \(why)"; nextGoal(now); return }
            fault("Gave up: \(why)"); return
        }
        state = .recovering
        recoveryStep = (failures - 1) % 3
        recoveryUntil = now + (recoveryStep == 1 ? p.waitMs : 0)
        backupStart = nil
        status = "Recovery \(failures)/\(p.retries) (\(why)): " + ["clearing obstacles", "waiting", "backing up"][recoveryStep]
        if recoveryStep == 0 { onClearObstacles?() }
    }

    private func recover(_ now: Int64, _ pose: Pose2) {
        switch recoveryStep {
        case 0: endRecovery(now)
        case 1: send(.stop); if now >= recoveryUntil { endRecovery(now) }
        default:
            if backupStart == nil { backupStart = pose }
            let s = backupStart!
            if hypot(pose.x - s.x, pose.y - s.y) >= p.backupDist || rpp.arcCollides(pose, -p.backupSpeed, 0) { send(.stop); endRecovery(now) }
            else { send(VelCmd(v: -p.backupSpeed, yawRate: 0, note: "backing up")) }
        }
    }

    private func endRecovery(_ now: Int64) {
        state = .following; path = []; lastPlanMs = 0
        progressPose = nil; progressMs = now; rpp.reset()
    }

    private func send(_ c: VelCmd) { lastCmd = c; drive?.sendVelocity(c.v, c.yawRate) }
}

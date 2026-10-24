import Foundation

/// One LiDAR rotation as the LD14 driver sends it (OBSTACLE_DISTANCE): distances
/// in cm, bin 0 straight ahead, incrementDeg per bin clockwise. A bin of
/// maxCm + 1 is clear, 65535 unknown. A port of core/Lidar.kt.
struct LidarScan {
    let cm: [Int]
    let incrementDeg: Float
    let offsetDeg: Float
    let minCm: Int
    let maxCm: Int
    let timeMs: Int64

    /// Valid returns as (angle_deg, range_m), angle in the hoverboard_laser
    /// convention: robot frame, wrapped to (-180, 180], 0 straight ahead,
    /// positive to the LEFT. Returns closer than minRange are dropped (the car itself).
    func points(_ minRange: Float) -> (a: [Float], r: [Float]) {
        var a = [Float](), r = [Float]()
        for (i, d) in cm.enumerated() {
            if d == 0xffff || d > maxCm || d < minCm { continue }
            let m = Float(d) / 100
            if m < minRange { continue }
            let cw = offsetDeg + (Float(i) + 0.5) * incrementDeg          // bin centre, clockwise
            a.append(LidarScan.wrap180(-cw)); r.append(m)
        }
        return (a, r)
    }

    /// Closest return within halfAngle of centreDeg (left-positive), or nil.
    func nearest(_ centreDeg: Float, _ halfAngle: Float, _ minRange: Float) -> (angle: Float, range: Float)? {
        let (a, r) = points(minRange)
        var best = -1
        for i in a.indices where abs(LidarScan.wrap180(a[i] - centreDeg)) <= halfAngle && (best < 0 || r[i] < r[best]) { best = i }
        return best < 0 ? nil : (a[best], r[best])
    }

    static func wrap180(_ d: Float) -> Float {
        var x = (d + 180).truncatingRemainder(dividingBy: 360)
        if x < 0 { x += 360 }
        return x - 180
    }
}

/// What a behaviour wants: throttle and steer in -1..1 as MANUAL_CONTROL sticks
/// (steer positive = right), why for the screen, alarm for the warning sound.
struct LidarCommand {
    var throttle: Float
    var steer: Float
    var why: String
    var alarm = false
    static let STOP = LidarCommand(throttle: 0, steer: 0, why: "stopped")
}

/// Yahboom's PID (per-sample, no dt) with an output clamp, as in hoverboard_laser/common.py.
final class SinglePid {
    var kp: Float, ki: Float, kd: Float, limit: Float
    private var integral: Float = 0
    private var prevError: Float = 0
    init(_ kp: Float, _ ki: Float, _ kd: Float, limit: Float = .greatestFiniteMagnitude) {
        self.kp = kp; self.ki = ki; self.kd = kd; self.limit = limit
    }
    func compute(_ target: Float, _ current: Float) -> Float {
        let error = target - current
        integral += error
        let derivative = error - prevError
        prevError = error
        return max(-limit, min(limit, kp * error + ki * integral + kd * derivative))
    }
    func reset() { integral = 0; prevError = 0 }
}

/// hoverboard_laser's three behaviours (laser_Avoidance, laser_Tracker,
/// laser_Warning) on the LD14 scan, driving the rover with MANUAL_CONTROL.
/// With skidSteer false, a turn on the spot becomes reversing with the wheels the
/// other way (the nose swings towards the turn), and only while the rear is clear.
final class LidarBehaviours {
    enum Mode: String, CaseIterable, Identifiable {
        case off = "Off", avoid = "Avoid", follow = "Follow", warn = "Warn"
        var id: String { rawValue }
    }

    /// hoverboard_laser's parameters, scaled for an RC car (tested in SITL).
    struct Params {
        // laser_Avoidance
        var linear: Float = 0.15
        var angular: Float = 0.8
        var laserAngle: Float = 45
        var responseDist: Float = 1.0
        var minPoints = 1
        var minRange: Float = 0.30
        // laser_Tracker
        var priorityAngle: Float = 10
        var trackAngle: Float = 30
        var followDist: Float = 1.0
        var maxLinear: Float = 0.15
        var maxAngular: Float = 0.5
        // laser_Warning
        var warnAngle: Float = 10
        var warnDist: Float = 0.5
        // vehicle
        var skidSteer = false
    }

    var p = Params()
    var mode = Mode.off {
        didSet { if mode != oldValue { reset() } }
    }

    private var held = LidarCommand.STOP
    private var holdUntilMs: Int64 = 0
    private let linPid = SinglePid(2, 0, 2)
    private let angPid = SinglePid(3, 0, 5)

    func reset() { held = .STOP; holdUntilMs = 0; linPid.reset(); angPid.reset() }

    func update(_ scan: LidarScan, _ nowMs: Int64) -> LidarCommand {
        switch mode {
        case .off: return .STOP
        case .avoid: return avoid(scan, nowMs)
        case .follow: return follow(scan)
        case .warn: return warn(scan)
        }
    }

    private func blocked(_ a: [Float], _ r: [Float], _ near: Float, _ test: (Float) -> Bool) -> Bool {
        var n = 0
        for i in a.indices where r[i] < near && test(a[i]) { n += 1 }
        return n >= p.minPoints
    }

    private func rearClear(_ scan: LidarScan) -> Bool {
        (scan.nearest(180, 30, p.minRange)?.range ?? .greatestFiniteMagnitude) > p.responseDist
    }

    /// A turn on the spot to the left (w > 0) or right: pivot when skid steering, else back up.
    private func spin(_ scan: LidarScan, _ w: Float, _ why: String) -> LidarCommand {
        let steer = -max(-1, min(1, w))                  // w left-positive, steer right-positive
        if p.skidSteer { return LidarCommand(throttle: 0, steer: steer, why: why) }
        if !rearClear(scan) { return LidarCommand(throttle: 0, steer: 0, why: "\(why), but the rear is blocked: stopped") }
        // Reversing with the wheels turned right swings the nose left, so steer the other way.
        return LidarCommand(throttle: -p.linear, steer: -steer, why: "\(why) (reversing)")
    }

    private func avoid(_ scan: LidarScan, _ nowMs: Int64) -> LidarCommand {
        // Something close straight ahead overrides a held manoeuvre that is still going forward.
        let ahead = scan.nearest(0, 30, p.minRange)?.range ?? .greatestFiniteMagnitude
        if ahead < p.responseDist * 0.6 && (held.throttle > 0 || nowMs >= holdUntilMs) {
            let left = scan.nearest(60, 30, p.minRange)?.range ?? .greatestFiniteMagnitude
            let right = scan.nearest(-60, 30, p.minRange)?.range ?? .greatestFiniteMagnitude
            let w = left >= right ? p.angular : -p.angular
            held = spin(scan, w, String(format: "too close ahead (%.2f m): back off", ahead))
            holdUntilMs = nowMs + (p.skidSteer ? 400 : 800)
            return held
        }
        if nowMs < holdUntilMs { return held }
        let (a, r) = scan.points(p.minRange)
        let near = p.responseDist * 1.5
        let la = p.laserAngle
        let front = blocked(a, r, near) { abs($0) <= 20 }
        let left = blocked(a, r, near) { $0 > 20 && $0 < la }
        let right = blocked(a, r, near) { $0 < -20 && $0 > -la }
        let v = p.linear, w = p.angular
        let slow = p.skidSteer ? v * 0.5 : v

        // laser_Avoidance's decision table and hold times.
        let cmd: LidarCommand, hold: Float
        if front && left && right { cmd = spin(scan, -w, "boxed in: turn right"); hold = 0.4 }
        else if front && right { cmd = turn(scan, slow, w, "front+right blocked: turn left"); hold = 0.2 }
        else if front && left { cmd = turn(scan, slow, -w, "front+left blocked: turn right"); hold = 0.2 }
        else if front { cmd = spin(scan, w, "front blocked: turn left"); hold = 0.2 }
        else if left && right { cmd = LidarCommand(throttle: slow, steer: w, why: "narrow gap: turn right"); hold = 0.4 }
        else if left { cmd = LidarCommand(throttle: v, steer: w, why: "left blocked: veer right"); hold = 0.2 }
        else if right { cmd = LidarCommand(throttle: v, steer: -w, why: "right blocked: veer left"); hold = 0.2 }
        else { cmd = LidarCommand(throttle: v, steer: 0, why: "clear: forward"); hold = 0 }
        // A car needs longer than a hoverboard to swing its nose round.
        let scale: Float = p.skidSteer ? 1 : 2
        held = cmd
        holdUntilMs = nowMs + Int64(hold * scale * 1000)
        return cmd
    }

    /// Front blocked with one side open: creep forward while turning if there is room, else back up.
    private func turn(_ scan: LidarScan, _ v: Float, _ w: Float, _ why: String) -> LidarCommand {
        let ahead = scan.nearest(0, 20, p.minRange)?.range ?? .greatestFiniteMagnitude
        if p.skidSteer || ahead > p.responseDist { return LidarCommand(throttle: v, steer: -w, why: why) }
        return spin(scan, w, why)
    }

    private func follow(_ scan: LidarScan) -> LidarCommand {
        let (a, r) = scan.points(p.minRange)
        let dist = p.followDist
        var pick = -1
        for i in a.indices where abs(a[i]) < p.priorityAngle && r[i] < dist + 0.5 && (pick < 0 || r[i] < r[pick]) { pick = i }
        if pick < 0 { for i in a.indices where abs(a[i]) < p.trackAngle && (pick < 0 || r[i] < r[pick]) { pick = i } }
        if pick < 0 { linPid.reset(); angPid.reset(); return LidarCommand(throttle: 0, steer: 0, why: "nothing to follow") }
        var minDist = r[pick]; let minAng = a[pick]
        if abs(minDist - dist) < 0.1 { minDist = dist }
        let side = minAng > 0 ? "left" : minAng < 0 ? "right" : "ahead"
        let whereText = String(format: "%.2f m at %+.0f° (%@)", r[pick], minAng, side)
        linPid.limit = p.maxLinear; angPid.limit = p.maxAngular
        var w = angPid.compute(minAng / 48, 0)            // > 0 when the object is to the left
        if abs(w) < 0.1 { w = 0 }
        if p.skidSteer {
            let v = -linPid.compute(dist, minDist)
            return LidarCommand(throttle: v, steer: max(-1, min(1, -w)), why: "following \(whereText)")
        }
        // Slow down over the last metre, and back up to swing the nose round onto a close target to the side.
        if abs(minAng) > 20 && minDist < dist + 1 {
            var c = spin(scan, minAng > 0 ? p.maxAngular : -p.maxAngular, "following \(whereText): swing towards it")
            c.throttle = max(-p.maxLinear, min(p.maxLinear, c.throttle))
            return c
        }
        var v = max(-p.maxLinear, min(p.maxLinear, p.maxLinear * (minDist - dist)))
        if v < 0 && !rearClear(scan) { v = 0 }
        let turn = abs(minAng) < 4 ? 0 : max(-p.maxAngular, min(p.maxAngular, p.maxAngular * minAng / p.trackAngle))
        // Reversing a car turns the nose the other way.
        let steer = v < 0 ? turn : -turn
        return LidarCommand(throttle: v, steer: max(-1, min(1, steer)), why: "following \(whereText)")
    }

    private func warn(_ scan: LidarScan) -> LidarCommand {
        guard let n = scan.nearest(0, p.warnAngle, p.minRange) else { return LidarCommand(throttle: 0, steer: 0, why: "nothing ahead") }
        let close = n.range <= p.warnDist
        return LidarCommand(throttle: 0, steer: 0,
                            why: String(format: "nearest ahead %.2f m at %+.0f°", n.range, n.angle) + (close ? ": TOO CLOSE" : ""),
                            alarm: close)
    }
}

/// Runs a behaviour against the vehicle's latest scan, ten times a second.
/// Avoid and Follow drive with MANUAL_CONTROL (Manual, Acro and Steering); Warn
/// only watches. Stops the car when the scan goes stale.
final class LidarDriver {
    private let vehicle: Vehicle
    let behaviours: LidarBehaviours
    private(set) var running = false
    private(set) var last = LidarCommand.STOP
    private var lastScanMs: Int64 = 0
    private var zeros = 0
    /// True once this driver has sent a non-zero stick: ArduPilot holds an override for RC_OVERRIDE_TIME.
    private var drove = false

    init(vehicle: Vehicle, behaviours: LidarBehaviours) { self.vehicle = vehicle; self.behaviours = behaviours }

    var drives: Bool { behaviours.mode == .avoid || behaviours.mode == .follow }

    /// (Re)starts with the current mode, first stopping anything an earlier mode left moving.
    func start() { halt(); behaviours.reset(); last = .STOP; lastScanMs = 0; running = true }

    func stop() { halt(); running = false; last = .STOP }

    /// Zero sticks now, and twice more on the next ticks in case one is lost.
    private func halt() {
        if !drove { return }
        drove = false
        vehicle.manualControl(steer: 0, thr: 0)
        zeros = 2
    }

    /// One step. paused (the user's stick is held) lets the user drive instead;
    /// limit caps both sticks (the Drive power setting).
    func tick(_ nowMs: Int64, paused: Bool, limit: Float) -> LidarCommand {
        if zeros > 0 { zeros -= 1; vehicle.manualControl(steer: 0, thr: 0) }
        if !running { return .STOP }
        guard let scan = vehicle.scan, nowMs - scan.timeMs <= 1000 else {
            // Zero sticks with retries: a single lost zero would leave the last command driving the car.
            halt()
            last = LidarCommand(throttle: 0, steer: 0, why: vehicle.scan == nil ? "waiting for a LiDAR scan" : "LiDAR scan is old: stopped")
            return last
        }
        if paused {
            drove = false                     // the user's stick has taken over the override
            behaviours.reset()
            last = LidarCommand(throttle: 0, steer: 0, why: "paused: stick in use")
            return last
        }
        // Follow's PID steps once per scan; Avoid (hold timer) and Warn every tick.
        if scan.timeMs != lastScanMs || behaviours.mode != .follow {
            lastScanMs = scan.timeMs
            last = behaviours.update(scan, nowMs)
        }
        if drives {
            vehicle.manualControl(steer: max(-limit, min(limit, last.steer)), thr: max(-limit, min(limit, last.throttle)))
            if last.throttle != 0 || last.steer != 0 { drove = true }
        }
        return last
    }
}

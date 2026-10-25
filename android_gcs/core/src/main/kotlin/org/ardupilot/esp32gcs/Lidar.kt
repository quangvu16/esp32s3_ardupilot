package org.ardupilot.esp32gcs

import kotlin.math.abs

/**
 * One LiDAR rotation as the LD14 driver sends it (OBSTACLE_DISTANCE): distances
 * in cm, bin 0 straight ahead, [incrementDeg] per bin clockwise. A bin of
 * maxCm + 1 is clear, 65535 unknown.
 */
class LidarScan(val cm: IntArray, val incrementDeg: Float, val offsetDeg: Float,
                val minCm: Int, val maxCm: Int, val timeMs: Long, val vehicleTimeUs: Long = 0L) {
    /**
     * Valid returns as (angle_deg, range_m), angle in the hoverboard_laser
     * convention: robot frame, wrapped to (-180, 180], 0 straight ahead,
     * positive to the LEFT. Returns closer than [minRange] are dropped
     * (the car's own body).
     */
    fun points(minRange: Float): Pair<FloatArray, FloatArray> {
        val a = ArrayList<Float>(cm.size); val r = ArrayList<Float>(cm.size)
        for (i in cm.indices) {
            val d = cm[i]
            if (d == 0xffff || d > maxCm || d < minCm) continue
            val m = d / 100f
            if (m < minRange) continue
            val cw = offsetDeg + (i + 0.5f) * incrementDeg          // bin centre, clockwise
            a += wrap180(-cw); r += m
        }
        return Pair(a.toFloatArray(), r.toFloatArray())
    }

    /** Closest return within [halfAngle] of [centreDeg] (left-positive), or null. */
    fun nearest(centreDeg: Float, halfAngle: Float, minRange: Float): Pair<Float, Float>? {
        val (a, r) = points(minRange)
        var best = -1
        for (i in a.indices) if (abs(wrap180(a[i] - centreDeg)) <= halfAngle && (best < 0 || r[i] < r[best])) best = i
        return if (best < 0) null else Pair(a[best], r[best])
    }

    companion object {
        fun wrap180(d: Float): Float { var x = (d + 180f) % 360f; if (x < 0) x += 360f; return x - 180f }
    }
}

/**
 * What a behaviour wants: [throttle] and [steer] in -1..1 as MANUAL_CONTROL
 * sticks (steer positive = right), [why] for the screen, [alarm] for the
 * warning sound.
 */
data class LidarCommand(val throttle: Float, val steer: Float, val why: String, val alarm: Boolean = false) {
    companion object { val STOP = LidarCommand(0f, 0f, "stopped") }
}

/** Yahboom's PID (per-sample, no dt) with an output clamp, as in hoverboard_laser/common.py. */
class SinglePid(var kp: Float, var ki: Float, var kd: Float, var limit: Float = Float.MAX_VALUE) {
    private var integral = 0f
    private var prevError = 0f
    fun compute(target: Float, current: Float): Float {
        val error = target - current
        integral += error
        val derivative = error - prevError
        prevError = error
        return (kp * error + ki * integral + kd * derivative).coerceIn(-limit, limit)
    }
    fun reset() { integral = 0f; prevError = 0f }
}

/**
 * hoverboard_laser's three behaviours (laser_Avoidance, laser_Tracker,
 * laser_Warning) on the LD14 scan from ArduPilot, driving the rover with
 * MANUAL_CONTROL instead of /cmd_vel.
 *
 * The hoverboard turns on the spot; an RC car can't. With [skidSteer] false,
 * a turn on the spot becomes reversing with the wheels the other way (the nose
 * swings towards the turn), and only while the rear is clear.
 */
class LidarBehaviours {
    enum class Mode(val label: String) { OFF("Off"), AVOID("Avoid"), FOLLOW("Follow"), WARN("Warn") }

    /**
     * hoverboard_laser's parameters. Speeds are stick fractions here, and the
     * distances are larger: an RC car is much faster than the hoverboard's
     * 0.2 m/s and can't stop or turn on the spot. Tested in SITL with these.
     */
    class Params {
        // laser_Avoidance
        var linear = 0.15f            // forward stick when clear (hoverboard: 0.2 m/s)
        var angular = 0.8f            // steering stick when turning (hoverboard: 0.8 rad/s)
        var laserAngle = 45f          // left/right sectors reach this far out
        var responseDist = 1.0f       // m; a sector is blocked inside responseDist * 1.5 (hoverboard: 0.55)
        var minPoints = 1             // 5-degree bins needed to call a sector blocked (hoverboard: 5 raw points)
        var minRange = 0.30f          // m; closer returns are the car itself
        // laser_Tracker
        var priorityAngle = 10f
        var trackAngle = 30f
        var followDist = 1.0f         // m kept from the target (hoverboard: ResponseDist 0.55)
        var maxLinear = 0.15f         // hoverboard: 0.25 m/s
        var maxAngular = 0.5f         // full steering at the edge of the search angle (hoverboard: 0.8 rad/s)
        // laser_Warning
        var warnAngle = 10f
        var warnDist = 0.5f
        // vehicle
        var skidSteer = false
    }

    val p = Params()
    var mode = Mode.OFF
        set(v) { if (v != field) { field = v; reset() } }

    private var held = LidarCommand.STOP
    private var holdUntilMs = 0L
    private val linPid = SinglePid(2f, 0f, 2f)
    private val angPid = SinglePid(3f, 0f, 5f)

    fun reset() { held = LidarCommand.STOP; holdUntilMs = 0L; linPid.reset(); angPid.reset() }

    /** The command for this scan; [nowMs] drives the hold timer of Avoid. */
    fun update(scan: LidarScan, nowMs: Long): LidarCommand = when (mode) {
        Mode.OFF -> LidarCommand.STOP
        Mode.AVOID -> avoid(scan, nowMs)
        Mode.FOLLOW -> follow(scan)
        Mode.WARN -> warn(scan)
    }

    private fun blocked(a: FloatArray, r: FloatArray, near: Float, test: (Float) -> Boolean): Boolean {
        var n = 0
        for (i in a.indices) if (r[i] < near && test(a[i])) n++
        return n >= p.minPoints
    }

    private fun rearClear(scan: LidarScan) =
        (scan.nearest(180f, 30f, p.minRange)?.second ?: Float.MAX_VALUE) > p.responseDist

    /** A turn on the spot to the left (w > 0) or right: pivot when skid steering, else back up. */
    private fun spin(scan: LidarScan, w: Float, why: String): LidarCommand {
        val steer = -w.coerceIn(-1f, 1f)                  // w left-positive, steer right-positive
        if (p.skidSteer) return LidarCommand(0f, steer, why)
        if (!rearClear(scan)) return LidarCommand(0f, 0f, "$why, but the rear is blocked: stopped")
        // Reversing with the wheels turned right swings the nose left, so steer the other way.
        return LidarCommand(-p.linear, -steer, "$why (reversing)")
    }

    private fun avoid(scan: LidarScan, nowMs: Long): LidarCommand {
        // Not in laser_Avoidance (0.2 m/s): a car covers the hold time's distance quickly, so
        // something close straight ahead overrides a held manoeuvre that is still going forward.
        val ahead = scan.nearest(0f, 30f, p.minRange)?.second ?: Float.MAX_VALUE
        if (ahead < p.responseDist * 0.6f && (held.throttle > 0f || nowMs >= holdUntilMs)) {
            val left = scan.nearest(60f, 30f, p.minRange)?.second ?: Float.MAX_VALUE
            val right = scan.nearest(-60f, 30f, p.minRange)?.second ?: Float.MAX_VALUE
            val w = if (left >= right) p.angular else -p.angular
            held = spin(scan, w, "too close ahead (%.2f m): back off".format(ahead))
            holdUntilMs = nowMs + if (p.skidSteer) 400L else 800L
            return held
        }
        if (nowMs < holdUntilMs) return held
        val (a, r) = scan.points(p.minRange)
        val near = p.responseDist * 1.5f
        val la = p.laserAngle
        val front = blocked(a, r, near) { abs(it) <= 20f }
        val left = blocked(a, r, near) { it > 20f && it < la }
        val right = blocked(a, r, near) { it < -20f && it > -la }
        val v = p.linear; val w = p.angular
        // The hoverboard creeps at half speed while turning; half the stick stalls a car
        // (throttle dead zone), and a car must roll to turn at all.
        val slow = if (p.skidSteer) v * 0.5f else v

        // laser_Avoidance's decision table and hold times.
        val (cmd, hold) = when {
            front && left && right -> spin(scan, -w, "boxed in: turn right") to 0.4f
            front && right -> turn(scan, slow, w, "front+right blocked: turn left") to 0.2f
            front && left -> turn(scan, slow, -w, "front+left blocked: turn right") to 0.2f
            front -> spin(scan, w, "front blocked: turn left") to 0.2f
            left && right -> LidarCommand(slow, w, "narrow gap: turn right") to 0.4f
            left -> LidarCommand(v, w, "left blocked: veer right") to 0.2f
            right -> LidarCommand(v, -w, "right blocked: veer left") to 0.2f
            else -> LidarCommand(v, 0f, "clear: forward") to 0f
        }
        // A car needs longer than a hoverboard to swing its nose round.
        val scale = if (p.skidSteer) 1f else 2f
        held = cmd
        holdUntilMs = nowMs + (hold * scale * 1000).toLong()
        return cmd
    }

    /**
     * Front blocked with one side open: the hoverboard creeps forward while
     * turning. A car only does that if there is room ahead; otherwise it backs up.
     */
    private fun turn(scan: LidarScan, v: Float, w: Float, why: String): LidarCommand {
        val ahead = scan.nearest(0f, 20f, p.minRange)?.second ?: Float.MAX_VALUE
        if (p.skidSteer || ahead > p.responseDist) return LidarCommand(v, -w, why)
        return spin(scan, w, why)
    }

    private fun follow(scan: LidarScan): LidarCommand {
        val (a, r) = scan.points(p.minRange)
        val dist = p.followDist
        var pick = -1
        for (i in a.indices) if (abs(a[i]) < p.priorityAngle && r[i] < dist + 0.5f && (pick < 0 || r[i] < r[pick])) pick = i
        if (pick < 0) for (i in a.indices) if (abs(a[i]) < p.trackAngle && (pick < 0 || r[i] < r[pick])) pick = i
        if (pick < 0) { linPid.reset(); angPid.reset(); return LidarCommand(0f, 0f, "nothing to follow") }
        var minDist = r[pick]; val minAng = a[pick]
        if (abs(minDist - dist) < 0.1f) minDist = dist
        val where = "%.2f m at %+.0f° (%s)".format(r[pick], minAng, if (minAng > 0) "left" else if (minAng < 0) "right" else "ahead")
        linPid.limit = p.maxLinear; angPid.limit = p.maxAngular
        var w = angPid.compute(minAng / 48f, 0f)            // > 0 when the object is to the left
        if (abs(w) < 0.1f) w = 0f
        if (p.skidSteer) {
            // laser_Tracker as it is: the hoverboard stops and turns on the spot.
            val v = -linPid.compute(dist, minDist)
            return LidarCommand(v, (-w).coerceIn(-1f, 1f), "following $where")
        }
        // A car arrives too fast with laser_Tracker's PID (full stick 8 cm out) and can't turn
        // onto a close target to the side: slow down over the last metre, and back up to
        // swing the nose round when the target is close and off to one side.
        if (abs(minAng) > 20f && minDist < dist + 1f) {
            val c = spin(scan, if (minAng > 0) p.maxAngular else -p.maxAngular, "following $where: swing towards it")
            return c.copy(throttle = c.throttle.coerceIn(-p.maxLinear, p.maxLinear))
        }
        var v = (p.maxLinear * (minDist - dist)).coerceIn(-p.maxLinear, p.maxLinear)
        if (v < 0f && !rearClear(scan)) v = 0f
        // Steering in proportion to the angle: laser_Tracker's PID saturates at a few degrees,
        // and in Acro full stick is ACRO_TURN_RATE, so the car swung past the target between scans.
        val turn = if (abs(minAng) < 4f) 0f else (p.maxAngular * minAng / p.trackAngle).coerceIn(-p.maxAngular, p.maxAngular)
        // Reversing a car turns the nose the other way.
        val steer = if (v < 0f) turn else -turn
        return LidarCommand(v, steer.coerceIn(-1f, 1f), "following $where")
    }

    private fun warn(scan: LidarScan): LidarCommand {
        val n = scan.nearest(0f, p.warnAngle, p.minRange) ?: return LidarCommand(0f, 0f, "nothing ahead", false)
        val close = n.second <= p.warnDist
        return LidarCommand(0f, 0f, "nearest ahead %.2f m at %+.0f°".format(n.second, n.first) + if (close) ": TOO CLOSE" else "", close)
    }
}

/**
 * Runs a behaviour against the vehicle's latest scan, ten times a second.
 * Avoid and Follow drive with MANUAL_CONTROL (works in Manual, Acro and
 * Steering); Warn only watches. Stops the car when the scan goes stale.
 */
class LidarDriver(private val vehicle: Vehicle, val behaviours: LidarBehaviours) {
    @Volatile var running = false
        private set
    var last: LidarCommand = LidarCommand.STOP
        private set
    private var lastScanMs = 0L
    private var zeros = 0
    /** True once this driver has sent a non-zero stick: ArduPilot holds an override for RC_OVERRIDE_TIME. */
    private var drove = false

    val drives get() = behaviours.mode == LidarBehaviours.Mode.AVOID || behaviours.mode == LidarBehaviours.Mode.FOLLOW

    /** (Re)starts with the current mode, first stopping anything an earlier mode left moving. */
    fun start() { halt(); behaviours.reset(); last = LidarCommand.STOP; lastScanMs = 0L; running = true }

    fun stop() {
        halt()
        running = false
        last = LidarCommand.STOP
    }

    /** Zero sticks now, and twice more on the next ticks in case one is lost. */
    private fun halt() {
        if (!drove) return
        drove = false
        vehicle.manualControl(0f, 0f)
        zeros = 2
    }

    /**
     * One step. [paused] (the phone stick is held) lets the user drive instead,
     * like hoverboard_laser's /JoyState. [limit] caps both sticks (the Data
     * tab's power setting).
     */
    fun tick(nowMs: Long, paused: Boolean, limit: Float): LidarCommand {
        if (zeros > 0) { zeros--; vehicle.manualControl(0f, 0f) }
        if (!running) return LidarCommand.STOP
        val scan = vehicle.scan
        if (scan == null || nowMs - scan.timeMs > 1000) {
            // Zero sticks with the retries of halt(): a single lost zero would leave the
            // last command driving the car until RC_OVERRIDE_TIME runs out.
            halt()
            last = LidarCommand(0f, 0f, if (scan == null) "waiting for a LiDAR scan" else "LiDAR scan is old: stopped")
            return last
        }
        if (paused) {
            drove = false                     // the phone stick has taken over the override
            behaviours.reset()
            last = LidarCommand(0f, 0f, "paused: phone stick in use")
            return last
        }
        // Follow's PID steps once per scan, like laser_Tracker's scan callback. Avoid (hold timer)
        // and Warn are re-evaluated every tick, so a changed setting shows at once.
        if (scan.timeMs != lastScanMs || behaviours.mode != LidarBehaviours.Mode.FOLLOW) {
            lastScanMs = scan.timeMs
            last = behaviours.update(scan, nowMs)
        }
        if (drives) {
            vehicle.manualControl(last.steer.coerceIn(-limit, limit), last.throttle.coerceIn(-limit, limit))
            if (last.throttle != 0f || last.steer != 0f) drove = true
        }
        return last
    }
}

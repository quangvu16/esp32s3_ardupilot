package org.ardupilot.esp32gcs.nav

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin

/** Speed (m/s, negative = reverse) and turn rate (rad/s, clockwise positive), Nav2's cmd_vel. */
data class VelCmd(val v: Double, val yawRate: Double, val note: String = "") {
    companion object { val STOP = VelCmd(0.0, 0.0, "stop") }
}

/**
 * Nav2's Regulated Pure Pursuit controller for a car (Ackermann): steer for a
 * point [lookahead] ahead on the path, slow down on tight curves, near obstacles
 * and when arriving, and check the arc ahead for collisions. A car can't turn on
 * the spot (Nav2's use_rotate_to_heading is off); when the path starts behind it,
 * it reverses with the wheels turned to swing the nose round (a 3-point turn).
 */
class Rpp(private val cm: Costmap, val p: Params = Params()) {
    class Params {
        var desiredVel = 0.4                // m/s (Nav2 0.5)
        var lookaheadTime = 1.2             // s (Nav2 1.5)
        var minLookahead = 0.35             // m (Nav2 0.3)
        var maxLookahead = 0.9              // m (Nav2 0.9)
        var minTurnRadius = 0.9             // m: the car's tightest turn (Rover TURN_RADIUS)
        var regMinRadius = 0.9              // regulated_linear_scaling_min_radius (Nav2 0.9)
        var regMinSpeed = 0.2               // regulated_linear_scaling_min_speed (Nav2 0.25)
        var costScalingDist = 0.6           // cost_scaling_dist (Nav2 0.6)
        var costScalingGain = 1.0           // cost_scaling_gain (Nav2 1.0)
        var approachDist = 0.6              // approach_velocity_scaling_dist
        var minApproachVel = 0.12           // m/s; an RC car's ESC needs a little (Nav2 0.05)
        var maxAccel = 0.5                  // m/s² (velocity smoother)
        var maxDecel = 1.0
        var timeToCollision = 1.5           // s ahead checked along the arc (Nav2 1.0)
        var goalTolerance = 0.25            // m (xy_goal_tolerance)
        var reverseSpeed = 0.25             // m/s for the 3-point turn
        var reverseTurnStart = 1.75         // rad: carrot further round than this -> reverse
        var reverseTurnEnd = 0.9            // rad: until it is within this
        var dt = 0.1                        // s between calls
    }

    private var vLast = 0.0
    private var reversing = false
    private var reversedM = 0.0

    fun reset() { vLast = 0.0; reversing = false; reversedM = 0.0 }

    /** Distance left along [path] from the closest point to [pose]. */
    fun remaining(pose: Pose2, path: List<DoubleArray>): Double {
        val k = closest(pose, path)
        var d = hypot(path[k][0] - pose.x, path[k][1] - pose.y)
        for (i in k + 1 until path.size) d += hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        return d
    }

    fun closest(pose: Pose2, path: List<DoubleArray>): Int {
        var best = 0; var bd = Double.MAX_VALUE
        for (i in path.indices) {
            val d = hypot(path[i][0] - pose.x, path[i][1] - pose.y)
            if (d < bd) { bd = d; best = i }
        }
        return best
    }

    /** One control step. [vNow]: the car's speed (m/s). */
    fun step(pose: Pose2, path: List<DoubleArray>, vNow: Double): VelCmd {
        if (path.isEmpty()) return VelCmd.STOP
        val goal = path.last()
        val toGoal = remaining(pose, path)
        if (hypot(goal[0] - pose.x, goal[1] - pose.y) < p.goalTolerance && toGoal < p.goalTolerance * 2) {
            vLast = 0.0; return VelCmd(0.0, 0.0, "arrived")
        }
        // the carrot: lookahead distance along the path from the closest point
        val ld = (abs(vNow) * p.lookaheadTime).coerceIn(p.minLookahead, p.maxLookahead)
        val c0 = closest(pose, path)
        var carrot = path.last()
        var acc = 0.0
        for (i in c0 + 1 until path.size) {
            acc += hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            if (acc >= ld) { carrot = path[i]; break }
        }
        val local = pose.toLocal(carrot[0], carrot[1])
        val bearing = atan2(local[1], local[0])

        // 3-point turn: the path goes off behind the car
        if (!reversing && abs(bearing) > p.reverseTurnStart && toGoal > p.goalTolerance * 2) { reversing = true; reversedM = 0.0 }
        if (reversing) {
            if (abs(bearing) < p.reverseTurnEnd || reversedM > 1.5) { reversing = false }
            else {
                val v = -p.reverseSpeed
                // reversing, a clockwise turn rate swings the nose clockwise: turn towards the carrot
                val w = sign(bearing) * p.reverseSpeed / p.minTurnRadius
                if (arcCollides(pose, v, w)) { reversing = false; vLast = 0.0; return VelCmd(0.0, 0.0, "blocked behind") }
                reversedM += p.reverseSpeed * p.dt
                vLast = v
                return VelCmd(v, w, "3-point turn")
            }
        }

        // pure pursuit curvature, limited to what the steering can do
        val d2 = local[0] * local[0] + local[1] * local[1]
        var kappa = if (d2 > 1e-6) 2.0 * local[1] / d2 else 0.0
        val kMax = 1.0 / p.minTurnRadius
        kappa = kappa.coerceIn(-kMax, kMax)

        var v = p.desiredVel
        // regulation: curvature
        if (abs(kappa) > 1e-6) {
            val r = 1.0 / abs(kappa)
            if (r < p.regMinRadius) v *= r / p.regMinRadius
        }
        // regulation: proximity to obstacles
        val dObs = cm.distanceForCost(cm.costAt(pose.x, pose.y))
        if (dObs < p.costScalingDist) v *= p.costScalingGain * dObs / p.costScalingDist
        v = max(v, p.regMinSpeed)
        // approach
        if (toGoal < p.approachDist) v = max(p.minApproachVel, v * toGoal / p.approachDist)
        v = min(v, p.desiredVel)
        // velocity smoother
        v = v.coerceIn(vLast - p.maxDecel * p.dt, vLast + p.maxAccel * p.dt)
        if (v < 0) v = 0.0
        val vCheck = max(v, p.regMinSpeed)
        if (arcCollides(pose, vCheck, vCheck * kappa)) { vLast = 0.0; return VelCmd(0.0, 0.0, "obstacle ahead") }
        vLast = v
        return VelCmd(v, v * kappa, "following")
    }

    /** Simulate the arc for [p.timeToCollision] (or until the carrot) and check the costmap. */
    fun arcCollides(pose: Pose2, v: Double, w: Double): Boolean {
        val steps = 15
        val dt = p.timeToCollision / steps
        var x = pose.x; var y = pose.y; var yaw = pose.yaw
        // Already too close to something (e.g. parked by a wall): moving is allowed as
        // long as it doesn't get any closer.
        val start = cm.costAt(x, y)
        for (i in 1..steps) {
            x += v * cos(yaw) * dt; y += v * sin(yaw) * dt; yaw += w * dt
            val c = cm.costAt(x, y)
            if (start < Costmap.INSCRIBED) { if (c >= Costmap.INSCRIBED) return true }
            else if (c > start) return true
        }
        return false
    }
}

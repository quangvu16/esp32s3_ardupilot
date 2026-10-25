package org.ardupilot.esp32gcs.nav

import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/*
 * Indoor navigation in the app, after ROS 2 Nav2 (github.com/ros-navigation/navigation2):
 * a saved map (map_server), AMCL localisation, a costmap with inflation, an A* planner,
 * Regulated Pure Pursuit and a small behaviour state machine with recoveries and
 * patrols. No ROS: the car's LiDAR scan and pose come over MAVLink, and the car is
 * driven with velocity + turn-rate targets in Guided mode.
 *
 * Frames, as ArduPilot's: x north, y east (metres), yaw clockwise from north (rad).
 * Robot frame: x forward, y right; a scan angle is clockwise from straight ahead.
 * "odom" is the car's own frame (EKF / onboard LiDAR SLAM, LOCAL_POSITION_NED),
 * which starts again at every boot; "map" is the saved map's frame. AMCL finds
 * the map <- odom transform, as Nav2's AMCL publishes map -> odom.
 */

/** A 2D pose or transform. */
data class Pose2(val x: Double = 0.0, val y: Double = 0.0, val yaw: Double = 0.0) {
    /** this ⊕ p: p (in this pose's frame) expressed in the outer frame. */
    operator fun plus(p: Pose2): Pose2 {
        val c = cos(yaw); val s = sin(yaw)
        return Pose2(x + c * p.x - s * p.y, y + s * p.x + c * p.y, wrapPi(yaw + p.yaw))
    }

    /** The inverse transform: inverse() + this = identity. */
    fun inverse(): Pose2 {
        val c = cos(yaw); val s = sin(yaw)
        return Pose2(-(c * x + s * y), -(-s * x + c * y), wrapPi(-yaw))
    }

    /** A point given in this pose's frame, in the outer frame. */
    fun apply(px: Double, py: Double): DoubleArray {
        val c = cos(yaw); val s = sin(yaw)
        return doubleArrayOf(x + c * px - s * py, y + s * px + c * py)
    }

    /** A point given in the outer frame, in this pose's frame. */
    fun toLocal(px: Double, py: Double): DoubleArray {
        val c = cos(yaw); val s = sin(yaw)
        val dx = px - x; val dy = py - y
        return doubleArrayOf(c * dx + s * dy, -s * dx + c * dy)
    }

    fun distanceTo(o: Pose2) = hypot(o.x - x, o.y - y)
}

fun wrapPi(a: Double): Double {
    var r = (a + PI) % (2 * PI)
    if (r < 0) r += 2 * PI
    return r - PI
}

fun angleTo(fromX: Double, fromY: Double, toX: Double, toY: Double) = atan2(toY - fromY, toX - fromX)

/** A named goal in the map frame (Nav2's waypoint / pose). */
data class Waypoint(val name: String, val x: Double, val y: Double, val yaw: Double = Double.NaN)

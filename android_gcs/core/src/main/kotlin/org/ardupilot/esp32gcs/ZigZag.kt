package org.ardupilot.esp32gcs

import kotlin.math.cos
import kotlin.math.hypot

/**
 * Copter's ZigZag mode as a Rover mission: drive A to B, step sideways, drive
 * back parallel to it, and so on. Rover has no ZigZag mode, so the app writes
 * the lanes as waypoints and the car runs them in Auto. The settings follow
 * Copter's ZIGZ_ parameters; [runOutM] is for a car, which can't turn tighter
 * than TURN_RADIUS: each line is extended past both ends so the car turns
 * outside the area it covers.
 */
class ZigZag {
    var a: DoubleArray? = null           // lat, lon
    var b: DoubleArray? = null
    var sideDistM = 2f                   // ZIGZ_SIDE_DIST: distance between lines
    var right = true                     // ZIGZ_DIRECTION 1 (right) or 3 (left) of A->B
    var lines = 5                        // ZIGZ_LINE_NUM: lines to drive
    var waitS = 0f                       // ZIGZ_WP_DELAY: wait at the end of each line
    var runOutM = 0f                     // extra length past both ends of each line, for turning

    val lengthM get() = a?.let { p -> b?.let { q -> distance(p, q) } } ?: 0.0

    /** Why the lanes can't be made yet, or null. */
    fun problem(): String? = when {
        a == null -> "Set point A"
        b == null -> "Set point B"
        lengthM < 1.0 -> "A and B are less than 1 m apart"
        sideDistM <= 0f -> "Line spacing must be more than 0"
        lines < 1 -> "At least 1 line"
        lines > 200 -> "At most 200 lines"
        else -> null
    }

    /**
     * The waypoints: line 1 runs A to B, line 2 back from B' to A' one spacing to
     * the side, and so on. Each line is two waypoints (its start and end, moved
     * out by the run-out); the car waits [waitS] at each line's end. Line 1 starts
     * at A itself: A is usually where the car stands, and a start behind it would
     * make the car loop round first.
     */
    fun waypoints(): List<MissionItem> {
        val p = a ?: return emptyList(); val q = b ?: return emptyList()
        val len = distance(p, q)
        // flat-earth metres around A: x east, y north
        val (bx, by) = toLocal(p, q)
        val ux = bx / len; val uy = by / len                 // along A->B
        val sx = if (right) uy else -uy                      // sideways: A->B turned 90 degrees right or left
        val sy = if (right) -ux else ux
        val out = ArrayList<MissionItem>()
        for (i in 0 until lines) {
            val off = i * sideDistM.toDouble()
            val r = runOutM.toDouble()
            // start and end of this line, A side and B side
            val rs = if (i == 0) 0.0 else r                  // no run-out behind A on the first line
            val ax = sx * off - ux * rs; val ay = sy * off - uy * rs
            val ex = bx + sx * off + ux * r; val ey = by + sy * off + uy * r
            val (x1, y1, x2, y2) = if (i % 2 == 0) listOf(ax, ay, ex, ey) else listOf(ex, ey, ax, ay)
            out += wp(p, x1, y1, 0f)
            out += wp(p, x2, y2, waitS)
        }
        return out
    }

    private fun wp(origin: DoubleArray, x: Double, y: Double, wait: Float): MissionItem {
        val (la, lo) = fromLocal(origin, x, y)
        return MissionItem(lat = la, lon = lo, p1 = wait)
    }

    companion object {
        private const val M_PER_DEG = 111320.0

        fun toLocal(origin: DoubleArray, p: DoubleArray): Pair<Double, Double> =
            Pair((p[1] - origin[1]) * M_PER_DEG * cos(Math.toRadians(origin[0])), (p[0] - origin[0]) * M_PER_DEG)

        fun fromLocal(origin: DoubleArray, x: Double, y: Double): Pair<Double, Double> =
            Pair(origin[0] + y / M_PER_DEG, origin[1] + x / (M_PER_DEG * cos(Math.toRadians(origin[0]))))

        fun distance(p: DoubleArray, q: DoubleArray): Double {
            val (x, y) = toLocal(p, q)
            return hypot(x, y)
        }
    }
}

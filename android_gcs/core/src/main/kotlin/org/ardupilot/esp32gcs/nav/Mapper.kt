package org.ardupilot.esp32gcs.nav

import kotlin.math.abs

/**
 * Builds a map from scans whose poses are known (the car's own LiDAR SLAM pose):
 * log-odds ray tracing, as a "mapping with known poses" step of slam_toolbox or
 * Hector. Each ray lowers the cells it passes (free) and raises the cell it ends
 * in (occupied). A ray with no return clears only a short way, since a dark or
 * shiny wall can also give no return.
 */
class Mapper(sizeM: Double = 40.0, val res: Double = 0.10, centre: Pose2 = Pose2()) {
    val grid: OccGrid
    var scans = 0
        private set

    init {
        val n = (sizeM / res).toInt()
        grid = OccGrid(n, n, res, centre.x - n * res / 2, centre.y - n * res / 2)
    }

    /** Add a scan given in the map's frame. */
    @Synchronized fun add(scan: OdomScan) {
        for (k in 0 until scan.size) {
            if (scan.hit[k]) ray(scan.ox[k], scan.oy[k], scan.px[k], scan.py[k], FREE_STEP, true)
            else ray(scan.ox[k], scan.oy[k], scan.px[k], scan.py[k], FREE_STEP_NO_RETURN, false)
        }
        scans++
    }

    /** A copy for drawing or saving, taken under the lock. */
    @Synchronized fun snapshot(): OccGrid = grid.copy()

    private fun ray(x0: Double, y0: Double, x1: Double, y1: Double, freeStep: Int, hit: Boolean) {
        var r0 = grid.row(x0); var c0 = grid.col(y0)
        val r1 = grid.row(x1); val c1 = grid.col(y1)
        val dr = abs(r1 - r0); val dc = abs(c1 - c0)
        val sr = if (r1 > r0) 1 else -1; val sc = if (c1 > c0) 1 else -1
        var err = dc - dr
        val steps = maxOf(dr, dc)
        for (s in 0..steps) {
            if (!grid.inside(r0, c0)) return
            val i = grid.index(r0, c0)
            val last = s == steps
            // a ray with no return stops at a known wall: the LiDAR may just have missed it
            if (!hit && grid.occupied(i)) return
            if (last && hit) {
                grid.cells[i] = (grid.cells[i] + OCC_STEP).coerceAtMost(LIMIT).toByte()
            } else if (!hit || s < steps - 1) {
                // the cell just before a hit is left alone: a wall seen at a grazing angle
                // would otherwise be eaten away by its own neighbouring rays
                grid.cells[i] = (grid.cells[i] - freeStep).coerceAtLeast(-LIMIT).toByte()
            }
            if (last) return
            val e2 = 2 * err
            if (e2 > -dr) { err -= dr; c0 += sc }
            if (e2 < dc) { err += dc; r0 += sr }
        }
    }

    companion object {
        const val OCC_STEP = 24
        const val FREE_STEP = 6
        const val FREE_STEP_NO_RETURN = 2
        const val LIMIT = 120
    }
}

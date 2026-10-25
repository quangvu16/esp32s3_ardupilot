package org.ardupilot.esp32gcs.nav

import kotlin.math.cos
import kotlin.math.sin

/**
 * The Indoor screen's view of the map, shared by the apps: north up, east right,
 * [scale] pixels a metre, map point ([cx], [cy]) at the middle of the screen.
 * The view follows the car until the user drags it.
 */
class Viewport {
    var cx = 0.0; var cy = 0.0
    var scale = 60.0
    var width = 1.0; var height = 1.0
    var follow = true

    @Suppress("UNUSED_PARAMETER") fun sx(x: Double, y: Double) = width / 2 + (y - cy) * scale
    @Suppress("UNUSED_PARAMETER") fun sy(x: Double, y: Double) = height / 2 - (x - cx) * scale
    /** Screen to map: x north, y east. */
    @Suppress("UNUSED_PARAMETER") fun mapX(sx: Double, sy: Double) = cx - (sy - height / 2) / scale
    @Suppress("UNUSED_PARAMETER") fun mapY(sx: Double, sy: Double) = cy + (sx - width / 2) / scale

    /** Move the view by a finger's movement as Android's onScroll gives it (old - new position, pixels). */
    fun pan(dxPx: Double, dyPx: Double) { cy += dxPx / scale; cx -= dyPx / scale; follow = false }

    fun zoom(factor: Double, fx: Double, fy: Double) {
        val mx = mapX(fx, fy); val my = mapY(fx, fy)
        scale = (scale * factor).coerceIn(8.0, 400.0)
        // keep the map point under the fingers where it is
        cx = mx + (fy - height / 2) / scale
        cy = my - (fx - width / 2) / scale
    }

    fun centreOn(p: Pose2) { cx = p.x; cy = p.y }

    /** A map's bounds in view: centre on its known cells. */
    fun fit(g: OccGrid) {
        var r0 = g.height; var r1 = -1; var c0 = g.width; var c1 = -1
        for (r in 0 until g.height) for (c in 0 until g.width) if (g.known(g.index(r, c))) {
            if (r < r0) r0 = r; if (r > r1) r1 = r; if (c < c0) c0 = c; if (c > c1) c1 = c
        }
        if (r1 < 0) return
        cx = (g.cellX(r0) + g.cellX(r1)) / 2; cy = (g.cellY(c0) + g.cellY(c1)) / 2
        val spanX = (r1 - r0 + 4) * g.res; val spanY = (c1 - c0 + 4) * g.res
        scale = minOf(height / spanX, width / spanY).coerceIn(8.0, 400.0)
    }

    /** The car's outline as a triangle (3 screen points, x then y), [size] metres long. */
    fun carTriangle(p: Pose2, size: Double): DoubleArray {
        val pts = doubleArrayOf(size * 0.6, 0.0, -size * 0.4, size * 0.3, -size * 0.4, -size * 0.3)
        val out = DoubleArray(6)
        for (k in 0..2) {
            val lx = pts[2 * k]; val ly = pts[2 * k + 1]
            val wx = p.x + cos(p.yaw) * lx - sin(p.yaw) * ly
            val wy = p.y + sin(p.yaw) * lx + cos(p.yaw) * ly
            out[2 * k] = sx(wx, wy); out[2 * k + 1] = sy(wx, wy)
        }
        return out
    }

    companion object {
        /**
         * The map as ARGB pixels (width = grid columns, height = grid rows, top row
         * north), with the costmap's inflation tinted if given.
         */
        fun pixels(g: OccGrid, cost: ByteArray?): IntArray {
            val px = IntArray(g.width * g.height)
            for (r in 0 until g.height) for (c in 0 until g.width) {
                val i = g.index(r, c)
                var argb = when {
                    g.occupied(i) -> 0xFF1F2A24.toInt()
                    g.free(i) -> 0xFFFFFFFF.toInt()
                    else -> 0xFFD9DED9.toInt()
                }
                if (cost != null && !g.occupied(i)) {
                    val k = cost[i].toInt() and 0xff
                    argb = when {
                        k == Costmap.LETHAL -> 0xFFC2410C.toInt()          // an obstacle the LiDAR sees now
                        k == Costmap.INSCRIBED -> 0xFFF4B9A0.toInt()
                        k in 1..252 && g.free(i) -> blend(0xFFFFFFFF.toInt(), 0xFF8FB3EE.toInt(), k / 252.0 * 0.6)
                        else -> argb
                    }
                }
                px[(g.height - 1 - r) * g.width + c] = argb
            }
            return px
        }

        private fun blend(a: Int, b: Int, f: Double): Int {
            fun ch(s: Int) = ((((a shr s) and 0xff) * (1 - f) + ((b shr s) and 0xff) * f).toInt() and 0xff) shl s
            return (0xFF shl 24) or ch(16) or ch(8) or ch(0)
        }
    }
}

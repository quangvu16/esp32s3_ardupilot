package org.ardupilot.esp32gcs.nav

import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.ln

/**
 * Nav2's layered costmap (costmap_2d) on the saved map's grid: a static layer
 * (the map's walls), an obstacle layer (what the LiDAR sees now: people, boxes,
 * a closed door) and an inflation layer around both. Costs as Nav2's: 254 lethal
 * (a wall), 253 inscribed (the car's centre here means touching), 252..1 falling
 * off as 252·e^(-k(d - r_inscribed)) out to the inflation radius, 0 free,
 * 255 unknown.
 */
class Costmap(val map: OccGrid, private val p: Params = Params()) {
    class Params {
        var inscribedRadius = 0.20          // m: half the car's width plus a little (Nav2: from the footprint)
        var inflationRadius = 0.55          // m (Nav2 default 0.55)
        var costScaling = 4.0               // cost_scaling_factor (Nav2 10; gentler keeps the car mid-corridor)
        var obstacleRange = 3.5             // m: scan points marked as obstacles out to here
        var raytraceRange = 4.0             // m: and free space cleared out to here
        var obstacleKeepMs = 8000L          // a mark not seen again for this long is dropped
        var unknownIsLethal = true          // allow_unknown = false
    }

    val params get() = p
    val width = map.width
    val height = map.height
    private val staticCost = ByteArray(width * height)        // inflated static layer, as unsigned
    private val obstacleMs = LongArray(width * height)        // when each obstacle cell was last marked, 0 = none
    private val master = ByteArray(width * height)
    private val kernel: IntArray                              // dr, dc, cost triples
    @Volatile var version = 0
        private set

    init {
        val cells = ceil(p.inflationRadius / map.res).toInt()
        val k = ArrayList<Int>()
        for (dr in -cells..cells) for (dc in -cells..cells) {
            val c = costAtDistance(hypot(dr * map.res, dc * map.res))
            if (c > 0) { k += dr; k += dc; k += c }
        }
        kernel = k.toIntArray()
        for (i in staticCost.indices) {
            staticCost[i] = when {
                map.occupied(i) -> LETHAL
                !map.known(i) && p.unknownIsLethal -> UNKNOWN
                else -> 0
            }.toByte()
        }
        val walls = (0 until width * height).filter { map.occupied(it) }
        for (i in walls) stamp(staticCost, i / width, i % width)
        rebuild(0L)
    }

    /** Nav2's inflation formula. */
    fun costAtDistance(d: Double): Int = when {
        d <= 1e-9 -> LETHAL
        d <= p.inscribedRadius -> INSCRIBED
        d <= p.inflationRadius -> (252.0 * exp(-p.costScaling * (d - p.inscribedRadius))).toInt().coerceIn(1, 252)
        else -> 0
    }

    /** Distance to the nearest obstacle for a cost (the inverse of [costAtDistance]), for RPP's slow-down. */
    fun distanceForCost(c: Int): Double = when {
        c >= INSCRIBED -> 0.0
        c <= 0 -> p.inflationRadius
        else -> p.inscribedRadius + ln(252.0 / c) / p.costScaling
    }

    fun cost(i: Int) = master[i].toInt() and 0xff

    fun costAt(x: Double, y: Double): Int {
        val r = map.row(x); val c = map.col(y)
        return if (map.inside(r, c)) cost(map.index(r, c)) else if (p.unknownIsLethal) UNKNOWN else 0
    }

    /** True if a car centred here would touch something. */
    fun blocked(x: Double, y: Double): Boolean { val c = costAt(x, y); return c >= INSCRIBED }

    /** The obstacle layer: mark what [scan] (map frame) hits, clear what it sees through. */
    @Synchronized fun addScan(scan: OdomScan, nowMs: Long) {
        for (k in 0 until scan.size) {
            val dx = scan.px[k] - scan.ox[k]; val dy = scan.py[k] - scan.oy[k]
            val len = hypot(dx, dy)
            if (len < 1e-6) continue
            val clearLen = minOf(len - map.res, p.raytraceRange)
            clear(scan.ox[k], scan.oy[k], scan.ox[k] + dx / len * clearLen, scan.oy[k] + dy / len * clearLen)
            if (scan.hit[k] && len <= p.obstacleRange) {
                val r = map.row(scan.px[k]); val c = map.col(scan.py[k])
                if (map.inside(r, c) && !map.occupied(map.index(r, c))) obstacleMs[map.index(r, c)] = nowMs
            }
        }
        rebuild(nowMs)
    }

    /** Nav2's "clear costmap" recovery: forget everything the LiDAR marked. */
    @Synchronized fun clearObstacles() { obstacleMs.fill(0L); rebuild(0L) }

    @Synchronized fun snapshot(): ByteArray = master.copyOf()

    private fun clear(x0: Double, y0: Double, x1: Double, y1: Double) {
        var r0 = map.row(x0); var c0 = map.col(y0)
        val r1 = map.row(x1); val c1 = map.col(y1)
        val dr = abs(r1 - r0); val dc = abs(c1 - c0)
        val sr = if (r1 > r0) 1 else -1; val sc = if (c1 > c0) 1 else -1
        var err = dc - dr
        while (true) {
            if (!map.inside(r0, c0)) return
            obstacleMs[map.index(r0, c0)] = 0L
            if (r0 == r1 && c0 == c1) return
            val e2 = 2 * err
            if (e2 > -dr) { err -= dr; c0 += sc }
            if (e2 < dc) { err += dc; r0 += sr }
        }
    }

    private fun rebuild(nowMs: Long) {
        staticCost.copyInto(master)
        for (i in obstacleMs.indices) {
            val t = obstacleMs[i]
            if (t == 0L) continue
            if (nowMs - t > p.obstacleKeepMs) { obstacleMs[i] = 0L; continue }
            master[i] = LETHAL.toByte()
            stamp(master, i / width, i % width)
        }
        version++
    }

    private fun stamp(layer: ByteArray, r: Int, c: Int) {
        var k = 0
        while (k < kernel.size) {
            val rr = r + kernel[k]; val cc = c + kernel[k + 1]; val cost = kernel[k + 2]
            k += 3
            if (rr !in 0 until height || cc !in 0 until width) continue
            val i = rr * width + cc
            val old = layer[i].toInt() and 0xff
            if (old == UNKNOWN && cost < LETHAL) continue
            if (cost > old || old == UNKNOWN) layer[i] = cost.toByte()
        }
    }

    companion object {
        const val LETHAL = 254
        const val INSCRIBED = 253
        const val UNKNOWN = 255
    }
}

package org.ardupilot.esp32gcs.nav

import java.util.PriorityQueue
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max

/**
 * The global planner: cost-aware A* on the costmap, 8-connected, as Nav2's
 * Smac 2D planner. Each step costs its length × (1 + travelMult · cost/252), so
 * the path keeps away from walls where it can. Then a simple smoother (Nav2's
 * SimpleSmoother) rounds the grid's zig-zags, and the path is given every
 * [spacing] metres.
 */
class Planner(private val cm: Costmap) {
    var travelMult = 2.0                 // cost_travel_multiplier (Nav2 Smac 2D: 2.0)
    var goalTolerance = 0.3              // m: a blocked goal is moved to the nearest free cell this close
    var maxIterations = 400_000
    var spacing = 0.10

    /** A path from [start] to [goal] (map frame) as x, y pairs, or null. */
    fun plan(start: Pose2, goalX: Double, goalY: Double): List<DoubleArray>? {
        val map = cm.map
        val w = map.width; val h = map.height
        val s = nearestFree(map.row(start.x), map.col(start.y), 0.35) ?: return null
        val g = nearestFree(map.row(goalX), map.col(goalY), goalTolerance) ?: return null
        val n = w * h
        val gCost = FloatArray(n) { Float.MAX_VALUE }
        val parent = IntArray(n) { -1 }
        val closed = BooleanArray(n)
        val open = PriorityQueue<Long>()                      // (f as float bits) << 32 | index
        val gr = g / w; val gc = g % w
        fun heur(i: Int) = hypot((i / w - gr).toDouble(), (i % w - gc).toDouble()).toFloat()
        fun key(f: Float, i: Int) = (f.toRawBits().toLong() shl 32) or i.toLong()
        gCost[s] = 0f
        open.add(key(heur(s), s))
        var iters = 0
        while (open.isNotEmpty() && iters++ < maxIterations) {
            val cur = (open.poll() and 0xffffffffL).toInt()
            if (closed[cur]) continue
            if (cur == g) return smooth(cells(parent, g))
            closed[cur] = true
            val cr = cur / w; val cc = cur % w
            for (k in 0 until 8) {
                val nr = cr + DR[k]; val nc = cc + DC[k]
                if (nr !in 0 until h || nc !in 0 until w) continue
                val ni = nr * w + nc
                if (closed[ni]) continue
                val c = cm.cost(ni)
                if (c >= Costmap.INSCRIBED) continue          // lethal, inscribed or unknown
                if (k >= 4) {                                  // no squeezing diagonally between two blocked cells
                    if (cm.cost(cr * w + nc) >= Costmap.INSCRIBED && cm.cost(nr * w + cc) >= Costmap.INSCRIBED) continue
                }
                val ng = gCost[cur] + STEP[k] * (1f + (travelMult * c / 252.0).toFloat())
                if (ng < gCost[ni]) {
                    gCost[ni] = ng; parent[ni] = cur
                    open.add(key(ng + heur(ni), ni))
                }
            }
        }
        return null
    }

    /** Is the path still clear (for replanning when the obstacle layer changes)? */
    fun pathBlocked(path: List<DoubleArray>, from: Int = 0): Boolean {
        for (k in from until path.size) if (cm.blocked(path[k][0], path[k][1])) return true
        return false
    }

    private fun nearestFree(r: Int, c: Int, radius: Double): Int? {
        val map = cm.map
        val cells = (radius / map.res).toInt()
        var best: Int? = null; var bestD = Double.MAX_VALUE
        for (dr in -cells..cells) for (dc in -cells..cells) {
            val rr = r + dr; val cc = c + dc
            if (!map.inside(rr, cc)) continue
            val i = map.index(rr, cc)
            if (cm.cost(i) >= Costmap.INSCRIBED) continue
            val d = hypot(dr.toDouble(), dc.toDouble())
            if (d <= cells + 1e-9 && d < bestD) { bestD = d; best = i }
        }
        return best
    }

    private fun cells(parent: IntArray, goal: Int): List<DoubleArray> {
        val map = cm.map
        val out = ArrayList<DoubleArray>()
        var i = goal
        while (i >= 0) { out += doubleArrayOf(map.cellX(i / map.width), map.cellY(i % map.width)); i = parent[i] }
        out.reverse()
        return out
    }

    /** Nav2 SimpleSmoother: pull each point towards its neighbours, but not into anything. */
    private fun smooth(path: List<DoubleArray>): List<DoubleArray> {
        if (path.size < 3) return path
        val orig = path.map { it.copyOf() }
        val p = path.map { it.copyOf() }
        val wData = 0.2; val wSmooth = 0.3
        repeat(60) {
            for (k in 1 until p.size - 1) for (d in 0..1) {
                val old = p[k][d]
                p[k][d] += wData * (orig[k][d] - p[k][d]) + wSmooth * (p[k - 1][d] + p[k + 1][d] - 2 * p[k][d])
                if (cm.blocked(p[k][0], p[k][1])) p[k][d] = old
            }
        }
        // even spacing
        val out = ArrayList<DoubleArray>()
        out += p[0]
        var carry = 0.0
        for (k in 1 until p.size) {
            val ax = p[k - 1][0]; val ay = p[k - 1][1]
            val seg = hypot(p[k][0] - ax, p[k][1] - ay)
            var at = spacing - carry
            while (at <= seg) {
                val f = at / seg
                out += doubleArrayOf(ax + f * (p[k][0] - ax), ay + f * (p[k][1] - ay))
                at += spacing
            }
            carry = max(0.0, seg - (at - spacing))
        }
        val last = p.last()
        if (abs(out.last()[0] - last[0]) + abs(out.last()[1] - last[1]) > 1e-6) out += last
        return out
    }

    companion object {
        private val DR = intArrayOf(1, -1, 0, 0, 1, 1, -1, -1)
        private val DC = intArrayOf(0, 0, 1, -1, 1, -1, 1, -1)
        private val STEP = floatArrayOf(1f, 1f, 1f, 1f, 1.4142f, 1.4142f, 1.4142f, 1.4142f)
    }
}

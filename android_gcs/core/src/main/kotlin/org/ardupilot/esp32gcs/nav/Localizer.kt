package org.ardupilot.esp32gcs.nav

import java.util.Random
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Where the car is in a saved map: Nav2's AMCL (adaptive Monte Carlo localisation)
 * with its likelihood-field laser model. The particles move with the car's own
 * odometry (the onboard LiDAR SLAM / EKF pose, which is smooth but starts again at
 * every boot) and are weighed by how well the scan lands on the map's walls. The
 * result is the map ← odom transform [offset], which Nav2's AMCL publishes as
 * map -> odom: the car's map pose is offset + odom pose.
 */
class Localizer(val map: OccGrid, private val p: Params = Params(), seed: Long = 1) {
    /** Nav2 AMCL defaults, with the changes the research suggested for this car (docs/INDOOR_NAV.md). */
    class Params {
        var particles = 600                 // Nav2: 500..2000, adaptive; fixed here
        var beams = 72                      // max_beams (Nav2 60)
        var sigmaHit = 0.15                 // m (Nav2 0.2)
        var zHit = 0.5; var zRand = 0.5     // Nav2 defaults
        var maxDist = 1.0                   // laser_likelihood_max_dist, m (Nav2 2.0)
        var alpha1 = 0.1; var alpha2 = 0.1; var alpha3 = 0.1; var alpha4 = 0.1   // odometry noise (Nav2 0.2)
        var updateMinD = 0.10               // m (Nav2 0.25)
        var updateMinA = 0.10               // rad (Nav2 0.2)
        var initSigmaXY = 0.3; var initSigmaYaw = 0.2
    }

    private val rnd = Random(seed)
    private val n get() = p.particles
    private var x = DoubleArray(n); private var y = DoubleArray(n); private var t = DoubleArray(n)
    private var w = DoubleArray(n) { 1.0 / n }
    private val field = FloatArray(map.width * map.height)   // distance to the nearest wall, m

    @Volatile var offset = Pose2()            // map ← odom
        private set
    @Volatile var pose = Pose2()              // in the map
        private set
    @Volatile var spreadXY = Double.MAX_VALUE // particle standard deviation, m
        private set
    @Volatile var spreadYaw = Double.MAX_VALUE
        private set
    @Volatile var score = 0.0                 // share of beams within sigmaHit of a wall, last update
        private set
    @Volatile var updates = 0
        private set
    var initialised = false
        private set
    private var lastOdom: Pose2? = null
    private var sinceUpdate = Pose2()
    private var forceUpdates = 0

    init { buildField() }

    val confident get() = initialised && spreadXY < 0.25 && spreadYaw < 0.15 && score > 0.4

    /** Nav2's "2D Pose Estimate": scatter the particles around a map pose. */
    @Synchronized fun setPose(mapPose: Pose2, odom: Pose2, sigmaXY: Double = p.initSigmaXY, sigmaYaw: Double = p.initSigmaYaw) {
        for (i in 0 until n) {
            x[i] = mapPose.x + rnd.nextGaussian() * sigmaXY
            y[i] = mapPose.y + rnd.nextGaussian() * sigmaXY
            t[i] = wrapPi(mapPose.yaw + rnd.nextGaussian() * sigmaYaw)
            w[i] = 1.0 / n
        }
        lastOdom = odom
        initialised = true
        forceUpdates = 5            // weigh the first scans even while the car stands still
        summarise(odom)
    }

    /**
     * Global localisation: score every other free cell (0.2 m) at 48 headings with
     * a thinned scan and a forgiving match, refine the best 30 by hill-climbing
     * with the normal match, and start the particles at the best 5. For "where am
     * I?" when the car did not start on its usual mark; driving a little then lets
     * AMCL pick the right one.
     */
    @Synchronized fun findGlobally(scan: OdomScan): Boolean {
        val (lx, ly) = scan.local()
        val idx = thin(lx.size, 30, scan.hit)
        if (idx.size < 8) return false
        val keep = 30
        val cx = DoubleArray(keep); val cy = DoubleArray(keep); val cyaw = DoubleArray(keep); val cs = DoubleArray(keep) { -1.0 }
        val heads = 48
        for (r in 0 until map.height step 2) for (c in 0 until map.width step 2) {
            if (!map.free(map.index(r, c))) continue
            val px = map.cellX(r); val py = map.cellY(c)
            for (h in 0 until heads) {
                val yaw = -PI + h * 2 * PI / heads
                val sc = scoreAt(px, py, yaw, lx, ly, idx, 0.3)
                if (sc > cs[keep - 1]) {
                    var k = keep - 1
                    while (k > 0 && cs[k - 1] < sc) { cx[k] = cx[k - 1]; cy[k] = cy[k - 1]; cyaw[k] = cyaw[k - 1]; cs[k] = cs[k - 1]; k-- }
                    cx[k] = px; cy[k] = py; cyaw[k] = yaw; cs[k] = sc
                }
            }
        }
        if (cs[0] <= 0) return false
        // refine
        for (k in 0 until keep) {
            if (cs[k] <= 0) continue
            var bx = cx[k]; var by = cy[k]; var bt = cyaw[k]
            var best = scoreAt(bx, by, bt, lx, ly, idx, p.sigmaHit)
            var step = 0.1; var astep = 0.06
            repeat(30) {
                var improved = false
                for ((dx, dy, dt) in listOf(Triple(step, 0.0, 0.0), Triple(-step, 0.0, 0.0), Triple(0.0, step, 0.0),
                        Triple(0.0, -step, 0.0), Triple(0.0, 0.0, astep), Triple(0.0, 0.0, -astep))) {
                    val sc = scoreAt(bx + dx, by + dy, bt + dt, lx, ly, idx, p.sigmaHit)
                    if (sc > best) { best = sc; bx += dx; by += dy; bt += dt; improved = true }
                }
                if (!improved) { step /= 2; astep /= 2 }
            }
            cx[k] = bx; cy[k] = by; cyaw[k] = wrapPi(bt); cs[k] = best
        }
        val order = (0 until keep).filter { cs[it] > 0 }.sortedByDescending { cs[it] }.take(5)
        for (i in 0 until n) {
            val b = order[i % order.size]
            x[i] = cx[b] + rnd.nextGaussian() * 0.1; y[i] = cy[b] + rnd.nextGaussian() * 0.1
            t[i] = wrapPi(cyaw[b] + rnd.nextGaussian() * 0.05); w[i] = 1.0 / n
        }
        lastOdom = scan.end; initialised = true; forceUpdates = 10
        summarise(scan.end)
        return true
    }

    /**
     * One step: move the particles by the odometry since the last call, and weigh
     * them by [scan] (in the odom frame) when the car has moved enough (Nav2's
     * update_min_d / update_min_a). Returns true when the particles were weighed.
     */
    @Synchronized fun update(scan: OdomScan): Boolean {
        if (!initialised) return false
        val odom = scan.end
        val last = lastOdom ?: odom
        // odometry delta in the car's frame at the last pose
        val d = last.inverse() + odom
        lastOdom = odom
        val trans = hypot(d.x, d.y); val rot = abs(d.yaw)
        if (trans > 1e-4 || rot > 1e-4) {
            val sRot = p.alpha1 * rot + p.alpha2 * trans
            val sTrans = p.alpha3 * trans + p.alpha4 * rot
            for (i in 0 until n) {
                val dx = d.x + rnd.nextGaussian() * sTrans
                val dy = d.y + rnd.nextGaussian() * sTrans * 0.5
                val c = cos(t[i]); val s = sin(t[i])
                x[i] += c * dx - s * dy; y[i] += s * dx + c * dy
                t[i] = wrapPi(t[i] + d.yaw + rnd.nextGaussian() * sRot)
            }
        }
        sinceUpdate = Pose2(sinceUpdate.x + abs(d.x), sinceUpdate.y + abs(d.y), sinceUpdate.yaw + rot)
        val moved = hypot(sinceUpdate.x, sinceUpdate.y) >= p.updateMinD || sinceUpdate.yaw >= p.updateMinA
        if (!moved && forceUpdates <= 0) { summarise(odom); return false }
        if (forceUpdates > 0) forceUpdates--
        sinceUpdate = Pose2()

        val (lx, ly) = scan.local()
        val idx = thin(lx.size, p.beams, scan.hit)
        if (idx.isEmpty()) { summarise(odom); return false }
        var total = 0.0
        val zr = p.zRand / scan.maxRange
        val k = 1.0 / (2 * p.sigmaHit * p.sigmaHit)
        for (i in 0 until n) {
            val c = cos(t[i]); val s = sin(t[i])
            var q = 1.0
            for (j in idx) {
                val wx = x[i] + c * lx[j] - s * ly[j]; val wy = y[i] + s * lx[j] + c * ly[j]
                val dist = distAt(wx, wy)
                val pz = p.zHit * exp(-dist * dist * k) + zr
                q += pz * pz * pz                          // AMCL's likelihood_field combination
            }
            w[i] *= q
            total += w[i]
        }
        if (total <= 0 || total.isNaN()) { for (i in 0 until n) w[i] = 1.0 / n } else for (i in 0 until n) w[i] /= total
        // resample when the effective number of particles drops below half
        var sq = 0.0
        for (i in 0 until n) sq += w[i] * w[i]
        if (1.0 / sq < n / 2.0) resample()
        summarise(odom)
        // how well the mean pose explains the scan, for "localisation lost"
        val m = pose
        var good = 0
        for (j in idx) {
            val pnt = m.apply(lx[j], ly[j])
            if (distAt(pnt[0], pnt[1]) < 2 * p.sigmaHit) good++
        }
        score = good.toDouble() / idx.size
        updates++
        return true
    }

    private fun resample() {          // low-variance resampling
        val nx = DoubleArray(n); val ny = DoubleArray(n); val nt = DoubleArray(n)
        val step = 1.0 / n
        var r = rnd.nextDouble() * step
        var c = w[0]; var i = 0
        for (m in 0 until n) {
            val u = r + m * step
            while (u > c && i < n - 1) { i++; c += w[i] }
            nx[m] = x[i]; ny[m] = y[i]; nt[m] = t[i]
        }
        x = nx; y = ny; t = nt
        for (m in 0 until n) w[m] = step
    }

    private fun summarise(odom: Pose2) {
        var mx = 0.0; var my = 0.0; var sc = 0.0; var ss = 0.0
        for (i in 0 until n) { mx += w[i] * x[i]; my += w[i] * y[i]; sc += w[i] * cos(t[i]); ss += w[i] * sin(t[i]) }
        val myaw = atan2(ss, sc)
        var vx = 0.0; var vy = 0.0
        for (i in 0 until n) { vx += w[i] * (x[i] - mx) * (x[i] - mx); vy += w[i] * (y[i] - my) * (y[i] - my) }
        spreadXY = sqrt(vx + vy)
        spreadYaw = sqrt(-2 * kotlin.math.ln(hypot(sc, ss).coerceIn(1e-9, 1.0)))
        pose = Pose2(mx, my, myaw)
        offset = pose + odom.inverse()
    }

    /** Particles for drawing: x, y, yaw. */
    @Synchronized fun particles(): Array<DoubleArray> = arrayOf(x.copyOf(), y.copyOf(), t.copyOf())

    private fun scoreAt(px: Double, py: Double, yaw: Double, lx: DoubleArray, ly: DoubleArray, idx: IntArray, sigma: Double): Double {
        val c = cos(yaw); val s = sin(yaw)
        var q = 0.0
        val k = 1.0 / (2 * sigma * sigma)
        for (j in idx) {
            val d = distAt(px + c * lx[j] - s * ly[j], py + s * lx[j] + c * ly[j])
            q += exp(-d * d * k)
        }
        return q
    }

    private fun thin(size: Int, want: Int, hit: BooleanArray): IntArray {
        val hits = (0 until size).filter { hit[it] }
        if (hits.size <= want) return hits.toIntArray()
        return IntArray(want) { hits[(it.toLong() * hits.size / want).toInt()] }
    }

    private fun distAt(wx: Double, wy: Double): Double {
        val r = map.row(wx); val c = map.col(wy)
        if (!map.inside(r, c)) return p.maxDist
        return field[map.index(r, c)].toDouble()
    }

    /** Distance from every cell to the nearest occupied cell (two-pass 3-4 chamfer), capped at maxDist. */
    private fun buildField() {
        val w = map.width; val h = map.height
        val inf = 1e9f
        for (i in field.indices) field[i] = if (map.occupied(i)) 0f else inf
        val a = map.res.toFloat(); val b = (map.res * 1.4142).toFloat()
        for (r in 0 until h) for (c in 0 until w) {
            val i = r * w + c
            var v = field[i]
            if (c > 0) v = minOf(v, field[i - 1] + a)
            if (r > 0) {
                v = minOf(v, field[i - w] + a)
                if (c > 0) v = minOf(v, field[i - w - 1] + b)
                if (c < w - 1) v = minOf(v, field[i - w + 1] + b)
            }
            field[i] = v
        }
        for (r in h - 1 downTo 0) for (c in w - 1 downTo 0) {
            val i = r * w + c
            var v = field[i]
            if (c < w - 1) v = minOf(v, field[i + 1] + a)
            if (r < h - 1) {
                v = minOf(v, field[i + w] + a)
                if (c < w - 1) v = minOf(v, field[i + w + 1] + b)
                if (c > 0) v = minOf(v, field[i + w - 1] + b)
            }
            field[i] = v
        }
        val cap = p.maxDist.toFloat()
        for (i in field.indices) if (field[i] > cap) field[i] = cap
    }
}

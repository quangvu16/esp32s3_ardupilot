package org.ardupilot.esp32gcs.nav

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * One LiDAR rotation: [angles] clockwise from straight ahead (rad, 0..2π), [ranges]
 * in metres (+Infinity: nothing within [maxRange] that way). [endMs] is the end of
 * the rotation on the autopilot's clock (OBSTACLE_DISTANCE time_usec / 1000), the
 * same clock as LOCAL_POSITION_NED's time_boot_ms.
 */
class ScanFrame(val endMs: Long, val angles: FloatArray, val ranges: FloatArray, val maxRange: Float)

/**
 * The car's recent poses in its own frame (odom: EKF / onboard SLAM), from
 * LOCAL_POSITION_NED and ATTITUDE, by autopilot time. Thread-safe.
 */
class PoseHistory(private val keepMs: Long = 3000) {
    private val pt = LongArray(256); private val px = DoubleArray(256); private val py = DoubleArray(256)
    private val yt = LongArray(256); private val yy = DoubleArray(256)
    private var pn = 0; private var ph = 0; private var yn = 0; private var yh = 0

    @Synchronized fun addPosition(tMs: Long, x: Double, y: Double) {
        if (pn > 0 && tMs < pt[(ph + 255) % 256]) clear()       // the autopilot rebooted
        pt[ph] = tMs; px[ph] = x; py[ph] = y; ph = (ph + 1) % 256; if (pn < 256) pn++
    }

    @Synchronized fun addYaw(tMs: Long, yaw: Double) {
        if (yn > 0 && tMs < yt[(yh + 255) % 256]) { yn = 0; yh = 0 }
        yt[yh] = tMs; yy[yh] = yaw; yh = (yh + 1) % 256; if (yn < 256) yn++
    }

    @Synchronized fun clear() { pn = 0; ph = 0; yn = 0; yh = 0 }

    @Synchronized fun latestMs(): Long = if (pn == 0 || yn == 0) -1 else minOf(pt[(ph + 255) % 256], yt[(yh + 255) % 256])

    /** The pose at autopilot time [tMs], interpolated; up to [extrapolateMs] past the newest sample is held. */
    @Synchronized fun at(tMs: Long, extrapolateMs: Long = 250): Pose2? {
        val p = interp(pt, pn, ph, tMs, extrapolateMs) { i, j, f -> doubleArrayOf(px[i] + f * (px[j] - px[i]), py[i] + f * (py[j] - py[i])) } ?: return null
        val a = interp(yt, yn, yh, tMs, extrapolateMs) { i, j, f -> doubleArrayOf(yy[i] + f * wrapPi(yy[j] - yy[i])) } ?: return null
        return Pose2(p[0], p[1], wrapPi(a[0]))
    }

    private inline fun interp(t: LongArray, n: Int, head: Int, tMs: Long, extra: Long,
                              mix: (Int, Int, Double) -> DoubleArray): DoubleArray? {
        if (n == 0) return null
        val newest = (head + 255) % 256
        val oldest = (head - n + 256) % 256
        if (tMs >= t[newest]) return if (tMs - t[newest] <= extra) mix(newest, newest, 0.0) else null
        if (tMs < t[oldest] || t[newest] - tMs > keepMs) return null
        var k = newest
        for (s in 1 until n) {
            val prev = (k + 255) % 256
            if (t[prev] <= tMs) {
                val span = (t[k] - t[prev]).toDouble()
                return mix(prev, k, if (span > 0) (tMs - t[prev]) / span else 0.0)
            }
            k = prev
        }
        return null
    }
}

/**
 * A scan with the car's motion during the rotation taken out: every point is
 * placed where the car was when it was measured (the LD14 turns at ~6 rotations
 * a second, so at 0.5 m/s the first point of a rotation was taken 8 cm further
 * back). Points and the ray origins are in the odom frame; [end] is the car's
 * odom pose at the end of the rotation.
 */
class OdomScan(val end: Pose2, val ox: DoubleArray, val oy: DoubleArray,
               val px: DoubleArray, val py: DoubleArray, val hit: BooleanArray, val maxRange: Double) {
    val size get() = px.size

    /** The points in the car's frame at the end of the rotation (x forward, y right). */
    fun local(): Pair<DoubleArray, DoubleArray> {
        val inv = end.inverse()
        val lx = DoubleArray(size); val ly = DoubleArray(size)
        for (i in 0 until size) { val p = inv.apply(px[i], py[i]); lx[i] = p[0]; ly[i] = p[1] }
        return Pair(lx, ly)
    }

    companion object {
        /**
         * [periodS]: time of one rotation (LD14 ~1/6 s, LD06 ~1/10 s); point k at angle
         * a (clockwise from ahead) was measured (1 - a/2π)·period before the end, since the
         * rotation starts at the sensor's 0°. [sensorX]/[sensorY]: the LiDAR on the car.
         * Points nearer than [minRange] are the car itself. A ray with no return ends at
         * [clearRange] (for clearing free space only).
         */
        fun build(scan: ScanFrame, poses: PoseHistory, periodS: Double, sensorX: Double, sensorY: Double,
                  minRange: Double = 0.2, clearRange: Double = 4.0): OdomScan? {
            val end = poses.at(scan.endMs) ?: return null
            val n = scan.angles.size
            val ox = DoubleArray(n); val oy = DoubleArray(n); val px = DoubleArray(n); val py = DoubleArray(n)
            val hit = BooleanArray(n)
            var m = 0
            var cachedMs = Long.MIN_VALUE; var pose = end
            for (k in 0 until n) {
                var r = scan.ranges[k].toDouble()
                val isHit = r.isFinite()
                if (isHit && r < minRange) continue
                if (!isHit) r = clearRange
                val a = scan.angles[k].toDouble()
                val tMs = scan.endMs - ((1.0 - a / (2 * PI)) * periodS * 1000).toLong()
                if (tMs / 10 != cachedMs / 10) { pose = poses.at(tMs) ?: end; cachedMs = tMs }
                val s = pose.apply(sensorX, sensorY)
                val w = pose.apply(sensorX + r * cos(a), sensorY + r * sin(a))
                ox[m] = s[0]; oy[m] = s[1]; px[m] = w[0]; py[m] = w[1]; hit[m] = isHit
                m++
            }
            return OdomScan(end, ox.copyOf(m), oy.copyOf(m), px.copyOf(m), py.copyOf(m), hit.copyOf(m), scan.maxRange.toDouble())
        }
    }

    /** The same scan moved into another frame by [t] (e.g. map ← odom). */
    fun transformed(t: Pose2): OdomScan {
        val n = size
        val ox2 = DoubleArray(n); val oy2 = DoubleArray(n); val px2 = DoubleArray(n); val py2 = DoubleArray(n)
        for (i in 0 until n) {
            val o = t.apply(ox[i], oy[i]); val p = t.apply(px[i], py[i])
            ox2[i] = o[0]; oy2[i] = o[1]; px2[i] = p[0]; py2[i] = p[1]
        }
        return OdomScan(t + end, ox2, oy2, px2, py2, hit, maxRange)
    }
}

package org.ardupilot.esp32gcs

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.os.SystemClock
import android.view.MotionEvent
import android.view.View
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

/**
 * The LiDAR scan seen from above, car in the middle with its front up, and
 * the active behaviour's sectors on top. Tap to change the range shown.
 */
class LidarView(context: Context) : View(context) {
    /** A sector to shade: angles left-positive (hoverboard_laser convention), out to [radius] m. */
    class Wedge(val fromDeg: Float, val toDeg: Float, val radius: Float, val hot: Boolean)

    var scan: LidarScan? = null
    var minRange = 0.3f
    var wedges: List<Wedge> = emptyList()
    var target: Pair<Float, Float>? = null       // (angle left-positive, m) the behaviour is looking at
    var info = ""

    private val ranges = floatArrayOf(2f, 4f, 8f)
    private var rangeIdx = 1

    private val d = resources.displayMetrics.density
    private val c = { id: Int -> context.getColor(id) }
    private val bg = Paint().apply { color = c(R.color.surface) }
    private val ring = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = d; color = c(R.color.line) }
    private val label = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = c(R.color.muted); textSize = 10 * d }
    private val dot = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = c(R.color.fg) }
    private val near = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(239, 68, 68) }
    private val car = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = c(R.color.accent) }
    private val hotFill = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.argb(70, 239, 68, 68) }
    private val coolFill = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.argb(45, 34, 197, 94) }
    private val targetPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeWidth = 2.5f * d; color = Color.rgb(59, 130, 246)
    }
    private val infoPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = c(R.color.muted); textSize = 11 * d }

    override fun onMeasure(w: Int, h: Int) {
        val width = MeasureSpec.getSize(w)
        setMeasuredDimension(width, min(width, (340 * d).toInt()))
    }

    override fun onTouchEvent(e: MotionEvent): Boolean {
        if (e.actionMasked == MotionEvent.ACTION_UP) { rangeIdx = (rangeIdx + 1) % ranges.size; invalidate(); performClick() }
        return true
    }

    override fun performClick(): Boolean = super.performClick()

    /** Screen point for an angle (left-positive, degrees) and a range in metres. */
    private fun xy(cx: Float, cy: Float, pxPerM: Float, deg: Float, m: Float): Pair<Float, Float> {
        val a = Math.toRadians(deg.toDouble())
        return Pair(cx - (m * pxPerM * sin(a)).toFloat(), cy - (m * pxPerM * cos(a)).toFloat())
    }

    override fun onDraw(cv: Canvas) {
        cv.drawRect(0f, 0f, width.toFloat(), height.toFloat(), bg)
        val cx = width / 2f; val cy = height / 2f
        val rMax = ranges[rangeIdx]
        val pxPerM = (min(cx, cy) - 10 * d) / rMax
        val step = if (rMax <= 2f) 0.5f else 1f
        var k = step
        label.textAlign = Paint.Align.LEFT
        while (k <= rMax + 0.01f) {
            cv.drawCircle(cx, cy, k * pxPerM, ring)
            if ((k / step).toInt() % (if (rMax > 4f) 2 else 1) == 0) cv.drawText("%.1f m".format(k).replace(".0 m", " m"), cx + 3 * d, cy - k * pxPerM + 11 * d, label)
            k += step
        }
        cv.drawLine(cx, cy - rMax * pxPerM, cx, cy + rMax * pxPerM, ring)
        cv.drawLine(cx - rMax * pxPerM, cy, cx + rMax * pxPerM, cy, ring)

        for (w in wedges) {
            val r = min(w.radius, rMax) * pxPerM
            val oval = RectF(cx - r, cy - r, cx + r, cy + r)
            // Android arcs: 0 deg = +x (right), clockwise. Ours: 0 = up, positive = left.
            val start = -90f - w.toDeg
            cv.drawArc(oval, start, w.toDeg - w.fromDeg, true, if (w.hot) hotFill else coolFill)
        }

        val s = scan
        val fresh = s != null && SystemClock.elapsedRealtime() - s.timeMs < 2000
        if (s != null && fresh) {
            val (a, r) = s.points(minRange)
            val rr = 2.2f * d
            for (i in a.indices) {
                if (r[i] > rMax) continue
                val (x, y) = xy(cx, cy, pxPerM, a[i], r[i])
                cv.drawCircle(x, y, if (r[i] < 0.6f) rr * 1.4f else rr, if (r[i] < 0.6f) near else dot)
            }
        }
        target?.let { (a, m) ->
            if (m <= rMax) { val (x, y) = xy(cx, cy, pxPerM, a, m); cv.drawCircle(x, y, 7 * d, targetPaint) }
        }

        // the car, front up
        val cw = 9 * d; val ch = 15 * d
        cv.drawRoundRect(RectF(cx - cw, cy - ch, cx + cw, cy + ch), 3 * d, 3 * d, car)
        val nose = Path().apply { moveTo(cx, cy - ch - 7 * d); lineTo(cx - 6 * d, cy - ch - 1 * d); lineTo(cx + 6 * d, cy - ch - 1 * d); close() }
        cv.drawPath(nose, car)

        infoPaint.textAlign = Paint.Align.LEFT
        cv.drawText(if (s == null) "No scan yet: needs PRX1_TYPE 19 and firmware that sends the LiDAR scan"
            else if (!fresh) "Scan is old: LiDAR stopped?" else info, 8 * d, height - 8 * d, infoPaint)
        infoPaint.textAlign = Paint.Align.RIGHT
        cv.drawText("${rMax.toInt()} m · tap to zoom", width - 8 * d, 14 * d, infoPaint)
    }
}

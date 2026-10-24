package org.ardupilot.esp32gcs

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.os.SystemClock
import android.view.View
import kotlin.math.min

/**
 * The LiDAR proximity view (Mission Planner's Ctrl-F → Proximity): the 8 sectors
 * ArduPilot reports as DISTANCE_SENSOR, vehicle nose up, closer = redder.
 */
class RadarView(context: Context) : View(context) {
    var sectors = FloatArray(8)
    var times = LongArray(8)
    var maxRange = 8f

    private val d = resources.displayMetrics.density
    private val c = { id: Int -> context.getColor(id) }
    private val ring = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeWidth = d; color = c(R.color.line)
    }
    private val label = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = c(R.color.muted); textSize = 10 * d }
    private val fill = Paint(Paint.ANTI_ALIAS_FLAG)
    private val car = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = c(R.color.accent) }
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = c(R.color.fg); textSize = 11 * d; textAlign = Paint.Align.CENTER; isFakeBoldText = true
    }

    override fun onMeasure(w: Int, h: Int) {
        val s = min(MeasureSpec.getSize(w), (260 * d).toInt())
        setMeasuredDimension(MeasureSpec.getSize(w), s)
    }

    override fun onDraw(cv: Canvas) {
        val cx = width / 2f; val cy = height / 2f
        val r = min(cx, cy) - 14 * d
        val now = SystemClock.elapsedRealtime()
        for (k in 1..4) cv.drawCircle(cx, cy, r * k / 4, ring)
        label.textAlign = Paint.Align.LEFT
        for (k in 1..4) cv.drawText("%.0f m".format(maxRange * k / 4), cx + 3 * d, cy - r * k / 4 + 11 * d, label)
        for (i in 0 until 8) {
            val dist = sectors[i]
            if (now - times[i] > 3000 || dist <= 0f) continue
            val rr = r * (dist / maxRange).coerceIn(0.05f, 1f)
            val t = (dist / 2f).coerceIn(0f, 1f)      // red under ~0.5 m, green from 2 m
            fill.color = Color.argb(200, (239 - 205 * t).toInt(), (68 + 129 * t).toInt(), (68 + 26 * t).toInt())
            fill.style = Paint.Style.STROKE; fill.strokeWidth = 6 * d
            // Sector i is centred on i*45° clockwise from the nose; Android angles start at +x.
            val start = i * 45f - 90f - 22.5f
            cv.drawArc(RectF(cx - rr, cy - rr, cx + rr, cy + rr), start + 2, 41f, false, fill)
            val a = Math.toRadians((i * 45 - 90).toDouble())
            val tr = rr + 12 * d
            cv.drawText("%.1f".format(dist), cx + (tr * kotlin.math.cos(a)).toFloat(),
                cy + (tr * kotlin.math.sin(a)).toFloat() + 4 * d, text)
        }
        cv.drawRect(cx - 7 * d, cy - 12 * d, cx + 7 * d, cy + 12 * d, car)
        cv.drawText("▲", cx, cy - 15 * d, text)
    }
}

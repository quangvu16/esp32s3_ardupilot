package org.ardupilot.esp32gcs

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.view.View
import kotlin.math.min

/**
 * Mission Planner style HUD: artificial horizon (roll, pitch), heading tape,
 * ground speed, mode / armed state, and the GPS, battery and waypoint lines.
 */
class HudView(context: Context) : View(context) {
    var roll = 0f              // degrees
    var pitch = 0f
    var heading = 0f
    var speed = 0f
    var throttle = 0
    var mode = "—"
    var armed = false
    var connected = false
    var gps = ""
    var battery = ""
    var wp = ""
    var ekfOk = true

    private val d = resources.displayMetrics.density
    private val sky = Paint().apply { color = Color.rgb(46, 116, 196) }
    private val ground = Paint().apply { color = Color.rgb(122, 86, 44) }
    private val white = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE; strokeWidth = 1.5f * d; textSize = 11 * d
    }
    private val big = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE; textSize = 15 * d; isFakeBoldText = true
        setShadowLayer(2 * d, 0f, 0f, Color.BLACK)
    }
    private val small = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE; textSize = 12 * d; setShadowLayer(2 * d, 0f, 0f, Color.BLACK)
    }
    private val plane = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(250, 204, 21); strokeWidth = 3 * d; style = Paint.Style.STROKE
    }
    private val tapeBg = Paint().apply { color = Color.argb(110, 0, 0, 0) }
    private val armPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { textSize = 15 * d; isFakeBoldText = true }

    override fun onMeasure(w: Int, h: Int) {
        val width = MeasureSpec.getSize(w)
        setMeasuredDimension(width, min((width * 0.62f).toInt(), (260 * d).toInt()))
    }

    override fun onDraw(c: Canvas) {
        val w = width.toFloat(); val h = height.toFloat()
        val cx = w / 2; val cy = h / 2
        val pxPerDeg = h / 50f

        // Horizon
        c.save()
        c.clipRect(0f, 0f, w, h)
        c.rotate(-roll, cx, cy)
        val off = pitch * pxPerDeg
        val ext = w + h      // past every edge at any roll
        c.drawRect(-ext, -ext, ext * 2, cy + off, sky)
        c.drawRect(-ext, cy + off, ext * 2, ext * 2, ground)
        c.drawLine(-ext, cy + off, ext * 2, cy + off, white)
        white.textAlign = Paint.Align.LEFT
        for (deg in -30..30 step 10) {
            if (deg == 0) continue
            val y = cy + off - deg * pxPerDeg
            val half = if (deg % 20 == 0) w * 0.12f else w * 0.07f
            c.drawLine(cx - half, y, cx + half, y, white)
            c.drawText("$deg", cx + half + 4 * d, y + 4 * d, white)
        }
        c.restore()

        // Roll scale
        c.save()
        val r = h * 0.42f
        for (a in intArrayOf(-45, -30, -20, -10, 0, 10, 20, 30, 45)) {
            c.save(); c.rotate(a.toFloat(), cx, cy)
            c.drawLine(cx, cy - r, cx, cy - r - (if (a % 30 == 0) 9 else 5) * d, white)
            c.restore()
        }
        c.rotate(-roll, cx, cy)
        val tri = Path().apply {
            moveTo(cx, cy - r + 1 * d); lineTo(cx - 6 * d, cy - r + 11 * d); lineTo(cx + 6 * d, cy - r + 11 * d); close()
        }
        c.drawPath(tri, plane.apply { style = Paint.Style.FILL })
        plane.style = Paint.Style.STROKE
        c.restore()

        // Vehicle symbol
        c.drawLine(cx - 50 * d, cy, cx - 16 * d, cy, plane)
        c.drawLine(cx - 16 * d, cy, cx - 8 * d, cy + 8 * d, plane)
        c.drawLine(cx - 8 * d, cy + 8 * d, cx, cy, plane)
        c.drawLine(cx, cy, cx + 8 * d, cy + 8 * d, plane)
        c.drawLine(cx + 8 * d, cy + 8 * d, cx + 16 * d, cy, plane)
        c.drawLine(cx + 16 * d, cy, cx + 50 * d, cy, plane)

        // Heading tape
        val tapeH = 26 * d
        c.drawRect(0f, 0f, w, tapeH, tapeBg)
        val pxPerHdg = w / 90f
        white.textAlign = Paint.Align.CENTER
        val base = heading.toInt() - 50
        for (k in base..base + 100) {
            if (k % 5 != 0) continue
            val x = cx + (k - heading) * pxPerHdg
            val hdg = ((k % 360) + 360) % 360
            val tall = hdg % 15 == 0
            c.drawLine(x, tapeH, x, tapeH - (if (tall) 8 else 4) * d, white)
            if (hdg % 45 == 0) {
                val lbl = when (hdg) { 0 -> "N"; 45 -> "NE"; 90 -> "E"; 135 -> "SE"; 180 -> "S"; 225 -> "SW"; 270 -> "W"; 315 -> "NW"; else -> "" }
                c.drawText(lbl, x, 12 * d, white)
            } else if (hdg % 15 == 0) c.drawText("$hdg", x, 12 * d, white)
        }
        big.textAlign = Paint.Align.CENTER
        c.drawRect(cx - 24 * d, tapeH, cx + 24 * d, tapeH + 20 * d, tapeBg)
        c.drawText("%03d°".format(((heading.toInt() % 360) + 360) % 360), cx, tapeH + 16 * d, big)

        // Left: speed and throttle. Right: mode and armed state.
        small.textAlign = Paint.Align.LEFT
        big.textAlign = Paint.Align.LEFT
        c.drawText("%.1f m/s".format(speed), 8 * d, cy - 4 * d, big)
        c.drawText("thr $throttle%", 8 * d, cy + 14 * d, small)
        big.textAlign = Paint.Align.RIGHT
        c.drawText(mode, w - 8 * d, cy - 4 * d, big)
        armPaint.textAlign = Paint.Align.RIGHT
        armPaint.color = if (!connected) Color.LTGRAY else if (armed) Color.rgb(248, 113, 113) else Color.rgb(134, 239, 172)
        armPaint.setShadowLayer(2 * d, 0f, 0f, Color.BLACK)
        c.drawText(if (!connected) "NO LINK" else if (armed) "ARMED" else "DISARMED", w - 8 * d, cy + 16 * d, armPaint)
        if (!ekfOk && connected) {
            armPaint.color = Color.rgb(250, 204, 21)
            c.drawText("EKF", w - 8 * d, cy + 34 * d, armPaint)
        }

        // Bottom lines
        small.textAlign = Paint.Align.LEFT
        c.drawRect(0f, h - 34 * d, w, h, tapeBg)
        c.drawText(gps, 8 * d, h - 20 * d, small)
        c.drawText(wp, 8 * d, h - 5 * d, small)
        small.textAlign = Paint.Align.RIGHT
        c.drawText(battery, w - 8 * d, h - 20 * d, small)
    }
}

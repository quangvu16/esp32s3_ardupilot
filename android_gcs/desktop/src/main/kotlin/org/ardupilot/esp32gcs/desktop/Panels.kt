package org.ardupilot.esp32gcs.desktop

import org.ardupilot.esp32gcs.LidarScan
import org.ardupilot.esp32gcs.Platform
import java.awt.BasicStroke
import java.awt.Color
import java.awt.Dimension
import java.awt.Font
import java.awt.Graphics
import java.awt.Graphics2D
import java.awt.RenderingHints
import java.awt.event.MouseAdapter
import java.awt.event.MouseEvent
import java.awt.geom.Arc2D
import java.awt.geom.Ellipse2D
import java.awt.geom.Path2D
import java.awt.geom.RoundRectangle2D
import javax.swing.JPanel
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

private fun Graphics2D.smooth() = setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)

/** The Android HudView: horizon, heading tape, speed, mode and armed state, GPS / battery / waypoint lines. */
class HudPanel : JPanel() {
    var roll = 0f; var pitch = 0f; var heading = 0f; var speed = 0f; var throttle = 0
    var mode = "—"; var armed = false; var connected = false
    var gps = ""; var battery = ""; var wp = ""

    init { preferredSize = Dimension(360, 230); minimumSize = Dimension(240, 160) }

    override fun paintComponent(g0: Graphics) {
        val g = g0 as Graphics2D; g.smooth()
        val w = width.toDouble(); val h = height.toDouble(); val cx = w / 2; val cy = h / 2
        val ppd = h / 50
        val t = g.transform
        g.rotate(Math.toRadians(-roll.toDouble()), cx, cy)
        val off = pitch * ppd; val ext = w + h
        g.color = Color(46, 116, 196); g.fill(java.awt.geom.Rectangle2D.Double(-ext, -ext, 3 * ext, ext + cy + off))
        g.color = Color(122, 86, 44); g.fill(java.awt.geom.Rectangle2D.Double(-ext, cy + off, 3 * ext, 2 * ext))
        g.color = Color.WHITE; g.stroke = BasicStroke(1.2f)
        g.drawLine((-ext).toInt(), (cy + off).toInt(), (2 * ext).toInt(), (cy + off).toInt())
        g.font = Font(Font.SANS_SERIF, Font.PLAIN, 10)
        for (deg in -30..30 step 10) {
            if (deg == 0) continue
            val y = cy + off - deg * ppd; val half = if (deg % 20 == 0) w * 0.12 else w * 0.07
            g.drawLine((cx - half).toInt(), y.toInt(), (cx + half).toInt(), y.toInt())
            g.drawString("$deg", (cx + half + 4).toFloat(), (y + 4).toFloat())
        }
        g.transform = t
        val yellow = Color(250, 204, 21)
        g.color = yellow; g.stroke = BasicStroke(3f)
        val sym = Path2D.Double().apply {
            moveTo(cx - 45, cy); lineTo(cx - 14, cy); lineTo(cx - 7, cy + 7); lineTo(cx, cy)
            lineTo(cx + 7, cy + 7); lineTo(cx + 14, cy); lineTo(cx + 45, cy)
        }
        g.draw(sym)
        // heading tape
        val tape = 24.0
        g.color = Color(0, 0, 0, 110); g.fill(java.awt.geom.Rectangle2D.Double(0.0, 0.0, w, tape))
        val pph = w / 90
        g.color = Color.WHITE; g.stroke = BasicStroke(1f)
        val base = heading.toInt() - 50
        for (k in base..base + 100) {
            if (k % 5 != 0) continue
            val x = cx + (k - heading) * pph; val hd = ((k % 360) + 360) % 360
            g.drawLine(x.toInt(), tape.toInt(), x.toInt(), (tape - if (hd % 15 == 0) 7 else 3).toInt())
            val lbl = mapOf(0 to "N", 45 to "NE", 90 to "E", 135 to "SE", 180 to "S", 225 to "SW", 270 to "W", 315 to "NW")[hd]
                ?: if (hd % 15 == 0) "$hd" else null
            lbl?.let { s -> g.drawString(s, (x - g.fontMetrics.stringWidth(s) / 2).toFloat(), 12f) }
        }
        g.color = Color(0, 0, 0, 110); g.fillRect((cx - 24).toInt(), tape.toInt(), 48, 19)
        g.color = Color.WHITE; g.font = Font(Font.SANS_SERIF, Font.BOLD, 13)
        val hs = "%03d°".format(((heading.toInt() % 360) + 360) % 360)
        g.drawString(hs, (cx - g.fontMetrics.stringWidth(hs) / 2).toFloat(), (tape + 15).toFloat())
        g.font = Font(Font.SANS_SERIF, Font.BOLD, 15)
        g.drawString("%.1f m/s".format(speed), 8f, (cy - 4).toFloat())
        g.font = Font(Font.SANS_SERIF, Font.PLAIN, 12); g.drawString("thr $throttle%", 8f, (cy + 13).toFloat())
        g.font = Font(Font.SANS_SERIF, Font.BOLD, 15)
        g.drawString(mode, (w - 8 - g.fontMetrics.stringWidth(mode)).toFloat(), (cy - 4).toFloat())
        val arm = if (!connected) "NO LINK" else if (armed) "ARMED" else "DISARMED"
        g.color = if (!connected) Color.LIGHT_GRAY else if (armed) Color(248, 113, 113) else Color(134, 239, 172)
        g.drawString(arm, (w - 8 - g.fontMetrics.stringWidth(arm)).toFloat(), (cy + 15).toFloat())
        g.color = Color(0, 0, 0, 110); g.fill(java.awt.geom.Rectangle2D.Double(0.0, h - 34, w, 34.0))
        g.color = Color.WHITE; g.font = Font(Font.SANS_SERIF, Font.PLAIN, 12)
        g.drawString(gps, 8f, (h - 19).toFloat()); g.drawString(wp, 8f, (h - 4).toFloat())
        g.drawString(battery, (w - 8 - g.fontMetrics.stringWidth(battery)).toFloat(), (h - 19).toFloat())
    }
}

/** The Android LidarView: the scan from above, car in the middle and front up, with the behaviour's sectors. */
class LidarPanel : JPanel() {
    class Wedge(val fromDeg: Float, val toDeg: Float, val radius: Float, val hot: Boolean)

    var scan: LidarScan? = null
    var minRange = 0.3f
    var wedges: List<Wedge> = emptyList()
    var target: Pair<Float, Float>? = null
    var info = ""
    private val ranges = floatArrayOf(2f, 4f, 8f)
    private var rangeIdx = 1

    init {
        preferredSize = Dimension(460, 460)
        background = Color.WHITE
        toolTipText = "Click to change the range shown"
        addMouseListener(object : MouseAdapter() {
            override fun mouseClicked(e: MouseEvent) { rangeIdx = (rangeIdx + 1) % ranges.size; repaint() }
        })
    }

    private fun xy(cx: Double, cy: Double, ppm: Double, deg: Float, m: Float): Pair<Double, Double> {
        val a = Math.toRadians(deg.toDouble())
        return Pair(cx - m * ppm * sin(a), cy - m * ppm * cos(a))
    }

    override fun paintComponent(g0: Graphics) {
        val g = g0 as Graphics2D; g.smooth()
        g.color = Color.WHITE; g.fillRect(0, 0, width, height)
        val cx = width / 2.0; val cy = height / 2.0
        val rMax = ranges[rangeIdx]
        val ppm = (min(cx, cy) - 12) / rMax
        val line = Color(0xCF, 0xD7, 0xD1); val muted = Color(0x5A, 0x68, 0x61)
        g.stroke = BasicStroke(1f); g.font = Font(Font.SANS_SERIF, Font.PLAIN, 11)
        val step = if (rMax <= 2f) 0.5f else 1f
        var k = step
        while (k <= rMax + 0.01f) {
            val r = k * ppm
            g.color = line; g.draw(Ellipse2D.Double(cx - r, cy - r, 2 * r, 2 * r))
            if ((k / step).toInt() % (if (rMax > 4f) 2 else 1) == 0) { g.color = muted; g.drawString("%.1f m".format(k).replace(".0 m", " m"), (cx + 3).toFloat(), (cy - r + 12).toFloat()) }
            k += step
        }
        g.color = line
        g.drawLine(cx.toInt(), (cy - rMax * ppm).toInt(), cx.toInt(), (cy + rMax * ppm).toInt())
        g.drawLine((cx - rMax * ppm).toInt(), cy.toInt(), (cx + rMax * ppm).toInt(), cy.toInt())
        for (w in wedges) {
            val r = min(w.radius, rMax) * ppm
            // Java2D arcs: 0 deg = +x, counter-clockwise. Ours: 0 = up, positive = left: up is 90.
            g.color = if (w.hot) Color(239, 68, 68, 70) else Color(34, 197, 94, 45)
            g.fill(Arc2D.Double(cx - r, cy - r, 2 * r, 2 * r, 90.0 + w.fromDeg, (w.toDeg - w.fromDeg).toDouble(), Arc2D.PIE))
        }
        val s = scan
        val fresh = s != null && Platform.clock() - s.timeMs < 2000
        if (s != null && fresh) {
            val (a, r) = s.points(minRange)
            for (i in a.indices) {
                if (r[i] > rMax) continue
                val (x, y) = xy(cx, cy, ppm, a[i], r[i])
                val close = r[i] < 0.6f; val d = if (close) 7.0 else 5.0
                g.color = if (close) Color(239, 68, 68) else Color(0x17, 0x20, 0x1C)
                g.fill(Ellipse2D.Double(x - d / 2, y - d / 2, d, d))
            }
        }
        target?.let { (a, m) ->
            if (m <= rMax) { val (x, y) = xy(cx, cy, ppm, a, m); g.color = Color(59, 130, 246); g.stroke = BasicStroke(2.5f); g.draw(Ellipse2D.Double(x - 9, y - 9, 18.0, 18.0)) }
        }
        g.color = Color(0x1F, 0x5F, 0xD1)
        g.fill(RoundRectangle2D.Double(cx - 9, cy - 15, 18.0, 30.0, 5.0, 5.0))
        g.fill(Path2D.Double().apply { moveTo(cx, cy - 22); lineTo(cx - 6, cy - 16); lineTo(cx + 6, cy - 16); closePath() })
        g.color = muted; g.font = Font(Font.SANS_SERIF, Font.PLAIN, 12)
        g.drawString(if (s == null) "No scan yet: needs PRX1_TYPE 19 and firmware v4.6.3-esp32s3-3 or newer"
            else if (!fresh) "Scan is old: LiDAR stopped?" else info, 8f, (height - 8).toFloat())
        val z = "${rMax.toInt()} m · click to zoom"
        g.drawString(z, (width - 8 - g.fontMetrics.stringWidth(z)).toFloat(), 16f)
    }
}

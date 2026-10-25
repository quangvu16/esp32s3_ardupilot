package org.ardupilot.esp32gcs.desktop

import org.ardupilot.esp32gcs.nav.IndoorNav
import org.ardupilot.esp32gcs.nav.Pose2
import org.ardupilot.esp32gcs.nav.Viewport
import java.awt.BasicStroke
import java.awt.Color
import java.awt.Font
import java.awt.Graphics
import java.awt.Graphics2D
import java.awt.RenderingHints
import java.awt.event.MouseAdapter
import java.awt.event.MouseEvent
import java.awt.event.MouseWheelEvent
import java.awt.geom.Path2D
import java.awt.image.BufferedImage
import javax.swing.JPanel
import javax.swing.SwingUtilities
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.pow

/**
 * The Indoor tab's map (as the Android app's IndoorView): drag to pan, wheel to
 * zoom, click for [onTap], right-drag to set the car's pose (press where it is,
 * release the way it faces), double-click to follow the car again.
 */
class IndoorPanel(private val nav: IndoorNav) : JPanel() {
    var onTap: ((Double, Double) -> Unit)? = null
    var onPose: ((Double, Double, Double) -> Unit)? = null

    val vp = Viewport()
    private var image: BufferedImage? = null
    private var imageKey = -1L
    private var fitted = false
    private var dragFrom: IntArray? = null
    private var poseStart: IntArray? = null
    private var poseEnd: IntArray? = null

    init {
        background = Color(217, 222, 217)
        val m = object : MouseAdapter() {
            override fun mousePressed(e: MouseEvent) {
                if (SwingUtilities.isRightMouseButton(e)) { poseStart = intArrayOf(e.x, e.y); poseEnd = null }
                else dragFrom = intArrayOf(e.x, e.y)
            }
            override fun mouseDragged(e: MouseEvent) {
                if (poseStart != null) { poseEnd = intArrayOf(e.x, e.y); repaint(); return }
                val f = dragFrom ?: return
                vp.pan((f[0] - e.x).toDouble(), (f[1] - e.y).toDouble())
                dragFrom = intArrayOf(e.x, e.y); repaint()
            }
            override fun mouseReleased(e: MouseEvent) {
                val ps = poseStart
                if (ps != null) {
                    if (hypot((e.x - ps[0]).toDouble(), (e.y - ps[1]).toDouble()) > 12) {
                        val yaw = atan2((e.x - ps[0]).toDouble(), -(e.y - ps[1]).toDouble())
                        onPose?.invoke(vp.mapX(ps[0].toDouble(), ps[1].toDouble()), vp.mapY(ps[0].toDouble(), ps[1].toDouble()), yaw)
                    }
                    poseStart = null; poseEnd = null; repaint()
                }
                dragFrom = null
            }
            override fun mouseClicked(e: MouseEvent) {
                if (!SwingUtilities.isLeftMouseButton(e)) return
                if (e.clickCount == 2) { vp.follow = true; repaint(); return }
                onTap?.invoke(vp.mapX(e.x.toDouble(), e.y.toDouble()), vp.mapY(e.x.toDouble(), e.y.toDouble()))
            }
            override fun mouseWheelMoved(e: MouseWheelEvent) {
                vp.zoom(1.15.pow(-e.preciseWheelRotation), e.x.toDouble(), e.y.toDouble()); vp.follow = false; repaint()
            }
        }
        addMouseListener(m); addMouseMotionListener(m); addMouseWheelListener(m)
    }

    fun refit() { fitted = false; vp.follow = true; repaint() }

    override fun paintComponent(g0: Graphics) {
        super.paintComponent(g0)
        val g = g0 as Graphics2D
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)
        vp.width = width.toDouble(); vp.height = height.toDouble()
        val grid = nav.grid
        if (grid == null) {
            g.color = Color(90, 104, 97); g.font = Font(Font.SANS_SERIF, Font.PLAIN, 14)
            g.drawString("No map: Map new, or Load map", 16, 28); return
        }
        val car = nav.pose
        if (!fitted) { vp.fit(grid); fitted = true }
        if (vp.follow && car != null) vp.centreOn(car)

        val cm = nav.costmap
        val key = nav.gridVersion.toLong() * 1_000_003L + (cm?.version ?: 0)
        if (image == null || imageKey != key || image!!.width != grid.width) {
            val img = BufferedImage(grid.width, grid.height, BufferedImage.TYPE_INT_ARGB)
            img.setRGB(0, 0, grid.width, grid.height, Viewport.pixels(grid, cm?.snapshot()), 0, grid.width)
            image = img; imageKey = key
        }
        val left = vp.sx(0.0, grid.originY); val top = vp.sy(grid.originX + grid.height * grid.res, 0.0)
        val size = grid.res * vp.scale
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_NEAREST_NEIGHBOR)
        g.drawImage(image, left.toInt(), top.toInt(), (grid.width * size).toInt(), (grid.height * size).toInt(), null)

        nav.localizer?.takeIf { !it.confident }?.let { loc ->
            val (xs, ys, _) = loc.particles()
            g.color = Color(31, 95, 209, 110)
            for (i in xs.indices step 3) g.fillRect(vp.sx(xs[i], ys[i]).toInt() - 1, vp.sy(xs[i], ys[i]).toInt() - 1, 3, 3)
        }
        val sp = nav.scanPoints
        g.color = Color(220, 38, 38)
        for (i in sp[0].indices) g.fillOval(vp.sx(sp[0][i], sp[1][i]).toInt() - 2, vp.sy(sp[0][i], sp[1][i]).toInt() - 2, 4, 4)
        val nv = nav.navigator
        val path = nv?.path ?: emptyList()
        if (path.size > 1) {
            val p = Path2D.Double()
            path.forEachIndexed { k, pt -> if (k == 0) p.moveTo(vp.sx(pt[0], pt[1]), vp.sy(pt[0], pt[1])) else p.lineTo(vp.sx(pt[0], pt[1]), vp.sy(pt[0], pt[1])) }
            g.color = Color(31, 95, 209); g.stroke = BasicStroke(3f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND); g.draw(p)
        }
        g.font = Font(Font.SANS_SERIF, Font.BOLD, 12)
        for (w in nav.waypoints) {
            val x = vp.sx(w.x, w.y).toInt(); val y = vp.sy(w.x, w.y).toInt()
            g.color = Color(217, 119, 6); g.fillOval(x - 6, y - 6, 12, 12)
            g.color = Color(23, 32, 28); g.drawString(w.name, x + 8, y - 6)
        }
        nv?.goal?.takeIf { nv.active }?.let { gl ->
            g.color = Color(47, 125, 79); g.stroke = BasicStroke(3f)
            g.drawOval(vp.sx(gl.x, gl.y).toInt() - 11, vp.sy(gl.x, gl.y).toInt() - 11, 22, 22)
        }
        if (car != null) drawCar(g, car)
        val ps = poseStart
        if (ps != null) {
            g.color = Color(124, 58, 237); g.stroke = BasicStroke(3f)
            g.drawOval(ps[0] - 10, ps[1] - 10, 20, 20)
            poseEnd?.let { pe -> g.drawLine(ps[0], ps[1], pe[0], pe[1]) }
            g.drawString("Release the way the car faces", ps[0] + 14, ps[1] - 14)
        }
        g.color = Color(23, 32, 28); g.stroke = BasicStroke(2f)
        g.drawLine(12, height - 14, 12 + vp.scale.toInt(), height - 14)
        g.font = Font(Font.SANS_SERIF, Font.PLAIN, 11); g.drawString("1 m", 12, height - 20)
        g.drawString("drag: pan · wheel: zoom · click: go / add · right-drag: set pose · double-click: follow the car", 70, height - 8)
    }

    private fun drawCar(g: Graphics2D, p: Pose2) {
        val t = vp.carTriangle(p, maxOf(0.35, 22 / vp.scale))
        val path = Path2D.Double().apply { moveTo(t[0], t[1]); lineTo(t[2], t[3]); lineTo(t[4], t[5]); closePath() }
        g.color = Color(31, 95, 209); g.fill(path)
        g.color = Color.WHITE; g.stroke = BasicStroke(1.5f); g.draw(path)
    }
}

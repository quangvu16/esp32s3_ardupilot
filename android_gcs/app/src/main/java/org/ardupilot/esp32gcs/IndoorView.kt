package org.ardupilot.esp32gcs

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.view.GestureDetector
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.View
import org.ardupilot.esp32gcs.nav.IndoorNav
import org.ardupilot.esp32gcs.nav.Pose2
import org.ardupilot.esp32gcs.nav.Viewport
import kotlin.math.atan2
import kotlin.math.hypot

/**
 * The Indoor screen's map: the saved (or growing) map with the costmap's safety
 * margin tinted, the LiDAR scan, the planned path, the waypoints and the car.
 * Drag to pan, pinch to zoom, tap for [onTap], and press-and-drag to set the
 * car's pose (where you press, facing where you drag) for [onPose].
 */
class IndoorView(context: Context, private val nav: IndoorNav) : View(context) {
    var onTap: ((Double, Double) -> Unit)? = null
    var onPose: ((Double, Double, Double) -> Unit)? = null
    var showParticles = true

    val vp = Viewport()
    private val d = resources.displayMetrics.density
    private var bitmap: Bitmap? = null
    private var bitmapKey = -1L
    private var fitted = false

    private val bg = Paint().apply { color = Color.rgb(217, 222, 217) }
    private val mapPaint = Paint().apply { isFilterBitmap = false }
    private val scanPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(220, 38, 38) }
    private val pathPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeWidth = 3 * d; color = Color.rgb(31, 95, 209); strokeCap = Paint.Cap.ROUND
    }
    private val carPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(31, 95, 209) }
    private val carEdge = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 1.5f * d; color = Color.WHITE }
    private val wpPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(217, 119, 6) }
    private val goalPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 3 * d; color = Color.rgb(47, 125, 79) }
    private val label = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(23, 32, 28); textSize = 12 * d; isFakeBoldText = true }
    private val partPaint = Paint().apply { color = Color.argb(110, 31, 95, 209) }
    private val posePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 3 * d; color = Color.rgb(124, 58, 237) }
    private val hint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(90, 104, 97); textSize = 13 * d }

    // press-and-drag pose
    private var poseStart: FloatArray? = null
    private var poseEnd: FloatArray? = null

    private val scaler = ScaleGestureDetector(context, object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
        override fun onScale(s: ScaleGestureDetector): Boolean {
            vp.zoom(s.scaleFactor.toDouble(), s.focusX.toDouble(), s.focusY.toDouble()); vp.follow = false; invalidate(); return true
        }
    })

    private val gestures = GestureDetector(context, object : GestureDetector.SimpleOnGestureListener() {
        override fun onDown(e: MotionEvent) = true
        override fun onScroll(e1: MotionEvent?, e2: MotionEvent, dx: Float, dy: Float): Boolean {
            if (poseStart != null) return true
            vp.pan(dx.toDouble(), dy.toDouble()); invalidate(); return true
        }
        override fun onSingleTapConfirmed(e: MotionEvent): Boolean {
            onTap?.invoke(vp.mapX(e.x.toDouble(), e.y.toDouble()), vp.mapY(e.x.toDouble(), e.y.toDouble())); return true
        }
        override fun onDoubleTap(e: MotionEvent): Boolean { vp.follow = true; invalidate(); return true }
        override fun onLongPress(e: MotionEvent) {
            poseStart = floatArrayOf(e.x, e.y); poseEnd = null
            performHapticFeedback(android.view.HapticFeedbackConstants.LONG_PRESS)
            invalidate()
        }
    })

    override fun onTouchEvent(e: MotionEvent): Boolean {
        parent?.requestDisallowInterceptTouchEvent(true)
        val ps = poseStart
        if (ps != null) {
            when (e.actionMasked) {
                MotionEvent.ACTION_MOVE -> { poseEnd = floatArrayOf(e.x, e.y); invalidate() }
                MotionEvent.ACTION_UP -> {
                    val pe = poseEnd
                    if (pe != null && hypot(pe[0] - ps[0], pe[1] - ps[1]) > 15 * d) {
                        val x = vp.mapX(ps[0].toDouble(), ps[1].toDouble()); val y = vp.mapY(ps[0].toDouble(), ps[1].toDouble())
                        // screen up = north, right = east; yaw clockwise from north
                        val yaw = atan2((pe[0] - ps[0]).toDouble(), -(pe[1] - ps[1]).toDouble())
                        onPose?.invoke(x, y, yaw)
                    }
                    poseStart = null; poseEnd = null; invalidate()
                }
                MotionEvent.ACTION_CANCEL -> { poseStart = null; poseEnd = null; invalidate() }
            }
            return true
        }
        scaler.onTouchEvent(e)
        if (!scaler.isInProgress) gestures.onTouchEvent(e)
        return true
    }

    override fun onDraw(c: Canvas) {
        vp.width = width.toDouble(); vp.height = height.toDouble()
        c.drawRect(0f, 0f, width.toFloat(), height.toFloat(), bg)
        val g = nav.grid
        if (g == null) {
            c.drawText("No map: Map new, or Load", 16 * d, 28 * d, hint)
            return
        }
        val car = nav.pose
        if (!fitted) { vp.fit(g); fitted = true }
        if (vp.follow && car != null) vp.centreOn(car)

        // the map, re-coloured when it or the costmap changes
        val cm = nav.costmap
        val key = nav.gridVersion.toLong() * 1_000_003L + (cm?.version ?: 0)
        if (bitmap == null || bitmapKey != key || bitmap!!.width != g.width) {
            val px = Viewport.pixels(g, cm?.snapshot())
            bitmap = Bitmap.createBitmap(px, g.width, g.height, Bitmap.Config.ARGB_8888)
            bitmapKey = key
        }
        val left = vp.sx(0.0, g.originY).toFloat()
        val top = vp.sy(g.originX + g.height * g.res, 0.0).toFloat()
        val size = (g.res * vp.scale).toFloat()
        c.drawBitmap(bitmap!!, null, RectF(left, top, left + g.width * size, top + g.height * size), mapPaint)

        // particles
        if (showParticles) nav.localizer?.let { loc ->
            if (!loc.confident) {
                val (xs, ys, _) = loc.particles()
                for (i in xs.indices step 3) c.drawCircle(vp.sx(xs[i], ys[i]).toFloat(), vp.sy(xs[i], ys[i]).toFloat(), 1.6f * d, partPaint)
            }
        }
        // scan
        val (sxs, sys) = nav.scanPoints.let { it[0] to it[1] }
        for (i in sxs.indices) c.drawCircle(vp.sx(sxs[i], sys[i]).toFloat(), vp.sy(sxs[i], sys[i]).toFloat(), 1.8f * d, scanPaint)
        // path
        val nv = nav.navigator
        val path = nv?.path ?: emptyList()
        if (path.size > 1) {
            val p = Path()
            path.forEachIndexed { k, pt ->
                val x = vp.sx(pt[0], pt[1]).toFloat(); val y = vp.sy(pt[0], pt[1]).toFloat()
                if (k == 0) p.moveTo(x, y) else p.lineTo(x, y)
            }
            c.drawPath(p, pathPaint)
        }
        // waypoints and the current goal
        for (w in nav.waypoints) {
            val x = vp.sx(w.x, w.y).toFloat(); val y = vp.sy(w.x, w.y).toFloat()
            c.drawCircle(x, y, 6 * d, wpPaint)
            c.drawText(w.name, x + 8 * d, y - 6 * d, label)
        }
        nv?.goal?.takeIf { nv.active }?.let { gl ->
            c.drawCircle(vp.sx(gl.x, gl.y).toFloat(), vp.sy(gl.x, gl.y).toFloat(), 11 * d, goalPaint)
        }
        // car
        if (car != null) drawCar(c, car)
        // pose being set
        val ps = poseStart
        if (ps != null) {
            c.drawCircle(ps[0], ps[1], 10 * d, posePaint)
            poseEnd?.let { pe -> c.drawLine(ps[0], ps[1], pe[0], pe[1], posePaint) }
            c.drawText("Drag the way the car faces", ps[0] + 14 * d, ps[1] - 14 * d, label)
        }
        // scale bar
        val oneM = vp.scale.toFloat()
        c.drawLine(12 * d, height - 14 * d, 12 * d + oneM, height - 14 * d, carEdge.apply { color = Color.rgb(23, 32, 28) })
        c.drawText("1 m", 12 * d, height - 20 * d, hint)
        carEdge.color = Color.WHITE
    }

    private fun drawCar(c: Canvas, p: Pose2) {
        val t = vp.carTriangle(p, maxOf(0.35, 22 * d / vp.scale))
        val path = Path().apply {
            moveTo(t[0].toFloat(), t[1].toFloat()); lineTo(t[2].toFloat(), t[3].toFloat()); lineTo(t[4].toFloat(), t[5].toFloat()); close()
        }
        c.drawPath(path, carPaint); c.drawPath(path, carEdge)
    }

    /** Start over on a new or loaded map: fit it to the screen again. */
    fun refit() { fitted = false; vp.follow = true; invalidate() }
}

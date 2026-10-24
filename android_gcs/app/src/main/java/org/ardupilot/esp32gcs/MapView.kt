package org.ardupilot.esp32gcs

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.Rect
import android.graphics.RectF
import android.util.LruCache
import android.view.GestureDetector
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.View
import java.io.File
import java.net.HttpURLConnection
import java.net.URL
import java.util.concurrent.Executors
import kotlin.math.PI
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.ln
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.tan

/**
 * Slippy map (OpenStreetMap or Esri satellite tiles, cached on disk so a field
 * viewed once at home still shows without internet) with the vehicle, home,
 * its trail and the mission drawn on top. Waypoints can be dragged when
 * [editable].
 */
class MapView(context: Context) : View(context) {
    enum class Source(val label: String, val url: String, val credit: String) {
        STREET("Map", "https://tile.openstreetmap.org/%d/%d/%d.png", "© OpenStreetMap contributors"),
        SATELLITE("Satellite",
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/%d/%3\$d/%2\$d",
            "Imagery © Esri, Maxar, Earthstar Geographics"),
    }

    var source = Source.STREET
        set(v) { field = v; invalidate() }

    var centerLat = 21.0285        // Hanoi until the vehicle has a position
    var centerLon = 105.8542
    var zoom = 5.0
    var follow = true
    var editable = false

    // Overlays, set by the activity.
    var vehicle: Triple<Double, Double, Float>? = null      // lat, lon, heading (deg)
    var home: Pair<Double, Double>? = null
    var guidedTarget: Pair<Double, Double>? = null
    var mission: List<MissionItem> = emptyList()
    var currentWp = -1                                      // index into mission (vehicle seq - 1)
    private val trail = ArrayList<Pair<Double, Double>>()

    var onTap: ((Double, Double) -> Unit)? = null
    var onLongPress: ((Double, Double) -> Unit)? = null
    var onWaypointMoved: ((Int, Double, Double) -> Unit)? = null
    var onWaypointTap: ((Int) -> Unit)? = null

    private val density = resources.displayMetrics.density
    private val tilePx = 256f * density / 1.5f        // on-screen size of one tile at an integer zoom
    private val c = { id: Int -> context.getColor(id) }

    private val memory = object : LruCache<String, Bitmap>(80) {}
    private val loading = HashSet<String>()
    private val failed = HashMap<String, Long>()
    private val pool = Executors.newFixedThreadPool(3)
    private val diskDir = File(context.cacheDir, "tiles")

    private val tilePaint = Paint(Paint.FILTER_BITMAP_FLAG)
    private val blank = Paint().apply { color = c(R.color.sunk) }
    private val routePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeWidth = 3 * density; color = Color.rgb(250, 204, 21)
    }
    private val trailPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeWidth = 2 * density; color = Color.argb(200, 236, 72, 153)
    }
    private val wpFill = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(250, 204, 21) }
    private val wpCurrent = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(34, 197, 94) }
    private val wpText = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.BLACK; textSize = 11 * density; textAlign = Paint.Align.CENTER; isFakeBoldText = true
    }
    private val outline = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeWidth = 1.5f * density; color = Color.BLACK
    }
    private val homeFill = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(239, 68, 68) }
    private val homeText = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE; textSize = 11 * density; textAlign = Paint.Align.CENTER; isFakeBoldText = true
    }
    private val vehicleFill = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(239, 68, 68) }
    private val vehicleRim = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeWidth = 2 * density; color = Color.WHITE
    }
    private val targetPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeWidth = 3 * density; color = Color.rgb(59, 130, 246)
    }
    private val credit = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(220, 20, 20, 20); textSize = 9 * density; textAlign = Paint.Align.RIGHT
    }
    private val creditBg = Paint().apply { color = Color.argb(170, 255, 255, 255) }
    private val scalePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.BLACK; strokeWidth = 2 * density; textSize = 10 * density
    }

    // --- projection: world pixels at the current zoom ----------------------------------
    private fun worldSize() = tilePx * 2.0.pow(zoom)
    private fun lonToX(lon: Double) = (lon + 180.0) / 360.0 * worldSize()
    private fun latToY(lat: Double): Double {
        val r = Math.toRadians(lat.coerceIn(-85.0, 85.0))
        return (1 - ln(tan(r) + 1 / cos(r)) / PI) / 2 * worldSize()
    }
    private fun xToLon(x: Double) = x / worldSize() * 360.0 - 180.0
    private fun yToLat(y: Double) = Math.toDegrees(atan(kotlin.math.sinh(PI * (1 - 2 * y / worldSize()))))

    fun screen(lat: Double, lon: Double): Pair<Float, Float> =
        Pair((lonToX(lon) - lonToX(centerLon) + width / 2).toFloat(),
            (latToY(lat) - latToY(centerLat) + height / 2).toFloat())

    fun geo(sx: Float, sy: Float): Pair<Double, Double> =
        Pair(yToLat(latToY(centerLat) + sy - height / 2), xToLon(lonToX(centerLon) + sx - width / 2))

    fun centerOn(lat: Double, lon: Double, z: Double? = null) {
        centerLat = lat; centerLon = lon
        if (z != null) zoom = z
        invalidate()
    }

    fun addTrail(lat: Double, lon: Double) {
        val last = trail.lastOrNull()
        if (last != null && distanceM(last.first, last.second, lat, lon) < 0.5) return
        trail.add(Pair(lat, lon))
        if (trail.size > 2000) trail.subList(0, 500).clear()
    }

    fun clearTrail() { trail.clear(); invalidate() }

    /** Fits the mission, home and vehicle in the view. */
    fun fit() {
        val pts = ArrayList<Pair<Double, Double>>()
        mission.filter { it.hasPosition }.forEach { pts.add(Pair(it.lat, it.lon)) }
        home?.let { pts.add(it) }
        vehicle?.let { pts.add(Pair(it.first, it.second)) }
        if (pts.isEmpty() || width == 0) return
        val minLat = pts.minOf { it.first }; val maxLat = pts.maxOf { it.first }
        val minLon = pts.minOf { it.second }; val maxLon = pts.maxOf { it.second }
        centerLat = (minLat + maxLat) / 2; centerLon = (minLon + maxLon) / 2
        zoom = 19.0
        while (zoom > 3) {
            val (x1, y1) = screen(maxLat, minLon); val (x2, y2) = screen(minLat, maxLon)
            if (x2 - x1 < width * 0.8 && y2 - y1 < height * 0.8) break
            zoom -= 0.5
        }
        follow = false
        invalidate()
    }

    // --- gestures ---------------------------------------------------------------
    // -1: none; >= 0: dragging that waypoint; <= -2: finger down on waypoint (-dragWp - 2), not moved yet
    private var dragWp = -1

    private val scaler = ScaleGestureDetector(context, object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
        override fun onScale(d: ScaleGestureDetector): Boolean {
            val (lat, lon) = geo(d.focusX, d.focusY)
            zoom = (zoom + ln(d.scaleFactor.toDouble()) / ln(2.0)).coerceIn(2.0, 21.0)
            // Keep the point under the fingers still.
            val (sx, sy) = screen(lat, lon)
            val (cl, co) = geo(width / 2f + (sx - d.focusX), height / 2f + (sy - d.focusY))
            centerLat = cl; centerLon = co
            invalidate()
            return true
        }
    })

    private val gestures = GestureDetector(context, object : GestureDetector.SimpleOnGestureListener() {
        override fun onDown(e: MotionEvent) = true
        override fun onScroll(e1: MotionEvent?, e2: MotionEvent, dx: Float, dy: Float): Boolean {
            if (dragWp != -1) return true      // a finger on a waypoint drags it, not the map
            follow = false
            val (lat, lon) = geo(width / 2f + dx, height / 2f + dy)
            centerLat = lat; centerLon = lon
            invalidate()
            return true
        }
        override fun onSingleTapConfirmed(e: MotionEvent): Boolean {
            val wp = hitWaypoint(e.x, e.y)
            if (wp >= 0 && onWaypointTap != null) { onWaypointTap?.invoke(wp); return true }
            val (lat, lon) = geo(e.x, e.y)
            onTap?.invoke(lat, lon)
            return true
        }
        override fun onDoubleTap(e: MotionEvent): Boolean {
            val (lat, lon) = geo(e.x, e.y)
            zoom = (zoom + 1).coerceAtMost(21.0)
            centerLat = lat; centerLon = lon; follow = false
            invalidate()
            return true
        }
        override fun onLongPress(e: MotionEvent) {
            if (dragWp != -1) return
            val (lat, lon) = geo(e.x, e.y)
            onLongPress?.invoke(lat, lon)
        }
    })

    private fun hitWaypoint(x: Float, y: Float): Int {
        var best = -1; var bestD = 22 * density
        mission.forEachIndexed { i, it ->
            if (!it.hasPosition) return@forEachIndexed
            val (sx, sy) = screen(it.lat, it.lon)
            val d = hypot(sx - x, sy - y)
            if (d < bestD) { best = i; bestD = d }
        }
        return best
    }

    override fun onTouchEvent(e: MotionEvent): Boolean {
        parent?.requestDisallowInterceptTouchEvent(true)
        if (editable) when (e.actionMasked) {
            MotionEvent.ACTION_DOWN -> dragWp = hitWaypoint(e.x, e.y).let { if (it >= 0) -it - 2 else -1 }
            MotionEvent.ACTION_MOVE -> if (dragWp != -1 && e.pointerCount == 1) {
                // Start dragging only after the finger really moves (taps still select).
                val i = if (dragWp >= 0) dragWp else -dragWp - 2
                val (sx, sy) = screen(mission[i].lat, mission[i].lon)
                if (dragWp >= 0 || hypot(sx - e.x, sy - e.y) > 12 * density) {
                    dragWp = i
                    val (lat, lon) = geo(e.x, e.y)
                    mission[i].lat = lat; mission[i].lon = lon
                    invalidate()
                    return true
                }
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                if (dragWp >= 0) {
                    val i = dragWp; dragWp = -1
                    onWaypointMoved?.invoke(i, mission[i].lat, mission[i].lon)
                    return true
                }
                dragWp = -1
            }
        }
        scaler.onTouchEvent(e)
        if (!scaler.isInProgress) gestures.onTouchEvent(e)
        return true
    }

    // --- drawing ----------------------------------------------------------------------
    override fun onDraw(canvas: Canvas) {
        if (follow) vehicle?.let { centerLat = it.first; centerLon = it.second }
        drawTiles(canvas)

        if (trail.size > 1) {
            val p = Path()
            trail.forEachIndexed { i, (la, lo) ->
                val (x, y) = screen(la, lo)
                if (i == 0) p.moveTo(x, y) else p.lineTo(x, y)
            }
            canvas.drawPath(p, trailPaint)
        }

        val pts = mission.withIndex().filter { it.value.hasPosition }
        if (pts.isNotEmpty()) {
            val p = Path()
            home?.let { (la, lo) -> val (x, y) = screen(la, lo); p.moveTo(x, y) }
            pts.forEachIndexed { k, (_, it) ->
                val (x, y) = screen(it.lat, it.lon)
                if (k == 0 && home == null) p.moveTo(x, y) else p.lineTo(x, y)
            }
            canvas.drawPath(p, routePaint)
            val r = 11 * density
            for ((i, it) in pts) {
                val (x, y) = screen(it.lat, it.lon)
                canvas.drawCircle(x, y, r, if (i == currentWp) wpCurrent else wpFill)
                canvas.drawCircle(x, y, r, outline)
                canvas.drawText("${i + 1}", x, y + wpText.textSize * 0.36f, wpText)
            }
        }

        home?.let { (la, lo) ->
            val (x, y) = screen(la, lo)
            val r = 10 * density
            canvas.drawRect(x - r, y - r, x + r, y + r, homeFill)
            canvas.drawRect(x - r, y - r, x + r, y + r, outline)
            canvas.drawText("H", x, y + homeText.textSize * 0.36f, homeText)
        }

        guidedTarget?.let { (la, lo) ->
            val (x, y) = screen(la, lo)
            val r = 12 * density
            canvas.drawCircle(x, y, r, targetPaint)
            canvas.drawLine(x - r * 1.5f, y, x + r * 1.5f, y, targetPaint)
            canvas.drawLine(x, y - r * 1.5f, x, y + r * 1.5f, targetPaint)
        }

        vehicle?.let { (la, lo, hdg) ->
            val (x, y) = screen(la, lo)
            canvas.save()
            canvas.rotate(hdg, x, y)
            val s = 14 * density
            val p = Path().apply {
                moveTo(x, y - s); lineTo(x + s * 0.7f, y + s * 0.8f); lineTo(x, y + s * 0.4f)
                lineTo(x - s * 0.7f, y + s * 0.8f); close()
            }
            canvas.drawPath(p, vehicleFill)
            canvas.drawPath(p, vehicleRim)
            canvas.restore()
        }

        drawScale(canvas)
        val t = source.credit
        val w = credit.measureText(t)
        canvas.drawRect(width - w - 8 * density, height - 14 * density, width.toFloat(), height.toFloat(), creditBg)
        canvas.drawText(t, width - 4 * density, height - 4 * density, credit)
    }

    private fun drawScale(canvas: Canvas) {
        val mPerPx = 156543.03392 * cos(Math.toRadians(centerLat)) / 2.0.pow(zoom) * (256 / tilePx)
        val target = 100 * density * mPerPx
        val nice = listOf(1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0,
            10000.0, 20000.0, 50000.0, 100000.0, 200000.0, 500000.0, 1e6).lastOrNull { it <= target } ?: 1.0
        val px = (nice / mPerPx).toFloat()
        val x0 = 12 * density; val y0 = height - 22 * density
        canvas.drawRect(x0 - 3 * density, y0 - 14 * density, x0 + px + 3 * density, y0 + 5 * density, creditBg)
        canvas.drawLine(x0, y0, x0 + px, y0, scalePaint)
        canvas.drawText(if (nice >= 1000) "${(nice / 1000).toInt()} km" else "${nice.toInt()} m", x0, y0 - 4 * density, scalePaint)
    }

    private fun drawTiles(canvas: Canvas) {
        canvas.drawRect(0f, 0f, width.toFloat(), height.toFloat(), blank)
        val z = floor(zoom).toInt().coerceIn(0, 19)
        val size = (tilePx * 2.0.pow(zoom - z)).toFloat()
        val n = 1 shl z
        val cx = lonToX(centerLon); val cy = latToY(centerLat)
        val left = cx - width / 2; val top = cy - height / 2
        val tx0 = floor(left / size).toInt(); val ty0 = floor(top / size).toInt()
        val tx1 = floor((left + width) / size).toInt(); val ty1 = floor((top + height) / size).toInt()
        val dst = RectF()
        for (ty in ty0..ty1) {
            if (ty < 0 || ty >= n) continue
            for (txr in tx0..tx1) {
                val tx = ((txr % n) + n) % n
                dst.set((txr * size - left).toFloat(), (ty * size - top).toFloat(),
                    (txr * size - left).toFloat() + size, (ty * size - top).toFloat() + size)
                val bmp = tile(z, tx, ty)
                if (bmp != null) { canvas.drawBitmap(bmp, null, dst, tilePaint); continue }
                // Not loaded yet: stretch a parent tile that is.
                for (up in 1..4) {
                    val pz = z - up
                    if (pz < 0) break
                    val pb = memory.get(key(pz, tx shr up, ty shr up)) ?: continue
                    val part = pb.width shr up
                    val sx = (tx and ((1 shl up) - 1)) * part
                    val sy = (ty and ((1 shl up) - 1)) * part
                    canvas.drawBitmap(pb, Rect(sx, sy, sx + part, sy + part), dst, tilePaint)
                    break
                }
            }
        }
    }

    private fun key(z: Int, x: Int, y: Int) = "${source.name}/$z/$x/$y"

    private fun tile(z: Int, x: Int, y: Int): Bitmap? {
        val k = key(z, x, y)
        memory.get(k)?.let { return it }
        synchronized(loading) {
            if (k in loading) return null
            val f = failed[k]
            if (f != null && System.currentTimeMillis() - f < 30000) return null
            loading.add(k)
        }
        val src = source
        pool.execute {
            var bmp: Bitmap? = null
            try {
                val file = File(diskDir, "$k.png")
                if (file.exists()) bmp = BitmapFactory.decodeFile(file.path)
                if (bmp == null) {
                    val url = URL(src.url.format(z, x, y))
                    val net = internetNetwork(context)
                    val conn = (net?.openConnection(url) ?: url.openConnection()) as HttpURLConnection
                    conn.connectTimeout = 6000; conn.readTimeout = 8000
                    conn.setRequestProperty("User-Agent", "Esp32RoverGcs/1.0 (ArduPilot ESP32-S3 ground station)")
                    val bytes = conn.inputStream.use { it.readBytes() }
                    bmp = BitmapFactory.decodeByteArray(bytes, 0, bytes.size)
                    if (bmp != null) {
                        file.parentFile?.mkdirs()
                        file.writeBytes(bytes)
                    }
                }
            } catch (_: Exception) {}
            synchronized(loading) {
                loading.remove(k)
                if (bmp == null) failed[k] = System.currentTimeMillis()
            }
            if (bmp != null) { memory.put(k, bmp); postInvalidate() }
        }
        return null
    }

    companion object {
        fun distanceM(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
            val r = 6371000.0
            val dLat = Math.toRadians(lat2 - lat1); val dLon = Math.toRadians(lon2 - lon1)
            val a = sin(dLat / 2).pow(2) + cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) * sin(dLon / 2).pow(2)
            return 2 * r * kotlin.math.asin(kotlin.math.sqrt(a))
        }
    }
}

package org.ardupilot.esp32gcs.desktop

import org.ardupilot.esp32gcs.FenceZone
import org.ardupilot.esp32gcs.LidarScan
import org.ardupilot.esp32gcs.MissionItem
import org.ardupilot.esp32gcs.Platform
import java.awt.BasicStroke
import java.awt.Color
import java.awt.Font
import java.awt.Graphics
import java.awt.Graphics2D
import java.awt.RenderingHints
import java.awt.event.MouseAdapter
import java.awt.event.MouseEvent
import java.awt.event.MouseWheelEvent
import java.awt.geom.Ellipse2D
import java.awt.geom.Path2D
import java.awt.image.BufferedImage
import java.io.File
import java.net.HttpURLConnection
import java.net.URL
import java.util.concurrent.Executors
import javax.imageio.ImageIO
import javax.swing.JPanel
import javax.swing.SwingUtilities
import kotlin.math.PI
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.ln
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sinh
import kotlin.math.tan

/**
 * The Android MapView for the desktop: OpenStreetMap or Esri satellite tiles
 * (cached on disk), the car, its trail, home, the mission, fences, zig-zag A/B
 * and the LiDAR scan. Drag to pan, wheel to zoom; waypoints and fence corners
 * can be dragged when [editable].
 */
class MapPanel : JPanel() {
    enum class Source(val label: String, val url: String, val credit: String) {
        STREET("Map", "https://tile.openstreetmap.org/%d/%d/%d.png", "© OpenStreetMap contributors"),
        SATELLITE("Satellite", "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/%d/%3\$d/%2\$d",
            "Imagery © Esri, Maxar, Earthstar Geographics"),
    }

    var source = Source.STREET
        set(v) { field = v; repaint() }
    var centerLat = 21.0285
    var centerLon = 105.8542
    var zoom = 5.0
    var follow = true
    var editable = false

    var vehicle: Triple<Double, Double, Float>? = null       // lat, lon, heading (deg)
    var home: Pair<Double, Double>? = null
    var guidedTarget: Pair<Double, Double>? = null
    var mission: List<MissionItem> = emptyList()
    var currentWp = -1
    var fences: List<FenceZone> = emptyList()
    var fenceEditing = false
    var selectedZone = -1
    var scan: LidarScan? = null
    var zigA: DoubleArray? = null
    var zigB: DoubleArray? = null
    private val trail = ArrayList<Pair<Double, Double>>()

    var onClick: ((Double, Double) -> Unit)? = null
    var onMenu: ((Double, Double, Int, Int) -> Unit)? = null          // right click: lat, lon, screen x, y
    var onWaypointClick: ((Int) -> Unit)? = null
    var onFenceClick: ((zone: Int, point: Int) -> Unit)? = null
    var onEdited: (() -> Unit)? = null

    private val tilePx = 256.0
    private val memory = object : LinkedHashMap<String, BufferedImage>(128, 0.75f, true) {
        override fun removeEldestEntry(e: MutableMap.MutableEntry<String, BufferedImage>?) = size > 160
    }
    private val loading = HashSet<String>()
    private val failed = HashMap<String, Long>()
    private val pool = Executors.newFixedThreadPool(4) { Thread(it, "tiles").apply { isDaemon = true } }
    private val diskDir = File(System.getProperty("user.home"), ".esp32gcs/tiles")

    // --- projection --------------------------------------------------------------------
    private fun worldSize() = tilePx * 2.0.pow(zoom)
    private fun lonToX(lon: Double) = (lon + 180.0) / 360.0 * worldSize()
    private fun latToY(lat: Double): Double {
        val r = Math.toRadians(lat.coerceIn(-85.0, 85.0))
        return (1 - ln(tan(r) + 1 / cos(r)) / PI) / 2 * worldSize()
    }
    private fun xToLon(x: Double) = x / worldSize() * 360.0 - 180.0
    private fun yToLat(y: Double) = Math.toDegrees(atan(sinh(PI * (1 - 2 * y / worldSize()))))

    fun screen(lat: Double, lon: Double) = Pair(lonToX(lon) - lonToX(centerLon) + width / 2.0, latToY(lat) - latToY(centerLat) + height / 2.0)
    fun geo(sx: Double, sy: Double) = Pair(yToLat(latToY(centerLat) + sy - height / 2.0), xToLon(lonToX(centerLon) + sx - width / 2.0))
    private fun metersPerPixel() = 156543.03392 * cos(Math.toRadians(centerLat)) / 2.0.pow(zoom) * (256 / tilePx)

    fun centerOn(lat: Double, lon: Double, z: Double? = null) { centerLat = lat; centerLon = lon; if (z != null) zoom = z; repaint() }

    fun addTrail(lat: Double, lon: Double) {
        val last = trail.lastOrNull()
        if (last != null && hypot((last.first - lat) * 111320, (last.second - lon) * 111320 * cos(Math.toRadians(lat))) < 0.5) return
        trail.add(Pair(lat, lon))
        if (trail.size > 3000) trail.subList(0, 800).clear()
    }

    fun clearTrail() { trail.clear(); repaint() }

    fun fit() {
        val pts = ArrayList<Pair<Double, Double>>()
        mission.filter { it.hasPosition }.forEach { pts.add(Pair(it.lat, it.lon)) }
        fences.forEach { z -> z.points.forEach { pts.add(Pair(it[0], it[1])) } }
        home?.let { pts.add(it) }; vehicle?.let { pts.add(Pair(it.first, it.second)) }
        zigA?.let { pts.add(Pair(it[0], it[1])) }; zigB?.let { pts.add(Pair(it[0], it[1])) }
        if (pts.isEmpty() || width == 0) return
        centerLat = (pts.minOf { it.first } + pts.maxOf { it.first }) / 2
        centerLon = (pts.minOf { it.second } + pts.maxOf { it.second }) / 2
        zoom = 20.0
        while (zoom > 3) {
            val (x1, y1) = screen(pts.maxOf { it.first }, pts.minOf { it.second })
            val (x2, y2) = screen(pts.minOf { it.first }, pts.maxOf { it.second })
            if (x2 - x1 < width * 0.8 && y2 - y1 < height * 0.8) break
            zoom -= 0.25
        }
        follow = false
        repaint()
    }

    // --- mouse ---------------------------------------------------------------------------
    private class Handle(val zone: Int, val point: Int, val get: () -> DoubleArray, val set: (Double, Double) -> Unit)

    private fun handles(): List<Handle> {
        val out = ArrayList<Handle>()
        if (fenceEditing) fences.forEachIndexed { zi, z ->
            z.points.forEachIndexed { pi, p -> out += Handle(zi, pi, { p }, { la, lo -> p[0] = la; p[1] = lo }) }
        } else mission.forEachIndexed { i, it ->
            if (it.hasPosition) out += Handle(-1, i, { doubleArrayOf(it.lat, it.lon) }, { la, lo -> it.lat = la; it.lon = lo })
        }
        return out
    }

    private fun hit(x: Int, y: Int): Handle? {
        var best: Handle? = null; var bestD = 12.0
        for (h in handles()) {
            val p = h.get(); val (sx, sy) = screen(p[0], p[1])
            val d = hypot(sx - x, sy - y)
            if (d < bestD) { best = h; bestD = d }
        }
        return best
    }

    init {
        isOpaque = true
        val m = object : MouseAdapter() {
            var lastX = 0; var lastY = 0; var drag: Handle? = null; var moved = false
            override fun mousePressed(e: MouseEvent) {
                requestFocusInWindow()
                lastX = e.x; lastY = e.y; moved = false
                drag = if (editable && SwingUtilities.isLeftMouseButton(e)) hit(e.x, e.y) else null
                if (e.isPopupTrigger) menu(e)
            }
            override fun mouseDragged(e: MouseEvent) {
                if (Math.abs(e.x - lastX) + Math.abs(e.y - lastY) > 3) moved = true
                val h = drag
                if (h != null) {
                    val (la, lo) = geo(e.x.toDouble(), e.y.toDouble()); h.set(la, lo); repaint(); return
                }
                if (!moved) return
                follow = false
                val (la, lo) = geo(width / 2.0 - (e.x - lastX), height / 2.0 - (e.y - lastY))
                centerLat = la; centerLon = lo; lastX = e.x; lastY = e.y
                repaint()
            }
            override fun mouseReleased(e: MouseEvent) {
                if (e.isPopupTrigger) { menu(e); return }
                val h = drag; drag = null
                if (h != null && moved) { onEdited?.invoke(); return }
                if (moved || !SwingUtilities.isLeftMouseButton(e)) return
                if (h != null && h.zone < 0) { onWaypointClick?.invoke(h.point); return }
                if (h != null && h.zone >= 0) { onFenceClick?.invoke(h.zone, h.point); return }
                val (la, lo) = geo(e.x.toDouble(), e.y.toDouble())
                onClick?.invoke(la, lo)
            }
            fun menu(e: MouseEvent) {
                val (la, lo) = geo(e.x.toDouble(), e.y.toDouble())
                onMenu?.invoke(la, lo, e.x, e.y)
            }
            override fun mouseWheelMoved(e: MouseWheelEvent) {
                val (la, lo) = geo(e.x.toDouble(), e.y.toDouble())
                zoom = (zoom - e.preciseWheelRotation * 0.5).coerceIn(2.0, 21.0)
                val (sx, sy) = screen(la, lo)
                val (cl, co) = geo(width / 2.0 + (sx - e.x), height / 2.0 + (sy - e.y))
                centerLat = cl; centerLon = co; follow = false
                repaint()
            }
        }
        addMouseListener(m); addMouseMotionListener(m); addMouseWheelListener(m)
    }

    // --- drawing ---------------------------------------------------------------------------
    override fun paintComponent(g0: Graphics) {
        val g = g0 as Graphics2D
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR)
        if (follow) vehicle?.let { centerLat = it.first; centerLon = it.second }
        drawTiles(g)

        if (trail.size > 1) {
            val p = Path2D.Double()
            trail.forEachIndexed { i, (la, lo) -> val (x, y) = screen(la, lo); if (i == 0) p.moveTo(x, y) else p.lineTo(x, y) }
            g.color = Color(236, 72, 153, 200); g.stroke = BasicStroke(2f); g.draw(p)
        }
        drawFences(g)
        drawScan(g)

        val pts = mission.withIndex().filter { it.value.hasPosition }
        if (pts.isNotEmpty()) {
            val p = Path2D.Double()
            home?.let { (la, lo) -> val (x, y) = screen(la, lo); p.moveTo(x, y) }
            pts.forEachIndexed { k, (_, it) -> val (x, y) = screen(it.lat, it.lon); if (k == 0 && home == null) p.moveTo(x, y) else p.lineTo(x, y) }
            g.color = Color(250, 204, 21); g.stroke = BasicStroke(3f); g.draw(p)
            g.font = Font(Font.SANS_SERIF, Font.BOLD, 11)
            for ((i, it) in pts) {
                val (x, y) = screen(it.lat, it.lon)
                circle(g, x, y, 10.0, if (i == currentWp) Color(34, 197, 94) else Color(250, 204, 21), Color.BLACK)
                centered(g, "${i + 1}", x, y, Color.BLACK)
            }
        }
        drawAB(g)
        home?.let { (la, lo) ->
            val (x, y) = screen(la, lo)
            g.color = Color(239, 68, 68); g.fillRect((x - 9).toInt(), (y - 9).toInt(), 18, 18)
            g.color = Color.BLACK; g.stroke = BasicStroke(1.5f); g.drawRect((x - 9).toInt(), (y - 9).toInt(), 18, 18)
            g.font = Font(Font.SANS_SERIF, Font.BOLD, 11); centered(g, "H", x, y, Color.WHITE)
        }
        guidedTarget?.let { (la, lo) ->
            val (x, y) = screen(la, lo)
            g.color = Color(59, 130, 246); g.stroke = BasicStroke(3f)
            g.draw(Ellipse2D.Double(x - 11, y - 11, 22.0, 22.0))
            g.drawLine((x - 17).toInt(), y.toInt(), (x + 17).toInt(), y.toInt()); g.drawLine(x.toInt(), (y - 17).toInt(), x.toInt(), (y + 17).toInt())
        }
        vehicle?.let { (la, lo, hdg) ->
            val (x, y) = screen(la, lo)
            val s = 13.0
            val p = Path2D.Double().apply {
                moveTo(0.0, -s); lineTo(s * 0.7, s * 0.8); lineTo(0.0, s * 0.4); lineTo(-s * 0.7, s * 0.8); closePath()
            }
            val t = g.transform
            g.translate(x, y); g.rotate(Math.toRadians(hdg.toDouble()))
            g.color = Color(239, 68, 68); g.fill(p); g.color = Color.WHITE; g.stroke = BasicStroke(2f); g.draw(p)
            g.transform = t
        }
        drawScale(g)
        val credit = source.credit
        g.font = Font(Font.SANS_SERIF, Font.PLAIN, 10)
        val w = g.fontMetrics.stringWidth(credit)
        g.color = Color(255, 255, 255, 170); g.fillRect(width - w - 8, height - 15, w + 8, 15)
        g.color = Color(20, 20, 20); g.drawString(credit, width - w - 4, height - 4)
    }

    private fun circle(g: Graphics2D, x: Double, y: Double, r: Double, fill: Color, line: Color) {
        val e = Ellipse2D.Double(x - r, y - r, 2 * r, 2 * r)
        g.color = fill; g.fill(e); g.color = line; g.stroke = BasicStroke(1.5f); g.draw(e)
    }

    private fun centered(g: Graphics2D, s: String, x: Double, y: Double, c: Color) {
        val fm = g.fontMetrics
        g.color = c; g.drawString(s, (x - fm.stringWidth(s) / 2.0).toFloat(), (y + fm.ascent * 0.38).toFloat())
    }

    private fun drawScan(g: Graphics2D) {
        val s = scan ?: return
        val (la, lo, hdg) = vehicle ?: return
        if (Platform.clock() - s.timeMs > 2000) return
        val (vx, vy) = screen(la, lo)
        val mpp = metersPerPixel()
        g.color = Color(249, 115, 22)
        for (i in s.cm.indices) {
            val d = s.cm[i]
            if (d == 0xffff || d > s.maxCm || d < s.minCm) continue
            val b = Math.toRadians((hdg + s.offsetDeg + (i + 0.5f) * s.incrementDeg).toDouble())
            val m = d / 100.0
            g.fill(Ellipse2D.Double(vx + m * sin(b) / mpp - 2.2, vy - m * cos(b) / mpp - 2.2, 4.4, 4.4))
        }
    }

    private fun drawAB(g: Graphics2D) {
        val a = zigA; val b = zigB
        val purple = Color(147, 51, 234)
        if (a != null && b != null) {
            val (x1, y1) = screen(a[0], a[1]); val (x2, y2) = screen(b[0], b[1])
            g.color = purple; g.stroke = BasicStroke(2f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10f, floatArrayOf(8f, 5f), 0f)
            g.drawLine(x1.toInt(), y1.toInt(), x2.toInt(), y2.toInt())
        }
        g.font = Font(Font.SANS_SERIF, Font.BOLD, 11)
        for ((p, label) in listOf(a to "A", b to "B")) {
            p ?: continue
            val (x, y) = screen(p[0], p[1])
            circle(g, x, y, 10.0, purple, Color.WHITE)
            centered(g, label, x, y, Color.WHITE)
        }
    }

    private fun drawFences(g: Graphics2D) {
        fences.forEachIndexed { zi, z ->
            val line = if (z.kind.inclusion) Color(22, 163, 74) else Color(220, 38, 38)
            val fill = if (z.kind.inclusion) Color(22, 163, 74, 28) else Color(220, 38, 38, 70)
            val wide = zi == selectedZone && fenceEditing
            g.stroke = BasicStroke(if (wide) 4f else 2.5f)
            when {
                z.kind.polygon && z.points.isNotEmpty() -> {
                    val p = Path2D.Double()
                    z.points.forEachIndexed { i, pt -> val (x, y) = screen(pt[0], pt[1]); if (i == 0) p.moveTo(x, y) else p.lineTo(x, y) }
                    if (z.points.size >= 3) { p.closePath(); g.color = fill; g.fill(p) }
                    g.color = line; g.draw(p)
                }
                z.isCircle && z.points.isNotEmpty() -> {
                    val (x, y) = screen(z.points[0][0], z.points[0][1])
                    val r = z.radius / metersPerPixel()
                    val e = Ellipse2D.Double(x - r, y - r, 2 * r, 2 * r)
                    g.color = fill; g.fill(e); g.color = line; g.draw(e)
                }
                z.kind == FenceZone.Kind.RETURN && z.points.isNotEmpty() -> {
                    val (x, y) = screen(z.points[0][0], z.points[0][1])
                    circle(g, x, y, 9.0, Color(239, 68, 68), Color.WHITE); g.font = Font(Font.SANS_SERIF, Font.BOLD, 11); centered(g, "R", x, y, Color.WHITE)
                }
            }
            if (fenceEditing) for (pt in z.points) {
                val (x, y) = screen(pt[0], pt[1])
                val s = if (wide) 5 else 4
                g.color = Color.WHITE; g.fillRect(x.toInt() - s, y.toInt() - s, 2 * s, 2 * s)
                g.color = line; g.stroke = BasicStroke(1.5f); g.drawRect(x.toInt() - s, y.toInt() - s, 2 * s, 2 * s)
            }
        }
    }

    private fun drawScale(g: Graphics2D) {
        val mpp = metersPerPixel()
        val target = 100 * mpp
        val nice = listOf(1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10000.0, 20000.0, 50000.0,
            100000.0, 200000.0, 500000.0, 1e6).lastOrNull { it <= target } ?: 1.0
        val px = (nice / mpp).toInt()
        val x0 = 12; val y0 = height - 22
        g.color = Color(255, 255, 255, 170); g.fillRect(x0 - 4, y0 - 15, px + 8, 22)
        g.color = Color.BLACK; g.stroke = BasicStroke(2f); g.drawLine(x0, y0, x0 + px, y0)
        g.font = Font(Font.SANS_SERIF, Font.PLAIN, 11)
        g.drawString(if (nice >= 1000) "${(nice / 1000).toInt()} km" else "${nice.toInt()} m", x0, y0 - 4)
    }

    private fun drawTiles(g: Graphics2D) {
        g.color = Color(0xE3, 0xE8, 0xE4); g.fillRect(0, 0, width, height)
        val z = floor(zoom).toInt().coerceIn(0, 19)
        val size = tilePx * 2.0.pow(zoom - z)
        val n = 1 shl z
        val left = lonToX(centerLon) - width / 2.0; val top = latToY(centerLat) - height / 2.0
        val tx0 = floor(left / size).toInt(); val ty0 = floor(top / size).toInt()
        val tx1 = floor((left + width) / size).toInt(); val ty1 = floor((top + height) / size).toInt()
        for (ty in ty0..ty1) {
            if (ty < 0 || ty >= n) continue
            for (txr in tx0..tx1) {
                val tx = ((txr % n) + n) % n
                val dx = (txr * size - left).toInt(); val dy = (ty * size - top).toInt()
                val ds = (size + 1).toInt()
                val img = tile(z, tx, ty)
                if (img != null) { g.drawImage(img, dx, dy, ds, ds, null); continue }
                for (up in 1..4) {
                    val pz = z - up
                    if (pz < 0) break
                    val pb = synchronized(memory) { memory[key(pz, tx shr up, ty shr up)] } ?: continue
                    val part = pb.width shr up
                    val sx = (tx and ((1 shl up) - 1)) * part; val sy = (ty and ((1 shl up) - 1)) * part
                    g.drawImage(pb, dx, dy, dx + ds, dy + ds, sx, sy, sx + part, sy + part, null)
                    break
                }
            }
        }
    }

    private fun key(z: Int, x: Int, y: Int) = "${source.name}/$z/$x/$y"

    private fun tile(z: Int, x: Int, y: Int): BufferedImage? {
        val k = key(z, x, y)
        synchronized(memory) { memory[k] }?.let { return it }
        synchronized(loading) {
            if (k in loading) return null
            val f = failed[k]
            if (f != null && System.currentTimeMillis() - f < 30000) return null
            loading.add(k)
        }
        val src = source
        pool.execute {
            var img: BufferedImage? = null
            try {
                val file = File(diskDir, "$k.png")
                if (file.exists()) img = ImageIO.read(file)
                if (img == null) {
                    val conn = URL(src.url.format(z, x, y)).openConnection() as HttpURLConnection
                    conn.connectTimeout = 6000; conn.readTimeout = 8000
                    conn.setRequestProperty("User-Agent", "Esp32RoverGcs-Desktop/1.3 (ArduPilot ESP32-S3 ground station)")
                    val bytes = conn.inputStream.use { it.readBytes() }
                    img = ImageIO.read(bytes.inputStream())
                    if (img != null) { file.parentFile?.mkdirs(); file.writeBytes(bytes) }
                }
            } catch (_: Exception) {}
            synchronized(loading) { loading.remove(k); if (img == null) failed[k] = System.currentTimeMillis() }
            if (img != null) { synchronized(memory) { memory[k] = img }; repaint() }
        }
        return null
    }
}

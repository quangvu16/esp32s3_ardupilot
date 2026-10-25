package org.ardupilot.esp32gcs.nav

import org.ardupilot.esp32gcs.LidarScan
import org.ardupilot.esp32gcs.Platform
import org.ardupilot.esp32gcs.Vehicle
import java.io.File
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Indoor navigation for the apps' Indoor screen, from the car's LiDAR scan and
 * pose: map a room, save it, find the car in it (AMCL), and drive to a tapped
 * goal or round a patrol, Nav2 style. Everything runs on one worker thread; the
 * screen calls the methods below from any thread and reads the @Volatile state.
 *
 * Needs the car's LiDAR SLAM (PRX1_SLAM 1, EKF on ExternalNav) for its pose, and
 * PRX1_SCAN_FULL 1 for 1° scans (5° scans work, with coarser maps).
 */
class IndoorNav(private val vehicle: Vehicle, val dir: File = File(Platform.dataDir, "maps")) {
    enum class Mode { OFF, MAPPING, NAVIGATION }

    /** Settings the screen may change. Defaults suit a 1:10 RC car with an LD14. */
    class Settings {
        var robotRadius = 0.20              // m: half the car's width + margin (costmap inscribed radius)
        var inflation = 0.55                // m
        var speed = 0.4                     // m/s
        var rotationPeriodS = Double.NaN    // s per LiDAR rotation; NaN: from PRX1_TYPE (LD14 1/6, LD06 1/10)
        var mapSizeM = 40.0
        var mapRes = 0.10
    }

    val settings = Settings()
    private val exec = Executors.newSingleThreadScheduledExecutor { Thread(it, "indoor-nav").apply { isDaemon = true } }

    @Volatile var mode = Mode.OFF
        private set
    @Volatile var status = "Off"
        private set
    @Volatile var mapName: String? = null
        private set
    /** The map drawn on screen: the saved one, or the one being built (refreshed about once a second). */
    @Volatile var grid: OccGrid? = null
        private set
    @Volatile var gridVersion = 0
        private set
    @Volatile var waypoints: List<Waypoint> = emptyList()
        private set
    @Volatile var patrol: List<String> = emptyList()
        private set
    /** The car in the map (null until known). */
    @Volatile var pose: Pose2? = null
        private set
    /** The latest scan's points in the map frame, for drawing. */
    @Volatile var scanPoints: Array<DoubleArray> = arrayOf(DoubleArray(0), DoubleArray(0))
        private set
    @Volatile var localizer: Localizer? = null
        private set
    @Volatile var costmap: Costmap? = null
        private set
    @Volatile var navigator: Navigator? = null
        private set

    private var mapper: Mapper? = null
    /** For tests: every scan added to the map being built. */
    @Volatile var onMapScan: ((OdomScan) -> Unit)? = null
    private var lastScanCount = -1
    private var lastGridMs = 0L
    private var lastScanMs = 0L

    private val driver = object : NavDriver {
        override fun sendVelocity(v: Double, yawRate: Double) = vehicle.sendVelocity(v, yawRate)
        override fun setGuided() = vehicle.setMode(MODE_GUIDED)
        override fun setHold() = vehicle.setMode(MODE_HOLD)
        override val inGuided get() = vehicle.customMode == MODE_GUIDED
        override val armed get() = vehicle.armed
    }

    init {
        exec.scheduleAtFixedRate({ safe { tick() } }, 100, 100, TimeUnit.MILLISECONDS)
    }

    // ---------------------------------------------------------------- actions

    /** Start a new map where the car is. Drive slowly round the room, then [saveMap]. */
    fun startMapping() = post {
        stopNavigation("Mapping")
        val here = odomNow() ?: Pose2()
        mapper = Mapper(settings.mapSizeM, settings.mapRes, here)
        mode = Mode.MAPPING; mapName = null
        waypoints = emptyList(); patrol = emptyList()
        localizer = null; costmap = null; navigator = null
        grid = mapper!!.snapshot(); gridVersion++
        vehicle.fastPose = true
        status = "Mapping: drive slowly round the room, then Save"
    }

    /** Save the map being built (or the loaded one, with its waypoints) and start navigating in it. */
    fun saveMap(name: String) = post {
        val g = (if (mode == Mode.MAPPING) mapper?.snapshot() else grid) ?: run { status = "No map to save"; return@post }
        MapFiles.save(dir, name, g, waypoints, patrol)
        mapName = name
        if (mode == Mode.MAPPING) {
            // the map was built in this boot's odom frame: the car is where the map says
            useMap(g)
            localizer?.setPose(odomNow() ?: Pose2(), odomNow() ?: Pose2(), 0.05, 0.03)
        }
        status = "Saved \"$name\""
    }

    fun loadMap(name: String) = post {
        stopNavigation("Loading")
        val saved = MapFiles.load(dir, name)
        mapName = name
        waypoints = saved.waypoints; patrol = saved.patrol
        useMap(saved.grid)
        // Usually the car starts on the same mark as when the map was made, so its own
        // frame lines up with the map's: start there and let AMCL fine-tune.
        val odom = odomNow()
        if (odom != null) localizer?.setPose(odom, odom)
        status = "Loaded \"$name\": " + if (odom != null) "check the car's position, or Set pose / Find me" else "waiting for the car's position"
    }

    fun deleteMap(name: String) = post { MapFiles.delete(dir, name); if (mapName == name) mapName = null }
    fun maps(): List<String> = MapFiles.list(dir)

    /** Nav2's "2D Pose Estimate": the car is here, facing [yaw], in the map. */
    fun setPose(x: Double, y: Double, yaw: Double) = post {
        val odom = odomNow() ?: run { status = "No position from the car"; return@post }
        localizer?.setPose(Pose2(x, y, yaw), odom) ?: run { status = "Load or make a map first"; return@post }
        status = "Pose set"
    }

    /** Search the whole map for the car (when it didn't start on its mark). */
    fun findMe() = post {
        val loc = localizer ?: run { status = "Load or make a map first"; return@post }
        val s = latestOdomScan() ?: run { status = "No scan yet"; return@post }
        status = if (loc.findGlobally(s)) "Searching… drive a little to confirm" else "Couldn't place the car"
    }

    fun goTo(x: Double, y: Double) = post { startGoals(listOf(Waypoint("goal", x, y)), false) }

    fun goToWaypoint(name: String) = post {
        val w = waypoints.firstOrNull { it.name == name } ?: return@post
        startGoals(listOf(w), false)
    }

    fun startPatrol() = post {
        val list = patrol.mapNotNull { n -> waypoints.firstOrNull { it.name == n } }.ifEmpty { waypoints }
        if (list.size < 2) { status = "A patrol needs 2 or more waypoints"; return@post }
        startGoals(list, true)
    }

    fun addWaypoint(x: Double, y: Double, name: String? = null) = post {
        val n = name ?: nextName()
        waypoints = waypoints.filter { it.name != n } + Waypoint(n, x, y)
        patrol = patrol + n
        saveWaypoints()
    }

    fun removeWaypoint(name: String) = post {
        waypoints = waypoints.filter { it.name != name }; patrol = patrol.filter { it != name }
        saveWaypoints()
    }

    fun clearWaypoints() = post { waypoints = emptyList(); patrol = emptyList(); saveWaypoints() }

    fun stop() = post { stopNavigation("Stopped") }

    /** Leave indoor navigation (the screen was closed). */
    fun off() = post {
        stopNavigation("Off"); mode = Mode.OFF; mapper = null; vehicle.fastPose = false; status = "Off"
    }

    fun close() { exec.shutdownNow() }

    // ---------------------------------------------------------------- internals

    private fun post(block: () -> Unit) { exec.execute { safe(block) } }

    private fun safe(block: () -> Unit) {
        try { block() } catch (e: Exception) { status = "Error: ${e.message}"; Platform.logError("IndoorNav", e) }
    }

    private fun useMap(g: OccGrid) {
        mapper = null
        grid = g; gridVersion++
        val cm = Costmap(g, Costmap.Params().apply {
            inscribedRadius = settings.robotRadius; inflationRadius = maxOf(settings.inflation, settings.robotRadius + 0.1)
        })
        costmap = cm
        localizer = Localizer(g)
        val rpp = Rpp(cm, Rpp.Params().apply {
            desiredVel = settings.speed
            vehicle.paramValue("TURN_RADIUS")?.let { if (it > 0.1f) minTurnRadius = it.toDouble(); regMinRadius = minTurnRadius }
        })
        navigator = Navigator(Planner(cm), rpp, driver).also { it.onClearObstacles = { cm.clearObstacles() } }
        mode = Mode.NAVIGATION
        vehicle.fastPose = true
    }

    private fun startGoals(list: List<Waypoint>, loop: Boolean) {
        val nav = navigator ?: run { status = "Load or make a map first"; return }
        if (!vehicle.armed) { status = "Arm the car first"; return }
        if (localizer?.confident != true) {
            // start as soon as AMCL is sure (it usually is within a few scans)
            pending = Triple(list, loop, Platform.clock() + 10_000)
            status = "Waiting to be sure where the car is…"
            return
        }
        pending = null
        nav.start(list, loop, Platform.clock())
    }

    private var pending: Triple<List<Waypoint>, Boolean, Long>? = null

    private fun stopNavigation(why: String) {
        pending = null
        navigator?.let { if (it.active) it.stop(why) }
    }

    private fun saveWaypoints() { mapName?.let { MapFiles.saveWaypoints(dir, it, waypoints, patrol) } }

    private fun nextName(): String {
        var k = 1
        while (waypoints.any { it.name == "P$k" }) k++
        return "P$k"
    }

    private fun odomNow(): Pose2? {
        val t = vehicle.poses.latestMs()
        return if (t < 0) null else vehicle.poses.at(t)
    }

    private fun periodS(): Double {
        if (!settings.rotationPeriodS.isNaN()) return settings.rotationPeriodS
        return if (vehicle.paramValue("PRX1_TYPE")?.toInt() == 16) 0.1 else 1.0 / 6
    }

    private fun latestOdomScan(): OdomScan? {
        val s = vehicle.scan ?: return null
        return toOdom(s)
    }

    /**
     * The car's own pose is usable: the EKF has its attitude and a horizontal position
     * (from the LiDAR SLAM). Before that, at start-up, the pose jumps and the heading is
     * not yet the SLAM's, which would smear the map.
     */
    val poseReady get() = vehicle.ekfFlags and 0x01 != 0 && vehicle.ekfFlags and (0x08 or 0x10) != 0 &&
        vehicle.ekfFlags and 0x400 == 0

    private fun toOdom(s: LidarScan): OdomScan? {
        if (s.vehicleTimeUs <= 0L || !poseReady) return null
        val n = s.cm.size
        val ang = FloatArray(n); val rng = FloatArray(n)
        for (i in 0 until n) {
            val d = s.cm[i]
            ang[i] = Math.toRadians(((s.offsetDeg + (i + 0.5f) * s.incrementDeg) % 360f).toDouble()).toFloat()
            rng[i] = when {
                d == 0xffff || d < s.minCm -> Float.NaN
                d > s.maxCm -> Float.POSITIVE_INFINITY
                else -> d / 100f
            }
        }
        val keep = (0 until n).filter { !rng[it].isNaN() }
        val frame = ScanFrame(s.vehicleTimeUs / 1000, FloatArray(keep.size) { ang[keep[it]] },
            FloatArray(keep.size) { rng[keep[it]] }, s.maxCm / 100f)
        val sx = vehicle.paramValue("PRX1_SLAM_X")?.toDouble() ?: 0.0
        val sy = vehicle.paramValue("PRX1_SLAM_Y")?.toDouble() ?: 0.0
        return OdomScan.build(frame, vehicle.poses, periodS(), sx, sy, minRange = 0.2, clearRange = 3.0)
    }

    private fun tick() {
        val now = Platform.clock()
        val scan = vehicle.scan
        if (scan != null && vehicle.scanCount != lastScanCount) {
            lastScanCount = vehicle.scanCount
            lastScanMs = scan.timeMs
            processScan(scan, now)
        }
        val nav = navigator
        val loc = localizer
        pending?.let { (list, loop, until) ->
            if (loc?.confident == true) startGoals(list, loop)
            else if (now > until) { pending = null; status = "Not sure where the car is: Set pose or Find me first" }
        }
        if (mode == Mode.NAVIGATION && nav != null && loc != null) {
            val odom = odomNow()
            val p = if (odom != null && loc.initialised) loc.offset + odom else null
            if (p != null) pose = p
            val vNow = vehicle.localVx * cos(vehicle.yaw.toDouble()) + vehicle.localVy * sin(vehicle.yaw.toDouble())
            nav.tick(now, p, loc.confident, if (vehicle.localTime == 0L) Long.MAX_VALUE else now - vehicle.localTime,
                if (lastScanMs == 0L) Long.MAX_VALUE else now - lastScanMs, vNow)
            if (nav.active || nav.state == Navigator.State.FAILED || nav.state == Navigator.State.DONE) status = nav.status
        } else if (mode == Mode.MAPPING) {
            pose = odomNow()
        }
    }

    private fun processScan(scan: LidarScan, now: Long) {
        val os = toOdom(scan) ?: run {
            if (mode != Mode.OFF && !poseReady) status = "Waiting for the car's position (EKF on the LiDAR SLAM)…"
            return
        }
        when (mode) {
            Mode.MAPPING -> {
                val m = mapper ?: return
                m.add(os)
                onMapScan?.invoke(os)
                scanPoints = arrayOf(os.px.copyOf(), os.py.copyOf())
                if (now - lastGridMs > 1000) { lastGridMs = now; grid = m.snapshot(); gridVersion++ }
                status = "Mapping: ${m.scans} scans. Drive slowly round the room, then Save"
            }
            Mode.NAVIGATION -> {
                val loc = localizer ?: return
                loc.update(os)
                if (!loc.initialised) return
                val inMap = os.transformed(loc.offset)
                costmap?.addScan(inMap, now)
                scanPoints = arrayOf(inMap.px.copyOf(), inMap.py.copyOf())
                val nav = navigator
                if ((nav == null || !nav.active) && pending == null) {
                    status = when {
                        loc.confident -> "Ready: tap a goal, or run the patrol"
                        else -> "Localising… (spread %.2f m, match %.0f%%)".format(loc.spreadXY, loc.score * 100)
                    }
                }
            }
            Mode.OFF -> {}
        }
    }

    companion object {
        const val MODE_HOLD = 4
        const val MODE_GUIDED = 15
        /** Heading of a pose for drawing (an arrow from x, y). */
        fun arrow(p: Pose2, len: Double) = doubleArrayOf(p.x + len * cos(p.yaw), p.y + len * sin(p.yaw))
        const val TWO_PI = 2 * PI
    }
}

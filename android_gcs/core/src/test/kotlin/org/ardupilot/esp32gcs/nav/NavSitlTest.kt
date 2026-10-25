package org.ardupilot.esp32gcs.nav

import org.ardupilot.esp32gcs.Platform
import org.ardupilot.esp32gcs.TcpLink
import org.ardupilot.esp32gcs.Vehicle
import org.ardupilot.esp32gcs.VehicleEvent
import java.io.File
import kotlin.system.exitProcess

/*
 * The apps' indoor navigation against ArduPilot SITL (Tools/lidar_slam_test/sitl_nav.py
 * starts SITL with the room and runs this): the real Vehicle and IndoorNav code over
 * MAVLink. Maps the room while driving a loop, saves the map, reboots the autopilot,
 * loads the map, then drives to B and round a patrol. Writes the app's map pose
 * against the autopilot's clock to poses.csv for the script to compare with the truth.
 */
/** Called a few times during the patrol (the Windows app's test uses it to take screenshots). */
var navSitlSnapshot: ((IndoorNav, Int) -> Unit)? = null

fun main(args: Array<String>) {
    val work = File(args.getOrElse(0) { "/tmp/navsitl" }).apply { mkdirs() }
    Platform.dataDir = work
    val v = Vehicle()
    val texts = ArrayList<String>()
    v.onEvent = { e -> if (e is VehicleEvent.Text) synchronized(texts) { texts += e.text; println("  < ${e.text}") } }
    v.connect { TcpLink("127.0.0.1", 5760) }
    fun clock() = Platform.clock()
    var navRef: IndoorNav? = null
    fun waitFor(what: String, ms: Long, cond: () -> Boolean) {
        val end = clock() + ms
        var shown = clock()
        while (!cond()) {
            if (clock() > end) { println("TIMEOUT: $what (${navRef?.status})"); exitProcess(2) }
            if (clock() - shown > 5000) { shown = clock(); navRef?.let { println("  … ${it.status}") } }
            Thread.sleep(100)
        }
        println("$what: ok")
    }
    fun sawText(s: String) = synchronized(texts) { texts.any { it.contains(s) } }

    waitFor("heartbeat", 30_000) { v.connected }
    v.fetchParams()
    waitFor("parameters", 60_000) { !v.paramsFetching && v.paramCount > 0 && v.paramValue("PRX1_SCAN_FULL") != null }
    waitFor("SLAM tracking", 60_000) { sawText("SLAM: tracking") }
    val nav = IndoorNav(v, File(work, "maps"))
    navRef = nav
    nav.settings.rotationPeriodS = 360.0 / 2152            // the simulated LD06
    nav.settings.speed = 0.5
    waitFor("pose and 1° scans", 20_000) { v.localTime > 0 && (v.scan?.cm?.size ?: 0) == 360 }

    // ---------------------------------------------------------------- mapping
    val scanLog = java.io.PrintWriter(File(work, "mapscans.csv"))
    scanLog.println("endx,endy,endyaw,n,hits,out_origin,out_end,maxlen")
    nav.onMapScan = { s ->
        var outO = 0; var outE = 0; var maxLen = 0.0; var hits = 0
        for (k in 0 until s.size) {
            if (s.hit[k]) hits++
            if (s.ox[k] !in -2.6..7.6 || s.oy[k] !in -1.9..6.3) outO++
            if (s.px[k] !in -2.8..7.8 || s.py[k] !in -2.1..6.5) outE++
            maxLen = maxOf(maxLen, Math.hypot(s.px[k] - s.ox[k], s.py[k] - s.oy[k]))
        }
        synchronized(scanLog) { scanLog.println("%.2f,%.2f,%.1f,%d,%d,%d,%d,%.2f".format(s.end.x, s.end.y, Math.toDegrees(s.end.yaw), s.size, hits, outO, outE, maxLen)) }
    }
    nav.startMapping()
    // a lap round the box in the middle, in Auto: the room exists only for the LiDAR,
    // so an open-loop drive that turns too wide would leave it through a wall
    waitFor("GPS-like position (EKF origin)", 20_000) { v.hasPosition }
    val lat0 = v.lat; val lon0 = v.lon
    fun at(n: Double, e: Double) = org.ardupilot.esp32gcs.MissionItem(
        lat = lat0 + Math.toDegrees(n / 6378137.0), lon = lon0 + Math.toDegrees(e / (6378137.0 * Math.cos(Math.toRadians(lat0)))))
    v.uploadMission(listOf(at(4.5, 0.0), at(4.5, 4.4), at(0.0, 4.4), at(0.0, 0.0)))
    Thread.sleep(500)
    waitFor("mission uploaded", 20_000) { !v.missionBusy }
    v.setMode(10); v.arm(true, force = true)
    waitFor("armed in Auto", 10_000) { v.armed && v.customMode == 10 }
    var lastPrint = 0L
    val lapEnd = clock() + 90_000
    while (clock() < lapEnd && v.customMode == 10) {
        Thread.sleep(200)
        if (clock() - lastPrint > 5000) {
            lastPrint = clock()
            println("  mapping: ${nav.status} | WP ${v.missionSeq}")
        }
    }
    v.sendVelocity(0.0, 0.0); Thread.sleep(1500)
    println("mapping: ${nav.status}")
    synchronized(scanLog) { scanLog.close() }
    nav.saveMap("sitlroom")
    Thread.sleep(500)
    nav.addWaypoint(0.0, 0.0, "A"); nav.addWaypoint(5.0, 0.0, "B"); nav.addWaypoint(5.0, 4.4, "C")
    Thread.sleep(500)
    v.setMode(4); v.arm(false, force = true)
    println("saved: " + File(work, "maps").list()!!.sorted().joinToString())

    // ---------------------------------------------------------------- new boot
    synchronized(texts) { texts.clear() }
    v.reboot()
    Thread.sleep(3000)
    waitFor("heartbeat after reboot", 60_000) { v.connected }
    waitFor("SLAM tracking after reboot", 60_000) { sawText("SLAM: tracking") }
    waitFor("pose after reboot", 20_000) { v.localTime > 0 && clock() - v.localTime < 500 }
    Thread.sleep(2000)
    nav.loadMap("sitlroom")
    waitFor("localised", 30_000) { nav.localizer?.confident == true }

    val csv = File(work, "poses.csv").printWriter()
    csv.println("boot_ms,x,y,yaw,state")
    val logger = Thread {
        while (true) {
            val p = nav.pose
            if (p != null) synchronized(csv) { csv.println("${v.vehicleBootMs},${p.x},${p.y},${p.yaw},${nav.navigator?.state}") }
            try { Thread.sleep(100) } catch (e: InterruptedException) { break }
        }
    }.apply { isDaemon = true; start() }

    v.setMode(15); v.arm(true, force = true)
    waitFor("armed", 10_000) { v.armed }
    nav.goToWaypoint("B")
    Thread.sleep(1000)
    waitFor("arrived at B", 120_000) { nav.navigator?.state == Navigator.State.DONE || nav.navigator?.state == Navigator.State.FAILED }
    println("B: ${nav.status}")
    val bOk = nav.navigator?.state == Navigator.State.DONE
    v.setMode(15)
    Thread.sleep(1000)
    nav.startPatrol()
    var laps = 0; var last = 0
    val end = clock() + 240_000
    var shots = 0; var nextShot = clock() + 8000
    while (clock() < end && laps < 1) {
        val n = nav.navigator ?: break
        if (n.state == Navigator.State.FAILED) break
        if (n.goalIndex < last) laps++
        last = n.goalIndex
        if (clock() > nextShot && shots < 4) { navSitlSnapshot?.invoke(nav, shots++); nextShot = clock() + 9000 }
        Thread.sleep(200)
    }
    println("patrol: $laps lap(s), ${nav.status}")
    nav.stop()
    Thread.sleep(1000)
    v.arm(false, force = true)
    logger.interrupt()
    synchronized(csv) { csv.close() }
    val ok = bOk && laps >= 1
    println(if (ok) "NAV PASS" else "NAV FAIL")
    exitProcess(if (ok) 0 else 1)
}

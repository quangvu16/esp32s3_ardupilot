package org.ardupilot.esp32gcs.nav

import java.io.File
import java.util.Random
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.system.exitProcess

/*
 * Offline test of the indoor navigation core: a simulated car (Ackermann, turn
 * radius 0.9 m) with a simulated LD14 in the room of SITL's SIM_ROOM=1.
 *   1. map the room from known poses, save and load it (PGM + YAML)
 *   2. a new "boot": the car's own frame is off from the map's; AMCL finds it
 *   3. drive to goals with A* + RPP, past a box that is not in the map
 *   4. a patrol of 3 waypoints, twice round
 * Run: see android_gcs/core/src/test/README.md. Exit code 0 = pass.
 */

private class Seg(val x1: Double, val y1: Double, val x2: Double, val y2: Double)

private fun box(x0: Double, y0: Double, x1: Double, y1: Double) =
    listOf(Seg(x0, y0, x1, y0), Seg(x1, y0, x1, y1), Seg(x1, y1, x0, y1), Seg(x0, y1, x0, y0))

private val ROOM = box(-2.5, -1.8, 7.5, 6.2) + box(2.0, 1.8, 2.8, 2.6) + box(6.1, -1.4, 7.1, -0.6) +
    box(-2.1, 5.0, -1.3, 5.8) + listOf(Seg(4.0, 6.2, 4.0, 5.0))

private fun cast(walls: List<Seg>, ox: Double, oy: Double, a: Double): Double {
    val dx = cos(a); val dy = sin(a)
    var best = Double.MAX_VALUE
    for (s in walls) {
        val ex = s.x2 - s.x1; val ey = s.y2 - s.y1
        val den = dx * ey - dy * ex
        if (abs(den) < 1e-12) continue
        val t = ((s.x1 - ox) * ey - (s.y1 - oy) * ex) / den
        val u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / den
        if (t > 0 && u >= 0 && u <= 1 && t < best) best = t
    }
    return best
}

/** The car, the LiDAR and the car's own (odom) frame, which is [odomFromWorld] off the world. */
private class Sim(var truth: Pose2, val odomFromWorld: Pose2, var walls: List<Seg> = ROOM) {
    val rnd = Random(7)
    val poses = PoseHistory()
    var tMs = 0L
    var v = 0.0; var w = 0.0
    val turnRadius = 0.9
    fun odom() = odomFromWorld + truth

    /** Advance 10 ms with a speed and turn-rate command, as Rover's Guided mode would. */
    fun step(cmdV: Double, cmdW: Double) {
        val dt = 0.01
        v += (cmdV - v).coerceIn(-1.0 * dt, 1.0 * dt)          // 1 m/s² acceleration
        val wMax = abs(v) / turnRadius
        w = cmdW.coerceIn(-wMax, wMax)
        truth = Pose2(truth.x + v * cos(truth.yaw) * dt, truth.y + v * sin(truth.yaw) * dt, wrapPi(truth.yaw + w * dt))
        tMs += 10
        if (tMs % 100 == 0L) {
            val o = odom()
            poses.addPosition(tMs, o.x + rnd.nextGaussian() * 0.01, o.y + rnd.nextGaussian() * 0.01)
            poses.addYaw(tMs, o.yaw + rnd.nextGaussian() * 0.005)
        }
    }

    /** One LiDAR rotation measured over the last 1/6 s (call after stepping through it). */
    val history = ArrayList<Pose2>()
    fun scan(): ScanFrame {
        val n = 360
        val ang = FloatArray(n); val rng = FloatArray(n)
        for (i in 0 until n) {
            val a = (i + 0.5) * PI / 180
            // where the car was when this degree was measured
            val back = ((1.0 - a / (2 * PI)) * 16.7).toInt().coerceIn(0, history.size - 1)
            val p = history[history.size - 1 - back]
            var r = cast(walls, p.x, p.y, p.yaw + a)
            if (r > 8.0 || rnd.nextDouble() < 0.05) r = Double.POSITIVE_INFINITY else r += rnd.nextGaussian() * 0.012
            ang[i] = a.toFloat(); rng[i] = r.toFloat()
        }
        return ScanFrame(tMs, ang, rng, 8f)
    }

    fun run(ms: Int, cmd: () -> VelCmd, onScan: (ScanFrame) -> Unit) {
        var c = cmd()
        for (k in 0 until ms / 10) {
            if (k % 10 == 0) c = cmd()
            step(c.v, c.yawRate)
            history += truth
            if (history.size > 40) history.removeAt(0)
            if (tMs % 170 == 0L && history.size >= 17) onScan(scan())
        }
    }
}

private var failures = 0
private fun check(ok: Boolean, what: String) { println((if (ok) "  ok    " else "  FAIL  ") + what); if (!ok) failures++ }

fun main() {
    val dir = File(System.getProperty("java.io.tmpdir"), "navtest").apply { deleteRecursively(); mkdirs() }

    // ------------------------------------------------------------ 1. mapping
    println("1. mapping from the car's own poses")
    val s1 = Sim(Pose2(0.0, 0.0, 0.0), Pose2())
    val mapper = Mapper(30.0, 0.1, Pose2())
    // drive a loop round the box by hand: forward, then circles
    val legs = listOf(VelCmd(0.4, 0.0) to 9000, VelCmd(0.4, 0.4 / 1.2) to 4712, VelCmd(0.4, 0.0) to 6000,
        VelCmd(0.4, 0.4 / 1.2) to 4712, VelCmd(0.4, 0.0) to 9000, VelCmd(0.4, 0.4 / 1.2) to 4712,
        VelCmd(0.4, 0.0) to 6000, VelCmd(0.4, 0.4 / 1.2) to 4712)
    for ((c, ms) in legs) s1.run(ms, { c }) { sc ->
        OdomScan.build(sc, s1.poses, 1.0 / 6, 0.0, 0.0)?.let { mapper.add(it) }
    }
    val built = mapper.snapshot()
    var occ = 0; var free = 0
    for (i in built.cells.indices) { if (built.occupied(i)) occ++ else if (built.free(i)) free++ }
    println("  %d scans, %d occupied and %d free cells".format(mapper.scans, occ, free))
    // every occupied cell should lie on a wall
    var onWall = 0
    for (r in 0 until built.height) for (c in 0 until built.width) if (built.occupied(built.index(r, c))) {
        val x = built.cellX(r); val y = built.cellY(c)
        if (ROOM.any { distToSeg(x, y, it) < 0.15 }) onWall++
    }
    check(onWall >= occ * 0.95, "occupied cells on real walls: %.1f%%".format(100.0 * onWall / occ))
    check(free > 4000, "free space mapped")
    MapFiles.save(dir, "room", built, listOf(Waypoint("A", 0.0, 0.0), Waypoint("B", 5.0, 0.0), Waypoint("C", 5.0, 4.4)), listOf("A", "B", "C"))
    val loaded = MapFiles.load(dir, "room")
    var same = 0
    for (i in built.cells.indices) if (built.occupied(i) == loaded.grid.occupied(i) && built.free(i) == loaded.grid.free(i)) same++
    check(same == built.cells.size && loaded.waypoints.size == 3 && loaded.patrol == listOf("A", "B", "C"), "PGM + YAML + JSON round trip")
    println("  files: " + dir.list()!!.sorted().joinToString())

    // ------------------------------------------------------------ 2. localisation
    println("2. a new boot: the car's frame is 0.4 m, -0.3 m, 8° off the map's")
    val off = Pose2(0.4, -0.3, Math.toRadians(8.0))
    val s2 = Sim(Pose2(0.3, 0.2, 0.0), off.inverse())     // odom = off⁻¹ ⊕ world, so map ← odom = off
    val map = loaded.grid
    val loc = Localizer(map)
    val cm = Costmap(map)
    s2.run(400, { VelCmd.STOP }) {}
    // the usual start mark: assume odom == map, let AMCL fix the rest
    loc.setPose(s2.poses.at(s2.tMs)!!, s2.poses.at(s2.tMs)!!)
    var t0 = System.nanoTime(); var updates = 0
    s2.run(6000, { VelCmd(0.3, 0.0) }) { sc ->
        OdomScan.build(sc, s2.poses, 1.0 / 6, 0.0, 0.0)?.let { if (loc.update(it)) updates++ }
    }
    val ms = (System.nanoTime() - t0) / 1e6 / maxOf(updates, 1)
    val errLoc = loc.pose.distanceTo(s2.truth)
    val errYaw = Math.toDegrees(abs(wrapPi(loc.pose.yaw - s2.truth.yaw)))
    println("  %d AMCL updates, %.1f ms each (whole step incl. the simulator)".format(updates, ms))
    check(errLoc < 0.10 && errYaw < 2.0, "AMCL pose error %.3f m, %.2f°".format(errLoc, errYaw))
    check(loc.confident, "confident (spread %.3f m, %.2f rad, match %.0f%%)".format(loc.spreadXY, loc.spreadYaw, loc.score * 100))

    // global localisation from scratch
    val loc2 = Localizer(map)
    val sc0 = OdomScan.build(s2.run1Scan(), s2.poses, 1.0 / 6, 0.0, 0.0)!!
    t0 = System.nanoTime()
    loc2.findGlobally(sc0)
    val gms = (System.nanoTime() - t0) / 1e6
    s2.run(5000, { VelCmd(0.3, 0.0) }) { sc -> OdomScan.build(sc, s2.poses, 1.0 / 6, 0.0, 0.0)?.let { loc2.update(it) } }
    check(loc2.pose.distanceTo(s2.truth) < 0.15, "Find me (global, %.0f ms): error %.3f m".format(gms, loc2.pose.distanceTo(s2.truth)))

    // ------------------------------------------------------------ 3. navigation
    println("3. navigation: A* + Regulated Pure Pursuit, a box in the way that is not on the map")
    val planner = Planner(cm)
    val rpp = Rpp(cm, Rpp.Params().apply { minTurnRadius = 0.9 })
    val drv = object : NavDriver {
        var cmd = VelCmd.STOP
        override fun sendVelocity(v: Double, yawRate: Double) { cmd = VelCmd(v, yawRate) }
        override fun setGuided() {}
        override fun setHold() { cmd = VelCmd.STOP }
        override val inGuided = true
        override val armed = true
    }
    val nav = Navigator(planner, rpp, drv).also { it.onClearObstacles = { cm.clearObstacles() } }
    var minClear = Double.MAX_VALUE
    fun drive(goals: List<Waypoint>, loop: Boolean, maxMs: Int, stopAfter: (Int) -> Boolean = { false }): Int {
        nav.start(goals, loop, s2.tMs)
        var steps = 0; var laps = 0; var lastIdx = 0
        while (steps * 100 < maxMs && (nav.active)) {
            s2.run(100, { drv.cmd }) { sc ->
                OdomScan.build(sc, s2.poses, 1.0 / 6, 0.0, 0.0)?.let { os ->
                    loc.update(os)
                    cm.addScan(os.transformed(loc.offset), s2.tMs)
                }
            }
            val odom = s2.poses.at(s2.tMs)!!
            nav.tick(s2.tMs, loc.offset + odom, loc.confident, 0, 0, s2.v)
            minClear = minOf(minClear, s2.walls.minOf { distToSeg(s2.truth.x, s2.truth.y, it) })
            if (nav.goalIndex < lastIdx) laps++
            lastIdx = nav.goalIndex
            if (stopAfter(laps)) { nav.stop(); break }
            steps++
        }
        return laps
    }
    // back to A first: the path starts behind the car (a 3-point turn)
    drive(listOf(Waypoint("A", 0.0, 0.0)), false, 90_000)
    val eA = hypot(s2.truth.x, s2.truth.y)
    check(nav.state == Navigator.State.DONE && eA < 0.35, "reached A behind the car: ${nav.status}, %.2f m from A".format(eA))
    check(minClear > 0.15, "never closer than %.2f m to a wall".format(minClear))
    minClear = Double.MAX_VALUE
    val unseen = box(2.3, -0.4, 2.7, 0.3)               // on the straight line from A to B, not on the map
    s2.walls = ROOM + unseen
    drive(listOf(Waypoint("B", 5.0, 0.0)), false, 90_000)
    val eB = hypot(s2.truth.x - 5.0, s2.truth.y - 0.0)
    check(nav.state == Navigator.State.DONE && eB < 0.35, "reached B round the unseen box: ${nav.status}, %.2f m from B".format(eB))
    check(minClear > 0.15, "never closer than %.2f m to a wall or the box".format(minClear))

    println("4. patrol A -> B -> C, twice round")
    minClear = Double.MAX_VALUE
    val wps = loaded.waypoints
    val laps = drive(wps, true, 400_000) { it >= 2 }
    check(laps >= 2, "patrol laps done: $laps (${nav.status})")
    check(minClear > 0.15, "never closer than %.2f m to a wall or the box".format(minClear))
    check(loc.pose.distanceTo(s2.truth) < 0.15, "still localised: error %.3f m".format(loc.pose.distanceTo(s2.truth)))

    println(if (failures == 0) "PASS" else "FAIL ($failures)")
    exitProcess(if (failures == 0) 0 else 1)
}

private fun Sim.run1Scan(): ScanFrame { var f: ScanFrame? = null; run(200, { VelCmd.STOP }) { f = it }; return f!! }

private fun distToSeg(px: Double, py: Double, s: Seg): Double {
    val ex = s.x2 - s.x1; val ey = s.y2 - s.y1
    val l2 = ex * ex + ey * ey
    val t = if (l2 > 0) (((px - s.x1) * ex + (py - s.y1) * ey) / l2).coerceIn(0.0, 1.0) else 0.0
    return hypot(px - (s.x1 + t * ex), py - (s.y1 + t * ey))
}

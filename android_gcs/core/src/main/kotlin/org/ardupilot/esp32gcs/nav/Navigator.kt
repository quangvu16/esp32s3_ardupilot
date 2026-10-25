package org.ardupilot.esp32gcs.nav

import kotlin.math.hypot

/** What the navigator needs from the car. */
interface NavDriver {
    /** Speed (m/s, negative reverse) and turn rate (rad/s, clockwise) in Guided mode. */
    fun sendVelocity(v: Double, yawRate: Double)
    fun setGuided()
    fun setHold()
    val inGuided: Boolean
    val armed: Boolean
}

/**
 * Nav2's bt_navigator and waypoint_follower as a small state machine: plan with
 * A* and replan once a second (Nav2's default tree: RateController 1 Hz), follow
 * with Regulated Pure Pursuit, and when stuck run the recoveries in turn (clear
 * the obstacle layer, wait, back up), up to 6 times before giving up on a goal.
 * A patrol goes through its waypoints in a loop, skipping one it can't reach
 * (stop_on_failure false).
 *
 * Safety: the car stops (zero speed, then Hold) when the pose or the scan is
 * stale, when localisation is lost, or when someone else changes the mode.
 */
class Navigator(private val planner: Planner, private val rpp: Rpp, private val drive: NavDriver) {
    enum class State { IDLE, FOLLOWING, BLOCKED, RECOVERING, WAITING, DONE, FAILED }

    class Params {
        var replanMs = 1000L                 // Nav2 default tree: replan at 1 Hz
        var progressDist = 0.5               // progress checker: 0.5 m ...
        var progressMs = 10_000L             // ... in 10 s
        var blockedReplanMs = 1500L          // stopped by an obstacle this long: plan round it
        var retries = 6                      // RecoveryNode number_of_retries
        var waitMs = 5000L                   // Wait recovery (Nav2 5 s)
        var backupDist = 0.3                 // m (Nav2 0.15; an RC car needs more to change anything)
        var backupSpeed = 0.2                // m/s (Nav2 0.025; below an ESC's dead band)
        var pauseAtWaypointMs = 3000L        // patrol: wait at each waypoint
        var staleMs = 1000L                  // pose or scan older than this: stop
        var lostGraceMs = 2000L              // localisation unsure for this long: stop
    }

    val p = Params()
    @Volatile var state = State.IDLE
        private set
    @Volatile var status = "Idle"
        private set
    @Volatile var path: List<DoubleArray> = emptyList()
        private set
    @Volatile var goals: List<Waypoint> = emptyList()
        private set
    @Volatile var goalIndex = 0
        private set
    @Volatile var loop = false
        private set
    @Volatile var lastCmd = VelCmd.STOP
        private set

    private var lastPlanMs = 0L
    private var failures = 0
    private var blockedSince = 0L
    private var progressPose: Pose2? = null
    private var progressMs = 0L
    private var recoveryStep = 0
    private var recoveryUntil = 0L
    private var backupStart: Pose2? = null
    private var waitUntil = 0L
    private var lostSince = 0L
    private var guidedAsked = 0L

    val goal get() = goals.getOrNull(goalIndex)
    val active get() = state == State.FOLLOWING || state == State.BLOCKED || state == State.RECOVERING || state == State.WAITING

    /** Go to one goal, or through [list] in order ([loop]: round and round, a patrol). */
    @Synchronized fun start(list: List<Waypoint>, loop: Boolean, nowMs: Long) {
        if (list.isEmpty()) return
        goals = list; this.loop = loop; goalIndex = 0
        beginGoal(nowMs)
        drive.setGuided(); guidedAsked = nowMs
    }

    @Synchronized fun stop(why: String = "Stopped") {
        val was = active
        state = State.IDLE; status = why; path = emptyList()
        if (was) { drive.sendVelocity(0.0, 0.0); drive.setHold() }
        lastCmd = VelCmd.STOP
    }

    private fun beginGoal(nowMs: Long) {
        state = State.FOLLOWING; failures = 0; recoveryStep = 0
        path = emptyList(); lastPlanMs = 0L; blockedSince = 0L
        progressPose = null; progressMs = nowMs
        rpp.reset()
        status = "Going to ${goal?.name}"
    }

    private fun fault(why: String) {
        state = State.FAILED; status = why; path = emptyList()
        drive.sendVelocity(0.0, 0.0); drive.setHold()
        lastCmd = VelCmd.STOP
    }

    /**
     * One step, 10 times a second. [pose]: the car in the map (null if not known);
     * [confident]: localisation is good; [poseAgeMs] / [scanAgeMs]: how old the
     * latest data is; [vNow]: the car's speed.
     */
    @Synchronized fun tick(nowMs: Long, pose: Pose2?, confident: Boolean, poseAgeMs: Long, scanAgeMs: Long, vNow: Double) {
        if (!active) return
        // --- safety -------------------------------------------------------------
        if (pose == null || poseAgeMs > p.staleMs) { fault("Stopped: no position from the car"); return }
        if (scanAgeMs > p.staleMs) { fault("Stopped: no LiDAR scan"); return }
        if (!confident) {
            if (lostSince == 0L) lostSince = nowMs
            if (nowMs - lostSince > p.lostGraceMs) { fault("Stopped: lost in the map (set the pose again)"); return }
            send(VelCmd(0.0, 0.0, "localising")); status = "Localising…"; return
        }
        lostSince = 0L
        if (!drive.armed) { fault("Stopped: the car is disarmed"); return }
        if (!drive.inGuided) {
            if (nowMs - guidedAsked < 3000) { send(VelCmd.STOP); return }      // the mode change is on its way
            stop("Stopped: the mode was changed"); return
        }
        val g = goal ?: run { stop("Done"); return }

        when (state) {
            State.WAITING -> {
                send(VelCmd.STOP)
                if (nowMs >= waitUntil) nextGoal(nowMs)
                return
            }
            State.RECOVERING -> { recover(nowMs, pose); return }
            else -> {}
        }

        // --- plan ---------------------------------------------------------------
        val due = path.isEmpty() || nowMs - lastPlanMs >= p.replanMs ||
            (state == State.BLOCKED && nowMs - blockedSince >= p.blockedReplanMs)
        if (due) {
            lastPlanMs = nowMs
            val np = planner.plan(pose, g.x, g.y)
            if (np == null) {
                if (path.isEmpty() || planner.pathBlocked(path, rpp.closest(pose, path))) { startRecovery(nowMs, "no path"); return }
            } else {
                path = np
                if (state == State.BLOCKED) { state = State.FOLLOWING; blockedSince = 0L }
            }
        }

        // --- follow -------------------------------------------------------------
        val cmd = rpp.step(pose, path, vNow)
        when (cmd.note) {
            "arrived" -> { send(VelCmd.STOP); arrived(nowMs); return }
            "obstacle ahead", "blocked behind" -> {
                if (state != State.BLOCKED) { state = State.BLOCKED; blockedSince = nowMs; lastPlanMs = 0L }
                status = "Blocked: ${cmd.note}, replanning"
            }
            else -> { if (state == State.BLOCKED) state = State.FOLLOWING; status = "Going to ${g.name} (${cmd.note})" }
        }
        send(cmd)

        // --- progress checker -----------------------------------------------------
        val pp = progressPose
        if (pp == null || pose.distanceTo(pp) >= p.progressDist) { progressPose = pose; progressMs = nowMs }
        else if (nowMs - progressMs > p.progressMs) startRecovery(nowMs, "no progress")
    }

    private fun arrived(nowMs: Long) {
        failures = 0
        if (goalIndex == goals.size - 1 && !loop) {
            state = State.DONE; status = "Arrived at ${goal?.name}"; path = emptyList()
            drive.setHold(); return
        }
        state = State.WAITING; waitUntil = nowMs + p.pauseAtWaypointMs
        status = "At ${goal?.name}, waiting"
    }

    private fun nextGoal(nowMs: Long) {
        goalIndex++
        if (goalIndex >= goals.size) {
            if (!loop) { state = State.DONE; status = "Done"; drive.setHold(); return }
            goalIndex = 0
        }
        beginGoal(nowMs)
    }

    private fun startRecovery(nowMs: Long, why: String) {
        failures++
        if (failures > p.retries) {
            if (goals.size > 1) {                    // patrol: skip this waypoint (stop_on_failure false)
                status = "Skipped ${goal?.name}: $why"
                nextGoal(nowMs); return
            }
            fault("Gave up: $why"); return
        }
        state = State.RECOVERING
        recoveryStep = (failures - 1) % 3
        recoveryUntil = nowMs + if (recoveryStep == 1) p.waitMs else 0L
        backupStart = null
        status = "Recovery ${failures}/${p.retries} ($why): " + listOf("clearing obstacles", "waiting", "backing up")[recoveryStep]
        if (recoveryStep == 0) onClearObstacles?.invoke()
    }

    /** Set by the owner: Nav2's ClearEntireCostmap. */
    var onClearObstacles: (() -> Unit)? = null

    private fun recover(nowMs: Long, pose: Pose2) {
        when (recoveryStep) {
            0 -> endRecovery(nowMs)
            1 -> { send(VelCmd.STOP); if (nowMs >= recoveryUntil) endRecovery(nowMs) }
            else -> {
                val s = backupStart ?: pose.also { backupStart = it }
                val moved = hypot(pose.x - s.x, pose.y - s.y)
                if (moved >= p.backupDist || rpp.arcCollides(pose, -p.backupSpeed, 0.0)) { send(VelCmd.STOP); endRecovery(nowMs) }
                else send(VelCmd(-p.backupSpeed, 0.0, "backing up"))
            }
        }
    }

    private fun endRecovery(nowMs: Long) {
        state = State.FOLLOWING; path = emptyList(); lastPlanMs = 0L
        progressPose = null; progressMs = nowMs; rpp.reset()
    }

    private fun send(c: VelCmd) { lastCmd = c; drive.sendVelocity(c.v, c.yawRate) }
}

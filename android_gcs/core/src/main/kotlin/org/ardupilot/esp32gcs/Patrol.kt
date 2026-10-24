package org.ardupilot.esp32gcs

/**
 * Patrol: drive the plan round and round in Auto, with ArduPilot's BendyRuler
 * (OA_TYPE 1) steering round whatever the LiDAR sees. The loop is a DO_JUMP at
 * the end of the mission, so the car keeps patrolling on its own even if the
 * phone or PC loses the link.
 */
object Patrol {
    const val CMD_DO_JUMP = 177

    /**
     * BendyRuler settings for a small car with the LD14 (README step 8), tested in
     * SITL's post field: a 3 m margin left no way between obstacles 8 m apart, and
     * ArduPilot's simple avoidance (AVOID_MARGIN 2 m by default) stopped the car
     * before BendyRuler could steer.
     */
    val BENDY_RULER = linkedMapOf(
        "OA_TYPE" to 1f,             // BendyRuler
        "OA_MARGIN_MAX" to 2f,       // m kept from obstacles; above WP_RADIUS
        "WP_RADIUS" to 1f,           // m; below OA_MARGIN_MAX, at least TURN_RADIUS
        "OA_BR_LOOKAHEAD" to 5f,     // m probed ahead, under the LD14's 8 m
        "OA_DB_DIST_MAX" to 8f,      // m, the LD14's range
        "WP_SPEED" to 1f,            // m/s: a car's turns carry it on at speed
        "AVOID_MARGIN" to 1f,        // m; simple avoidance as a last stop, below OA_MARGIN_MAX
    )

    /**
     * The plan as a patrol: any DO_JUMP already at its end is replaced, the last
     * positioned waypoint waits [waitS] at the end of each lap, and a DO_JUMP back
     * to the first item repeats it. [laps] 0 = forever.
     */
    fun apply(plan: List<MissionItem>, laps: Int, waitS: Float): List<MissionItem> {
        val items = plan.map { it.copy() }.toMutableList()
        while (items.isNotEmpty() && items.last().command == CMD_DO_JUMP) items.removeAt(items.size - 1)
        items.lastOrNull { it.command == Mav.CMD_NAV_WAYPOINT && it.hasPosition }?.p1 = waitS.coerceAtLeast(0f)
        if (laps == 1) return items
        // Mission item 1 is the plan's first item (item 0 is home). num_times counts repeats.
        items += MissionItem(command = CMD_DO_JUMP, p1 = 1f, p2 = if (laps <= 0) -1f else (laps - 1).toFloat())
        return items
    }

    /** Laps a plan drives: 1 without a DO_JUMP at the end, 0 = forever. */
    fun laps(plan: List<MissionItem>): Int {
        val j = plan.lastOrNull()?.takeIf { it.command == CMD_DO_JUMP } ?: return 1
        return if (j.p2 < 0) 0 else j.p2.toInt() + 1
    }

    /** What stops BendyRuler from working well, given the vehicle's parameters (null = not read). */
    fun bendyRulerProblems(param: (String) -> Float?): List<String> {
        val out = ArrayList<String>()
        val prx = param("PRX1_TYPE")
        if (prx == null) return listOf("Read the parameters first")
        if (prx == 0f) out += "No LiDAR: PRX1_TYPE is 0 (19 = LD14)"
        val oa = param("OA_TYPE")
        val bendy = oa == 1f || oa == 3f
        if (!bendy) out += "OA_TYPE is ${oa?.toInt() ?: "?"}: BendyRuler is 1 (or 3 with fences)"
        if (bendy && param("OA_BR_LOOKAHEAD") == null) out += "Refresh the parameters: the OA_BR_ ones appear after the reboot"
        for ((k, want) in BENDY_RULER) {
            val v = param(k) ?: continue
            if (k != "OA_TYPE" && v != want) out += "$k is ${fmt(v)} (suggested ${fmt(want)})"
        }
        val margin = param("OA_MARGIN_MAX"); val wpr = param("WP_RADIUS")
        if (margin != null && wpr != null && margin <= wpr) out += "OA_MARGIN_MAX (${fmt(margin)}) should be above WP_RADIUS (${fmt(wpr)})"
        val avoid = param("AVOID_ENABLE")?.toInt() ?: 0; val avoidMargin = param("AVOID_MARGIN")
        if (avoid and 2 != 0 && margin != null && avoidMargin != null && avoidMargin >= margin)
            out += "AVOID_MARGIN (${fmt(avoidMargin)}) stops the car before BendyRuler steers: keep it below OA_MARGIN_MAX"
        return out
    }

    /**
     * What "Set up BendyRuler" writes now, and whether a reboot must follow. The
     * OA_BR_ parameters only exist once the board has rebooted with OA_TYPE 1, so
     * the first press writes OA_TYPE (and what already exists) and reboots; the
     * second press, after it reconnects, writes the rest.
     */
    fun setupWrites(param: (String) -> Float?): Pair<Map<String, Float>, Boolean> {
        val oa = param("OA_TYPE")
        val bendyNow = oa == 1f || oa == 3f
        val writes = BENDY_RULER.filter { (k, v) ->
            if (k == "OA_TYPE") !bendyNow else param(k) != null && param(k) != v
        }
        return Pair(writes, !bendyNow)
    }

    private fun fmt(f: Float) = if (f == f.toLong().toFloat()) f.toLong().toString() else f.toString()
}

/** Counts patrol laps from MISSION_CURRENT: a lap ends when Auto jumps back to an earlier item. */
class LapCounter {
    var lap = 0
        private set
    private var lastSeq = -1

    fun reset() { lap = 0; lastSeq = -1 }

    fun update(seq: Int, inAuto: Boolean): Int {
        if (!inAuto) { lastSeq = -1; return lap }
        if (lap == 0) lap = 1
        if (lastSeq >= 0 && seq < lastSeq) lap++
        lastSeq = seq
        return lap
    }
}

package org.ardupilot.esp32gcs

/**
 * Geofence zones as ArduPilot stores them (AC_PolyFence_loader): inclusion and
 * exclusion polygons and circles, and an optional return point. They travel as
 * mission items of mission_type FENCE, one item per polygon vertex.
 */
class FenceZone(var kind: Kind, val points: MutableList<DoubleArray> = ArrayList(), var radius: Float = 0f) {
    enum class Kind(val label: String, val polygon: Boolean, val inclusion: Boolean) {
        POLYGON_IN("Stay-in polygon", true, true),
        POLYGON_OUT("Stay-out polygon", true, false),
        CIRCLE_IN("Stay-in circle", false, true),
        CIRCLE_OUT("Stay-out circle", false, false),
        RETURN("Return point", false, false),
    }

    val isCircle get() = kind == Kind.CIRCLE_IN || kind == Kind.CIRCLE_OUT

    fun copy() = FenceZone(kind, points.map { it.copyOf() }.toMutableList(), radius)
}

object Fence {
    const val CMD_RETURN_POINT = 5000
    const val CMD_POLYGON_INCLUSION = 5001
    const val CMD_POLYGON_EXCLUSION = 5002
    const val CMD_CIRCLE_INCLUSION = 5003
    const val CMD_CIRCLE_EXCLUSION = 5004

    /** Fence storage on the esp32s3devkit board (16 KB parameter storage, Rover): 160 + 256 + 256 bytes. */
    const val BOARD_STORAGE_BYTES = 672

    fun toItems(zones: List<FenceZone>): List<MissionItem> {
        val out = ArrayList<MissionItem>()
        for (z in zones) when (z.kind) {
            FenceZone.Kind.POLYGON_IN, FenceZone.Kind.POLYGON_OUT -> {
                val cmd = if (z.kind == FenceZone.Kind.POLYGON_IN) CMD_POLYGON_INCLUSION else CMD_POLYGON_EXCLUSION
                for (p in z.points) out += item(cmd, p, z.points.size.toFloat())
            }
            FenceZone.Kind.CIRCLE_IN, FenceZone.Kind.CIRCLE_OUT -> {
                val cmd = if (z.kind == FenceZone.Kind.CIRCLE_IN) CMD_CIRCLE_INCLUSION else CMD_CIRCLE_EXCLUSION
                z.points.firstOrNull()?.let { out += item(cmd, it, z.radius) }
            }
            FenceZone.Kind.RETURN -> z.points.firstOrNull()?.let { out += item(CMD_RETURN_POINT, it, 0f) }
        }
        return out
    }

    private fun item(cmd: Int, p: DoubleArray, p1: Float) =
        MissionItem(command = cmd, lat = p[0], lon = p[1], p1 = p1, frame = Mav.FRAME_GLOBAL_RELATIVE_ALT_INT)

    /** Groups downloaded items back into zones; consecutive vertices of one polygon carry its vertex count in param1. */
    fun fromItems(items: List<MissionItem>): List<FenceZone> {
        val zones = ArrayList<FenceZone>()
        var i = 0
        while (i < items.size) {
            val it = items[i]
            val p = doubleArrayOf(it.lat, it.lon)
            when (it.command) {
                CMD_POLYGON_INCLUSION, CMD_POLYGON_EXCLUSION -> {
                    val n = it.p1.toInt().coerceAtLeast(1)
                    val z = FenceZone(if (it.command == CMD_POLYGON_INCLUSION) FenceZone.Kind.POLYGON_IN else FenceZone.Kind.POLYGON_OUT)
                    for (k in i until minOf(i + n, items.size)) z.points += doubleArrayOf(items[k].lat, items[k].lon)
                    zones += z
                    i += n
                    continue
                }
                CMD_CIRCLE_INCLUSION -> zones += FenceZone(FenceZone.Kind.CIRCLE_IN, mutableListOf(p), it.p1)
                CMD_CIRCLE_EXCLUSION -> zones += FenceZone(FenceZone.Kind.CIRCLE_OUT, mutableListOf(p), it.p1)
                CMD_RETURN_POINT -> zones += FenceZone(FenceZone.Kind.RETURN, mutableListOf(p))
            }
            i++
        }
        return zones
    }

    /** Bytes ArduPilot needs to store these zones (AC_PolyFence_loader::fence_storage_space_required). */
    fun storageBytes(zones: List<FenceZone>): Int = 4 + zones.sumOf { z ->
        1 + when {
            z.kind.polygon -> 1 + 8 * z.points.size
            z.isCircle -> 12
            else -> 8
        }
    }

    /** The first problem ArduPilot would refuse the upload for, or null. */
    fun problem(zones: List<FenceZone>): String? {
        if (zones.count { it.kind == FenceZone.Kind.RETURN } > 1) return "Only one return point is allowed"
        zones.forEachIndexed { i, z ->
            val name = "${z.kind.label} ${i + 1}"
            if (z.kind.polygon && z.points.size < 3) return "$name needs at least 3 corners (has ${z.points.size})"
            if (z.kind.polygon && z.points.size > 255) return "$name has more than 255 corners"
            if (z.isCircle && z.radius <= 0f) return "$name needs a radius"
            if (z.points.isEmpty()) return "$name has no position"
        }
        return null
    }
}

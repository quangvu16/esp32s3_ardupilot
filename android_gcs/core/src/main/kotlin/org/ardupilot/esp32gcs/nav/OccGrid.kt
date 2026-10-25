package org.ardupilot.esp32gcs.nav

import java.io.ByteArrayOutputStream
import java.io.File
import java.util.Locale
import kotlin.math.floor

/**
 * An occupancy grid: [width] cells east by [height] cells north, [res] metres a
 * cell, cell (row 0, col 0) at [originX] north / [originY] east of the frame's
 * origin. Each cell is a log-odds value -127 (free) .. 127 (occupied), 0 unknown.
 */
class OccGrid(val width: Int, val height: Int, val res: Double, val originX: Double, val originY: Double) {
    val cells = ByteArray(width * height)

    fun row(x: Double) = floor((x - originX) / res).toInt()
    fun col(y: Double) = floor((y - originY) / res).toInt()
    fun inside(r: Int, c: Int) = r in 0 until height && c in 0 until width
    fun index(r: Int, c: Int) = r * width + c
    /** Centre of a cell, north and east. */
    fun cellX(r: Int) = originX + (r + 0.5) * res
    fun cellY(c: Int) = originY + (c + 0.5) * res

    fun occupied(i: Int) = cells[i] > OCC_THRESHOLD
    fun free(i: Int) = cells[i] < FREE_THRESHOLD
    fun known(i: Int) = occupied(i) || free(i)

    fun copy(): OccGrid = OccGrid(width, height, res, originX, originY).also { cells.copyInto(it.cells) }

    companion object {
        // Nav2 map_saver thresholds (occupied_thresh 0.65, free_thresh 0.25) in log-odds, x 20
        const val OCC_THRESHOLD = 12        // ln(0.65 / 0.35) * 20
        const val FREE_THRESHOLD = -21      // ln(0.25 / 0.75) * 20
    }
}

/**
 * Maps on disk in the ROS map_server format (Nav2 keeps it): an 8-bit PGM image
 * (254 free, 0 occupied, 205 unknown, top row north) and a YAML file with the
 * resolution and the bottom-left corner. free_thresh is 0.196, as ROS's map_saver
 * writes, so that 205 reads back as unknown ((255 - 205) / 255 = 0.19608). ROS maps are x east, y north, so the
 * YAML origin is [east, north, 0]. The map loads in RViz / Nav2 as it is.
 * Beside them, <name>.json keeps the named waypoints and the patrol.
 */
object MapFiles {
    data class Saved(val grid: OccGrid, val waypoints: List<Waypoint>, val patrol: List<String>)

    fun list(dir: File): List<String> =
        (dir.listFiles { f -> f.name.endsWith(".yaml") } ?: emptyArray()).map { it.name.removeSuffix(".yaml") }.sorted()

    fun save(dir: File, name: String, grid: OccGrid, waypoints: List<Waypoint>, patrol: List<String>) {
        dir.mkdirs()
        val pgm = ByteArrayOutputStream()
        pgm.write("P5\n# esp32s3_ardupilot indoor map\n${grid.width} ${grid.height}\n255\n".toByteArray(Charsets.US_ASCII))
        for (r in grid.height - 1 downTo 0) for (c in 0 until grid.width) {
            val i = grid.index(r, c)
            pgm.write(if (grid.occupied(i)) 0 else if (grid.free(i)) 254 else 205)
        }
        File(dir, "$name.pgm").writeBytes(pgm.toByteArray())
        File(dir, "$name.yaml").writeText(
            "image: $name.pgm\nmode: trinary\nresolution: ${fmt(grid.res)}\n" +
            "origin: [${fmt(grid.originY)}, ${fmt(grid.originX)}, 0.0]\nnegate: 0\n" +
            "occupied_thresh: 0.65\nfree_thresh: 0.196\n")
        saveWaypoints(dir, name, waypoints, patrol)
    }

    fun saveWaypoints(dir: File, name: String, waypoints: List<Waypoint>, patrol: List<String>) {
        val sb = StringBuilder("{\n  \"waypoints\": [")
        waypoints.forEachIndexed { k, w ->
            sb.append(if (k == 0) "\n" else ",\n")
            sb.append("    {\"name\": ${quote(w.name)}, \"x\": ${fmt(w.x)}, \"y\": ${fmt(w.y)}")
            if (!w.yaw.isNaN()) sb.append(", \"yaw\": ${fmt(w.yaw)}")
            sb.append("}")
        }
        sb.append("\n  ],\n  \"patrol\": [").append(patrol.joinToString(", ") { quote(it) }).append("]\n}\n")
        File(dir, "$name.json").writeText(sb.toString())
    }

    fun load(dir: File, name: String): Saved {
        val yaml = File(dir, "$name.yaml").readText()
        fun field(key: String) = Regex("(?m)^\\s*$key\\s*:\\s*(.+)$").find(yaml)?.groupValues?.get(1)?.trim()
            ?: throw IllegalArgumentException("$name.yaml: no $key")
        val image = field("image").trim('"', '\'')
        val res = field("resolution").toDouble()
        val origin = field("origin").trim('[', ']').split(",").map { it.trim().toDouble() }
        val negate = (Regex("(?m)^\\s*negate\\s*:\\s*(\\d)").find(yaml)?.groupValues?.get(1) ?: "0") == "1"
        val occT = Regex("(?m)^\\s*occupied_thresh\\s*:\\s*([0-9.]+)").find(yaml)?.groupValues?.get(1)?.toDouble() ?: 0.65
        val freeT = Regex("(?m)^\\s*free_thresh\\s*:\\s*([0-9.]+)").find(yaml)?.groupValues?.get(1)?.toDouble() ?: 0.25
        val (w, h, px) = readPgm(File(dir, image).let { if (it.isAbsolute) it else File(dir, image) })
        val grid = OccGrid(w, h, res, originX = origin[1], originY = origin[0])
        for (r in 0 until h) for (c in 0 until w) {
            val v = px[(h - 1 - r) * w + c].toInt() and 0xff
            val p = if (negate) v / 255.0 else (255 - v) / 255.0
            grid.cells[grid.index(r, c)] = when {
                p > occT -> 100
                p < freeT -> -100
                else -> 0
            }.toByte()
        }
        val (wps, patrol) = loadWaypoints(File(dir, "$name.json"))
        return Saved(grid, wps, patrol)
    }

    fun delete(dir: File, name: String) {
        for (ext in listOf("yaml", "pgm", "json")) File(dir, "$name.$ext").delete()
    }

    private fun readPgm(f: File): Triple<Int, Int, ByteArray> {
        val b = f.readBytes()
        var i = 0
        fun token(): String {
            while (i < b.size) {
                val ch = b[i].toInt().toChar()
                if (ch == '#') { while (i < b.size && b[i] != '\n'.code.toByte()) i++ } else if (ch.isWhitespace()) i++ else break
            }
            val s = i
            while (i < b.size && !b[i].toInt().toChar().isWhitespace()) i++
            return String(b, s, i - s, Charsets.US_ASCII)
        }
        require(token() == "P5") { "${f.name}: not a binary PGM" }
        val w = token().toInt(); val h = token().toInt(); val max = token().toInt()
        require(max in 1..255) { "${f.name}: 16-bit PGM not supported" }
        i++                                     // the single whitespace before the pixels
        require(b.size - i >= w * h) { "${f.name}: short" }
        return Triple(w, h, b.copyOfRange(i, i + w * h))
    }

    private fun loadWaypoints(f: File): Pair<List<Waypoint>, List<String>> {
        if (!f.exists()) return Pair(emptyList(), emptyList())
        val t = f.readText()
        val wps = Regex("\\{\\s*\"name\"\\s*:\\s*\"((?:[^\"\\\\]|\\\\.)*)\"([^}]*)\\}").findAll(t).map { m ->
            fun num(k: String) = Regex("\"$k\"\\s*:\\s*(-?[0-9.eE+-]+)").find(m.groupValues[2])?.groupValues?.get(1)?.toDouble()
            Waypoint(unquote(m.groupValues[1]), num("x") ?: 0.0, num("y") ?: 0.0, num("yaw") ?: Double.NaN)
        }.toList()
        val patrol = Regex("\"patrol\"\\s*:\\s*\\[([^\\]]*)\\]").find(t)?.groupValues?.get(1)?.let { body ->
            Regex("\"((?:[^\"\\\\]|\\\\.)*)\"").findAll(body).map { unquote(it.groupValues[1]) }.toList()
        } ?: emptyList()
        return Pair(wps, patrol)
    }

    private fun fmt(v: Double) = String.format(Locale.ROOT, "%.4f", v)
    private fun quote(s: String) = "\"" + s.replace("\\", "\\\\").replace("\"", "\\\"") + "\""
    private fun unquote(s: String) = s.replace("\\\"", "\"").replace("\\\\", "\\")
}

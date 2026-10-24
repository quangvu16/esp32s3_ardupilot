package org.ardupilot.esp32gcs

import java.io.InputStream
import java.net.InetSocketAddress
import java.net.Socket
import java.util.Calendar
import java.util.TimeZone
import kotlin.math.abs

/**
 * NTRIP 1.0 client: pulls RTCM3 from a caster and hands each complete RTCM3
 * frame to [onFrame] (for GPS_RTCM_DATA), like Mission Planner's RTK inject.
 * Sends the rover's position as GGA every 10 s for casters that need it (VRS).
 */
class NtripClient(
    val host: String, val port: Int, val mount: String,
    private val user: String, private val pass: String,
    private val position: () -> Pair<Double, Double>?,
    private val onFrame: (ByteArray) -> Unit,
) {
    @Volatile var status = "Stopped"
        private set
    @Volatile var bytes = 0L
        private set
    @Volatile var frames = 0
        private set
    /** RTCM message type -> count, e.g. 1005, 1074, 1084… */
    val types = java.util.TreeMap<Int, Int>()

    @Volatile private var running = false
    private var sock: Socket? = null
    private var thread: Thread? = null

    fun start() {
        running = true
        thread = Thread({ loop() }, "ntrip").apply { isDaemon = true; start() }
    }

    fun stop() {
        running = false
        try { sock?.close() } catch (_: Exception) {}
        thread?.interrupt()
        status = "Stopped"
    }

    private fun loop() {
        while (running) {
            try {
                session()
            } catch (e: Exception) {
                if (!running) break
                status = "Error: ${e.message ?: e.javaClass.simpleName}; retrying"
            }
            try { Thread.sleep(3000) } catch (_: InterruptedException) { break }
        }
    }

    private fun session() {
        status = "Connecting to $host:$port…"
        val s = Socket()
        sock = s
        Platform.net.bindInternet(s)
        s.connect(InetSocketAddress(host, port), 8000)
        s.soTimeout = 15000
        val out = s.getOutputStream()
        val auth = java.util.Base64.getEncoder().encodeToString("$user:$pass".toByteArray())
        val req = "GET /$mount HTTP/1.0\r\n" +
            "User-Agent: NTRIP Esp32RoverGcs/1.0\r\n" +
            "Accept: */*\r\n" +
            (if (user.isNotEmpty()) "Authorization: Basic $auth\r\n" else "") +
            "Connection: close\r\n\r\n"
        out.write(req.toByteArray()); out.flush()
        val ins = s.getInputStream()
        val head = readLine(ins)
        if (!head.contains("200")) {
            throw java.io.IOException(if (head.contains("401")) "wrong user or password"
                else if (head.startsWith("SOURCETABLE")) "mountpoint not found" else head.ifEmpty { "no answer" })
        }
        // Skip the rest of the headers (NTRIP 1 "ICY 200 OK" usually has none).
        if (head.startsWith("HTTP")) while (readLine(ins).isNotEmpty()) {}
        status = "Connected: $mount"
        var lastGga = 0L
        val buf = ByteArray(4096)
        val rtcm = Rtcm3Splitter { frame, type ->
            frames++
            synchronized(types) { types[type] = (types[type] ?: 0) + 1 }
            onFrame(frame)
        }
        while (running) {
            val now = System.currentTimeMillis()
            if (now - lastGga > 10000) {
                lastGga = now
                position()?.let { (la, lo) -> out.write(gga(la, lo).toByteArray()); out.flush() }
            }
            val n = ins.read(buf)
            if (n < 0) throw java.io.IOException("caster closed the stream")
            bytes += n
            rtcm.feed(buf, n)
        }
        s.close()
    }

    private fun readLine(ins: InputStream): String {
        val sb = StringBuilder()
        while (true) {
            val c = ins.read()
            if (c < 0 || c == '\n'.code) break
            if (c != '\r'.code) sb.append(c.toChar())
            if (sb.length > 512) break
        }
        return sb.toString()
    }

    companion object {
        fun gga(lat: Double, lon: Double): String {
            val t = Calendar.getInstance(TimeZone.getTimeZone("UTC"))
            fun dm(v: Double, degDigits: Int): String {
                val a = abs(v); val d = a.toInt(); val m = (a - d) * 60
                return "%0${degDigits}d%07.4f".format(java.util.Locale.US, d, m)
            }
            val body = "GPGGA,%02d%02d%02d.00,%s,%s,%s,%s,1,12,1.0,0.0,M,0.0,M,,".format(
                java.util.Locale.US,
                t.get(Calendar.HOUR_OF_DAY), t.get(Calendar.MINUTE), t.get(Calendar.SECOND),
                dm(lat, 2), if (lat >= 0) "N" else "S", dm(lon, 3), if (lon >= 0) "E" else "W")
            var cs = 0
            for (ch in body) cs = cs xor ch.code
            return "$$body*%02X\r\n".format(cs)
        }
    }
}

/** Finds RTCM3 frames (0xD3, 10-bit length, payload, CRC-24Q) in a byte stream. */
class Rtcm3Splitter(private val out: (ByteArray, Int) -> Unit) {
    private val buf = ByteArray(8192)
    private var len = 0

    fun feed(data: ByteArray, n: Int) {
        var off = 0
        while (off < n) {
            val k = minOf(n - off, buf.size - len)
            System.arraycopy(data, off, buf, len, k)
            len += k; off += k
            scan()
            if (len == buf.size) len = 0      // garbage that never framed
        }
    }

    private fun scan() {
        var i = 0
        while (i < len) {
            if (buf[i] != 0xD3.toByte()) { i++; continue }
            if (len - i < 6) break
            val plen = ((buf[i + 1].toInt() and 0x03) shl 8) or (buf[i + 2].toInt() and 0xff)
            val total = 3 + plen + 3
            if (len - i < total) break
            val crc = ((buf[i + total - 3].toInt() and 0xff) shl 16) or
                ((buf[i + total - 2].toInt() and 0xff) shl 8) or (buf[i + total - 1].toInt() and 0xff)
            if (crc24q(buf, i, 3 + plen) != crc) { i++; continue }
            val type = if (plen >= 2) ((buf[i + 3].toInt() and 0xff) shl 4) or ((buf[i + 4].toInt() and 0xff) shr 4) else 0
            out(buf.copyOfRange(i, i + total), type)
            i += total
        }
        System.arraycopy(buf, i, buf, 0, len - i)
        len -= i
    }

    private fun crc24q(b: ByteArray, from: Int, n: Int): Int {
        var crc = 0
        for (k in from until from + n) {
            crc = crc xor ((b[k].toInt() and 0xff) shl 16)
            repeat(8) {
                crc = crc shl 1
                if (crc and 0x1000000 != 0) crc = crc xor 0x1864CFB
            }
        }
        return crc and 0xffffff
    }
}

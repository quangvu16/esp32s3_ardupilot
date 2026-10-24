package org.ardupilot.esp32gcs

import java.io.InputStream
import java.io.OutputStream
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.Socket
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit

/**
 * A byte pipe to the autopilot. The ESP32 board's access point has no internet,
 * so Android would route a plain socket over mobile data; [Platform.net] binds it
 * to the WiFi network instead (map tiles and NTRIP stay on the default network).
 */
abstract class Link {
    @Volatile var open = false
        protected set
    var onBytes: ((ByteArray, Int) -> Unit)? = null
    var onClosed: ((String) -> Unit)? = null

    private val txq = LinkedBlockingQueue<ByteArray>(256)
    private var threads = listOf<Thread>()
    @Volatile private var stopping = false

    protected abstract fun connect()
    protected abstract fun read(buf: ByteArray): Int
    protected abstract fun write(b: ByteArray)
    protected abstract fun shut()
    abstract val name: String

    fun start() {
        val rx = Thread({
            try {
                connect()
                open = true
                val buf = ByteArray(2048)
                while (!stopping) {
                    val n = read(buf)
                    if (n < 0) throw java.io.IOException("closed by the board")
                    if (n > 0) onBytes?.invoke(buf, n)
                }
            } catch (e: Exception) {
                if (!stopping) fail(e.message ?: e.javaClass.simpleName)
            }
        }, "link-rx")
        val tx = Thread({
            try {
                while (!stopping) {
                    val b = txq.poll(200, TimeUnit.MILLISECONDS) ?: continue
                    if (b === CLOSE) break
                    if (open) write(b)
                }
            } catch (e: Exception) {
                if (!stopping) fail(e.message ?: e.javaClass.simpleName)
            }
        }, "link-tx")
        threads = listOf(rx, tx)
        threads.forEach { it.isDaemon = true; it.start() }
    }

    /** Queues a frame; dropped if the link is backed up (telemetry, not a file transfer). */
    fun send(b: ByteArray) {
        if (open) txq.offer(b)
    }

    private fun fail(why: String) {
        if (stopping) return
        stopping = true
        open = false
        try { shut() } catch (_: Exception) {}
        onClosed?.invoke(why)
    }

    /**
     * Sends what is still queued (up to 300 ms), then closes. The last frame is
     * often a zero-stick MANUAL_CONTROL from stopping a LiDAR behaviour or the
     * keyboard: dropped, the car would keep the old sticks for RC_OVERRIDE_TIME.
     */
    fun close() {
        val tx = threads.getOrNull(1)
        if (open && tx != null && tx.isAlive && Thread.currentThread() !== tx && txq.offer(CLOSE)) {
            try { tx.join(300) } catch (_: InterruptedException) {}
        }
        stopping = true
        open = false
        try { shut() } catch (_: Exception) {}
        threads.forEach { it.interrupt() }
    }

    private companion object {
        /** Queue marker: [close] waits for the tx thread to reach it. */
        val CLOSE = ByteArray(0)
    }
}

/** MAVLink over TCP: the board listens on 192.168.4.1:5760 (also ArduPilot SITL's default). */
class TcpLink(private val host: String, private val port: Int) : Link() {
    private val sock = Socket()
    private lateinit var ins: InputStream
    private lateinit var outs: OutputStream
    override val name get() = "TCP $host:$port"

    override fun connect() {
        if (isLocal(host)) Platform.net.bindLocal(sock)
        sock.tcpNoDelay = true
        sock.connect(InetSocketAddress(host, port), 5000)
        sock.soTimeout = 0
        ins = sock.getInputStream()
        outs = sock.getOutputStream()
    }

    override fun read(buf: ByteArray) = ins.read(buf)
    override fun write(b: ByteArray) { outs.write(b); outs.flush() }
    override fun shut() = sock.close()
}

/**
 * MAVLink over UDP. With [host] blank it listens on [port] and answers whoever
 * sends: the board's HAL_ESP32_WIFI 2 mode broadcasts to 192.168.4.255:14550,
 * and SITL / mavproxy --out send there too. With a host it sends there first.
 */
class UdpLink(private val host: String, private val port: Int) : Link() {
    private lateinit var sock: DatagramSocket
    @Volatile private var peer: InetSocketAddress? = null
    override val name get() = if (host.isBlank()) "UDP listen :$port" else "UDP $host:$port"

    override fun connect() {
        sock = if (host.isBlank()) DatagramSocket(port).apply { broadcast = true } else DatagramSocket()
        // Replies to the board must leave through its WiFi, not mobile data.
        if (host.isBlank() || isLocal(host)) Platform.net.bindLocal(sock)
        if (host.isNotBlank()) peer = InetSocketAddress(InetAddress.getByName(host), port)
        sock.soTimeout = 0
    }

    override fun read(buf: ByteArray): Int {
        val d = DatagramPacket(buf, buf.size)
        sock.receive(d)
        if (peer == null || host.isBlank()) peer = InetSocketAddress(d.address, d.port)
        return d.length
    }

    override fun write(b: ByteArray) {
        val to = peer ?: return
        sock.send(DatagramPacket(b, b.size, to))
    }

    override fun shut() { if (this::sock.isInitialized) sock.close() }
}

/** Private, link-local or loopback IPv4 literal: an address on the board's WiFi, not on the internet. */
fun isLocal(host: String): Boolean {
    val p = host.split('.').mapNotNull { it.toIntOrNull() }
    if (p.size != 4) return false
    return p[0] == 10 || p[0] == 127 || (p[0] == 192 && p[1] == 168) ||
        (p[0] == 172 && p[1] in 16..31) || (p[0] == 169 && p[1] == 254)
}

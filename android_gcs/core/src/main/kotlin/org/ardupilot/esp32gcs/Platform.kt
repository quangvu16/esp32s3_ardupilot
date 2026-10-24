package org.ardupilot.esp32gcs

import java.net.DatagramSocket
import java.net.Socket
import java.net.URL
import java.net.URLConnection

/**
 * What the shared code (MAVLink, vehicle, links, NTRIP, LiDAR, patrol) needs
 * from the app around it. The Android app and the Windows app each fill it in
 * at start-up; the defaults suit a plain JVM.
 */
object Platform {
    /** Monotonic milliseconds (Android: SystemClock.elapsedRealtime). */
    @Volatile var clock: () -> Long = { System.nanoTime() / 1_000_000 }
    /** Runs on the UI thread (Android: the main Looper; Windows: the Swing event thread). */
    @Volatile var post: (Runnable) -> Unit = { it.run() }
    @Volatile var logError: (String, Throwable) -> Unit = { m, e -> System.err.println(m); e.printStackTrace() }
    @Volatile var net: NetRoutes = NetRoutes()
}

/**
 * Which network a socket leaves through. On a PC the routing table already sends
 * 192.168.4.x to the board's WiFi, so nothing is needed. On Android the board's
 * WiFi has no internet and mobile data is the default network, so board sockets
 * are bound to WiFi and internet sockets (tiles, NTRIP) to mobile data.
 */
open class NetRoutes {
    open fun bindLocal(s: Socket) {}
    open fun bindLocal(s: DatagramSocket) {}
    open fun bindInternet(s: Socket) {}
    open fun openInternet(url: URL): URLConnection = url.openConnection()
}

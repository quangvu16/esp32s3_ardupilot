package org.ardupilot.esp32gcs

import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import java.net.DatagramSocket
import java.net.Socket
import java.net.URL
import java.net.URLConnection

/** Fills in [Platform] for Android. Call before creating the Vehicle. */
fun installAndroidPlatform(ctx: Context) {
    val main = Handler(Looper.getMainLooper())
    Platform.clock = { SystemClock.elapsedRealtime() }
    Platform.post = { main.post(it) }
    Platform.logError = { m, e -> android.util.Log.e("Gcs", m, e) }
    Platform.net = AndroidNetRoutes(ctx.applicationContext)
}

/** The board's WiFi has no internet: MAVLink is bound to WiFi, tiles and NTRIP to the network that has internet. */
class AndroidNetRoutes(private val ctx: Context) : NetRoutes() {
    private fun cm() = ctx.getSystemService(ConnectivityManager::class.java)

    @Suppress("DEPRECATION")
    private fun wifi(): Network? = cm()?.let { cm ->
        cm.allNetworks.firstOrNull { cm.getNetworkCapabilities(it)?.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) == true }
    }

    @Suppress("DEPRECATION")
    private fun internet(): Network? = cm()?.let { cm ->
        cm.allNetworks.firstOrNull {
            cm.getNetworkCapabilities(it)?.hasCapability(NetworkCapabilities.NET_CAPABILITY_VALIDATED) == true
        } ?: cm.activeNetwork
    }

    override fun bindLocal(s: Socket) { wifi()?.bindSocket(s) }
    override fun bindLocal(s: DatagramSocket) { wifi()?.bindSocket(s) }
    override fun bindInternet(s: Socket) { internet()?.bindSocket(s) }
    override fun openInternet(url: URL): URLConnection = internet()?.openConnection(url) ?: url.openConnection()
}

package org.ardupilot.esp32gcs

import android.content.Context
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import kotlin.math.abs

/** One mission item as ArduPilot stores it (lat/lon in degrees, frame relative-alt). */
data class MissionItem(
    var command: Int = Mav.CMD_NAV_WAYPOINT,
    var lat: Double = 0.0,
    var lon: Double = 0.0,
    var alt: Float = 0f,
    var p1: Float = 0f, var p2: Float = 0f, var p3: Float = 0f, var p4: Float = 0f,
    var frame: Int = Mav.FRAME_GLOBAL_RELATIVE_ALT_INT,
    var autocontinue: Int = 1,
) {
    /** Commands whose x/y are a position (drawn on the map). */
    val hasPosition get() = lat != 0.0 || lon != 0.0
}

class Param(val name: String, var value: Float, var type: Int, var index: Int)

/** Things the UI shows once (toasts, dialogs, logs), delivered on the main thread. */
sealed class VehicleEvent {
    data class Text(val severity: Int, val text: String) : VehicleEvent()
    data class Link(val text: String) : VehicleEvent()
    data class ParamsDone(val received: Int, val total: Int) : VehicleEvent()
    data class ParamWritten(val name: String, val ok: Boolean, val value: Float) : VehicleEvent()
    /** [type]: [Vehicle.MISSION_TYPE_MISSION] or [Vehicle.MISSION_TYPE_FENCE]. */
    data class MissionUploaded(val ok: Boolean, val text: String, val type: Int = 0) : VehicleEvent()
    data class MissionDownloaded(val items: List<MissionItem>?, val text: String, val type: Int = 0) : VehicleEvent()
    data class CommandAck(val command: Int, val result: Int) : VehicleEvent()
    data class AccelCalPos(val position: Int) : VehicleEvent()
    data class MagCalReport(val status: Int, val fitness: Float) : VehicleEvent()
}

/**
 * The autopilot as seen from the phone. Telemetry fields are written on the link
 * thread and read by the UI on a timer; the parameter, mission and command
 * protocols run on [timer] and the link thread, guarded by `synchronized(this)`.
 */
class Vehicle(private val ctx: Context) {
    var onEvent: ((VehicleEvent) -> Unit)? = null

    private val main = Handler(Looper.getMainLooper())
    private val enc = MavEncoder()
    private val parser = MavParser()
    private val timer = Executors.newSingleThreadScheduledExecutor { Thread(it, "vehicle").apply { isDaemon = true } }

    @Volatile var link: Link? = null
        private set
    private var wantLink: (() -> Link)? = null
    private var reconnectAt = 0L
    @Volatile var linkText = "Not connected"
        private set

    // --- telemetry -------------------------------------------------------------
    @Volatile var sysId = 0
    @Volatile var compId = 1
    @Volatile var lastHeartbeat = 0L
    @Volatile var vehicleType = 0
    @Volatile var autopilot = 0
    @Volatile var armed = false
    @Volatile var customMode = -1
    @Volatile var systemStatus = 0
    @Volatile var roll = 0f
    @Volatile var pitch = 0f
    @Volatile var yaw = 0f
    @Volatile var yawRate = 0f
    @Volatile var groundSpeed = 0f
    @Volatile var heading = 0
    @Volatile var throttle = 0
    @Volatile var lat = 0.0
    @Volatile var lon = 0.0
    @Volatile var relAlt = 0f
    @Volatile var hasPosition = false
    @Volatile var gpsFix = 0
    @Volatile var gpsSats = 0
    @Volatile var gpsHdop = 0f
    @Volatile var gpsHAcc = 0f
    @Volatile var voltage = 0f
    @Volatile var current = 0f
    @Volatile var batteryPct = -1
    @Volatile var sensorsPresent = 0L
    @Volatile var sensorsHealth = 0L
    @Volatile var sensorsEnabled = 0L
    @Volatile var homeLat = 0.0
    @Volatile var homeLon = 0.0
    @Volatile var hasHome = false
    @Volatile var missionSeq = 0
    @Volatile var wpDist = 0
    @Volatile var navBearing = 0
    @Volatile var targetBearing = 0
    @Volatile var xtrack = 0f
    @Volatile var ekfFlags = 0
    @Volatile var ekfVariance = FloatArray(5)
    val rc = IntArray(16)
    @Volatile var rcCount = 0
    @Volatile var rssi = 0
    @Volatile var rcTime = 0L
    /** Proximity sectors (DISTANCE_SENSOR orientation 0-7 = 0°, 45° … 315°), metres, 0 = none. */
    val proximity = FloatArray(8)
    val proximityTime = LongArray(8)
    @Volatile var proximityMax = 8f
    @Volatile var magCalPct = IntArray(3)
    @Volatile var magCalStatus = IntArray(3)
    @Volatile var rxPackets = 0
    @Volatile var rtcmBytes = 0L

    val messages = ArrayList<VehicleEvent.Text>()      // newest last, guarded by itself

    val connected get() = SystemClock.elapsedRealtime() - lastHeartbeat < 4000
    val modeName get() = ROVER_MODES[customMode] ?: if (customMode < 0) "—" else "Mode $customMode"

    // --- parameters ------------------------------------------------------------
    val params = sortedMapOf<String, Param>()      // guarded by this
    @Volatile var paramCount = 0
    @Volatile var paramsFetching = false
    private var paramLastRx = 0L
    private var paramRounds = 0
    private val paramIndices = HashSet<Int>()
    private class PendingSet(val name: String, val value: Float, var tries: Int, var sentAt: Long)
    private val paramSets = LinkedHashMap<String, PendingSet>()
    val paramWritesPending get() = synchronized(this) { paramSets.size }

    // --- mission ---------------------------------------------------------------
    private enum class Tx { IDLE, UPLOAD, DOWNLOAD }
    private var tx = Tx.IDLE
    private var txItems = ArrayList<MissionItem>()
    private var txCount = 0
    private var txNext = 0
    private var txSentAt = 0L
    private var txTries = 0
    private var txType = MISSION_TYPE_MISSION
    @Volatile var missionProgress = ""
        private set

    // --- commands --------------------------------------------------------------
    private class PendingCmd(val cmd: Int, val p: FloatArray, var tries: Int, var sentAt: Long)
    private val commands = HashMap<Int, PendingCmd>()

    private var lastStreamsAt = 0L
    private var lastAttitudeAt = 0L
    private var lastHeartbeatTx = 0L

    init {
        timer.scheduleAtFixedRate({ tick() }, 100, 100, TimeUnit.MILLISECONDS)
    }

    // =========================================================================
    // Link
    // =========================================================================

    fun connect(make: () -> Link) {
        disconnect()
        wantLink = make
        reconnectAt = 0L
    }

    fun disconnect() {
        wantLink = null
        link?.close()
        link = null
        lastHeartbeat = 0L
        linkText = "Not connected"
    }

    private fun openLink() {
        val l = wantLink?.invoke() ?: return
        linkText = "Connecting to ${l.name}…"
        l.onBytes = { b, n ->
            parser.feed(b, n) { handle(it) }
        }
        l.onClosed = { why ->
            if (link === l) {
                link = null
                linkText = "Lost ${l.name}: $why"
                post(VehicleEvent.Link(linkText))
                reconnectAt = SystemClock.elapsedRealtime() + 2000
            }
        }
        link = l
        l.start()
        lastStreamsAt = 0L
    }

    private fun send(msgId: Int, b: java.nio.ByteBuffer) {
        link?.send(enc.frame(msgId, b))
    }

    private fun post(e: VehicleEvent) = main.post { onEvent?.invoke(e) }

    private fun tick() {
        try { tickInner() } catch (e: Exception) { android.util.Log.e("Vehicle", "tick", e) }
    }

    private fun tickInner() {
        val now = SystemClock.elapsedRealtime()
        if (link == null && wantLink != null && now >= reconnectAt) {
            reconnectAt = now + 3000
            openLink()
        }
        val l = link ?: return
        if (l.open && linkText.startsWith("Connecting")) linkText = "Waiting for heartbeat on ${l.name}"
        if (now - lastHeartbeatTx >= 1000) {
            lastHeartbeatTx = now
            sendHeartbeat()
        }
        if (connected) {
            linkText = l.name
            // Ask for telemetry once, and again if attitude stops (e.g. after a reboot).
            if (now - lastStreamsAt > 15000 || (now - lastAttitudeAt > 3000 && now - lastStreamsAt > 3000)) {
                lastStreamsAt = now
                requestStreams()
            }
        }
        synchronized(this) {
            tickParams(now)
            tickMission(now)
            tickCommands(now)
        }
    }

    private fun sendHeartbeat() {
        val b = Mav.payload(Mav.HEARTBEAT)
        b.put(4, 6)          // MAV_TYPE_GCS
        b.put(5, 8)          // MAV_AUTOPILOT_INVALID
        b.put(7, 4)          // MAV_STATE_ACTIVE
        b.put(8, 3)
        send(Mav.HEARTBEAT, b)
    }

    /** Stream rates kept low: the ESP32's WiFi is shared with the GPS corrections. */
    private fun requestStreams() {
        val rates = intArrayOf(
            2 /* EXTENDED_STATUS */, 2,
            3 /* RC_CHANNELS */, 2,
            6 /* POSITION */, 3,
            10 /* EXTRA1: attitude */, 8,
            11 /* EXTRA2: VFR_HUD */, 4,
            12 /* EXTRA3: distance sensors, EKF, battery, mag cal */, 3,
        )
        for (i in rates.indices step 2) {
            val b = Mav.payload(Mav.REQUEST_DATA_STREAM)
            b.putShort(0, rates[i + 1].toShort())
            b.put(2, sysId.toByte()); b.put(3, compId.toByte())
            b.put(4, rates[i].toByte()); b.put(5, 1)
            send(Mav.REQUEST_DATA_STREAM, b)
        }
    }

    // =========================================================================
    // Receive
    // =========================================================================

    private fun handle(m: MavPacket) {
        rxPackets++
        if (m.msgId == Mav.HEARTBEAT) {
            val type = m.u8(4)
            val ap = m.u8(5)
            // Only the autopilot: not other GCSs (type 6), not peripherals (autopilot INVALID).
            if (type == 6 || ap == 8) return
            if (sysId == 0 || sysId != m.sysId) { sysId = m.sysId; compId = m.compId }
            if (m.sysId != sysId || m.compId != compId) return
            val first = !connected
            customMode = m.u32(0).toInt()
            vehicleType = type
            autopilot = ap
            armed = m.u8(6) and 0x80 != 0
            systemStatus = m.u8(7)
            lastHeartbeat = SystemClock.elapsedRealtime()
            if (first) post(VehicleEvent.Link("Connected to system $sysId"))
            return
        }
        if (m.sysId != sysId) return
        when (m.msgId) {
            Mav.SYS_STATUS -> {
                sensorsPresent = m.u32(0); sensorsEnabled = m.u32(4); sensorsHealth = m.u32(8)
                voltage = m.u16(14) / 1000f
                current = if (m.i16(16) == -1) 0f else m.i16(16) / 100f
                batteryPct = m.i8(30)
            }
            Mav.ATTITUDE -> {
                roll = m.f32(4); pitch = m.f32(8); yaw = m.f32(12); yawRate = m.f32(24)
                lastAttitudeAt = SystemClock.elapsedRealtime()
            }
            Mav.GLOBAL_POSITION_INT -> {
                val la = m.i32(4); val lo = m.i32(8)
                if (la != 0 || lo != 0) {
                    lat = la / 1e7; lon = lo / 1e7; hasPosition = true
                }
                relAlt = m.i32(16) / 1000f
            }
            Mav.GPS_RAW_INT -> {
                gpsFix = m.u8(28); gpsSats = m.u8(29)
                gpsHdop = if (m.u16(20) == 0xffff) 99.99f else m.u16(20) / 100f
                gpsHAcc = m.u32(34) / 1000f
            }
            Mav.VFR_HUD -> { groundSpeed = m.f32(4); heading = m.i16(16); throttle = m.u16(18) }
            Mav.HOME_POSITION -> {
                homeLat = m.i32(0) / 1e7; homeLon = m.i32(4) / 1e7; hasHome = homeLat != 0.0 || homeLon != 0.0
            }
            Mav.MISSION_CURRENT -> missionSeq = m.u16(0)
            Mav.NAV_CONTROLLER_OUTPUT -> {
                navBearing = m.i16(20); targetBearing = m.i16(22); wpDist = m.u16(24); xtrack = m.f32(16)
            }
            Mav.RC_CHANNELS -> {
                val n = minOf(m.u8(40), 16)
                for (i in 0 until 16) rc[i] = m.u16(4 + 2 * i)
                rcCount = n; rssi = m.u8(41); rcTime = SystemClock.elapsedRealtime()
            }
            Mav.EKF_STATUS_REPORT -> {
                ekfFlags = m.u16(20)
                ekfVariance = floatArrayOf(m.f32(0), m.f32(4), m.f32(8), m.f32(12), m.f32(16))
            }
            Mav.DISTANCE_SENSOR -> {
                val orient = m.u8(12)
                if (orient in 0..7) {
                    val d = m.u16(8); val max = m.u16(6)
                    proximity[orient] = if (d == 0 || d >= max) 0f else d / 100f
                    proximityTime[orient] = SystemClock.elapsedRealtime()
                    if (max in 1..5000) proximityMax = max / 100f
                }
            }
            Mav.BATTERY_STATUS -> if (m.u8(32) == 0 && m.i8(35) >= 0) batteryPct = m.i8(35)
            Mav.STATUSTEXT -> {
                val t = VehicleEvent.Text(m.u8(0), m.str(1, 50))
                synchronized(messages) {
                    messages.add(t)
                    if (messages.size > 300) messages.subList(0, 100).clear()
                }
                post(t)
            }
            Mav.MAG_CAL_PROGRESS -> {
                val id = m.u8(12)
                if (id < 3) { magCalPct[id] = m.u8(16); magCalStatus[id] = m.u8(14) }
            }
            Mav.MAG_CAL_REPORT -> {
                val id = m.u8(40)
                if (id < 3) { magCalStatus[id] = m.u8(42); magCalPct[id] = 100 }
                post(VehicleEvent.MagCalReport(m.u8(42), m.f32(0)))
            }
            Mav.COMMAND_LONG -> {
                // The autopilot asks us to pose the vehicle during accel calibration.
                if (m.u16(28) == Mav.CMD_ACCELCAL_VEHICLE_POS) post(VehicleEvent.AccelCalPos(m.f32(0).toInt()))
            }
            Mav.COMMAND_ACK -> {
                val cmd = m.u16(0); val res = m.u8(2)
                if (res == 5 /* IN_PROGRESS */) return
                val was = synchronized(this) { commands.remove(cmd) }
                if (was != null) post(VehicleEvent.CommandAck(cmd, res))
            }
            Mav.PARAM_VALUE -> synchronized(this) { onParamValue(m) }
            Mav.MISSION_REQUEST_INT, Mav.MISSION_REQUEST, Mav.MISSION_COUNT,
            Mav.MISSION_ITEM_INT, Mav.MISSION_ACK -> synchronized(this) { onMission(m) }
        }
    }

    // =========================================================================
    // Commands and modes
    // =========================================================================

    fun command(cmd: Int, vararg p: Float) {
        val pp = FloatArray(7) { p.getOrElse(it) { 0f } }
        synchronized(this) { commands[cmd] = PendingCmd(cmd, pp, 1, SystemClock.elapsedRealtime()) }
        sendCommandLong(cmd, pp, 0)
    }

    private fun sendCommandLong(cmd: Int, p: FloatArray, confirmation: Int) {
        val b = Mav.payload(Mav.COMMAND_LONG)
        for (i in 0 until 7) b.putFloat(4 * i, p[i])
        b.putShort(28, cmd.toShort())
        b.put(30, sysId.toByte()); b.put(31, compId.toByte()); b.put(32, confirmation.toByte())
        send(Mav.COMMAND_LONG, b)
    }

    private fun tickCommands(now: Long) {
        val it = commands.values.iterator()
        while (it.hasNext()) {
            val c = it.next()
            if (now - c.sentAt < 1500) continue
            if (c.tries >= 3) {
                it.remove()
                post(VehicleEvent.CommandAck(c.cmd, -1))
            } else {
                c.tries++; c.sentAt = now
                sendCommandLong(c.cmd, c.p, c.tries - 1)
            }
        }
    }

    fun setMode(mode: Int) = command(Mav.CMD_DO_SET_MODE, 1f /* custom mode enabled */, mode.toFloat())
    fun arm(on: Boolean, force: Boolean = false) =
        command(Mav.CMD_COMPONENT_ARM_DISARM, if (on) 1f else 0f, if (force) 21196f else 0f)
    fun reboot() = command(Mav.CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1f)
    fun setHomeHere() = command(Mav.CMD_DO_SET_HOME, 1f)
    fun startMission() = command(Mav.CMD_MISSION_START)

    fun setHome(lat: Double, lon: Double) {
        val b = Mav.payload(Mav.COMMAND_INT)
        b.putInt(16, (lat * 1e7).toInt()); b.putInt(20, (lon * 1e7).toInt())
        b.putShort(28, Mav.CMD_DO_SET_HOME.toShort())
        b.put(30, sysId.toByte()); b.put(31, compId.toByte()); b.put(32, Mav.FRAME_GLOBAL_RELATIVE_ALT_INT.toByte())
        send(Mav.COMMAND_INT, b)
    }

    fun changeSpeed(mps: Float) = command(Mav.CMD_DO_CHANGE_SPEED, 1f /* ground speed */, mps, -1f)

    /** Drive to a point in Guided mode. */
    fun goTo(lat: Double, lon: Double) {
        val b = Mav.payload(Mav.SET_POSITION_TARGET_GLOBAL_INT)
        b.putInt(4, (lat * 1e7).toInt()); b.putInt(8, (lon * 1e7).toInt())
        b.putShort(48, 0x0FF8)       // use position only
        b.put(50, sysId.toByte()); b.put(51, compId.toByte())
        b.put(52, Mav.FRAME_GLOBAL_RELATIVE_ALT_INT.toByte())
        send(Mav.SET_POSITION_TARGET_GLOBAL_INT, b)
    }

    fun setCurrentWaypoint(seq: Int) {
        val b = Mav.payload(Mav.MISSION_SET_CURRENT)
        b.putShort(0, seq.toShort()); b.put(2, sysId.toByte()); b.put(3, compId.toByte())
        send(Mav.MISSION_SET_CURRENT, b)
    }

    /**
     * Steering and throttle from the phone, -1..1 each. Rover maps MANUAL_CONTROL
     * y to steering and z to throttle (an RC override that times out after
     * RC_OVERRIDE_TIME). Needs SYSID_MYGCS = 255.
     */
    fun manualControl(steer: Float, thr: Float) {
        val b = Mav.payload(Mav.MANUAL_CONTROL)
        b.putShort(0, Short.MAX_VALUE)                      // x unused
        b.putShort(2, (steer.coerceIn(-1f, 1f) * 1000).toInt().toShort())
        b.putShort(4, (thr.coerceIn(-1f, 1f) * 1000).toInt().toShort())
        b.putShort(6, Short.MAX_VALUE)                      // r unused
        b.put(10, sysId.toByte())
        send(Mav.MANUAL_CONTROL, b)
    }

    // --- calibration ------------------------------------------------------------
    fun startAccelCal() = command(Mav.CMD_PREFLIGHT_CALIBRATION, 0f, 0f, 0f, 0f, 1f)
    fun accelCalLevelOnly() = command(Mav.CMD_PREFLIGHT_CALIBRATION, 0f, 0f, 0f, 0f, 2f)
    fun accelCalPosition(pos: Int) {
        // Answers the vehicle's request; ArduPilot does not ack it, so send it untracked.
        sendCommandLong(Mav.CMD_ACCELCAL_VEHICLE_POS, floatArrayOf(pos.toFloat(), 0f, 0f, 0f, 0f, 0f, 0f), 0)
    }
    fun startMagCal() {
        magCalPct = IntArray(3); magCalStatus = IntArray(3)
        command(Mav.CMD_DO_START_MAG_CAL, 0f, 1f /* retry */, 1f /* autosave */, 0f, 0f)
    }
    fun acceptMagCal() = command(Mav.CMD_DO_ACCEPT_MAG_CAL)
    fun cancelMagCal() = command(Mav.CMD_DO_CANCEL_MAG_CAL)

    // --- RTK corrections ------------------------------------------------------------
    private var rtcmSeq = 0

    /** Sends one RTCM3 frame as GPS_RTCM_DATA, in up to 4 fragments of 180 bytes. */
    fun injectRtcm(frame: ByteArray) {
        rtcmBytes += frame.size
        val seq = rtcmSeq; rtcmSeq = (rtcmSeq + 1) and 0x1f
        if (frame.size <= 180) {
            sendRtcm(seq shl 3, frame, 0, frame.size)
            return
        }
        if (frame.size > 4 * 180) return
        var off = 0; var frag = 0
        while (off < frame.size) {
            val n = minOf(180, frame.size - off)
            sendRtcm(1 or (frag shl 1) or (seq shl 3), frame, off, n)
            off += n; frag++
        }
        // A frame that is an exact multiple of 180 must end with an empty fragment.
        if (frame.size % 180 == 0 && frag < 4) sendRtcm(1 or (frag shl 1) or (seq shl 3), frame, 0, 0)
    }

    private fun sendRtcm(flags: Int, data: ByteArray, off: Int, n: Int) {
        val b = Mav.payload(Mav.GPS_RTCM_DATA)
        b.put(0, flags.toByte()); b.put(1, n.toByte())
        for (i in 0 until n) b.put(2 + i, data[off + i])
        send(Mav.GPS_RTCM_DATA, b)
    }

    // =========================================================================
    // Parameters
    // =========================================================================

    fun fetchParams() = synchronized(this) {
        params.clear(); paramIndices.clear(); paramCount = 0
        paramsFetching = true; paramRounds = 0
        paramLastRx = SystemClock.elapsedRealtime()
        val b = Mav.payload(Mav.PARAM_REQUEST_LIST)
        b.put(0, sysId.toByte()); b.put(1, compId.toByte())
        send(Mav.PARAM_REQUEST_LIST, b)
    }

    fun paramValue(name: String): Float? = synchronized(this) { params[name]?.value }

    /** Queues a write; the result arrives as [VehicleEvent.ParamWritten]. */
    fun setParam(name: String, value: Float) = synchronized(this) {
        paramSets[name] = PendingSet(name, value, 0, 0L)
    }

    private fun sendParamSet(name: String, value: Float) {
        val b = Mav.payload(Mav.PARAM_SET)
        b.putFloat(0, value)
        b.put(4, sysId.toByte()); b.put(5, compId.toByte())
        Mav.putString(b, 6, name, 16)
        b.put(22, (params[name]?.type ?: 9).toByte())
        send(Mav.PARAM_SET, b)
    }

    private fun requestParam(index: Int) {
        val b = Mav.payload(Mav.PARAM_REQUEST_READ)
        b.putShort(0, index.toShort())
        b.put(2, sysId.toByte()); b.put(3, compId.toByte())
        send(Mav.PARAM_REQUEST_READ, b)
    }

    private fun onParamValue(m: MavPacket) {
        val name = m.str(8, 16)
        val value = m.f32(0)
        val count = m.u16(4)
        val index = m.u16(6)
        val p = params[name]
        if (p == null) params[name] = Param(name, value, m.u8(24), index)
        else { p.value = value; if (index != 0xffff) p.index = index }
        if (index != 0xffff) {
            paramCount = count
            paramIndices.add(index)
            paramLastRx = SystemClock.elapsedRealtime()
            if (paramsFetching && paramIndices.size >= count) {
                paramsFetching = false
                post(VehicleEvent.ParamsDone(paramIndices.size, count))
            }
        }
        paramSets[name]?.let { s ->
            if (s.sentAt == 0L) return@let
            paramSets.remove(name)
            val ok = abs(value - s.value) <= maxOf(1e-6f, abs(s.value) * 1e-5f)
            post(VehicleEvent.ParamWritten(name, ok, value))
        }
    }

    private fun tickParams(now: Long) {
        if (paramsFetching && now - paramLastRx > 1500) {
            if (paramCount == 0 || paramRounds >= 8) {
                paramsFetching = false
                post(VehicleEvent.ParamsDone(paramIndices.size, paramCount))
            } else {
                // Ask again for the ones lost on the way, a few at a time.
                paramRounds++
                paramLastRx = now
                var asked = 0
                for (i in 0 until paramCount) {
                    if (i !in paramIndices) { requestParam(i); if (++asked >= 20) break }
                }
            }
        }
        // Writes: up to 4 in flight, each retried twice.
        var inFlight = 0
        val it = paramSets.values.iterator()
        while (it.hasNext()) {
            val s = it.next()
            if (s.sentAt != 0L && now - s.sentAt < 1200) { inFlight++; continue }
            if (s.tries >= 3) {
                it.remove()
                post(VehicleEvent.ParamWritten(s.name, false, params[s.name]?.value ?: Float.NaN))
                continue
            }
            if (inFlight >= 4) break
            s.tries++; s.sentAt = now; inFlight++
            sendParamSet(s.name, s.value)
        }
    }

    // =========================================================================
    // Mission and fence transfers: the same protocol, told apart by mission_type.
    // Waypoint lists start with home (item 0, as ArduPilot stores it); fences don't.
    // =========================================================================

    val missionBusy get() = synchronized(this) { tx != Tx.IDLE }

    fun uploadMission(items: List<MissionItem>) = synchronized(this) {
        if (tx != Tx.IDLE) return@synchronized
        val home = MissionItem(
            lat = if (hasHome) homeLat else if (hasPosition) lat else items.firstOrNull()?.lat ?: 0.0,
            lon = if (hasHome) homeLon else if (hasPosition) lon else items.firstOrNull()?.lon ?: 0.0,
            frame = 0 /* MAV_FRAME_GLOBAL */)
        startUpload(MISSION_TYPE_MISSION, listOf(home) + items)
    }

    /** Fence items as [Fence.toItems] makes them; an empty list clears the fence. */
    fun uploadFence(items: List<MissionItem>) = synchronized(this) {
        if (tx != Tx.IDLE) return@synchronized
        startUpload(MISSION_TYPE_FENCE, items)
    }

    fun downloadMission() = startDownload(MISSION_TYPE_MISSION)
    fun downloadFence() = startDownload(MISSION_TYPE_FENCE)

    private fun startUpload(type: Int, items: List<MissionItem>) {
        txItems = ArrayList(items.map { it.copy() })
        txType = type
        tx = Tx.UPLOAD; txTries = 0
        missionProgress = "Uploading…"
        sendMissionCount()
    }

    private fun startDownload(type: Int) = synchronized(this) {
        if (tx != Tx.IDLE) return@synchronized
        txType = type
        tx = Tx.DOWNLOAD; txTries = 0; txCount = -1; txNext = 0
        txItems = ArrayList()
        missionProgress = "Downloading…"
        sendMissionRequestList()
    }

    fun clearMission() {
        val b = Mav.payload(Mav.MISSION_CLEAR_ALL)
        b.put(0, sysId.toByte()); b.put(1, compId.toByte()); b.put(2, MISSION_TYPE_MISSION.toByte())
        send(Mav.MISSION_CLEAR_ALL, b)
    }

    private fun sendMissionCount() {
        val b = Mav.payload(Mav.MISSION_COUNT)
        b.putShort(0, txItems.size.toShort()); b.put(2, sysId.toByte()); b.put(3, compId.toByte())
        b.put(4, txType.toByte())
        send(Mav.MISSION_COUNT, b)
        txSentAt = SystemClock.elapsedRealtime()
    }

    private fun sendMissionRequestList() {
        val b = Mav.payload(Mav.MISSION_REQUEST_LIST)
        b.put(0, sysId.toByte()); b.put(1, compId.toByte()); b.put(2, txType.toByte())
        send(Mav.MISSION_REQUEST_LIST, b)
        txSentAt = SystemClock.elapsedRealtime()
    }

    private fun sendMissionRequest(seq: Int) {
        val b = Mav.payload(Mav.MISSION_REQUEST_INT)
        b.putShort(0, seq.toShort()); b.put(2, sysId.toByte()); b.put(3, compId.toByte())
        b.put(4, txType.toByte())
        send(Mav.MISSION_REQUEST_INT, b)
        txSentAt = SystemClock.elapsedRealtime()
    }

    private fun sendMissionItem(seq: Int) {
        val it = txItems[seq]
        val b = Mav.payload(Mav.MISSION_ITEM_INT)
        b.putFloat(0, it.p1); b.putFloat(4, it.p2); b.putFloat(8, it.p3); b.putFloat(12, it.p4)
        b.putInt(16, (it.lat * 1e7).toInt()); b.putInt(20, (it.lon * 1e7).toInt()); b.putFloat(24, it.alt)
        b.putShort(28, seq.toShort()); b.putShort(30, it.command.toShort())
        b.put(32, sysId.toByte()); b.put(33, compId.toByte())
        b.put(34, it.frame.toByte()); b.put(35, if (seq == 0 && txType == MISSION_TYPE_MISSION) 1 else 0)
        b.put(36, it.autocontinue.toByte()); b.put(37, txType.toByte())
        send(Mav.MISSION_ITEM_INT, b)
        txSentAt = SystemClock.elapsedRealtime()
    }

    private fun sendMissionAck(result: Int) {
        val b = Mav.payload(Mav.MISSION_ACK)
        b.put(0, sysId.toByte()); b.put(1, compId.toByte()); b.put(2, result.toByte()); b.put(3, txType.toByte())
        send(Mav.MISSION_ACK, b)
    }

    private fun onMission(m: MavPacket) {
        if (tx == Tx.IDLE) return
        // mission_type sits after the fields every version has; ignore the other list's traffic.
        val type = when (m.msgId) {
            Mav.MISSION_REQUEST_INT, Mav.MISSION_REQUEST, Mav.MISSION_COUNT -> m.u8(4)
            Mav.MISSION_ITEM_INT -> m.u8(37)
            Mav.MISSION_ACK -> m.u8(3)
            else -> return
        }
        if (type != txType) return
        val fence = txType == MISSION_TYPE_FENCE
        val what = if (fence) "Fence" else "Mission"
        when (tx) {
            Tx.UPLOAD -> when (m.msgId) {
                Mav.MISSION_REQUEST_INT, Mav.MISSION_REQUEST -> {
                    val seq = m.u16(0)
                    if (seq < txItems.size) {
                        txTries = 0
                        missionProgress = "Uploading ${seq + 1}/${txItems.size}"
                        sendMissionItem(seq)
                    }
                }
                Mav.MISSION_ACK -> {
                    val result = m.u8(2)
                    tx = Tx.IDLE
                    missionProgress = ""
                    val n = if (fence) txItems.size else txItems.size - 1
                    post(VehicleEvent.MissionUploaded(result == 0,
                        if (result == 0) "$what uploaded: $n items" else "$what upload refused: ${missionResult(result)}", txType))
                }
            }
            Tx.DOWNLOAD -> when (m.msgId) {
                Mav.MISSION_COUNT -> if (txCount < 0) {
                    txCount = m.u16(0); txTries = 0
                    if (txCount == 0) {
                        tx = Tx.IDLE; missionProgress = ""
                        sendMissionAck(0)
                        post(VehicleEvent.MissionDownloaded(emptyList(), "No ${what.lowercase()} on the vehicle", txType))
                    } else sendMissionRequest(0)
                }
                Mav.MISSION_ITEM_INT -> {
                    val seq = m.u16(28)
                    if (seq != txNext) return
                    txItems.add(MissionItem(
                        command = m.u16(30), lat = m.i32(16) / 1e7, lon = m.i32(20) / 1e7, alt = m.f32(24),
                        p1 = m.f32(0), p2 = m.f32(4), p3 = m.f32(8), p4 = m.f32(12),
                        frame = m.u8(34), autocontinue = m.u8(36)))
                    txNext++; txTries = 0
                    missionProgress = "Downloading $txNext/$txCount"
                    if (txNext < txCount) sendMissionRequest(txNext)
                    else {
                        tx = Tx.IDLE; missionProgress = ""
                        sendMissionAck(0)
                        // A mission's item 0 is home: report the rest.
                        val items = if (fence) txItems.toList() else txItems.drop(1)
                        post(VehicleEvent.MissionDownloaded(items, "$what downloaded: ${items.size} items", txType))
                    }
                }
            }
            Tx.IDLE -> {}
        }
    }

    private fun tickMission(now: Long) {
        if (tx == Tx.IDLE || now - txSentAt < 1500) return
        if (++txTries > 5) {
            val up = tx == Tx.UPLOAD
            tx = Tx.IDLE; missionProgress = ""
            val what = if (txType == MISSION_TYPE_FENCE) "Fence" else "Mission"
            if (up) post(VehicleEvent.MissionUploaded(false, "$what upload timed out", txType))
            else post(VehicleEvent.MissionDownloaded(null, "$what download timed out", txType))
            return
        }
        when (tx) {
            Tx.UPLOAD -> sendMissionCount()
            Tx.DOWNLOAD -> if (txCount < 0) sendMissionRequestList() else sendMissionRequest(txNext)
            Tx.IDLE -> {}
        }
    }

    private fun missionResult(t: Int) = when (t) {
        1 -> "error (see the Log tab)"; 2 -> "unsupported frame"; 3 -> "unsupported command"; 4 -> "no space"
        5 -> "invalid"; in 6..12 -> "invalid param${t - 5}"; 13 -> "invalid sequence"; 14 -> "denied"; 15 -> "cancelled"
        else -> "code $t"
    }

    fun close() {
        disconnect()
        timer.shutdownNow()
    }

    companion object {
        const val MISSION_TYPE_MISSION = 0
        const val MISSION_TYPE_FENCE = 1

        val ROVER_MODES = linkedMapOf(
            0 to "Manual", 1 to "Acro", 3 to "Steering", 4 to "Hold", 5 to "Loiter",
            6 to "Follow", 7 to "Simple", 8 to "Dock", 9 to "Circle", 10 to "Auto",
            11 to "RTL", 12 to "SmartRTL", 15 to "Guided", 16 to "Initialising",
        )
        val GPS_FIX = arrayOf("No GPS", "No fix", "2D fix", "3D fix", "DGPS", "RTK Float", "RTK Fixed", "Static", "PPP")
        val SEVERITY = arrayOf("EMERG", "ALERT", "CRIT", "ERROR", "WARN", "NOTICE", "INFO", "DEBUG")

        fun cmdResult(r: Int) = when (r) {
            0 -> "accepted"; 1 -> "temporarily rejected"; 2 -> "denied"; 3 -> "unsupported"
            4 -> "failed"; 5 -> "in progress"; 6 -> "cancelled"; -1 -> "no answer"; else -> "result $r"
        }
    }
}

package org.ardupilot.esp32gcs

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * The bit of MAVLink this app needs: v1/v2 framing, the X.25 CRC, and the
 * messages it reads or sends. Field offsets, lengths and CRC extras are those
 * of ArduPilot 4.6's generated headers (build/<board>/libraries/GCS_MAVLink/
 * include/mavlink/v2.0); payloads are little-endian, fields sorted by size.
 */
object Mav {
    const val HEARTBEAT = 0
    const val SYS_STATUS = 1
    const val PARAM_REQUEST_READ = 20
    const val PARAM_REQUEST_LIST = 21
    const val PARAM_VALUE = 22
    const val PARAM_SET = 23
    const val GPS_RAW_INT = 24
    const val ATTITUDE = 30
    const val GLOBAL_POSITION_INT = 33
    const val MISSION_REQUEST = 40
    const val MISSION_SET_CURRENT = 41
    const val MISSION_CURRENT = 42
    const val MISSION_REQUEST_LIST = 43
    const val MISSION_COUNT = 44
    const val MISSION_CLEAR_ALL = 45
    const val MISSION_ACK = 47
    const val MISSION_REQUEST_INT = 51
    const val NAV_CONTROLLER_OUTPUT = 62
    const val RC_CHANNELS = 65
    const val REQUEST_DATA_STREAM = 66
    const val MANUAL_CONTROL = 69
    const val RC_CHANNELS_OVERRIDE = 70
    const val MISSION_ITEM_INT = 73
    const val VFR_HUD = 74
    const val COMMAND_INT = 75
    const val COMMAND_LONG = 76
    const val COMMAND_ACK = 77
    const val SET_POSITION_TARGET_GLOBAL_INT = 86
    const val DISTANCE_SENSOR = 132
    const val BATTERY_STATUS = 147
    const val MAG_CAL_PROGRESS = 191
    const val MAG_CAL_REPORT = 192
    const val EKF_STATUS_REPORT = 193
    const val GPS_RTCM_DATA = 233
    const val HOME_POSITION = 242
    const val STATUSTEXT = 253

    /** message id -> (CRC extra, full payload length) */
    val INFO: Map<Int, IntArray> = mapOf(
        HEARTBEAT to intArrayOf(50, 9),
        SYS_STATUS to intArrayOf(124, 31),
        PARAM_REQUEST_READ to intArrayOf(214, 20),
        PARAM_REQUEST_LIST to intArrayOf(159, 2),
        PARAM_VALUE to intArrayOf(220, 25),
        PARAM_SET to intArrayOf(168, 23),
        GPS_RAW_INT to intArrayOf(24, 52),
        ATTITUDE to intArrayOf(39, 28),
        GLOBAL_POSITION_INT to intArrayOf(104, 28),
        MISSION_REQUEST to intArrayOf(230, 5),
        MISSION_SET_CURRENT to intArrayOf(28, 4),
        MISSION_CURRENT to intArrayOf(28, 6),
        MISSION_REQUEST_LIST to intArrayOf(132, 3),
        MISSION_COUNT to intArrayOf(221, 5),
        MISSION_CLEAR_ALL to intArrayOf(232, 3),
        MISSION_ACK to intArrayOf(153, 4),
        MISSION_REQUEST_INT to intArrayOf(196, 5),
        NAV_CONTROLLER_OUTPUT to intArrayOf(183, 26),
        RC_CHANNELS to intArrayOf(118, 42),
        REQUEST_DATA_STREAM to intArrayOf(148, 6),
        MANUAL_CONTROL to intArrayOf(243, 30),
        RC_CHANNELS_OVERRIDE to intArrayOf(124, 38),
        MISSION_ITEM_INT to intArrayOf(38, 38),
        VFR_HUD to intArrayOf(20, 20),
        COMMAND_INT to intArrayOf(158, 35),
        COMMAND_LONG to intArrayOf(152, 33),
        COMMAND_ACK to intArrayOf(143, 10),
        SET_POSITION_TARGET_GLOBAL_INT to intArrayOf(5, 53),
        DISTANCE_SENSOR to intArrayOf(85, 39),
        BATTERY_STATUS to intArrayOf(154, 54),
        MAG_CAL_PROGRESS to intArrayOf(92, 27),
        MAG_CAL_REPORT to intArrayOf(36, 54),
        EKF_STATUS_REPORT to intArrayOf(71, 26),
        GPS_RTCM_DATA to intArrayOf(35, 182),
        HOME_POSITION to intArrayOf(104, 60),
        STATUSTEXT to intArrayOf(83, 54),
    )

    // MAV_CMD
    const val CMD_NAV_WAYPOINT = 16
    const val CMD_NAV_RETURN_TO_LAUNCH = 20
    const val CMD_DO_SET_MODE = 176
    const val CMD_DO_CHANGE_SPEED = 178
    const val CMD_DO_SET_HOME = 179
    const val CMD_PREFLIGHT_CALIBRATION = 241
    const val CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
    const val CMD_MISSION_START = 300
    const val CMD_COMPONENT_ARM_DISARM = 400
    const val CMD_DO_START_MAG_CAL = 42424
    const val CMD_DO_ACCEPT_MAG_CAL = 42425
    const val CMD_DO_CANCEL_MAG_CAL = 42426
    const val CMD_ACCELCAL_VEHICLE_POS = 42429

    const val FRAME_GLOBAL_RELATIVE_ALT = 3
    const val FRAME_GLOBAL_RELATIVE_ALT_INT = 6

    fun crcAccumulate(b: Int, crc: Int): Int {
        var tmp = (b xor crc) and 0xff
        tmp = (tmp xor (tmp shl 4)) and 0xff
        return ((crc shr 8) xor (tmp shl 8) xor (tmp shl 3) xor (tmp shr 4)) and 0xffff
    }

    fun crc(buf: ByteArray, from: Int, to: Int, extra: Int): Int {
        var c = 0xffff
        for (i in from until to) c = crcAccumulate(buf[i].toInt() and 0xff, c)
        return crcAccumulate(extra, c)
    }

    /** A zeroed little-endian buffer of the message's full payload length. */
    fun payload(msgId: Int): ByteBuffer =
        ByteBuffer.allocate(INFO.getValue(msgId)[1]).order(ByteOrder.LITTLE_ENDIAN)

    fun putString(b: ByteBuffer, at: Int, s: String, max: Int) {
        val bytes = s.toByteArray(Charsets.US_ASCII)
        for (i in 0 until minOf(bytes.size, max)) b.put(at + i, bytes[i])
    }

    fun getString(b: ByteBuffer, at: Int, max: Int): String {
        val sb = StringBuilder()
        for (i in 0 until max) {
            val c = b.get(at + i).toInt() and 0xff
            if (c == 0) break
            sb.append(c.toChar())
        }
        return sb.toString()
    }
}

/** A received message. [p] is the payload, zero-padded back to full length (v2 trims trailing zeros). */
class MavPacket(val msgId: Int, val sysId: Int, val compId: Int, val p: ByteBuffer) {
    fun u8(at: Int) = p.get(at).toInt() and 0xff
    fun i8(at: Int) = p.get(at).toInt()
    fun u16(at: Int) = p.getShort(at).toInt() and 0xffff
    fun i16(at: Int) = p.getShort(at).toInt()
    fun u32(at: Int) = p.getInt(at).toLong() and 0xffffffffL
    fun i32(at: Int) = p.getInt(at)
    fun f32(at: Int) = p.getFloat(at)
    fun str(at: Int, max: Int) = Mav.getString(p, at, max)
}

/** Splits a byte stream into packets. Frames of unknown messages are skipped (no CRC extra to check them). */
class MavParser {
    private var buf = ByteArray(4096)
    private var len = 0
    var crcErrors = 0
        private set

    fun feed(data: ByteArray, n: Int, out: (MavPacket) -> Unit) {
        if (len + n > buf.size) buf = buf.copyOf(maxOf(buf.size * 2, len + n))
        System.arraycopy(data, 0, buf, len, n)
        len += n
        var i = 0
        while (i < len) {
            val magic = buf[i].toInt() and 0xff
            if (magic != 0xFD && magic != 0xFE) { i++; continue }
            if (len - i < 2) break
            val plen = buf[i + 1].toInt() and 0xff
            val v2 = magic == 0xFD
            val hdr = if (v2) 10 else 6
            if (len - i < hdr) break
            val signed = v2 && (buf[i + 2].toInt() and 0x01) != 0
            val total = hdr + plen + 2 + if (signed) 13 else 0
            if (len - i < total) break
            val msgId = if (v2) (buf[i + 7].toInt() and 0xff) or
                    ((buf[i + 8].toInt() and 0xff) shl 8) or ((buf[i + 9].toInt() and 0xff) shl 16)
            else buf[i + 5].toInt() and 0xff
            val info = Mav.INFO[msgId]
            if (info == null) {
                // Can't check the CRC; trust the length only if the next byte looks like a frame start.
                val next = i + total
                if (next >= len || buf[next] == 0xFD.toByte() || buf[next] == 0xFE.toByte()) i = next else i++
                continue
            }
            val crc = Mav.crc(buf, i + 1, i + hdr + plen, info[0])
            val got = (buf[i + hdr + plen].toInt() and 0xff) or ((buf[i + hdr + plen + 1].toInt() and 0xff) shl 8)
            if (crc != got) { crcErrors++; i++; continue }
            val p = ByteBuffer.allocate(maxOf(info[1], plen)).order(ByteOrder.LITTLE_ENDIAN)
            p.put(buf, i + hdr, plen)
            p.clear()
            val sys = buf[i + if (v2) 5 else 3].toInt() and 0xff
            val comp = buf[i + if (v2) 6 else 4].toInt() and 0xff
            out(MavPacket(msgId, sys, comp, p))
            i += total
        }
        System.arraycopy(buf, i, buf, 0, len - i)
        len -= i
    }
}

/** Builds MAVLink 2 frames as the GCS (system 255, component 190, like Mission Planner). */
class MavEncoder(private val sysId: Int = 255, private val compId: Int = 190) {
    private var seq = 0

    @Synchronized
    fun frame(msgId: Int, payload: ByteBuffer): ByteArray {
        val full = payload.array()
        var n = full.size
        while (n > 1 && full[n - 1] == 0.toByte()) n--     // v2 payload truncation
        val f = ByteArray(10 + n + 2)
        f[0] = 0xFD.toByte()
        f[1] = n.toByte()
        f[2] = 0; f[3] = 0
        f[4] = seq.toByte(); seq = (seq + 1) and 0xff
        f[5] = sysId.toByte()
        f[6] = compId.toByte()
        f[7] = msgId.toByte(); f[8] = (msgId shr 8).toByte(); f[9] = (msgId shr 16).toByte()
        System.arraycopy(full, 0, f, 10, n)
        val crc = Mav.crc(f, 1, 10 + n, Mav.INFO.getValue(msgId)[0])
        f[10 + n] = crc.toByte(); f[11 + n] = (crc shr 8).toByte()
        return f
    }
}

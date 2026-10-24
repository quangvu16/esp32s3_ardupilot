package org.ardupilot.esp32gcs

import android.app.Activity
import android.app.AlertDialog
import android.content.Intent
import android.content.SharedPreferences
import android.graphics.Color
import android.graphics.Typeface
import android.net.Uri
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.text.Editable
import android.text.InputType
import android.text.TextWatcher
import android.view.Gravity
import android.view.View
import android.view.ViewGroup
import android.view.WindowManager
import android.widget.BaseAdapter
import android.widget.Button
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.HorizontalScrollView
import android.widget.LinearLayout
import android.widget.ListView
import android.widget.ProgressBar
import android.widget.ScrollView
import android.widget.SeekBar
import android.widget.Spinner
import android.widget.ArrayAdapter
import android.widget.TextView
import android.widget.Toast
import kotlin.math.abs

/**
 * A small Mission Planner for ArduRover on the ESP32-S3: Data (HUD, map, modes,
 * arm, drive stick), Plan (waypoints), Params (full list, .parm files), Setup
 * (connection, calibrations, radio, LiDAR, RTK/NTRIP) and the message log.
 */
class MainActivity : Activity() {
    private lateinit var vehicle: Vehicle
    private lateinit var prefs: SharedPreferences
    private val ui = Handler(Looper.getMainLooper())
    private val d by lazy { resources.displayMetrics.density }
    private fun col(id: Int) = getColor(id)

    // chrome
    private lateinit var pillLink: TextView
    private lateinit var pillMode: TextView
    private lateinit var pillGps: TextView
    private lateinit var pillBatt: TextView
    private lateinit var content: FrameLayout
    private val tabs = ArrayList<Button>()
    private val pages = ArrayList<View>()
    private var page = 0

    // data page
    private lateinit var hud: HudView
    private lateinit var flyMap: MapView
    private lateinit var lastMsg: TextView
    private lateinit var armBtn: Button
    private lateinit var drivePanel: LinearLayout
    private lateinit var stick: JoystickView
    private var driveLimit = 0.3f
    private var lastStickSent = 0L
    private var stickZeros = 0

    // plan page
    private lateinit var planMap: MapView
    private val plan = ArrayList<MissionItem>()
    private lateinit var planList: LinearLayout
    private lateinit var planStatus: TextView
    private var planAddMode = true
    private var planSpeed = 0f
    private var planFence = false                       // the Plan tab edits the fence, not the mission
    private val fence = ArrayList<FenceZone>()
    private var fenceZone = -1                          // zone that map taps add corners to
    private lateinit var missionTools: View
    private lateinit var fenceTools: View
    private lateinit var planModeBtn: Button

    // params page
    private lateinit var paramList: ListView
    private lateinit var paramStatus: TextView
    private lateinit var paramWriteBtn: Button
    private var paramFilter = ""
    private val paramShown = ArrayList<Param>()
    private val staged = LinkedHashMap<String, Float>()        // edited or loaded, not yet written
    private var paramWriteOk = 0
    private var paramWriteFail = ArrayList<String>()

    // setup page
    private lateinit var linkType: Spinner
    private lateinit var linkHost: EditText
    private lateinit var linkPort: EditText
    private lateinit var accelText: TextView
    private lateinit var accelNext: Button
    private var accelPos = 0
    private lateinit var magText: TextView
    private val rcBars = ArrayList<ProgressBar>()
    private val rcLabels = ArrayList<TextView>()
    private var rcCal = false
    private val rcMin = IntArray(16) { 3000 }
    private val rcMax = IntArray(16)
    private lateinit var radar: RadarView
    private lateinit var proxText: TextView
    private var ntrip: NtripClient? = null
    private lateinit var ntripStatus: TextView
    private lateinit var ntripBtn: Button
    private val ntripFields = HashMap<String, EditText>()

    // lidar page
    private val lidarBehaviours = LidarBehaviours()
    private lateinit var lidarDriver: LidarDriver
    private lateinit var lidarView: LidarView
    private lateinit var lidarStatus: TextView
    private lateinit var lidarStart: Button
    private lateinit var lidarHint: TextView
    private val lidarModeBtns = HashMap<LidarBehaviours.Mode, Button>()
    private val lidarParamRows = HashMap<LidarBehaviours.Mode, LinearLayout>()
    private var tone: android.media.ToneGenerator? = null
    private var lastBeepMs = 0L
    private var lidarSound = true

    // log page
    private lateinit var logText: TextView
    private var logShown = -1

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        prefs = getSharedPreferences("gcs", MODE_PRIVATE)
        vehicle = Vehicle(applicationContext)
        vehicle.onEvent = { onVehicleEvent(it) }
        driveLimit = prefs.getFloat("drive_limit", 0.3f)

        val root = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        root.addView(topBar())
        content = FrameLayout(this)
        root.addView(content, LinearLayout.LayoutParams(-1, 0, 1f))
        root.addView(tabBar())
        pages += dataPage(); pages += planPage(); pages += paramsPage(); pages += setupPage(); pages += lidarPage(); pages += logPage()
        pages.forEach { content.addView(it, FrameLayout.LayoutParams(-1, -1)); it.visibility = View.GONE }
        setContentView(root)
        showPage(0)

        if (prefs.getBoolean("autoconnect", true)) connect()
        ui.post(refresh)
        ui.post(lidarLoop)
    }

    override fun onPause() {
        super.onPause()
        stopLidar()
        stick.release()
    }

    override fun onDestroy() {
        ui.removeCallbacksAndMessages(null)
        ntrip?.stop()
        lidarDriver.stop()
        tone?.release()
        vehicle.close()
        super.onDestroy()
    }

    // =========================================================================
    // View helpers
    // =========================================================================

    private fun lp(w: Int, h: Int, weight: Float = 0f) = LinearLayout.LayoutParams(w, h, weight)
    private fun dp(v: Int) = (v * d).toInt()

    private fun text(s: String = "", size: Float = 14f, color: Int = R.color.fg, bold: Boolean = false) =
        TextView(this).apply {
            text = s; textSize = size; setTextColor(col(color))
            if (bold) typeface = Typeface.DEFAULT_BOLD
        }

    private fun section(s: String) = TextView(this, null, 0, R.style.Section).apply {
        text = s.uppercase()
        layoutParams = lp(-1, -2).apply { topMargin = dp(18); bottomMargin = dp(6) }
    }

    private fun button(s: String, accent: Boolean = false, onClick: (Button) -> Unit) =
        Button(this, null, 0, R.style.ToolButton).apply {
            text = s
            if (accent) { backgroundTintList = android.content.res.ColorStateList.valueOf(col(R.color.accent)); setTextColor(col(R.color.on_accent)) }
            layoutParams = lp(-2, dp(40)).apply { marginEnd = dp(6); bottomMargin = dp(6) }
            setOnClickListener { onClick(this) }
        }

    private fun pill() = TextView(this, null, 0, R.style.Pill).apply {
        layoutParams = lp(-2, -2).apply { marginEnd = dp(6) }
    }

    private fun setPill(p: TextView, s: String, tone: Int) {
        p.text = s
        val (bg, fg) = when (tone) {
            1 -> R.color.ok_soft to R.color.ok
            2 -> R.color.warn_soft to R.color.warn
            3 -> R.color.bad_soft to R.color.bad
            else -> R.color.sunk to R.color.muted
        }
        p.backgroundTintList = android.content.res.ColorStateList.valueOf(col(bg))
        p.setTextColor(col(fg))
    }

    private fun row(vararg v: View) = LinearLayout(this).apply {
        orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
        v.forEach { addView(it) }
    }

    /** A row that wraps its buttons onto more lines when they don't fit. */
    private fun flow(vararg v: View) = FlowLayout(this).apply { v.forEach { addView(it) } }

    private fun scrollRow(vararg v: View) = HorizontalScrollView(this).apply {
        isHorizontalScrollBarEnabled = false
        addView(row(*v).apply { setPadding(dp(8), dp(6), dp(8), 0) })
    }

    private fun edit(hint: String, value: String, type: Int = InputType.TYPE_CLASS_TEXT) = EditText(this).apply {
        this.hint = hint; setText(value); inputType = type; textSize = 14f; isSingleLine = true
    }

    private fun labeled(label: String, v: View) = LinearLayout(this).apply {
        orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
        addView(text(label, 13f, R.color.muted), lp(dp(96), -2))
        addView(v, lp(0, -2, 1f))
    }

    private fun toast(s: String) = Toast.makeText(this, s, Toast.LENGTH_SHORT).show()

    private fun confirm(title: String, msg: String, ok: String = "OK", action: () -> Unit) {
        AlertDialog.Builder(this).setTitle(title).setMessage(msg)
            .setPositiveButton(ok) { _, _ -> action() }.setNegativeButton("Cancel", null).show()
    }

    private fun requireLink(): Boolean {
        if (!vehicle.connected) { toast("Not connected to the vehicle"); return false }
        return true
    }

    // =========================================================================
    // Chrome: status pills and tabs
    // =========================================================================

    private fun topBar(): View {
        pillLink = pill(); pillMode = pill(); pillGps = pill(); pillBatt = pill()
        pillLink.setOnClickListener { showPage(3) }
        return HorizontalScrollView(this).apply {
            isHorizontalScrollBarEnabled = false
            setBackgroundColor(col(R.color.surface))
            addView(row(pillLink, pillMode, pillGps, pillBatt).apply { setPadding(dp(8), dp(8), dp(8), dp(8)) })
        }
    }

    private fun tabBar(): View {
        val bar = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL; setBackgroundColor(col(R.color.surface))
            setPadding(dp(4), dp(4), dp(4), dp(4))
        }
        listOf("Data", "Plan", "Params", "Setup", "LiDAR", "Log").forEachIndexed { i, s ->
            val b = Button(this, null, 0, R.style.ToolButton).apply {
                text = s; elevation = 0f
                setOnClickListener { showPage(i) }
            }
            tabs += b
            bar.addView(b, lp(0, dp(42), 1f).apply { marginEnd = dp(3) })
        }
        return bar
    }

    private fun showPage(i: Int) {
        page = i
        pages.forEachIndexed { k, v -> v.visibility = if (k == i) View.VISIBLE else View.GONE }
        tabs.forEachIndexed { k, b ->
            b.backgroundTintList = android.content.res.ColorStateList.valueOf(col(if (k == i) R.color.accent_soft else R.color.surface))
            b.setTextColor(col(if (k == i) R.color.accent else R.color.fg))
        }
        if (i == 1) planMap.invalidate()
        if (i == 2) refreshParamList()
        if (i == PAGE_LOG) logShown = -1
    }

    // =========================================================================
    // Data page
    // =========================================================================

    private fun dataPage(): View {
        val v = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        hud = HudView(this)
        v.addView(hud, lp(-1, -2))
        lastMsg = text("", 12f, R.color.muted).apply {
            setPadding(dp(10), dp(4), dp(10), dp(4)); maxLines = 1; setOnClickListener { showPage(PAGE_LOG) }
        }
        v.addView(lastMsg)

        armBtn = button("Arm", accent = true) { armOrDisarm() }
        armBtn.setOnLongClickListener {
            if (!vehicle.armed) confirm("Force arm", "Arm even though the pre-arm checks fail?", "Force arm") { vehicle.arm(true, force = true) }
            true
        }
        val modes = listOf(0 to "Manual", 4 to "Hold", 10 to "Auto", 11 to "RTL", 15 to "Guided", 1 to "Acro",
            3 to "Steering", 5 to "Loiter", 12 to "SmartRTL")
        val modeBtns = modes.map { (num, name) -> button(name) { if (requireLink()) vehicle.setMode(num) } }
        v.addView(scrollRow(armBtn, *modeBtns.toTypedArray()))

        val frame = FrameLayout(this)
        flyMap = MapView(this)
        flyMap.onLongPress = { lat, lon -> mapMenu(lat, lon) }
        flyMap.onTap = { lat, lon ->
            if (vehicle.connected && vehicle.customMode == 15) confirm("Guided", "Drive here?", "Go") { goTo(lat, lon) }
        }
        frame.addView(flyMap, FrameLayout.LayoutParams(-1, -1))
        val mapTools = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL; setPadding(dp(6), dp(6), dp(6), dp(6))
            addView(button("Follow") { flyMap.follow = true; flyMap.invalidate() })
            addView(button("Fit") { flyMap.fit() })
            addView(button(mapSourceLabel()) { b -> toggleMapSource(); b.text = mapSourceLabel() })
            addView(button("Drive") { toggleDrive() })
        }
        frame.addView(mapTools, FrameLayout.LayoutParams(-2, -2, Gravity.END or Gravity.TOP))

        drivePanel = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL; gravity = Gravity.CENTER_HORIZONTAL
            setBackgroundResource(R.drawable.rounded)
            backgroundTintList = android.content.res.ColorStateList.valueOf(col(R.color.surface))
            setPadding(dp(10), dp(8), dp(10), dp(8)); elevation = 6 * d
            visibility = View.GONE
        }
        val limitText = text("Power ${(driveLimit * 100).toInt()}%", 12f, R.color.muted)
        val limit = SeekBar(this).apply {
            max = 90; progress = (driveLimit * 100).toInt() - 10
            setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
                override fun onProgressChanged(s: SeekBar?, p: Int, fromUser: Boolean) {
                    driveLimit = (p + 10) / 100f; limitText.text = "Power ${p + 10}%"
                    prefs.edit().putFloat("drive_limit", driveLimit).apply()
                }
                override fun onStartTrackingTouch(s: SeekBar?) {}
                override fun onStopTrackingTouch(s: SeekBar?) {}
            })
        }
        stick = JoystickView(this)
        stick.onStick = { _, _ -> sendStick(force = true) }
        drivePanel.addView(text("Phone stick: RC override (Manual, Acro, Steering…). Needs SYSID_MYGCS=255.", 11f, R.color.muted).apply {
            gravity = Gravity.CENTER; maxWidth = dp(240)
        })
        drivePanel.addView(stick)
        drivePanel.addView(limitText)
        drivePanel.addView(limit, lp(dp(220), -2))
        frame.addView(drivePanel, FrameLayout.LayoutParams(-2, -2, Gravity.BOTTOM or Gravity.START).apply {
            setMargins(dp(8), dp(8), dp(8), dp(8))
        })
        v.addView(frame, lp(-1, 0, 1f))
        return v
    }

    private fun mapSourceLabel() = if (prefs.getString("map", "STREET") == "STREET") "Satellite" else "Map"

    private fun toggleMapSource() {
        val next = if (prefs.getString("map", "STREET") == "STREET") MapView.Source.SATELLITE else MapView.Source.STREET
        prefs.edit().putString("map", next.name).apply()
        flyMap.source = next; planMap.source = next
    }

    private fun toggleDrive() {
        if (drivePanel.visibility == View.VISIBLE) {
            stick.release(); drivePanel.visibility = View.GONE
        } else drivePanel.visibility = View.VISIBLE
    }

    private fun sendStick(force: Boolean = false) {
        val now = SystemClock.elapsedRealtime()
        if (!force && now - lastStickSent < 100) return
        if (stick.held) {
            lastStickSent = now; stickZeros = 3
            vehicle.manualControl(stick.stickX * driveLimit, stick.stickY * driveLimit)
        } else if (stickZeros > 0) {
            lastStickSent = now; stickZeros--
            vehicle.manualControl(0f, 0f)
        }
    }

    private fun armOrDisarm() {
        if (!requireLink()) return
        if (vehicle.armed) vehicle.arm(false)
        else confirm("Arm", "Arm the motors? Wheels off the ground for the first test.", "Arm") { vehicle.arm(true) }
    }

    private fun goTo(lat: Double, lon: Double) {
        if (!requireLink()) return
        if (vehicle.customMode != 15) vehicle.setMode(15)
        // Repeat once: the first may arrive before the mode change.
        vehicle.goTo(lat, lon)
        ui.postDelayed({ vehicle.goTo(lat, lon) }, 700)
        flyMap.guidedTarget = Pair(lat, lon)
    }

    private fun mapMenu(lat: Double, lon: Double) {
        val items = arrayOf("Drive here (Guided)", "Set home here", "Add waypoint here")
        AlertDialog.Builder(this).setTitle("%.7f, %.7f".format(lat, lon))
            .setItems(items) { _, which ->
                when (which) {
                    0 -> goTo(lat, lon)
                    1 -> if (requireLink()) confirm("Set home", "Move home (RTL point) here?") { vehicle.setHome(lat, lon) }
                    2 -> { plan.add(newWaypoint(lat, lon)); planChanged() }
                }
            }.show()
    }

    // =========================================================================
    // Plan page
    // =========================================================================

    private fun planPage(): View {
        val v = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        val addBtn = button("Tap adds: on") { b -> planAddMode = !planAddMode; b.text = "Tap adds: " + if (planAddMode) "on" else "off" }
        planModeBtn = button("Fence", accent = false) { setPlanFence(!planFence) }
        fenceTools = scrollRow(
            button("Mission") { setPlanFence(false) },
            button("Read") { if (requireLink()) vehicle.downloadFence() },
            button("Write", accent = true) { writeFence() },
            button("+ Stay-in") { addZone(FenceZone.Kind.POLYGON_IN) },
            button("+ Stay-out") { addZone(FenceZone.Kind.POLYGON_OUT) },
            button("+ Circle") { addZone(FenceZone.Kind.CIRCLE_OUT) },
            button("Enable…") { enableFence() },
            button("Clear") { confirm("Clear fence", "Remove all zones from the fence on the phone? Write afterwards to clear the vehicle's fence too.") { fence.clear(); fenceZone = -1; fenceChanged() } },
        ).apply { visibility = View.GONE }
        v.addView(fenceTools)
        missionTools = scrollRow(
            planModeBtn,
            button("Read") { if (requireLink()) vehicle.downloadMission() },
            button("Write", accent = true) { writeMission() },
            addBtn,
            button("Speed…") { askPlanSpeed() },
            button("Clear") { confirm("Clear plan", "Remove all waypoints from the plan on the phone?") { plan.clear(); planChanged() } },
            button("Start Auto") { startAuto() },
            button("Load") { openFile(REQ_LOAD_MISSION) },
            button("Save") { saveFile(REQ_SAVE_MISSION, "mission.waypoints") },
        )
        v.addView(missionTools)
        planStatus = text("Tap the map to add a waypoint, drag one to move it, tap it to edit.", 12f, R.color.muted).apply {
            setPadding(dp(10), dp(2), dp(10), dp(4))
        }
        v.addView(planStatus)
        planMap = MapView(this).apply {
            editable = true
            mission = plan
            onTap = { lat, lon ->
                if (planFence) fenceTap(lat, lon)
                else if (planAddMode) { plan.add(newWaypoint(lat, lon)); planChanged() }
            }
            onWaypointTap = { i -> editItem(i) }
            onWaypointMoved = { _, _, _ -> planChanged() }
            onFenceTap = { zone, point -> editZone(zone, point) }
            onFenceMoved = { zone -> fenceZone = zone; fenceChanged() }
        }
        v.addView(planMap, lp(-1, 0, 1f))
        planList = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setPadding(dp(10), dp(4), dp(10), dp(4)) }
        v.addView(ScrollView(this).apply { addView(planList) }, lp(-1, dp(150)))
        planChanged()
        return v
    }

    private fun newWaypoint(lat: Double, lon: Double) = MissionItem(lat = lat, lon = lon)

    // --- fence ------------------------------------------------------------------------

    private fun setPlanFence(on: Boolean) {
        planFence = on
        fenceTools.visibility = if (on) View.VISIBLE else View.GONE
        missionTools.visibility = if (on) View.GONE else View.VISIBLE
        planMap.fenceEditing = on
        if (on) fenceChanged() else planChanged()
    }

    private fun addZone(kind: FenceZone.Kind) {
        fence.add(FenceZone(kind, radius = if (kind == FenceZone.Kind.CIRCLE_OUT) 3f else 0f))
        fenceZone = fence.size - 1
        fenceChanged()
        planStatus.text = if (kind.polygon) "Tap the map to place the corners of ${kind.label.lowercase()} ${fenceZone + 1} (3 or more)."
            else "Tap the map to place the centre of the circle."
    }

    /** A map tap in fence mode: a corner for the zone being drawn, or a circle's centre. */
    private fun fenceTap(lat: Double, lon: Double) {
        val z = fence.getOrNull(fenceZone)
        if (z == null) { toast("Add a zone first: + Stay-in, + Stay-out or + Circle"); return }
        when {
            z.kind.polygon -> z.points.add(doubleArrayOf(lat, lon))
            z.points.isEmpty() -> { z.points.add(doubleArrayOf(lat, lon)); if (z.isCircle) askRadius(z) }
            else -> { z.points[0][0] = lat; z.points[0][1] = lon }
        }
        fenceChanged()
    }

    private fun askRadius(z: FenceZone) {
        val e = edit("metres", fmt(z.radius), InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL)
        AlertDialog.Builder(this).setTitle("${z.kind.label} radius")
            .setView(FrameLayout(this).apply { setPadding(dp(20), 0, dp(20), 0); addView(e) })
            .setPositiveButton("OK") { _, _ -> z.radius = e.text.toString().toFloatOrNull()?.takeIf { it > 0 } ?: z.radius; fenceChanged() }
            .setNegativeButton("Cancel", null).show()
    }

    private fun editZone(zi: Int, point: Int) {
        val z = fence[zi]
        fenceZone = zi
        fenceChanged()
        val acts = ArrayList<Pair<String, () -> Unit>>()
        acts += "Add corners to this zone" to { planStatus.text = "Tapping the map adds corners to ${z.kind.label.lowercase()} ${zi + 1}." }
        if (z.kind.polygon && point >= 0) acts += "Delete this corner" to { z.points.removeAt(point); fenceChanged() }
        if (z.isCircle) acts += "Change radius (now ${fmt(z.radius)} m)" to { askRadius(z) }
        if (z.kind.polygon || z.isCircle) {
            val other = when (z.kind) {
                FenceZone.Kind.POLYGON_IN -> FenceZone.Kind.POLYGON_OUT; FenceZone.Kind.POLYGON_OUT -> FenceZone.Kind.POLYGON_IN
                FenceZone.Kind.CIRCLE_IN -> FenceZone.Kind.CIRCLE_OUT; else -> FenceZone.Kind.CIRCLE_IN
            }
            acts += "Make it ${other.label.lowercase()}" to { z.kind = other; fenceChanged() }
        }
        acts += "Delete the whole zone" to { fence.removeAt(zi); fenceZone = -1; fenceChanged() }
        AlertDialog.Builder(this).setTitle("${zi + 1}. ${z.kind.label}")
            .setItems(acts.map { it.first }.toTypedArray()) { _, w -> acts[w].second() }.show()
    }

    private fun fenceChanged() {
        planMap.fences = fence; flyMap.fences = fence
        planMap.selectedZone = fenceZone
        planMap.invalidate(); flyMap.invalidate()
        if (!planFence) return
        planList.removeAllViews()
        fence.forEachIndexed { i, z ->
            val what = when {
                z.kind.polygon -> "${z.points.size} corners"
                z.isCircle -> "radius ${fmt(z.radius)} m" + if (z.points.isEmpty()) ", no centre yet" else ""
                else -> ""
            }
            planList.addView(text("${i + 1}. ${z.kind.label}, $what" + if (i == fenceZone) "  ◀ adding" else "", 13f,
                if (z.kind.inclusion) R.color.ok else R.color.bad).apply {
                setPadding(0, dp(5), 0, dp(5)); setOnClickListener { editZone(i, -1) }
            })
        }
        if (fence.isEmpty()) planList.addView(text("No fence zones. Stay-in: the car must stay inside. Stay-out: it must keep out.", 13f, R.color.muted))
        else {
            val bytes = Fence.storageBytes(fence)
            planList.addView(text("Uses $bytes of the board's ${Fence.BOARD_STORAGE_BYTES} bytes of fence storage" +
                (Fence.problem(fence)?.let { "\n⚠ $it" } ?: ""), 12f,
                if (bytes > Fence.BOARD_STORAGE_BYTES || Fence.problem(fence) != null) R.color.warn else R.color.muted))
        }
    }

    private fun writeFence() {
        if (!requireLink()) return
        Fence.problem(fence)?.let { toast(it); return }
        val bytes = Fence.storageBytes(fence)
        val send = { vehicle.uploadFence(Fence.toItems(fence)) }
        when {
            fence.isEmpty() -> confirm("Clear the vehicle's fence", "Write an empty fence? This deletes all fence zones on the vehicle.", "Clear") { send() }
            bytes > Fence.BOARD_STORAGE_BYTES -> confirm("Fence may not fit",
                "This fence needs $bytes bytes; the ESP32-S3 board stores ${Fence.BOARD_STORAGE_BYTES}. It will refuse it (\"Fence exceeds storage size\"). Write anyway?", "Write") { send() }
            else -> send()
        }
    }

    /** Turns the fence on: polygons and circles in FENCE_TYPE, Hold on a breach unless an action is already set. */
    private fun enableFence() {
        if (!requireLink()) return
        val en = vehicle.paramValue("FENCE_ENABLE")
        val type = vehicle.paramValue("FENCE_TYPE")?.toInt()
        val action = vehicle.paramValue("FENCE_ACTION")?.toInt()
        if (en == null || type == null) { toast("Read the parameters first (Params → Refresh)"); return }
        val newType = type or 4                   // bit 2: inclusion/exclusion circles + polygons
        val newAction = if (action == null || action == 0) 2 else action
        confirm("Enable the fence",
            "FENCE_ENABLE ${en.toInt()} → 1\nFENCE_TYPE $type → $newType (adds polygons and circles)\n" +
                "FENCE_ACTION ${action ?: "?"} → $newAction (${if (newAction == 2) "Hold" else "unchanged"})\n\n" +
                "The car will not arm while it is outside a stay-in zone or inside a stay-out zone.", "Enable") {
            vehicle.setParam("FENCE_TYPE", newType.toFloat())
            vehicle.setParam("FENCE_ACTION", newAction.toFloat())
            vehicle.setParam("FENCE_ENABLE", 1f)
        }
    }

    private fun itemLabel(it: MissionItem) = when (it.command) {
        Mav.CMD_NAV_WAYPOINT -> "Waypoint" + if (it.p1 > 0) ", wait ${it.p1.toInt()} s" else ""
        19 -> "Loiter ${it.p1.toInt()} s"
        17 -> "Loiter forever"
        Mav.CMD_NAV_RETURN_TO_LAUNCH -> "Return to launch"
        Mav.CMD_DO_CHANGE_SPEED -> "Speed %.1f m/s".format(it.p2)
        177 -> "Jump to ${it.p1.toInt()} (${it.p2.toInt()}×)"
        else -> "Command ${it.command}"
    }

    private fun planChanged() {
        planMap.mission = plan; flyMap.mission = plan
        planMap.invalidate(); flyMap.invalidate()
        if (planFence) return                     // the list shows fence zones
        planList.removeAllViews()
        var total = 0.0
        var prev: Pair<Double, Double>? = if (vehicle.hasHome) Pair(vehicle.homeLat, vehicle.homeLon) else null
        plan.forEachIndexed { i, it ->
            var dist = ""
            if (it.hasPosition) {
                prev?.let { p -> val m = MapView.distanceM(p.first, p.second, it.lat, it.lon); total += m; dist = "  %.0f m".format(m) }
                prev = Pair(it.lat, it.lon)
            }
            planList.addView(text("${i + 1}. ${itemLabel(it)}$dist", 13f).apply {
                setPadding(0, dp(5), 0, dp(5)); setOnClickListener { editItem(i) }
            })
        }
        if (plan.isEmpty()) planList.addView(text("No waypoints", 13f, R.color.muted))
        else planList.addView(text("Total %.0f m".format(total), 12f, R.color.muted))
    }

    private fun editItem(i: Int) {
        val it = plan[i]
        val box = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setPadding(dp(20), dp(8), dp(20), 0) }
        val kinds = listOf(Mav.CMD_NAV_WAYPOINT to "Waypoint", 19 to "Loiter (time)", Mav.CMD_NAV_RETURN_TO_LAUNCH to "Return to launch",
            Mav.CMD_DO_CHANGE_SPEED to "Change speed")
        val spin = Spinner(this).apply {
            adapter = ArrayAdapter(this@MainActivity, android.R.layout.simple_spinner_dropdown_item, kinds.map { k -> k.second })
            setSelection(kinds.indexOfFirst { k -> k.first == it.command }.coerceAtLeast(0))
        }
        val num = InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL
        val wait = edit("seconds", if (it.command == Mav.CMD_DO_CHANGE_SPEED) "" else fmt(it.p1), num)
        val speed = edit("m/s", if (it.command == Mav.CMD_DO_CHANGE_SPEED) fmt(it.p2) else "", num)
        box.addView(spin)
        box.addView(labeled("Wait / loiter", wait))
        box.addView(labeled("Speed", speed))
        if (it.hasPosition) box.addView(text("%.7f, %.7f".format(it.lat, it.lon), 12f, R.color.muted))
        AlertDialog.Builder(this).setTitle("Item ${i + 1}").setView(box)
            .setPositiveButton("Save") { _, _ ->
                val cmd = kinds[spin.selectedItemPosition].first
                it.command = cmd
                when (cmd) {
                    Mav.CMD_DO_CHANGE_SPEED -> { it.p1 = 1f; it.p2 = speed.text.toString().toFloatOrNull() ?: 1f; it.p3 = -1f; it.lat = 0.0; it.lon = 0.0 }
                    Mav.CMD_NAV_RETURN_TO_LAUNCH -> { it.p1 = 0f; it.lat = 0.0; it.lon = 0.0 }
                    else -> it.p1 = wait.text.toString().toFloatOrNull() ?: 0f
                }
                if ((cmd == Mav.CMD_NAV_WAYPOINT || cmd == 19) && !it.hasPosition) {
                    toast("Drag it into place: it has no position yet")
                    val c = planMap.geo(planMap.width / 2f, planMap.height / 2f)
                    it.lat = c.first; it.lon = c.second
                }
                planChanged()
            }
            .setNeutralButton("Delete") { _, _ -> plan.removeAt(i); planChanged() }
            .setNegativeButton("More…") { _, _ -> moreItemActions(i) }
            .show()
    }

    private fun moreItemActions(i: Int) {
        val acts = arrayOf("Move up", "Move down", "Insert speed change before", "Go to this item now")
        AlertDialog.Builder(this).setTitle("Item ${i + 1}").setItems(acts) { _, w ->
            when (w) {
                0 -> if (i > 0) { plan.add(i - 1, plan.removeAt(i)); planChanged() }
                1 -> if (i < plan.size - 1) { plan.add(i + 1, plan.removeAt(i)); planChanged() }
                2 -> { plan.add(i, MissionItem(command = Mav.CMD_DO_CHANGE_SPEED, p1 = 1f, p2 = 1f, p3 = -1f)); planChanged(); editItem(i) }
                3 -> if (requireLink()) vehicle.setCurrentWaypoint(i + 1)
            }
        }.show()
    }

    private fun askPlanSpeed() {
        val e = edit("m/s, empty = WP_SPEED", if (planSpeed > 0) fmt(planSpeed) else "",
            InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL)
        AlertDialog.Builder(this).setTitle("Mission speed")
            .setMessage("Adds a speed change at the start of the mission when written.")
            .setView(FrameLayout(this).apply { setPadding(dp(20), 0, dp(20), 0); addView(e) })
            .setPositiveButton("OK") { _, _ -> planSpeed = e.text.toString().toFloatOrNull() ?: 0f }
            .setNegativeButton("Cancel", null).show()
    }

    private fun writeMission() {
        if (!requireLink()) return
        if (plan.isEmpty()) { toast("The plan is empty"); return }
        val items = ArrayList<MissionItem>()
        if (planSpeed > 0 && plan.firstOrNull()?.command != Mav.CMD_DO_CHANGE_SPEED)
            items.add(MissionItem(command = Mav.CMD_DO_CHANGE_SPEED, p1 = 1f, p2 = planSpeed, p3 = -1f))
        items.addAll(plan)
        vehicle.uploadMission(items)
    }

    private fun startAuto() {
        if (!requireLink()) return
        confirm("Start mission", "Switch to Auto and run the mission from the start?", "Start") {
            vehicle.setCurrentWaypoint(1)
            vehicle.setMode(10)
        }
    }

    // =========================================================================
    // Params page
    // =========================================================================

    private fun paramsPage(): View {
        val v = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        paramWriteBtn = button("Write 0", accent = true) { writeStaged() }
        v.addView(scrollRow(
            button("Refresh") { if (requireLink()) { vehicle.fetchParams(); paramStatus.text = "Reading…" } },
            paramWriteBtn,
            button("Load file") { openFile(REQ_LOAD_PARAMS) },
            button("Save file") { saveFile(REQ_SAVE_PARAMS, "rover.parm") },
            button("Undo edits") { staged.clear(); refreshParamList() },
            button("Reboot") { if (requireLink()) confirm("Reboot", "Reboot the autopilot? (It must be disarmed.)", "Reboot") { vehicle.reboot() } },
        ))
        val search = edit("Search, e.g. SERVO1 or PRX", "").apply {
            addTextChangedListener(object : TextWatcher {
                override fun afterTextChanged(s: Editable?) { paramFilter = s.toString().trim().uppercase(); refreshParamList() }
                override fun beforeTextChanged(s: CharSequence?, a: Int, b: Int, c: Int) {}
                override fun onTextChanged(s: CharSequence?, a: Int, b: Int, c: Int) {}
            })
        }
        v.addView(search, lp(-1, -2).apply { setMargins(dp(10), 0, dp(10), 0) })
        paramStatus = text("Press Refresh to read the parameters", 12f, R.color.muted).apply { setPadding(dp(12), dp(2), dp(12), dp(4)) }
        v.addView(paramStatus)
        paramList = ListView(this).apply {
            adapter = paramAdapter
            setOnItemClickListener { _, _, pos, _ -> editParam(paramShown[pos]) }
        }
        v.addView(paramList, lp(-1, 0, 1f))
        return v
    }

    private val paramAdapter = object : BaseAdapter() {
        override fun getCount() = paramShown.size
        override fun getItem(p: Int) = paramShown[p]
        override fun getItemId(p: Int) = p.toLong()
        override fun getView(p: Int, convert: View?, parent: ViewGroup?): View {
            val r = (convert as? LinearLayout) ?: LinearLayout(this@MainActivity).apply {
                orientation = LinearLayout.HORIZONTAL; setPadding(dp(12), dp(9), dp(12), dp(9))
                addView(text("", 14f).apply { typeface = Typeface.MONOSPACE }, lp(0, -2, 1f))
                addView(text("", 14f, bold = true).apply { typeface = Typeface.MONOSPACE; gravity = Gravity.END })
            }
            val prm = paramShown[p]
            val name = r.getChildAt(0) as TextView
            val value = r.getChildAt(1) as TextView
            name.text = prm.name
            val st = staged[prm.name]
            if (st != null) {
                value.text = "${fmtParam(prm.value, prm.type)} → ${fmtParam(st, prm.type)}"
                value.setTextColor(col(R.color.warn))
            } else {
                value.text = fmtParam(prm.value, prm.type)
                value.setTextColor(col(R.color.fg))
            }
            return r
        }
    }

    private fun refreshParamList() {
        val all = synchronized(vehicle) { vehicle.params.values.toList() }
        paramShown.clear()
        paramShown.addAll(all.filter { paramFilter.isEmpty() || it.name.contains(paramFilter) })
        paramAdapter.notifyDataSetChanged()
        paramWriteBtn.text = "Write ${staged.size}"
    }

    private fun editParam(p: Param) {
        val e = edit("value", fmtParam(staged[p.name] ?: p.value, p.type),
            InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL or InputType.TYPE_NUMBER_FLAG_SIGNED)
        AlertDialog.Builder(this).setTitle(p.name)
            .setMessage("On the vehicle: ${fmtParam(p.value, p.type)}")
            .setView(FrameLayout(this).apply { setPadding(dp(20), 0, dp(20), 0); addView(e) })
            .setPositiveButton("Write now") { _, _ ->
                val v = e.text.toString().toFloatOrNull() ?: return@setPositiveButton toast("Not a number")
                staged.remove(p.name)
                paramWriteOk = 0; paramWriteFail.clear()
                vehicle.setParam(p.name, v)
                refreshParamList()
            }
            .setNeutralButton("Stage") { _, _ ->
                val v = e.text.toString().toFloatOrNull() ?: return@setNeutralButton toast("Not a number")
                if (v == p.value) staged.remove(p.name) else staged[p.name] = v
                refreshParamList()
            }
            .setNegativeButton("Cancel", null).show()
    }

    private fun writeStaged() {
        if (!requireLink() || staged.isEmpty()) return
        paramWriteOk = 0; paramWriteFail.clear()
        for ((k, v) in staged) vehicle.setParam(k, v)
        paramStatus.text = "Writing ${staged.size}…"
        staged.clear()
        refreshParamList()
    }

    private fun loadParamFile(textIn: String) {
        var n = 0; var unknown = 0
        for (raw in textIn.lines()) {
            val line = raw.substringBefore('#').trim()
            if (line.isEmpty()) continue
            val parts = line.split(Regex("[,\\s\\t]+"))
            if (parts.size < 2) continue
            val name = parts[0].trim(); val value = parts[1].toFloatOrNull() ?: continue
            val cur = vehicle.paramValue(name)
            if (cur == null) { unknown++; if (vehicle.params.isNotEmpty()) continue }
            if (cur != null && abs(cur - value) < 1e-6f) continue
            staged[name] = value; n++
        }
        refreshParamList()
        val msg = "$n values differ from the vehicle" + if (unknown > 0) ", $unknown not on this vehicle (yet)" else ""
        paramStatus.text = "Loaded: $msg. Press Write."
        if (vehicle.params.isEmpty()) toast("Read the parameters first, so only the changes are written")
    }

    private fun paramFileText(): String {
        val sb = StringBuilder("# ArduRover parameters saved by ESP32 Rover GCS\n")
        synchronized(vehicle) { vehicle.params.values.toList() }.forEach { sb.append("${it.name},${fmtParam(it.value, it.type)}\n") }
        return sb.toString()
    }

    private fun fmt(f: Float) = if (f == f.toLong().toFloat()) f.toLong().toString() else f.toString()

    private fun fmtParam(v: Float, type: Int): String =
        if (type in 1..8 || (v == v.toLong().toFloat() && abs(v) < 1e7)) v.toLong().toString()
        else "%.6g".format(java.util.Locale.US, v).let { if (it.contains('.') && !it.contains('e')) it.trimEnd('0').trimEnd('.') else it }

    // =========================================================================
    // Setup page
    // =========================================================================

    private fun setupPage(): View {
        val v = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setPadding(dp(14), 0, dp(14), dp(30)) }

        v.addView(section("Connection"))
        v.addView(text("Join the board's WiFi (ardupilot123, password ardupilot123) first. Mobile data stays on for maps and NTRIP.", 12f, R.color.muted))
        linkType = Spinner(this).apply {
            adapter = ArrayAdapter(this@MainActivity, android.R.layout.simple_spinner_dropdown_item, listOf("TCP", "UDP"))
            setSelection(if (prefs.getString("link_type", "TCP") == "UDP") 1 else 0)
        }
        linkHost = edit("192.168.4.1", prefs.getString("link_host", "192.168.4.1")!!)
        linkPort = edit("5760", prefs.getInt("link_port", 5760).toString(), InputType.TYPE_CLASS_NUMBER)
        v.addView(labeled("Protocol", linkType))
        v.addView(labeled("Address", linkHost))
        v.addView(labeled("Port", linkPort))
        v.addView(flow(
            button("Connect", accent = true) { saveLink(); connect() },
            button("Disconnect") { prefs.edit().putBoolean("autoconnect", false).apply(); vehicle.disconnect() },
        ))
        v.addView(text("TCP 192.168.4.1:5760 is the board's default (HAL_ESP32_WIFI 1). For a UDP build (HAL_ESP32_WIFI 2) choose UDP, leave the address empty and use port 14550.", 11f, R.color.muted))

        v.addView(section("Accelerometer"))
        accelText = text("Mount the board in the car first. Then 6 positions, or level only.", 13f)
        v.addView(accelText)
        accelNext = button("Done, next position", accent = true) {
            if (accelPos in 1..6) vehicle.accelCalPosition(accelPos)
        }.apply { isEnabled = false }
        v.addView(flow(
            button("Calibrate accel") { if (requireLink()) { vehicle.startAccelCal(); accelText.text = "Starting…" } },
            button("Level only") { if (requireLink()) confirm("Level", "Car standing level?") { vehicle.accelCalLevelOnly() } },
            accelNext,
        ))

        v.addView(section("Compass"))
        magText = text("Start, then turn the car slowly in every direction (on every side) until 100%.", 13f)
        v.addView(magText)
        v.addView(flow(
            button("Start") { if (requireLink()) vehicle.startMagCal() },
            button("Accept") { if (requireLink()) vehicle.acceptMagCal() },
            button("Cancel") { if (requireLink()) vehicle.cancelMagCal() },
        ))

        v.addView(section("Radio"))
        v.addView(text("Ch 1 steering, ch 3 throttle, ch 8 mode switch.", 12f, R.color.muted))
        for (i in 0 until 8) {
            val lbl = text("${i + 1}", 12f).apply { typeface = Typeface.MONOSPACE }
            val bar = ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal).apply { max = 1200 }
            rcLabels += lbl; rcBars += bar
            v.addView(LinearLayout(this).apply {
                orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
                addView(bar, lp(0, dp(18), 1f)); addView(lbl, lp(dp(150), -2).apply { marginStart = dp(8) })
            })
        }
        val rcBtn = button("Calibrate radio") { b -> radioCal(b) }
        v.addView(flow(rcBtn))

        v.addView(section("LiDAR proximity"))
        radar = RadarView(this)
        v.addView(radar, lp(-1, -2))
        proxText = text("Needs PRX1_TYPE=19 (LD14). Put a box in front: it should show at the top; else set PRX1_YAW_CORR.", 12f, R.color.muted)
        v.addView(proxText)

        v.addView(section("RTK / NTRIP"))
        v.addView(text("Corrections from a caster, sent to the F9P over MAVLink (GPS_RTCM_DATA).", 12f, R.color.muted))
        for ((key, hint, def) in listOf(
            Triple("host", "caster, e.g. rtk2go.com", ""), Triple("port", "2101", "2101"),
            Triple("mount", "mountpoint", ""), Triple("user", "user (e-mail for rtk2go)", ""), Triple("pass", "password", ""))) {
            val e = edit(hint, prefs.getString("ntrip_$key", def)!!,
                when (key) { "port" -> InputType.TYPE_CLASS_NUMBER; "pass" -> InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_PASSWORD; else -> InputType.TYPE_CLASS_TEXT })
            ntripFields[key] = e
            v.addView(labeled(key.replaceFirstChar { it.uppercase() }, e))
        }
        ntripBtn = button("Start NTRIP", accent = true) { toggleNtrip() }
        v.addView(flow(ntripBtn))
        ntripStatus = text("Stopped", 12f, R.color.muted)
        v.addView(ntripStatus)

        v.addView(section("Vehicle"))
        v.addView(flow(
            button("Set home here") { if (requireLink()) confirm("Set home", "Make the car's current position home?") { vehicle.setHomeHere() } },
            button("Reboot") { if (requireLink()) confirm("Reboot", "Reboot the autopilot?", "Reboot") { vehicle.reboot() } },
            button("Clear mission on vehicle") { if (requireLink()) confirm("Clear", "Delete the mission stored on the vehicle?") { vehicle.clearMission() } },
        ))
        return ScrollView(this).apply { addView(v) }
    }

    private fun saveLink() {
        prefs.edit()
            .putString("link_type", linkType.selectedItem.toString())
            .putString("link_host", linkHost.text.toString().trim())
            .putInt("link_port", linkPort.text.toString().toIntOrNull() ?: 5760)
            .putBoolean("autoconnect", true).apply()
    }

    private fun connect() {
        val type = prefs.getString("link_type", "TCP")
        val host = prefs.getString("link_host", "192.168.4.1")!!
        val port = prefs.getInt("link_port", 5760)
        vehicle.connect {
            if (type == "UDP") UdpLink(applicationContext, host, port) else TcpLink(applicationContext, host, port)
        }
    }

    private fun radioCal(b: Button) {
        if (!rcCal) {
            if (!requireLink()) return
            rcMin.fill(3000); rcMax.fill(0)
            rcCal = true
            b.text = "Save calibration"
            toast("Move every stick and switch to both ends, centre the sticks, then Save")
            return
        }
        rcCal = false
        b.text = "Calibrate radio"
        var n = 0
        for (i in 0 until 8) {
            if (rcMax[i] - rcMin[i] < 200) continue
            val ch = i + 1
            vehicle.setParam("RC${ch}_MIN", rcMin[i].toFloat())
            vehicle.setParam("RC${ch}_MAX", rcMax[i].toFloat())
            // Trim = the centred stick now; switches and the throttle of a car stay at their middle.
            val trim = vehicle.rc[i].coerceIn(rcMin[i], rcMax[i])
            vehicle.setParam("RC${ch}_TRIM", trim.toFloat())
            n++
        }
        paramWriteOk = 0; paramWriteFail.clear()
        toast(if (n == 0) "No channel moved: nothing saved" else "Writing min/max/trim for $n channels")
    }

    private fun toggleNtrip() {
        ntrip?.let { it.stop(); ntrip = null; ntripBtn.text = "Start NTRIP"; ntripStatus.text = "Stopped"; return }
        val f = ntripFields.mapValues { it.value.text.toString().trim() }
        if (f["host"].isNullOrEmpty() || f["mount"].isNullOrEmpty()) { toast("Caster and mountpoint are needed"); return }
        prefs.edit().apply { f.forEach { (k, v) -> putString("ntrip_$k", v) } }.apply()
        val c = NtripClient(applicationContext, f["host"]!!, f["port"]?.toIntOrNull() ?: 2101, f["mount"]!!,
            f["user"]!!, f["pass"]!!,
            position = { if (vehicle.hasPosition) Pair(vehicle.lat, vehicle.lon) else null },
            onFrame = { vehicle.injectRtcm(it) })
        c.start()
        ntrip = c
        ntripBtn.text = "Stop NTRIP"
    }

    // =========================================================================
    // LiDAR page: hoverboard_laser's Avoid / Follow / Warn on the LD14 scan
    // =========================================================================

    private fun lidarPage(): View {
        loadLidarParams()
        lidarDriver = LidarDriver(vehicle, lidarBehaviours)
        val v = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setPadding(dp(14), dp(8), dp(14), dp(30)) }
        lidarView = LidarView(this)
        v.addView(lidarView, lp(-1, -2))
        lidarStatus = text("Off", 15f, bold = true).apply { setPadding(0, dp(8), 0, dp(4)) }
        v.addView(lidarStatus)

        val modes = LidarBehaviours.Mode.values().map { m ->
            button(m.label) { setLidarMode(m) }.also { lidarModeBtns[m] = it }
        }
        lidarStart = button("Start", accent = true) { if (lidarDriver.running) stopLidar() else startLidar() }
        v.addView(flow(*modes.toTypedArray(), lidarStart))
        lidarHint = text("", 12f, R.color.muted)
        v.addView(lidarHint)

        v.addView(section("Settings"))
        val skid = button(if (lidarBehaviours.p.skidSteer) "Vehicle: skid steer" else "Vehicle: RC car") { b ->
            lidarBehaviours.p.skidSteer = !lidarBehaviours.p.skidSteer
            b.text = if (lidarBehaviours.p.skidSteer) "Vehicle: skid steer" else "Vehicle: RC car"
            saveLidarParams()
        }
        val sound = button(if (lidarSound) "Warn sound: on" else "Warn sound: off") { b ->
            lidarSound = !lidarSound; b.text = if (lidarSound) "Warn sound: on" else "Warn sound: off"; saveLidarParams()
        }
        v.addView(flow(skid, sound))
        v.addView(text("RC car: turns that the hoverboard makes on the spot become backing up with the wheels the other " +
            "way, only while the rear is clear. Skid steer: on the spot, as in hoverboard_laser.", 11f, R.color.muted))
        val pp = lidarBehaviours.p
        lidarParamRows[LidarBehaviours.Mode.AVOID] = paramGroup(
            Triple("Speed (stick, 0-1)", { pp.linear }, { x: Float -> pp.linear = x.coerceIn(0f, 1f) }),
            Triple("Steering (stick, 0-1)", { pp.angular }, { x: Float -> pp.angular = x.coerceIn(0f, 1f) }),
            Triple("Response dist (m)", { pp.responseDist }, { x: Float -> pp.responseDist = x.coerceIn(0.2f, 5f) }),
            Triple("Side sector to (deg)", { pp.laserAngle }, { x: Float -> pp.laserAngle = x.coerceIn(25f, 90f) }),
            Triple("Bins to block", { pp.minPoints.toFloat() }, { x: Float -> pp.minPoints = x.toInt().coerceIn(1, 10) }),
        )
        lidarParamRows[LidarBehaviours.Mode.FOLLOW] = paramGroup(
            Triple("Keep distance (m)", { pp.followDist }, { x: Float -> pp.followDist = x.coerceIn(0.3f, 5f) }),
            Triple("Max speed (stick)", { pp.maxLinear }, { x: Float -> pp.maxLinear = x.coerceIn(0f, 1f) }),
            Triple("Max steering (stick)", { pp.maxAngular }, { x: Float -> pp.maxAngular = x.coerceIn(0f, 1f) }),
            Triple("Priority angle (deg)", { pp.priorityAngle }, { x: Float -> pp.priorityAngle = x.coerceIn(0f, 90f) }),
            Triple("Search angle (deg)", { pp.trackAngle }, { x: Float -> pp.trackAngle = x.coerceIn(5f, 180f) }),
        )
        lidarParamRows[LidarBehaviours.Mode.WARN] = paramGroup(
            Triple("Warn distance (m)", { pp.warnDist }, { x: Float -> pp.warnDist = x.coerceIn(0.1f, 8f) }),
            Triple("Warn angle (deg)", { pp.warnAngle }, { x: Float -> pp.warnAngle = x.coerceIn(5f, 180f) }),
        )
        lidarParamRows.values.forEach { v.addView(it) }
        v.addView(paramGroup(Triple("Ignore closer than (m)", { pp.minRange }, { x: Float -> pp.minRange = x.coerceIn(0f, 2f) })))
        setLidarMode(lidarBehaviours.mode)
        return ScrollView(this).apply { addView(v) }
    }

    private fun paramGroup(vararg rows: Triple<String, () -> Float, (Float) -> Unit>) = LinearLayout(this).apply {
        orientation = LinearLayout.VERTICAL
        for ((label, get, set) in rows) {
            val e = edit("", fmt(get()), InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL).apply {
                addTextChangedListener(object : TextWatcher {
                    override fun afterTextChanged(t: Editable?) { t.toString().toFloatOrNull()?.let { set(it); saveLidarParams() } }
                    override fun beforeTextChanged(t: CharSequence?, a: Int, b: Int, c: Int) {}
                    override fun onTextChanged(t: CharSequence?, a: Int, b: Int, c: Int) {}
                })
            }
            addView(labeled(label, e))
        }
    }

    private fun setLidarMode(m: LidarBehaviours.Mode) {
        val wasRunning = lidarDriver.running
        lidarBehaviours.mode = m
        lidarModeBtns.forEach { (k, b) ->
            b.backgroundTintList = android.content.res.ColorStateList.valueOf(col(if (k == m) R.color.accent_soft else R.color.surface))
            b.setTextColor(col(if (k == m) R.color.accent else R.color.fg))
        }
        lidarParamRows.forEach { (k, row) -> row.visibility = if (k == m) View.VISIBLE else View.GONE }
        lidarHint.text = when (m) {
            LidarBehaviours.Mode.OFF -> "Shows the scan only. Choose a behaviour from hoverboard_laser."
            LidarBehaviours.Mode.AVOID -> "laser_Avoidance: drives forward and steers away from obstacles in the front, left and right sectors."
            LidarBehaviours.Mode.FOLLOW -> "laser_Tracker: follows the nearest object ahead and keeps the set distance from it."
            LidarBehaviours.Mode.WARN -> "laser_Warning: beeps and vibrates the phone when something ahead is too close. Doesn't drive."
        } + if (m == LidarBehaviours.Mode.AVOID || m == LidarBehaviours.Mode.FOLLOW)
            " Drives with stick overrides (Manual, Acro or Steering mode); touching the Drive stick pauses it, Stop or leaving the app stops the car." else ""
        prefs.edit().putString("lidar_mode", m.name).apply()
        if (m == LidarBehaviours.Mode.OFF) stopLidar()
        else if (wasRunning) lidarDriver.start()          // switch behaviour on the fly, cancelling the old one's sticks
    }

    private fun startLidar() {
        val m = lidarBehaviours.mode
        if (m == LidarBehaviours.Mode.OFF) { toast("Choose Avoid, Follow or Warn first"); return }
        if (vehicle.scan == null) { toast("No LiDAR scan from the vehicle yet"); return }
        if (lidarDriver.drives) {
            if (!requireLink()) return
            if (vehicle.customMode !in listOf(0, 1, 3)) {
                confirm("Mode", "Stick overrides only drive in Manual, Acro or Steering. Switch to Manual and start?", "Manual") {
                    vehicle.setMode(0); lidarDriver.start(); lidarStart.text = "Stop"
                }
                return
            }
            if (!vehicle.armed) toast("The car is disarmed: arm it on the Data tab to drive")
        }
        lidarDriver.start()
        lidarStart.text = "Stop"
    }

    private fun stopLidar() {
        if (!this::lidarDriver.isInitialized) return
        lidarDriver.stop()
        lidarStart.text = "Start"
    }

    private val lidarLoop = object : Runnable {
        override fun run() {
            try { lidarTick() } catch (e: Exception) { android.util.Log.e("Gcs", "lidar", e) }
            ui.postDelayed(this, 100)
        }
    }

    private fun lidarTick() {
        val now = SystemClock.elapsedRealtime()
        val cmd = lidarDriver.tick(now, stick.held, driveLimit)
        if (lidarDriver.running && cmd.alarm && lidarSound && now - lastBeepMs > 450) {
            lastBeepMs = now
            if (tone == null) tone = android.media.ToneGenerator(android.media.AudioManager.STREAM_ALARM, 80)
            tone?.startTone(android.media.ToneGenerator.TONE_PROP_BEEP, 150)
            getSystemService(android.os.Vibrator::class.java)
                ?.vibrate(android.os.VibrationEffect.createOneShot(120, android.os.VibrationEffect.DEFAULT_AMPLITUDE))
        }
        if (page != PAGE_LIDAR) return
        val s = vehicle.scan
        val p = lidarBehaviours.p
        lidarView.scan = s
        lidarView.minRange = p.minRange
        lidarView.info = s?.let { sc ->
            val n = sc.nearest(0f, 180f, p.minRange)
            "${vehicle.scanCount} scans" + (n?.let { "  ·  nearest %.2f m at %+.0f°".format(it.second, it.first) } ?: "  ·  nothing in range")
        } ?: ""
        lidarView.wedges = lidarWedges(s)
        lidarView.target = if (lidarBehaviours.mode == LidarBehaviours.Mode.FOLLOW && s != null)
            s.nearest(0f, p.trackAngle, p.minRange) else null
        lidarView.invalidate()
        lidarStatus.text = if (lidarDriver.running) cmd.why.replaceFirstChar { it.uppercase() } +
            (if (lidarDriver.drives) "   thr %+.2f  steer %+.2f".format(cmd.throttle, cmd.steer) else "")
            else "${lidarBehaviours.mode.label}: not running"
        lidarStatus.setTextColor(col(if (cmd.alarm) R.color.bad else R.color.fg))
    }

    /** The sectors the current behaviour looks at, red where it sees something. */
    private fun lidarWedges(s: LidarScan?): List<LidarView.Wedge> {
        val p = lidarBehaviours.p
        fun hot(from: Float, to: Float, r: Float) = s?.points(p.minRange)?.let { (a, d) ->
            var n = 0; for (i in a.indices) if (a[i] > from && a[i] < to && d[i] < r) n++; n
        } ?: 0
        return when (lidarBehaviours.mode) {
            LidarBehaviours.Mode.AVOID -> {
                val r = p.responseDist * 1.5f
                listOf(LidarView.Wedge(-20f, 20f, r, hot(-20f, 20f, r) >= p.minPoints),
                    LidarView.Wedge(20f, p.laserAngle, r, hot(20f, p.laserAngle, r) >= p.minPoints),
                    LidarView.Wedge(-p.laserAngle, -20f, r, hot(-p.laserAngle, -20f, r) >= p.minPoints))
            }
            LidarBehaviours.Mode.FOLLOW -> listOf(LidarView.Wedge(-p.trackAngle, p.trackAngle, p.followDist, false))
            LidarBehaviours.Mode.WARN -> listOf(LidarView.Wedge(-p.warnAngle, p.warnAngle, p.warnDist, hot(-p.warnAngle, p.warnAngle, p.warnDist) > 0))
            LidarBehaviours.Mode.OFF -> emptyList()
        }
    }

    private fun loadLidarParams() {
        val p = lidarBehaviours.p
        fun f(k: String, d: Float) = prefs.getFloat("lidar_$k", d)
        p.linear = f("linear", p.linear); p.angular = f("angular", p.angular); p.responseDist = f("responseDist", p.responseDist)
        p.laserAngle = f("laserAngle", p.laserAngle); p.minPoints = f("minPoints", p.minPoints.toFloat()).toInt()
        p.followDist = f("followDist", p.followDist); p.maxLinear = f("maxLinear", p.maxLinear); p.maxAngular = f("maxAngular", p.maxAngular)
        p.priorityAngle = f("priorityAngle", p.priorityAngle); p.trackAngle = f("trackAngle", p.trackAngle)
        p.warnDist = f("warnDist", p.warnDist); p.warnAngle = f("warnAngle", p.warnAngle); p.minRange = f("minRange", p.minRange)
        p.skidSteer = prefs.getBoolean("lidar_skidSteer", false)
        lidarSound = prefs.getBoolean("lidar_sound", true)
        lidarBehaviours.mode = runCatching { LidarBehaviours.Mode.valueOf(prefs.getString("lidar_mode", "OFF")!!) }.getOrDefault(LidarBehaviours.Mode.OFF)
    }

    private fun saveLidarParams() {
        val p = lidarBehaviours.p
        prefs.edit().apply {
            mapOf("linear" to p.linear, "angular" to p.angular, "responseDist" to p.responseDist, "laserAngle" to p.laserAngle,
                "minPoints" to p.minPoints.toFloat(), "followDist" to p.followDist, "maxLinear" to p.maxLinear,
                "maxAngular" to p.maxAngular, "priorityAngle" to p.priorityAngle, "trackAngle" to p.trackAngle,
                "warnDist" to p.warnDist, "warnAngle" to p.warnAngle, "minRange" to p.minRange).forEach { (k, x) -> putFloat("lidar_$k", x) }
            putBoolean("lidar_skidSteer", p.skidSteer)
            putBoolean("lidar_sound", lidarSound)
        }.apply()
    }

    // =========================================================================
    // Log page
    // =========================================================================

    private fun logPage(): View {
        logText = text("", 12f).apply { typeface = Typeface.MONOSPACE; setPadding(dp(12), dp(8), dp(12), dp(8)); setTextIsSelectable(true) }
        return ScrollView(this).apply { addView(logText) }
    }

    private fun refreshLog() {
        val msgs = synchronized(vehicle.messages) { vehicle.messages.toList() }
        if (msgs.size == logShown) return
        logShown = msgs.size
        logText.text = if (msgs.isEmpty()) "No messages yet" else
            msgs.asReversed().joinToString("\n") { "${Vehicle.SEVERITY.getOrElse(it.severity) { "?" }.padEnd(6)} ${it.text}" }
    }

    // =========================================================================
    // Events and the refresh loop
    // =========================================================================

    private fun onVehicleEvent(e: VehicleEvent) {
        when (e) {
            is VehicleEvent.Text -> {
                lastMsg.text = e.text
                lastMsg.setTextColor(col(if (e.severity <= 3) R.color.bad else if (e.severity == 4) R.color.warn else R.color.muted))
                if (e.severity <= 3 || e.text.startsWith("PreArm") || e.text.startsWith("Arm")) toast(e.text)
            }
            is VehicleEvent.Link -> toast(e.text)
            is VehicleEvent.ParamsDone -> {
                paramStatus.text = if (e.received >= e.total && e.total > 0) "${e.total} parameters"
                    else "Got ${e.received} of ${e.total}: press Refresh to try again"
                // Values the file staged that now match the vehicle are no longer changes.
                staged.entries.removeIf { (k, v) -> vehicle.paramValue(k)?.let { abs(it - v) < 1e-6f } == true }
                refreshParamList()
            }
            is VehicleEvent.ParamWritten -> {
                if (e.ok) paramWriteOk++ else paramWriteFail.add(e.name)
                val left = vehicle.paramWritesPending
                paramStatus.text = "Written $paramWriteOk" + (if (paramWriteFail.isNotEmpty()) ", failed: ${paramWriteFail.joinToString()}" else "") +
                    if (left > 0) " ($left left)" else ""
                if (left == 0 && paramWriteFail.isEmpty() && paramWriteOk == 1) toast("${e.name} = ${fmt(e.value)}")
                if (left == 0) refreshParamList()
            }
            is VehicleEvent.MissionUploaded -> { planStatus.text = e.text; toast(e.text) }
            is VehicleEvent.MissionDownloaded -> {
                planStatus.text = e.text; toast(e.text)
                if (e.items != null && e.type == Vehicle.MISSION_TYPE_FENCE) {
                    fence.clear(); fence.addAll(Fence.fromItems(e.items)); fenceZone = -1; fenceChanged(); planMap.fit()
                } else if (e.items != null) { plan.clear(); plan.addAll(e.items); planChanged(); planMap.fit() }
            }
            is VehicleEvent.CommandAck -> {
                val what = when (e.command) {
                    Mav.CMD_COMPONENT_ARM_DISARM -> "Arm/disarm"; Mav.CMD_DO_SET_MODE -> "Mode change"
                    Mav.CMD_PREFLIGHT_CALIBRATION -> "Calibration"; Mav.CMD_DO_START_MAG_CAL -> "Compass calibration"
                    Mav.CMD_DO_ACCEPT_MAG_CAL -> "Accept compass"; Mav.CMD_DO_CANCEL_MAG_CAL -> "Cancel compass"
                    Mav.CMD_PREFLIGHT_REBOOT_SHUTDOWN -> "Reboot"; Mav.CMD_DO_SET_HOME -> "Set home"
                    else -> "Command ${e.command}"
                }
                if (e.result != 0) toast("$what: ${Vehicle.cmdResult(e.result)}")
                else if (e.command == Mav.CMD_PREFLIGHT_REBOOT_SHUTDOWN) toast("Rebooting")
            }
            is VehicleEvent.AccelCalPos -> {
                accelPos = e.position
                val name = when (e.position) {
                    1 -> "LEVEL"; 2 -> "on its LEFT side"; 3 -> "on its RIGHT side"; 4 -> "NOSE DOWN"
                    5 -> "NOSE UP"; 6 -> "on its BACK"; else -> ""
                }
                when (e.position) {
                    in 1..6 -> { accelText.text = "Place the car $name, hold it still, then press Done."; accelNext.isEnabled = true }
                    16777215 -> { accelText.text = "Accelerometer calibrated. Reboot the board."; accelNext.isEnabled = false; toast("Accel calibration done") }
                    16777216 -> { accelText.text = "Calibration FAILED. Try again, holding each position still."; accelNext.isEnabled = false }
                }
                if (page != 3 && e.position in 1..6) toast("Accel cal: place the car $name (Setup tab)")
            }
            is VehicleEvent.MagCalReport -> {
                val ok = e.status == 4
                magText.text = if (ok) "Compass calibrated (fitness %.1f). Reboot the board.".format(e.fitness)
                    else "Compass calibration failed (status ${e.status}). Try again away from metal."
            }
        }
    }

    private val refresh = object : Runnable {
        override fun run() {
            try { refreshUi() } catch (e: Exception) { android.util.Log.e("Gcs", "refresh", e) }
            ui.postDelayed(this, 200)
        }
    }

    private fun refreshUi() {
        val v = vehicle
        val conn = v.connected
        setPill(pillLink, if (conn) "● ${v.linkText}" else "○ ${v.linkText}", if (conn) 1 else if (v.link != null) 2 else 0)
        setPill(pillMode, if (conn) "${v.modeName}${if (v.armed) " · ARMED" else ""}" else "mode —", if (!conn) 0 else if (v.armed) 3 else 1)
        val fix = Vehicle.GPS_FIX.getOrElse(v.gpsFix) { "fix ${v.gpsFix}" }
        setPill(pillGps, if (conn) "$fix · ${v.gpsSats} sats" else "GPS —",
            if (!conn) 0 else when { v.gpsFix >= 6 -> 1; v.gpsFix >= 3 -> 1; v.gpsFix >= 2 -> 2; else -> 3 })
        setPill(pillBatt, if (conn && v.voltage > 0.5f) "%.2f V%s".format(v.voltage, if (v.batteryPct >= 0) " · ${v.batteryPct}%" else "") else "battery —",
            if (conn && v.batteryPct in 0..20) 3 else 0)

        armBtn.text = if (v.armed) "Disarm" else "Arm"
        // EKF flags: attitude, horizontal velocity and horizontal position (absolute) all good.
        val ekfOk = v.ekfFlags and 0x01 != 0 && v.ekfFlags and 0x10 != 0
        hud.apply {
            roll = Math.toDegrees(v.roll.toDouble()).toFloat()
            pitch = Math.toDegrees(v.pitch.toDouble()).toFloat()
            heading = v.heading.toFloat()
            speed = v.groundSpeed; throttle = v.throttle
            mode = v.modeName; armed = v.armed; connected = conn
            this.ekfOk = ekfOk || v.ekfFlags == 0
            gps = if (conn) "GPS $fix  ${v.gpsSats} sats  hdop %.2f".format(v.gpsHdop) +
                (if (v.gpsHAcc > 0) "  ±%.2f m".format(v.gpsHAcc) else "") else "GPS —"
            battery = if (conn && v.voltage > 0.5f) "%.2f V  %.1f A".format(v.voltage, v.current) else ""
            wp = if (conn && v.customMode == 10) "WP ${v.missionSeq}  ${v.wpDist} m  xtrack %.1f m".format(v.xtrack)
                else if (v.rtcmBytes > 0) "RTCM sent ${v.rtcmBytes / 1024} kB" else ""
            invalidate()
        }

        val pos = if (v.hasPosition) Triple(v.lat, v.lon, v.yaw.let { Math.toDegrees(it.toDouble()).toFloat() }) else null
        val home = if (v.hasHome) Pair(v.homeLat, v.homeLon) else null
        for (m in listOf(flyMap, planMap)) {
            if (pos != null && m.vehicle == null && m === flyMap) m.centerOn(pos.first, pos.second, 18.0)
            if (pos != null && m.vehicle == null && m === planMap && plan.isEmpty()) m.centerOn(pos.first, pos.second, 18.0)
            m.vehicle = pos; m.home = home
            m.currentWp = if (v.customMode == 10) v.missionSeq - 1 else -1
        }
        if (pos != null && conn) flyMap.addTrail(pos.first, pos.second)
        flyMap.scan = v.scan
        if (v.customMode != 15) flyMap.guidedTarget = null
        if (page == 0) flyMap.invalidate() else if (page == 1) planMap.invalidate()
        if (v.missionBusy) planStatus.text = v.missionProgress
        if (v.paramsFetching) paramStatus.text = "Reading ${v.params.size} / ${v.paramCount}…"

        sendStick()

        if (page == 3) {
            val fresh = SystemClock.elapsedRealtime() - v.rcTime < 2000
            for (i in 0 until 8) {
                val pwm = v.rc[i]
                if (rcCal && fresh && pwm in 800..2200) { rcMin[i] = minOf(rcMin[i], pwm); rcMax[i] = maxOf(rcMax[i], pwm) }
                rcBars[i].progress = if (fresh && pwm > 0) (pwm - 900).coerceIn(0, 1200) else 0
                rcLabels[i].text = "${i + 1}: " + (if (fresh && pwm > 0) "$pwm" else "—") +
                    if (rcCal && rcMax[i] > 0) "  ${rcMin[i]}-${rcMax[i]}" else ""
            }
            radar.sectors = v.proximity; radar.times = v.proximityTime; radar.maxRange = v.proximityMax
            radar.invalidate()
            val p = v.magCalPct
            if (p.any { it > 0 } && !magText.text.startsWith("Compass")) magText.text = "Progress: " +
                p.withIndex().filter { it.value > 0 }.joinToString { "compass ${it.index + 1} ${it.value}%" }
            ntrip?.let { n ->
                val types = synchronized(n.types) { n.types.entries.joinToString(" ") { "${it.key}" } }
                ntripStatus.text = "${n.status}\n${n.bytes / 1024} kB, ${n.frames} frames${if (types.isNotEmpty()) "\nRTCM: $types" else ""}"
            }
        }
        if (page == PAGE_LOG) refreshLog()
    }

    // =========================================================================
    // Files (Storage Access Framework)
    // =========================================================================

    private fun openFile(req: Int) {
        startActivityForResult(Intent(Intent.ACTION_OPEN_DOCUMENT).apply {
            addCategory(Intent.CATEGORY_OPENABLE); type = "*/*"
        }, req)
    }

    private fun saveFile(req: Int, name: String) {
        startActivityForResult(Intent(Intent.ACTION_CREATE_DOCUMENT).apply {
            addCategory(Intent.CATEGORY_OPENABLE); type = "text/plain"; putExtra(Intent.EXTRA_TITLE, name)
        }, req)
    }

    @Deprecated("Activity result API needs AndroidX")
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        val uri: Uri = data?.data ?: return
        if (resultCode != RESULT_OK) return
        try {
            when (requestCode) {
                REQ_LOAD_PARAMS -> loadParamFile(readText(uri))
                REQ_SAVE_PARAMS -> { writeText(uri, paramFileText()); toast("Saved ${vehicle.params.size} parameters") }
                REQ_LOAD_MISSION -> {
                    val items = parseWaypoints(readText(uri))
                    plan.clear(); plan.addAll(items); planChanged(); planMap.fit()
                    toast("Loaded ${items.size} items. Press Write to send them.")
                }
                REQ_SAVE_MISSION -> { writeText(uri, waypointsText()); toast("Saved ${plan.size} items") }
            }
        } catch (e: Exception) {
            toast("File error: ${e.message}")
        }
    }

    private fun readText(uri: Uri) = contentResolver.openInputStream(uri)!!.use { it.readBytes().toString(Charsets.UTF_8) }
    private fun writeText(uri: Uri, s: String) = contentResolver.openOutputStream(uri, "wt")!!.use { it.write(s.toByteArray()) }

    /** Mission Planner / QGC "QGC WPL 110" .waypoints files; line 0 is home. */
    private fun parseWaypoints(s: String): List<MissionItem> {
        val lines = s.lines().map { it.trim() }.filter { it.isNotEmpty() }
        require(lines.firstOrNull()?.startsWith("QGC WPL") == true) { "not a QGC WPL / Mission Planner .waypoints file" }
        return lines.drop(1).mapNotNull { l ->
            val f = l.split(Regex("\\s+"))
            if (f.size < 12) return@mapNotNull null
            MissionItem(command = f[3].toInt(), p1 = f[4].toFloat(), p2 = f[5].toFloat(), p3 = f[6].toFloat(), p4 = f[7].toFloat(),
                lat = f[8].toDouble(), lon = f[9].toDouble(), alt = f[10].toFloat(), frame = f[2].toInt(), autocontinue = f[11].toInt())
                .takeIf { f[0] != "0" }
        }
    }

    private fun waypointsText(): String {
        val sb = StringBuilder("QGC WPL 110\n")
        val hl = if (vehicle.hasHome) vehicle.homeLat else plan.firstOrNull { it.hasPosition }?.lat ?: 0.0
        val ho = if (vehicle.hasHome) vehicle.homeLon else plan.firstOrNull { it.hasPosition }?.lon ?: 0.0
        sb.append("0\t1\t0\t16\t0\t0\t0\t0\t%.8f\t%.8f\t0\t1\n".format(java.util.Locale.US, hl, ho))
        plan.forEachIndexed { i, it ->
            sb.append("%d\t0\t%d\t%d\t%s\t%s\t%s\t%s\t%.8f\t%.8f\t%s\t%d\n".format(java.util.Locale.US,
                i + 1, it.frame, it.command, fmt(it.p1), fmt(it.p2), fmt(it.p3), fmt(it.p4), it.lat, it.lon, fmt(it.alt), it.autocontinue))
        }
        return sb.toString()
    }

    companion object {
        const val PAGE_LIDAR = 4
        const val PAGE_LOG = 5
        const val REQ_LOAD_PARAMS = 1
        const val REQ_SAVE_PARAMS = 2
        const val REQ_LOAD_MISSION = 3
        const val REQ_SAVE_MISSION = 4
    }
}

/** Lays children out left to right, wrapping to a new line when the width runs out. */
class FlowLayout(context: android.content.Context) : ViewGroup(context) {
    override fun onMeasure(wSpec: Int, hSpec: Int) {
        val maxW = MeasureSpec.getSize(wSpec) - paddingLeft - paddingRight
        var x = 0; var y = 0; var lineH = 0
        for (i in 0 until childCount) {
            val c = getChildAt(i)
            measureChild(c, MeasureSpec.makeMeasureSpec(maxW, MeasureSpec.AT_MOST), hSpec)
            val m = c.layoutParams as? MarginLayoutParams
            val cw = c.measuredWidth + (m?.marginEnd ?: 0); val ch = c.measuredHeight + (m?.bottomMargin ?: 0)
            if (x > 0 && x + cw > maxW) { x = 0; y += lineH; lineH = 0 }
            x += cw; lineH = maxOf(lineH, ch)
        }
        setMeasuredDimension(MeasureSpec.getSize(wSpec), resolveSize(y + lineH + paddingTop + paddingBottom, hSpec))
    }

    override fun onLayout(changed: Boolean, l: Int, t: Int, r: Int, b: Int) {
        val maxW = r - l - paddingLeft - paddingRight
        var x = 0; var y = 0; var lineH = 0
        for (i in 0 until childCount) {
            val c = getChildAt(i)
            val m = c.layoutParams as? MarginLayoutParams
            val cw = c.measuredWidth + (m?.marginEnd ?: 0); val ch = c.measuredHeight + (m?.bottomMargin ?: 0)
            if (x > 0 && x + cw > maxW) { x = 0; y += lineH; lineH = 0 }
            c.layout(paddingLeft + x, paddingTop + y, paddingLeft + x + c.measuredWidth, paddingTop + y + c.measuredHeight)
            x += cw; lineH = maxOf(lineH, ch)
        }
    }

    override fun generateLayoutParams(attrs: android.util.AttributeSet?) = MarginLayoutParams(context, attrs)
    override fun generateDefaultLayoutParams() = MarginLayoutParams(-2, -2)
    override fun checkLayoutParams(p: LayoutParams?) = p is MarginLayoutParams
    override fun generateLayoutParams(p: LayoutParams?) = MarginLayoutParams(p)
}

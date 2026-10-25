package org.ardupilot.esp32gcs.desktop

import org.ardupilot.esp32gcs.Fence
import org.ardupilot.esp32gcs.FenceZone
import org.ardupilot.esp32gcs.LapCounter
import org.ardupilot.esp32gcs.LidarBehaviours
import org.ardupilot.esp32gcs.LidarDriver
import org.ardupilot.esp32gcs.LidarScan
import org.ardupilot.esp32gcs.Mav
import org.ardupilot.esp32gcs.MissionItem
import org.ardupilot.esp32gcs.Param
import org.ardupilot.esp32gcs.Patrol
import org.ardupilot.esp32gcs.Platform
import org.ardupilot.esp32gcs.TcpLink
import org.ardupilot.esp32gcs.UdpLink
import org.ardupilot.esp32gcs.Vehicle
import org.ardupilot.esp32gcs.VehicleEvent
import org.ardupilot.esp32gcs.ZigZag
import org.ardupilot.esp32gcs.nav.IndoorNav
import java.awt.BorderLayout
import java.awt.Color
import java.awt.Component
import java.awt.Dimension
import java.awt.FlowLayout
import java.awt.Font
import java.awt.GridBagConstraints
import java.awt.GridBagLayout
import java.awt.Insets
import java.awt.KeyboardFocusManager
import java.awt.event.KeyEvent
import java.io.File
import java.util.prefs.Preferences
import javax.swing.BorderFactory
import javax.swing.Box
import javax.swing.BoxLayout
import javax.swing.DefaultListModel
import javax.swing.JButton
import javax.swing.JCheckBox
import javax.swing.JComboBox
import javax.swing.JComponent
import javax.swing.JFileChooser
import javax.swing.JLabel
import javax.swing.JList
import javax.swing.JMenuItem
import javax.swing.JOptionPane
import javax.swing.JPanel
import javax.swing.JPopupMenu
import javax.swing.JScrollPane
import javax.swing.JSlider
import javax.swing.JSplitPane
import javax.swing.JTabbedPane
import javax.swing.JTable
import javax.swing.JTextArea
import javax.swing.JTextField
import javax.swing.JToggleButton
import javax.swing.ListSelectionModel
import javax.swing.Timer
import javax.swing.event.DocumentEvent
import javax.swing.event.DocumentListener
import javax.swing.filechooser.FileNameExtensionFilter
import javax.swing.table.AbstractTableModel
import kotlin.math.abs

/**
 * ESP32 Rover GCS for Windows: the Android app's Fly, Plan (waypoints, zig-zag,
 * patrol, fences), LiDAR, Params and Log, on the same core. Calibration and
 * NTRIP stay with Mission Planner on the PC.
 */
class GcsPanel : JPanel(BorderLayout()) {
    private val prefs = Preferences.userRoot().node("esp32-rover-gcs")
    private val vehicle = Vehicle()

    // status bar
    private val linkLabel = JLabel("Not connected")
    private val modeLabel = JLabel("mode —")
    private val gpsLabel = JLabel("GPS —")
    private val battLabel = JLabel("battery —")
    private val lastMsg = JLabel(" ")
    private val tabs = JTabbedPane()

    // connection (declared before init, which builds the UI)
    private val linkType = JComboBox(arrayOf("TCP", "UDP"))
    private val hostField = JTextField(12)
    private val portField = JTextField(5)

    // fly
    private val hud = HudPanel()
    private val flyMap = MapPanel()
    private val armBtn = JButton("Arm")
    private val keyDrive = JCheckBox("Drive with arrow keys")
    private val power = JSlider(10, 100, prefs.getInt("drive_power", 30))
    private val keysDown = HashSet<Int>()
    private var keyZeros = 0

    // plan
    private val planMap = MapPanel()
    private val plan = ArrayList<MissionItem>()
    private val planList = DefaultListModel<String>()
    private val planStatus = JLabel("Click the map to add a waypoint, drag one to move it, click it to edit. Right-click for more.")
    private val zigzag = ZigZag()
    private val lapCounter = LapCounter()
    private var planSpeed = 0f
    private val fence = ArrayList<FenceZone>()
    private var fenceZone = -1
    private val fenceMode = JToggleButton("Fence")
    private val missionBar = JPanel(FlowLayout(FlowLayout.LEFT, 6, 4))
    private val fenceBar = JPanel(FlowLayout(FlowLayout.LEFT, 6, 4))

    // lidar
    private val behaviours = LidarBehaviours()
    private val driver = LidarDriver(vehicle, behaviours)
    private val lidarPanel = LidarPanel()
    private val lidarStatus = JLabel("Off")
    private val lidarStart = JButton("Start")
    private val lidarHint = JTextArea(4, 40)
    private val modeButtons = HashMap<LidarBehaviours.Mode, JToggleButton>()
    private var lastBeep = 0L

    // params
    private val params = ArrayList<Param>()
    private val staged = LinkedHashMap<String, Float>()
    private val paramFilter = JTextField(20)
    private val paramStatus = JLabel("Press Refresh to read the parameters")
    private lateinit var paramModel: AbstractTableModel

    // indoor
    private val indoor = IndoorNav(vehicle)
    private val indoorPanel = IndoorPanel(indoor)
    private val indoorStatus = JLabel("Off")
    private val indoorInfo = JLabel(" ")
    private val indoorArm = JButton("Arm")
    private val indoorAddMode = JToggleButton("Click adds a waypoint")
    private val indoorScanBtn = JButton("Use 1° scans (PRX1_SCAN_FULL 1)")
    private val indoorWps = DefaultListModel<String>()
    private var indoorWpShown: List<String> = emptyList()

    // log
    private val logArea = JTextArea()
    private var logShown = -1

    init {
        vehicle.onEvent = { onEvent(it) }
        loadSettings()

        add(topBar(), BorderLayout.NORTH)
        tabs.addTab("Fly", flyTab())
        tabs.addTab("Plan", planTab())
        tabs.addTab("LiDAR", lidarTab())
        tabs.addTab("Params", paramsTab())
        tabs.addTab("Indoor", indoorTab())
        tabs.addTab("Log", JScrollPane(logArea.apply { isEditable = false; font = Font(Font.MONOSPACED, Font.PLAIN, 12) }))
        add(tabs, BorderLayout.CENTER)
        add(lastMsg.apply { border = BorderFactory.createEmptyBorder(3, 8, 3, 8) }, BorderLayout.SOUTH)

        keyboard()
        Timer(200) { try { refresh() } catch (e: Exception) { Platform.logError("refresh", e) } }.start()
        Timer(100) { try { lidarTick(); keyTick() } catch (e: Exception) { Platform.logError("lidar", e) } }.start()
        if (prefs.getBoolean("autoconnect", true)) connect()
    }

    /** On closing the window: stop any LiDAR behaviour (zero sticks) and close the link. */
    fun shutdown() {
        driver.stop()
        indoor.stop(); indoor.close()
        vehicle.close()
    }

    // =========================================================================
    // helpers
    // =========================================================================

    private fun btn(s: String, f: () -> Unit) = JButton(s).apply { addActionListener { f() } }
    private fun row(vararg c: Component) = JPanel(FlowLayout(FlowLayout.LEFT, 6, 4)).apply { c.forEach { add(it) } }
    private fun info(msg: String) = JOptionPane.showMessageDialog(this, msg)
    private fun ask(title: String, msg: String): Boolean =
        JOptionPane.showConfirmDialog(this, msg, title, JOptionPane.OK_CANCEL_OPTION) == JOptionPane.OK_OPTION
    private fun fmt(f: Float) = if (f == f.toLong().toFloat()) f.toLong().toString() else f.toString()
    private fun requireLink(): Boolean { if (!vehicle.connected) info("Not connected to the vehicle"); return vehicle.connected }

    /** A small form: label / field rows. */
    private fun form(vararg rows: Pair<String, JComponent>) = JPanel(GridBagLayout()).apply {
        val c = GridBagConstraints().apply { insets = Insets(3, 3, 3, 3); anchor = GridBagConstraints.WEST }
        rows.forEachIndexed { i, (l, f) ->
            c.gridx = 0; c.gridy = i; c.fill = GridBagConstraints.NONE; c.weightx = 0.0; add(JLabel(l), c)
            c.gridx = 1; c.fill = GridBagConstraints.HORIZONTAL; c.weightx = 1.0; add(f, c)
        }
    }

    // =========================================================================
    // top bar and connection
    // =========================================================================

    private fun topBar(): JComponent {
        linkType.selectedItem = prefs.get("link_type", "TCP")
        hostField.text = prefs.get("link_host", "192.168.4.1")
        portField.text = prefs.getInt("link_port", 5760).toString()
        val bar = row(linkType, hostField, portField,
            btn("Connect") {
                prefs.put("link_type", linkType.selectedItem as String); prefs.put("link_host", hostField.text.trim())
                prefs.putInt("link_port", portField.text.trim().toIntOrNull() ?: 5760); prefs.putBoolean("autoconnect", true)
                connect()
            },
            btn("Disconnect") { prefs.putBoolean("autoconnect", false); vehicle.disconnect() },
            Box.createHorizontalStrut(16), linkLabel, Box.createHorizontalStrut(12), modeLabel,
            Box.createHorizontalStrut(12), gpsLabel, Box.createHorizontalStrut(12), battLabel)
        bar.toolTipText = "Board: TCP 192.168.4.1:5760 on the ardupilot123 WiFi. SITL: TCP 127.0.0.1:5760. UDP builds: UDP, empty address, 14550."
        return bar
    }

    private fun connect() {
        val type = prefs.get("link_type", "TCP"); val host = prefs.get("link_host", "192.168.4.1"); val port = prefs.getInt("link_port", 5760)
        vehicle.connect { if (type == "UDP") UdpLink(host, port) else TcpLink(host, port) }
    }

    // =========================================================================
    // Fly
    // =========================================================================

    private fun flyTab(): JComponent {
        val modes = listOf(0 to "Manual", 4 to "Hold", 10 to "Auto", 11 to "RTL", 15 to "Guided", 1 to "Acro", 3 to "Steering", 5 to "Loiter", 12 to "SmartRTL")
        armBtn.addActionListener {
            if (!requireLink()) return@addActionListener
            if (vehicle.armed) vehicle.arm(false) else if (ask("Arm", "Arm the motors? Wheels off the ground for the first test.")) vehicle.arm(true)
        }
        val modeRow = row(armBtn, *modes.map { (n, s) -> btn(s) { if (requireLink()) vehicle.setMode(n) } }.toTypedArray())
        power.addChangeListener { prefs.putInt("drive_power", power.value) }
        keyDrive.toolTipText = "Arrow keys or W A S D: stick overrides (Manual, Acro, Steering). Needs SYSID_MYGCS = 255."
        val side = JPanel(BorderLayout()).apply {
            add(hud, BorderLayout.NORTH)
            add(JPanel().apply {
                layout = BoxLayout(this, BoxLayout.Y_AXIS)
                add(row(keyDrive)); add(row(JLabel("Power"), power))
                add(row(btn("Follow") { flyMap.follow = true }, btn("Fit") { flyMap.fit() },
                    btn("Map / Satellite") { toggleSource() }, btn("Clear trail") { flyMap.clearTrail() }))
                add(row(JLabel("<html><small>Click the map in Guided to drive there.<br>Right-click: drive here, set home.</small></html>")))
            }, BorderLayout.CENTER)
            preferredSize = Dimension(380, 0)
        }
        flyMap.onClick = { la, lo -> if (vehicle.connected && vehicle.customMode == 15 && ask("Guided", "Drive here?")) goTo(la, lo) }
        flyMap.onMenu = { la, lo, x, y -> flyMenu(la, lo, x, y) }
        return JPanel(BorderLayout()).apply {
            add(modeRow, BorderLayout.NORTH)
            add(JSplitPane(JSplitPane.HORIZONTAL_SPLIT, side, flyMap).apply { resizeWeight = 0.0 }, BorderLayout.CENTER)
        }
    }

    private fun toggleSource() {
        val next = if (flyMap.source == MapPanel.Source.STREET) MapPanel.Source.SATELLITE else MapPanel.Source.STREET
        flyMap.source = next; planMap.source = next; prefs.put("map", next.name)
    }

    private fun goTo(la: Double, lo: Double) {
        if (!requireLink()) return
        if (vehicle.customMode != 15) vehicle.setMode(15)
        vehicle.goTo(la, lo)
        Timer(700) { vehicle.goTo(la, lo) }.apply { isRepeats = false }.start()
        flyMap.guidedTarget = Pair(la, lo)
    }

    private fun flyMenu(la: Double, lo: Double, x: Int, y: Int) {
        JPopupMenu().apply {
            add(JMenuItem("Drive here (Guided)").apply { addActionListener { goTo(la, lo) } })
            add(JMenuItem("Set home here").apply { addActionListener { if (requireLink() && ask("Set home", "Move home here?")) vehicle.setHome(la, lo) } })
        }.show(flyMap, x, y)
    }

    /** Arrow keys / WASD while "Drive with arrow keys" is ticked: the phone stick of the Android app. */
    private fun keyboard() {
        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher { e ->
            if (!keyDrive.isSelected || e.component is JTextField) return@addKeyEventDispatcher false
            val k = e.keyCode
            if (k !in listOf(KeyEvent.VK_UP, KeyEvent.VK_DOWN, KeyEvent.VK_LEFT, KeyEvent.VK_RIGHT, KeyEvent.VK_W, KeyEvent.VK_A, KeyEvent.VK_S, KeyEvent.VK_D))
                return@addKeyEventDispatcher false
            when (e.id) { KeyEvent.KEY_PRESSED -> keysDown.add(k); KeyEvent.KEY_RELEASED -> keysDown.remove(k) }
            true
        }
        // A key let go while another window has the focus (a dialog, Alt-Tab to another app)
        // never sends KEY_RELEASED here, and the car would keep driving: forget the held keys
        // whenever the active window changes.
        KeyboardFocusManager.getCurrentKeyboardFocusManager().addPropertyChangeListener("activeWindow") { keysDown.clear() }
    }

    private val keysHeld get() = keyDrive.isSelected && keysDown.isNotEmpty()

    private fun keyTick() {
        if (keysHeld) {
            val p = power.value / 100f
            val fwd = (if (KeyEvent.VK_UP in keysDown || KeyEvent.VK_W in keysDown) 1f else 0f) - (if (KeyEvent.VK_DOWN in keysDown || KeyEvent.VK_S in keysDown) 1f else 0f)
            val right = (if (KeyEvent.VK_RIGHT in keysDown || KeyEvent.VK_D in keysDown) 1f else 0f) - (if (KeyEvent.VK_LEFT in keysDown || KeyEvent.VK_A in keysDown) 1f else 0f)
            vehicle.manualControl(right * p, fwd * p); keyZeros = 3
        } else if (keyZeros > 0) { keyZeros--; vehicle.manualControl(0f, 0f) }
    }

    // =========================================================================
    // Plan: waypoints, zig-zag, patrol, fences
    // =========================================================================

    private fun planTab(): JComponent {
        listOf(btn("Read") { if (requireLink()) vehicle.downloadMission() },
            btn("Write") { writeMission() },
            btn("Speed…") { askSpeed() },
            btn("Zig zag…") { zigZagDialog() },
            btn("Patrol…") { patrolDialog() },
            btn("Clear") { if (ask("Clear plan", "Remove all waypoints from the plan?")) { plan.clear(); planChanged() } },
            btn("Start Auto") { startAuto() },
            btn("Load…") { loadMission() },
            btn("Save…") { saveMission() },
            btn("Fit") { planMap.fit() }).forEach { missionBar.add(it) }
        listOf(btn("Read") { if (requireLink()) vehicle.downloadFence() },
            btn("Write") { writeFence() },
            btn("+ Stay-in") { addZone(FenceZone.Kind.POLYGON_IN) },
            btn("+ Stay-out") { addZone(FenceZone.Kind.POLYGON_OUT) },
            btn("+ Circle") { addZone(FenceZone.Kind.CIRCLE_OUT) },
            btn("Enable…") { enableFence() },
            btn("Clear") { if (ask("Clear fence", "Remove all fence zones? Write afterwards to clear the vehicle's too.")) { fence.clear(); fenceZone = -1; fenceChanged() } })
            .forEach { fenceBar.add(it) }
        fenceBar.isVisible = false
        fenceMode.addActionListener { setFenceMode(fenceMode.isSelected) }

        planMap.editable = true
        planMap.mission = plan
        planMap.onClick = { la, lo -> if (fenceMode.isSelected) fenceClick(la, lo) else { plan.add(MissionItem(lat = la, lon = lo)); planChanged() } }
        planMap.onWaypointClick = { editItem(it) }
        planMap.onFenceClick = { z, p -> editZone(z, p) }
        planMap.onEdited = { if (fenceMode.isSelected) fenceChanged() else planChanged() }
        planMap.onMenu = { la, lo, x, y ->
            JPopupMenu().apply {
                add(JMenuItem("Zig-zag point A here").apply { addActionListener { zigzag.a = doubleArrayOf(la, lo); zigChanged() } })
                add(JMenuItem("Zig-zag point B here").apply { addActionListener { zigzag.b = doubleArrayOf(la, lo); zigChanged() } })
                add(JMenuItem("Add waypoint here").apply { addActionListener { plan.add(MissionItem(lat = la, lon = lo)); planChanged() } })
            }.show(planMap, x, y)
        }
        val list = JList(planList).apply {
            selectionMode = ListSelectionModel.SINGLE_SELECTION
            addMouseListener(object : java.awt.event.MouseAdapter() {
                override fun mouseClicked(e: java.awt.event.MouseEvent) {
                    if (e.clickCount == 2 && selectedIndex >= 0) { if (fenceMode.isSelected) editZone(selectedIndex, -1) else editItem(selectedIndex) }
                }
            })
        }
        val top = JPanel(BorderLayout()).apply {
            add(row(fenceMode), BorderLayout.WEST)
            add(JPanel().apply { layout = BoxLayout(this, BoxLayout.Y_AXIS); add(missionBar); add(fenceBar) }, BorderLayout.CENTER)
            add(planStatus.apply { border = BorderFactory.createEmptyBorder(0, 8, 4, 8) }, BorderLayout.SOUTH)
        }
        planChanged()
        return JPanel(BorderLayout()).apply {
            add(top, BorderLayout.NORTH)
            add(JSplitPane(JSplitPane.HORIZONTAL_SPLIT, planMap, JScrollPane(list).apply { preferredSize = Dimension(320, 0) }).apply { resizeWeight = 1.0 }, BorderLayout.CENTER)
        }
    }

    private fun itemLabel(it: MissionItem) = when (it.command) {
        Mav.CMD_NAV_WAYPOINT -> "Waypoint" + if (it.p1 > 0) ", wait ${it.p1.toInt()} s" else ""
        19 -> "Loiter ${it.p1.toInt()} s"
        Mav.CMD_NAV_RETURN_TO_LAUNCH -> "Return to launch"
        Mav.CMD_DO_CHANGE_SPEED -> "Speed %.1f m/s".format(it.p2)
        Patrol.CMD_DO_JUMP -> "Repeat from item ${it.p1.toInt()}: " + if (it.p2 < 0) "forever (patrol)" else "${it.p2.toInt()} more times (patrol)"
        else -> "Command ${it.command}"
    }

    private fun planChanged() {
        planMap.mission = plan; flyMap.mission = plan
        planMap.repaint(); flyMap.repaint()
        if (fenceMode.isSelected) return
        planList.clear()
        var total = 0.0
        var prev: DoubleArray? = if (vehicle.hasHome) doubleArrayOf(vehicle.homeLat, vehicle.homeLon) else null
        plan.forEachIndexed { i, it ->
            var d = ""
            if (it.hasPosition) {
                prev?.let { p -> val m = ZigZag.distance(p, doubleArrayOf(it.lat, it.lon)); total += m; d = "  %.0f m".format(m) }
                prev = doubleArrayOf(it.lat, it.lon)
            }
            planList.addElement("${i + 1}. ${itemLabel(it)}$d")
        }
        planList.addElement(if (plan.isEmpty()) "No waypoints" else "Total %.0f m".format(total))
    }

    private fun editItem(i: Int) {
        if (i >= plan.size) return
        val it = plan[i]
        val kinds = listOf(Mav.CMD_NAV_WAYPOINT to "Waypoint", 19 to "Loiter (time)", Mav.CMD_NAV_RETURN_TO_LAUNCH to "Return to launch", Mav.CMD_DO_CHANGE_SPEED to "Change speed")
        val kind = JComboBox(kinds.map { k -> k.second }.toTypedArray()).apply { selectedIndex = kinds.indexOfFirst { k -> k.first == it.command }.coerceAtLeast(0) }
        val wait = JTextField(if (it.command == Mav.CMD_DO_CHANGE_SPEED) "" else fmt(it.p1), 6)
        val speed = JTextField(if (it.command == Mav.CMD_DO_CHANGE_SPEED) fmt(it.p2) else "", 6)
        val opts = arrayOf("Save", "Delete", "Move up", "Move down", "Cancel")
        val r = JOptionPane.showOptionDialog(this, form("Command" to kind, "Wait / loiter (s)" to wait, "Speed (m/s)" to speed),
            "Item ${i + 1}", JOptionPane.DEFAULT_OPTION, JOptionPane.PLAIN_MESSAGE, null, opts, opts[0])
        when (r) {
            0 -> {
                val cmd = kinds[kind.selectedIndex].first
                it.command = cmd
                when (cmd) {
                    Mav.CMD_DO_CHANGE_SPEED -> { it.p1 = 1f; it.p2 = speed.text.toFloatOrNull() ?: 1f; it.p3 = -1f; it.lat = 0.0; it.lon = 0.0 }
                    Mav.CMD_NAV_RETURN_TO_LAUNCH -> { it.p1 = 0f; it.lat = 0.0; it.lon = 0.0 }
                    else -> it.p1 = wait.text.toFloatOrNull() ?: 0f
                }
            }
            1 -> plan.removeAt(i)
            2 -> if (i > 0) plan.add(i - 1, plan.removeAt(i))
            3 -> if (i < plan.size - 1) plan.add(i + 1, plan.removeAt(i))
        }
        planChanged()
    }

    private fun askSpeed() {
        val s = JOptionPane.showInputDialog(this, "Mission speed in m/s (empty = WP_SPEED).\nAdded as a speed change at the start when written.", if (planSpeed > 0) fmt(planSpeed) else "")
        if (s != null) planSpeed = s.toFloatOrNull() ?: 0f
    }

    private fun writeMission() {
        if (!requireLink()) return
        if (plan.isEmpty()) { info("The plan is empty"); return }
        val items = ArrayList<MissionItem>()
        if (planSpeed > 0 && plan.firstOrNull()?.command != Mav.CMD_DO_CHANGE_SPEED)
            items.add(MissionItem(command = Mav.CMD_DO_CHANGE_SPEED, p1 = 1f, p2 = planSpeed, p3 = -1f))
        items.addAll(plan)
        // A DO_JUMP targets plan item 1; a speed change put in front shifts nothing it jumps over.
        vehicle.uploadMission(items)
    }

    private fun startAuto() {
        if (!requireLink()) return
        if (ask("Start mission", "Switch to Auto and run the mission from the start?")) {
            lapCounter.reset(); vehicle.setCurrentWaypoint(1); vehicle.setMode(10)
        }
    }

    // --- zig-zag ---

    private fun zigChanged() {
        planMap.zigA = zigzag.a; planMap.zigB = zigzag.b; flyMap.zigA = zigzag.a; flyMap.zigB = zigzag.b
        planMap.repaint(); flyMap.repaint()
        zigzag.a?.let { prefs.put("zz_a", "${it[0]},${it[1]}") }; zigzag.b?.let { prefs.put("zz_b", "${it[0]},${it[1]}") }
    }

    private fun zigZagDialog() {
        val turnR = vehicle.paramValue("TURN_RADIUS")
        if (zigzag.runOutM < 0f) zigzag.runOutM = turnR ?: 1f
        val ab = JLabel()
        fun showAB() {
            fun f(p: DoubleArray?) = p?.let { "%.6f, %.6f".format(it[0], it[1]) } ?: "not set"
            ab.text = "<html>A: ${f(zigzag.a)}<br>B: ${f(zigzag.b)}" + (if (zigzag.a != null && zigzag.b != null) "<br>A to B: %.1f m".format(zigzag.lengthM) else "") + "</html>"
        }
        showAB()
        val carA = btn("A = car here") { if (vehicle.hasPosition) { zigzag.a = doubleArrayOf(vehicle.lat, vehicle.lon); zigChanged(); showAB() } else info("No GPS position from the car") }
        val carB = btn("B = car here") { if (vehicle.hasPosition) { zigzag.b = doubleArrayOf(vehicle.lat, vehicle.lon); zigChanged(); showAB() } else info("No GPS position from the car") }
        val side = JTextField(fmt(zigzag.sideDistM), 6); val lines = JTextField(zigzag.lines.toString(), 6)
        val wait = JTextField(fmt(zigzag.waitS), 6); val runout = JTextField(fmt(zigzag.runOutM), 6)
        val dir = JComboBox(arrayOf("Right of A→B", "Left of A→B")).apply { selectedIndex = if (zigzag.right) 0 else 1 }
        val note = JLabel("<html><small>Or right-click the Plan map for A and B. Run-out extends each line past both ends so the car turns outside the area." +
            (turnR?.let { " TURN_RADIUS is ${fmt(it)} m: a U-turn needs about ${fmt(2 * it)} m." } ?: "") + "</small></html>")
        val panel = JPanel(BorderLayout(0, 6)).apply {
            add(JPanel(BorderLayout()).apply { add(ab, BorderLayout.CENTER); add(row(carA, carB), BorderLayout.SOUTH) }, BorderLayout.NORTH)
            add(form("Line spacing (m)" to side, "Lines" to lines, "Side" to dir, "Wait at ends (s)" to wait, "Run-out (m)" to runout), BorderLayout.CENTER)
            add(note.apply { preferredSize = Dimension(380, 60) }, BorderLayout.SOUTH)
        }
        if (JOptionPane.showConfirmDialog(this, panel, "Zig zag", JOptionPane.OK_CANCEL_OPTION, JOptionPane.PLAIN_MESSAGE) != JOptionPane.OK_OPTION) return
        zigzag.sideDistM = side.text.toFloatOrNull() ?: zigzag.sideDistM
        zigzag.lines = lines.text.toIntOrNull() ?: zigzag.lines
        zigzag.waitS = (wait.text.toFloatOrNull() ?: 0f).coerceAtLeast(0f)
        zigzag.runOutM = (runout.text.toFloatOrNull() ?: 0f).coerceAtLeast(0f)
        zigzag.right = dir.selectedIndex == 0
        prefs.putFloat("zz_side", zigzag.sideDistM); prefs.putInt("zz_lines", zigzag.lines); prefs.putBoolean("zz_right", zigzag.right)
        prefs.putFloat("zz_wait", zigzag.waitS); prefs.putFloat("zz_runout", zigzag.runOutM)
        zigzag.problem()?.let { info(it); return }
        if (plan.isNotEmpty() && !ask("Replace the plan", "Replace the ${plan.size} items with the zig-zag lanes?")) return
        plan.clear(); plan.addAll(zigzag.waypoints()); planChanged(); planMap.fit()
        planStatus.text = "Zig zag: ${zigzag.lines} lines of %.0f m, %s m apart. Write, then Start Auto.".format(zigzag.lengthM + 2 * zigzag.runOutM, fmt(zigzag.sideDistM))
    }

    // --- patrol ---

    private fun patrolDialog() {
        if (plan.none { it.hasPosition }) { info("Make a plan first: waypoints or Zig zag"); return }
        val cur = Patrol.laps(plan)
        val laps = JTextField(if (cur == 1) "0" else cur.toString(), 6)
        val lastWait = plan.lastOrNull { it.command == Mav.CMD_NAV_WAYPOINT && it.hasPosition }?.p1 ?: 0f
        val wait = JTextField(fmt(lastWait), 6)
        val problems = Patrol.bendyRulerProblems { vehicle.paramValue(it) }
        val status = JLabel("<html>" + (if (problems.isEmpty()) "<font color='#2F7D4F'>BendyRuler ready: it steers round what the LiDAR sees.</font>"
            else problems.joinToString("<br>") { "<font color='#A16207'>• $it</font>" }) + "</html>")
        val setup = btn("Set up BendyRuler") {
            if (!requireLink()) return@btn
            val (writes, reboot) = Patrol.setupWrites { vehicle.paramValue(it) }
            if (writes.isEmpty()) { info("BendyRuler is already set up"); return@btn }
            val changes = writes.entries.joinToString("\n") { (k, x) -> "$k ${vehicle.paramValue(k)?.let { fmt(it) } ?: "?"} → ${fmt(x)}" }
            val more = if (reboot) "\n\nOA_TYPE takes effect after a reboot. After the board reconnects, Refresh the parameters\n" +
                "and press Set up BendyRuler again: the OA_BR_ settings only exist after the reboot." else ""
            if (ask("BendyRuler", changes + more)) {
                writes.forEach { (k, x) -> vehicle.setParam(k, x) }
                if (reboot) Timer(2500) { vehicle.reboot() }.apply { isRepeats = false }.start()
            }
        }
        val panel = JPanel(BorderLayout(0, 8)).apply {
            add(form("Laps (0 = forever)" to laps, "Wait each lap (s)" to wait), BorderLayout.NORTH)
            add(JPanel(BorderLayout()).apply {
                border = BorderFactory.createTitledBorder("Obstacles: BendyRuler")
                add(status, BorderLayout.CENTER); add(row(setup), BorderLayout.SOUTH)
            }, BorderLayout.CENTER)
            add(JLabel("<html><small>Loops the plan with a DO_JUMP: the car keeps patrolling in Auto even without this app.<br>Stop it with Hold.</small></html>"), BorderLayout.SOUTH)
        }
        if (JOptionPane.showConfirmDialog(this, panel, "Patrol", JOptionPane.OK_CANCEL_OPTION, JOptionPane.PLAIN_MESSAGE) != JOptionPane.OK_OPTION) return
        val n = (laps.text.toIntOrNull() ?: 0).coerceAtLeast(0)
        val p = Patrol.apply(plan, n, (wait.text.toFloatOrNull() ?: 0f).coerceAtLeast(0f))
        plan.clear(); plan.addAll(p); planChanged()
        planStatus.text = (if (n == 0) "Patrol forever" else "Patrol: $n laps") + ". Write, then Start Auto; Hold stops it."
    }

    // --- fences ---

    private fun setFenceMode(on: Boolean) {
        missionBar.isVisible = !on; fenceBar.isVisible = on
        planMap.fenceEditing = on
        if (on) fenceChanged() else planChanged()
    }

    private fun addZone(kind: FenceZone.Kind) {
        fence.add(FenceZone(kind, radius = if (kind.polygon) 0f else 3f)); fenceZone = fence.size - 1; fenceChanged()
        planStatus.text = if (kind.polygon) "Click the map at each corner of ${kind.label.lowercase()} ${fenceZone + 1} (3 or more)." else "Click the map at the circle's centre."
    }

    private fun fenceClick(la: Double, lo: Double) {
        val z = fence.getOrNull(fenceZone) ?: run { info("Add a zone first: + Stay-in, + Stay-out or + Circle"); return }
        when {
            z.kind.polygon -> z.points.add(doubleArrayOf(la, lo))
            z.points.isEmpty() -> {
                z.points.add(doubleArrayOf(la, lo))
                JOptionPane.showInputDialog(this, "Radius in metres", fmt(z.radius))?.toFloatOrNull()?.takeIf { it > 0 }?.let { z.radius = it }
            }
            else -> { z.points[0][0] = la; z.points[0][1] = lo }
        }
        fenceChanged()
    }

    private fun editZone(zi: Int, point: Int) {
        val z = fence.getOrNull(zi) ?: return
        fenceZone = zi; fenceChanged()
        val opts = ArrayList<String>().apply {
            add("Add corners here"); if (z.kind.polygon && point >= 0) add("Delete corner")
            if (z.isCircle) add("Radius…"); if (z.kind.polygon || z.isCircle) add("Swap stay-in / stay-out")
            add("Delete zone"); add("Cancel")
        }
        val r = JOptionPane.showOptionDialog(this, "${zi + 1}. ${z.kind.label}", "Fence zone", JOptionPane.DEFAULT_OPTION, JOptionPane.PLAIN_MESSAGE, null, opts.toTypedArray(), opts[0])
        when (opts.getOrNull(r)) {
            "Delete corner" -> z.points.removeAt(point)
            "Radius…" -> JOptionPane.showInputDialog(this, "Radius in metres", fmt(z.radius))?.toFloatOrNull()?.takeIf { it > 0 }?.let { z.radius = it }
            "Swap stay-in / stay-out" -> z.kind = when (z.kind) {
                FenceZone.Kind.POLYGON_IN -> FenceZone.Kind.POLYGON_OUT; FenceZone.Kind.POLYGON_OUT -> FenceZone.Kind.POLYGON_IN
                FenceZone.Kind.CIRCLE_IN -> FenceZone.Kind.CIRCLE_OUT; else -> FenceZone.Kind.CIRCLE_IN
            }
            "Delete zone" -> { fence.removeAt(zi); fenceZone = -1 }
        }
        fenceChanged()
    }

    private fun fenceChanged() {
        planMap.fences = fence; flyMap.fences = fence; planMap.selectedZone = fenceZone
        planMap.repaint(); flyMap.repaint()
        if (!fenceMode.isSelected) return
        planList.clear()
        fence.forEachIndexed { i, z ->
            val what = if (z.kind.polygon) "${z.points.size} corners" else if (z.isCircle) "radius ${fmt(z.radius)} m" else ""
            planList.addElement("${i + 1}. ${z.kind.label}, $what" + if (i == fenceZone) "  ◀ adding" else "")
        }
        planList.addElement(if (fence.isEmpty()) "No fence zones" else
            "Uses ${Fence.storageBytes(fence)} of the board's ${Fence.BOARD_STORAGE_BYTES} bytes" + (Fence.problem(fence)?.let { "  ⚠ $it" } ?: ""))
    }

    private fun writeFence() {
        if (!requireLink()) return
        Fence.problem(fence)?.let { info(it); return }
        if (fence.isEmpty() && !ask("Clear the vehicle's fence", "Write an empty fence? This deletes all fence zones on the vehicle.")) return
        if (Fence.storageBytes(fence) > Fence.BOARD_STORAGE_BYTES &&
            !ask("Fence may not fit", "This fence needs ${Fence.storageBytes(fence)} bytes; the ESP32-S3 board stores ${Fence.BOARD_STORAGE_BYTES}. Write anyway?")) return
        vehicle.uploadFence(Fence.toItems(fence))
    }

    private fun enableFence() {
        if (!requireLink()) return
        val en = vehicle.paramValue("FENCE_ENABLE"); val type = vehicle.paramValue("FENCE_TYPE")?.toInt(); val action = vehicle.paramValue("FENCE_ACTION")?.toInt()
        if (en == null || type == null) { info("Read the parameters first (Params → Refresh)"); return }
        val newType = type or 4; val newAction = if (action == null || action == 0) 2 else action
        if (ask("Enable the fence", "FENCE_ENABLE ${en.toInt()} → 1\nFENCE_TYPE $type → $newType\nFENCE_ACTION ${action ?: "?"} → $newAction")) {
            vehicle.setParam("FENCE_TYPE", newType.toFloat()); vehicle.setParam("FENCE_ACTION", newAction.toFloat()); vehicle.setParam("FENCE_ENABLE", 1f)
        }
    }

    // --- files ---

    private fun chooser(ext: String, desc: String) = JFileChooser(prefs.get("dir", System.getProperty("user.home"))).apply {
        fileFilter = FileNameExtensionFilter(desc, ext)
    }

    private fun loadMission() {
        val c = chooser("waypoints", "Mission Planner .waypoints")
        if (c.showOpenDialog(this) != JFileChooser.APPROVE_OPTION) return
        prefs.put("dir", c.selectedFile.parent)
        try {
            val lines = c.selectedFile.readLines().map { it.trim() }.filter { it.isNotEmpty() }
            require(lines.firstOrNull()?.startsWith("QGC WPL") == true) { "not a QGC WPL / Mission Planner .waypoints file" }
            val items = lines.drop(1).mapNotNull { l ->
                val f = l.split(Regex("\\s+"))
                if (f.size < 12 || f[0] == "0") null else MissionItem(command = f[3].toInt(), p1 = f[4].toFloat(), p2 = f[5].toFloat(),
                    p3 = f[6].toFloat(), p4 = f[7].toFloat(), lat = f[8].toDouble(), lon = f[9].toDouble(), alt = f[10].toFloat(),
                    frame = f[2].toInt(), autocontinue = f[11].toInt())
            }
            plan.clear(); plan.addAll(items); planChanged(); planMap.fit()
        } catch (e: Exception) { info("Could not load: ${e.message}") }
    }

    private fun saveMission() {
        val c = chooser("waypoints", "Mission Planner .waypoints")
        c.selectedFile = File("mission.waypoints")
        if (c.showSaveDialog(this) != JFileChooser.APPROVE_OPTION) return
        prefs.put("dir", c.selectedFile.parent)
        val sb = StringBuilder("QGC WPL 110\n")
        val hl = if (vehicle.hasHome) vehicle.homeLat else plan.firstOrNull { it.hasPosition }?.lat ?: 0.0
        val ho = if (vehicle.hasHome) vehicle.homeLon else plan.firstOrNull { it.hasPosition }?.lon ?: 0.0
        sb.append("0\t1\t0\t16\t0\t0\t0\t0\t%.8f\t%.8f\t0\t1\n".format(java.util.Locale.US, hl, ho))
        plan.forEachIndexed { i, it ->
            sb.append("%d\t0\t%d\t%d\t%s\t%s\t%s\t%s\t%.8f\t%.8f\t%s\t%d\n".format(java.util.Locale.US,
                i + 1, it.frame, it.command, fmt(it.p1), fmt(it.p2), fmt(it.p3), fmt(it.p4), it.lat, it.lon, fmt(it.alt), it.autocontinue))
        }
        c.selectedFile.writeText(sb.toString())
    }

    // =========================================================================
    // LiDAR
    // =========================================================================

    private fun lidarTab(): JComponent {
        val group = javax.swing.ButtonGroup()
        val modeRow = row()
        LidarBehaviours.Mode.values().forEach { m ->
            val b = JToggleButton(m.label).apply { addActionListener { setLidarMode(m) } }
            group.add(b); modeButtons[m] = b; modeRow.add(b)
        }
        lidarStart.addActionListener { if (driver.running) stopLidar() else startLidar() }
        modeRow.add(lidarStart)
        lidarHint.apply { isEditable = false; lineWrap = true; wrapStyleWord = true; isOpaque = false; font = Font(Font.SANS_SERIF, Font.PLAIN, 12) }
        lidarStatus.font = Font(Font.SANS_SERIF, Font.BOLD, 15)
        lidarStatus.preferredSize = Dimension(400, 24)       // long behaviour messages, without relayout
        val p = behaviours.p
        fun field(get: () -> Float, set: (Float) -> Unit) = JTextField(fmt(get()), 6).apply {
            document.addDocumentListener(object : DocumentListener {
                fun upd() { text.toFloatOrNull()?.let { set(it); saveSettings() } }
                override fun insertUpdate(e: DocumentEvent?) = upd()
                override fun removeUpdate(e: DocumentEvent?) = upd()
                override fun changedUpdate(e: DocumentEvent?) = upd()
            })
        }
        val skid = JComboBox(arrayOf("RC car", "Skid steer")).apply {
            selectedIndex = if (p.skidSteer) 1 else 0
            addActionListener { p.skidSteer = selectedIndex == 1; saveSettings() }
        }
        val settings = form(
            "Vehicle" to skid,
            "Avoid: speed (stick 0-1)" to field({ p.linear }) { p.linear = it.coerceIn(0f, 1f) },
            "Avoid: steering (stick 0-1)" to field({ p.angular }) { p.angular = it.coerceIn(0f, 1f) },
            "Avoid: response dist (m)" to field({ p.responseDist }) { p.responseDist = it.coerceIn(0.2f, 5f) },
            "Avoid: side sector to (°)" to field({ p.laserAngle }) { p.laserAngle = it.coerceIn(25f, 90f) },
            "Avoid: bins to block" to field({ p.minPoints.toFloat() }) { p.minPoints = it.toInt().coerceIn(1, 10) },
            "Follow: keep distance (m)" to field({ p.followDist }) { p.followDist = it.coerceIn(0.3f, 5f) },
            "Follow: max speed (stick)" to field({ p.maxLinear }) { p.maxLinear = it.coerceIn(0f, 1f) },
            "Follow: max steering (stick)" to field({ p.maxAngular }) { p.maxAngular = it.coerceIn(0f, 1f) },
            "Follow: search angle (°)" to field({ p.trackAngle }) { p.trackAngle = it.coerceIn(5f, 180f) },
            "Warn: distance (m)" to field({ p.warnDist }) { p.warnDist = it.coerceIn(0.1f, 8f) },
            "Warn: angle (°)" to field({ p.warnAngle }) { p.warnAngle = it.coerceIn(5f, 180f) },
            "Ignore closer than (m)" to field({ p.minRange }) { p.minRange = it.coerceIn(0f, 2f) },
        )
        val right = JPanel(BorderLayout(0, 6)).apply {
            border = BorderFactory.createEmptyBorder(8, 8, 8, 8)
            add(JPanel().apply {
                layout = BoxLayout(this, BoxLayout.Y_AXIS)
                add(lidarStatus.apply { alignmentX = 0f }); add(modeRow.apply { alignmentX = 0f }); add(lidarHint.apply { alignmentX = 0f })
            }, BorderLayout.NORTH)
            add(JScrollPane(settings).apply { border = BorderFactory.createTitledBorder("Settings (hoverboard_laser names)") }, BorderLayout.CENTER)
            add(JLabel("<html><small>RC car: on-the-spot turns become backing up with the wheels the other way, only while the rear is clear.<br>" +
                "Use Acro. Keep AVOID_MARGIN below the follow distance. Arrow-key driving pauses the behaviour.</small></html>"), BorderLayout.SOUTH)
            preferredSize = Dimension(420, 0)
        }
        setLidarMode(behaviours.mode)
        return JSplitPane(JSplitPane.HORIZONTAL_SPLIT, lidarPanel, right).apply { resizeWeight = 1.0 }
    }

    private fun setLidarMode(m: LidarBehaviours.Mode) {
        val was = driver.running
        behaviours.mode = m
        modeButtons[m]?.isSelected = true
        lidarHint.text = when (m) {
            LidarBehaviours.Mode.OFF -> "Shows the scan only. Choose a behaviour from hoverboard_laser."
            LidarBehaviours.Mode.AVOID -> "laser_Avoidance: drives forward and steers away from obstacles in the front, left and right sectors."
            LidarBehaviours.Mode.FOLLOW -> "laser_Tracker: follows the nearest object ahead and keeps the set distance from it."
            LidarBehaviours.Mode.WARN -> "laser_Warning: beeps when something ahead is too close. Doesn't drive."
        } + if (m == LidarBehaviours.Mode.AVOID || m == LidarBehaviours.Mode.FOLLOW) " Drives with stick overrides (Manual, Acro or Steering); Stop or closing the app stops the car." else ""
        prefs.put("lidar_mode", m.name)
        if (m == LidarBehaviours.Mode.OFF) stopLidar() else if (was) driver.start()
    }

    private fun startLidar() {
        if (behaviours.mode == LidarBehaviours.Mode.OFF) { info("Choose Avoid, Follow or Warn first"); return }
        if (vehicle.scan == null) { info("No LiDAR scan from the vehicle yet"); return }
        if (driver.drives) {
            if (!requireLink()) return
            if (vehicle.customMode !in listOf(0, 1, 3)) {
                if (!ask("Mode", "Stick overrides only drive in Manual, Acro or Steering. Switch to Manual and start?")) return
                vehicle.setMode(0)
            }
            if (!vehicle.armed) info("The car is disarmed: arm it on the Fly tab to drive")
        }
        driver.start(); lidarStart.text = "Stop"
    }

    private fun stopLidar() { driver.stop(); lidarStart.text = "Start" }

    private fun lidarTick() {
        val now = Platform.clock()
        val cmd = driver.tick(now, keysHeld, power.value / 100f)
        if (driver.running && cmd.alarm && now - lastBeep > 450) { lastBeep = now; java.awt.Toolkit.getDefaultToolkit().beep() }
        if (tabs.selectedIndex != 2) return
        val s = vehicle.scan
        val p = behaviours.p
        lidarPanel.scan = s; lidarPanel.minRange = p.minRange
        lidarPanel.info = s?.let { sc ->
            "${vehicle.scanCount} scans" + (sc.nearest(0f, 180f, p.minRange)?.let { "  ·  nearest %.2f m at %+.0f°".format(it.second, it.first) } ?: "  ·  nothing in range")
        } ?: ""
        lidarPanel.wedges = wedges(s)
        lidarPanel.target = if (behaviours.mode == LidarBehaviours.Mode.FOLLOW && s != null) s.nearest(0f, p.trackAngle, p.minRange) else null
        lidarPanel.repaint()
        lidarStatus.text = if (driver.running) cmd.why.replaceFirstChar { it.uppercase() } +
            (if (driver.drives) "   thr %+.2f  steer %+.2f".format(cmd.throttle, cmd.steer) else "") else "${behaviours.mode.label}: not running"
        lidarStatus.foreground = if (cmd.alarm) Color(0xC2, 0x41, 0x0C) else Color(0x17, 0x20, 0x1C)
    }

    private fun wedges(s: LidarScan?): List<LidarPanel.Wedge> {
        val p = behaviours.p
        fun hot(from: Float, to: Float, r: Float) = s?.points(p.minRange)?.let { (a, d) -> a.indices.count { a[it] > from && a[it] < to && d[it] < r } } ?: 0
        return when (behaviours.mode) {
            LidarBehaviours.Mode.AVOID -> { val r = p.responseDist * 1.5f
                listOf(LidarPanel.Wedge(-20f, 20f, r, hot(-20f, 20f, r) >= p.minPoints),
                    LidarPanel.Wedge(20f, p.laserAngle, r, hot(20f, p.laserAngle, r) >= p.minPoints),
                    LidarPanel.Wedge(-p.laserAngle, -20f, r, hot(-p.laserAngle, -20f, r) >= p.minPoints)) }
            LidarBehaviours.Mode.FOLLOW -> listOf(LidarPanel.Wedge(-p.trackAngle, p.trackAngle, p.followDist, false))
            LidarBehaviours.Mode.WARN -> listOf(LidarPanel.Wedge(-p.warnAngle, p.warnAngle, p.warnDist, hot(-p.warnAngle, p.warnAngle, p.warnDist) > 0))
            LidarBehaviours.Mode.OFF -> emptyList()
        }
    }

    // =========================================================================
    // Params
    // =========================================================================

    private fun paramsTab(): JComponent {
        paramModel = object : AbstractTableModel() {
            override fun getRowCount() = params.size
            override fun getColumnCount() = 3
            override fun getColumnName(c: Int) = arrayOf("Name", "Vehicle", "To write")[c]
            override fun getValueAt(r: Int, c: Int): Any = params[r].let { p ->
                when (c) { 0 -> p.name; 1 -> fmtParam(p.value, p.type); else -> staged[p.name]?.let { fmtParam(it, p.type) } ?: "" }
            }
            override fun isCellEditable(r: Int, c: Int) = c == 2
            override fun setValueAt(v: Any?, r: Int, c: Int) {
                val p = params[r]; val x = v.toString().trim().toFloatOrNull()
                if (x == null || x == p.value) staged.remove(p.name) else staged[p.name] = x
                fireTableRowsUpdated(r, r); paramStatus.text = "${staged.size} to write"
            }
        }
        val table = JTable(paramModel).apply { font = Font(Font.MONOSPACED, Font.PLAIN, 13); rowHeight = 22 }
        paramFilter.document.addDocumentListener(object : DocumentListener {
            override fun insertUpdate(e: DocumentEvent?) = refreshParams()
            override fun removeUpdate(e: DocumentEvent?) = refreshParams()
            override fun changedUpdate(e: DocumentEvent?) = refreshParams()
        })
        val bar = row(JLabel("Search"), paramFilter,
            btn("Refresh") { if (requireLink()) { vehicle.fetchParams(); paramStatus.text = "Reading…" } },
            btn("Write") { if (requireLink() && staged.isNotEmpty()) { staged.forEach { (k, v) -> vehicle.setParam(k, v) }; paramStatus.text = "Writing ${staged.size}…"; staged.clear(); refreshParams() } },
            btn("Undo edits") { staged.clear(); refreshParams() },
            btn("Load .parm…") { loadParams() }, btn("Save .parm…") { saveParams() },
            btn("Reboot") { if (requireLink() && ask("Reboot", "Reboot the autopilot? (It must be disarmed.)")) vehicle.reboot() })
        return JPanel(BorderLayout()).apply {
            add(JPanel(BorderLayout()).apply { add(bar, BorderLayout.NORTH); add(paramStatus.apply { border = BorderFactory.createEmptyBorder(0, 8, 4, 8) }, BorderLayout.SOUTH) }, BorderLayout.NORTH)
            add(JScrollPane(table), BorderLayout.CENTER)
            add(JLabel(" Edit the 'To write' column, then Write.").apply { foreground = Color.GRAY }, BorderLayout.SOUTH)
        }
    }

    private fun fmtParam(v: Float, type: Int): String =
        if (type in 1..8 || (v == v.toLong().toFloat() && abs(v) < 1e7)) v.toLong().toString()
        else "%.6g".format(java.util.Locale.US, v).let { if (it.contains('.') && !it.contains('e')) it.trimEnd('0').trimEnd('.') else it }

    private fun refreshParams() {
        val f = paramFilter.text.trim().uppercase()
        params.clear()
        params.addAll(synchronized(vehicle) { vehicle.params.values.toList() }.filter { f.isEmpty() || it.name.contains(f) })
        paramModel.fireTableDataChanged()
    }

    private fun loadParams() {
        val c = chooser("parm", "ArduPilot .parm"); if (c.showOpenDialog(this) != JFileChooser.APPROVE_OPTION) return
        prefs.put("dir", c.selectedFile.parent)
        var n = 0
        for (raw in c.selectedFile.readLines()) {
            val line = raw.substringBefore('#').trim(); if (line.isEmpty()) continue
            val parts = line.split(Regex("[,\\s]+")); if (parts.size < 2) continue
            val v = parts[1].toFloatOrNull() ?: continue
            val cur = vehicle.paramValue(parts[0])
            if (cur == null && vehicle.params.isNotEmpty()) continue
            if (cur != null && abs(cur - v) < 1e-6f) continue
            staged[parts[0]] = v; n++
        }
        refreshParams(); paramStatus.text = "Loaded: $n values differ from the vehicle. Press Write."
    }

    private fun saveParams() {
        val c = chooser("parm", "ArduPilot .parm"); c.selectedFile = File("rover.parm")
        if (c.showSaveDialog(this) != JFileChooser.APPROVE_OPTION) return
        prefs.put("dir", c.selectedFile.parent)
        c.selectedFile.writeText("# ArduRover parameters saved by ESP32 Rover GCS\n" +
            synchronized(vehicle) { vehicle.params.values.toList() }.joinToString("") { "${it.name},${fmtParam(it.value, it.type)}\n" })
    }

    // =========================================================================
    // events and refresh
    // =========================================================================

    private fun onEvent(e: VehicleEvent) {
        when (e) {
            is VehicleEvent.Text -> {
                lastMsg.text = e.text
                lastMsg.foreground = if (e.severity <= 3) Color(0xC2, 0x41, 0x0C) else if (e.severity == 4) Color(0xA1, 0x62, 0x07) else Color.DARK_GRAY
            }
            is VehicleEvent.Link -> lastMsg.text = e.text
            is VehicleEvent.ParamsDone -> {
                paramStatus.text = if (e.received >= e.total && e.total > 0) "${e.total} parameters" else "Got ${e.received} of ${e.total}: Refresh again"
                staged.entries.removeIf { (k, v) -> vehicle.paramValue(k)?.let { abs(it - v) < 1e-6f } == true }
                refreshParams()
            }
            is VehicleEvent.ParamWritten -> {
                paramStatus.text = "${e.name} = ${fmt(e.value)}" + if (!e.ok) " (refused)" else ""
                if (vehicle.paramWritesPending == 0) refreshParams()
            }
            is VehicleEvent.MissionUploaded -> { planStatus.text = e.text }
            is VehicleEvent.MissionDownloaded -> {
                planStatus.text = e.text
                val items = e.items ?: return
                if (e.type == Vehicle.MISSION_TYPE_FENCE) { fence.clear(); fence.addAll(Fence.fromItems(items)); fenceZone = -1; fenceChanged() }
                else { plan.clear(); plan.addAll(items); planChanged() }
                planMap.fit()
            }
            is VehicleEvent.CommandAck -> if (e.result != 0) lastMsg.text = "Command ${e.command}: ${Vehicle.cmdResult(e.result)}"
            is VehicleEvent.AccelCalPos, is VehicleEvent.MagCalReport -> {}
        }
    }

    private fun refresh() {
        val v = vehicle; val conn = v.connected
        linkLabel.text = (if (conn) "● " else "○ ") + v.linkText
        linkLabel.foreground = if (conn) Color(0x2F, 0x7D, 0x4F) else Color.GRAY
        modeLabel.text = if (conn) v.modeName + if (v.armed) " · ARMED" else "" else "mode —"
        modeLabel.foreground = if (conn && v.armed) Color(0xC2, 0x41, 0x0C) else Color.DARK_GRAY
        val fix = Vehicle.GPS_FIX.getOrElse(v.gpsFix) { "fix ${v.gpsFix}" }
        gpsLabel.text = if (conn) "$fix · ${v.gpsSats} sats" else "GPS —"
        battLabel.text = if (conn && v.voltage > 0.5f) "%.2f V".format(v.voltage) + if (v.batteryPct >= 0) " · ${v.batteryPct}%" else "" else "battery —"
        armBtn.text = if (v.armed) "Disarm" else "Arm"
        val lap = lapCounter.update(v.missionSeq, conn && v.customMode == 10)
        val patrol = Patrol.laps(plan)
        hud.apply {
            roll = Math.toDegrees(v.roll.toDouble()).toFloat(); pitch = Math.toDegrees(v.pitch.toDouble()).toFloat()
            heading = v.heading.toFloat(); speed = v.groundSpeed; throttle = v.throttle
            mode = v.modeName; armed = v.armed; connected = conn
            gps = if (conn) "GPS $fix  ${v.gpsSats} sats  hdop %.2f".format(v.gpsHdop) else "GPS —"
            battery = if (conn && v.voltage > 0.5f) "%.2f V  %.1f A".format(v.voltage, v.current) else ""
            wp = if (conn && v.customMode == 10) (if (patrol == 1) "" else "Lap $lap" + (if (patrol > 0) "/$patrol" else "") + "  ") +
                "WP ${v.missionSeq}  ${v.wpDist} m  xtrack %.1f m".format(v.xtrack) else ""
            repaint()
        }
        val pos = if (v.hasPosition) Triple(v.lat, v.lon, Math.toDegrees(v.yaw.toDouble()).toFloat()) else null
        val home = if (v.hasHome) Pair(v.homeLat, v.homeLon) else null
        for (m in listOf(flyMap, planMap)) {
            if (pos != null && m.vehicle == null && (m === flyMap || plan.isEmpty())) m.centerOn(pos.first, pos.second, 18.0)
            m.vehicle = pos; m.home = home; m.scan = v.scan
            m.currentWp = if (v.customMode == 10) v.missionSeq - 1 else -1
        }
        if (pos != null && conn) flyMap.addTrail(pos.first, pos.second)
        if (v.customMode != 15) flyMap.guidedTarget = null
        when (tabs.selectedIndex) { 0 -> flyMap.repaint(); 1 -> planMap.repaint() }
        if (v.missionBusy) planStatus.text = v.missionProgress
        if (v.paramsFetching) paramStatus.text = "Reading ${v.params.size} / ${v.paramCount}…"
        if (tabs.selectedIndex == TAB_INDOOR) refreshIndoor()
        if (tabs.selectedIndex == TAB_LOG) {
            val msgs = synchronized(v.messages) { v.messages.toList() }
            if (msgs.size != logShown) {
                logShown = msgs.size
                logArea.text = msgs.asReversed().joinToString("\n") { "${Vehicle.SEVERITY.getOrElse(it.severity) { "?" }.padEnd(6)} ${it.text}" }
                logArea.caretPosition = 0
            }
        }
    }

    // =========================================================================
    // indoor: Nav2-style navigation in a saved map (core: nav/IndoorNav)
    // =========================================================================

    private fun indoorTab(): JComponent {
        indoor.settings.speed = prefs.getDouble("indoor_speed", 0.4)
        indoor.settings.robotRadius = prefs.getDouble("indoor_radius", 0.2)
        indoor.settings.inflation = prefs.getDouble("indoor_inflation", 0.55)
        indoorPanel.onTap = { x, y -> indoorClick(x, y) }
        indoorPanel.onPose = { x, y, yaw -> indoor.setPose(x, y, yaw) }
        indoorStatus.font = Font(Font.SANS_SERIF, Font.BOLD, 14)
        indoorStatus.preferredSize = Dimension(420, 22)
        indoorArm.addActionListener { if (requireLink()) vehicle.arm(!vehicle.armed) }
        indoorScanBtn.addActionListener { vehicle.setParam("PRX1_SCAN_FULL", 1f) }
        val stop = btn("STOP") { indoor.stop() }.apply { background = Color(0xC2, 0x41, 0x0C); foreground = Color.WHITE; font = font.deriveFont(Font.BOLD) }
        val wpList = JList(indoorWps).apply { visibleRowCount = 6; selectionMode = ListSelectionModel.SINGLE_SELECTION }
        fun selected() = indoor.waypoints.getOrNull(wpList.selectedIndex)
        fun field(v: Double) = JTextField("%.2f".format(java.util.Locale.ROOT, v), 5)
        val speed = field(indoor.settings.speed); val radius = field(indoor.settings.robotRadius); val infl = field(indoor.settings.inflation)
        val apply = btn("Apply") {
            speed.text.toDoubleOrNull()?.let { indoor.settings.speed = it.coerceIn(0.1, 1.5) }
            radius.text.toDoubleOrNull()?.let { indoor.settings.robotRadius = it.coerceIn(0.05, 1.0) }
            infl.text.toDoubleOrNull()?.let { indoor.settings.inflation = it.coerceIn(0.1, 2.0) }
            prefs.putDouble("indoor_speed", indoor.settings.speed); prefs.putDouble("indoor_radius", indoor.settings.robotRadius)
            prefs.putDouble("indoor_inflation", indoor.settings.inflation)
            info("Saved. New values apply when a map is loaded or saved.")
        }
        val right = JPanel().apply {
            layout = BoxLayout(this, BoxLayout.Y_AXIS)
            border = BorderFactory.createEmptyBorder(8, 8, 8, 8)
            fun add2(c: JComponent) { c.alignmentX = 0f; add(c) }
            add2(indoorStatus); add2(indoorInfo)
            add2(row(stop, indoorArm, btn("Patrol") { if (requireLink()) indoor.startPatrol() }))
            add2(row(indoorAddMode, btn("Find me") { indoor.findMe() }))
            add2(JLabel("<html><small>Click the map to drive there (Guided). Right-drag on the map to set where the car is " +
                "and which way it faces.</small></html>"))
            add2(JLabel(" "))
            add2(JLabel("Map").apply { font = font.deriveFont(Font.BOLD) })
            add2(row(btn("Map new") {
                if (ask("Map a new room", "Put the car where it will always start, then drive it slowly round the room. " +
                        "Save when the walls are complete.")) { indoor.startMapping(); indoorPanel.refit() }
            }, btn("Save map") { saveIndoorMap() }, btn("Load map") { loadIndoorMap() }))
            add2(indoorScanBtn)
            add2(JLabel(" "))
            add2(JLabel("Waypoints (the patrol goes in this order)").apply { font = font.deriveFont(Font.BOLD) })
            add2(JScrollPane(wpList).apply { preferredSize = Dimension(380, 120); maximumSize = Dimension(600, 160) })
            add2(row(btn("Go there") { selected()?.let { if (requireLink()) indoor.goToWaypoint(it.name) } },
                btn("Delete") { selected()?.let { indoor.removeWaypoint(it.name) } },
                btn("Delete all") { if (ask("Waypoints", "Delete all waypoints?")) indoor.clearWaypoints() }))
            add2(JLabel(" "))
            add2(JLabel("Settings").apply { font = font.deriveFont(Font.BOLD) })
            add2(form("Speed (m/s)" to speed, "Car radius (m)" to radius, "Keep away from walls (m)" to infl).apply { maximumSize = Dimension(600, 110) })
            add2(row(apply))
            add(Box.createVerticalGlue())
            preferredSize = Dimension(420, 0)
        }
        return JSplitPane(JSplitPane.HORIZONTAL_SPLIT, indoorPanel, JScrollPane(right)).apply { resizeWeight = 1.0 }
    }

    private fun indoorClick(x: Double, y: Double) {
        if (indoor.grid == null) { info("Load or make a map first"); return }
        if (indoorAddMode.isSelected) { indoor.addWaypoint(x, y); return }
        if (!requireLink()) return
        if (!vehicle.armed) { info("Arm the car first"); return }
        if (ask("Go here?", "Drive to %.1f m N, %.1f m E in Guided mode.".format(x, y))) indoor.goTo(x, y)
    }

    private fun saveIndoorMap() {
        if (indoor.grid == null) { info("Nothing to save: Map new first"); return }
        val n = JOptionPane.showInputDialog(this, "Map name", indoor.mapName ?: "room") ?: return
        val clean = n.trim().replace(Regex("[^A-Za-z0-9_-]"), "_")
        if (clean.isNotEmpty()) indoor.saveMap(clean)
    }

    private fun loadIndoorMap() {
        val names = indoor.maps()
        if (names.isEmpty()) { info("No saved maps yet (they are kept in ${indoor.dir})"); return }
        val pick = JOptionPane.showInputDialog(this, "Map (kept in ${indoor.dir})", "Load map", JOptionPane.PLAIN_MESSAGE,
            null, names.toTypedArray(), indoor.mapName ?: names[0]) as String? ?: return
        indoor.loadMap(pick); indoorPanel.refit()
    }

    private fun refreshIndoor() {
        indoorPanel.repaint()
        indoorStatus.text = indoor.status
        val loc = indoor.localizer
        val scanRes = vehicle.scan?.let { if (it.cm.size >= 360) "1°" else "${it.incrementDeg.toInt()}°" } ?: "none"
        indoorInfo.text = listOfNotNull(
            indoor.mapName?.let { "Map: $it" } ?: if (indoor.mode == IndoorNav.Mode.MAPPING) "Mapping" else "No map",
            loc?.let { if (it.initialised) "pose ±%.2f m, match %.0f%%".format(it.spreadXY, it.score * 100) else "pose not set" },
            "scan $scanRes",
            if (vehicle.paramValue("PRX1_SLAM")?.toInt() == 0) "PRX1_SLAM off" else null,
        ).joinToString("  ·  ")
        indoorArm.text = if (vehicle.armed) "Disarm" else "Arm"
        indoorScanBtn.isVisible = vehicle.paramValue("PRX1_SCAN_FULL")?.toInt() == 0
        val names = indoor.waypoints.map { "%s   (%.1f N, %.1f E)".format(it.name, it.x, it.y) }
        if (names != indoorWpShown) { indoorWpShown = names; indoorWps.clear(); names.forEach { indoorWps.addElement(it) } }
    }

    // =========================================================================
    // settings
    // =========================================================================

    private fun loadSettings() {
        val src = runCatching { MapPanel.Source.valueOf(prefs.get("map", "STREET")) }.getOrDefault(MapPanel.Source.STREET)
        flyMap.source = src; planMap.source = src
        fun pt(k: String) = prefs.get(k, null)?.split(',')?.mapNotNull { it.toDoubleOrNull() }?.takeIf { it.size == 2 }?.let { doubleArrayOf(it[0], it[1]) }
        zigzag.a = pt("zz_a"); zigzag.b = pt("zz_b")
        zigzag.sideDistM = prefs.getFloat("zz_side", 2f); zigzag.lines = prefs.getInt("zz_lines", 5)
        zigzag.right = prefs.getBoolean("zz_right", true); zigzag.waitS = prefs.getFloat("zz_wait", 0f); zigzag.runOutM = prefs.getFloat("zz_runout", -1f)
        planMap.zigA = zigzag.a; planMap.zigB = zigzag.b; flyMap.zigA = zigzag.a; flyMap.zigB = zigzag.b
        val p = behaviours.p
        fun f(k: String, d: Float) = prefs.getFloat("lidar_$k", d)
        p.linear = f("linear", p.linear); p.angular = f("angular", p.angular); p.responseDist = f("responseDist", p.responseDist)
        p.laserAngle = f("laserAngle", p.laserAngle); p.minPoints = f("minPoints", p.minPoints.toFloat()).toInt()
        p.followDist = f("followDist", p.followDist); p.maxLinear = f("maxLinear", p.maxLinear); p.maxAngular = f("maxAngular", p.maxAngular)
        p.trackAngle = f("trackAngle", p.trackAngle); p.warnDist = f("warnDist", p.warnDist); p.warnAngle = f("warnAngle", p.warnAngle)
        p.minRange = f("minRange", p.minRange); p.skidSteer = prefs.getBoolean("lidar_skidSteer", false)
        behaviours.mode = runCatching { LidarBehaviours.Mode.valueOf(prefs.get("lidar_mode", "OFF")) }.getOrDefault(LidarBehaviours.Mode.OFF)
    }

    private fun saveSettings() {
        val p = behaviours.p
        mapOf("linear" to p.linear, "angular" to p.angular, "responseDist" to p.responseDist, "laserAngle" to p.laserAngle,
            "minPoints" to p.minPoints.toFloat(), "followDist" to p.followDist, "maxLinear" to p.maxLinear, "maxAngular" to p.maxAngular,
            "trackAngle" to p.trackAngle, "warnDist" to p.warnDist, "warnAngle" to p.warnAngle, "minRange" to p.minRange)
            .forEach { (k, x) -> prefs.putFloat("lidar_$k", x) }
        prefs.putBoolean("lidar_skidSteer", p.skidSteer)
    }

    companion object {
        const val TAB_INDOOR = 4
        const val TAB_LOG = 5
    }
}

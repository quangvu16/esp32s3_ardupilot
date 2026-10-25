import Foundation

/// The Indoor screen's view of the map: north up, east right, scale px a metre.
struct Viewport {
    var cx = 0.0, cy = 0.0, scale = 60.0, width = 1.0, height = 1.0
    var follow = true
    func sx(_ y: Double) -> Double { width / 2 + (y - cy) * scale }
    func sy(_ x: Double) -> Double { height / 2 - (x - cx) * scale }
    func mapX(_ sy: Double) -> Double { cx - (sy - height / 2) / scale }
    func mapY(_ sx: Double) -> Double { cy + (sx - width / 2) / scale }
    mutating func zoom(_ f: Double, _ fx: Double, _ fy: Double) {
        let mx = mapX(fy), my = mapY(fx)
        scale = min(max(scale * f, 8), 400)
        cx = mx + (fy - height / 2) / scale
        cy = my - (fx - width / 2) / scale
    }
    mutating func fit(_ g: OccGrid) {
        var r0 = g.height, r1 = -1, c0 = g.width, c1 = -1
        for r in 0..<g.height { for c in 0..<g.width where g.known(g.index(r, c)) {
            r0 = min(r0, r); r1 = max(r1, r); c0 = min(c0, c); c1 = max(c1, c)
        } }
        if r1 < 0 { return }
        cx = (g.cellX(r0) + g.cellX(r1)) / 2; cy = (g.cellY(c0) + g.cellY(c1)) / 2
        let spanX = Double(r1 - r0 + 4) * g.res, spanY = Double(c1 - c0 + 4) * g.res
        scale = min(max(min(height / spanX, width / spanY), 8), 400)
    }

    /// The map as RGBA bytes (top row north) with the costmap's margin tinted.
    static func rgba(_ g: OccGrid, _ cost: [UInt8]?) -> [UInt8] {
        var px = [UInt8](repeating: 255, count: g.width * g.height * 4)
        for r in 0..<g.height {
            for c in 0..<g.width {
                let i = g.index(r, c)
                var rgb: (UInt8, UInt8, UInt8) = g.occupied(i) ? (31, 42, 36) : g.free(i) ? (255, 255, 255) : (217, 222, 217)
                if let cost = cost, !g.occupied(i) {
                    let k = Int(cost[i])
                    if k == Costmap.lethal { rgb = (194, 65, 12) }
                    else if k == Costmap.inscribed { rgb = (244, 185, 160) }
                    else if k >= 1 && k <= 252 && g.free(i) {
                        let f = Double(k) / 252 * 0.6
                        rgb = (UInt8(255 * (1 - f) + 143 * f), UInt8(255 * (1 - f) + 179 * f), UInt8(255 * (1 - f) + 238 * f))
                    }
                }
                let o = ((g.height - 1 - r) * g.width + c) * 4
                px[o] = rgb.0; px[o + 1] = rgb.1; px[o + 2] = rgb.2
            }
        }
        return px
    }
}

/**
 Indoor navigation for the Indoor tab (port of nav/IndoorNav.kt): map a room, save
 it, find the car in it (AMCL) and drive to a tapped goal or round a patrol. The
 maths runs on its own queue; the main thread feeds it the Vehicle's data 10 times
 a second (feed) and reads `snap` for drawing.
 */
final class IndoorNav: NavDriver {
    enum Mode: String { case off, mapping, navigation }

    struct Settings {
        var robotRadius = 0.20, inflation = 0.55, speed = 0.4
        var rotationPeriodS: Double? = nil         // nil: from PRX1_TYPE (LD14 1/6 s, LD06 1/10 s)
        var mapSizeM = 40.0, mapRes = 0.10
    }

    /// What the screen draws, copied out under a lock.
    struct Snapshot {
        var mode = Mode.off
        var status = "Off"
        var mapName: String?
        var grid: OccGrid?
        var gridVersion = 0
        var cost: [UInt8]?
        var costVersion = 0
        var pose: Pose2?
        var scanX = [Double](), scanY = [Double]()
        var path = [(Double, Double)]()
        var waypoints = [Waypoint]()
        var goal: Waypoint?
        var active = false
        var particlesX = [Double](), particlesY = [Double]()
        var locInitialised = false
        var locSpread = 0.0, locScore = 0.0
        var speedCmd = 0.0
    }

    let dir: URL
    var settings = Settings()
    private let vehicle: Vehicle
    private let q = DispatchQueue(label: "indoor-nav")
    private let lock = NSLock()
    private var _snap = Snapshot()
    var snap: Snapshot { lock.lock(); defer { lock.unlock() }; return _snap }

    // --- on q only
    private var mode = Mode.off
    private var status = "Off"
    private var mapName: String?
    private var grid: OccGrid?
    private var gridVersion = 0
    private var waypoints = [Waypoint]()
    private var patrol = [String]()
    private var mapper: Mapper?
    private var localizer: Localizer?
    private var costmap: Costmap?
    private var navigator: Navigator?
    private var pose: Pose2?
    private var scanX = [Double](), scanY = [Double]()
    private var lastGridMs: Int64 = 0
    private var lastCostVersion = -1

    // --- the car, copied from the main thread by feed()
    private struct CarState {
        var armed = false, customMode = -1, ekfFlags = 0
        var localTime: Int64 = 0, vNow = 0.0
        var turnRadius: Float?, sensorX = 0.0, sensorY = 0.0, prxType = 19
    }
    private var car = CarState()
    private var lastScanCount = -1
    private var lastScanMs: Int64 = 0
    private var latestScan: LidarScan?

    init(vehicle: Vehicle) {
        self.vehicle = vehicle
        let base = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask).first
            ?? FileManager.default.temporaryDirectory
        dir = base.appendingPathComponent("Esp32RoverGcs/maps", isDirectory: true)
    }

    // MARK: - NavDriver (called on q; the Vehicle lives on the main thread)
    func sendVelocity(_ v: Double, _ yawRate: Double) { DispatchQueue.main.async { self.vehicle.sendVelocity(v, yawRate) } }
    func setGuided() { DispatchQueue.main.async { self.vehicle.setMode(IndoorNav.modeGuided) } }
    func setHold() { DispatchQueue.main.async { self.vehicle.setMode(IndoorNav.modeHold) } }
    var inGuided: Bool { car.customMode == IndoorNav.modeGuided }
    var armed: Bool { car.armed }
    static let modeHold = 4, modeGuided = 15

    // MARK: - main thread

    /// Call 10 times a second on the main thread.
    func feed() {
        let v = vehicle
        var c = CarState()
        c.armed = v.armed; c.customMode = v.customMode; c.ekfFlags = v.ekfFlags
        c.localTime = v.localTime
        c.vNow = v.localVx * cos(Double(v.yaw)) + v.localVy * sin(Double(v.yaw))
        c.turnRadius = v.paramValue("TURN_RADIUS")
        c.sensorX = Double(v.paramValue("PRX1_SLAM_X") ?? 0); c.sensorY = Double(v.paramValue("PRX1_SLAM_Y") ?? 0)
        c.prxType = Int(v.paramValue("PRX1_TYPE") ?? 19)
        var newScan: LidarScan? = nil
        if let s = v.scan, v.scanCount != lastScanCount { lastScanCount = v.scanCount; newScan = s }
        let latest = v.scan
        q.async {
            self.car = c
            self.latestScan = latest
            let now = clockMs()
            if let s = newScan { self.lastScanMs = s.timeMs; self.process(s, now) }
            self.tick(now)
            self.publish()
        }
    }

    func startMapping() { run {
        self.stopNavigation("Mapping")
        let here = self.odomNow() ?? Pose2()
        self.mapper = Mapper(sizeM: self.settings.mapSizeM, res: self.settings.mapRes, centre: here)
        self.mode = .mapping; self.mapName = nil; self.waypoints = []; self.patrol = []
        self.localizer = nil; self.costmap = nil; self.navigator = nil
        self.grid = self.mapper!.grid.copy(); self.gridVersion += 1
        DispatchQueue.main.async { self.vehicle.fastPose = true }
        self.status = "Mapping: drive slowly round the room, then Save"
    } }

    func saveMap(_ name: String) { run {
        guard let g = self.mode == .mapping ? self.mapper?.grid.copy() : self.grid else { self.status = "No map to save"; return }
        do { try MapFiles.save(self.dir, name, g, self.waypoints, self.patrol) } catch { self.status = "Couldn't save: \(error.localizedDescription)"; return }
        self.mapName = name
        if self.mode == .mapping {
            self.useMap(g)
            let o = self.odomNow() ?? Pose2()
            self.localizer?.setPose(o, odom: o, sigmaXY: 0.05, sigmaYaw: 0.03)
        }
        self.status = "Saved \"\(name)\""
    } }

    func loadMap(_ name: String) { run {
        self.stopNavigation("Loading")
        do {
            let saved = try MapFiles.load(self.dir, name)
            self.mapName = name; self.waypoints = saved.waypoints; self.patrol = saved.patrol
            self.useMap(saved.grid)
            if let o = self.odomNow() { self.localizer?.setPose(o, odom: o) }
            self.status = "Loaded \"\(name)\": check the car's position, or Set pose / Find me"
        } catch { self.status = "Couldn't load: \(error.localizedDescription)" }
    } }

    func deleteMap(_ name: String) { run { MapFiles.delete(self.dir, name); if self.mapName == name { self.mapName = nil } } }
    func maps() -> [String] { MapFiles.list(dir) }

    func setPose(_ x: Double, _ y: Double, _ yaw: Double) { run {
        guard let o = self.odomNow() else { self.status = "No position from the car"; return }
        guard let loc = self.localizer else { self.status = "Load or make a map first"; return }
        loc.setPose(Pose2(x: x, y: y, yaw: yaw), odom: o)
        self.status = "Pose set"
    } }

    func findMe() { run {
        guard let loc = self.localizer else { self.status = "Load or make a map first"; return }
        guard let s = self.latestScan, let os = self.toOdom(s) else { self.status = "No scan yet"; return }
        self.status = loc.findGlobally(os) ? "Searching… drive a little to confirm" : "Couldn't place the car"
    } }

    func goTo(_ x: Double, _ y: Double) { run { self.startGoals([Waypoint(name: "goal", x: x, y: y)], loop: false) } }
    func goToWaypoint(_ name: String) { run {
        if let w = self.waypoints.first(where: { $0.name == name }) { self.startGoals([w], loop: false) }
    } }
    func startPatrol() { run {
        var list = self.patrol.compactMap { n in self.waypoints.first { $0.name == n } }
        if list.isEmpty { list = self.waypoints }
        if list.count < 2 { self.status = "A patrol needs 2 or more waypoints"; return }
        self.startGoals(list, loop: true)
    } }
    func addWaypoint(_ x: Double, _ y: Double) { run {
        var k = 1
        while self.waypoints.contains(where: { $0.name == "P\(k)" }) { k += 1 }
        self.waypoints.append(Waypoint(name: "P\(k)", x: x, y: y)); self.patrol.append("P\(k)")
        self.saveWaypoints()
    } }
    func removeWaypoint(_ name: String) { run {
        self.waypoints.removeAll { $0.name == name }; self.patrol.removeAll { $0 == name }; self.saveWaypoints()
    } }
    func clearWaypoints() { run { self.waypoints = []; self.patrol = []; self.saveWaypoints() } }
    func stop() { run { self.stopNavigation("Stopped") } }

    // MARK: - internals (q)

    private func run(_ block: @escaping () -> Void) { q.async { block(); self.publish() } }

    private func useMap(_ g: OccGrid) {
        mapper = nil
        grid = g; gridVersion += 1
        var cp = Costmap.Params()
        cp.inscribedRadius = settings.robotRadius; cp.inflationRadius = max(settings.inflation, settings.robotRadius + 0.1)
        let cm = Costmap(map: g, params: cp)
        costmap = cm
        localizer = Localizer(map: g)
        var rp = Rpp.Params()
        rp.desiredVel = settings.speed
        if let tr = car.turnRadius, tr > 0.1 { rp.minTurnRadius = Double(tr); rp.regMinRadius = Double(tr) }
        let nav = Navigator(planner: Planner(cm), rpp: Rpp(cm, rp), drive: self)
        nav.onClearObstacles = { cm.clearObstacles() }
        navigator = nav
        mode = .navigation
        DispatchQueue.main.async { self.vehicle.fastPose = true }
    }

    private var pending: (list: [Waypoint], loop: Bool, until: Int64)?

    private func startGoals(_ list: [Waypoint], loop: Bool) {
        guard let nav = navigator else { status = "Load or make a map first"; return }
        guard car.armed else { status = "Arm the car first"; return }
        guard localizer?.confident == true else {
            // start as soon as AMCL is sure (it usually is within a few scans)
            pending = (list, loop, clockMs() + 10_000)
            status = "Waiting to be sure where the car is…"
            return
        }
        pending = nil
        nav.start(list, loop: loop, nowMs: clockMs())
    }

    private func stopNavigation(_ why: String) { pending = nil; if let n = navigator, n.active { n.stop(why) } }

    private func saveWaypoints() {
        guard let n = mapName else { return }
        try? MapFiles.saveWaypoints(dir, n, waypoints, patrol)
    }

    private func odomNow() -> Pose2? {
        let t = vehicle.poses.latestMs()
        return t < 0 ? nil : vehicle.poses.at(t)
    }

    /// The EKF has its attitude and a horizontal position (from the LiDAR SLAM).
    private var poseReady: Bool { car.ekfFlags & 0x01 != 0 && car.ekfFlags & (0x08 | 0x10) != 0 && car.ekfFlags & 0x400 == 0 }

    private func toOdom(_ s: LidarScan) -> OdomScan? {
        guard s.vehicleTimeUs > 0, poseReady else { return nil }
        var ang = [Double](), rng = [Double]()
        for (i, d) in s.cm.enumerated() {
            if d == 0xffff || d < s.minCm { continue }
            let deg = (Double(s.offsetDeg) + (Double(i) + 0.5) * Double(s.incrementDeg)).truncatingRemainder(dividingBy: 360)
            ang.append(deg * .pi / 180)
            rng.append(d > s.maxCm ? .infinity : Double(d) / 100)
        }
        let period = settings.rotationPeriodS ?? (car.prxType == 16 ? 0.1 : 1.0 / 6)
        let frame = ScanFrame(endMs: s.vehicleTimeUs / 1000, angles: ang, ranges: rng, maxRange: Double(s.maxCm) / 100)
        return OdomScan.build(frame, vehicle.poses, periodS: period, sensorX: car.sensorX, sensorY: car.sensorY)
    }

    private func process(_ s: LidarScan, _ now: Int64) {
        guard let os = toOdom(s) else {
            if mode != .off && !poseReady { status = "Waiting for the car's position (EKF on the LiDAR SLAM)…" }
            return
        }
        switch mode {
        case .mapping:
            guard let m = mapper else { return }
            m.add(os)
            scanX = os.px; scanY = os.py
            if now - lastGridMs > 1000 { lastGridMs = now; grid = m.grid.copy(); gridVersion += 1 }
            status = "Mapping: \(m.scans) scans. Drive slowly round the room, then Save"
        case .navigation:
            guard let loc = localizer else { return }
            loc.update(os)
            if !loc.initialised { return }
            let inMap = os.transformed(loc.offset)
            costmap?.addScan(inMap, nowMs: now)
            scanX = inMap.px; scanY = inMap.py
            if navigator?.active != true && pending == nil {
                status = loc.confident ? "Ready: tap a goal, or run the patrol"
                    : String(format: "Localising… (spread %.2f m, match %.0f%%)", loc.spreadXY, loc.score * 100)
            }
        case .off: break
        }
    }

    private func tick(_ now: Int64) {
        if let p = pending {
            if localizer?.confident == true { startGoals(p.list, loop: p.loop) }
            else if now > p.until { pending = nil; status = "Not sure where the car is: Set pose or Find me first" }
        }
        if mode == .navigation, let nav = navigator, let loc = localizer {
            if let o = odomNow(), loc.initialised { pose = loc.offset + o }
            let poseAge = car.localTime == 0 ? Int64.max : now - car.localTime
            let scanAge = lastScanMs == 0 ? Int64.max : now - lastScanMs
            nav.tick(nowMs: now, pose: loc.initialised ? pose : nil, confident: loc.confident, poseAgeMs: poseAge, scanAgeMs: scanAge, vNow: car.vNow)
            if nav.active || nav.state == .failed || nav.state == .done { status = nav.status }
        } else if mode == .mapping {
            pose = odomNow()
        }
    }

    private func publish() {
        var s = Snapshot()
        s.mode = mode; s.status = status; s.mapName = mapName
        s.grid = grid; s.gridVersion = gridVersion
        let ver = costmap?.version ?? -1
        let costChanged = ver != lastCostVersion
        let cost = costChanged ? costmap?.master : nil
        lastCostVersion = ver
        s.costVersion = ver
        s.pose = pose; s.scanX = scanX; s.scanY = scanY
        s.waypoints = waypoints
        if let n = navigator { s.path = n.path; s.goal = n.active ? n.goal : nil; s.active = n.active; s.speedCmd = n.lastCmd.v }
        if let l = localizer {
            s.locInitialised = l.initialised; s.locSpread = l.spreadXY; s.locScore = l.score
            if !l.confident { let (x, y) = l.particles(); s.particlesX = x; s.particlesY = y }
        }
        lock.lock()
        s.cost = costChanged ? cost : _snap.cost
        _snap = s
        lock.unlock()
    }
}

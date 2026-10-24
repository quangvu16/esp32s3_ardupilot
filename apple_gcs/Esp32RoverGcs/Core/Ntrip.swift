import Foundation
import Network

/// NTRIP 1.0 client: pulls RTCM3 from a caster and hands each complete RTCM3 frame
/// to onFrame (for GPS_RTCM_DATA), like Mission Planner's RTK inject. Sends the
/// rover's position as GGA every 10 s for casters that need it (VRS).
/// On an iPhone it goes out over mobile data when overCellular is set: the board's
/// WiFi has no internet. A port of core/Ntrip.kt.
final class NtripClient {
    let host: String
    let port: UInt16
    let mount: String
    private let user: String
    private let pass: String
    private let overCellular: Bool
    private let position: () -> Coord?
    private let onFrame: ([UInt8]) -> Void

    private(set) var status = "Stopped"
    private(set) var bytes: Int64 = 0
    private(set) var frames = 0
    /// RTCM message type -> count, e.g. 1005, 1074, 1084…
    private(set) var types = [Int: Int]()

    private var running = false
    private var conn: NWConnection?
    private var header = [UInt8]()
    private var gotHeader = false
    private var ggaTimer: Timer?
    private var retry: DispatchWorkItem?
    private lazy var splitter = Rtcm3Splitter { [weak self] frame, type in
        guard let self = self else { return }
        self.frames += 1
        self.types[type, default: 0] += 1
        self.onFrame(frame)
    }

    init(host: String, port: UInt16, mount: String, user: String, pass: String, overCellular: Bool,
         position: @escaping () -> Coord?, onFrame: @escaping ([UInt8]) -> Void) {
        self.host = host; self.port = port; self.mount = mount; self.user = user; self.pass = pass
        self.overCellular = overCellular; self.position = position; self.onFrame = onFrame
    }

    func start() {
        running = true
        session()
    }

    func stop() {
        running = false
        retry?.cancel()
        ggaTimer?.invalidate(); ggaTimer = nil
        conn?.cancel(); conn = nil
        status = "Stopped"
    }

    private func session() {
        guard running else { return }
        status = "Connecting to \(host):\(port)…"
        header = []; gotHeader = false
        let tcp = NWProtocolTCP.Options()
        tcp.connectionTimeout = 8
        let params = NWParameters(tls: nil, tcp: tcp)
        #if os(iOS)
        if overCellular { params.requiredInterfaceType = .cellular }
        #endif
        let c = NWConnection(host: NWEndpoint.Host(host), port: NWEndpoint.Port(rawValue: port)!, using: params)
        conn = c
        c.stateUpdateHandler = { [weak self] st in
            guard let self = self, self.conn === c else { return }
            switch st {
            case .ready: self.sendRequest(c)
            case .failed(let e): self.lost("Error: \(e.localizedDescription)")
            case .waiting(let e): self.lost("Error: \(e.localizedDescription)")
            default: break
            }
        }
        c.start(queue: .main)
    }

    private func lost(_ why: String) {
        ggaTimer?.invalidate(); ggaTimer = nil
        conn?.cancel(); conn = nil
        guard running else { return }
        status = "\(why); retrying"
        let w = DispatchWorkItem { [weak self] in self?.session() }
        retry = w
        DispatchQueue.main.asyncAfter(deadline: .now() + 3, execute: w)
    }

    private func sendRequest(_ c: NWConnection) {
        let auth = Data("\(user):\(pass)".utf8).base64EncodedString()
        var req = "GET /\(mount) HTTP/1.0\r\n" +
            "User-Agent: NTRIP Esp32RoverGcs/1.0\r\n" +
            "Accept: */*\r\n"
        if !user.isEmpty { req += "Authorization: Basic \(auth)\r\n" }
        req += "Connection: close\r\n\r\n"
        c.send(content: Data(req.utf8), completion: .contentProcessed { _ in })
        receive(c)
    }

    private func sendGga() {
        guard let c = conn, gotHeader, let p = position() else { return }
        c.send(content: Data(NtripClient.gga(p.lat, p.lon).utf8), completion: .contentProcessed { _ in })
    }

    private func receive(_ c: NWConnection) {
        c.receive(minimumIncompleteLength: 1, maximumLength: 4096) { [weak self] data, _, done, err in
            guard let self = self, self.conn === c else { return }
            if let d = data, !d.isEmpty { self.got([UInt8](d)) }
            if let e = err { self.lost("Error: \(e.localizedDescription)"); return }
            if done { self.lost("caster closed the stream"); return }
            if self.conn === c { self.receive(c) }
        }
    }

    private func got(_ d: [UInt8]) {
        if gotHeader {
            bytes += Int64(d.count)
            splitter.feed(d)
            return
        }
        header += d
        // The answer's first line: "ICY 200 OK" (NTRIP 1) or "HTTP/1.x 200 OK" and headers.
        guard let nl = header.firstIndex(of: 0x0A) else {
            if header.count > 2048 { lost("Error: no answer") }
            return
        }
        let first = String(decoding: header[..<nl], as: UTF8.self).trimmingCharacters(in: .whitespacesAndNewlines)
        if !first.contains("200") {
            let why = first.contains("401") ? "wrong user or password" : first.hasPrefix("SOURCETABLE") ? "mountpoint not found" : (first.isEmpty ? "no answer" : first)
            running = false
            ggaTimer?.invalidate()
            conn?.cancel(); conn = nil
            status = "Error: \(why)"
            return
        }
        var bodyStart = nl + 1
        if first.hasPrefix("HTTP") {
            // Skip the headers up to the blank line.
            let crlf2: [UInt8] = [0x0D, 0x0A, 0x0D, 0x0A]
            guard let end = find(header, crlf2) else { return }
            bodyStart = end + 4
        }
        gotHeader = true
        status = "Connected: \(mount)"
        let rest = Array(header[bodyStart...])
        header = []
        if !rest.isEmpty { bytes += Int64(rest.count); splitter.feed(rest) }
        sendGga()
        let t = Timer(timeInterval: 10, repeats: true) { [weak self] _ in self?.sendGga() }
        RunLoop.main.add(t, forMode: .common)
        ggaTimer = t
    }

    private func find(_ hay: [UInt8], _ needle: [UInt8]) -> Int? {
        if hay.count < needle.count { return nil }
        for i in 0...(hay.count - needle.count) where Array(hay[i..<i + needle.count]) == needle { return i }
        return nil
    }

    static func gga(_ lat: Double, _ lon: Double) -> String {
        var cal = Calendar(identifier: .gregorian)
        cal.timeZone = TimeZone(identifier: "UTC")!
        let t = cal.dateComponents([.hour, .minute, .second], from: Date())
        func dm(_ v: Double, _ degDigits: Int) -> String {
            let a = abs(v), d = Int(a), m = (a - Double(d)) * 60
            return String(format: "%0\(degDigits)ld%07.4f", d, m)
        }
        let body = String(format: "GPGGA,%02ld%02ld%02ld.00,%@,%@,%@,%@,1,12,1.0,0.0,M,0.0,M,,",
                          t.hour ?? 0, t.minute ?? 0, t.second ?? 0,
                          dm(lat, 2), lat >= 0 ? "N" : "S", dm(lon, 3), lon >= 0 ? "E" : "W")
        var cs: UInt8 = 0
        for ch in body.utf8 { cs ^= ch }
        return "$" + body + String(format: "*%02X\r\n", cs)
    }
}

/// Finds RTCM3 frames (0xD3, 10-bit length, payload, CRC-24Q) in a byte stream.
final class Rtcm3Splitter {
    private var buf = [UInt8]()
    private let out: ([UInt8], Int) -> Void

    init(_ out: @escaping ([UInt8], Int) -> Void) { self.out = out }

    func feed(_ data: [UInt8]) {
        buf += data
        var i = 0
        while i < buf.count {
            if buf[i] != 0xD3 { i += 1; continue }
            if buf.count - i < 6 { break }
            let plen = (Int(buf[i + 1]) & 0x03) << 8 | Int(buf[i + 2])
            let total = 3 + plen + 3
            if buf.count - i < total { break }
            let crc = Int(buf[i + total - 3]) << 16 | Int(buf[i + total - 2]) << 8 | Int(buf[i + total - 1])
            if Rtcm3Splitter.crc24q(buf, i, 3 + plen) != crc { i += 1; continue }
            let type = plen >= 2 ? (Int(buf[i + 3]) << 4 | Int(buf[i + 4]) >> 4) : 0
            out(Array(buf[i..<i + total]), type)
            i += total
        }
        buf.removeFirst(i)
        if buf.count > 8192 { buf.removeAll() }      // garbage that never framed
    }

    static func crc24q(_ b: [UInt8], _ from: Int, _ n: Int) -> Int {
        var crc = 0
        for k in from..<from + n {
            crc ^= Int(b[k]) << 16
            for _ in 0..<8 {
                crc <<= 1
                if crc & 0x1000000 != 0 { crc ^= 0x1864CFB }
            }
        }
        return crc & 0xffffff
    }
}

import Foundation
import Network

/// A byte pipe to the autopilot, on Network.framework. Every callback runs on
/// the main queue, like the rest of the app, so nothing needs locking.
class Link {
    var onBytes: ((Data) -> Void)?
    var onClosed: ((String) -> Void)?
    private(set) var open = false
    var name: String { "" }

    fileprivate var stopping = false

    func start() {}
    func send(_ d: Data) {}
    /// Sends what is queued, then closes (the last frame is often a zero-stick MANUAL_CONTROL).
    func close() {}

    fileprivate func setOpen(_ v: Bool) { open = v }

    fileprivate func fail(_ why: String) {
        if stopping { return }
        stopping = true
        open = false
        onClosed?(why)
    }
}

/// MAVLink over TCP: the board listens on 192.168.4.1:5760 (also ArduPilot SITL's default).
final class TcpLink: Link {
    private let host: String
    private let port: UInt16
    private var conn: NWConnection?
    override var name: String { "TCP \(host):\(port)" }

    init(host: String, port: UInt16) { self.host = host; self.port = port }

    override func start() {
        let tcp = NWProtocolTCP.Options()
        tcp.noDelay = true
        tcp.connectionTimeout = 5
        let params = NWParameters(tls: nil, tcp: tcp)
        // The board's WiFi has no internet; never send its traffic over mobile data.
        if isLocal(host) { params.prohibitedInterfaceTypes = [.cellular] }
        let c = NWConnection(host: NWEndpoint.Host(host), port: NWEndpoint.Port(rawValue: port)!, using: params)
        conn = c
        c.stateUpdateHandler = { [weak self] st in
            guard let self = self else { return }
            switch st {
            case .ready: self.setOpen(true); self.receive()
            case .failed(let e): self.fail(e.localizedDescription); c.cancel()
            case .waiting(let e): self.fail(e.localizedDescription); c.cancel()
            case .cancelled: self.fail("closed")
            default: break
            }
        }
        c.start(queue: .main)
    }

    private func receive() {
        conn?.receive(minimumIncompleteLength: 1, maximumLength: 4096) { [weak self] data, _, done, err in
            guard let self = self, !self.stopping else { return }
            if let d = data, !d.isEmpty { self.onBytes?(d) }
            if let e = err { self.fail(e.localizedDescription); return }
            if done { self.fail("closed by the board"); return }
            self.receive()
        }
    }

    override func send(_ d: Data) {
        guard open, let c = conn else { return }
        c.send(content: d, completion: .contentProcessed { _ in })
    }

    override func close() {
        stopping = true
        setOpen(false)
        guard let c = conn else { return }
        conn = nil
        // Let queued frames go out first: a send marked final, then cancel when it is processed.
        c.send(content: nil, contentContext: .finalMessage, isComplete: true, completion: .contentProcessed { _ in c.cancel() })
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.3) { c.cancel() }
    }
}

/// MAVLink over UDP. With [host] blank it listens on [port] and answers whoever
/// sends (SITL / mavproxy --out, or the board's HAL_ESP32_WIFI 2 mode).
final class UdpLink: Link {
    private let host: String
    private let port: UInt16
    private var listener: NWListener?
    private var conn: NWConnection?
    override var name: String { host.isEmpty ? "UDP listen :\(port)" : "UDP \(host):\(port)" }

    init(host: String, port: UInt16) { self.host = host; self.port = port }

    override func start() {
        let params = NWParameters.udp
        params.prohibitedInterfaceTypes = [.cellular]
        if host.isEmpty {
            params.allowLocalEndpointReuse = true
            do {
                let l = try NWListener(using: params, on: NWEndpoint.Port(rawValue: port)!)
                listener = l
                l.newConnectionHandler = { [weak self] c in
                    guard let self = self else { c.cancel(); return }
                    self.conn?.cancel()
                    self.use(c)
                }
                l.stateUpdateHandler = { [weak self] st in
                    if case .failed(let e) = st { self?.fail(e.localizedDescription) }
                    if case .ready = st { self?.setOpen(true) }
                }
                l.start(queue: .main)
            } catch {
                DispatchQueue.main.async { self.fail(error.localizedDescription) }
            }
        } else {
            let c = NWConnection(host: NWEndpoint.Host(host), port: NWEndpoint.Port(rawValue: port)!, using: params)
            use(c)
        }
    }

    private func use(_ c: NWConnection) {
        conn = c
        c.stateUpdateHandler = { [weak self] st in
            guard let self = self else { return }
            switch st {
            case .ready: self.setOpen(true); self.receive(c)
            case .failed(let e): if self.listener == nil { self.fail(e.localizedDescription) }
            default: break
            }
        }
        c.start(queue: .main)
    }

    private func receive(_ c: NWConnection) {
        c.receiveMessage { [weak self] data, _, _, err in
            guard let self = self, !self.stopping, self.conn === c else { return }
            if let d = data, !d.isEmpty { self.onBytes?(d) }
            if err == nil { self.receive(c) }
        }
    }

    override func send(_ d: Data) {
        guard open, let c = conn else { return }
        c.send(content: d, completion: .contentProcessed { _ in })
    }

    override func close() {
        stopping = true
        setOpen(false)
        let c = conn, l = listener
        conn = nil; listener = nil
        // UDP sends are immediate; give the last frames a moment before cancelling.
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.2) { c?.cancel(); l?.cancel() }
    }
}

/// Private, link-local or loopback IPv4 literal: an address on the board's WiFi, not on the internet.
func isLocal(_ host: String) -> Bool {
    let p = host.split(separator: ".").compactMap { Int($0) }
    if p.count != 4 { return false }
    return p[0] == 10 || p[0] == 127 || (p[0] == 192 && p[1] == 168) ||
        (p[0] == 172 && (16...31).contains(p[1])) || (p[0] == 169 && p[1] == 254)
}

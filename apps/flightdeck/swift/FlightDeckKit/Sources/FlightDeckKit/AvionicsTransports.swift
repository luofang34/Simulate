import Foundation
import Network

/// TCP client connecting to an `avionics-link` Connext bridge or cockpit Wi-Fi relay.
///
/// Mirrors `avionics-link`'s `AccessoryLink` streaming byte-pipe contract:
/// incoming stream bytes are delivered verbatim to `FlightDeckSession`, while outbound
/// `0x02` handshake/keepalive and `FPN/RP` route-poll frames are flushed back onto the link.
/// (Direct iPadOS Lightning/USB-C `ExternalAccessory` attachment is deferred alongside the iPad UI.)
public final class ConnextTcpClient: @unchecked Sendable {
    private let queue = DispatchQueue(label: "sim.flightdeck.connext.tcp")
    private var connection: NWConnection?
    private let onStateChange: @Sendable (Bool, String) -> Void
    private let onBytes: @Sendable (Data) -> Void

    public init(
        onStateChange: @escaping @Sendable (Bool, String) -> Void,
        onBytes: @escaping @Sendable (Data) -> Void
    ) {
        self.onStateChange = onStateChange
        self.onBytes = onBytes
    }

    public func connect(host: String, port: UInt16) {
        disconnect()
        guard let nwPort = NWEndpoint.Port(rawValue: port) else {
            onStateChange(false, "Invalid TCP port \(port)")
            return
        }
        let conn = NWConnection(host: NWEndpoint.Host(host), port: nwPort, using: .tcp)
        self.connection = conn
        conn.stateUpdateHandler = { [weak self] state in
            guard let self else { return }
            switch state {
            case .ready:
                self.onStateChange(true, "Connext TCP connected (\(host):\(port))")
                self.receiveLoop(on: conn)
            case let .failed(err):
                self.onStateChange(false, "Connext TCP failed: \(err.localizedDescription)")
            case .cancelled:
                self.onStateChange(false, "Connext TCP disconnected")
            default:
                break
            }
        }
        conn.start(queue: queue)
    }

    public func send(_ packets: [[UInt8]]) {
        guard let conn = connection, !packets.isEmpty else { return }
        for pkt in packets where !pkt.isEmpty {
            conn.send(content: Data(pkt), completion: .contentProcessed { _ in })
        }
    }

    public func disconnect() {
        connection?.cancel()
        connection = nil
    }

    private func receiveLoop(on conn: NWConnection) {
        conn.receive(minimumIncompleteLength: 1, maximumLength: 8192) { [weak self] data, _, isEOF, error in
            guard let self else { return }
            if let data, !data.isEmpty {
                self.onBytes(data)
            }
            if isEOF || error != nil {
                self.onStateChange(false, "Connext TCP stream closed")
                return
            }
            self.receiveLoop(on: conn)
        }
    }
}

/// UDP listener receiving GDL-90 ownship GNSS, AHRS (`0x65`), and ADS-B traffic (`0x14`) on port `4000`.
public final class Gdl90UdpListener: @unchecked Sendable {
    private let queue = DispatchQueue(label: "sim.flightdeck.gdl90.udp")
    private var listener: NWListener?
    private let onStateChange: @Sendable (Bool, String) -> Void
    private let onDatagram: @Sendable (Data) -> Void

    public init(
        onStateChange: @escaping @Sendable (Bool, String) -> Void,
        onDatagram: @escaping @Sendable (Data) -> Void
    ) {
        self.onStateChange = onStateChange
        self.onDatagram = onDatagram
    }

    public func start(port: UInt16 = 4000) {
        stop()
        guard let nwPort = NWEndpoint.Port(rawValue: port),
              let nwListener = try? NWListener(using: .udp, on: nwPort) else {
            onStateChange(false, "Unable to bind UDP :\(port)")
            return
        }
        self.listener = nwListener
        nwListener.stateUpdateHandler = { [weak self] state in
            guard let self else { return }
            switch state {
            case .ready:
                self.onStateChange(true, "Listening for GDL-90 on UDP :\(port)")
            case let .failed(err):
                self.onStateChange(false, "GDL-90 listener error: \(err.localizedDescription)")
            case .cancelled:
                self.onStateChange(false, "GDL-90 listener stopped")
            default:
                break
            }
        }
        nwListener.newConnectionHandler = { [weak self] conn in
            guard let self else { return }
            conn.start(queue: self.queue)
            self.receiveDatagrams(on: conn)
        }
        nwListener.start(queue: queue)
    }

    public func stop() {
        listener?.cancel()
        listener = nil
    }

    private func receiveDatagrams(on conn: NWConnection) {
        conn.receiveMessage { [weak self] data, _, _, error in
            guard let self else { return }
            if let data, !data.isEmpty {
                self.onDatagram(data)
            }
            if error == nil {
                self.receiveDatagrams(on: conn)
            }
        }
    }
}

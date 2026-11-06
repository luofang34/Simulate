use std::collections::VecDeque;
use std::io::{ErrorKind, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, UdpSocket};
use std::time::{Duration, Instant};

use sim_core::{
    EndpointError, ExternalEndpoint, SimInstant, StampedResponse, TICKS_PER_SECOND, TickSpan,
};

use crate::mavlink_hil::clock_sync::{DEFAULT_CLOCK_RESYNC_THRESHOLD, HilLinkStats};
use crate::mavlink_hil::messages::{HilActuatorControls, SIM_COMPONENT_ID, SIM_SYSTEM_ID};
use crate::mavlink_hil::reader::MavlinkHilReader;
use crate::{XilError, XilStimulus};

mod peer_link;
mod sequencer;

use peer_link::PeerLink;
use sequencer::OutboundSequencer;

/// Emission schedule and timeout configuration for MAVLink v2 HIL endpoints.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MavlinkHilConfig {
    /// Simulator source MAVLink system ID.
    pub system_id: u8,
    /// Simulator source MAVLink component ID.
    pub component_id: u8,
    /// Minimum master-clock interval between `HIL_GPS` messages (`0` = every step).
    pub gps_period: TickSpan,
    /// Minimum master-clock interval between `HEARTBEAT` messages.
    pub heartbeat_period: TickSpan,
    /// Whether to also emit `HIL_STATE_QUATERNION` ground truth alongside `HIL_SENSOR`.
    pub send_state_quaternion: bool,
    /// Maximum wall-clock wait during [`ExternalEndpoint::exchange_blocking`].
    pub blocking_timeout: Duration,
    /// Peer `time_usec` jump (either direction) treated as a peer reboot or
    /// pause and re-anchored; see
    /// [`PeerClockSynchronizer`](crate::mavlink_hil::PeerClockSynchronizer).
    pub clock_resync_threshold: TickSpan,
    /// When set, `HIL_ACTUATOR_CONTROLS` lagging the newest `HIL_SENSOR` sent
    /// by more than this are dropped rather than replacing fresher controls
    /// ([`DEFAULT_MAX_REPLY_AGE`](crate::mavlink_hil::DEFAULT_MAX_REPLY_AGE)
    /// suits a 100 ms loop budget). Off by default, since a free-running peer
    /// that stamps replies with a frozen clock would otherwise be cut off.
    pub max_reply_age: Option<TickSpan>,
}

impl Default for MavlinkHilConfig {
    fn default() -> Self {
        Self {
            system_id: SIM_SYSTEM_ID,
            component_id: SIM_COMPONENT_ID,
            gps_period: TickSpan::from_ticks(TICKS_PER_SECOND / 10),
            heartbeat_period: TickSpan::from_ticks(TICKS_PER_SECOND),
            send_state_quaternion: false,
            blocking_timeout: Duration::from_secs(2),
            clock_resync_threshold: DEFAULT_CLOCK_RESYNC_THRESHOLD,
            max_reply_age: None,
        }
    }
}

/// Outbound bytes held for a peer that is not reading; beyond this `post`
/// fails rather than block the simulation or grow without bound.
const TCP_TX_CAPACITY: usize = 65_536;

/// How long lockstep waits between attempts to flush to a full socket.
const TX_RETRY_INTERVAL: Duration = Duration::from_micros(100);

/// MAVLink v2 HIL endpoint over a TCP stream (for PX4 lockstep TCP port `4560`,
/// `mavlink-router`, or `socat` serial bridges).
///
/// The stream stays non-blocking: outbound bursts queue in a bounded buffer
/// that drains as the peer accepts bytes, so [`ExternalEndpoint::post`] never
/// stalls a real-time run on a slow peer.
#[derive(Debug)]
pub struct MavlinkHilTcpEndpoint {
    stream: TcpStream,
    sequencer: OutboundSequencer,
    link: PeerLink,
    tx_buf: VecDeque<u8>,
}

impl MavlinkHilTcpEndpoint {
    /// Wraps an already-connected [`TcpStream`], enabling `TCP_NODELAY` and
    /// non-blocking I/O.
    pub fn from_stream(stream: TcpStream, config: MavlinkHilConfig) -> Result<Self, XilError> {
        stream.set_nodelay(true)?;
        stream.set_nonblocking(true)?;
        Ok(Self {
            stream,
            sequencer: OutboundSequencer::new(config),
            link: PeerLink::new(&config),
            tx_buf: VecDeque::with_capacity(TCP_TX_CAPACITY),
        })
    }

    /// Binds `addr` (e.g. `127.0.0.1:4560`) and accepts one incoming flight
    /// controller connection, matching PX4 SITL's lockstep simulator startup.
    pub fn bind_and_accept(addr: SocketAddr, config: MavlinkHilConfig) -> Result<Self, XilError> {
        let listener = TcpListener::bind(addr)?;
        let (stream, _) = listener.accept()?;
        Self::from_stream(stream, config)
    }

    /// Dials a listening MAVLink router or UART bridge at `addr`.
    pub fn connect(addr: SocketAddr, config: MavlinkHilConfig) -> Result<Self, XilError> {
        let stream = TcpStream::connect(addr)?;
        Self::from_stream(stream, config)
    }

    /// Whether the most recent heartbeat or accepted actuator message
    /// reported armed.
    #[must_use]
    pub const fn peer_armed(&self) -> bool {
        self.link.peer_armed
    }

    /// Streaming parser diagnostics.
    #[must_use]
    pub const fn reader(&self) -> &MavlinkHilReader {
        &self.link.reader
    }

    /// Clock-mapping, correlation, and staleness counters.
    #[must_use]
    pub const fn link_stats(&self) -> HilLinkStats {
        self.link.stats()
    }

    /// Outbound bytes queued but not yet accepted by the socket (at most
    /// 65,536).
    #[must_use]
    pub fn pending_tx_bytes(&self) -> usize {
        self.tx_buf.len()
    }

    /// Writes as much queued outbound data as the socket accepts without
    /// blocking.
    pub fn flush_tx(&mut self) -> Result<(), XilError> {
        loop {
            let (front, _) = self.tx_buf.as_slices();
            if front.is_empty() {
                return Ok(());
            }
            match self.stream.write(front) {
                Ok(0) => return Err(std::io::Error::from(ErrorKind::WriteZero).into()),
                Ok(n) => drop(self.tx_buf.drain(..n.min(front.len()))),
                Err(err) if err.kind() == ErrorKind::WouldBlock => return Ok(()),
                Err(err) if err.kind() == ErrorKind::Interrupted => {}
                Err(err) => return Err(err.into()),
            }
        }
    }

    fn enqueue_burst(
        &mut self,
        at: SimInstant,
        stimulus: &XilStimulus,
    ) -> Result<(), EndpointError> {
        self.flush_tx()?;
        let bytes = self.sequencer.build_burst(at, stimulus)?;
        let queued = self.tx_buf.len();
        if queued.saturating_add(bytes.len()) > TCP_TX_CAPACITY {
            return Err(EndpointError {
                detail: format!(
                    "MAVLink HIL TCP outbound buffer full (peer not draining): \
                     {queued} queued + {} new bytes exceeds {TCP_TX_CAPACITY}",
                    bytes.len()
                ),
            });
        }
        self.tx_buf.extend(bytes.iter().copied());
        self.link.note_sensor_sent(at);
        self.flush_tx()?;
        Ok(())
    }

    fn flush_all_until(&mut self, deadline: Instant) -> Result<(), EndpointError> {
        loop {
            self.flush_tx()?;
            if self.tx_buf.is_empty() {
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(EndpointError {
                    detail: format!(
                        "timed out flushing {} HIL bytes to MAVLink TCP peer",
                        self.tx_buf.len()
                    ),
                });
            }
            std::thread::sleep(TX_RETRY_INTERVAL);
        }
    }

    /// One read that waits at most until `deadline`, leaving the stream
    /// non-blocking afterwards. `Ok(None)` means nothing arrived in time.
    fn read_until(&mut self, deadline: Instant, buf: &mut [u8]) -> Result<Option<usize>, XilError> {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Ok(None);
        }
        self.stream.set_nonblocking(false)?;
        let read = self
            .stream
            .set_read_timeout(Some(remaining))
            .and_then(|()| self.stream.read(buf));
        self.stream.set_nonblocking(true)?;
        match read {
            Ok(n) => Ok(Some(n)),
            Err(err)
                if matches!(
                    err.kind(),
                    ErrorKind::WouldBlock | ErrorKind::TimedOut | ErrorKind::Interrupted
                ) =>
            {
                Ok(None)
            }
            Err(err) => Err(err.into()),
        }
    }
}

impl ExternalEndpoint for MavlinkHilTcpEndpoint {
    type Stimulus = XilStimulus;
    type Response = HilActuatorControls;

    fn exchange_blocking(
        &mut self,
        at: SimInstant,
        stimulus: &Self::Stimulus,
    ) -> Result<Self::Response, EndpointError> {
        let deadline = Instant::now() + self.sequencer.config.blocking_timeout;
        self.enqueue_burst(at, stimulus)?;
        self.flush_all_until(deadline)?;

        let mut buf = [0_u8; 512];
        loop {
            if let Some(stamped) = self.link.next_fresh_controls() {
                return Ok(stamped.response);
            }
            match self.read_until(deadline, &mut buf)? {
                None if Instant::now() >= deadline => {
                    return Err(EndpointError {
                        detail: "timed out waiting for HIL_ACTUATOR_CONTROLS over TCP".to_owned(),
                    });
                }
                None => {}
                Some(0) => {
                    return Err(EndpointError {
                        detail: "MAVLink HIL TCP peer closed connection".to_owned(),
                    });
                }
                Some(n) => {
                    if let Some(chunk) = buf.get(..n) {
                        self.link.reader.push_bytes(chunk);
                    }
                }
            }
        }
    }

    fn post(&mut self, at: SimInstant, stimulus: &Self::Stimulus) -> Result<(), EndpointError> {
        self.enqueue_burst(at, stimulus)
    }

    fn try_receive(&mut self) -> Result<Option<StampedResponse<Self::Response>>, EndpointError> {
        self.flush_tx()?;
        let mut buf = [0_u8; 512];
        loop {
            match self.stream.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => {
                    if let Some(chunk) = buf.get(..n) {
                        self.link.reader.push_bytes(chunk);
                    }
                }
                Err(err) if err.kind() == ErrorKind::WouldBlock => break,
                Err(err) if err.kind() == ErrorKind::Interrupted => {}
                Err(err) => return Err(XilError::from(err).into()),
            }
        }
        Ok(self.link.next_fresh_controls())
    }
}

/// MAVLink v2 HIL endpoint over UDP datagrams.
#[derive(Debug)]
pub struct MavlinkHilUdpEndpoint {
    socket: UdpSocket,
    peer: SocketAddr,
    sequencer: OutboundSequencer,
    link: PeerLink,
}

impl MavlinkHilUdpEndpoint {
    /// Binds `local_addr` and sends HIL telemetry datagrams to `peer_addr`.
    pub fn bind(
        local_addr: SocketAddr,
        peer_addr: SocketAddr,
        config: MavlinkHilConfig,
    ) -> Result<Self, XilError> {
        let socket = UdpSocket::bind(local_addr)?;
        Ok(Self {
            socket,
            peer: peer_addr,
            sequencer: OutboundSequencer::new(config),
            link: PeerLink::new(&config),
        })
    }

    /// Local bound socket address.
    pub fn local_addr(&self) -> Result<SocketAddr, XilError> {
        Ok(self.socket.local_addr()?)
    }

    /// Whether the most recent heartbeat or accepted actuator message
    /// reported armed.
    #[must_use]
    pub const fn peer_armed(&self) -> bool {
        self.link.peer_armed
    }

    /// Clock-mapping, correlation, and staleness counters.
    #[must_use]
    pub const fn link_stats(&self) -> HilLinkStats {
        self.link.stats()
    }
}

impl ExternalEndpoint for MavlinkHilUdpEndpoint {
    type Stimulus = XilStimulus;
    type Response = HilActuatorControls;

    fn exchange_blocking(
        &mut self,
        at: SimInstant,
        stimulus: &Self::Stimulus,
    ) -> Result<Self::Response, EndpointError> {
        let timeout = self.sequencer.config.blocking_timeout;
        let bytes = self.sequencer.build_burst(at, stimulus)?;
        self.socket.set_nonblocking(false).map_err(XilError::from)?;
        self.socket
            .set_read_timeout(Some(timeout))
            .map_err(XilError::from)?;
        self.socket
            .send_to(bytes, self.peer)
            .map_err(XilError::from)?;
        self.link.note_sensor_sent(at);

        let deadline = Instant::now() + timeout;
        let mut buf = [0_u8; 512];
        loop {
            if let Some(stamped) = self.link.next_fresh_controls() {
                return Ok(stamped.response);
            }
            if Instant::now() >= deadline {
                return Err(EndpointError {
                    detail: "timed out waiting for HIL_ACTUATOR_CONTROLS over UDP".to_owned(),
                });
            }
            let (n, _) = self.socket.recv_from(&mut buf).map_err(XilError::from)?;
            if let Some(chunk) = buf.get(..n) {
                self.link.reader.push_bytes(chunk);
            }
        }
    }

    fn post(&mut self, at: SimInstant, stimulus: &Self::Stimulus) -> Result<(), EndpointError> {
        let bytes = self.sequencer.build_burst(at, stimulus)?;
        self.socket
            .send_to(bytes, self.peer)
            .map_err(XilError::from)?;
        self.link.note_sensor_sent(at);
        Ok(())
    }

    fn try_receive(&mut self) -> Result<Option<StampedResponse<Self::Response>>, EndpointError> {
        self.socket.set_nonblocking(true).map_err(XilError::from)?;
        let mut buf = [0_u8; 512];
        loop {
            match self.socket.recv_from(&mut buf) {
                Ok((n, _)) => {
                    if let Some(chunk) = buf.get(..n) {
                        self.link.reader.push_bytes(chunk);
                    }
                }
                Err(err) if err.kind() == ErrorKind::WouldBlock => break,
                Err(err) => return Err(XilError::from(err).into()),
            }
        }
        Ok(self.link.next_fresh_controls())
    }
}

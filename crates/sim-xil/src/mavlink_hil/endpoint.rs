use std::io::{ErrorKind, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, UdpSocket};
use std::time::{Duration, Instant};

use sim_core::{
    EndpointError, ExternalEndpoint, SimInstant, StampedResponse, TICKS_PER_SECOND, TickSpan,
};

use crate::mavlink_hil::messages::{
    HIL_SENSOR_FIELDS_ALL, HilActuatorControls, HilGpsSample, HilSensorSample,
    HilStateQuaternionSample, SIM_COMPONENT_ID, SIM_SYSTEM_ID, encode_heartbeat, encode_hil_gps,
    encode_hil_sensor, encode_hil_state_quaternion,
};
use crate::mavlink_hil::reader::{MavlinkHilFrame, MavlinkHilReader};
use crate::{XilError, XilStimulus};

const TICKS_PER_MICROSECOND: u64 = TICKS_PER_SECOND / 1_000_000;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(1_000_000));

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
        }
    }
}

#[derive(Debug)]
struct OutboundSequencer {
    config: MavlinkHilConfig,
    seq: u8,
    last_gps_at: Option<SimInstant>,
    last_heartbeat_at: Option<SimInstant>,
    scratch: Vec<u8>,
}

impl OutboundSequencer {
    fn new(config: MavlinkHilConfig) -> Self {
        Self {
            config,
            seq: 0,
            last_gps_at: None,
            last_heartbeat_at: None,
            scratch: Vec::with_capacity(256),
        }
    }

    fn build_burst(&mut self, at: SimInstant, stimulus: &XilStimulus) -> Result<&[u8], XilError> {
        self.scratch.clear();
        let time_usec = (at.ticks_u128() / u128::from(TICKS_PER_MICROSECOND)) as u64;
        let mut frame = [0_u8; 280];

        if should_emit(at, self.last_heartbeat_at, self.config.heartbeat_period) {
            let n = encode_heartbeat(
                self.next_seq(),
                self.config.system_id,
                self.config.component_id,
                false,
                &mut frame,
            )?;
            append_prefix(&mut self.scratch, &frame, n);
            self.last_heartbeat_at = Some(at);
        }

        let sensor = HilSensorSample::from_stimulus(time_usec, stimulus, HIL_SENSOR_FIELDS_ALL)?;
        let n = encode_hil_sensor(
            self.next_seq(),
            self.config.system_id,
            self.config.component_id,
            &sensor,
            &mut frame,
        )?;
        append_prefix(&mut self.scratch, &frame, n);

        if should_emit(at, self.last_gps_at, self.config.gps_period) {
            let gps = HilGpsSample::from_stimulus(time_usec, stimulus)?;
            let n = encode_hil_gps(
                self.next_seq(),
                self.config.system_id,
                self.config.component_id,
                &gps,
                &mut frame,
            )?;
            append_prefix(&mut self.scratch, &frame, n);
            self.last_gps_at = Some(at);
        }

        if self.config.send_state_quaternion {
            let truth = HilStateQuaternionSample::from_stimulus(time_usec, stimulus)?;
            let n = encode_hil_state_quaternion(
                self.next_seq(),
                self.config.system_id,
                self.config.component_id,
                &truth,
                &mut frame,
            )?;
            append_prefix(&mut self.scratch, &frame, n);
        }

        Ok(&self.scratch)
    }

    const fn next_seq(&mut self) -> u8 {
        let current = self.seq;
        self.seq = self.seq.wrapping_add(1);
        current
    }
}

/// MAVLink v2 HIL endpoint over a TCP stream (for PX4 lockstep TCP port `4560`,
/// `mavlink-router`, or `socat` serial bridges).
#[derive(Debug)]
pub struct MavlinkHilTcpEndpoint {
    stream: TcpStream,
    sequencer: OutboundSequencer,
    reader: MavlinkHilReader,
    peer_armed: bool,
}

impl MavlinkHilTcpEndpoint {
    /// Wraps an already-connected [`TcpStream`], enabling `TCP_NODELAY`.
    pub fn from_stream(stream: TcpStream, config: MavlinkHilConfig) -> Result<Self, XilError> {
        stream.set_nodelay(true)?;
        Ok(Self {
            stream,
            sequencer: OutboundSequencer::new(config),
            reader: MavlinkHilReader::new(),
            peer_armed: false,
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

    /// Whether the most recent heartbeat or actuator message reported armed.
    #[must_use]
    pub const fn peer_armed(&self) -> bool {
        self.peer_armed
    }

    /// Streaming parser diagnostics.
    #[must_use]
    pub const fn reader(&self) -> &MavlinkHilReader {
        &self.reader
    }

    fn drain_ready_controls(&mut self) -> Option<HilActuatorControls> {
        while let Some(frame) = self.reader.pop_frame() {
            match frame {
                MavlinkHilFrame::Heartbeat { armed, .. } => {
                    self.peer_armed = armed;
                }
                MavlinkHilFrame::ActuatorControls { controls, .. } => {
                    self.peer_armed = controls.is_armed();
                    return Some(controls);
                }
            }
        }
        None
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
        let timeout = self.sequencer.config.blocking_timeout;
        let bytes = self.sequencer.build_burst(at, stimulus)?;
        self.stream.set_nonblocking(false).map_err(XilError::from)?;
        self.stream
            .set_read_timeout(Some(timeout))
            .map_err(XilError::from)?;
        self.stream.write_all(bytes).map_err(XilError::from)?;

        let deadline = Instant::now() + timeout;
        let mut buf = [0_u8; 512];
        loop {
            if let Some(controls) = self.drain_ready_controls() {
                return Ok(controls);
            }
            if Instant::now() >= deadline {
                return Err(EndpointError {
                    detail: "timed out waiting for HIL_ACTUATOR_CONTROLS over TCP".to_owned(),
                });
            }
            let n = self.stream.read(&mut buf).map_err(XilError::from)?;
            if n == 0 {
                return Err(EndpointError {
                    detail: "MAVLink HIL TCP peer closed connection".to_owned(),
                });
            }
            if let Some(chunk) = buf.get(..n) {
                self.reader.push_bytes(chunk);
            }
        }
    }

    fn post(&mut self, at: SimInstant, stimulus: &Self::Stimulus) -> Result<(), EndpointError> {
        let bytes = self.sequencer.build_burst(at, stimulus)?;
        self.stream.set_nonblocking(false).map_err(XilError::from)?;
        self.stream.write_all(bytes).map_err(XilError::from)?;
        Ok(())
    }

    fn try_receive(&mut self) -> Result<Option<StampedResponse<Self::Response>>, EndpointError> {
        self.stream.set_nonblocking(true).map_err(XilError::from)?;
        let mut buf = [0_u8; 512];
        loop {
            match self.stream.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => {
                    if let Some(chunk) = buf.get(..n) {
                        self.reader.push_bytes(chunk);
                    }
                }
                Err(err) if err.kind() == ErrorKind::WouldBlock => break,
                Err(err) => return Err(XilError::from(err).into()),
            }
        }
        Ok(self.drain_ready_controls().map(stamp_controls))
    }
}

/// MAVLink v2 HIL endpoint over UDP datagrams.
#[derive(Debug)]
pub struct MavlinkHilUdpEndpoint {
    socket: UdpSocket,
    peer: SocketAddr,
    sequencer: OutboundSequencer,
    reader: MavlinkHilReader,
    peer_armed: bool,
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
            reader: MavlinkHilReader::new(),
            peer_armed: false,
        })
    }

    /// Local bound socket address.
    pub fn local_addr(&self) -> Result<SocketAddr, XilError> {
        Ok(self.socket.local_addr()?)
    }

    fn drain_ready_controls(&mut self) -> Option<HilActuatorControls> {
        while let Some(frame) = self.reader.pop_frame() {
            match frame {
                MavlinkHilFrame::Heartbeat { armed, .. } => {
                    self.peer_armed = armed;
                }
                MavlinkHilFrame::ActuatorControls { controls, .. } => {
                    self.peer_armed = controls.is_armed();
                    return Some(controls);
                }
            }
        }
        None
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

        let deadline = Instant::now() + timeout;
        let mut buf = [0_u8; 512];
        loop {
            if let Some(controls) = self.drain_ready_controls() {
                return Ok(controls);
            }
            if Instant::now() >= deadline {
                return Err(EndpointError {
                    detail: "timed out waiting for HIL_ACTUATOR_CONTROLS over UDP".to_owned(),
                });
            }
            let (n, _) = self.socket.recv_from(&mut buf).map_err(XilError::from)?;
            if let Some(chunk) = buf.get(..n) {
                self.reader.push_bytes(chunk);
            }
        }
    }

    fn post(&mut self, at: SimInstant, stimulus: &Self::Stimulus) -> Result<(), EndpointError> {
        let bytes = self.sequencer.build_burst(at, stimulus)?;
        self.socket
            .send_to(bytes, self.peer)
            .map_err(XilError::from)?;
        Ok(())
    }

    fn try_receive(&mut self) -> Result<Option<StampedResponse<Self::Response>>, EndpointError> {
        self.socket.set_nonblocking(true).map_err(XilError::from)?;
        let mut buf = [0_u8; 512];
        loop {
            match self.socket.recv_from(&mut buf) {
                Ok((n, _)) => {
                    if let Some(chunk) = buf.get(..n) {
                        self.reader.push_bytes(chunk);
                    }
                }
                Err(err) if err.kind() == ErrorKind::WouldBlock => break,
                Err(err) => return Err(XilError::from(err).into()),
            }
        }
        Ok(self.drain_ready_controls().map(stamp_controls))
    }
}

fn should_emit(now: SimInstant, last: Option<SimInstant>, period: TickSpan) -> bool {
    match last {
        None => true,
        Some(prev) => now.duration_since(prev) >= period,
    }
}

fn append_prefix(dst: &mut Vec<u8>, src: &[u8], len: usize) {
    if let Some(prefix) = src.get(..len) {
        dst.extend_from_slice(prefix);
    }
}

fn stamp_controls(controls: HilActuatorControls) -> StampedResponse<HilActuatorControls> {
    let ticks = controls.time_usec.saturating_mul(TICKS_PER_MICROSECOND);
    StampedResponse {
        deliver_at: SimInstant::START.advanced(TickSpan::from_ticks(ticks)),
        response: controls,
    }
}

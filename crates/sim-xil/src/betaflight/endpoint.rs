use std::io::ErrorKind;
use std::net::{SocketAddr, UdpSocket};
use std::time::Duration;

use sim_core::{EndpointError, ExternalEndpoint, SimInstant, StampedResponse};

use crate::betaflight::fdm::{BetaflightFdmPacket, BetaflightPacketClock, BetaflightServoPacket};
use crate::{XilError, XilStimulus};

/// Default Betaflight SITL UDP port receiving 144-byte `fdm_packet`s.
pub const BETAFLIGHT_FDM_PORT: u16 = 9003;
/// Default Betaflight SITL UDP port sending motor `pwm_pkt` / `servo_pkt`s.
pub const BETAFLIGHT_SERVO_PORT: u16 = 9002;

/// Configuration for [`BetaflightUdpEndpoint`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BetaflightUdpConfig {
    /// Whether to stamp `fdm_packet.timestamp` from [`BetaflightPacketClock`]
    /// (`0.025 s` increments) instead of the master clock's true seconds.
    pub use_fixed_packet_clock: bool,
    /// Maximum wall-clock wait during [`ExternalEndpoint::exchange_blocking`].
    pub blocking_timeout: Duration,
}

impl Default for BetaflightUdpConfig {
    fn default() -> Self {
        Self {
            use_fixed_packet_clock: false,
            blocking_timeout: Duration::from_secs(2),
        }
    }
}

/// UDP [`ExternalEndpoint`] speaking Betaflight SITL's binary FDM + PWM protocol.
#[derive(Debug)]
pub struct BetaflightUdpEndpoint {
    tx_socket: UdpSocket,
    rx_socket: UdpSocket,
    fdm_target: SocketAddr,
    config: BetaflightUdpConfig,
    packet_clock: BetaflightPacketClock,
    last_posted_at: SimInstant,
}

impl BetaflightUdpEndpoint {
    /// Binds `servo_listen_addr` (commonly `127.0.0.1:9002`) for incoming motor
    /// packets and an ephemeral TX socket sending `fdm_packet`s to `fdm_target`
    /// (commonly `127.0.0.1:9003`).
    pub fn bind(
        servo_listen_addr: SocketAddr,
        fdm_target: SocketAddr,
        config: BetaflightUdpConfig,
    ) -> Result<Self, XilError> {
        let rx_socket = UdpSocket::bind(servo_listen_addr)?;
        let tx_bind: SocketAddr = if fdm_target.is_ipv4() {
            ([127, 0, 0, 1], 0).into()
        } else {
            ([0, 0, 0, 0, 0, 0, 0, 1], 0).into()
        };
        let tx_socket = UdpSocket::bind(tx_bind)?;
        Ok(Self {
            tx_socket,
            rx_socket,
            fdm_target,
            config,
            packet_clock: BetaflightPacketClock::new(),
            last_posted_at: SimInstant::START,
        })
    }

    /// Local bound address receiving motor servo datagrams.
    pub fn servo_local_addr(&self) -> Result<SocketAddr, XilError> {
        Ok(self.rx_socket.local_addr()?)
    }

    fn send_fdm(&mut self, at: SimInstant, stimulus: &XilStimulus) -> Result<(), XilError> {
        let timestamp_s = if self.config.use_fixed_packet_clock {
            self.packet_clock.next_timestamp_s()
        } else {
            at.duration_since(SimInstant::START).as_seconds()
        };
        let packet = BetaflightFdmPacket::from_stimulus(timestamp_s, stimulus)?;
        let bytes = packet.to_le_bytes();
        self.tx_socket.send_to(&bytes, self.fdm_target)?;
        self.last_posted_at = at;
        Ok(())
    }
}

impl ExternalEndpoint for BetaflightUdpEndpoint {
    type Stimulus = XilStimulus;
    type Response = BetaflightServoPacket;

    fn exchange_blocking(
        &mut self,
        at: SimInstant,
        stimulus: &Self::Stimulus,
    ) -> Result<Self::Response, EndpointError> {
        self.send_fdm(at, stimulus)?;
        self.rx_socket
            .set_nonblocking(false)
            .map_err(XilError::from)?;
        self.rx_socket
            .set_read_timeout(Some(self.config.blocking_timeout))
            .map_err(XilError::from)?;

        let mut buf = [0_u8; 128];
        let (n, _) = self.rx_socket.recv_from(&mut buf).map_err(XilError::from)?;
        let slice = buf.get(..n).unwrap_or(&[]);
        BetaflightServoPacket::from_le_bytes(slice).map_err(Into::into)
    }

    fn post(&mut self, at: SimInstant, stimulus: &Self::Stimulus) -> Result<(), EndpointError> {
        self.send_fdm(at, stimulus)?;
        Ok(())
    }

    fn try_receive(&mut self) -> Result<Option<StampedResponse<Self::Response>>, EndpointError> {
        self.rx_socket
            .set_nonblocking(true)
            .map_err(XilError::from)?;
        let mut newest: Option<BetaflightServoPacket> = None;
        let mut buf = [0_u8; 128];
        loop {
            match self.rx_socket.recv_from(&mut buf) {
                Ok((n, _)) => {
                    let slice = buf.get(..n).unwrap_or(&[]);
                    newest = Some(BetaflightServoPacket::from_le_bytes(slice)?);
                }
                Err(err) if err.kind() == ErrorKind::WouldBlock => break,
                Err(err) => return Err(XilError::from(err).into()),
            }
        }
        Ok(newest.map(|response| StampedResponse {
            deliver_at: self.last_posted_at,
            response,
        }))
    }
}

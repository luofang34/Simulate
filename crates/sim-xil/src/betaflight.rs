//! Betaflight SITL (`fdm_packet` / `pwm_pkt`) and MultiWii Serial Protocol
//! (`MSP` v1 and v2) sans-IO codecs and native UDP [`sim_core::ExternalEndpoint`].
//!
//! Covers Betaflight SITL's UDP binary interface (`9003` FDM input, `9002`
//! motor PWM output) as well as MSP v1 (`$M`) and MSP v2 (`$X`, CRC-8 DVB-S2
//! polynomial `0xD5`) for RC channel injection and telemetry inspection on
//! SITL or physical UART-attached Betaflight/iNav controllers.

#[cfg(not(target_arch = "wasm32"))]
mod endpoint;
mod fdm;
mod msp;
#[cfg(test)]
mod tests;

#[cfg(not(target_arch = "wasm32"))]
pub use endpoint::{
    BETAFLIGHT_FDM_PORT, BETAFLIGHT_SERVO_PORT, BetaflightUdpConfig, BetaflightUdpEndpoint,
};
pub use fdm::{
    BETAFLIGHT_FDM_PACKET_BYTES, BETAFLIGHT_PACKET_TIME_STEP_S, BETAFLIGHT_SERVO_MIN_BYTES,
    BetaflightFdmPacket, BetaflightPacketClock, BetaflightServoPacket,
};
pub use msp::{
    MSP_ALTITUDE, MSP_ATTITUDE, MSP_RAW_IMU, MSP_SET_RAW_RC, MSP_STATUS, MspAltitude, MspAttitude,
    MspDirection, MspFrame, MspRawImu, MspRawRc, MspStatus, MspStreamDecoder, MspVersion,
    crc8_dvb_s2, encode_msp_v1, encode_msp_v2,
};

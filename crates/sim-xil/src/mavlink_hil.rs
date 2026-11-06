//! MAVLink v2 (`0xFD`) Hardware-in-the-Loop (`HIL_*`) sans-IO wire codec and
//! native TCP/UDP [`sim_core::ExternalEndpoint`] transports.
//!
//! Speaks standard `common.xml` `HEARTBEAT`, `HIL_SENSOR`, `HIL_GPS`,
//! `HIL_STATE_QUATERNION`, and `HIL_ACTUATOR_CONTROLS` so any MAVLink HIL
//! flight controller (PX4 SITL, ArduPilot SITL, external Aviate SITL, or a
//! physical autopilot over a TCP/UDP-to-UART bridge) connects without
//! simulator-side controller logic. Peer `time_usec` stamps are mapped onto
//! the master clock by [`PeerClockSynchronizer`], and replies are correlated
//! with the sensor frames that provoked them by [`HilCorrelationTracker`].

mod clock_sync;
mod crc;
#[cfg(not(target_arch = "wasm32"))]
mod endpoint;
mod messages;
mod reader;
#[cfg(test)]
mod tests;

pub use clock_sync::{
    ActuatorVerdict, DEFAULT_CLOCK_RESYNC_THRESHOLD, DEFAULT_MAX_REPLY_AGE, HilCorrelationTracker,
    HilLinkStats, PeerClockSynchronizer,
};
pub use crc::{
    CRC_EXTRA_HEARTBEAT, CRC_EXTRA_HIL_ACTUATOR_CONTROLS, CRC_EXTRA_HIL_GPS, CRC_EXTRA_HIL_SENSOR,
    CRC_EXTRA_HIL_STATE_QUATERNION, HEARTBEAT_MSG_ID, HIL_ACTUATOR_CONTROLS_MSG_ID, HIL_GPS_MSG_ID,
    HIL_SENSOR_MSG_ID, HIL_STATE_QUATERNION_MSG_ID, MAVLINK_V2_HEADER_LEN, MAVLINK_V2_MAGIC,
    crc_extra_for, encode_v2_frame, x25_crc,
};
#[cfg(not(target_arch = "wasm32"))]
pub use endpoint::{MavlinkHilConfig, MavlinkHilTcpEndpoint, MavlinkHilUdpEndpoint};
pub use messages::{
    HIL_SENSOR_FIELDS_ALL, HIL_SENSOR_FIELDS_BARO, HIL_SENSOR_FIELDS_DIFF_PRESSURE,
    HIL_SENSOR_FIELDS_IMU, HIL_SENSOR_FIELDS_MAG, HilActuatorControls, HilControlMapping,
    HilGpsSample, HilSensorSample, HilStateQuaternionSample, MAV_MODE_FLAG_SAFETY_ARMED,
    SIM_COMPONENT_ID, SIM_SYSTEM_ID, decode_hil_actuator_controls, encode_heartbeat,
    encode_hil_actuator_controls, encode_hil_gps, encode_hil_sensor, encode_hil_state_quaternion,
};
pub use reader::{MavlinkHilFrame, MavlinkHilReader};

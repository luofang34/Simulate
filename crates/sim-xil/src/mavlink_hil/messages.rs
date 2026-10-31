use sim_atmosphere::geometric_to_geopotential_m;
use sim_core::ControlInputs;

use crate::mavlink_hil::crc::{
    CRC_EXTRA_HEARTBEAT, CRC_EXTRA_HIL_ACTUATOR_CONTROLS, CRC_EXTRA_HIL_GPS, CRC_EXTRA_HIL_SENSOR,
    CRC_EXTRA_HIL_STATE_QUATERNION, HEARTBEAT_MSG_ID, HIL_ACTUATOR_CONTROLS_MSG_ID, HIL_GPS_MSG_ID,
    HIL_SENSOR_MSG_ID, HIL_STATE_QUATERNION_MSG_ID, encode_v2_frame,
};
use crate::{STANDARD_GRAVITY_M_S2, XilError, XilStimulus};

/// Default MAVLink system ID used by the simulator HIL host.
pub const SIM_SYSTEM_ID: u8 = 1;
/// Default MAVLink component ID used by the simulator HIL host.
pub const SIM_COMPONENT_ID: u8 = 195;

/// `MAV_MODE_FLAG_SAFETY_ARMED` bit in `HEARTBEAT.base_mode` and
/// `HIL_ACTUATOR_CONTROLS.mode`.
pub const MAV_MODE_FLAG_SAFETY_ARMED: u8 = 0x80;

/// `HIL_SENSOR.fields_updated` bitmask for 3-axis accel + 3-axis gyro (`0..=5`).
pub const HIL_SENSOR_FIELDS_IMU: u32 = 0x003F;
/// `HIL_SENSOR.fields_updated` bitmask for 3-axis magnetometer (`6..=8`).
pub const HIL_SENSOR_FIELDS_MAG: u32 = 0x01C0;
/// `HIL_SENSOR.fields_updated` bitmask for differential pressure (`10`).
pub const HIL_SENSOR_FIELDS_DIFF_PRESSURE: u32 = 0x0400;
/// `HIL_SENSOR.fields_updated` bitmask for static pressure, pressure alt, temp (`9, 11, 12`).
pub const HIL_SENSOR_FIELDS_BARO: u32 = 0x1A00;
/// `HIL_SENSOR.fields_updated` bitmask when IMU, mag, baro, and pitot all update.
pub const HIL_SENSOR_FIELDS_ALL: u32 = HIL_SENSOR_FIELDS_IMU
    | HIL_SENSOR_FIELDS_MAG
    | HIL_SENSOR_FIELDS_DIFF_PRESSURE
    | HIL_SENSOR_FIELDS_BARO;

const KELVIN_OFFSET: f64 = 273.15;
const SEA_LEVEL_PRESSURE_PA: f64 = 101_325.0;

/// Decoded `HIL_ACTUATOR_CONTROLS` (`#93`) command from a flight controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HilActuatorControls {
    /// Timestamp stamped by the flight controller (`µs`).
    pub time_usec: u64,
    /// Normalized actuator outputs (`-1..1` for surfaces, `0..1` for motors/throttles).
    pub controls: [f32; 16],
    /// System mode (`MAV_MODE_FLAG_*`), including [`MAV_MODE_FLAG_SAFETY_ARMED`].
    pub mode: u8,
    /// Actuator bitflags (`u64`).
    pub flags: u64,
}

impl HilActuatorControls {
    /// Whether [`MAV_MODE_FLAG_SAFETY_ARMED`] is asserted in `mode`.
    #[must_use]
    pub const fn is_armed(&self) -> bool {
        (self.mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0
    }

    /// Projects standard fixed-wing channels onto [`ControlInputs`]. When
    /// disarmed and `zero_throttle_when_disarmed` is set, throttle is clamped
    /// to `0.0`.
    #[must_use]
    pub fn to_control_inputs(&self, mapping: &HilControlMapping) -> ControlInputs {
        let ch = |idx: usize| self.controls.get(idx).copied().map_or(0.0, f64::from);
        let raw_throttle = if mapping.throttle_bipolar {
            0.5 * (ch(mapping.throttle_channel) + 1.0)
        } else {
            ch(mapping.throttle_channel)
        };
        let throttle = if !self.is_armed() && mapping.zero_throttle_when_disarmed {
            0.0
        } else {
            raw_throttle.clamp(0.0, 1.0)
        };
        ControlInputs {
            roll: ch(mapping.aileron_channel),
            pitch: ch(mapping.elevator_channel),
            yaw: ch(mapping.rudder_channel),
            throttle,
        }
        .clamped()
    }
}

/// Channel assignment mapping `HIL_ACTUATOR_CONTROLS.controls` to fixed-wing
/// [`ControlInputs`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HilControlMapping {
    /// Index of roll/aileron command (`-1..1`).
    pub aileron_channel: usize,
    /// Index of pitch/elevator command (`-1..1`).
    pub elevator_channel: usize,
    /// Index of yaw/rudder command (`-1..1`).
    pub rudder_channel: usize,
    /// Index of throttle command (`0..1` or `-1..1` when `throttle_bipolar`).
    pub throttle_channel: usize,
    /// Whether throttle arrives on `[-1, 1]` instead of `[0, 1]`.
    pub throttle_bipolar: bool,
    /// Clamps throttle to `0` while `MAV_MODE_FLAG_SAFETY_ARMED` is clear.
    pub zero_throttle_when_disarmed: bool,
}

impl Default for HilControlMapping {
    /// Standard PX4 / MAVLink fixed-wing HIL actuator assignment:
    /// `[0] = aileron`, `[1] = elevator`, `[2] = rudder`, `[3] = throttle (0..1)`.
    fn default() -> Self {
        Self {
            aileron_channel: 0,
            elevator_channel: 1,
            rudder_channel: 2,
            throttle_channel: 3,
            throttle_bipolar: false,
            zero_throttle_when_disarmed: true,
        }
    }
}

/// Typed `HIL_SENSOR` (`#107`) payload fields.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HilSensorSample {
    /// Simulation timestamp (`µs`).
    pub time_usec: u64,
    /// Body FRD specific force (`m/s²`).
    pub accel_frd_m_s2: [f32; 3],
    /// Body FRD angular velocity (`rad/s`).
    pub gyro_frd_rad_s: [f32; 3],
    /// Body FRD magnetic field (`Gauss`).
    pub mag_frd_gauss: [f32; 3],
    /// Static pressure (`hPa`).
    pub abs_pressure_hpa: f32,
    /// Differential pitot pressure (`hPa`).
    pub diff_pressure_hpa: f32,
    /// Barometric pressure altitude (`m`).
    pub pressure_alt_m: f32,
    /// Ambient temperature (`°C`).
    pub temperature_deg_c: f32,
    /// Updated sensor bitmask (`HIL_SENSOR_FIELDS_*`).
    pub fields_updated: u32,
}

/// Typed `HIL_GPS` (`#113`) payload fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HilGpsSample {
    /// Simulation timestamp (`µs`).
    pub time_usec: u64,
    /// Geodetic latitude (`deg * 1e7`).
    pub lat_deg_e7: i32,
    /// Geodetic longitude (`deg * 1e7`).
    pub lon_deg_e7: i32,
    /// Altitude (`mm`).
    pub alt_mm: i32,
    /// Horizontal dilution / accuracy (`cm`).
    pub eph_cm: u16,
    /// Vertical dilution / accuracy (`cm`).
    pub epv_cm: u16,
    /// Ground speed (`cm/s`).
    pub vel_cm_s: u16,
    /// NED velocity components (`[vn, ve, vd]` in `cm/s`).
    pub vel_ned_cm_s: [i16; 3],
    /// Course over ground (`cdeg`, `0..35999`, or `u16::MAX` when stationary).
    pub cog_cdeg: u16,
    /// GNSS fix type (`3` = 3D fix).
    pub fix_type: u8,
    /// Visible satellite count.
    pub satellites_visible: u8,
}

/// Typed `HIL_STATE_QUATERNION` (`#115`) ground-truth payload fields.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HilStateQuaternionSample {
    /// Simulation timestamp (`µs`).
    pub time_usec: u64,
    /// Body FRD to local NED unit quaternion (`[w, x, y, z]`).
    pub attitude_quat_wxyz: [f32; 4],
    /// Body FRD angular velocity (`rad/s`).
    pub body_rates_rad_s: [f32; 3],
    /// Geodetic latitude (`deg * 1e7`).
    pub lat_deg_e7: i32,
    /// Geodetic longitude (`deg * 1e7`).
    pub lon_deg_e7: i32,
    /// Altitude (`mm`).
    pub alt_mm: i32,
    /// True NED velocity (`cm/s`).
    pub vel_ned_cm_s: [i16; 3],
    /// Indicated airspeed (`cm/s`).
    pub ind_airspeed_cm_s: u16,
    /// True airspeed (`cm/s`).
    pub true_airspeed_cm_s: u16,
    /// Body FRD coordinate linear acceleration (`mG`).
    pub linear_accel_mg: [i16; 3],
}

impl HilSensorSample {
    /// Derives a `HIL_SENSOR` sample from `stimulus` at `time_usec`.
    pub fn from_stimulus(
        time_usec: u64,
        stimulus: &XilStimulus,
        fields_updated: u32,
    ) -> Result<Self, XilError> {
        let _ = stimulus.ned_kinematics()?;
        let pressure_ratio = (stimulus.static_pressure_pa / SEA_LEVEL_PRESSURE_PA).max(1.0e-9);
        let pressure_alt_m = 44_330.0 * (1.0 - pressure_ratio.powf(0.190_294_95));
        let f = stimulus.specific_force_body_m_s2;
        let w = stimulus.vehicle.angular_rate_body;
        let m = stimulus.mag_field_body_gauss;
        Ok(Self {
            time_usec,
            accel_frd_m_s2: [f.x as f32, f.y as f32, f.z as f32],
            gyro_frd_rad_s: [w.x as f32, w.y as f32, w.z as f32],
            mag_frd_gauss: [m.x as f32, m.y as f32, m.z as f32],
            abs_pressure_hpa: (stimulus.static_pressure_pa * 0.01) as f32,
            diff_pressure_hpa: (stimulus.differential_pressure_pa * 0.01) as f32,
            pressure_alt_m: geometric_to_geopotential_m(pressure_alt_m) as f32,
            temperature_deg_c: (stimulus.temperature_k - KELVIN_OFFSET) as f32,
            fields_updated,
        })
    }
}

impl HilGpsSample {
    /// Derives an ideal 3D-fix `HIL_GPS` sample from `stimulus` at `time_usec`.
    pub fn from_stimulus(time_usec: u64, stimulus: &XilStimulus) -> Result<Self, XilError> {
        let ned = stimulus.ned_kinematics()?;
        let vn = clamp_i16(ned.velocity_ned_m_s.x * 100.0);
        let ve = clamp_i16(ned.velocity_ned_m_s.y * 100.0);
        let vd = clamp_i16(ned.velocity_ned_m_s.z * 100.0);
        let ground_m_s = ned.velocity_ned_m_s.truncate().length();
        let cog_cdeg = if ground_m_s > 0.05 {
            let deg = ned
                .velocity_ned_m_s
                .y
                .atan2(ned.velocity_ned_m_s.x)
                .to_degrees();
            let wrapped = deg.rem_euclid(360.0);
            clamp_u16((wrapped * 100.0).round().min(35_999.0))
        } else {
            u16::MAX
        };
        Ok(Self {
            time_usec,
            lat_deg_e7: clamp_i32(ned.geodetic.lat_rad.to_degrees() * 1.0e7),
            lon_deg_e7: clamp_i32(ned.geodetic.lon_rad.to_degrees() * 1.0e7),
            alt_mm: clamp_i32(ned.geodetic.height_m * 1_000.0),
            eph_cm: 30,
            epv_cm: 50,
            vel_cm_s: clamp_u16(ground_m_s * 100.0),
            vel_ned_cm_s: [vn, ve, vd],
            cog_cdeg,
            fix_type: 3,
            satellites_visible: 12,
        })
    }
}

impl HilStateQuaternionSample {
    /// Derives a `HIL_STATE_QUATERNION` sample from `stimulus` at `time_usec`.
    pub fn from_stimulus(time_usec: u64, stimulus: &XilStimulus) -> Result<Self, XilError> {
        let ned = stimulus.ned_kinematics()?;
        let q = ned.attitude_frd_to_ned;
        let w = stimulus.vehicle.angular_rate_body;
        let a = stimulus.linear_accel_body_m_s2;
        let mg_scale = 1_000.0 / STANDARD_GRAVITY_M_S2;
        Ok(Self {
            time_usec,
            attitude_quat_wxyz: [q.w as f32, q.x as f32, q.y as f32, q.z as f32],
            body_rates_rad_s: [w.x as f32, w.y as f32, w.z as f32],
            lat_deg_e7: clamp_i32(ned.geodetic.lat_rad.to_degrees() * 1.0e7),
            lon_deg_e7: clamp_i32(ned.geodetic.lon_rad.to_degrees() * 1.0e7),
            alt_mm: clamp_i32(ned.geodetic.height_m * 1_000.0),
            vel_ned_cm_s: [
                clamp_i16(ned.velocity_ned_m_s.x * 100.0),
                clamp_i16(ned.velocity_ned_m_s.y * 100.0),
                clamp_i16(ned.velocity_ned_m_s.z * 100.0),
            ],
            ind_airspeed_cm_s: clamp_u16(stimulus.airspeed_indicated_m_s.max(0.0) * 100.0),
            true_airspeed_cm_s: clamp_u16(stimulus.airspeed_true_m_s.max(0.0) * 100.0),
            linear_accel_mg: [
                clamp_i16(a.x * mg_scale),
                clamp_i16(a.y * mg_scale),
                clamp_i16(a.z * mg_scale),
            ],
        })
    }
}

/// Encodes a MAVLink v2 `HEARTBEAT` (`#0`) frame into `out`.
pub fn encode_heartbeat(
    seq: u8,
    system_id: u8,
    component_id: u8,
    armed: bool,
    out: &mut [u8],
) -> Result<usize, XilError> {
    let base_mode = if armed { MAV_MODE_FLAG_SAFETY_ARMED } else { 0 };
    let payload = [0, 0, 0, 0, 6, 8, base_mode, 4, 3];
    encode_v2_frame(
        seq,
        system_id,
        component_id,
        HEARTBEAT_MSG_ID,
        CRC_EXTRA_HEARTBEAT,
        &payload,
        out,
    )
}

/// Encodes a MAVLink v2 `HIL_SENSOR` (`#107`) frame into `out`.
pub fn encode_hil_sensor(
    seq: u8,
    system_id: u8,
    component_id: u8,
    sample: &HilSensorSample,
    out: &mut [u8],
) -> Result<usize, XilError> {
    let mut payload = [0_u8; 64];
    put_bytes(&mut payload, 0, &sample.time_usec.to_le_bytes());
    let [ax, ay, az] = sample.accel_frd_m_s2;
    let [gx, gy, gz] = sample.gyro_frd_rad_s;
    let [mx, my, mz] = sample.mag_frd_gauss;
    let floats = [
        ax,
        ay,
        az,
        gx,
        gy,
        gz,
        mx,
        my,
        mz,
        sample.abs_pressure_hpa,
        sample.diff_pressure_hpa,
        sample.pressure_alt_m,
        sample.temperature_deg_c,
    ];
    for (idx, value) in floats.into_iter().enumerate() {
        put_bytes(&mut payload, 8 + idx * 4, &value.to_le_bytes());
    }
    put_bytes(&mut payload, 60, &sample.fields_updated.to_le_bytes());
    encode_v2_frame(
        seq,
        system_id,
        component_id,
        HIL_SENSOR_MSG_ID,
        CRC_EXTRA_HIL_SENSOR,
        &payload,
        out,
    )
}

/// Encodes a MAVLink v2 `HIL_GPS` (`#113`) frame into `out`.
pub fn encode_hil_gps(
    seq: u8,
    system_id: u8,
    component_id: u8,
    sample: &HilGpsSample,
    out: &mut [u8],
) -> Result<usize, XilError> {
    let mut payload = [0_u8; 36];
    put_bytes(&mut payload, 0, &sample.time_usec.to_le_bytes());
    put_bytes(&mut payload, 8, &sample.lat_deg_e7.to_le_bytes());
    put_bytes(&mut payload, 12, &sample.lon_deg_e7.to_le_bytes());
    put_bytes(&mut payload, 16, &sample.alt_mm.to_le_bytes());
    put_bytes(&mut payload, 20, &sample.eph_cm.to_le_bytes());
    put_bytes(&mut payload, 22, &sample.epv_cm.to_le_bytes());
    let [vn, ve, vd] = sample.vel_ned_cm_s;
    put_bytes(&mut payload, 24, &sample.vel_cm_s.to_le_bytes());
    put_bytes(&mut payload, 26, &vn.to_le_bytes());
    put_bytes(&mut payload, 28, &ve.to_le_bytes());
    put_bytes(&mut payload, 30, &vd.to_le_bytes());
    put_bytes(&mut payload, 32, &sample.cog_cdeg.to_le_bytes());
    put_bytes(
        &mut payload,
        34,
        &[sample.fix_type, sample.satellites_visible],
    );
    encode_v2_frame(
        seq,
        system_id,
        component_id,
        HIL_GPS_MSG_ID,
        CRC_EXTRA_HIL_GPS,
        &payload,
        out,
    )
}

/// Encodes a MAVLink v2 `HIL_STATE_QUATERNION` (`#115`) frame into `out`.
pub fn encode_hil_state_quaternion(
    seq: u8,
    system_id: u8,
    component_id: u8,
    sample: &HilStateQuaternionSample,
    out: &mut [u8],
) -> Result<usize, XilError> {
    let mut payload = [0_u8; 64];
    put_bytes(&mut payload, 0, &sample.time_usec.to_le_bytes());
    for (idx, q) in sample.attitude_quat_wxyz.into_iter().enumerate() {
        put_bytes(&mut payload, 8 + idx * 4, &q.to_le_bytes());
    }
    for (idx, rate) in sample.body_rates_rad_s.into_iter().enumerate() {
        put_bytes(&mut payload, 24 + idx * 4, &rate.to_le_bytes());
    }
    put_bytes(&mut payload, 36, &sample.lat_deg_e7.to_le_bytes());
    put_bytes(&mut payload, 40, &sample.lon_deg_e7.to_le_bytes());
    put_bytes(&mut payload, 44, &sample.alt_mm.to_le_bytes());
    for (idx, v) in sample.vel_ned_cm_s.into_iter().enumerate() {
        put_bytes(&mut payload, 48 + idx * 2, &v.to_le_bytes());
    }
    put_bytes(&mut payload, 54, &sample.ind_airspeed_cm_s.to_le_bytes());
    put_bytes(&mut payload, 56, &sample.true_airspeed_cm_s.to_le_bytes());
    for (idx, acc) in sample.linear_accel_mg.into_iter().enumerate() {
        put_bytes(&mut payload, 58 + idx * 2, &acc.to_le_bytes());
    }
    encode_v2_frame(
        seq,
        system_id,
        component_id,
        HIL_STATE_QUATERNION_MSG_ID,
        CRC_EXTRA_HIL_STATE_QUATERNION,
        &payload,
        out,
    )
}

/// Encodes a MAVLink v2 `HIL_ACTUATOR_CONTROLS` (`#93`) frame into `out`.
pub fn encode_hil_actuator_controls(
    seq: u8,
    system_id: u8,
    component_id: u8,
    controls: &HilActuatorControls,
    out: &mut [u8],
) -> Result<usize, XilError> {
    let mut payload = [0_u8; 81];
    put_bytes(&mut payload, 0, &controls.time_usec.to_le_bytes());
    put_bytes(&mut payload, 8, &controls.flags.to_le_bytes());
    for (idx, value) in controls.controls.into_iter().enumerate() {
        put_bytes(&mut payload, 16 + idx * 4, &value.to_le_bytes());
    }
    put_bytes(&mut payload, 80, &[controls.mode]);
    encode_v2_frame(
        seq,
        system_id,
        component_id,
        HIL_ACTUATOR_CONTROLS_MSG_ID,
        CRC_EXTRA_HIL_ACTUATOR_CONTROLS,
        &payload,
        out,
    )
}

/// Decodes a MAVLink v2 `HIL_ACTUATOR_CONTROLS` (`#93`) payload with zero
/// extension of truncated trailing zero bytes.
#[must_use]
pub fn decode_hil_actuator_controls(payload: &[u8]) -> HilActuatorControls {
    let mut controls = [0.0_f32; 16];
    for (idx, slot) in controls.iter_mut().enumerate() {
        *slot = f32_at(payload, 16 + idx * 4);
    }
    HilActuatorControls {
        time_usec: u64_at(payload, 0),
        flags: u64_at(payload, 8),
        controls,
        mode: payload.get(80).copied().unwrap_or(0),
    }
}

fn put_bytes(buf: &mut [u8], offset: usize, src: &[u8]) {
    if let Some(dst) = buf.get_mut(offset..offset.saturating_add(src.len())) {
        dst.copy_from_slice(src);
    }
}

fn u64_at(payload: &[u8], off: usize) -> u64 {
    let mut bytes = [0_u8; 8];
    for (idx, slot) in bytes.iter_mut().enumerate() {
        *slot = payload.get(off.saturating_add(idx)).copied().unwrap_or(0);
    }
    u64::from_le_bytes(bytes)
}

fn f32_at(payload: &[u8], off: usize) -> f32 {
    let mut bytes = [0_u8; 4];
    for (idx, slot) in bytes.iter_mut().enumerate() {
        *slot = payload.get(off.saturating_add(idx)).copied().unwrap_or(0);
    }
    f32::from_le_bytes(bytes)
}

fn clamp_i32(val: f64) -> i32 {
    val.round().clamp(f64::from(i32::MIN), f64::from(i32::MAX)) as i32
}

fn clamp_i16(val: f64) -> i16 {
    val.round().clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16
}

fn clamp_u16(val: f64) -> u16 {
    val.round().clamp(0.0, f64::from(u16::MAX)) as u16
}

use sim_geo::BodyEllipsoid;

use crate::{XilError, XilStimulus};

/// Wire size of Betaflight SITL's `fdm_packet` (`18 * sizeof(double) = 144` bytes).
pub const BETAFLIGHT_FDM_PACKET_BYTES: usize = 18 * 8;

/// Minimum wire size of Betaflight SITL's 4-motor `servo_packet` (`4 * sizeof(float) = 16` bytes).
pub const BETAFLIGHT_SERVO_MIN_BYTES: usize = 4 * 4;

/// Nominal Betaflight SITL FDM packet time step (`40 Hz`, `0.025 s`).
pub const BETAFLIGHT_PACKET_TIME_STEP_S: f64 = 0.025;

/// Deterministic packet-timestamp generator advancing in steps of
/// [`BETAFLIGHT_PACKET_TIME_STEP_S`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BetaflightPacketClock {
    step: u64,
}

impl BetaflightPacketClock {
    /// Creates a packet clock starting at `0.0 s`.
    #[must_use]
    pub const fn new() -> Self {
        Self { step: 0 }
    }

    /// Returns the current packet timestamp (`s`) and advances by one step.
    pub const fn next_timestamp_s(&mut self) -> f64 {
        let timestamp = (self.step as f64) * BETAFLIGHT_PACKET_TIME_STEP_S;
        self.step = self.step.wrapping_add(1);
        timestamp
    }

    /// Number of timestamps issued so far.
    #[must_use]
    pub const fn step_count(&self) -> u64 {
        self.step
    }
}

/// Betaflight SITL `fdm_packet` (`144` bytes little-endian `f64`).
///
/// Frame conventions match `betaflight/src/main/target/SITL/sitl.c`:
/// - `imu_angular_velocity_rpy`: body FRD `[p, q, r]` (`rad/s`).
/// - `imu_linear_acceleration_xyz`: negated FLU specific force (`m/s²`), i.e.
///   `[0, 0, -9.80665]` when resting level on Earth.
/// - `imu_orientation_quat`: FLU-to-NWU Hamilton quaternion `[w, x, y, z]`.
/// - `velocity_xyz`: local ENU `[v_east, v_north, v_up]` (`m/s`).
/// - `position_llh`: `[longitude_deg, latitude_deg, altitude_m]`.
/// - `pressure_pa`: ambient static pressure (`Pa`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BetaflightFdmPacket {
    /// Monotonic simulation timestamp (`s`).
    pub timestamp_s: f64,
    /// Body FRD angular velocity `[roll, pitch, yaw]` (`rad/s`) relative to
    /// inertial space, as the gyroscopes sense it.
    pub imu_angular_velocity_rpy: [f64; 3],
    /// Negated FLU specific force `[x, y, z]` (`m/s²`).
    pub imu_linear_acceleration_xyz: [f64; 3],
    /// FLU-to-NWU Hamilton quaternion `[w, x, y, z]`.
    pub imu_orientation_quat: [f64; 4],
    /// Local ENU velocity `[east, north, up]` (`m/s`).
    pub velocity_xyz: [f64; 3],
    /// Geodetic `[longitude_deg, latitude_deg, altitude_m]`.
    pub position_llh: [f64; 3],
    /// Static atmospheric pressure (`Pa`).
    pub pressure_pa: f64,
}

impl BetaflightFdmPacket {
    /// Projects `stimulus` into Betaflight SITL's wire frames at `timestamp_s`.
    pub fn from_stimulus(timestamp_s: f64, stimulus: &XilStimulus) -> Result<Self, XilError> {
        if !timestamp_s.is_finite() {
            return Err(XilError::InvalidStimulus {
                field: "timestamp_s",
            });
        }
        let ned = stimulus.ned_kinematics()?;
        let w = stimulus.vehicle.inertial_rate_body(&BodyEllipsoid::WGS84);
        let f = stimulus.specific_force_body_m_s2;
        let q = ned.attitude_frd_to_ned;
        let v_enu = ned.velocity_enu_m_s;

        Ok(Self {
            timestamp_s,
            imu_angular_velocity_rpy: [w.x, w.y, w.z],
            // FRD specific force `[fx, fy, fz]` -> FLU `[fx, -fy, -fz]` -> negated `[-fx, fy, fz]`.
            imu_linear_acceleration_xyz: [-f.x, f.y, f.z],
            // Conjugating `q_frd_to_ned` by 180 deg around +X maps FRD->NED to FLU->NWU.
            imu_orientation_quat: [q.w, q.x, -q.y, -q.z],
            velocity_xyz: [v_enu.x, v_enu.y, v_enu.z],
            position_llh: [
                ned.geodetic.lon_rad.to_degrees(),
                ned.geodetic.lat_rad.to_degrees(),
                ned.geodetic.height_m,
            ],
            pressure_pa: stimulus.static_pressure_pa,
        })
    }

    /// Serializes the packet into its 144-byte little-endian `f64` wire image.
    #[must_use]
    pub fn to_le_bytes(&self) -> [u8; BETAFLIGHT_FDM_PACKET_BYTES] {
        let [wx, wy, wz] = self.imu_angular_velocity_rpy;
        let [ax, ay, az] = self.imu_linear_acceleration_xyz;
        let [qw, qx, qy, qz] = self.imu_orientation_quat;
        let [vx, vy, vz] = self.velocity_xyz;
        let [lon, lat, alt] = self.position_llh;
        let scalars = [
            self.timestamp_s,
            wx,
            wy,
            wz,
            ax,
            ay,
            az,
            qw,
            qx,
            qy,
            qz,
            vx,
            vy,
            vz,
            lon,
            lat,
            alt,
            self.pressure_pa,
        ];
        let mut out = [0_u8; BETAFLIGHT_FDM_PACKET_BYTES];
        for (idx, val) in scalars.into_iter().enumerate() {
            if let Some(dst) = out.get_mut(idx * 8..(idx + 1) * 8) {
                dst.copy_from_slice(&val.to_le_bytes());
            }
        }
        out
    }

    /// Parses a 144-byte little-endian `fdm_packet`.
    pub fn from_le_bytes(bytes: &[u8]) -> Result<Self, XilError> {
        if bytes.len() != BETAFLIGHT_FDM_PACKET_BYTES {
            return Err(XilError::InvalidPacketLength {
                protocol: "Betaflight FdmPacket",
                expected: BETAFLIGHT_FDM_PACKET_BYTES,
                actual: bytes.len(),
            });
        }
        Ok(Self {
            timestamp_s: f64_at(bytes, 0),
            imu_angular_velocity_rpy: [f64_at(bytes, 8), f64_at(bytes, 16), f64_at(bytes, 24)],
            imu_linear_acceleration_xyz: [f64_at(bytes, 32), f64_at(bytes, 40), f64_at(bytes, 48)],
            imu_orientation_quat: [
                f64_at(bytes, 56),
                f64_at(bytes, 64),
                f64_at(bytes, 72),
                f64_at(bytes, 80),
            ],
            velocity_xyz: [f64_at(bytes, 88), f64_at(bytes, 96), f64_at(bytes, 104)],
            position_llh: [f64_at(bytes, 112), f64_at(bytes, 120), f64_at(bytes, 128)],
            pressure_pa: f64_at(bytes, 136),
        })
    }
}

/// Decoded Betaflight SITL motor command packet (`pwm_pkt` / `servo_pkt`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BetaflightServoPacket {
    /// Primary quadrotor normalized motor setpoints (`[0.0, 1.0]`).
    pub motors: [f32; 4],
    /// Total number of `f32` channels present on the wire (`4..=16`).
    pub motor_count: usize,
    /// Full up-to-16 motor array for hexacopter/octocopter SITL rigs.
    pub all_motors: [f32; 16],
}

impl BetaflightServoPacket {
    /// Encodes the first 4 motor throttles as a 16-byte little-endian packet.
    #[must_use]
    pub fn to_quad_bytes(&self) -> [u8; BETAFLIGHT_SERVO_MIN_BYTES] {
        let mut out = [0_u8; BETAFLIGHT_SERVO_MIN_BYTES];
        for (idx, val) in self.motors.into_iter().enumerate() {
            if let Some(dst) = out.get_mut(idx * 4..(idx + 1) * 4) {
                dst.copy_from_slice(&val.to_le_bytes());
            }
        }
        out
    }

    /// Decodes a Betaflight SITL motor packet (`16..=64` bytes, multiple of `4`).
    pub fn from_le_bytes(bytes: &[u8]) -> Result<Self, XilError> {
        if bytes.len() < BETAFLIGHT_SERVO_MIN_BYTES
            || bytes.len() > 64
            || !bytes.len().is_multiple_of(4)
        {
            return Err(XilError::InvalidPacketLength {
                protocol: "Betaflight ServoPacket",
                expected: BETAFLIGHT_SERVO_MIN_BYTES,
                actual: bytes.len(),
            });
        }
        let count = bytes.len() / 4;
        let mut all_motors = [0.0_f32; 16];
        for (idx, slot) in all_motors.iter_mut().take(count).enumerate() {
            let v = f32_at(bytes, idx * 4);
            if !v.is_finite() {
                return Err(XilError::ProtocolViolation {
                    protocol: "Betaflight ServoPacket",
                    detail: format!("non-finite motor output at channel {idx}"),
                });
            }
            *slot = v.clamp(0.0, 1.0);
        }
        let [m0, m1, m2, m3, ..] = all_motors;
        Ok(Self {
            motors: [m0, m1, m2, m3],
            motor_count: count,
            all_motors,
        })
    }
}

fn f64_at(bytes: &[u8], off: usize) -> f64 {
    let mut raw = [0_u8; 8];
    for (idx, slot) in raw.iter_mut().enumerate() {
        *slot = bytes.get(off.saturating_add(idx)).copied().unwrap_or(0);
    }
    f64::from_le_bytes(raw)
}

fn f32_at(bytes: &[u8], off: usize) -> f32 {
    let mut raw = [0_u8; 4];
    for (idx, slot) in raw.iter_mut().enumerate() {
        *slot = bytes.get(off.saturating_add(idx)).copied().unwrap_or(0);
    }
    f32::from_le_bytes(raw)
}

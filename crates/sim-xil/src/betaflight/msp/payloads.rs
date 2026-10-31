use crate::XilError;

/// Typed `MSP_STATUS` (`101`, `11` base bytes) payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspStatus {
    /// Loop cycle time (`µs`).
    pub cycle_time_us: u16,
    /// I2C error counter.
    pub i2c_errors: u16,
    /// Present sensor bitmask (`ACC=1, BARO=2, MAG=4, GPS=8`).
    pub active_sensors: u16,
    /// Active flight-mode box flags.
    pub flight_mode_flags: u32,
    /// Active PID profile index.
    pub current_pid_profile: u8,
}

impl MspStatus {
    /// Encodes the 11-byte `MSP_STATUS` base payload.
    #[must_use]
    pub fn to_bytes(&self) -> [u8; 11] {
        let [c0, c1] = self.cycle_time_us.to_le_bytes();
        let [e0, e1] = self.i2c_errors.to_le_bytes();
        let [s0, s1] = self.active_sensors.to_le_bytes();
        let [f0, f1, f2, f3] = self.flight_mode_flags.to_le_bytes();
        [
            c0,
            c1,
            e0,
            e1,
            s0,
            s1,
            f0,
            f1,
            f2,
            f3,
            self.current_pid_profile,
        ]
    }

    /// Decodes an `MSP_STATUS` payload (accepts extended Betaflight payloads `>= 11` bytes).
    pub fn from_bytes(payload: &[u8]) -> Result<Self, XilError> {
        if payload.len() < 11 {
            return Err(XilError::InvalidPacketLength {
                protocol: "MSP_STATUS",
                expected: 11,
                actual: payload.len(),
            });
        }
        Ok(Self {
            cycle_time_us: u16_at(payload, 0),
            i2c_errors: u16_at(payload, 2),
            active_sensors: u16_at(payload, 4),
            flight_mode_flags: u32_at(payload, 6),
            current_pid_profile: payload.get(10).copied().unwrap_or(0),
        })
    }
}

/// Typed `MSP_RAW_IMU` (`102`, `18` bytes) payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspRawImu {
    /// Raw 3-axis accelerometer counts.
    pub acc: [i16; 3],
    /// Raw 3-axis gyroscope counts.
    pub gyro: [i16; 3],
    /// Raw 3-axis magnetometer counts.
    pub mag: [i16; 3],
}

impl MspRawImu {
    /// Encodes the 18-byte `MSP_RAW_IMU` payload.
    #[must_use]
    pub fn to_bytes(&self) -> [u8; 18] {
        let [ax, ay, az] = self.acc;
        let [gx, gy, gz] = self.gyro;
        let [mx, my, mz] = self.mag;
        let words = [ax, ay, az, gx, gy, gz, mx, my, mz];
        let mut out = [0_u8; 18];
        for (idx, w) in words.into_iter().enumerate() {
            if let Some(dst) = out.get_mut(idx * 2..(idx + 1) * 2) {
                dst.copy_from_slice(&w.to_le_bytes());
            }
        }
        out
    }

    /// Decodes an 18-byte `MSP_RAW_IMU` payload.
    pub fn from_bytes(payload: &[u8]) -> Result<Self, XilError> {
        if payload.len() < 18 {
            return Err(XilError::InvalidPacketLength {
                protocol: "MSP_RAW_IMU",
                expected: 18,
                actual: payload.len(),
            });
        }
        Ok(Self {
            acc: [i16_at(payload, 0), i16_at(payload, 2), i16_at(payload, 4)],
            gyro: [i16_at(payload, 6), i16_at(payload, 8), i16_at(payload, 10)],
            mag: [
                i16_at(payload, 12),
                i16_at(payload, 14),
                i16_at(payload, 16),
            ],
        })
    }
}

/// Typed `MSP_ATTITUDE` (`108`, `6` bytes) payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspAttitude {
    /// Roll angle in decidegrees (`0.1 deg`).
    pub roll_decideg: i16,
    /// Pitch angle in decidegrees (`0.1 deg`).
    pub pitch_decideg: i16,
    /// Heading in degrees (`0..360`).
    pub yaw_deg: i16,
}

impl MspAttitude {
    /// Encodes the 6-byte `MSP_ATTITUDE` payload.
    #[must_use]
    pub fn to_bytes(&self) -> [u8; 6] {
        let [r0, r1] = self.roll_decideg.to_le_bytes();
        let [p0, p1] = self.pitch_decideg.to_le_bytes();
        let [y0, y1] = self.yaw_deg.to_le_bytes();
        [r0, r1, p0, p1, y0, y1]
    }

    /// Decodes a 6-byte `MSP_ATTITUDE` payload.
    pub fn from_bytes(payload: &[u8]) -> Result<Self, XilError> {
        if payload.len() < 6 {
            return Err(XilError::InvalidPacketLength {
                protocol: "MSP_ATTITUDE",
                expected: 6,
                actual: payload.len(),
            });
        }
        Ok(Self {
            roll_decideg: i16_at(payload, 0),
            pitch_decideg: i16_at(payload, 2),
            yaw_deg: i16_at(payload, 4),
        })
    }
}

/// Typed `MSP_ALTITUDE` (`109`, `6` bytes) payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspAltitude {
    /// Estimated altitude (`cm`).
    pub estimated_alt_cm: i32,
    /// Estimated vertical climb rate (`cm/s`).
    pub vario_cm_s: i16,
}

impl MspAltitude {
    /// Encodes the 6-byte `MSP_ALTITUDE` payload.
    #[must_use]
    pub fn to_bytes(&self) -> [u8; 6] {
        let [a0, a1, a2, a3] = self.estimated_alt_cm.to_le_bytes();
        let [v0, v1] = self.vario_cm_s.to_le_bytes();
        [a0, a1, a2, a3, v0, v1]
    }

    /// Decodes a 6-byte `MSP_ALTITUDE` payload.
    pub fn from_bytes(payload: &[u8]) -> Result<Self, XilError> {
        if payload.len() < 6 {
            return Err(XilError::InvalidPacketLength {
                protocol: "MSP_ALTITUDE",
                expected: 6,
                actual: payload.len(),
            });
        }
        Ok(Self {
            estimated_alt_cm: i32_at(payload, 0),
            vario_cm_s: i16_at(payload, 4),
        })
    }
}

/// Typed `MSP_SET_RAW_RC` (`200`, `16` channels `u16` PWM `1000..=2000 µs`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspRawRc {
    /// 16 PWM RC channels (`µs`, center `1500`).
    pub channels_us: [u16; 16],
}

impl Default for MspRawRc {
    fn default() -> Self {
        let mut channels_us = [1_500_u16; 16];
        if let Some(thr) = channels_us.get_mut(2) {
            *thr = 1_000;
        }
        Self { channels_us }
    }
}

impl MspRawRc {
    /// Encodes all 16 RC channels (`32` bytes little-endian `u16`).
    #[must_use]
    pub fn to_bytes(&self) -> [u8; 32] {
        let mut out = [0_u8; 32];
        for (idx, ch) in self.channels_us.into_iter().enumerate() {
            if let Some(dst) = out.get_mut(idx * 2..(idx + 1) * 2) {
                dst.copy_from_slice(&ch.to_le_bytes());
            }
        }
        out
    }

    /// Decodes `4..=16` little-endian `u16` RC channels, filling omitted high
    /// channels with `1500 µs`.
    pub fn from_bytes(payload: &[u8]) -> Result<Self, XilError> {
        if payload.len() < 8 || payload.len() > 32 || !payload.len().is_multiple_of(2) {
            return Err(XilError::InvalidPacketLength {
                protocol: "MSP_SET_RAW_RC",
                expected: 32,
                actual: payload.len(),
            });
        }
        let mut channels_us = [1_500_u16; 16];
        let count = payload.len() / 2;
        for (idx, slot) in channels_us.iter_mut().take(count).enumerate() {
            *slot = u16_at(payload, idx * 2);
        }
        Ok(Self { channels_us })
    }
}

fn u16_at(bytes: &[u8], off: usize) -> u16 {
    let b0 = bytes.get(off).copied().unwrap_or(0);
    let b1 = bytes.get(off.saturating_add(1)).copied().unwrap_or(0);
    u16::from_le_bytes([b0, b1])
}

fn i16_at(bytes: &[u8], off: usize) -> i16 {
    u16_at(bytes, off) as i16
}

fn u32_at(bytes: &[u8], off: usize) -> u32 {
    let mut raw = [0_u8; 4];
    for (idx, slot) in raw.iter_mut().enumerate() {
        *slot = bytes.get(off.saturating_add(idx)).copied().unwrap_or(0);
    }
    u32::from_le_bytes(raw)
}

fn i32_at(bytes: &[u8], off: usize) -> i32 {
    u32_at(bytes, off) as i32
}

//! GDL-90 (`UDP :4000`) HDLC framing, FAA §2.2.3 CRC-16-CCITT verification, and ownship/traffic/AHRS decoders.
//!
//! Supports standard FAA GDL-90 Public ICD messages (`0x00` Heartbeat, `0x0A` Ownship Report,
//! `0x0B` Ownship Geometric Altitude, `0x14` Traffic Report) plus the ForeFlight / Sentry / Stratux
//! AHRS extension (`0x65`, sub-ID `0x01`).

mod codec;

pub use codec::{encode_ahrs_frame, encode_gdl90_frame, encode_participant_frame};

use crate::error::FlightDeckError;
use crate::gdl90::codec::decode_unescaped_message;
use crate::telemetry::{OwnshipIdentity, OwnshipState, TelemetryGroup};
use world_surveillance::{AddressNamespace, EmitterCategory};

pub(crate) const FLAG_BYTE: u8 = 0x7E;
pub(crate) const ESCAPE_BYTE: u8 = 0x7D;
pub(crate) const ESCAPE_XOR: u8 = 0x20;

/// Participant report shared by GDL-90 `0x0A` (Ownship) and `0x14` (Traffic).
#[derive(Clone, Debug, PartialEq)]
pub struct Gdl90ParticipantReport {
    /// Address namespace (`Icao24`, `AdsbNonIcao`, `TisbTrackFile`, `SurfaceVehicle`, etc.).
    pub namespace: AddressNamespace,
    /// 24-bit participant address.
    pub address: u32,
    /// WGS-84 latitude in degrees.
    pub latitude_deg: f64,
    /// WGS-84 longitude in degrees.
    pub longitude_deg: f64,
    /// Pressure altitude in feet when valid (`raw != 0xFFF`).
    pub pressure_altitude_ft: Option<i32>,
    /// True ground track or heading in degrees `[0, 360)` when valid.
    pub track_or_heading_deg: Option<f64>,
    /// Ground speed in knots when valid (`raw != 0xFFF`).
    pub horizontal_speed_kt: Option<f64>,
    /// Vertical rate in feet per minute when valid (`raw != -2048`).
    pub vertical_speed_fpm: Option<f64>,
    /// Airborne (`true`) vs on-ground (`false`).
    pub airborne: bool,
    /// Navigation Integrity Category (`0..=11`).
    pub nic: u8,
    /// Navigation Accuracy Category for Position (`0..=11`).
    pub nac_p: u8,
    /// DO-260B / GDL-90 emitter category.
    pub emitter_category: EmitterCategory,
    /// Trimmed ASCII callsign / flight ID when non-empty.
    pub callsign: Option<String>,
    /// Traffic alert flag (`1` when receiver declares an alert).
    pub traffic_alert: bool,
}

/// ForeFlight / Sentry / Stratux AHRS & airspeed report (`0x65`, sub-ID `0x01`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gdl90AhrsReport {
    /// Roll angle in degrees, positive right-wing-down.
    pub roll_deg: Option<f64>,
    /// Pitch angle in degrees, positive nose-up.
    pub pitch_deg: Option<f64>,
    /// Heading in degrees `[0, 360)`.
    pub heading_deg: Option<f64>,
    /// Whether `heading_deg` is magnetic (`true`) or true (`false`).
    pub heading_is_magnetic: bool,
    /// Indicated airspeed in knots when valid.
    pub indicated_airspeed_kt: Option<f64>,
    /// True airspeed in knots when valid.
    pub true_airspeed_kt: Option<f64>,
}

/// Decoded GDL-90 message variant.
#[derive(Clone, Debug, PartialEq)]
pub enum Gdl90Message {
    /// `0x00` Heartbeat status.
    Heartbeat {
        /// Whether the receiver's own GNSS fix is currently valid.
        gps_pos_valid: bool,
        /// UTC seconds-of-day timestamp (`0..86400`).
        timestamp_s: u32,
    },
    /// `0x0A` Ownship kinematic report.
    Ownship(Gdl90ParticipantReport),
    /// `0x0B` Ownship geometric altitude above MSL in metres.
    OwnshipGeometricAltitudeM(f64),
    /// `0x14` ADS-B / TIS-B traffic report.
    Traffic(Gdl90ParticipantReport),
    /// `0x65` (`0x01`) AHRS attitude and airspeed report.
    Ahrs(Gdl90AhrsReport),
    /// Unrecognized but CRC-valid GDL-90 message ID.
    Unknown(u8),
}

/// Streaming GDL-90 frame synchronizer and CRC-checked decoder.
#[derive(Clone, Debug, Default)]
pub struct Gdl90Decoder {
    in_frame: bool,
    escape_next: bool,
    frame_buf: Vec<u8>,
    valid_frames: u64,
    crc_failures: u64,
}

impl Gdl90Decoder {
    /// Creates a new [`Gdl90Decoder`].
    #[must_use]
    pub const fn new() -> Self {
        Self {
            in_frame: false,
            escape_next: false,
            frame_buf: Vec::new(),
            valid_frames: 0,
            crc_failures: 0,
        }
    }

    /// Count of CRC-verified GDL-90 frames decoded.
    #[must_use]
    pub const fn valid_frames(&self) -> u64 {
        self.valid_frames
    }

    /// Count of frames discarded due to CRC mismatch.
    #[must_use]
    pub const fn crc_failures(&self) -> u64 {
        self.crc_failures
    }

    /// Feeds an arbitrary UDP datagram or byte stream chunk and returns all valid messages.
    pub fn ingest(&mut self, bytes: &[u8]) -> Vec<Gdl90Message> {
        let mut out = Vec::new();
        for &byte in bytes {
            if byte == FLAG_BYTE {
                if self.in_frame
                    && !self.frame_buf.is_empty()
                    && let Ok(msg) = self.finish_frame()
                {
                    out.push(msg);
                }
                self.frame_buf.clear();
                self.escape_next = false;
                self.in_frame = true;
                continue;
            }
            if !self.in_frame {
                continue;
            }
            if byte == ESCAPE_BYTE {
                self.escape_next = true;
                continue;
            }
            let unescaped = if self.escape_next {
                self.escape_next = false;
                byte ^ ESCAPE_XOR
            } else {
                byte
            };
            if self.frame_buf.len() < 512 {
                self.frame_buf.push(unescaped);
            } else {
                self.frame_buf.clear();
                self.in_frame = false;
            }
        }
        out
    }

    fn finish_frame(&mut self) -> Result<Gdl90Message, FlightDeckError> {
        let len = self.frame_buf.len();
        if len < 3 {
            return Err(FlightDeckError::Gdl90 {
                reason: "frame shorter than 3 bytes".to_owned(),
            });
        }
        let payload_len = len - 2;
        let payload = self
            .frame_buf
            .get(..payload_len)
            .ok_or_else(|| FlightDeckError::Gdl90 {
                reason: "missing payload slice".to_owned(),
            })?;
        let fcs_lo = *self
            .frame_buf
            .get(payload_len)
            .ok_or_else(|| FlightDeckError::Gdl90 {
                reason: "missing FCS low byte".to_owned(),
            })?;
        let fcs_hi =
            *self
                .frame_buf
                .get(payload_len + 1)
                .ok_or_else(|| FlightDeckError::Gdl90 {
                    reason: "missing FCS high byte".to_owned(),
                })?;
        let expected = crc16_gdl90(payload);
        let received = u16::from_le_bytes([fcs_lo, fcs_hi]);
        if expected != received {
            self.crc_failures = self.crc_failures.wrapping_add(1);
            return Err(FlightDeckError::Gdl90 {
                reason: format!("CRC mismatch: expected {expected:#06x}, got {received:#06x}"),
            });
        }
        self.valid_frames = self.valid_frames.wrapping_add(1);
        decode_unescaped_message(payload)
    }
}

/// Computes the FAA GDL-90 Public ICD §2.2.3 16-bit Frame Check Sequence.
#[must_use]
pub fn crc16_gdl90(payload: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in payload {
        let hi = (crc >> 8) as u8;
        let mut poly = u16::from(hi) << 8;
        for _ in 0..8 {
            poly = if (poly & 0x8000) != 0 {
                (poly << 1) ^ 0x1021
            } else {
                poly << 1
            };
        }
        crc = poly ^ (crc << 8) ^ u16::from(byte);
    }
    crc
}

/// Applies a [`Gdl90Message`] to `ownship` at `now_ms`, returning `Some(traffic)` when `0x14`.
pub fn apply_gdl90_message(
    ownship: &mut OwnshipState,
    msg: &Gdl90Message,
    now_ms: u64,
) -> Option<Gdl90ParticipantReport> {
    match msg {
        Gdl90Message::Heartbeat { gps_pos_valid, .. } => {
            if !gps_pos_valid {
                ownship.gnss_valid = false;
            }
        }
        Gdl90Message::Ownship(rep) => apply_ownship_report(ownship, rep, now_ms),
        Gdl90Message::OwnshipGeometricAltitudeM(alt_m) => {
            ownship.gnss_altitude_m = Some(*alt_m);
            if ownship.gnss_valid {
                ownship.clock.mark(TelemetryGroup::Position, now_ms);
            }
        }
        Gdl90Message::Ahrs(ahrs) => apply_ahrs_report(ownship, ahrs, now_ms),
        Gdl90Message::Traffic(rep) => return Some(rep.clone()),
        Gdl90Message::Unknown(_) => {}
    }
    None
}

fn apply_ownship_report(ownship: &mut OwnshipState, rep: &Gdl90ParticipantReport, now_ms: u64) {
    ownship.latitude_deg = Some(rep.latitude_deg);
    ownship.longitude_deg = Some(rep.longitude_deg);
    ownship.gnss_valid = rep.nic > 0 || rep.nac_p > 0 || rep.latitude_deg.abs() > 1e-4;
    if ownship.gnss_valid {
        ownship.clock.mark(TelemetryGroup::Position, now_ms);
    }
    if let Some(alt_ft) = rep.pressure_altitude_ft {
        ownship.pressure_altitude_ft = Some(f64::from(alt_ft));
        ownship.clock.mark(TelemetryGroup::PressureAltitude, now_ms);
    }
    if let (Some(trk), Some(gs)) = (rep.track_or_heading_deg, rep.horizontal_speed_kt) {
        ownship.ground_track_true_deg = Some(trk);
        ownship.ground_speed_kt = Some(gs);
        ownship.ground_vector_valid = true;
        ownship.clock.mark(TelemetryGroup::GroundVector, now_ms);
    }
    if let Some(vs) = rep.vertical_speed_fpm {
        ownship.vertical_speed_fpm = Some(vs);
        ownship.clock.mark(TelemetryGroup::VerticalSpeed, now_ms);
    }
    ownship.identity = Some(OwnshipIdentity {
        mode_s_code: rep.address,
        callsign: rep.callsign.clone(),
    });
}

fn apply_ahrs_report(ownship: &mut OwnshipState, ahrs: &Gdl90AhrsReport, now_ms: u64) {
    if let (Some(roll), Some(pitch)) = (ahrs.roll_deg, ahrs.pitch_deg) {
        ownship.roll_deg = Some(roll);
        ownship.pitch_deg = Some(pitch);
        ownship.clock.mark(TelemetryGroup::Attitude, now_ms);
    }
    if let Some(hdg) = ahrs.heading_deg {
        if ahrs.heading_is_magnetic {
            ownship.magnetic_heading_deg = Some(hdg);
        } else {
            ownship.true_heading_deg = Some(hdg);
        }
        ownship.clock.mark(TelemetryGroup::Attitude, now_ms);
    }
    if let Some(ias) = ahrs.indicated_airspeed_kt {
        ownship.indicated_airspeed_kt = Some(ias);
        ownship.clock.mark(TelemetryGroup::Airspeed, now_ms);
    }
}

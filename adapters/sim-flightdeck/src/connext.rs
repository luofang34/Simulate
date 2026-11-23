//! Connext (`avionics-link`) inner-record stream decoder, outbound keepalive/poll encoder,
//! and capture replay engine.

pub mod nmea;

use crate::error::FlightDeckError;
use crate::telemetry::{OwnshipIdentity, OwnshipState, RouteWaypoint, TelemetryGroup};
use nmea::{
    ConnextAttitude, ConnextGroundVector, ConnextPositionFix, ConnextTrafficTarget, decode_ahrs,
    decode_fpn_waypoints, decode_nmea_ground_vector, decode_nmea_position, decode_traffic_payload,
};

const MARKER: u8 = 0x01;
const CONTROL_KIND: u8 = 0x02;
const TRAFFIC_KIND: u8 = 0x04;
const DATA_KIND: u8 = 0x0c;
const CONTROL_HEADER: usize = 4;
const DATA_HEADER: usize = 8;

/// Sanitized real-hardware Connext session fixture from `~/avionics-link/captures/real-session.tsv`.
pub const REAL_SESSION_TSV: &str = "\
# Capture-derived record stream. Outer accessory framing removed.
0\tdevice_to_controller\t010f00020000000000000000000114
2002\tdevice_to_controller\t012f000c00100000f00a332e3232003547483030323735360047505320313735003030362d42323830302d31390000
2158\tdevice_to_controller\t010c000c00220000000040430159000c002000002447504747412c3035353731392e38302c343033312e3630323733372c4e2c30373433352e3736373737352c572c322c31342c30302e382c30303032342e382c4d2c303033332e372c4d2c2c2a34390d0a0118000c00210000000000000000000000000000000000000109000c102100000001a4000c02300000006c89a1a9d11c30f4ca2f000000008000004e3139383830202000fbffffff0400a8fcaa41b01c09cbca3fd1001c61fe13034a42553139393000008d05ffff00aa430c43c71c3835cb39d10020510271034457493233363300005d05ffff00ea60a35f361da495cabfd300a7e1ff510541544e3333343500007e12ffff00214b3c33661c6febc99fd500ebf1ff2905444c48343939000000381cffff010e000c0250001046504e2f5250010e000c1050001046504e2f5250
5000\tdevice_to_controller\t015f000c0250001046504e2f52503a463a34374e2c4e3430333135573037343335393a463a53424a2c4e3430333530573037343434353a463a4a464b2c4e3430333830573037333436333a463a4b4d54502c4e3431303436573037313535320138000c1050001046504e2f52503a4f4e3a34374e2c4e3430333135573037343335393a4f463a53424a2c4e343033353057303734343435
";

/// Typed event decoded from a Connext record stream.
#[derive(Clone, Debug, PartialEq)]
pub enum ConnextEvent {
    /// Device identification strings (`service 0x10`).
    DeviceInfo(Vec<String>),
    /// GNSS position fix (`service 0x20` `GGA`/`RMC`).
    Position(ConnextPositionFix),
    /// GNSS ground track, ground speed, and variation (`service 0x20` `RMC`).
    GroundVector(ConnextGroundVector),
    /// AHRS attitude sample (`service 0x21`, `source 0`).
    Attitude(ConnextAttitude),
    /// Barometric pressure altitude in feet (`service 0x22`, `subtype 0`).
    PressureAltitudeFeet(f64),
    /// ADS-B ownship identity and position-valid traffic targets (`service 0x30`).
    Traffic {
        /// Optional slot-0 ownship identity.
        ownship: Option<OwnshipIdentity>,
        /// Decoded traffic targets.
        targets: Vec<ConnextTrafficTarget>,
    },
    /// Full flight plan route (`service 0x50`, `source 0x02`).
    Route(Vec<RouteWaypoint>),
    /// Active flight-plan leg (`service 0x50`, `source 0x10`).
    ActiveLeg(Vec<RouteWaypoint>),
    /// Session control record (`kind 0x02`).
    Control(Vec<u8>),
}

/// Output of one Connext stream ingestion or timer tick.
#[derive(Clone, Debug, Default)]
pub struct ConnextBatch {
    /// Outbound wire records to write to the transport in order.
    pub writes: Vec<Vec<u8>>,
    /// Decoded telemetry and session events.
    pub events: Vec<ConnextEvent>,
}

/// Streaming Connext inner-record decoder and keepalive/route-poll coordinator.
#[derive(Clone, Debug, Default)]
pub struct ConnextIngestor {
    buffer: Vec<u8>,
    greeted: bool,
    last_keepalive_ms: Option<u64>,
    last_poll_ms: Option<u64>,
    decoded_records: u64,
}

impl ConnextIngestor {
    /// Creates an empty [`ConnextIngestor`].
    #[must_use]
    pub const fn new() -> Self {
        Self {
            buffer: Vec::new(),
            greeted: false,
            last_keepalive_ms: None,
            last_poll_ms: None,
            decoded_records: 0,
        }
    }

    /// Total records successfully decoded since creation or last reset.
    #[must_use]
    pub const fn decoded_records(&self) -> u64 {
        self.decoded_records
    }

    /// Resets buffered partial bytes and session timers after a transport reconnect.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.greeted = false;
        self.last_keepalive_ms = None;
        self.last_poll_ms = None;
    }

    /// Ingests raw bytes from a TCP relay, accessory bridge, or capture replay at `now_ms`.
    pub fn ingest(&mut self, bytes: &[u8], now_ms: u64) -> ConnextBatch {
        self.buffer.extend_from_slice(bytes);
        let mut batch = ConnextBatch::default();
        while self.buffer.len() >= CONTROL_HEADER {
            if self.buffer.first() != Some(&MARKER) {
                self.buffer.remove(0);
                continue;
            }
            let Some(len_lo) = self.buffer.get(1).copied() else {
                break;
            };
            let Some(len_hi) = self.buffer.get(2).copied() else {
                break;
            };
            let total_len = usize::from(u16::from_le_bytes([len_lo, len_hi]));
            let Some(kind) = self.buffer.get(3).copied() else {
                break;
            };
            let req_header = match kind {
                CONTROL_KIND => CONTROL_HEADER,
                DATA_KIND | TRAFFIC_KIND => DATA_HEADER,
                _ => {
                    self.buffer.remove(0);
                    continue;
                }
            };
            if total_len < req_header {
                self.buffer.remove(0);
                continue;
            }
            if self.buffer.len() < total_len {
                break;
            }
            let frame: Vec<u8> = self.buffer.drain(..total_len).collect();
            self.decoded_records = self.decoded_records.wrapping_add(1);
            self.decode_frame(&frame, now_ms, &mut batch);
        }
        batch
    }

    /// Emits periodic controller keepalive (`1 s`) and route/active-leg poll (`5 s`) writes.
    pub fn tick(&mut self, now_ms: u64) -> ConnextBatch {
        let mut batch = ConnextBatch::default();
        if !self.greeted {
            return batch;
        }
        if self
            .last_keepalive_ms
            .is_none_or(|last| now_ms.saturating_sub(last) >= 1_000)
        {
            self.last_keepalive_ms = Some(now_ms);
            batch.writes.push(encode_control_record(&[
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x01, 0x00,
            ]));
        }
        if self
            .last_poll_ms
            .is_none_or(|last| now_ms.saturating_sub(last) >= 5_000)
        {
            self.last_poll_ms = Some(now_ms);
            let mut poll = encode_data_record(DATA_KIND, 0x02, 0x50, 0x1000, b"FPN/RP");
            poll.extend_from_slice(&encode_data_record(
                DATA_KIND, 0x10, 0x50, 0x1000, b"FPN/RP",
            ));
            batch.writes.push(poll);
        }
        batch
    }

    fn decode_frame(&mut self, frame: &[u8], now_ms: u64, batch: &mut ConnextBatch) {
        let Some(&kind) = frame.get(3) else {
            return;
        };
        if kind == CONTROL_KIND {
            let body = frame.get(CONTROL_HEADER..).unwrap_or(&[]).to_vec();
            if !self.greeted && body.ends_with(&[0x01, 0x14]) {
                self.greeted = true;
                self.last_keepalive_ms = Some(now_ms);
                batch.writes.push(encode_control_record(&body));
            }
            batch.events.push(ConnextEvent::Control(body));
            return;
        }
        let Some(&source) = frame.get(4) else {
            return;
        };
        let Some(&service) = frame.get(5) else {
            return;
        };
        let Some(&sub_lo) = frame.get(6) else {
            return;
        };
        let Some(&sub_hi) = frame.get(7) else {
            return;
        };
        let subtype = u16::from_le_bytes([sub_lo, sub_hi]);
        let body = frame.get(DATA_HEADER..).unwrap_or(&[]);
        decode_service_payload(source, service, subtype, body, &mut batch.events);
    }
}

fn decode_service_payload(
    source: u8,
    service: u8,
    subtype: u16,
    body: &[u8],
    events: &mut Vec<ConnextEvent>,
) {
    match (service, subtype) {
        (0x10, _) => events.push(ConnextEvent::DeviceInfo(parse_device_fields(body))),
        (0x20, _) => {
            if let Some(pos) = decode_nmea_position(body) {
                events.push(ConnextEvent::Position(pos));
            }
            if let Some(vec) = decode_nmea_ground_vector(body) {
                events.push(ConnextEvent::GroundVector(vec));
            }
        }
        (0x21, _) => {
            if let Some(att) = decode_ahrs(source, body) {
                events.push(ConnextEvent::Attitude(att));
            }
        }
        (0x22, 0) => {
            if let Some(bytes) = body.get(0..4)
                && let Ok(arr) = <[u8; 4]>::try_from(bytes)
            {
                events.push(ConnextEvent::PressureAltitudeFeet(f64::from(
                    f32::from_le_bytes(arr),
                )));
            }
        }
        (0x30, _) => {
            let (ownship, targets) = decode_traffic_payload(source, body);
            events.push(ConnextEvent::Traffic { ownship, targets });
        }
        (0x50, 0x1000) => {
            let wps = decode_fpn_waypoints(body);
            if !wps.is_empty() {
                match source {
                    0x02 => events.push(ConnextEvent::Route(wps)),
                    0x10 => events.push(ConnextEvent::ActiveLeg(wps)),
                    _ => {}
                }
            }
        }
        _ => {}
    }
}

fn parse_device_fields(body: &[u8]) -> Vec<String> {
    let mut rem = body.get(2..).unwrap_or(&[]);
    let mut fields = Vec::new();
    while fields.len() < 4 {
        let Some(nul_pos) = rem.iter().position(|&b| b == 0) else {
            break;
        };
        if let Some(slice) = rem.get(..nul_pos)
            && let Ok(s) = core::str::from_utf8(slice)
        {
            fields.push(s.to_owned());
        }
        rem = rem.get(nul_pos.saturating_add(1)..).unwrap_or(&[]);
    }
    fields
}

/// Applies a [`ConnextEvent`] to `ownship` at `now_ms`, returning any ADS-B traffic targets.
pub fn apply_connext_event(
    ownship: &mut OwnshipState,
    event: &ConnextEvent,
    now_ms: u64,
) -> Vec<ConnextTrafficTarget> {
    match event {
        ConnextEvent::Position(fix) => {
            ownship.latitude_deg = Some(fix.latitude_deg);
            ownship.longitude_deg = Some(fix.longitude_deg);
            if fix.altitude_m.is_some() {
                ownship.gnss_altitude_m = fix.altitude_m;
            }
            if fix.satellites.is_some() {
                ownship.satellites = fix.satellites;
            }
            ownship.gnss_valid = fix.fix_valid;
            if fix.fix_valid {
                ownship.clock.mark(TelemetryGroup::Position, now_ms);
            }
        }
        ConnextEvent::GroundVector(vec) => {
            ownship.ground_track_true_deg = Some(vec.track_true_deg);
            ownship.ground_speed_kt = Some(vec.speed_kt);
            if vec.magnetic_variation_deg_east.is_some() {
                ownship.magnetic_variation_deg_east = vec.magnetic_variation_deg_east;
            }
            ownship.ground_vector_valid = vec.fix_valid;
            if vec.fix_valid {
                ownship.clock.mark(TelemetryGroup::GroundVector, now_ms);
            }
        }
        ConnextEvent::Attitude(att) => {
            ownship.raw_ahrs_axes_456 = Some(att.raw_axes_456);
            if let (Some(hdg), Some(pitch), Some(roll)) =
                (att.magnetic_heading_deg, att.pitch_deg, att.roll_deg)
            {
                ownship.magnetic_heading_deg = Some(hdg);
                ownship.pitch_deg = Some(pitch);
                ownship.roll_deg = Some(roll);
                ownship.clock.mark(TelemetryGroup::Attitude, now_ms);
            }
        }
        ConnextEvent::PressureAltitudeFeet(feet) => {
            ownship.pressure_altitude_ft = Some(*feet);
            ownship.clock.mark(TelemetryGroup::PressureAltitude, now_ms);
        }
        ConnextEvent::Traffic {
            ownship: id,
            targets,
        } => {
            if let Some(ident) = id {
                ownship.identity = Some(ident.clone());
            }
            return targets.clone();
        }
        ConnextEvent::Route(wps) => {
            ownship.route.waypoints.clone_from(wps);
        }
        ConnextEvent::ActiveLeg(wps) => {
            if let (Some(from), Some(to)) = (wps.first(), wps.get(1)) {
                ownship.route.active_leg = Some((from.clone(), to.clone()));
            }
        }
        ConnextEvent::DeviceInfo(_) | ConnextEvent::Control(_) => {}
    }
    // Connext carries neither airspeed nor vertical speed; keep them absent so Indicate flags them.
    ownship.indicated_airspeed_kt = None;
    ownship.vertical_speed_fpm = None;
    Vec::new()
}

/// Encodes one Connext addressed data record.
#[must_use]
pub fn encode_data_record(
    kind_wire: u8,
    source: u8,
    service: u8,
    subtype: u16,
    body: &[u8],
) -> Vec<u8> {
    let total = DATA_HEADER.saturating_add(body.len());
    let mut out = Vec::with_capacity(total);
    out.push(MARKER);
    out.extend_from_slice(&(total as u16).to_le_bytes());
    out.push(kind_wire);
    out.push(source);
    out.push(service);
    out.extend_from_slice(&subtype.to_le_bytes());
    out.extend_from_slice(body);
    out
}

/// Encodes one Connext control record (`0x02`).
#[must_use]
pub fn encode_control_record(body: &[u8]) -> Vec<u8> {
    let total = CONTROL_HEADER.saturating_add(body.len());
    let mut out = Vec::with_capacity(total);
    out.push(MARKER);
    out.extend_from_slice(&(total as u16).to_le_bytes());
    out.push(CONTROL_KIND);
    out.extend_from_slice(body);
    out
}

/// Encodes a confirmed Flight Stream AHRS `0x21` record (`0x103f6403`).
#[must_use]
pub fn encode_ahrs_record(mag_heading_deg: f64, pitch_deg: f64, roll_deg: f64) -> Vec<u8> {
    let scale = 32_768.0 / 180.0;
    let wrapped = if mag_heading_deg >= 180.0 {
        mag_heading_deg - 360.0
    } else {
        mag_heading_deg
    };
    let hdg_cnt = (wrapped * scale).round().clamp(-32_768.0, 32_767.0) as i16;
    let pitch_cnt = (pitch_deg * scale).round().clamp(-32_768.0, 32_767.0) as i16;
    let roll_cnt = (roll_deg * scale).round().clamp(-32_768.0, 32_767.0) as i16;
    let mut body = Vec::with_capacity(16);
    body.extend_from_slice(&0x103f_6403u32.to_le_bytes());
    body.extend_from_slice(&hdg_cnt.to_le_bytes());
    body.extend_from_slice(&pitch_cnt.to_le_bytes());
    body.extend_from_slice(&roll_cnt.to_le_bytes());
    body.extend_from_slice(&0i16.to_le_bytes());
    body.extend_from_slice(&0i16.to_le_bytes());
    body.extend_from_slice(&0i16.to_le_bytes());
    encode_data_record(DATA_KIND, 0, 0x21, 0, &body)
}

/// Timed capture row parsed from `.tsv` (`time_ms\tdirection\trecords_hex`).
#[derive(Clone, Debug)]
pub struct ReplayRow {
    /// Relative timestamp in milliseconds.
    pub time_ms: u64,
    /// Raw inner-record bytes emitted from device to controller.
    pub bytes: Vec<u8>,
}

/// Parses `device_to_controller` rows from a `real-session.tsv` capture text.
///
/// # Errors
///
/// Returns [`FlightDeckError::ConnextRecord`] if a row contains malformed hexadecimal bytes.
pub fn parse_capture_tsv(tsv: &str) -> Result<Vec<ReplayRow>, FlightDeckError> {
    let mut rows = Vec::new();
    for line in tsv.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let mut parts = trimmed.split('\t');
        let Some(time_str) = parts.next() else {
            continue;
        };
        let Some(direction) = parts.next() else {
            continue;
        };
        let Some(hex_str) = parts.next() else {
            continue;
        };
        if direction != "device_to_controller" {
            continue;
        }
        let time_ms: u64 = time_str
            .parse()
            .map_err(|_| FlightDeckError::ConnextRecord {
                reason: format!("invalid timestamp {time_str}"),
            })?;
        let bytes = decode_hex(hex_str)?;
        rows.push(ReplayRow { time_ms, bytes });
    }
    Ok(rows)
}

fn decode_hex(hex: &str) -> Result<Vec<u8>, FlightDeckError> {
    if !hex.len().is_multiple_of(2) {
        return Err(FlightDeckError::ConnextRecord {
            reason: "odd hex length".to_owned(),
        });
    }
    let mut out = Vec::with_capacity(hex.len() / 2);
    let mut idx = 0usize;
    while let Some(pair) = hex.get(idx..idx.saturating_add(2)) {
        let byte = u8::from_str_radix(pair, 16).map_err(|_| FlightDeckError::ConnextRecord {
            reason: format!("invalid hex byte {pair}"),
        })?;
        out.push(byte);
        idx = idx.saturating_add(2);
    }
    Ok(out)
}

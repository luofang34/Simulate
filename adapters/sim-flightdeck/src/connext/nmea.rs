//! Decoders for Connext NMEA navigation sentences, Flight Stream AHRS, ADS-B traffic, and FPN routes.

use crate::telemetry::{OwnshipIdentity, RouteWaypoint};

const AHRS_CONFIRMED_FORMAT: u32 = 0x103f_6403;
const AHRS_DEG_PER_COUNT: f64 = 180.0 / 32_768.0;
const TRAFFIC_ENTRY_LEN: usize = 31;

/// Position fix parsed from a `$GPGGA`, `$GNGGA`, `$GPRMC`, or `$GNRMC` sentence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConnextPositionFix {
    /// Latitude in degrees (`-90..=90`).
    pub latitude_deg: f64,
    /// Longitude in degrees (`-180..=180`).
    pub longitude_deg: f64,
    /// Geometric / MSL altitude in metres when carried (`GGA`).
    pub altitude_m: Option<f64>,
    /// Tracked satellite count when carried (`GGA`).
    pub satellites: Option<u8>,
    /// Fix validity (`quality > 0` for `GGA`, status `'A'` for `RMC`).
    pub fix_valid: bool,
}

/// Ground track and speed parsed from a `$GPRMC` or `$GNRMC` sentence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConnextGroundVector {
    /// True ground track in degrees `[0, 360)`.
    pub track_true_deg: f64,
    /// Ground speed in knots.
    pub speed_kt: f64,
    /// Magnetic variation in degrees, east positive.
    pub magnetic_variation_deg_east: Option<f64>,
    /// Fix validity (`'A'` active vs `'V'` void).
    pub fix_valid: bool,
}

/// Confirmed Flight Stream 210 / Connext AHRS attitude sample (`0x103f6403`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConnextAttitude {
    /// Wire format tag (`0x103f6403` for confirmed Euler layout).
    pub format: u32,
    /// Magnetic heading in degrees `[0, 360)`, when format is confirmed.
    pub magnetic_heading_deg: Option<f64>,
    /// Pitch in degrees (positive nose-up), when format is confirmed.
    pub pitch_deg: Option<f64>,
    /// Roll in degrees (positive right-wing-down), when format is confirmed.
    pub roll_deg: Option<f64>,
    /// Uninterpreted motion-correlated fields 4..6 preserved losslessly.
    pub raw_axes_456: [i16; 3],
}

/// One position-valid ADS-B traffic target decoded from Connext service `0x30`.
#[derive(Clone, Debug, PartialEq)]
pub struct ConnextTrafficTarget {
    /// Raw 32-bit target/slot identifier (low 24 bits carry address).
    pub raw_id: u32,
    /// Trimmed ASCII callsign when present (`entry[17..25]`).
    pub callsign: Option<String>,
    /// Latitude in degrees (`-90..=90`).
    pub latitude_deg: f64,
    /// Longitude in degrees (`-180..=180`).
    pub longitude_deg: f64,
    /// Pressure altitude in feet (`raw * 25 - 1000` when 12-bit altitude code is non-zero).
    pub pressure_altitude_ft: Option<i32>,
}

/// Decodes a position fix from a Connext NMEA sentence.
#[must_use]
pub fn decode_nmea_position(sentence_bytes: &[u8]) -> Option<ConnextPositionFix> {
    let text = core::str::from_utf8(sentence_bytes).ok()?;
    let trimmed = strip_nmea_checksum(text.trim());
    let fields: Vec<&str> = trimmed.split(',').collect();
    match fields.first().copied()? {
        "$GPGGA" | "$GNGGA" => decode_gga(&fields),
        "$GPRMC" | "$GNRMC" => decode_rmc_position(&fields),
        _ => None,
    }
}

/// Decodes ground track, speed, and variation from a `$GPRMC` / `$GNRMC` sentence.
#[must_use]
pub fn decode_nmea_ground_vector(sentence_bytes: &[u8]) -> Option<ConnextGroundVector> {
    let text = core::str::from_utf8(sentence_bytes).ok()?;
    let trimmed = strip_nmea_checksum(text.trim());
    let fields: Vec<&str> = trimmed.split(',').collect();
    let talker = *fields.first()?;
    if talker != "$GPRMC" && talker != "$GNRMC" {
        return None;
    }
    let status = *fields.get(2)?;
    if status != "A" && status != "V" {
        return None;
    }
    let speed_kt: f64 = fields.get(7)?.parse().ok()?;
    let track_true_deg: f64 = fields.get(8)?.parse().ok()?;
    let mag_var = decode_variation(&fields);
    Some(ConnextGroundVector {
        track_true_deg: track_true_deg.rem_euclid(360.0),
        speed_kt,
        magnetic_variation_deg_east: mag_var,
        fix_valid: status == "A",
    })
}

/// Decodes a 16-byte Connext AHRS service `0x21` payload (`source == 0`).
#[must_use]
pub fn decode_ahrs(source: u8, body: &[u8]) -> Option<ConnextAttitude> {
    if source != 0 || body.len() != 16 {
        return None;
    }
    let format = u32_le(body, 0)?;
    let a0 = i16_le(body, 4)?;
    let a1 = i16_le(body, 6)?;
    let a2 = i16_le(body, 8)?;
    let a3 = i16_le(body, 10)?;
    let a4 = i16_le(body, 12)?;
    let a5 = i16_le(body, 14)?;
    if format == AHRS_CONFIRMED_FORMAT {
        let signed_hdg = f64::from(a0) * AHRS_DEG_PER_COUNT;
        let mag_hdg = if signed_hdg < 0.0 {
            signed_hdg + 360.0
        } else {
            signed_hdg
        };
        Some(ConnextAttitude {
            format,
            magnetic_heading_deg: Some(mag_hdg.rem_euclid(360.0)),
            pitch_deg: Some(f64::from(a1) * AHRS_DEG_PER_COUNT),
            roll_deg: Some(f64::from(a2) * AHRS_DEG_PER_COUNT),
            raw_axes_456: [a3, a4, a5],
        })
    } else {
        Some(ConnextAttitude {
            format,
            magnetic_heading_deg: None,
            pitch_deg: None,
            roll_deg: None,
            raw_axes_456: [a3, a4, a5],
        })
    }
}

/// Decodes ownship Mode-S & callsign (if present in slot 0) and airborne traffic from service `0x30`.
#[must_use]
pub fn decode_traffic_payload(
    source: u8,
    body: &[u8],
) -> (Option<OwnshipIdentity>, Vec<ConnextTrafficTarget>) {
    let ownship = if source == 2 && body.first() == Some(&0) {
        body.get(1..32).and_then(|entry| {
            let b0 = u32::from(*entry.first()?);
            let b1 = u32::from(*entry.get(1)?) << 8;
            let b2 = u32::from(*entry.get(2)?) << 16;
            Some(OwnshipIdentity {
                mode_s_code: b0 | b1 | b2,
                callsign: decode_entry_callsign(entry),
            })
        })
    } else {
        None
    };

    let mut targets = Vec::new();
    let mut offset = 1usize;
    let skip_first_ownship = ownship.is_some();
    let mut slot_index = 0usize;
    while let Some(entry) = body.get(offset..offset.saturating_add(TRAFFIC_ENTRY_LEN)) {
        if !(skip_first_ownship && slot_index == 0)
            && let Some(target) = decode_traffic_entry(entry)
        {
            targets.push(target);
        }
        offset = offset.saturating_add(TRAFFIC_ENTRY_LEN);
        slot_index = slot_index.wrapping_add(1);
    }
    (ownship, targets)
}

/// Parses Garmin `FPN/RP:` route or active-leg ASCII payloads into ordered [`RouteWaypoint`]s.
#[must_use]
pub fn decode_fpn_waypoints(body: &[u8]) -> Vec<RouteWaypoint> {
    let Ok(text) = core::str::from_utf8(body) else {
        return Vec::new();
    };
    let Some(rest) = text
        .strip_prefix("FPN/RP:")
        .or_else(|| text.strip_prefix("FPN/RI:"))
    else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for token in rest.split(':') {
        let Some((ident_raw, coord_raw)) = token.split_once(',') else {
            continue;
        };
        let ident = ident_raw.trim();
        if ident.is_empty() {
            continue;
        }
        if let Some((lat, lon)) = parse_fpn_coord(coord_raw.trim()) {
            out.push(RouteWaypoint {
                ident: ident.to_owned(),
                latitude_deg: lat,
                longitude_deg: lon,
            });
        }
    }
    out
}

fn strip_nmea_checksum(s: &str) -> &str {
    s.split_once('*').map_or(s, |(body, _)| body)
}

fn decode_gga(fields: &[&str]) -> Option<ConnextPositionFix> {
    let lat = parse_nmea_coord(fields.get(2)?, fields.get(3)?)?;
    let lon = parse_nmea_coord(fields.get(4)?, fields.get(5)?)?;
    let quality: u8 = fields.get(6)?.parse().ok()?;
    let satellites: Option<u8> = fields.get(7).and_then(|s| s.parse().ok());
    let altitude_m: Option<f64> = fields.get(9).and_then(|s| s.parse().ok());
    Some(ConnextPositionFix {
        latitude_deg: lat,
        longitude_deg: lon,
        altitude_m,
        satellites,
        fix_valid: quality > 0,
    })
}

fn decode_rmc_position(fields: &[&str]) -> Option<ConnextPositionFix> {
    let status = *fields.get(2)?;
    if status != "A" {
        return None;
    }
    let lat = parse_nmea_coord(fields.get(3)?, fields.get(4)?)?;
    let lon = parse_nmea_coord(fields.get(5)?, fields.get(6)?)?;
    Some(ConnextPositionFix {
        latitude_deg: lat,
        longitude_deg: lon,
        altitude_m: None,
        satellites: None,
        fix_valid: true,
    })
}

fn parse_nmea_coord(raw: &str, hemi: &str) -> Option<f64> {
    let packed: f64 = raw.parse().ok()?;
    if !packed.is_finite() {
        return None;
    }
    let degrees = libm::floor(packed / 100.0);
    let minutes = packed - degrees * 100.0;
    let mut decimal = degrees + minutes / 60.0;
    match hemi {
        "N" | "E" => {}
        "S" | "W" => decimal = -decimal,
        _ => return None,
    }
    Some(decimal)
}

fn decode_variation(fields: &[&str]) -> Option<f64> {
    let mag: f64 = fields.get(10)?.parse().ok()?;
    match *fields.get(11)? {
        "E" => Some(mag),
        "W" => Some(-mag),
        _ => None,
    }
}

fn decode_traffic_entry(entry: &[u8]) -> Option<ConnextTrafficTarget> {
    let raw_id = u32_le(entry, 0)?;
    let lat = f64::from(i24_le(entry, 4)?) * (180.0 / 8_388_608.0);
    let lon = f64::from(i24_le(entry, 7)?) * (180.0 / 8_388_608.0);
    if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return None;
    }
    let lo = i32::from(*entry.get(10)?);
    let hi = i32::from(*entry.get(11)? & 0x0f) << 8;
    let raw_alt = lo | hi;
    let pressure_altitude_ft = (raw_alt != 0).then_some(raw_alt * 25 - 1_000);
    Some(ConnextTrafficTarget {
        raw_id,
        callsign: decode_entry_callsign(entry),
        latitude_deg: lat,
        longitude_deg: lon,
        pressure_altitude_ft,
    })
}

fn decode_entry_callsign(entry: &[u8]) -> Option<String> {
    let slice = entry.get(17..25)?;
    let start = usize::from(slice.first().is_some_and(|&b| b < 0x20));
    let sub = slice.get(start..)?;
    let mut end = sub.len();
    while end > 0 {
        let last = sub.get(end - 1).copied().unwrap_or(0);
        if last == 0 || last == b' ' {
            end -= 1;
        } else {
            break;
        }
    }
    let trimmed = sub.get(..end)?;
    if trimmed.is_empty() || trimmed.iter().any(|b| !b.is_ascii_graphic() && *b != b' ') {
        return None;
    }
    String::from_utf8(trimmed.to_vec()).ok()
}

fn parse_fpn_coord(coord: &str) -> Option<(f64, f64)> {
    // Format: N40315W074359 -> lat N 40 deg 31.5 min, lon W 074 deg 35.9 min
    if coord.len() < 13 {
        return None;
    }
    let lat_h = coord.get(0..1)?;
    let lat_deg: f64 = coord.get(1..3)?.parse().ok()?;
    let lat_tenths: f64 = coord.get(3..6)?.parse().ok()?;
    let lon_h = coord.get(6..7)?;
    let lon_deg: f64 = coord.get(7..10)?.parse().ok()?;
    let lon_tenths: f64 = coord.get(10..13)?.parse().ok()?;
    let mut lat = lat_deg + (lat_tenths / 10.0) / 60.0;
    let mut lon = lon_deg + (lon_tenths / 10.0) / 60.0;
    if lat_h == "S" {
        lat = -lat;
    } else if lat_h != "N" {
        return None;
    }
    if lon_h == "W" {
        lon = -lon;
    } else if lon_h != "E" {
        return None;
    }
    Some((lat, lon))
}

fn i16_le(bytes: &[u8], offset: usize) -> Option<i16> {
    let b0 = *bytes.get(offset)?;
    let b1 = *bytes.get(offset.saturating_add(1))?;
    Some(i16::from_le_bytes([b0, b1]))
}

fn i24_le(bytes: &[u8], offset: usize) -> Option<i32> {
    let b0 = i32::from(*bytes.get(offset)?);
    let b1 = i32::from(*bytes.get(offset.saturating_add(1))?) << 8;
    let b2 = i32::from(*bytes.get(offset.saturating_add(2))?) << 16;
    let mut val = b0 | b1 | b2;
    if (val & 0x0080_0000) != 0 {
        val |= -16_777_216; // sign-extend 24-bit
    }
    Some(val)
}

fn u32_le(bytes: &[u8], offset: usize) -> Option<u32> {
    let b0 = *bytes.get(offset)?;
    let b1 = *bytes.get(offset.saturating_add(1))?;
    let b2 = *bytes.get(offset.saturating_add(2))?;
    let b3 = *bytes.get(offset.saturating_add(3))?;
    Some(u32::from_le_bytes([b0, b1, b2, b3]))
}

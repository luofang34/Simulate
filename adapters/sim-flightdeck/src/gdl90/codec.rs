//! GDL-90 binary payload encoders and decoders (`0x00`, `0x0A`, `0x0B`, `0x14`, `0x65`).

use crate::error::FlightDeckError;
use crate::gdl90::{
    ESCAPE_BYTE, ESCAPE_XOR, FLAG_BYTE, Gdl90AhrsReport, Gdl90Message, Gdl90ParticipantReport,
    crc16_gdl90,
};
use crate::telemetry::FEET_TO_METRES;
use world::surveillance::{AddressNamespace, EmitterCategory};

/// Wraps an unescaped GDL-90 message payload with its LSB-first CRC-16 and `0x7E` flags.
#[must_use]
pub fn encode_gdl90_frame(payload: &[u8]) -> Vec<u8> {
    let crc = crc16_gdl90(payload);
    let fcs = crc.to_le_bytes();
    let mut out = Vec::with_capacity(payload.len().saturating_add(8));
    out.push(FLAG_BYTE);
    for &b in payload.iter().chain(fcs.iter()) {
        if b == FLAG_BYTE || b == ESCAPE_BYTE {
            out.push(ESCAPE_BYTE);
            out.push(b ^ ESCAPE_XOR);
        } else {
            out.push(b);
        }
    }
    out.push(FLAG_BYTE);
    out
}

/// Encodes a GDL-90 `0x0A` (Ownship) or `0x14` (Traffic) framed packet.
#[must_use]
pub fn encode_participant_frame(msg_id: u8, report: &Gdl90ParticipantReport) -> Vec<u8> {
    let mut p = Vec::with_capacity(28);
    p.push(msg_id);
    let addr_type: u8 = match report.namespace {
        AddressNamespace::Icao => 0,
        AddressNamespace::AdsbNonIcao | AddressNamespace::SelfAssigned => 1,
        AddressNamespace::TisbTrackFile | AddressNamespace::TisbModeATrackFile => 3,
        AddressNamespace::SurfaceVehicle => 4,
        AddressNamespace::FixedBeacon => 5,
        _ => 0,
    };
    let alert_nibble = if report.traffic_alert { 0x10 } else { 0x00 };
    p.push(alert_nibble | addr_type);
    p.push(((report.address >> 16) & 0xff) as u8);
    p.push(((report.address >> 8) & 0xff) as u8);
    p.push((report.address & 0xff) as u8);
    push_semicircles_be(&mut p, report.latitude_deg);
    push_semicircles_be(&mut p, report.longitude_deg);

    let raw_alt: u16 = report
        .pressure_altitude_ft
        .and_then(|ft| u16::try_from(((ft + 1_000) / 25).clamp(0, 0x0FFE)).ok())
        .unwrap_or(0x0FFF);
    let misc: u8 = (if report.airborne { 0x08 } else { 0x00 })
        | (if report.track_or_heading_deg.is_some() {
            0x01
        } else {
            0x00
        });
    p.push(((raw_alt >> 4) & 0xff) as u8);
    p.push((((raw_alt & 0x0f) as u8) << 4) | misc);
    p.push(((report.nic & 0x0f) << 4) | (report.nac_p & 0x0f));

    let spd_u16: u16 = report
        .horizontal_speed_kt
        .map(|kt| (kt.round().clamp(0.0, 4094.0)) as u16)
        .unwrap_or(0x0FFF);
    let vs_raw: u16 = report
        .vertical_speed_fpm
        .map(|fpm| {
            let units = (fpm / 64.0).round().clamp(-2047.0, 2047.0) as i16;
            (units as u16) & 0x0FFF
        })
        .unwrap_or(0x0800);
    p.push(((spd_u16 >> 4) & 0xff) as u8);
    p.push((((spd_u16 & 0x0f) as u8) << 4) | (((vs_raw >> 8) & 0x0f) as u8));
    p.push((vs_raw & 0xff) as u8);

    let trk_byte = report
        .track_or_heading_deg
        .map(|deg| ((deg.rem_euclid(360.0) / 360.0) * 256.0).round() as u8)
        .unwrap_or(0);
    p.push(trk_byte);
    p.push(encode_emitter_category(report.emitter_category));

    let mut cs_bytes = [b' '; 8];
    if let Some(cs) = &report.callsign {
        for (dst, src) in cs_bytes.iter_mut().zip(cs.bytes()) {
            *dst = src.to_ascii_uppercase();
        }
    }
    p.extend_from_slice(&cs_bytes);
    p.push(0);
    encode_gdl90_frame(&p)
}

/// Encodes a ForeFlight / Sentry AHRS (`0x65`, sub-ID `0x01`) framed packet.
#[must_use]
pub fn encode_ahrs_frame(report: &Gdl90AhrsReport) -> Vec<u8> {
    let mut p = Vec::with_capacity(12);
    p.push(0x65);
    p.push(0x01);
    let roll_i16 = report
        .roll_deg
        .map(|d| (d * 10.0).round().clamp(-1800.0, 1800.0) as i16)
        .unwrap_or(0x7FFF);
    let pitch_i16 = report
        .pitch_deg
        .map(|d| (d * 10.0).round().clamp(-900.0, 900.0) as i16)
        .unwrap_or(0x7FFF);
    let hdg_u16 = report
        .heading_deg
        .map(|d| {
            let tenths = (d.rem_euclid(360.0) * 10.0).round().clamp(0.0, 3599.0) as u16;
            if report.heading_is_magnetic {
                tenths | 0x8000
            } else {
                tenths
            }
        })
        .unwrap_or(0xFFFF);
    let ias_u16 = report
        .indicated_airspeed_kt
        .map(|kt| kt.round().clamp(0.0, 999.0) as u16)
        .unwrap_or(0xFFFF);
    let tas_u16 = report
        .true_airspeed_kt
        .map(|kt| kt.round().clamp(0.0, 999.0) as u16)
        .unwrap_or(0xFFFF);
    p.extend_from_slice(&roll_i16.to_be_bytes());
    p.extend_from_slice(&pitch_i16.to_be_bytes());
    p.extend_from_slice(&hdg_u16.to_be_bytes());
    p.extend_from_slice(&ias_u16.to_be_bytes());
    p.extend_from_slice(&tas_u16.to_be_bytes());
    encode_gdl90_frame(&p)
}

pub(super) fn decode_unescaped_message(payload: &[u8]) -> Result<Gdl90Message, FlightDeckError> {
    let msg_id = *payload.first().ok_or_else(|| FlightDeckError::Gdl90 {
        reason: "empty payload".to_owned(),
    })?;
    match msg_id {
        0x00 => decode_heartbeat(payload),
        0x0A => decode_participant(payload).map(Gdl90Message::Ownship),
        0x0B => decode_geo_altitude(payload),
        0x14 => decode_participant(payload).map(Gdl90Message::Traffic),
        0x65 => decode_ahrs_extension(payload),
        other => Ok(Gdl90Message::Unknown(other)),
    }
}

fn decode_heartbeat(p: &[u8]) -> Result<Gdl90Message, FlightDeckError> {
    let s1 = *p.get(1).ok_or_else(|| FlightDeckError::Gdl90 {
        reason: "truncated heartbeat".to_owned(),
    })?;
    let s2 = *p.get(2).ok_or_else(|| FlightDeckError::Gdl90 {
        reason: "truncated heartbeat".to_owned(),
    })?;
    let ts_lo = u32::from(*p.get(3).unwrap_or(&0));
    let ts_mid = u32::from(*p.get(4).unwrap_or(&0)) << 8;
    let ts_hi = u32::from((s2 >> 7) & 0x01) << 16;
    Ok(Gdl90Message::Heartbeat {
        gps_pos_valid: (s1 & 0x80) != 0,
        timestamp_s: ts_hi | ts_mid | ts_lo,
    })
}

fn decode_geo_altitude(p: &[u8]) -> Result<Gdl90Message, FlightDeckError> {
    let hi = *p.get(1).ok_or_else(|| FlightDeckError::Gdl90 {
        reason: "truncated geometric altitude".to_owned(),
    })?;
    let lo = *p.get(2).ok_or_else(|| FlightDeckError::Gdl90 {
        reason: "truncated geometric altitude".to_owned(),
    })?;
    let raw_five_ft = i16::from_be_bytes([hi, lo]);
    Ok(Gdl90Message::OwnshipGeometricAltitudeM(
        f64::from(raw_five_ft) * 5.0 * FEET_TO_METRES,
    ))
}

fn decode_participant(p: &[u8]) -> Result<Gdl90ParticipantReport, FlightDeckError> {
    if p.len() < 28 {
        return Err(FlightDeckError::Gdl90 {
            reason: format!("participant report too short: {}", p.len()),
        });
    }
    let b1 = *p.get(1).unwrap_or(&0);
    let namespace = match b1 & 0x0f {
        0 | 2 => AddressNamespace::Icao,
        1 => AddressNamespace::AdsbNonIcao,
        3 => AddressNamespace::TisbTrackFile,
        4 => AddressNamespace::SurfaceVehicle,
        5 => AddressNamespace::FixedBeacon,
        _ => AddressNamespace::Icao,
    };
    let address = (u32::from(*p.get(2).unwrap_or(&0)) << 16)
        | (u32::from(*p.get(3).unwrap_or(&0)) << 8)
        | u32::from(*p.get(4).unwrap_or(&0));
    let latitude_deg = f64::from(i24_be(p, 5)) * (180.0 / 8_388_608.0);
    let longitude_deg = f64::from(i24_be(p, 8)) * (180.0 / 8_388_608.0);
    let b11 = u16::from(*p.get(11).unwrap_or(&0));
    let b12 = *p.get(12).unwrap_or(&0);
    let raw_alt = (b11 << 4) | u16::from(b12 >> 4);
    let pressure_altitude_ft = (raw_alt != 0x0FFF).then(|| i32::from(raw_alt) * 25 - 1_000);
    let misc = b12 & 0x0f;
    let b13 = *p.get(13).unwrap_or(&0);
    let b14 = u16::from(*p.get(14).unwrap_or(&0));
    let b15 = *p.get(15).unwrap_or(&0);
    let b16 = *p.get(16).unwrap_or(&0);
    let raw_spd = (b14 << 4) | u16::from(b15 >> 4);
    let horizontal_speed_kt = (raw_spd != 0x0FFF).then_some(f64::from(raw_spd));
    let mut raw_vs = (i16::from(b15 & 0x0f) << 8) | i16::from(b16);
    if (raw_vs & 0x0800) != 0 {
        raw_vs |= -4096;
    }
    let vertical_speed_fpm = (raw_vs != -2048).then_some(f64::from(raw_vs) * 64.0);
    let trk_valid = (misc & 0x03) != 0;
    let track_or_heading_deg =
        trk_valid.then(|| f64::from(*p.get(17).unwrap_or(&0)) * (360.0 / 256.0));
    let emitter_category = decode_emitter_category(*p.get(18).unwrap_or(&0));
    let callsign = p
        .get(19..27)
        .and_then(|s| core::str::from_utf8(s).ok())
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(str::to_owned);
    Ok(Gdl90ParticipantReport {
        namespace,
        address,
        latitude_deg,
        longitude_deg,
        pressure_altitude_ft,
        track_or_heading_deg,
        horizontal_speed_kt,
        vertical_speed_fpm,
        airborne: (misc & 0x08) != 0,
        nic: b13 >> 4,
        nac_p: b13 & 0x0f,
        emitter_category,
        callsign,
        traffic_alert: (b1 >> 4) != 0,
    })
}

fn decode_ahrs_extension(p: &[u8]) -> Result<Gdl90Message, FlightDeckError> {
    if p.get(1) != Some(&0x01) || p.len() < 12 {
        return Ok(Gdl90Message::Unknown(0x65));
    }
    let roll_raw = i16::from_be_bytes([*p.get(2).unwrap_or(&0x7f), *p.get(3).unwrap_or(&0xff)]);
    let pitch_raw = i16::from_be_bytes([*p.get(4).unwrap_or(&0x7f), *p.get(5).unwrap_or(&0xff)]);
    let hdg_raw = u16::from_be_bytes([*p.get(6).unwrap_or(&0xff), *p.get(7).unwrap_or(&0xff)]);
    let ias_raw = u16::from_be_bytes([*p.get(8).unwrap_or(&0xff), *p.get(9).unwrap_or(&0xff)]);
    let tas_raw = u16::from_be_bytes([*p.get(10).unwrap_or(&0xff), *p.get(11).unwrap_or(&0xff)]);
    let hdg_tenths = hdg_raw & 0x7FFF;
    Ok(Gdl90Message::Ahrs(Gdl90AhrsReport {
        roll_deg: (roll_raw != 0x7FFF && (-1800..=1800).contains(&roll_raw))
            .then_some(f64::from(roll_raw) * 0.1),
        pitch_deg: (pitch_raw != 0x7FFF && (-900..=900).contains(&pitch_raw))
            .then_some(f64::from(pitch_raw) * 0.1),
        heading_deg: (hdg_tenths <= 3600)
            .then_some((f64::from(hdg_tenths) * 0.1).rem_euclid(360.0)),
        heading_is_magnetic: (hdg_raw & 0x8000) != 0,
        indicated_airspeed_kt: (ias_raw != 0xFFFF).then_some(f64::from(ias_raw)),
        true_airspeed_kt: (tas_raw != 0xFFFF).then_some(f64::from(tas_raw)),
    }))
}

fn push_semicircles_be(out: &mut Vec<u8>, deg: f64) {
    let raw = ((deg.clamp(-180.0, 180.0) / 180.0) * 8_388_608.0).round() as i32;
    out.push(((raw >> 16) & 0xff) as u8);
    out.push(((raw >> 8) & 0xff) as u8);
    out.push((raw & 0xff) as u8);
}

fn i24_be(bytes: &[u8], offset: usize) -> i32 {
    let b0 = i32::from(*bytes.get(offset).unwrap_or(&0)) << 16;
    let b1 = i32::from(*bytes.get(offset.saturating_add(1)).unwrap_or(&0)) << 8;
    let b2 = i32::from(*bytes.get(offset.saturating_add(2)).unwrap_or(&0));
    let mut val = b0 | b1 | b2;
    if (val & 0x0080_0000) != 0 {
        val |= -16_777_216;
    }
    val
}

fn decode_emitter_category(code: u8) -> Option<EmitterCategory> {
    match code {
        1 => Some(EmitterCategory::LightAircraft),
        2 => Some(EmitterCategory::SmallAircraft),
        3 => Some(EmitterCategory::LargeAircraft),
        4 => Some(EmitterCategory::HighVortexAircraft),
        5 => Some(EmitterCategory::HeavyAircraft),
        6 => Some(EmitterCategory::HighPerformanceAircraft),
        7 => Some(EmitterCategory::Rotorcraft),
        9 => Some(EmitterCategory::Glider),
        10 => Some(EmitterCategory::LighterThanAir),
        11 => Some(EmitterCategory::Parachutist),
        12 => Some(EmitterCategory::Ultralight),
        14 => Some(EmitterCategory::UncrewedAircraft),
        15 => Some(EmitterCategory::SpaceVehicle),
        17 => Some(EmitterCategory::SurfaceEmergencyVehicle),
        18 => Some(EmitterCategory::SurfaceServiceVehicle),
        19 => Some(EmitterCategory::PointObstacle),
        _ => None,
    }
}

fn encode_emitter_category(cat: Option<EmitterCategory>) -> u8 {
    match cat {
        Some(EmitterCategory::LightAircraft) => 1,
        Some(EmitterCategory::SmallAircraft) => 2,
        Some(EmitterCategory::LargeAircraft) => 3,
        Some(EmitterCategory::HighVortexAircraft) => 4,
        Some(EmitterCategory::HeavyAircraft) => 5,
        Some(EmitterCategory::HighPerformanceAircraft) => 6,
        Some(EmitterCategory::Rotorcraft) => 7,
        Some(EmitterCategory::Glider) => 9,
        Some(EmitterCategory::LighterThanAir) => 10,
        Some(EmitterCategory::Parachutist) => 11,
        Some(EmitterCategory::Ultralight) => 12,
        Some(EmitterCategory::UncrewedAircraft) => 14,
        Some(EmitterCategory::SpaceVehicle) => 15,
        Some(EmitterCategory::SurfaceEmergencyVehicle) => 17,
        Some(EmitterCategory::SurfaceServiceVehicle) => 18,
        Some(EmitterCategory::PointObstacle) => 19,
        _ => 0,
    }
}

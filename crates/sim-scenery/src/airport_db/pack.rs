//! The `SIMAPT01` binary airport pack: a 24-byte header (magic, version,
//! record count, payload length, CRC-32 of the payload) then the records,
//! little-endian.
//!
//! Coordinates are stored as signed 1e-7 degree integers (about a
//! centimetre). A magnetic heading equal to the one the airport's variation
//! gives, a 3° glide path and an undisplaced threshold are left implied;
//! the facility code is rebuilt from the primary code.

mod reader;

use flightbag::AuthoritySource;

use super::model::{
    AirportFrequency, AirportRecord, CompactCode, RunwayEndRecord, RunwayRecord,
    magnetic_heading_deg,
};
use crate::SceneryError;
pub(crate) use reader::decode_records;

/// The first eight bytes of every pack.
pub(crate) const MAGIC: &[u8; 8] = b"SIMAPT01";
/// The layout version this codec writes and reads.
pub(crate) const VERSION: u16 = 1;
/// Coordinate units per degree.
pub(crate) const COORD_SCALE: f64 = 1e7;
/// Glide path left implied, degrees.
pub(crate) const IMPLIED_GLIDEPATH_DEG: f32 = 3.0;

pub(crate) const AIRPORT_TOWERED: u8 = 1;
pub(crate) const AIRPORT_HARD: u8 = 1 << 1;
pub(crate) const RUNWAY_LIGHTED: u8 = 1;
pub(crate) const RUNWAY_ID_IMPLIED: u8 = 1 << 1;
pub(crate) const END_ILS: u8 = 1;
pub(crate) const END_VGSI: u8 = 1 << 1;
pub(crate) const END_LIGHTS: u8 = 1 << 2;
pub(crate) const END_DISPLACED: u8 = 1 << 3;
pub(crate) const END_GLIDEPATH: u8 = 1 << 4;
pub(crate) const END_MAGNETIC: u8 = 1 << 5;

/// The authority byte of `source`.
pub(crate) fn authority_byte(source: AuthoritySource) -> u8 {
    match source {
        AuthoritySource::FaaNasr => 0,
        AuthoritySource::FaaCifpArinc424 => 1,
        AuthoritySource::FaaDtpp => 2,
        AuthoritySource::OpenStreetMap => 3,
        AuthoritySource::NaturalEarth => 4,
        AuthoritySource::RegionalAip => 5,
        _ => u8::MAX,
    }
}

/// The authority a byte names.
pub(crate) fn authority_of(byte: u8) -> Option<AuthoritySource> {
    Some(match byte {
        0 => AuthoritySource::FaaNasr,
        1 => AuthoritySource::FaaCifpArinc424,
        2 => AuthoritySource::FaaDtpp,
        3 => AuthoritySource::OpenStreetMap,
        4 => AuthoritySource::NaturalEarth,
        5 => AuthoritySource::RegionalAip,
        _ => return None,
    })
}

/// Entry `i` of a 256-entry table; a `u8` index never misses.
fn at(table: &[u32; 256], i: u8) -> u32 {
    table.get(usize::from(i)).copied().unwrap_or(0)
}

/// The slicing-by-8 tables: `[k][n]` is the CRC of byte `n` followed by `k`
/// zero bytes.
fn crc32_tables() -> [[u32; 256]; 8] {
    let base: [u32; 256] = std::array::from_fn(|n| {
        (0..8).fold(n as u32, |c, _| {
            if c & 1 == 1 {
                0xEDB8_8320 ^ (c >> 1)
            } else {
                c >> 1
            }
        })
    });
    let mut tables = [base; 8];
    let mut previous = base;
    for table in tables.iter_mut().skip(1) {
        let next: [u32; 256] = std::array::from_fn(|n| {
            let p = previous.get(n).copied().unwrap_or(0);
            (p >> 8) ^ at(&base, p as u8)
        });
        *table = next;
        previous = next;
    }
    tables
}

/// IEEE CRC-32 (reflected, polynomial `0xEDB88320`), eight bytes a step.
pub(crate) fn crc32(bytes: &[u8]) -> u32 {
    let [t0, t1, t2, t3, t4, t5, t6, t7] = crc32_tables();
    let mut crc = u32::MAX;
    let (chunks, tail) = bytes.as_chunks::<8>();
    for &[b0, b1, b2, b3, b4, b5, b6, b7] in chunks {
        let low = crc ^ u32::from_le_bytes([b0, b1, b2, b3]);
        let [l0, l1, l2, l3] = low.to_le_bytes();
        crc = at(&t7, l0)
            ^ at(&t6, l1)
            ^ at(&t5, l2)
            ^ at(&t4, l3)
            ^ at(&t3, b4)
            ^ at(&t2, b5)
            ^ at(&t1, b6)
            ^ at(&t0, b7);
    }
    for &byte in tail {
        crc = at(&t0, (crc as u8) ^ byte) ^ (crc >> 8);
    }
    !crc
}

/// Encodes `records` as a pack.
pub(crate) fn encode_records(records: &[AirportRecord]) -> Result<Vec<u8>, SceneryError> {
    let mut payload = Writer::default();
    for record in records {
        payload.airport(record)?;
    }
    let count = u32::try_from(records.len()).map_err(|_| too_many("records", records.len()))?;
    let length =
        u32::try_from(payload.0.len()).map_err(|_| too_many("payload bytes", payload.0.len()))?;
    let mut out = Writer(Vec::with_capacity(payload.0.len().saturating_add(24)));
    out.0.extend_from_slice(MAGIC);
    out.u16(VERSION);
    out.u16(0);
    out.u32(count);
    out.u32(length);
    out.u32(crc32(&payload.0));
    out.0.extend_from_slice(&payload.0);
    Ok(out.0)
}

fn too_many(what: &str, count: usize) -> SceneryError {
    SceneryError::InvalidCatalog {
        detail: format!("airport pack cannot hold {count} {what}"),
    }
}

#[derive(Default)]
struct Writer(Vec<u8>);

impl Writer {
    fn u8(&mut self, value: u8) {
        self.0.push(value);
    }
    fn u16(&mut self, value: u16) {
        self.0.extend_from_slice(&value.to_le_bytes());
    }
    fn u32(&mut self, value: u32) {
        self.0.extend_from_slice(&value.to_le_bytes());
    }
    fn f32(&mut self, value: f32) {
        self.0.extend_from_slice(&value.to_le_bytes());
    }
    fn coord(&mut self, degrees: f64) {
        let units = (degrees * COORD_SCALE)
            .round()
            .clamp(f64::from(i32::MIN), f64::from(i32::MAX));
        self.0.extend_from_slice(&(units as i32).to_le_bytes());
    }
    fn code(&mut self, code: Option<CompactCode>) {
        let text = code.as_ref().map_or("", CompactCode::as_str);
        self.u8(text.len() as u8);
        self.0.extend_from_slice(text.as_bytes());
    }
    fn text(&mut self, text: &str) -> Result<(), SceneryError> {
        let length = u16::try_from(text.len()).map_err(|_| too_many("text bytes", text.len()))?;
        self.u16(length);
        self.0.extend_from_slice(text.as_bytes());
        Ok(())
    }

    fn airport(&mut self, record: &AirportRecord) -> Result<(), SceneryError> {
        let ids = &record.ids;
        self.code(Some(ids.primary));
        for code in [ids.icao, ids.faa_lid, ids.iata] {
            self.code(code);
        }
        self.text(&record.name)?;
        self.text(&record.municipality)?;
        self.0.extend_from_slice(&record.country_iso2);
        self.code(Some(record.region_code));
        self.coord(record.lat_deg);
        self.coord(record.lon_deg);
        self.f32(record.elevation_msl_m);
        self.f32(record.mag_var_deg);
        let towered = if record.towered { AIRPORT_TOWERED } else { 0 };
        self.u8(towered | if record.hard_surface { AIRPORT_HARD } else { 0 });
        self.u8(record.tier as u8);
        self.f32(record.longest_runway_m);
        self.u8(authority_byte(record.authority));
        for part in record.effective_ymd {
            self.u16(part);
        }
        let runways = u8::try_from(record.runways.len())
            .map_err(|_| too_many("runways", record.runways.len()))?;
        self.u8(runways);
        for runway in &record.runways {
            self.runway(runway, record.mag_var_deg);
        }
        let frequencies = u8::try_from(record.frequencies.len())
            .map_err(|_| too_many("frequencies", record.frequencies.len()))?;
        self.u8(frequencies);
        for AirportFrequency { role, khz } in &record.frequencies {
            self.u8(*role as u8);
            self.u32(*khz);
        }
        Ok(())
    }

    fn runway(&mut self, runway: &RunwayRecord, mag_var_deg: f32) {
        let [a, b] = &runway.ends;
        let implied = CompactCode::joined(a.designator, b.designator) == Some(runway.id);
        let lighted = if runway.lighted { RUNWAY_LIGHTED } else { 0 };
        self.u8(lighted | if implied { RUNWAY_ID_IMPLIED } else { 0 });
        if !implied {
            self.code(Some(runway.id));
        }
        self.f32(runway.length_m);
        self.f32(runway.width_m);
        self.u8(runway.surface as u8);
        for end in &runway.ends {
            self.end(end, mag_var_deg);
        }
    }

    fn end(&mut self, end: &RunwayEndRecord, mag_var_deg: f32) {
        self.code(Some(end.designator));
        self.coord(end.lat_deg);
        self.coord(end.lon_deg);
        self.f32(end.elevation_msl_m);
        self.f32(end.true_heading_deg);
        let derived = magnetic_heading_deg(end.true_heading_deg, mag_var_deg);
        let explicit_magnetic = end.magnetic_heading_deg.to_bits() != derived.to_bits();
        let displaced = end.displaced_threshold_m.to_bits() != 0.0_f32.to_bits();
        let glidepath = end.glidepath_deg.to_bits() != IMPLIED_GLIDEPATH_DEG.to_bits();
        let flags = [
            (end.has_ils, END_ILS),
            (end.has_vgsi, END_VGSI),
            (end.has_approach_lights, END_LIGHTS),
            (displaced, END_DISPLACED),
            (glidepath, END_GLIDEPATH),
            (explicit_magnetic, END_MAGNETIC),
        ]
        .into_iter()
        .fold(0, |flags, (set, bit)| if set { flags | bit } else { flags });
        self.u8(flags);
        if displaced {
            self.f32(end.displaced_threshold_m);
        }
        if glidepath {
            self.f32(end.glidepath_deg);
        }
        if explicit_magnetic {
            self.f32(end.magnetic_heading_deg);
        }
    }
}

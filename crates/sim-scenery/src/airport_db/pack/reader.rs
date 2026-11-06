//! Decoding a `SIMAPT01` pack with a bounds-checked cursor.

use super::{
    AIRPORT_HARD, AIRPORT_TOWERED, COORD_SCALE, END_DISPLACED, END_GLIDEPATH, END_ILS, END_LIGHTS,
    END_MAGNETIC, END_VGSI, IMPLIED_GLIDEPATH_DEG, MAGIC, RUNWAY_ID_IMPLIED, RUNWAY_LIGHTED,
    VERSION, authority_of, crc32,
};
use crate::SceneryError;
use crate::airport_db::model::{
    AerodromeTier, AirportFrequency, AirportIds, AirportRecord, CompactCode, FrequencyRole,
    RunwayEndRecord, RunwayRecord, RunwaySurfaceKind, facility_code_of, magnetic_heading_deg,
};

fn corrupt(detail: impl Into<String>) -> SceneryError {
    SceneryError::InvalidCatalog {
        detail: format!("airport pack: {}", detail.into()),
    }
}

/// Decodes every record of a pack, checking its magic, version, length and
/// CRC-32 first.
pub(crate) fn decode_records(bytes: &[u8]) -> Result<Vec<AirportRecord>, SceneryError> {
    let mut header = Reader(bytes);
    if header.take(MAGIC.len())? != MAGIC.as_slice() {
        return Err(corrupt("bad magic"));
    }
    let version = header.u16()?;
    if version != VERSION {
        return Err(corrupt(format!("version {version}, expected {VERSION}")));
    }
    header.u16()?;
    let count = header.u32()?;
    let length = usize::try_from(header.u32()?).map_err(|_| corrupt("payload too long"))?;
    let crc = header.u32()?;
    let payload = header.take(length)?;
    if !header.0.is_empty() {
        return Err(corrupt(format!("{} trailing bytes", header.0.len())));
    }
    if crc32(payload) != crc {
        return Err(corrupt("CRC-32 mismatch"));
    }
    let mut reader = Reader(payload);
    let mut records = Vec::with_capacity(usize::try_from(count).unwrap_or(0).min(payload.len()));
    for index in 0..count {
        records.push(
            reader
                .airport()
                .map_err(|err| corrupt(format!("record {index}: {err}")))?,
        );
    }
    if !reader.0.is_empty() {
        return Err(corrupt("payload longer than its records"));
    }
    Ok(records)
}

struct Reader<'a>(&'a [u8]);

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], SceneryError> {
        let (head, rest) = self
            .0
            .split_at_checked(n)
            .ok_or_else(|| corrupt("truncated"))?;
        self.0 = rest;
        Ok(head)
    }
    fn array<const N: usize>(&mut self) -> Result<[u8; N], SceneryError> {
        self.take(N)?.try_into().map_err(|_| corrupt("truncated"))
    }
    fn u8(&mut self) -> Result<u8, SceneryError> {
        let [byte] = self.array::<1>()?;
        Ok(byte)
    }
    fn u16(&mut self) -> Result<u16, SceneryError> {
        Ok(u16::from_le_bytes(self.array()?))
    }
    fn u32(&mut self) -> Result<u32, SceneryError> {
        Ok(u32::from_le_bytes(self.array()?))
    }
    fn f32(&mut self) -> Result<f32, SceneryError> {
        Ok(f32::from_le_bytes(self.array()?))
    }
    fn coord(&mut self) -> Result<f64, SceneryError> {
        Ok(f64::from(i32::from_le_bytes(self.array()?)) / COORD_SCALE)
    }
    fn code(&mut self) -> Result<Option<CompactCode>, SceneryError> {
        let length = usize::from(self.u8()?);
        if length == 0 {
            return Ok(None);
        }
        let text =
            std::str::from_utf8(self.take(length)?).map_err(|_| corrupt("code not UTF-8"))?;
        CompactCode::parse(text)
            .map(Some)
            .ok_or_else(|| corrupt(format!("bad code {text:?}")))
    }
    fn required_code(&mut self) -> Result<CompactCode, SceneryError> {
        self.code()?.ok_or_else(|| corrupt("empty code"))
    }
    fn text(&mut self) -> Result<Box<str>, SceneryError> {
        let length = usize::from(self.u16()?);
        let bytes = self.take(length)?;
        std::str::from_utf8(bytes)
            .map(Box::from)
            .map_err(|_| corrupt("text not UTF-8"))
    }

    fn airport(&mut self) -> Result<AirportRecord, SceneryError> {
        let primary = self.required_code()?;
        let (icao, faa_lid, iata) = (self.code()?, self.code()?, self.code()?);
        let name = self.text()?;
        let municipality = self.text()?;
        let country_iso2 = self.array::<2>()?;
        let region_code = self.required_code()?;
        let (lat_deg, lon_deg) = (self.coord()?, self.coord()?);
        let (elevation_msl_m, mag_var_deg) = (self.f32()?, self.f32()?);
        let flags = self.u8()?;
        let tier = AerodromeTier::from_u8(self.u8()?).ok_or_else(|| corrupt("bad tier"))?;
        let longest_runway_m = self.f32()?;
        let authority = authority_of(self.u8()?).ok_or_else(|| corrupt("bad authority"))?;
        let effective_ymd = [self.u16()?, self.u16()?, self.u16()?];
        let runway_count = self.u8()?;
        let runways = (0..runway_count)
            .map(|_| self.runway(mag_var_deg))
            .collect::<Result<Box<[_]>, _>>()?;
        let frequency_count = self.u8()?;
        let frequencies = (0..frequency_count)
            .map(|_| {
                let role = FrequencyRole::from_u8(self.u8()?)
                    .ok_or_else(|| corrupt("bad frequency role"))?;
                Ok(AirportFrequency {
                    role,
                    khz: self.u32()?,
                })
            })
            .collect::<Result<Box<[_]>, SceneryError>>()?;
        Ok(AirportRecord {
            ids: AirportIds {
                primary,
                icao,
                faa_lid,
                iata,
            },
            facility: facility_code_of(primary)
                .ok_or_else(|| corrupt(format!("no facility form of {primary}")))?,
            name,
            municipality,
            country_iso2,
            region_code,
            lat_deg,
            lon_deg,
            elevation_msl_m,
            mag_var_deg,
            towered: flags & AIRPORT_TOWERED != 0,
            tier,
            longest_runway_m,
            hard_surface: flags & AIRPORT_HARD != 0,
            authority,
            effective_ymd,
            runways,
            frequencies,
        })
    }

    fn runway(&mut self, mag_var_deg: f32) -> Result<RunwayRecord, SceneryError> {
        let flags = self.u8()?;
        let explicit_id = if flags & RUNWAY_ID_IMPLIED == 0 {
            Some(self.required_code()?)
        } else {
            None
        };
        let (length_m, width_m) = (self.f32()?, self.f32()?);
        let surface =
            RunwaySurfaceKind::from_u8(self.u8()?).ok_or_else(|| corrupt("bad surface"))?;
        let ends = [self.end(mag_var_deg)?, self.end(mag_var_deg)?];
        let [a, b] = &ends;
        let id = match explicit_id {
            Some(id) => id,
            None => CompactCode::joined(a.designator, b.designator)
                .ok_or_else(|| corrupt("runway id does not fit"))?,
        };
        Ok(RunwayRecord {
            id,
            length_m,
            width_m,
            surface,
            lighted: flags & RUNWAY_LIGHTED != 0,
            ends,
        })
    }

    fn end(&mut self, mag_var_deg: f32) -> Result<RunwayEndRecord, SceneryError> {
        let designator = self.required_code()?;
        let (lat_deg, lon_deg) = (self.coord()?, self.coord()?);
        let (elevation_msl_m, true_heading_deg) = (self.f32()?, self.f32()?);
        let flags = self.u8()?;
        let mut optional = |bit: u8, implied: f32| -> Result<f32, SceneryError> {
            if flags & bit != 0 {
                self.f32()
            } else {
                Ok(implied)
            }
        };
        let displaced_threshold_m = optional(END_DISPLACED, 0.0)?;
        let glidepath_deg = optional(END_GLIDEPATH, IMPLIED_GLIDEPATH_DEG)?;
        let magnetic_heading_deg = optional(
            END_MAGNETIC,
            magnetic_heading_deg(true_heading_deg, mag_var_deg),
        )?;
        Ok(RunwayEndRecord {
            designator,
            lat_deg,
            lon_deg,
            elevation_msl_m,
            displaced_threshold_m,
            true_heading_deg,
            magnetic_heading_deg,
            glidepath_deg,
            has_ils: flags & END_ILS != 0,
            has_vgsi: flags & END_VGSI != 0,
            has_approach_lights: flags & END_LIGHTS != 0,
        })
    }
}

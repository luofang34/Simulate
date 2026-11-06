//! The FAA NASR 28-day subscription CSVs (`APT_BASE`, `APT_RWY`,
//! `APT_RWY_END`, `FRQ`) read into airport records.
//!
//! Kept: operational (`ARPT_STATUS = O`) airports (`SITE_TYPE_CODE = A`)
//! open to the public (`FACILITY_USE_CODE = PU`), plus towered military and
//! Coast Guard fields; runways of at least 800 ft named `A/B` whose two
//! ends both carry surveyed coordinates. Distances and elevations are
//! published in feet and stored in metres.

use std::collections::HashMap;

use flightbag::AuthoritySource;

use super::assemble::{
    FEET_TO_M, ListedFrequency, MIN_RUNWAY_FT, RecordParts, SurveyedEnd, SurveyedRunway,
    khz_of_mhz, parse_ymd,
};
use super::csv::{CsvHeader, CsvReader, field, number};
use super::model::{AirportIds, AirportRecord, CompactCode, FrequencyRole, RunwaySurfaceKind};
use crate::SceneryError;

/// The NASR CSV texts of one edition.
#[derive(Debug, Clone, Copy)]
pub struct FaaNasrCsv<'a> {
    /// `APT_BASE.csv`: one row per landing facility.
    pub apt_base: &'a str,
    /// `APT_RWY.csv`: one row per runway.
    pub apt_rwy: &'a str,
    /// `APT_RWY_END.csv`: one row per runway end.
    pub apt_rwy_end: &'a str,
    /// `FRQ.csv`, when frequencies are wanted.
    pub frq: Option<&'a str>,
}

/// Parses one NASR edition into records with
/// [`AuthoritySource::FaaNasr`], in `APT_BASE` order, each dated by its
/// `EFF_DATE`.
pub fn parse_faa_nasr(csv: &FaaNasrCsv<'_>) -> Result<Vec<AirportRecord>, SceneryError> {
    let ends = read_runway_ends(csv.apt_rwy_end)?;
    let runways = read_runways(csv.apt_rwy, &ends)?;
    let frequencies = match csv.frq {
        Some(text) => read_frequencies(text)?,
        None => HashMap::new(),
    };
    read_airports(csv.apt_base, &runways, &frequencies)
}

fn feet(fields: &[String], index: usize) -> Option<f32> {
    number(fields, index).map(|ft| (ft * FEET_TO_M) as f32)
}

/// Runway ends keyed by site, runway and end designator.
type EndIndex = HashMap<(String, String, CompactCode), SurveyedEnd>;

fn read_runway_ends(text: &str) -> Result<EndIndex, SceneryError> {
    let mut reader = CsvReader::new("APT_RWY_END.csv", text);
    let header = CsvHeader::read(&mut reader)?;
    let [site, rwy, end_id, lat, lon, elev] = header.columns([
        "SITE_NO",
        "RWY_ID",
        "RWY_END_ID",
        "LAT_DECIMAL",
        "LONG_DECIMAL",
        "RWY_END_ELEV",
    ])?;
    let [displaced, glidepath, ils, vgsi, lights] = header.columns([
        "DISPLACED_THR_LEN",
        "VISUAL_GLIDE_PATH_ANGLE",
        "ILS_TYPE",
        "VGSI_CODE",
        "APCH_LGT_SYSTEM_CODE",
    ])?;
    let mut ends = HashMap::new();
    let mut row = Vec::new();
    while reader.next_record(&mut row)? {
        let (Some(designator), Some(lat_deg), Some(lon_deg)) = (
            CompactCode::parse(field(&row, end_id)),
            number(&row, lat),
            number(&row, lon),
        ) else {
            continue;
        };
        let present = |index: usize| !matches!(field(&row, index), "" | "N" | "NONE" | "NSTD");
        let end = SurveyedEnd {
            designator,
            lat_deg,
            lon_deg,
            elevation_msl_m: feet(&row, elev),
            displaced_threshold_m: feet(&row, displaced).unwrap_or(0.0),
            glidepath_deg: number(&row, glidepath)
                .map(|deg| deg as f32)
                .filter(|deg| *deg > 0.0),
            has_ils: present(ils),
            has_vgsi: present(vgsi),
            has_approach_lights: present(lights),
        };
        let key = (
            field(&row, site).to_owned(),
            field(&row, rwy).to_owned(),
            designator,
        );
        ends.insert(key, end);
    }
    Ok(ends)
}

fn read_runways(
    text: &str,
    ends: &EndIndex,
) -> Result<HashMap<String, Vec<SurveyedRunway>>, SceneryError> {
    let mut reader = CsvReader::new("APT_RWY.csv", text);
    let header = CsvHeader::read(&mut reader)?;
    let [site, rwy, len, width, surface, lights] = header.columns([
        "SITE_NO",
        "RWY_ID",
        "RWY_LEN",
        "RWY_WIDTH",
        "SURFACE_TYPE_CODE",
        "RWY_LGT_CODE",
    ])?;
    let mut runways: HashMap<String, Vec<SurveyedRunway>> = HashMap::new();
    let mut row = Vec::new();
    while reader.next_record(&mut row)? {
        let rwy_id = field(&row, rwy);
        let long_enough = number(&row, len).is_some_and(|ft| ft >= MIN_RUNWAY_FT);
        let (Some(id), Some((a, b))) = (CompactCode::parse(rwy_id), rwy_id.split_once('/')) else {
            continue;
        };
        let site_no = field(&row, site);
        let end = |designator: &str| {
            let designator = CompactCode::parse(designator)?;
            ends.get(&(site_no.to_owned(), rwy_id.to_owned(), designator))
                .copied()
        };
        let (Some(end_a), Some(end_b)) = (end(a), end(b)) else {
            continue;
        };
        if !long_enough {
            continue;
        }
        runways
            .entry(site_no.to_owned())
            .or_default()
            .push(SurveyedRunway {
                id,
                length_m: feet(&row, len).unwrap_or(0.0),
                width_m: feet(&row, width).unwrap_or(0.0),
                surface: RunwaySurfaceKind::from_code(field(&row, surface)),
                lighted: !matches!(field(&row, lights), "" | "NONE"),
                ends: [end_a, end_b],
            });
    }
    Ok(runways)
}

/// The roles one `FREQ_USE` entry (`"APCH/P DEP/P"`, `"LCL/S"`) names,
/// each with whether it is a secondary.
fn frequency_roles(usage: &str) -> Vec<(FrequencyRole, bool)> {
    usage
        .split_whitespace()
        .filter_map(|token| {
            let (name, secondary) = match token.split_once('/') {
                Some((name, suffix)) => (name, suffix.starts_with('S')),
                None => (token, false),
            };
            let role = match name {
                "LCL" => FrequencyRole::Tower,
                "GND" => FrequencyRole::Ground,
                "APCH" => FrequencyRole::Approach,
                "DEP" => FrequencyRole::Departure,
                "CTAF" => FrequencyRole::Ctaf,
                "UNICOM" => FrequencyRole::Unicom,
                "ATIS" | "D-ATIS" => FrequencyRole::Atis,
                "AWOS" | "ASOS" => FrequencyRole::Awos,
                _ if name.starts_with("AWOS") || name.starts_with("ASOS") => FrequencyRole::Awos,
                _ => return None,
            };
            Some((role, secondary))
        })
        .collect()
}

fn read_frequencies(text: &str) -> Result<HashMap<String, Vec<ListedFrequency>>, SceneryError> {
    let mut reader = CsvReader::new("FRQ.csv", text);
    let header = CsvHeader::read(&mut reader)?;
    let [facility_type, serviced, site_type, freq, usage] = header.columns([
        "FACILITY_TYPE",
        "SERVICED_FACILITY",
        "SERVICED_SITE_TYPE",
        "FREQ",
        "FREQ_USE",
    ])?;
    let mut out: HashMap<String, Vec<ListedFrequency>> = HashMap::new();
    let mut row = Vec::new();
    while reader.next_record(&mut row)? {
        if field(&row, site_type) != "AIRPORT" || field(&row, facility_type) == "NAVAID" {
            continue;
        }
        let Some(khz) = khz_of_mhz(field(&row, freq)) else {
            continue;
        };
        for (role, secondary) in frequency_roles(field(&row, usage)) {
            out.entry(field(&row, serviced).to_owned())
                .or_default()
                .push(ListedFrequency {
                    role,
                    secondary,
                    khz,
                });
        }
    }
    Ok(out)
}

/// Whether an `APT_BASE` row is a kept airport.
fn kept(site_type: &str, status: &str, facility_use: &str, ownership: &str, tower: &str) -> bool {
    let military = matches!(ownership, "MA" | "MN" | "MR" | "CG") && tower.starts_with("ATCT");
    site_type == "A" && status == "O" && (facility_use == "PU" || military)
}

fn read_airports(
    text: &str,
    runways: &HashMap<String, Vec<SurveyedRunway>>,
    frequencies: &HashMap<String, Vec<ListedFrequency>>,
) -> Result<Vec<AirportRecord>, SceneryError> {
    let mut reader = CsvReader::new("APT_BASE.csv", text);
    let header = CsvHeader::read(&mut reader)?;
    let [eff, site, site_type, state, arpt_id, city, country] = header.columns([
        "EFF_DATE",
        "SITE_NO",
        "SITE_TYPE_CODE",
        "STATE_CODE",
        "ARPT_ID",
        "CITY",
        "COUNTRY_CODE",
    ])?;
    let [name, facility_use, ownership, lat, lon, elev, var, hemis] = header.columns([
        "ARPT_NAME",
        "FACILITY_USE_CODE",
        "OWNERSHIP_TYPE_CODE",
        "LAT_DECIMAL",
        "LONG_DECIMAL",
        "ELEV",
        "MAG_VARN",
        "MAG_HEMIS",
    ])?;
    let [status, tower, icao] = header.columns(["ARPT_STATUS", "TWR_TYPE_CODE", "ICAO_ID"])?;
    let mut records = Vec::new();
    let mut row = Vec::new();
    while reader.next_record(&mut row)? {
        let tower_type = field(&row, tower);
        let selected = kept(
            field(&row, site_type),
            field(&row, status),
            field(&row, facility_use),
            field(&row, ownership),
            tower_type,
        );
        let site_runways = runways.get(field(&row, site));
        let ids = AirportIds::new(
            CompactCode::parse(field(&row, icao)),
            CompactCode::parse(field(&row, arpt_id)),
            None,
        );
        let (true, Some(site_runways), Some(ids), Some(effective_ymd)) =
            (selected, site_runways, ids, parse_ymd(field(&row, eff)))
        else {
            continue;
        };
        let variation = number(&row, var).unwrap_or(0.0) as f32;
        let parts = RecordParts {
            ids,
            name: field(&row, name).to_owned(),
            municipality: field(&row, city).to_owned(),
            country: field(&row, country).to_owned(),
            region: format!("US-{}", field(&row, state)),
            lat_deg: number(&row, lat).unwrap_or(f64::NAN),
            lon_deg: number(&row, lon).unwrap_or(f64::NAN),
            elevation_msl_m: feet(&row, elev).unwrap_or(f32::NAN),
            mag_var_deg: if field(&row, hemis) == "W" {
                -variation
            } else {
                variation
            },
            towered: tower_type.starts_with("ATCT"),
            authority: AuthoritySource::FaaNasr,
            effective_ymd,
            runways: site_runways.clone(),
            frequencies: frequencies
                .get(field(&row, arpt_id))
                .cloned()
                .unwrap_or_default(),
        };
        records.extend(parts.finish());
    }
    Ok(records)
}

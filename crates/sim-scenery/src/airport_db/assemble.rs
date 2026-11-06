//! Pieces both source parsers assemble records from: surveyed runway ends,
//! frequency lists and the finished record.

use flightbag::AuthoritySource;

use super::geometry::bearing_and_distance;
use super::model::{
    AerodromeTier, AirportFrequency, AirportIds, AirportRecord, CompactCode, FrequencyRole,
    RunwayEndRecord, RunwayRecord, RunwaySurfaceKind, facility_code_of, magnetic_heading_deg,
};

/// Feet to metres.
pub(crate) const FEET_TO_M: f64 = 0.3048;
/// Runways shorter than this are left out, feet.
pub(crate) const MIN_RUNWAY_FT: f64 = 800.0;
/// Glide path assumed where none is published, degrees.
const DEFAULT_GLIDEPATH_DEG: f32 = 3.0;
/// The VHF band frequencies are kept from, kilohertz.
const VHF_KHZ: std::ops::RangeInclusive<u32> = 108_000..=137_000;
/// Most frequencies kept per airport.
const MAX_FREQUENCIES: usize = 48;

/// One runway end as a source lists it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SurveyedEnd {
    pub(crate) designator: CompactCode,
    pub(crate) lat_deg: f64,
    pub(crate) lon_deg: f64,
    pub(crate) elevation_msl_m: Option<f32>,
    pub(crate) displaced_threshold_m: f32,
    pub(crate) glidepath_deg: Option<f32>,
    pub(crate) has_ils: bool,
    pub(crate) has_vgsi: bool,
    pub(crate) has_approach_lights: bool,
}

/// One runway as a source lists it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SurveyedRunway {
    pub(crate) id: CompactCode,
    pub(crate) length_m: f32,
    pub(crate) width_m: f32,
    pub(crate) surface: RunwaySurfaceKind,
    pub(crate) lighted: bool,
    pub(crate) ends: [SurveyedEnd; 2],
}

impl SurveyedRunway {
    /// The runway record, headings measured between the surveyed ends and
    /// missing end elevations taken from the field.
    pub(crate) fn record(&self, field_msl_m: f32, mag_var_deg: f32) -> RunwayRecord {
        let [a, b] = self.ends;
        RunwayRecord {
            id: self.id,
            length_m: self.length_m,
            width_m: self.width_m,
            surface: self.surface,
            lighted: self.lighted,
            ends: [
                end_record(&a, &b, field_msl_m, mag_var_deg),
                end_record(&b, &a, field_msl_m, mag_var_deg),
            ],
        }
    }
}

fn end_record(
    end: &SurveyedEnd,
    other: &SurveyedEnd,
    field_msl_m: f32,
    mag_var_deg: f32,
) -> RunwayEndRecord {
    let (bearing, _) =
        bearing_and_distance((end.lat_deg, end.lon_deg), (other.lat_deg, other.lon_deg));
    let true_heading_deg = bearing.to_degrees() as f32;
    RunwayEndRecord {
        designator: end.designator,
        lat_deg: end.lat_deg,
        lon_deg: end.lon_deg,
        elevation_msl_m: end.elevation_msl_m.unwrap_or(field_msl_m),
        displaced_threshold_m: end.displaced_threshold_m,
        true_heading_deg,
        magnetic_heading_deg: magnetic_heading_deg(true_heading_deg, mag_var_deg),
        glidepath_deg: end.glidepath_deg.unwrap_or(DEFAULT_GLIDEPATH_DEG),
        has_ils: end.has_ils,
        has_vgsi: end.has_vgsi,
        has_approach_lights: end.has_approach_lights,
    }
}

/// One frequency as a source lists it; `secondary` sorts after the
/// role's primary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct ListedFrequency {
    pub(crate) role: FrequencyRole,
    pub(crate) secondary: bool,
    pub(crate) khz: u32,
}

/// Kilohertz of a frequency written in megahertz (`"118.3"`).
pub(crate) fn khz_of_mhz(text: &str) -> Option<u32> {
    let mhz = text.trim().parse::<f64>().ok()?;
    let khz = (mhz * 1_000.0).round();
    (khz.is_finite() && khz >= 0.0 && khz <= f64::from(u32::MAX)).then_some(khz as u32)
}

/// Frequencies sorted by role, each role's primary first, VHF only, with
/// repeats of a role and frequency dropped.
pub(crate) fn frequency_list(mut listed: Vec<ListedFrequency>) -> Box<[AirportFrequency]> {
    listed.sort();
    let mut out: Vec<AirportFrequency> = Vec::new();
    for entry in listed {
        if !VHF_KHZ.contains(&entry.khz) {
            continue;
        }
        let frequency = AirportFrequency {
            role: entry.role,
            khz: entry.khz,
        };
        if !out.contains(&frequency) && out.len() < MAX_FREQUENCIES {
            out.push(frequency);
        }
    }
    out.into_boxed_slice()
}

/// What a record is made of, before classification.
pub(crate) struct RecordParts {
    pub(crate) ids: AirportIds,
    pub(crate) name: String,
    pub(crate) municipality: String,
    pub(crate) country: String,
    pub(crate) region: String,
    pub(crate) lat_deg: f64,
    pub(crate) lon_deg: f64,
    pub(crate) elevation_msl_m: f32,
    pub(crate) mag_var_deg: f32,
    pub(crate) towered: bool,
    pub(crate) authority: AuthoritySource,
    pub(crate) effective_ymd: [u16; 3],
    pub(crate) runways: Vec<SurveyedRunway>,
    pub(crate) frequencies: Vec<ListedFrequency>,
}

impl RecordParts {
    /// The classified record; `None` without a usable runway, a facility
    /// form of the primary code, or a finite position.
    pub(crate) fn finish(self) -> Option<AirportRecord> {
        let usable = self.lat_deg.is_finite()
            && self.lon_deg.is_finite()
            && self.lat_deg.abs() <= 90.0
            && self.elevation_msl_m.is_finite();
        if !usable || self.runways.is_empty() {
            return None;
        }
        let facility = facility_code_of(self.ids.primary)?;
        let country = self.country.as_bytes();
        let country_iso2 = [
            country
                .first()
                .copied()
                .unwrap_or(b'?')
                .to_ascii_uppercase(),
            country.get(1).copied().unwrap_or(b'?').to_ascii_uppercase(),
        ];
        let region_code = CompactCode::parse(&self.region)
            .or_else(|| CompactCode::parse(&self.country))
            .or_else(|| CompactCode::parse("ZZ"))?;
        let runways = self
            .runways
            .iter()
            .map(|runway| runway.record(self.elevation_msl_m, self.mag_var_deg))
            .collect();
        let mut record = AirportRecord {
            ids: self.ids,
            facility,
            name: self.name.trim().into(),
            municipality: self.municipality.trim().into(),
            country_iso2,
            region_code,
            lat_deg: self.lat_deg,
            lon_deg: self.lon_deg,
            elevation_msl_m: self.elevation_msl_m,
            mag_var_deg: self.mag_var_deg,
            towered: self.towered,
            tier: AerodromeTier::Tier4Minor,
            longest_runway_m: 0.0,
            hard_surface: false,
            authority: self.authority,
            effective_ymd: self.effective_ymd,
            runways,
            frequencies: frequency_list(self.frequencies),
        };
        record.classify();
        Some(record)
    }
}

/// `"2026/10/29"` (or `2026-10-29`) as `[2026, 10, 29]`.
pub(crate) fn parse_ymd(text: &str) -> Option<[u16; 3]> {
    let mut parts = text
        .trim()
        .split(['/', '-'])
        .map(|part| part.parse::<u16>().ok());
    let [year, month, day] = [parts.next()??, parts.next()??, parts.next()??];
    let valid = parts.next().is_none() && (1..=12).contains(&month) && (1..=31).contains(&day);
    valid.then_some([year, month, day])
}

/// `NEWARK LIBERTY INTL` as `Newark Liberty Intl`.
pub(crate) fn title_case(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut word_start = true;
    for c in text.chars() {
        if word_start {
            out.extend(c.to_uppercase());
        } else {
            out.extend(c.to_lowercase());
        }
        word_start = !c.is_alphanumeric() && c != '\'';
    }
    out
}

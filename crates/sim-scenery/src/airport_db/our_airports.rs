//! The OurAirports public-domain CSVs (`airports`, `runways`,
//! `airport-frequencies`) read into airport records outside the US.
//!
//! Kept: open `large_airport` and `medium_airport` rows plus
//! [`ICONIC_AIRPORTS`], outside the US, that have a runway of at least
//! 800 ft whose two ends both carry coordinates. OurAirports publishes no
//! magnetic variation, so it is estimated from how far the runway
//! designators sit from the surveyed true headings; it publishes no tower
//! flag, so an airport is towered when it lists a `TWR` frequency.

use std::collections::HashMap;

use flightbag::AuthoritySource;

use super::assemble::{
    FEET_TO_M, ListedFrequency, MIN_RUNWAY_FT, RecordParts, SurveyedEnd, SurveyedRunway, khz_of_mhz,
};
use super::csv::{CsvHeader, CsvReader, field, number};
use super::geometry::bearing_and_distance;
use super::model::{AirportIds, AirportRecord, CompactCode, FrequencyRole, RunwaySurfaceKind};
use crate::SceneryError;

/// Small or short airports kept whatever their OurAirports type, for the
/// approaches or polar stations they are known for.
pub const ICONIC_AIRPORTS: [&str; 13] = [
    "VNLK", "LOWI", "LXGB", "TNCM", "SBRJ", "NZQN", "TFFJ", "VQPR", "LPMA", "LFLJ", "LSZS", "SAWH",
    "NZSP",
];

/// Width assumed where a runway's is unpublished, metres.
const DEFAULT_WIDTH_M: f32 = 30.0;

/// The OurAirports CSV texts of one snapshot.
#[derive(Debug, Clone, Copy)]
pub struct OurAirportsCsv<'a> {
    /// `airports.csv`.
    pub airports: &'a str,
    /// `runways.csv`.
    pub runways: &'a str,
    /// `airport-frequencies.csv`, when frequencies are wanted.
    pub frequencies: Option<&'a str>,
    /// The date the snapshot was taken, `[year, month, day]`; OurAirports
    /// rows carry no edition date of their own.
    pub snapshot_ymd: [u16; 3],
}

/// Display names and the IATA code OurAirports gives a US airport, for
/// enriching FAA records that carry neither.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OurAirportsName {
    /// Airport name in mixed case.
    pub name: String,
    /// City served in mixed case.
    pub municipality: String,
    /// IATA code, when assigned.
    pub iata: Option<CompactCode>,
}

/// Parses the snapshot's kept airports into records with
/// [`AuthoritySource::RegionalAip`], in `airports.csv` order.
pub fn parse_our_airports(csv: &OurAirportsCsv<'_>) -> Result<Vec<AirportRecord>, SceneryError> {
    let runways = read_runways(csv.runways)?;
    let frequencies = match csv.frequencies {
        Some(text) => read_frequencies(text)?,
        None => HashMap::new(),
    };
    let mut reader = CsvReader::new("airports.csv", csv.airports);
    let header = CsvHeader::read(&mut reader)?;
    let columns = AirportColumns::new(&header)?;
    let mut records = Vec::new();
    let mut row = Vec::new();
    while reader.next_record(&mut row)? {
        let kind = field(&row, columns.kind);
        let ident = field(&row, columns.ident);
        let wanted = matches!(kind, "large_airport" | "medium_airport")
            || (kind != "closed" && ICONIC_AIRPORTS.contains(&ident));
        if !wanted || field(&row, columns.country) == "US" {
            continue;
        }
        let Some(site_runways) = runways.get(field(&row, columns.id)) else {
            continue;
        };
        let listed = frequencies
            .get(field(&row, columns.id))
            .cloned()
            .unwrap_or_default();
        records.extend(
            columns
                .parts(&row, site_runways, listed, csv.snapshot_ymd)
                .and_then(RecordParts::finish),
        );
    }
    Ok(records)
}

/// Names and IATA codes of US rows, keyed by every code the row lists
/// (ident, ICAO, GPS and local codes).
pub fn our_airports_us_names(
    airports: &str,
) -> Result<HashMap<CompactCode, OurAirportsName>, SceneryError> {
    let mut reader = CsvReader::new("airports.csv", airports);
    let header = CsvHeader::read(&mut reader)?;
    let columns = AirportColumns::new(&header)?;
    let mut out = HashMap::new();
    let mut row = Vec::new();
    while reader.next_record(&mut row)? {
        if field(&row, columns.country) != "US" || field(&row, columns.kind) == "closed" {
            continue;
        }
        let entry = OurAirportsName {
            name: field(&row, columns.name).to_owned(),
            municipality: field(&row, columns.municipality).to_owned(),
            iata: CompactCode::parse(field(&row, columns.iata)),
        };
        for index in [columns.local, columns.icao, columns.gps, columns.ident] {
            if let Some(code) = CompactCode::parse(field(&row, index)) {
                out.entry(code).or_insert_with(|| entry.clone());
            }
        }
    }
    Ok(out)
}

struct AirportColumns {
    id: usize,
    ident: usize,
    kind: usize,
    name: usize,
    lat: usize,
    lon: usize,
    elevation: usize,
    country: usize,
    region: usize,
    municipality: usize,
    icao: usize,
    iata: usize,
    gps: usize,
    local: usize,
}

impl AirportColumns {
    fn new(header: &CsvHeader) -> Result<Self, SceneryError> {
        let [id, ident, kind, name, lat, lon, elevation] = header.columns([
            "id",
            "ident",
            "type",
            "name",
            "latitude_deg",
            "longitude_deg",
            "elevation_ft",
        ])?;
        let [country, region, municipality, icao, iata, gps, local] = header.columns([
            "iso_country",
            "iso_region",
            "municipality",
            "icao_code",
            "iata_code",
            "gps_code",
            "local_code",
        ])?;
        Ok(Self {
            id,
            ident,
            kind,
            name,
            lat,
            lon,
            elevation,
            country,
            region,
            municipality,
            icao,
            iata,
            gps,
            local,
        })
    }

    fn parts(
        &self,
        row: &[String],
        runways: &[SurveyedRunway],
        frequencies: Vec<ListedFrequency>,
        effective_ymd: [u16; 3],
    ) -> Option<RecordParts> {
        let icao = [self.icao, self.gps, self.ident]
            .into_iter()
            .find_map(|index| {
                CompactCode::parse(field(row, index)).filter(CompactCode::is_icao_like)
            });
        let ident = CompactCode::parse(field(row, self.ident));
        let local = ident.filter(|code| Some(*code) != icao);
        let ids = AirportIds::new(icao, local, CompactCode::parse(field(row, self.iata)))?;
        let elevation_msl_m = number(row, self.elevation)
            .map(|ft| (ft * FEET_TO_M) as f32)
            .or_else(|| {
                runways
                    .iter()
                    .flat_map(|r| r.ends)
                    .find_map(|end| end.elevation_msl_m)
            })?;
        Some(RecordParts {
            ids,
            name: field(row, self.name).to_owned(),
            municipality: field(row, self.municipality).to_owned(),
            country: field(row, self.country).to_owned(),
            region: field(row, self.region).to_owned(),
            lat_deg: number(row, self.lat)?,
            lon_deg: number(row, self.lon)?,
            elevation_msl_m,
            mag_var_deg: estimated_variation_deg(runways),
            towered: frequencies.iter().any(|f| f.role == FrequencyRole::Tower),
            authority: AuthoritySource::RegionalAip,
            effective_ymd,
            runways: runways.to_vec(),
            frequencies,
        })
    }
}

/// Magnetic variation, east positive, as the mean offset of each numbered
/// designator's heading (`07` is 070° magnetic) from the surveyed true
/// heading; zero without a numbered end. Good to a few degrees, since
/// designators round headings to ten.
fn estimated_variation_deg(runways: &[SurveyedRunway]) -> f32 {
    let mut sum = 0.0;
    let mut count = 0_u32;
    for runway in runways {
        let [a, b] = runway.ends;
        for (end, other) in [(a, b), (b, a)] {
            let text = end.designator.as_str();
            if text.ends_with('T') {
                continue;
            }
            let digits: String = text.chars().take_while(char::is_ascii_digit).collect();
            let Some(number) = digits.parse::<u32>().ok().filter(|n| (1..=36).contains(n)) else {
                continue;
            };
            let (bearing, _) =
                bearing_and_distance((end.lat_deg, end.lon_deg), (other.lat_deg, other.lon_deg));
            let offset =
                (bearing.to_degrees() - f64::from(number) * 10.0 + 180.0).rem_euclid(360.0) - 180.0;
            sum += offset;
            count = count.wrapping_add(1);
        }
    }
    if count == 0 {
        0.0
    } else {
        (sum / f64::from(count)) as f32
    }
}

fn read_runways(text: &str) -> Result<HashMap<String, Vec<SurveyedRunway>>, SceneryError> {
    let mut reader = CsvReader::new("runways.csv", text);
    let header = CsvHeader::read(&mut reader)?;
    let [airport, length, width, surface, lighted, closed] = header.columns([
        "airport_ref",
        "length_ft",
        "width_ft",
        "surface",
        "lighted",
        "closed",
    ])?;
    let le = header.columns([
        "le_ident",
        "le_latitude_deg",
        "le_longitude_deg",
        "le_elevation_ft",
        "le_displaced_threshold_ft",
    ])?;
    let he = header.columns([
        "he_ident",
        "he_latitude_deg",
        "he_longitude_deg",
        "he_elevation_ft",
        "he_displaced_threshold_ft",
    ])?;
    let mut out: HashMap<String, Vec<SurveyedRunway>> = HashMap::new();
    let mut row = Vec::new();
    while reader.next_record(&mut row)? {
        let long_enough = number(&row, length).is_some_and(|ft| ft >= MIN_RUNWAY_FT);
        let (Some(a), Some(b)) = (surveyed_end(&row, le), surveyed_end(&row, he)) else {
            continue;
        };
        let id = CompactCode::parse(&format!("{}/{}", a.designator, b.designator));
        let (true, Some(id)) = (long_enough && field(&row, closed) != "1", id) else {
            continue;
        };
        out.entry(field(&row, airport).to_owned())
            .or_default()
            .push(SurveyedRunway {
                id,
                length_m: number(&row, length).map_or(0.0, |ft| (ft * FEET_TO_M) as f32),
                width_m: number(&row, width).map_or(DEFAULT_WIDTH_M, |ft| (ft * FEET_TO_M) as f32),
                surface: RunwaySurfaceKind::from_code(field(&row, surface)),
                lighted: field(&row, lighted) == "1",
                ends: [a, b],
            });
    }
    Ok(out)
}

/// One end from its `[ident, lat, lon, elevation, displaced]` columns.
fn surveyed_end(
    row: &[String],
    [ident, lat, lon, elevation, displaced]: [usize; 5],
) -> Option<SurveyedEnd> {
    Some(SurveyedEnd {
        designator: CompactCode::parse(field(row, ident))?,
        lat_deg: number(row, lat)?,
        lon_deg: number(row, lon)?,
        elevation_msl_m: number(row, elevation).map(|ft| (ft * FEET_TO_M) as f32),
        displaced_threshold_m: number(row, displaced).map_or(0.0, |ft| (ft * FEET_TO_M) as f32),
        glidepath_deg: None,
        has_ils: false,
        has_vgsi: false,
        has_approach_lights: false,
    })
}

fn read_frequencies(text: &str) -> Result<HashMap<String, Vec<ListedFrequency>>, SceneryError> {
    let mut reader = CsvReader::new("airport-frequencies.csv", text);
    let header = CsvHeader::read(&mut reader)?;
    let [airport, kind, mhz] = header.columns(["airport_ref", "type", "frequency_mhz"])?;
    let mut out: HashMap<String, Vec<ListedFrequency>> = HashMap::new();
    let mut row = Vec::new();
    while reader.next_record(&mut row)? {
        let role = match field(&row, kind).to_ascii_uppercase().as_str() {
            "TWR" => FrequencyRole::Tower,
            "GND" => FrequencyRole::Ground,
            "APP" => FrequencyRole::Approach,
            "DEP" => FrequencyRole::Departure,
            "CTAF" => FrequencyRole::Ctaf,
            "UNIC" | "UNICOM" => FrequencyRole::Unicom,
            "ATIS" => FrequencyRole::Atis,
            "AWOS" | "ASOS" => FrequencyRole::Awos,
            _ => continue,
        };
        let Some(khz) = khz_of_mhz(field(&row, mhz)) else {
            continue;
        };
        let secondary = out
            .get(field(&row, airport))
            .is_some_and(|listed| listed.iter().any(|f| f.role == role));
        out.entry(field(&row, airport).to_owned())
            .or_default()
            .push(ListedFrequency {
                role,
                secondary,
                khz,
            });
    }
    Ok(out)
}

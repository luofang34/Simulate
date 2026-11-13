//! A global airport database: FAA NASR airports across the US and its
//! territories, OurAirports airports elsewhere, with surveyed runway ends
//! and frequencies, indexed for globe views, picking and code search.
//!
//! [`AirportDatabase::builtin`] decodes the embedded `SIMAPT01` pack, built
//! from the FAA NASR edition and OurAirports snapshot named in its records.

mod assemble;
mod compile;
mod csv;
mod faa_nasr;
pub(crate) mod geometry;
mod model;
mod our_airports;
mod pack;
pub(crate) mod portrayal;
mod record;
mod search;
mod spatial;
mod surveyed;

#[cfg(test)]
mod tests;

use std::cmp::Ordering;
use std::path::Path;
use std::sync::{Arc, OnceLock};

use crate::SceneryError;

pub use compile::{AirportRenewalSource, RenewalScope};
pub use faa_nasr::{FaaNasrCsv, parse_faa_nasr};
pub use model::{
    AerodromeTier, AirportFrequency, AirportIds, AirportIdx, AirportRecord, CompactCode,
    FrequencyRole, RunwayEndRecord, RunwayRecord, RunwaySurfaceKind, facility_code_of,
    magnetic_heading_deg,
};
pub use our_airports::{
    ICONIC_AIRPORTS, OurAirportsCsv, OurAirportsName, our_airports_us_names, parse_our_airports,
};
pub use record::LandingRunwayGeometry;
pub use spatial::globe_visible_radius_deg;
pub(crate) use spatial::{CELL_DEG, LAT_CELLS, LON_CELLS};

use search::CodeIndex;
use spatial::GlobeGrid;

/// The embedded pack [`AirportDatabase::builtin`] decodes.
const BUILTIN_PACK: &[u8] = include_bytes!("airport_db/data/global_airports.bin");

/// Built-in aerodromes every globe view keeps when they are on the visible
/// hemisphere.
const FEATURED_CODES: [&str; 3] = ["47N", "KEWR", "KDOV"];

/// Airport records with a globe index, a code index and search keys.
#[derive(Debug, Clone, Default)]
pub struct AirportDatabase {
    records: Vec<AirportRecord>,
    grid: GlobeGrid,
    codes: CodeIndex,
    /// Lower-cased `name municipality` per record, built by the first
    /// search.
    search_keys: OnceLock<Vec<Box<str>>>,
    featured: Vec<AirportIdx>,
}

impl AirportDatabase {
    /// Indexes `records`, keeping their order.
    #[must_use]
    pub fn from_records(records: Vec<AirportRecord>) -> Self {
        let grid = GlobeGrid::build(&records);
        let codes = CodeIndex::build(&records);
        let mut database = Self {
            records,
            grid,
            codes,
            search_keys: OnceLock::new(),
            featured: Vec::new(),
        };
        database.featured = FEATURED_CODES
            .iter()
            .filter_map(|code| database.by_code(code).map(|(idx, _)| idx))
            .collect();
        database
    }

    /// The database embedded in the crate (or loaded from `SIMULATE_AIRPORT_PACK`
    /// when set on native hosts), decoded once per process. Empty only if the
    /// embedded pack failed to decode, which [`Self::decode_builtin`] reports.
    pub fn builtin() -> &'static Arc<AirportDatabase> {
        static BUILTIN: OnceLock<Arc<AirportDatabase>> = OnceLock::new();
        BUILTIN.get_or_init(|| {
            #[cfg(not(target_arch = "wasm32"))]
            if let Some(path) = std::env::var_os("SIMULATE_AIRPORT_PACK")
                && let Ok(db) = Self::from_pack_file(Path::new(&path))
            {
                return Arc::new(db);
            }
            Arc::new(Self::decode_builtin().unwrap_or_default())
        })
    }

    /// Decodes the embedded pack afresh.
    pub fn decode_builtin() -> Result<Self, SceneryError> {
        let mut records = pack::decode_records(BUILTIN_PACK)?;
        surveyed::upsert_nzsp(&mut records);
        Ok(Self::from_records(records))
    }

    /// Reads, CRC-verifies, and indexes a `SIMAPT01` pack file from disk,
    /// preserving surveyed polar stations (`NZSP`).
    pub fn from_pack_file(path: &Path) -> Result<Self, SceneryError> {
        let bytes = std::fs::read(path).map_err(|err| SceneryError::Read {
            path: path.display().to_string(),
            detail: err.to_string(),
        })?;
        let mut records = pack::decode_records(&bytes)?;
        surveyed::upsert_nzsp(&mut records);
        Ok(Self::from_records(records))
    }

    /// Encodes the records as a `SIMAPT01` pack.
    pub fn to_pack_bytes(&self) -> Result<Vec<u8>, SceneryError> {
        pack::encode_records(&self.records)
    }

    /// Decodes and indexes a `SIMAPT01` pack.
    pub fn from_pack_bytes(bytes: &[u8]) -> Result<Self, SceneryError> {
        Ok(Self::from_records(pack::decode_records(bytes)?))
    }

    /// Every record, in index order.
    #[must_use]
    pub fn records(&self) -> &[AirportRecord] {
        &self.records
    }

    /// The record `idx` names.
    #[must_use]
    pub fn get(&self, idx: AirportIdx) -> Option<&AirportRecord> {
        self.records.get(idx.index())
    }

    /// How many airports the database holds.
    #[must_use]
    pub fn len(&self) -> usize {
        self.records.len()
    }

    /// Whether the database holds no airport.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.records.is_empty()
    }

    /// How many runways all airports hold.
    #[must_use]
    pub fn runway_count(&self) -> usize {
        self.records.iter().map(|record| record.runways.len()).sum()
    }

    /// The airports whose reference point lies in grid cell `(lat_band, lon_band)` of the
    /// 2° globe grid.
    pub(crate) fn airports_in_cell(
        &self,
        lat_band: usize,
        lon_band: usize,
    ) -> impl Iterator<Item = (AirportIdx, &AirportRecord)> + '_ {
        self.grid
            .cell(lat_band, lon_band)
            .iter()
            .filter_map(|&idx| Some((idx, self.get(idx)?)))
    }

    /// Handles of every record, in index order.
    pub fn indices(&self) -> impl Iterator<Item = AirportIdx> + '_ {
        (0..self.records.len()).filter_map(|i| u32::try_from(i).ok().map(AirportIdx))
    }

    /// Inserts `record`, or replaces an existing record sharing its primary code in place,
    /// rebuilding the spatial, code, and free-text search indexes.
    pub fn upsert(&mut self, mut record: AirportRecord) -> Option<AirportIdx> {
        record.classify();
        let pos = if let Some(existing) = self
            .records
            .iter()
            .position(|r| r.ids.primary == record.ids.primary)
        {
            if let Some(slot) = self.records.get_mut(existing) {
                *slot = record;
            }
            existing
        } else {
            let idx = self.records.len();
            self.records.push(record);
            idx
        };
        self.grid = GlobeGrid::build(&self.records);
        self.codes = CodeIndex::build(&self.records);
        self.search_keys = OnceLock::new();
        self.featured = FEATURED_CODES
            .iter()
            .filter_map(|code| self.by_code(code).map(|(idx, _)| idx))
            .collect();
        u32::try_from(pos).ok().map(AirportIdx)
    }

    /// Orders two records by prominence: tier, then longest runway, then
    /// primary code.
    fn rank(&self, a: AirportIdx, b: AirportIdx) -> Ordering {
        match (self.get(a), self.get(b)) {
            (Some(a), Some(b)) => rank_records(a, b),
            _ => a.cmp(&b),
        }
    }
}

/// Prominence order: tier, then longest runway descending, then code.
pub(crate) fn rank_records(a: &AirportRecord, b: &AirportRecord) -> Ordering {
    a.tier
        .cmp(&b.tier)
        .then(b.longest_runway_m.total_cmp(&a.longest_runway_m))
        .then(a.ids.primary.cmp(&b.ids.primary))
}

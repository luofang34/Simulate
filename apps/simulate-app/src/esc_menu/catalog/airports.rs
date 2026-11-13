//! The hub's aerodrome list over the built-in airport database: one entry per surveyed
//! airport (the OpenStreetMap-mapped ones keep their stands), the corridor and the Moon
//! after them, and searches ranked by the database's own code and name index.

use sim_scenery::{
    AdHocAerodrome, AerodromeId, AirportDatabase, AirportIdx, AirportRecord, FrequencyRole,
    OsmAerodromeDiagram, RunwayStrip,
};

use super::{
    AerodromeEntry, AerodromeKind, HubCatalog, RunwayEntry, ScenarioEntry, VehicleDescriptor,
};

/// Most database matches a search lists.
const SEARCH_LIMIT: usize = 60;

impl HubCatalog {
    /// The built-in vehicles and scenarios plus every airport of
    /// [`AirportDatabase::builtin`] in record order (the OpenStreetMap-mapped `47N`,
    /// `KEWR` and `KDOV` as diagrams with their stands), then the corridor and Tranquility.
    #[must_use]
    pub fn with_global_airports() -> Self {
        let database = AirportDatabase::builtin();
        let diagrams: Vec<OsmAerodromeDiagram> = AerodromeId::ALL
            .into_iter()
            .filter_map(|id| OsmAerodromeDiagram::builtin(id).ok())
            .collect();
        let mut aerodromes: Vec<AerodromeEntry> = database
            .records()
            .iter()
            .enumerate()
            .filter_map(|(i, record)| {
                let idx = AirportIdx(u32::try_from(i).ok()?);
                let diagram = record
                    .osm_aerodrome()
                    .and_then(|id| diagrams.iter().find(|d| d.id == id));
                Some(match diagram {
                    Some(diagram) => mapped_entry(diagram, idx, record),
                    None => database_entry(idx, record),
                })
            })
            .collect();
        aerodromes.extend(special_entries());
        Self {
            vehicles: VehicleDescriptor::builtins(),
            aerodromes,
            ad_hoc: Vec::new(),
            scenarios: ScenarioEntry::all(),
            global_airports: true,
        }
    }

    /// Registers or updates an [`AdHocAerodrome`] (`ZMRS`, `EFHF`, `ZZ01`, ...) in the
    /// catalog, inserting new rows ahead of the trailing `DELAWARE` / `TRANQUILITY` rows
    /// (or upgrading an existing runway-less database row with the same code in place), and
    /// returns its `aerodromes` index.
    pub fn register_ad_hoc(&mut self, aerodrome: AdHocAerodrome) -> usize {
        let code = aerodrome.airport.ids.primary.as_str().to_owned();
        let ad_hoc_idx = if let Some(pos) = self
            .ad_hoc
            .iter()
            .position(|existing| existing.airport.ids.primary.as_str() == code)
        {
            if let Some(slot) = self.ad_hoc.get_mut(pos) {
                *slot = aerodrome;
            }
            pos
        } else {
            let idx = self.ad_hoc.len();
            self.ad_hoc.push(aerodrome);
            idx
        };
        let Some(stored) = self.ad_hoc.get(ad_hoc_idx) else {
            return 0;
        };
        let entry = ad_hoc_entry(ad_hoc_idx, stored);
        if let Some(row) = self.aerodromes.iter().position(|a| {
            a.code == code && (matches!(a.kind, AerodromeKind::AdHoc(_)) || a.runways.is_empty())
        }) {
            if let Some(slot) = self.aerodromes.get_mut(row) {
                *slot = entry;
            }
            return row;
        }
        let insert_at = self
            .aerodromes
            .iter()
            .position(|a| {
                matches!(
                    a.kind,
                    AerodromeKind::DelawareCorridor | AerodromeKind::Tranquility
                )
            })
            .unwrap_or(self.aerodromes.len());
        self.aerodromes.insert(insert_at, entry);
        insert_at
    }

    /// Looks up a registered [`AdHocAerodrome`] by facility code (case-insensitive).
    #[must_use]
    pub fn ad_hoc_by_code(&self, code: &str) -> Option<&AdHocAerodrome> {
        let upper = code.trim().to_ascii_uppercase();
        self.ad_hoc
            .iter()
            .find(|a| a.airport.ids.primary.as_str().eq_ignore_ascii_case(&upper))
    }

    /// Collects every [`RunwayStrip`] across all registered [`Self::ad_hoc`] aerodromes.
    #[must_use]
    pub fn ad_hoc_strips(&self) -> Vec<RunwayStrip> {
        self.ad_hoc
            .iter()
            .flat_map(|a| a.strips.iter().cloned())
            .collect()
    }

    /// Seeds the built-in historical ad-hoc aerodrome presets (`ZMRS` Bolshoy Moskvoretsky
    /// Bridge + Vasilevsky Descent, `ZMRQ` Red Square Plaza, and `EFHF` Helsinki-Malmi when
    /// missing runways) and stamps them onto `surface`.
    pub fn seed_ad_hoc_presets(&mut self, surface: &crate::world_surface::WorldSurfaceHandle) {
        if let Ok(bridge) = AdHocAerodrome::bolshoy_moskvoretsky_bridge() {
            let _row = self.register_ad_hoc(bridge);
        }
        if let Ok(plaza) = AdHocAerodrome::red_square_plaza() {
            let _row = self.register_ad_hoc(plaza);
        }
        let efhf_missing_runways = self
            .aerodromes
            .iter()
            .find(|a| a.code == "EFHF")
            .is_none_or(|a| a.runways.is_empty());
        if efhf_missing_runways && let Ok(efhf) = AdHocAerodrome::helsinki_malmi_efhf() {
            let _row = self.register_ad_hoc(efhf);
        }
        surface.set_ad_hoc_strips(self.ad_hoc_strips());
    }

    /// Catalog index of database airport `idx`.
    #[must_use]
    pub fn index_of_airport(&self, idx: AirportIdx) -> Option<usize> {
        let direct = idx.index();
        if self
            .aerodromes
            .get(direct)
            .is_some_and(|a| a.airport_idx == Some(idx))
        {
            return Some(direct);
        }
        self.aerodromes
            .iter()
            .position(|a| a.airport_idx == Some(idx))
    }
}

/// An OpenStreetMap-mapped airport: the diagram's runway ends and stands, the record's
/// surveyed elevation and frequencies.
fn mapped_entry(
    diagram: &OsmAerodromeDiagram,
    idx: AirportIdx,
    record: &AirportRecord,
) -> AerodromeEntry {
    AerodromeEntry {
        field_elevation_m: f64::from(record.elevation_msl_m),
        mag_var_deg: Some(f64::from(record.mag_var_deg)),
        lat_deg: Some(record.lat_deg),
        lon_deg: Some(record.lon_deg),
        location_hint: location_hint(record),
        airport_idx: Some(idx),
        ..AerodromeEntry::from_diagram(diagram)
    }
}

/// An ad-hoc aerodrome delineated at run time or from historical presets.
fn ad_hoc_entry(ad_hoc_idx: usize, aerodrome: &AdHocAerodrome) -> AerodromeEntry {
    let record = &aerodrome.airport;
    AerodromeEntry {
        code: record.ids.primary.as_str().to_owned(),
        name: record.name.to_string(),
        kind: AerodromeKind::AdHoc(ad_hoc_idx),
        field_elevation_m: f64::from(record.elevation_msl_m),
        mag_var_deg: Some(f64::from(record.mag_var_deg)),
        runways: runway_entries(record),
        stands: Vec::new(),
        lat_deg: Some(record.lat_deg),
        lon_deg: Some(record.lon_deg),
        location_hint: location_hint(record),
        airport_idx: None,
    }
}

/// A surveyed airport flown from its runway records; it has no mapped stands.
fn database_entry(idx: AirportIdx, record: &AirportRecord) -> AerodromeEntry {
    AerodromeEntry {
        code: record.ids.primary.as_str().to_owned(),
        name: record.name.to_string(),
        kind: AerodromeKind::Database(idx),
        field_elevation_m: f64::from(record.elevation_msl_m),
        mag_var_deg: Some(f64::from(record.mag_var_deg)),
        runways: runway_entries(record),
        stands: Vec::new(),
        lat_deg: Some(record.lat_deg),
        lon_deg: Some(record.lon_deg),
        location_hint: location_hint(record),
        airport_idx: Some(idx),
    }
}

/// Every runway end of `record`, runway by runway in the order its identifier names them.
#[must_use]
pub fn runway_entries(record: &AirportRecord) -> Vec<RunwayEntry> {
    record
        .runways
        .iter()
        .flat_map(|runway| {
            runway.ends.iter().map(|end| RunwayEntry {
                designator: end.designator.as_str().to_owned(),
                magnetic_bearing_deg: f64::from(end.magnetic_heading_deg).rem_euclid(360.0),
                true_bearing_deg: f64::from(end.true_heading_deg).rem_euclid(360.0),
                length_m: f64::from(runway.length_m),
            })
        })
        .collect()
}

/// `Newark, US-NJ - TWR 118.300`: where the airport is and the frequency to call first.
#[must_use]
pub fn location_hint(record: &AirportRecord) -> String {
    let region = record.region_code.as_str();
    let place = if record.municipality.is_empty() {
        region.to_owned()
    } else {
        format!("{}, {region}", record.municipality)
    };
    match record.primary_frequency_mhz() {
        Some((role, mhz)) => format!("{place} - {} {mhz:.3}", role_label(role)),
        None => place,
    }
}

const fn role_label(role: FrequencyRole) -> &'static str {
    match role {
        FrequencyRole::Tower => "TWR",
        FrequencyRole::Ctaf => "CTAF",
        FrequencyRole::Unicom => "UNICOM",
        FrequencyRole::Atis => "ATIS",
        FrequencyRole::Awos => "AWOS",
        FrequencyRole::Ground => "GND",
        FrequencyRole::Approach => "APP",
        FrequencyRole::Departure => "DEP",
    }
}

/// The Delaware visual-navigation corridor and Tranquility Base.
pub(super) fn special_entries() -> [AerodromeEntry; 2] {
    let special =
        |code: &str, name: &str, kind, field_elevation_m, mag_var_deg, at: Option<(f64, f64)>| {
            AerodromeEntry {
                code: code.to_owned(),
                name: name.to_owned(),
                kind,
                field_elevation_m,
                mag_var_deg,
                runways: Vec::new(),
                stands: Vec::new(),
                lat_deg: at.map(|(lat, _)| lat),
                lon_deg: at.map(|(_, lon)| lon),
                location_hint: String::new(),
                airport_idx: None,
            }
        };
    [
        special(
            "DELAWARE",
            "Delaware Visual-Navigation Corridor",
            AerodromeKind::DelawareCorridor,
            8.81,
            Some(-11.8),
            Some((39.13, -75.47)),
        ),
        special(
            "TRANQUILITY",
            "Tranquility Base - Sea of Tranquility - Moon",
            AerodromeKind::Tranquility,
            0.0,
            None,
            None,
        ),
    ]
}

/// The database's ranked search mapped to catalog rows, with the entries that are not
/// database airports around it: those whose code `query` starts (`DELAWARE`, `TRANQ`)
/// first, those merely matching after.
pub(super) fn ranked_matches(catalog: &HubCatalog, query: &str) -> Vec<usize> {
    let upper = query.trim().to_ascii_uppercase();
    let specials = catalog
        .aerodromes
        .iter()
        .enumerate()
        .filter(|(_, a)| a.airport_idx.is_none() && a.matches(query));
    let (by_code, others): (Vec<_>, Vec<_>) =
        specials.partition(|(_, a)| !upper.is_empty() && a.code.starts_with(&upper));
    let mut rows: Vec<usize> = by_code.into_iter().map(|(i, _)| i).collect();
    rows.extend(
        AirportDatabase::builtin()
            .search(query, SEARCH_LIMIT)
            .into_iter()
            .filter_map(|idx| catalog.index_of_airport(idx)),
    );
    rows.extend(others.into_iter().map(|(i, _)| i));
    rows
}

#[cfg(test)]
mod tests;

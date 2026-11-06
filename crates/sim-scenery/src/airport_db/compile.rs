//! Merging and renewing regional/global aeronautical data sources into one
//! [`AirportDatabase`].

use std::collections::{HashMap, HashSet};

use flightbag::AuthoritySource;

use super::AirportDatabase;
use super::assemble::title_case;
use super::faa_nasr::{FaaNasrCsv, parse_faa_nasr};
use super::model::{AirportIds, AirportRecord, CompactCode};
use super::our_airports::{OurAirportsCsv, OurAirportsName, parse_our_airports};
use crate::SceneryError;

/// Which existing records an [`AirportDatabase::renew`] cycle replaces.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RenewalScope {
    /// Replaces the complete edition of `authority` (for example, a full 28-day
    /// national or global baseline feed), retiring aerodromes removed from that
    /// authority while keeping every other authority untouched.
    ReplaceAuthority(AuthoritySource),
    /// Replaces every existing record in ISO 3166-1 alpha-2 `country_iso2`
    /// (for example, `*b"GB"` or `*b"JP"` when loading a state's national AIP).
    ReplaceCountry([u8; 2]),
    /// Upserts incoming records by aerodrome code (`primary`, `icao`, `faa_lid`),
    /// leaving all non-matching aerodromes untouched.
    UpsertByCode,
}

/// An aeronautical data cycle edition or regional supplement that
/// [`AirportDatabase::renew`] merges into an existing [`AirportDatabase`].
pub trait AirportRenewalSource {
    /// Scope of existing records replaced by this cycle update.
    fn scope(&self) -> RenewalScope;

    /// Parses or extracts the incoming [`AirportRecord`] set for this cycle.
    fn load_records(&self) -> Result<Vec<AirportRecord>, SceneryError>;
}

impl AirportRenewalSource for [AirportRecord] {
    fn scope(&self) -> RenewalScope {
        RenewalScope::UpsertByCode
    }

    fn load_records(&self) -> Result<Vec<AirportRecord>, SceneryError> {
        Ok(self.to_vec())
    }
}

impl AirportRenewalSource for Vec<AirportRecord> {
    fn scope(&self) -> RenewalScope {
        RenewalScope::UpsertByCode
    }

    fn load_records(&self) -> Result<Vec<AirportRecord>, SceneryError> {
        Ok(self.clone())
    }
}

impl AirportRenewalSource for OurAirportsCsv<'_> {
    fn scope(&self) -> RenewalScope {
        RenewalScope::ReplaceAuthority(AuthoritySource::RegionalAip)
    }

    fn load_records(&self) -> Result<Vec<AirportRecord>, SceneryError> {
        parse_our_airports(self)
    }
}

/// US regional [`AirportRenewalSource`] implementation backed by the 28-day
/// FAA NASR CSV subscription.
impl AirportRenewalSource for FaaNasrCsv<'_> {
    fn scope(&self) -> RenewalScope {
        RenewalScope::ReplaceAuthority(AuthoritySource::FaaNasr)
    }

    fn load_records(&self) -> Result<Vec<AirportRecord>, SceneryError> {
        parse_faa_nasr(self)
    }
}

impl AirportDatabase {
    /// One database from primary national records and international baseline
    /// records, sorted by primary code. Uppercase-only national records take
    /// their display names and IATA code from `us_names` where it lists one of
    /// their codes, and are title-cased otherwise; an international record
    /// whose code a primary record already holds is dropped.
    #[must_use]
    pub fn compile(
        mut faa: Vec<AirportRecord>,
        international: Vec<AirportRecord>,
        us_names: &HashMap<CompactCode, OurAirportsName>,
    ) -> Self {
        for record in &mut faa {
            enrich(record, us_names);
        }
        let taken: HashSet<CompactCode> = faa.iter().flat_map(|r| key_codes(&r.ids)).collect();
        let mut records = faa;
        records.extend(international.into_iter().filter(|record| {
            let icao_taken = record.ids.icao.is_some_and(|code| taken.contains(&code));
            !icao_taken && !taken.contains(&record.ids.primary)
        }));
        records.sort_by_key(|record| record.ids.primary);
        records.dedup_by(|a, b| a.ids.primary == b.ids.primary);
        super::surveyed::replace_nzsp_if_present(&mut records);
        Self::from_records(records)
    }

    /// Renews `self` from an aeronautical cycle update or regional supplement
    /// (`source`), preserving unaffected authorities/states and any mixed-case
    /// display names or IATA codes already known for uppercase-only feeds.
    pub fn renew<S: AirportRenewalSource + ?Sized>(
        &self,
        source: &S,
    ) -> Result<Self, SceneryError> {
        Ok(self.renew_with_records(source.scope(), source.load_records()?))
    }

    /// Renews `self` with pre-parsed `incoming` records under `scope`.
    #[must_use]
    pub fn renew_with_records(
        &self,
        scope: RenewalScope,
        mut incoming: Vec<AirportRecord>,
    ) -> Self {
        let prior_names = collect_prior_names(&self.records);
        let had_nzsp = self.by_code("NZSP").is_some();
        for record in &mut incoming {
            enrich_incoming(record, &prior_names);
        }
        let taken: HashSet<CompactCode> = incoming.iter().flat_map(|r| key_codes(&r.ids)).collect();
        let mut records: Vec<AirportRecord> = self
            .records
            .iter()
            .filter(|existing| !replaced_by_scope(existing, scope, &taken))
            .cloned()
            .collect();
        records.extend(incoming);
        records.sort_by_key(|record| record.ids.primary);
        records.dedup_by(|a, b| a.ids.primary == b.ids.primary);
        if had_nzsp {
            super::surveyed::upsert_nzsp(&mut records);
        }
        Self::from_records(records)
    }
}

fn key_codes(ids: &AirportIds) -> impl Iterator<Item = CompactCode> + use<> {
    [Some(ids.primary), ids.icao, ids.faa_lid]
        .into_iter()
        .flatten()
}

fn replaced_by_scope(
    existing: &AirportRecord,
    scope: RenewalScope,
    incoming_codes: &HashSet<CompactCode>,
) -> bool {
    let in_scope = match scope {
        RenewalScope::ReplaceAuthority(auth) => existing.authority == auth,
        RenewalScope::ReplaceCountry(iso2) => existing.country_iso2 == iso2,
        RenewalScope::UpsertByCode => false,
    };
    in_scope || key_codes(&existing.ids).any(|code| incoming_codes.contains(&code))
}

fn collect_prior_names(records: &[AirportRecord]) -> HashMap<CompactCode, OurAirportsName> {
    let mut map = HashMap::new();
    for record in records {
        let entry = OurAirportsName {
            name: record.name.to_string(),
            municipality: record.municipality.to_string(),
            iata: record.ids.iata,
        };
        for code in key_codes(&record.ids) {
            map.entry(code).or_insert_with(|| entry.clone());
        }
    }
    map
}

fn enrich_incoming(
    record: &mut AirportRecord,
    prior_names: &HashMap<CompactCode, OurAirportsName>,
) {
    let upper_only = !record.name.bytes().any(|b| b.is_ascii_lowercase());
    if upper_only {
        enrich(record, prior_names);
    } else if record.ids.iata.is_none() {
        record.ids.iata = key_codes(&record.ids)
            .find_map(|code| prior_names.get(&code).and_then(|entry| entry.iata));
    }
}

fn enrich(record: &mut AirportRecord, us_names: &HashMap<CompactCode, OurAirportsName>) {
    let found = key_codes(&record.ids).find_map(|code| us_names.get(&code));
    match found {
        Some(entry) => {
            if !entry.name.is_empty() {
                record.name = entry.name.as_str().into();
            }
            record.municipality = if entry.municipality.is_empty() {
                title_case(&record.municipality).into()
            } else {
                entry.municipality.as_str().into()
            };
            record.ids.iata = record.ids.iata.or(entry.iata);
        }
        None => {
            record.name = title_case(&record.name).into();
            record.municipality = title_case(&record.municipality).into();
        }
    }
}

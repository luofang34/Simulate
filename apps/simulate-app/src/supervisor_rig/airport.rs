//! The airport each flight is at, picked from the global airport database, and its
//! surface graphs with true WGS84 ellipsoidal heights for the active vertical datum.

use flightbag::{AirportSurfaceGraph, RunwayDesignator, RunwayEndSpec};
use sim_scenario::ScenarioProfile;
use sim_scenery::{AerodromeId, AirportDatabase, AirportIdx, AirportRecord, CatalogVerticalDatum};

use super::SupervisorRig;

/// The airports the built-in scenarios fly from: the corridor's, then the two pattern
/// scenarios'.
const FEATURED: [AerodromeId; 3] = [
    AerodromeId::DoverKdov,
    AerodromeId::CentralJersey47N,
    AerodromeId::NewarkKewr,
];

/// Surface graphs of the featured airports and the free-flight airport, built once per
/// datum and airport choice so per-frame runway lookups never rebuild them.
#[derive(Debug, Clone)]
pub(super) struct AirportGraphs {
    featured: Vec<(AerodromeId, AirportIdx)>,
    graphs: Vec<(AirportIdx, AirportSurfaceGraph)>,
}

impl AirportGraphs {
    pub(super) fn build(datum: CatalogVerticalDatum, free: Option<AirportIdx>) -> Self {
        let db = AirportDatabase::builtin();
        let featured: Vec<(AerodromeId, AirportIdx)> = FEATURED
            .into_iter()
            .filter_map(|id| Some((id, featured_idx(id)?)))
            .collect();
        let mut graphs: Vec<(AirportIdx, AirportSurfaceGraph)> = Vec::new();
        for idx in featured.iter().map(|(_, idx)| *idx).chain(free) {
            if graphs.iter().any(|(have, _)| *have == idx) {
                continue;
            }
            if let Some(record) = db.get(idx) {
                graphs.push((idx, record.to_flightbag_graph(datum)));
            }
        }
        Self { featured, graphs }
    }

    fn featured(&self, id: AerodromeId) -> Option<AirportIdx> {
        self.featured
            .iter()
            .find(|(have, _)| *have == id)
            .map(|(_, idx)| *idx)
    }

    fn graph(&self, idx: AirportIdx) -> Option<&AirportSurfaceGraph> {
        self.graphs
            .iter()
            .find(|(have, _)| *have == idx)
            .map(|(_, graph)| graph)
    }
}

/// The database index of `id`.
pub(super) fn featured_idx(id: AerodromeId) -> Option<AirportIdx> {
    AirportDatabase::builtin()
        .by_code(id.code())
        .map(|(idx, _)| idx)
}

/// The runway end a flight lines up on: `selected` in free flight, else the built-in
/// diagram's `pattern` runway, else the first.
fn pick_end(
    graph: &AirportSurfaceGraph,
    selected: Option<usize>,
    pattern: Option<RunwayDesignator>,
) -> Option<&RunwayEndSpec> {
    selected
        .and_then(|index| graph.runway_ends.get(index))
        .or_else(|| graph.runway_end(pattern?))
        .or_else(|| graph.runway_ends.first())
}

impl SupervisorRig {
    /// Sets the datum the world surface reads Earth heights in, which airport heights and
    /// world-anchored cameras are restated from.
    pub fn set_vertical_datum(&mut self, datum: CatalogVerticalDatum) {
        self.datum = datum;
        self.refresh_graphs();
    }

    /// See [`Self::set_vertical_datum`].
    #[must_use]
    pub const fn vertical_datum(&self) -> CatalogVerticalDatum {
        self.datum
    }

    /// Selects the free-flight airport from [`AirportDatabase::builtin`] and the runway end
    /// (an index into its surface graph's runway ends) a flight there lines up on.
    pub fn set_free_airport(&mut self, idx: AirportIdx, runway_end: usize) {
        self.free_airport = Some(idx);
        self.selected_runway_end = runway_end;
        if let Some(id) = AirportDatabase::builtin()
            .get(idx)
            .and_then(AirportRecord::osm_aerodrome)
        {
            self.free_aerodrome = id;
        }
        self.refresh_graphs();
    }

    /// The free-flight airport's database index.
    #[cfg(test)]
    #[must_use]
    pub const fn free_airport_idx(&self) -> Option<AirportIdx> {
        self.free_airport
    }

    /// The free-flight runway end, an index into the airport's runway ends.
    #[cfg(test)]
    #[must_use]
    pub const fn selected_runway_end(&self) -> usize {
        self.selected_runway_end
    }

    /// The database airport `profile` flies at: its scenario's airport, the free-flight
    /// airport without a profile, and none off Earth.
    #[must_use]
    pub fn active_airport_record(
        &self,
        profile: Option<ScenarioProfile>,
    ) -> Option<&'static AirportRecord> {
        AirportDatabase::builtin().get(self.active_airport_idx(profile)?)
    }

    /// The code of [`Self::active_airport_record`], else of [`Self::effective_aerodrome`].
    #[must_use]
    pub fn effective_airport_code(&self, profile: Option<ScenarioProfile>) -> &'static str {
        self.active_airport_record(profile).map_or_else(
            || self.effective_aerodrome(profile).code(),
            |record| record.ids.primary.as_str(),
        )
    }

    /// The active runway end with its threshold at its true ellipsoidal height in a world
    /// read in `datum`.
    #[cfg(test)]
    #[must_use]
    pub fn active_runway_in_datum(
        &self,
        profile: Option<ScenarioProfile>,
        datum: CatalogVerticalDatum,
    ) -> Option<RunwayEndSpec> {
        if datum == self.datum {
            return self.cached_runway(profile).copied();
        }
        let record = self.active_airport_record(profile)?;
        let graph = record.to_flightbag_graph(datum);
        pick_end(
            &graph,
            self.runway_choice(profile),
            self.pattern_runway(record),
        )
        .copied()
    }

    pub(super) fn active_airport_idx(
        &self,
        profile: Option<ScenarioProfile>,
    ) -> Option<AirportIdx> {
        match profile {
            Some(
                ScenarioProfile::Apollo11LunarLanding | ScenarioProfile::Apollo11CapsuleReentry,
            ) => None,
            Some(ScenarioProfile::Airport47NUncontrolledPattern) => {
                self.graphs.featured(AerodromeId::CentralJersey47N)
            }
            Some(ScenarioProfile::AirportKewrToweredPattern) => {
                self.graphs.featured(AerodromeId::NewarkKewr)
            }
            Some(
                ScenarioProfile::CorridorNominalCruise
                | ScenarioProfile::CorridorOffSeedAcquisition
                | ScenarioProfile::CorridorAggressiveBank
                | ScenarioProfile::CorridorSameCellLoiter,
            ) => self.graphs.featured(AerodromeId::DoverKdov),
            None => self.free_airport,
        }
    }

    /// The active runway from the graphs built for [`Self::vertical_datum`].
    pub(super) fn cached_runway(&self, profile: Option<ScenarioProfile>) -> Option<&RunwayEndSpec> {
        let idx = self.active_airport_idx(profile)?;
        let record = AirportDatabase::builtin().get(idx)?;
        pick_end(
            self.graphs.graph(idx)?,
            self.runway_choice(profile),
            self.pattern_runway(record),
        )
    }

    /// The pattern runway of `record`'s built-in diagram, if it has one.
    fn pattern_runway(&self, record: &AirportRecord) -> Option<RunwayDesignator> {
        let diagram = self.diagram(record.osm_aerodrome()?)?;
        Some(diagram.primary_runway_end()?.designator)
    }

    fn runway_choice(&self, profile: Option<ScenarioProfile>) -> Option<usize> {
        profile.is_none().then_some(self.selected_runway_end)
    }

    pub(super) fn refresh_graphs(&mut self) {
        self.graphs = AirportGraphs::build(self.datum, self.free_airport);
    }
}

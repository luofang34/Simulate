use flightbag::{AirportSurfaceGraph, FacilityCode, RunwayDesignator, RunwayEndSpec};
use sim_core::VirtualCameraSpec;
use sim_geo::Geodetic;
use sim_scenario::{
    OperatorViewportMode, RadioBeliefPolicy, ScenarioProfile, SimSupervisor, SimTimeScale,
};
use sim_scenery::{
    AerodromeId, AirportIdx, CatalogVerticalDatum, LandingRunwayGeometry, OsmAerodromeDiagram,
};
use world::GeodeticPosition;

use airport::{AirportGraphs, featured_idx};
use cameras::SUITE_CAMERA_COUNT;

mod airport;
mod cameras;
#[cfg(test)]
mod tests;
mod traffic;

/// Reference Delaware (`KDOV`) threshold latitude, degrees, inside `SyntheticCorridorSpec::delaware()`.
pub const DELAWARE_RUNWAY_LAT_DEG: f64 = 39.1220;
/// Reference Delaware (`KDOV`) threshold longitude, degrees, inside `SyntheticCorridorSpec::delaware()`.
pub const DELAWARE_RUNWAY_LON_DEG: f64 = -75.4680;
/// Reference Delaware (`KDOV`) threshold ellipsoidal height, metres.
pub const DELAWARE_RUNWAY_HEIGHT_M: f64 = 8.5;
/// Reference Delaware (`KDOV`) Runway `09` true bearing (`85°`, aligned with the synthetic corridor).
pub const DELAWARE_RUNWAY_BEARING_RAD: f64 = 85.0 * std::f64::consts::PI / 180.0;

/// Owns [`SimSupervisor`], OpenStreetMap airport surface diagrams (`KDOV`, `47N`, `KEWR`),
/// the free-flight airport from the global airport database, active virtual camera
/// channel selection, and truth-to-belief radio traffic filtering.
pub struct SupervisorRig {
    supervisor: SimSupervisor,
    delaware_fallback: Option<AirportSurfaceGraph>,
    diagrams: Vec<OsmAerodromeDiagram>,
    free_aerodrome: AerodromeId,
    free_airport: Option<AirportIdx>,
    selected_runway_end: usize,
    datum: CatalogVerticalDatum,
    graphs: AirportGraphs,
    virtual_camera_index: usize,
    /// The profile whose camera suite [`Self::cycle_virtual_camera`] last stepped through.
    camera_suite: Option<ScenarioProfile>,
    /// The flying vehicle's own views, cycled after the scenario's suite.
    vehicle_cameras: &'static [VirtualCameraSpec],
}

impl Default for SupervisorRig {
    fn default() -> Self {
        Self::new()
    }
}

impl SupervisorRig {
    /// Creates a supervisor seeded with OpenStreetMap diagrams (`KDOV`, `47N`, `KEWR`)
    /// and radio reception limits.
    #[must_use]
    pub fn new() -> Self {
        let mut supervisor = SimSupervisor::new();
        supervisor.set_radio_policy(RadioBeliefPolicy {
            adsb_range_m: 120_000.0,
            short_range_radio_m: 15_000.0,
            receive_weather: true,
            receive_notices: true,
        });
        let diagrams = AerodromeId::ALL
            .into_iter()
            .filter_map(|id| OsmAerodromeDiagram::builtin(id).ok())
            .collect();
        let datum = CatalogVerticalDatum::OrthometricMslEgm96;
        let free_airport = featured_idx(AerodromeId::DoverKdov);
        Self {
            supervisor,
            delaware_fallback: build_delaware_airport(),
            diagrams,
            free_aerodrome: AerodromeId::DoverKdov,
            free_airport,
            selected_runway_end: 0,
            datum,
            graphs: AirportGraphs::build(datum, free_airport),
            virtual_camera_index: 0,
            camera_suite: None,
            vehicle_cameras: &[],
        }
    }

    /// Underlying [`SimSupervisor`].
    #[cfg(test)]
    #[must_use]
    pub const fn supervisor(&self) -> &SimSupervisor {
        &self.supervisor
    }

    /// Active [`OperatorViewportMode`].
    #[must_use]
    pub const fn viewport_mode(&self) -> OperatorViewportMode {
        self.supervisor.viewport_mode()
    }

    /// Sets the active [`OperatorViewportMode`].
    #[cfg(any(test, not(target_arch = "wasm32")))]
    pub fn set_viewport_mode(&mut self, mode: OperatorViewportMode) {
        self.supervisor.set_viewport_mode(mode);
    }

    /// Cycles [`OperatorViewportMode`] through all five shared operator views.
    pub fn cycle_viewport_mode(&mut self) -> OperatorViewportMode {
        let next = match self.supervisor.viewport_mode() {
            OperatorViewportMode::Cockpit => OperatorViewportMode::Chase,
            OperatorViewportMode::Chase => OperatorViewportMode::SensorAperture,
            OperatorViewportMode::SensorAperture => OperatorViewportMode::ChartMap,
            OperatorViewportMode::ChartMap => OperatorViewportMode::VolumetricScene,
            OperatorViewportMode::VolumetricScene => OperatorViewportMode::Cockpit,
        };
        self.supervisor.set_viewport_mode(next);
        next
    }

    /// Selects a specific free-flight [`AerodromeId`], its database airport, and its first
    /// runway end.
    pub fn set_free_aerodrome(&mut self, id: AerodromeId) {
        self.free_aerodrome = id;
        self.sync_free_airport();
    }

    /// Points the free-flight database airport at [`Self::free_aerodrome`]'s.
    fn sync_free_airport(&mut self) {
        self.free_airport = featured_idx(self.free_aerodrome);
        self.selected_runway_end = 0;
        self.refresh_graphs();
    }

    /// Selects a specific virtual camera channel: the scenario suite's five, then the
    /// vehicle's own views.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn set_virtual_camera_index(&mut self, index: usize) {
        self.virtual_camera_index = index % self.camera_count();
    }

    /// Gives the cycle the flying vehicle's own views; a selection past the new vehicle's
    /// last view goes back to the first camera.
    pub fn set_vehicle_cameras(&mut self, cameras: &'static [VirtualCameraSpec]) {
        self.vehicle_cameras = cameras;
        if self.virtual_camera_index >= self.camera_count() {
            self.virtual_camera_index = 0;
        }
    }

    /// Cameras [`Self::cycle_virtual_camera`] visits.
    fn camera_count(&self) -> usize {
        SUITE_CAMERA_COUNT.saturating_add(self.vehicle_cameras.len())
    }

    /// Cycles the active free-flight airport (`KDOV` → `47N` → `KEWR`).
    #[cfg(test)]
    pub fn cycle_aerodrome(&mut self) -> AerodromeId {
        self.step_aerodrome(true)
    }

    /// Steps the active free-flight airport forwards (`KDOV` → `47N` → `KEWR`) or backwards.
    pub fn step_aerodrome(&mut self, forward: bool) -> AerodromeId {
        self.free_aerodrome = match (self.free_aerodrome, forward) {
            (AerodromeId::DoverKdov, true) | (AerodromeId::NewarkKewr, false) => {
                AerodromeId::CentralJersey47N
            }
            (AerodromeId::CentralJersey47N, true) | (AerodromeId::DoverKdov, false) => {
                AerodromeId::NewarkKewr
            }
            (AerodromeId::NewarkKewr, true) | (AerodromeId::CentralJersey47N, false) => {
                AerodromeId::DoverKdov
            }
        };
        self.sync_free_airport();
        self.free_aerodrome
    }

    /// Active airport ID for `profile` (`47N` / `KEWR` for pattern scenarios, `KDOV` for
    /// corridor scenarios, otherwise free-flight airport).
    #[must_use]
    pub const fn effective_aerodrome(&self, profile: Option<ScenarioProfile>) -> AerodromeId {
        match profile {
            Some(ScenarioProfile::Airport47NUncontrolledPattern) => AerodromeId::CentralJersey47N,
            Some(ScenarioProfile::AirportKewrToweredPattern) => AerodromeId::NewarkKewr,
            Some(
                ScenarioProfile::CorridorNominalCruise
                | ScenarioProfile::CorridorOffSeedAcquisition
                | ScenarioProfile::CorridorAggressiveBank
                | ScenarioProfile::CorridorSameCellLoiter,
            ) => AerodromeId::DoverKdov,
            Some(
                ScenarioProfile::Apollo11LunarLanding | ScenarioProfile::Apollo11CapsuleReentry,
            )
            | None => self.free_aerodrome,
        }
    }

    /// Looks up the conflated [`OsmAerodromeDiagram`] for `id`.
    #[must_use]
    pub fn diagram(&self, id: AerodromeId) -> Option<&OsmAerodromeDiagram> {
        self.diagrams.iter().find(|d| d.id == id)
    }

    /// Active [`RunwayEndSpec`] for conformal OTW and SVS runway outlines, its threshold at
    /// its true ellipsoidal height in [`Self::vertical_datum`]: the scenario's pattern
    /// runway, or the selected runway end of the free-flight airport. None off Earth.
    #[must_use]
    pub fn active_runway(&self, profile: Option<ScenarioProfile>) -> Option<&RunwayEndSpec> {
        if matches!(
            profile,
            Some(ScenarioProfile::Apollo11LunarLanding | ScenarioProfile::Apollo11CapsuleReentry)
        ) {
            return None;
        }
        self.cached_runway(profile).or_else(|| {
            self.delaware_fallback
                .as_ref()
                .and_then(|airport| airport.runway_ends.first())
        })
    }

    /// Surveyed 3D landing geometry for the active runway (including published displaced
    /// landing threshold, longitudinal runway slope, and glidepath angle) in
    /// [`Self::vertical_datum`], free of any live simulator DEM query. None off Earth.
    #[must_use]
    pub fn active_landing_runway(
        &self,
        profile: Option<ScenarioProfile>,
    ) -> Option<LandingRunwayGeometry> {
        let rwy = self.active_runway(profile)?;
        if let Some(record) = self.active_airport_record(profile)
            && let Some(landing) = record.landing_runway(rwy.designator.as_str(), self.datum)
        {
            return Some(landing);
        }
        Some(LandingRunwayGeometry::from_runway_end_spec_in_datum(
            rwy, self.datum,
        ))
    }

    /// Steps the virtual camera channel (`Nose Nav`, `Belly Nadir`, `Left Pattern Window`,
    /// `Tower PTZ Auto-Track`, `Threshold Final Cam`, then the vehicle's own views) in
    /// `SensorAperture` view.
    ///
    /// From any other view, or with another profile's suite showing, it starts at the first
    /// camera; past the last camera it returns to the `Cockpit` view (and its HUD).
    pub fn cycle_virtual_camera(&mut self, profile: Option<ScenarioProfile>) -> VirtualCameraSpec {
        let in_aperture = self.supervisor.viewport_mode() == OperatorViewportMode::SensorAperture;
        if !in_aperture || self.camera_suite != profile {
            self.virtual_camera_index = 0;
            self.supervisor
                .set_viewport_mode(OperatorViewportMode::SensorAperture);
        } else if self.virtual_camera_index.wrapping_add(1) < self.camera_count() {
            self.virtual_camera_index = self.virtual_camera_index.wrapping_add(1);
        } else {
            self.virtual_camera_index = 0;
            self.supervisor
                .set_viewport_mode(OperatorViewportMode::Cockpit);
        }
        self.camera_suite = profile;
        self.active_virtual_camera(profile)
    }

    /// Currently selected [`VirtualCameraSpec`] for `profile`'s active airport (see
    /// [`Self::active_virtual_camera_on_body`]).
    #[must_use]
    pub fn active_virtual_camera(&self, profile: Option<ScenarioProfile>) -> VirtualCameraSpec {
        self.active_virtual_camera_on_body(profile).1
    }

    /// Active [`SimTimeScale`].
    #[must_use]
    pub const fn time_scale(&self) -> SimTimeScale {
        self.supervisor.time_scale()
    }

    /// Sets the simulation rate.
    pub fn set_time_scale(&mut self, scale: SimTimeScale) {
        self.supervisor.set_time_scale(scale);
    }

    /// Toggles between [`SimTimeScale::Paused`] and [`SimTimeScale::RealTime`].
    pub fn toggle_pause(&mut self) -> SimTimeScale {
        let next = match self.supervisor.time_scale() {
            SimTimeScale::Paused => SimTimeScale::RealTime,
            SimTimeScale::RealTime | SimTimeScale::Warp(_) => SimTimeScale::Paused,
        };
        self.supervisor.set_time_scale(next);
        next
    }

    /// Steps simulation warp up `ladder`, a rising list of rates above real time, and
    /// back to real time from its top or from a pause.
    pub fn cycle_warp(&mut self, ladder: &[f64]) -> SimTimeScale {
        let now = match self.supervisor.time_scale() {
            SimTimeScale::Paused => f64::INFINITY,
            scale @ (SimTimeScale::RealTime | SimTimeScale::Warp(_)) => scale.factor(),
        };
        let next = ladder
            .iter()
            .copied()
            .find(|rate| *rate > now * 1.05)
            .map_or(SimTimeScale::RealTime, SimTimeScale::Warp);
        self.supervisor.set_time_scale(next);
        next
    }
}

/// Default Delaware free-flight spawn position aligned with Runway `09`.
#[must_use]
pub fn default_delaware_spawn() -> Geodetic {
    Geodetic::from_degrees(DELAWARE_RUNWAY_LAT_DEG, DELAWARE_RUNWAY_LON_DEG, 450.0)
}

fn build_delaware_airport() -> Option<AirportSurfaceGraph> {
    let kdov = FacilityCode::parse("KDOV").ok()?;
    let rwy09 = RunwayDesignator::parse("09").ok()?;
    Some(AirportSurfaceGraph {
        facility: kdov,
        reference_point: GeodeticPosition::wgs84(
            DELAWARE_RUNWAY_LAT_DEG,
            DELAWARE_RUNWAY_LON_DEG,
            DELAWARE_RUNWAY_HEIGHT_M,
        ),
        runway_ends: vec![RunwayEndSpec {
            designator: rwy09,
            threshold: GeodeticPosition::wgs84(
                DELAWARE_RUNWAY_LAT_DEG,
                DELAWARE_RUNWAY_LON_DEG,
                DELAWARE_RUNWAY_HEIGHT_M,
            ),
            true_bearing_rad: DELAWARE_RUNWAY_BEARING_RAD,
            length_m: 2_926.0,
            width_m: 45.7,
        }],
        taxi_edges: Vec::new(),
        hold_short_nodes: Vec::new(),
        parking_spots: Vec::new(),
    })
}

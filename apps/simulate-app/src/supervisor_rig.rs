use flightbag::{AirportSurfaceGraph, FacilityCode, RunwayDesignator, RunwayEndSpec};
use glam::DVec3;
use sim_core::{SimInstant, VirtualCameraSpec};
use sim_geo::{Geodetic, UtcTime};
use sim_maplibre::{DynamicEntity3D, ViewSlotMask, entities_from_surveillance_belief};
use sim_scenario::{
    OperatorViewportMode, RadioBeliefPolicy, ScenarioProfile, SimSupervisor, SimTimeScale,
    pattern_virtual_cameras,
};
use sim_scenery::{AerodromeId, OsmAerodromeDiagram};
use world::surveillance::{
    AddressNamespace, Callsign, DeliveryPath, EmitterCategory, IntegrityQuality, ObservationSource,
    TrackKey, TrackObservation,
};
use world::{GeodeticPosition, LocalTangentPlane, Ned};

/// Reference Delaware (`KDOV`) threshold latitude, degrees, inside `SyntheticCorridorSpec::delaware()`.
pub const DELAWARE_RUNWAY_LAT_DEG: f64 = 39.1220;
/// Reference Delaware (`KDOV`) threshold longitude, degrees, inside `SyntheticCorridorSpec::delaware()`.
pub const DELAWARE_RUNWAY_LON_DEG: f64 = -75.4680;
/// Reference Delaware (`KDOV`) threshold ellipsoidal height, metres.
pub const DELAWARE_RUNWAY_HEIGHT_M: f64 = 8.5;
/// Reference Delaware (`KDOV`) Runway `09` true bearing (`85°`, aligned with the synthetic corridor).
pub const DELAWARE_RUNWAY_BEARING_RAD: f64 = 85.0 * std::f64::consts::PI / 180.0;

const EPOCH_UNIX_S: f64 = 1_782_864_000.0;

/// Owns [`SimSupervisor`], OpenStreetMap airport surface diagrams (`KDOV`, `47N`, `KEWR`),
/// active virtual camera channel selection, and truth-to-belief radio traffic filtering.
pub struct SupervisorRig {
    supervisor: SimSupervisor,
    delaware_fallback: Option<AirportSurfaceGraph>,
    diagrams: Vec<OsmAerodromeDiagram>,
    free_aerodrome: AerodromeId,
    virtual_camera_index: usize,
    start_utc: UtcTime,
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
        Self {
            supervisor,
            delaware_fallback: build_delaware_airport(),
            diagrams,
            free_aerodrome: AerodromeId::DoverKdov,
            virtual_camera_index: 0,
            start_utc: UtcTime::from_unix_seconds(EPOCH_UNIX_S),
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

    /// Selects a specific free-flight [`AerodromeId`].
    #[cfg(not(target_arch = "wasm32"))]
    pub fn set_free_aerodrome(&mut self, id: AerodromeId) {
        self.free_aerodrome = id;
    }

    /// Selects a specific virtual camera channel index (`0..5`).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn set_virtual_camera_index(&mut self, index: usize) {
        self.virtual_camera_index = index % 5;
    }

    /// Cycles the active free-flight airport (`KDOV` → `47N` → `KEWR`).
    pub fn cycle_aerodrome(&mut self) -> AerodromeId {
        self.free_aerodrome = match self.free_aerodrome {
            AerodromeId::DoverKdov => AerodromeId::CentralJersey47N,
            AerodromeId::CentralJersey47N => AerodromeId::NewarkKewr,
            AerodromeId::NewarkKewr => AerodromeId::DoverKdov,
        };
        self.free_aerodrome
    }

    /// Active airport ID for `profile` (`47N` / `KEWR` for pattern scenarios, otherwise free-flight airport).
    #[must_use]
    pub const fn effective_aerodrome(&self, profile: Option<ScenarioProfile>) -> AerodromeId {
        match profile {
            Some(ScenarioProfile::Airport47NUncontrolledPattern) => AerodromeId::CentralJersey47N,
            Some(ScenarioProfile::AirportKewrToweredPattern) => AerodromeId::NewarkKewr,
            _ => self.free_aerodrome,
        }
    }

    /// Looks up the conflated [`OsmAerodromeDiagram`] for `id`.
    #[must_use]
    pub fn diagram(&self, id: AerodromeId) -> Option<&OsmAerodromeDiagram> {
        self.diagrams.iter().find(|d| d.id == id)
    }

    /// Active [`RunwayEndSpec`] for conformal OTW and SVS runway outlines.
    #[must_use]
    pub fn active_runway(&self, profile: Option<ScenarioProfile>) -> Option<&RunwayEndSpec> {
        let id = self.effective_aerodrome(profile);
        self.diagram(id)
            .and_then(OsmAerodromeDiagram::primary_runway_end)
            .or_else(|| {
                self.delaware_fallback
                    .as_ref()
                    .and_then(|airport| airport.runway_ends.first())
            })
    }

    /// Cycles the active virtual camera channel (`Nose Nav`, `Belly Nadir`, `Left Pattern Window`,
    /// `Tower PTZ Auto-Track`, `Threshold Final Cam`) and enters `SensorAperture` view.
    pub fn cycle_virtual_camera(&mut self, profile: Option<ScenarioProfile>) -> VirtualCameraSpec {
        if self.supervisor.viewport_mode() == OperatorViewportMode::SensorAperture {
            self.virtual_camera_index = self.virtual_camera_index.wrapping_add(1) % 5;
        } else {
            self.supervisor
                .set_viewport_mode(OperatorViewportMode::SensorAperture);
        }
        self.active_virtual_camera(profile)
    }

    /// Currently selected [`VirtualCameraSpec`] for `profile`'s active airport.
    #[must_use]
    pub fn active_virtual_camera(&self, profile: Option<ScenarioProfile>) -> VirtualCameraSpec {
        let id = self.effective_aerodrome(profile);
        if let Some(diag) = self.diagram(id) {
            let specs = pattern_virtual_cameras(diag);
            if let Some(spec) = specs.get(self.virtual_camera_index % specs.len().max(1)) {
                return *spec;
            }
        }
        VirtualCameraSpec {
            id: "nose_nav",
            title: "Nose Nav Camera",
            placement: sim_core::VirtualCameraPlacement::BodyMounted(
                sim_core::CameraMount::forward_nose(DVec3::new(1.25, 0.0, -0.15)),
            ),
            base_intrinsics: VirtualCameraSpec::pinhole_800x600(53.0_f64.to_radians()),
            period: sim_core::CAMERA_DEFAULT_PERIOD,
            depth_readout: true,
        }
    }

    /// Active [`SimTimeScale`].
    #[must_use]
    pub const fn time_scale(&self) -> SimTimeScale {
        self.supervisor.time_scale()
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

    /// Cycles simulation warp (`1x` → `2x` → `4x` → `1x`).
    pub fn cycle_warp(&mut self) -> SimTimeScale {
        let next = match self.supervisor.time_scale() {
            SimTimeScale::Paused | SimTimeScale::Warp(_)
                if (self.supervisor.time_scale().factor() - 4.0).abs() < 0.1 =>
            {
                SimTimeScale::RealTime
            }
            SimTimeScale::RealTime => SimTimeScale::Warp(2.0),
            SimTimeScale::Warp(w) if w < 3.0 => SimTimeScale::Warp(4.0),
            SimTimeScale::Paused | SimTimeScale::Warp(_) => SimTimeScale::RealTime,
        };
        self.supervisor.set_time_scale(next);
        next
    }

    /// Updates omniscient truth traffic kinematics at `now` and projects radio-visible tracks
    /// into [`SimSupervisor::operator_view`]'s `belief_world`.
    pub fn sync(&mut self, ownship_ecef: DVec3, now: SimInstant, profile: Option<ScenarioProfile>) {
        let t_s = now.duration_since(SimInstant::START).as_seconds().max(0.0);
        let now_us = (t_s * 1_000_000.0) as u64;
        let id = self.effective_aerodrome(profile);
        let anchor = self.diagram(id).map_or_else(
            || {
                GeodeticPosition::wgs84(
                    DELAWARE_RUNWAY_LAT_DEG,
                    DELAWARE_RUNWAY_LON_DEG,
                    DELAWARE_RUNWAY_HEIGHT_M,
                )
            },
            |d| d.surface_graph.reference_point,
        );
        self.ingest_truth_traffic(anchor, t_s, now_us);
        self.supervisor
            .sync_belief(ownship_ecef, now, self.start_utc);
    }

    /// Current revision of the operator's `belief_world` surveillance store.
    #[must_use]
    pub fn belief_surveillance_revision(&self) -> u64 {
        self.supervisor
            .operator_view()
            .belief_world
            .surveillance
            .revision()
    }

    /// Avionics-belief 3D traffic entities derived strictly from `OperatorBeliefView::belief_world`.
    #[must_use]
    pub fn belief_entities(&self, visibility: ViewSlotMask) -> Vec<DynamicEntity3D> {
        let store = &self.supervisor.operator_view().belief_world.surveillance;
        let mut entities = entities_from_surveillance_belief(store, 0.0);
        for entity in &mut entities {
            entity.placement.view_mask = visibility;
        }
        entities
    }

    /// Omniscient physical-truth 3D traffic entities (rendered only in `TruthSceneMap`).
    #[must_use]
    pub fn truth_entities(&self, visibility: ViewSlotMask) -> Vec<DynamicEntity3D> {
        let mut entities =
            entities_from_surveillance_belief(&self.supervisor.truth_world().surveillance, 0.0);
        for entity in &mut entities {
            entity.placement.view_mask = visibility;
        }
        entities
    }

    fn ingest_truth_traffic(&mut self, anchor: GeodeticPosition, t_s: f64, now_us: u64) {
        let Ok(plane) = LocalTangentPlane::new(anchor) else {
            return;
        };
        let near_pos = plane.to_geodetic(Ned {
            north_m: 850.0 + 12.0 * (t_s * 0.25).sin() * 20.0,
            east_m: 2_800.0 + 46.0 * t_s,
            down_m: -420.0,
        });
        let _near_outcome = self.supervisor.truth_world_mut().surveillance.ingest(
            TrackObservation {
                key: TrackKey {
                    namespace: AddressNamespace::Icao24,
                    address: 0xA1_B2_C3,
                },
                source: ObservationSource::Adsb1090,
                delivery: DeliveryPath::Simulator,
                observed_at_us: now_us,
                position: near_pos,
                velocity_ned_m_s: Ned {
                    north_m: 3.0,
                    east_m: 46.0,
                    down_m: 0.0,
                },
                heading_rad: Some(DELAWARE_RUNWAY_BEARING_RAD),
                callsign: Callsign::parse("N172SP"),
                category: EmitterCategory::Light,
                on_ground: false,
                quality: IntegrityQuality::default(),
            },
            now_us,
        );

        let far_pos = plane.to_geodetic(Ned {
            north_m: 1_200.0,
            east_m: 45_000.0,
            down_m: -120.0,
        });
        let _far_outcome = self.supervisor.truth_world_mut().surveillance.ingest(
            TrackObservation {
                key: TrackKey {
                    namespace: AddressNamespace::RemoteId,
                    address: 0xD0_00_99,
                },
                source: ObservationSource::RemoteId,
                delivery: DeliveryPath::Simulator,
                observed_at_us: now_us,
                position: far_pos,
                velocity_ned_m_s: Ned {
                    north_m: 0.0,
                    east_m: 8.0,
                    down_m: 0.0,
                },
                heading_rad: Some(1.57),
                callsign: Callsign::parse("UAS99"),
                category: EmitterCategory::Uav,
                on_ground: false,
                quality: IntegrityQuality::default(),
            },
            now_us,
        );
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

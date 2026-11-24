//! Unified flight-deck, EFB, GCS, and synthetic-vision session orchestrator combining telemetry links,
//! Digital Crown passthrough blend, 3D terrain, 3D airspace, ADS-B traffic, and `Indicate` instruments.

mod view;

use crate::airspace::{AirspaceAlert, AirspaceCoordinator, AirspaceRenderVolume};
use crate::blend::{BlendWeights, CrownBlend};
use crate::connext::{
    ConnextIngestor, REAL_SESSION_TSV, ReplayRow, apply_connext_event, parse_capture_tsv,
};
use crate::error::FlightDeckError;
use crate::gdl90::{Gdl90Decoder, apply_gdl90_message};
use crate::instruments::{CameraViewPose, HeadPose, InstrumentScenes, InstrumentSuite};
use crate::kml::{KmlFlightTrack, KmlReplayPlayer, KmlReplayStatus};
use crate::planet::{PlanetaryReference, SkyEvaluator, SkySnapshot};
use crate::scenarios::{FlightDeckScenario, advance_connext_replay_stream, evaluate_scenario};
use crate::telemetry::OwnshipState;
use crate::terrain::{SyntheticTerrainGenerator, TerrainMesh};
use crate::traffic::{TrafficCoordinator, TrafficDisplayTarget};
use world::surveillance::DeliveryPath;
use world_core::DecimalYear;

/// Active telemetry feed driving the Synthetic Vision System.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TelemetrySourceMode {
    /// Built-in deterministic flight & traffic scenario (`0..=2`).
    Scenario(FlightDeckScenario),
    /// Recorded `avionics-link` Connext hardware session (`47N`, `N19880`, `JBU1990`, etc.).
    ConnextReplay,
    /// Live Connext TCP relay / `ExternalAccessory` byte stream (`avionics-link` wire protocol).
    ConnextLive,
    /// Live Wi-Fi GDL-90 UDP (`:4000`) receiver stream (ForeFlight / Sentry / Stratux / Garmin).
    Gdl90Live,
    /// FlightAware / Google Earth KML (`<gx:Track>`, `<LineString>`) or GPX flight track replay.
    KmlReplay,
}

impl Default for TelemetrySourceMode {
    fn default() -> Self {
        Self::Scenario(FlightDeckScenario::NjCorridorCruise)
    }
}

impl TelemetrySourceMode {
    /// Maps a compact integer selector (`0..=6`) to [`TelemetrySourceMode`].
    #[must_use]
    pub const fn from_code(code: u8) -> Self {
        match code {
            0 => Self::Scenario(FlightDeckScenario::NjCorridorCruise),
            1 => Self::Scenario(FlightDeckScenario::HighDensityTrafficCpa),
            2 => Self::Scenario(FlightDeckScenario::KewrApproachWithTfr),
            3 => Self::ConnextReplay,
            4 => Self::ConnextLive,
            6 => Self::KmlReplay,
            _ => Self::Gdl90Live,
        }
    }

    /// Compact integer selector (`0..=6`) for wire serialization.
    #[must_use]
    pub const fn code(self) -> u8 {
        match self {
            Self::Scenario(FlightDeckScenario::NjCorridorCruise) => 0,
            Self::Scenario(FlightDeckScenario::HighDensityTrafficCpa) => 1,
            Self::Scenario(FlightDeckScenario::KewrApproachWithTfr) => 2,
            Self::ConnextReplay => 3,
            Self::ConnextLive => 4,
            Self::Gdl90Live => 5,
            Self::KmlReplay => 6,
        }
    }
}

/// Complete, self-contained XR Synthetic Vision frame snapshot.
#[derive(Clone, Debug)]
pub struct FlightDeckFrameSnapshot {
    /// Explicitly declared planetary body and vertical datum for this frame.
    pub planet: PlanetaryReference,
    /// Active telemetry source mode.
    pub source_mode: TelemetrySourceMode,
    /// Resolved Digital Crown virtual/reality opacity weights (`passthrough_fraction >= 0.15`).
    pub blend: BlendWeights,
    /// Current ownship navigation and attitude state.
    pub ownship: OwnshipState,
    /// Current headset pose relative to cockpit forward axes.
    pub head_pose: HeadPose,
    /// 3D synthetic terrain and runway/HITS mesh in local ENU metres.
    pub terrain: TerrainMesh,
    /// Extruded 3D controlled/SUA/TFR airspace volumes in local ENU metres.
    pub airspaces: Vec<AirspaceRenderVolume>,
    /// Highest-priority airspace containment or proximity alert.
    pub airspace_alert: AirspaceAlert,
    /// 3D ADS-B/TIS-B traffic targets sorted by slant range with callsigns and CPA threat levels.
    pub traffic: Vec<TrafficDisplayTarget>,
    /// Emitted `Indicate` HWD, HUD, PFD, HSI, and Terrain Volume scene buffers.
    pub instruments: InstrumentScenes,
    /// Total Connext inner records decoded in this session.
    pub connext_records: u64,
    /// Total CRC-valid GDL-90 frames decoded in this session.
    pub gdl90_frames: u64,
}

/// Top-level stateful session driving iPadOS EFB/GCS and Apple Vision Pro flight-deck rendering and cockpit
/// instruments on an explicitly declared [`PlanetaryReference`].
pub struct FlightDeckSession {
    planet: PlanetaryReference,
    source_mode: TelemetrySourceMode,
    mode_started_ms: Option<u64>,
    epoch: DecimalYear,
    crown: CrownBlend,
    head_pose: HeadPose,
    camera_view: Option<CameraViewPose>,
    onboard_dem_datum_m: Option<f64>,
    flight_plan_label: Option<String>,
    designated_track_id: Option<u64>,
    ownship: OwnshipState,
    connext: ConnextIngestor,
    replay_rows: Vec<ReplayRow>,
    replay_cursor: usize,
    kml_player: Option<KmlReplayPlayer>,
    kml_raw_bytes: Option<Vec<u8>>,
    gdl90: Gdl90Decoder,
    traffic: TrafficCoordinator,
    airspace: AirspaceCoordinator,
    terrain: SyntheticTerrainGenerator,
    instruments: InstrumentSuite,
    sky: SkyEvaluator,
}

impl FlightDeckSession {
    /// Creates an initialized [`FlightDeckSession`] on an explicitly declared [`PlanetaryReference`].
    ///
    /// Callers must declare `planet` explicitly (`PlanetaryReference::earth_wgs84_egm96()`,
    /// `moon_iau()`, `mars_iau()`, etc.) — there is intentionally no implicit Earth constructor.
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError`] if built-in terrain, airspace, or capture fixtures fail validation.
    pub fn new_on(planet: PlanetaryReference) -> Result<Self, FlightDeckError> {
        let replay_rows = parse_capture_tsv(REAL_SESSION_TSV)?;
        Ok(Self {
            planet,
            source_mode: TelemetrySourceMode::default(),
            mode_started_ms: None,
            epoch: crate::airspace::catalog::DEFAULT_EPOCH,
            crown: CrownBlend::default(),
            head_pose: HeadPose::default(),
            camera_view: None,
            onboard_dem_datum_m: None,
            flight_plan_label: None,
            designated_track_id: None,
            ownship: OwnshipState::default(),
            connext: ConnextIngestor::new(),
            replay_rows,
            replay_cursor: 0,
            kml_player: None,
            kml_raw_bytes: None,
            gdl90: Gdl90Decoder::new(),
            traffic: TrafficCoordinator::new()?,
            airspace: AirspaceCoordinator::builtin_ny_nj()?,
            terrain: SyntheticTerrainGenerator::builtin_ny_nj()?,
            instruments: InstrumentSuite::new(),
            sky: SkyEvaluator::new(),
        })
    }

    /// Explicitly declared [`PlanetaryReference`] for this session.
    #[must_use]
    pub const fn planet(&self) -> PlanetaryReference {
        self.planet
    }

    /// Re-declares the active [`PlanetaryReference`], resetting Earth-specific traffic/geomag
    /// state and re-evaluating any loaded KML track against the new body's ellipsoid and gravity.
    pub fn set_planet(&mut self, planet: PlanetaryReference, now_ms: u64) {
        if self.planet == planet {
            return;
        }
        self.planet = planet;
        self.mode_started_ms = None;
        self.ownship = OwnshipState::default();
        self.traffic.clear();
        if let Some(bytes) = self.kml_raw_bytes.clone()
            && let Ok(track) = KmlFlightTrack::parse_bytes_on(&bytes, &self.planet)
        {
            let mut player = KmlReplayPlayer::new(track, now_ms);
            player.advance_and_sample(now_ms, self.epoch, &self.planet, &mut self.ownship);
            self.kml_player = Some(player);
        }
    }

    /// Sets the conformal HUD vertical field of view (`radians`, clamped to `5°..=120°`).
    pub fn set_hud_vertical_fov_rad(&mut self, fov_rad: f32) {
        self.instruments.set_hud_vertical_fov_rad(fov_rad);
    }

    /// Evaluates ephemeris-driven Sun, Moon, atmosphere, and Johnson-V starfield on `self.planet`
    /// at `utc_unix_s`, using valid ownship coordinates when present or the supplied fallback camera
    /// coordinates otherwise.
    pub fn evaluate_sky(
        &mut self,
        utc_unix_s: f64,
        fallback_lat_deg: f64,
        fallback_lon_deg: f64,
        fallback_alt_datum_m: f64,
    ) -> SkySnapshot {
        let (lat, lon, alt) = if self.ownship.gnss_valid {
            (
                self.ownship.latitude_deg.unwrap_or(fallback_lat_deg),
                self.ownship.longitude_deg.unwrap_or(fallback_lon_deg),
                self.ownship
                    .best_altitude_m()
                    .unwrap_or(fallback_alt_datum_m),
            )
        } else {
            (fallback_lat_deg, fallback_lon_deg, fallback_alt_datum_m)
        };
        self.sky.evaluate(&self.planet, utc_unix_s, lat, lon, alt)
    }

    /// Packed `[dir_x, dir_y, dir_z, v_mag, r, g, b]` `f32` buffer from the latest [`Self::evaluate_sky`].
    #[must_use]
    pub fn sky_stars_f32(&self) -> &[f32] {
        self.sky.packed_stars_f32()
    }

    /// Number of stars currently stored in [`Self::sky_stars_f32`].
    #[must_use]
    pub fn sky_star_count(&self) -> u32 {
        self.sky.star_count()
    }

    /// Switches telemetry source mode and resets transient link/track state cleanly.
    pub fn set_source_mode(&mut self, mode: TelemetrySourceMode) {
        if self.source_mode != mode {
            self.source_mode = mode;
            self.mode_started_ms = None;
            self.ownship = OwnshipState::default();
            self.connext.reset();
            self.replay_cursor = 0;
            self.traffic.clear();
        }
    }

    /// Current [`TelemetrySourceMode`].
    #[must_use]
    pub const fn source_mode(&self) -> TelemetrySourceMode {
        self.source_mode
    }

    /// Sets the normalized Digital Crown turn (`0.0..=1.0`, preserving `>= 15%` passthrough).
    pub fn set_crown_turn(&mut self, turn: f32) {
        self.crown.set_turn(turn);
    }

    /// Current [`CrownBlend`] controller.
    #[must_use]
    pub const fn crown_blend(&self) -> CrownBlend {
        self.crown
    }

    /// Updates pilot headset orientation (`radians`) for HWD off-axis hysteresis.
    pub fn set_head_pose(&mut self, head_pose: HeadPose) {
        self.head_pose = head_pose;
    }

    /// Designates (or clears) a specific ADS-B traffic `track_id` for HUD target lock cues.
    pub fn set_designated_track(&mut self, track_id: Option<u64>) {
        self.designated_track_id = track_id;
    }

    /// Shared access to [`AirspaceCoordinator`].
    #[must_use]
    pub const fn airspace(&self) -> &AirspaceCoordinator {
        &self.airspace
    }

    /// Mutable access to [`AirspaceCoordinator`] (for loading `PMTiles` tiles or NOTAMs).
    pub fn airspace_mut(&mut self) -> &mut AirspaceCoordinator {
        &mut self.airspace
    }

    /// Ingests raw Connext bytes on a live TCP/accessory link at `now_ms`, returning outbound writes.
    pub fn ingest_connext_bytes(&mut self, bytes: &[u8], now_ms: u64) -> Vec<Vec<u8>> {
        let batch = self.connext.ingest(bytes, now_ms);
        for ev in &batch.events {
            let targets = apply_connext_event(&mut self.ownship, ev, now_ms);
            if self.planet.is_earth_wgs84() {
                for t in &targets {
                    self.traffic
                        .ingest_connext_target(t, DeliveryPath::InstalledAvionics, now_ms)
                        .ok();
                }
            }
        }
        self.ownship.refresh_geomag_on(self.epoch, &self.planet);
        batch.writes
    }

    /// Runs the `250 ms` Connext keepalive & `FPN/RP` route-poll timer at `now_ms`.
    pub fn tick_connext(&mut self, now_ms: u64) -> Vec<Vec<u8>> {
        self.connext.tick(now_ms).writes
    }

    /// Ingests raw GDL-90 UDP datagram bytes at `now_ms`.
    pub fn ingest_gdl90_bytes(&mut self, bytes: &[u8], now_ms: u64) {
        let msgs = self.gdl90.ingest(bytes);
        for msg in &msgs {
            if let Some(target) = apply_gdl90_message(&mut self.ownship, msg, now_ms)
                && self.planet.is_earth_wgs84()
            {
                self.traffic.ingest_gdl90_target(&target, now_ms).ok();
            }
        }
        self.ownship.refresh_geomag_on(self.epoch, &self.planet);
    }

    /// Parses KML or GPX flight track bytes on `self.planet`, activates
    /// [`TelemetrySourceMode::KmlReplay`], and initializes ownship kinematics at `t = 0`.
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::KmlTrack`] if `bytes` cannot be decoded as a valid KML/GPX track.
    pub fn load_kml_bytes(&mut self, bytes: &[u8], now_ms: u64) -> Result<(), FlightDeckError> {
        let track = KmlFlightTrack::parse_bytes_on(bytes, &self.planet)?;
        let mut player = KmlReplayPlayer::new(track, now_ms);
        self.source_mode = TelemetrySourceMode::KmlReplay;
        self.mode_started_ms = Some(now_ms);
        self.ownship = OwnshipState::default();
        self.traffic.clear();
        player.advance_and_sample(now_ms, self.epoch, &self.planet, &mut self.ownship);
        self.kml_player = Some(player);
        self.kml_raw_bytes = Some(bytes.to_vec());
        Ok(())
    }

    /// Clears any loaded KML/GPX track preview and returns to `Scenario(NjCorridorCruise)`.
    pub fn clear_kml_replay(&mut self) {
        self.kml_player = None;
        self.kml_raw_bytes = None;
        if self.source_mode == TelemetrySourceMode::KmlReplay {
            self.set_source_mode(TelemetrySourceMode::default());
        }
    }

    /// Updates KML replay pause state and speed multiplier (`0.25x..=30.0x`) at `now_ms`.
    pub fn set_kml_playback(&mut self, paused: bool, speed_multiplier: f64, now_ms: u64) {
        if let Some(player) = &mut self.kml_player {
            player.set_playback(paused, speed_multiplier, now_ms);
        }
    }

    /// Seeks KML replay cursor to normalized `fraction` (`0.0..=1.0`) at `now_ms`.
    pub fn seek_kml_fraction(&mut self, fraction: f64, now_ms: u64) {
        if let Some(player) = &mut self.kml_player {
            player.seek_fraction(fraction, now_ms);
            player.advance_and_sample(now_ms, self.epoch, &self.planet, &mut self.ownship);
        }
    }

    /// Current KML replay progress, speed, and metadata snapshot.
    #[must_use]
    pub fn kml_status(&self) -> KmlReplayStatus {
        self.kml_player
            .as_ref()
            .map_or_else(KmlReplayStatus::default, KmlReplayPlayer::status)
    }

    /// Loaded KML track metadata and `[lon, lat, alt_m]` polyline JSON (`"{}"` when none loaded).
    #[must_use]
    pub fn kml_track_json(&self) -> String {
        self.kml_player
            .as_ref()
            .map_or_else(|| "{}".to_owned(), KmlReplayPlayer::summary_json)
    }

    /// Advances active scenario or capture replay to monotonic `now_ms` and produces a full [`FlightDeckFrameSnapshot`].
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError`] if scenario evaluation or `Indicate` scene emission fails.
    pub fn render_frame(
        &mut self,
        now_ms: u64,
    ) -> Result<FlightDeckFrameSnapshot, FlightDeckError> {
        let start_ms = *self.mode_started_ms.get_or_insert(now_ms);
        let elapsed_ms = now_ms.saturating_sub(start_ms);

        match self.source_mode {
            TelemetrySourceMode::Scenario(scen) => {
                evaluate_scenario(
                    scen,
                    elapsed_ms,
                    now_ms,
                    self.epoch,
                    &self.planet,
                    &mut self.ownship,
                    &mut self.traffic,
                )?;
            }
            TelemetrySourceMode::ConnextReplay => {
                self.pump_connext_replay(elapsed_ms, now_ms);
            }
            TelemetrySourceMode::KmlReplay => {
                if let Some(player) = &mut self.kml_player {
                    player.advance_and_sample(now_ms, self.epoch, &self.planet, &mut self.ownship);
                }
            }
            TelemetrySourceMode::ConnextLive | TelemetrySourceMode::Gdl90Live => {
                self.ownship.refresh_geomag_on(self.epoch, &self.planet);
            }
        }

        let (traffic_list, hud_targets) = self.traffic.snapshot(
            &self.ownship,
            &self.planet,
            self.designated_track_id,
            now_ms,
        );
        let route_label = self.effective_route_label();
        let eye_override = self.active_eye_override();
        let (terrain_mesh, approach_sel) = self.terrain.build_mesh_with_eye(
            &self.ownship,
            &self.planet,
            eye_override,
            route_label.as_deref(),
        );
        let hud_runway = approach_sel.as_ref().map(|a| a.hud_runway);
        let (airspaces, airspace_alert) = self.airspace.snapshot(
            &self.ownship,
            &self.planet,
            terrain_mesh.ground_elevation_msl_m,
            self.epoch,
        );
        let instruments = self.instruments.emit_with_view(
            &self.ownship,
            &self.planet,
            self.head_pose,
            self.camera_view,
            hud_targets,
            approach_sel.as_ref(),
            hud_runway,
            terrain_mesh.ground_elevation_msl_m,
            now_ms,
        )?;

        Ok(FlightDeckFrameSnapshot {
            planet: self.planet,
            source_mode: self.source_mode,
            blend: self.crown.weights(),
            ownship: self.ownship.clone(),
            head_pose: self.head_pose,
            terrain: terrain_mesh,
            airspaces,
            airspace_alert,
            traffic: traffic_list,
            instruments,
            connext_records: self.connext.decoded_records(),
            gdl90_frames: self.gdl90.valid_frames(),
        })
    }

    fn pump_connext_replay(&mut self, elapsed_ms: u64, now_ms: u64) {
        if !self.planet.is_earth_wgs84() {
            return;
        }
        while let Some(row) = self.replay_rows.get(self.replay_cursor) {
            if row.time_ms > elapsed_ms {
                break;
            }
            let row_bytes = row.bytes.clone();
            self.replay_cursor = self.replay_cursor.wrapping_add(1);
            let batch = self.connext.ingest(&row_bytes, now_ms);
            for ev in &batch.events {
                let targets = apply_connext_event(&mut self.ownship, ev, now_ms);
                for t in &targets {
                    self.traffic
                        .ingest_connext_target(t, DeliveryPath::Replay, now_ms)
                        .ok();
                }
            }
        }
        advance_connext_replay_stream(
            elapsed_ms,
            now_ms,
            self.epoch,
            &self.planet,
            &mut self.connext,
            &mut self.ownship,
            &mut self.traffic,
        );
    }
}

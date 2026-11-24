//! Boundary between the single-copy flight/C2 operator session (`sokoly.app`
//! and `simulator.sokoly.app`) and the simulator-only [`SimSupervisor`].
//!
//! Real flight and simulated flight read the exact same [`OperatorBeliefView`]:
//! delayed/range-limited avionics belief ([`world::MovingWorld`]), published
//! non-moving aeronautical baseline ([`flightbag::EditionManifest`]) and the
//! active [`OperatorViewportMode`]. Simulator-only "god-mode" powers — time
//! warp, omniscient physical truth weather/traffic/TFR injection, radio-link
//! filtering and airport runway/gate repositioning — live exclusively on
//! [`SimSupervisor`].

use crate::error::ScenarioError;
use flightbag::{AirportSurfaceGraph, EditionManifest, RunwayDesignator};
use glam::DVec3;
use sim_core::{SimInstant, VehicleState};
use sim_geo::{Geodetic, UtcTime, ecef_to_geodetic};
use world::notices::{FacilityKey, RunwayKey};
use world::surveillance::{
    AddressNamespace, DeliveryPath, EmitterCategory, ObservationOrigin, ObservationTime,
    PositionObservation, ProducerInstanceId, SourceRef, TimeBasis, TrackSnapshot,
    TrafficObservation,
};
use world::{
    DecimalYear, GeodeticPosition, LocalTangentPlane, MovingWorld, MovingWorldConfig, WorldInstant,
};

#[cfg(test)]
mod tests;

const METRES_PER_FOOT: f64 = 0.3048;

/// Shared central viewport mode across `sokoly.app` and `simulator.sokoly.app`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum OperatorViewportMode {
    /// Out-the-window cockpit view with conformal `Indicate` HUD symbology.
    #[default]
    Cockpit,
    /// External chase camera tracking the active vehicle.
    Chase,
    /// Mounted optical/thermal sensor or gimbal aperture view.
    SensorAperture,
    /// 2D/3D aeronautical chart and terrain map (`Flightbag` + `maplibre`).
    ChartMap,
    /// Interactive 3D volumetric instrument (e.g. terrain/traffic/orbital volume).
    VolumetricScene,
}

/// Simulator-only master-clock pacing mode.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum SimTimeScale {
    /// Simulation clock frozen; camera and volumetric inspection remain interactive.
    Paused,
    /// 1:1 wall-clock pacing.
    #[default]
    RealTime,
    /// Accelerated or slowed simulation rate (`> 0.0`).
    Warp(f64),
}

impl SimTimeScale {
    /// Effective clock multiplier passed to [`sim_core::RealTimeClock::advance`].
    pub fn factor(self) -> f64 {
        match self {
            Self::Paused => 0.0,
            Self::RealTime => 1.0,
            Self::Warp(w) if w.is_finite() && w > 0.0 => w,
            Self::Warp(_) => 0.0,
        }
    }
}

/// Single-copy read-only avionics/operator view shared identically between live
/// vehicles (`sokoly.app`) and simulated vehicles (`simulator.sokoly.app`).
///
/// Notice that [`SimSupervisor`]'s omniscient `truth_world` is intentionally
/// absent here: cockpit displays (`Indicate`), EFB maps and autonomy only ever
/// read `belief_world`.
#[derive(Clone, Copy, Debug)]
pub struct OperatorBeliefView<'a> {
    /// Per-vehicle avionics/radio belief of weather, traffic and notices.
    pub belief_world: &'a MovingWorld,
    /// Active published `Flightbag` aeronautical edition, when loaded.
    pub edition: Option<&'a EditionManifest>,
    /// Active operator viewport mode.
    pub viewport_mode: OperatorViewportMode,
}

/// Range and datalink reception policy projecting physical truth into avionics belief.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RadioBeliefPolicy {
    /// Maximum horizontal reception range for ADS-B / UAT / TIS-B / radar tracks, meters.
    pub adsb_range_m: f64,
    /// Maximum horizontal reception range for short-range FLARM / Remote ID tracks, meters.
    pub short_range_radio_m: f64,
    /// Whether FIS-B / network weather products replicate from truth to belief.
    pub receive_weather: bool,
    /// Whether TFR / closure / SUA operational notices replicate from truth to belief.
    pub receive_notices: bool,
}

impl Default for RadioBeliefPolicy {
    fn default() -> Self {
        Self {
            adsb_range_m: 150_000.0,
            short_range_radio_m: 15_000.0,
            receive_weather: true,
            receive_notices: true,
        }
    }
}

/// Why spawning a vehicle onto a `Flightbag` airport surface element was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum RunwaySpawnError {
    /// Requested runway designator does not exist in the airport surface graph.
    #[error("runway end is not present in the airport surface graph")]
    UnknownRunway,
    /// Requested parking stand label does not exist in the airport surface graph.
    #[error("parking stand is not present in the airport surface graph")]
    UnknownParkingSpot,
    /// Active NOTAM in `world::notices` marks the aerodrome or runway closed.
    #[error("runway or aerodrome is closed by an active operational notice")]
    ClosedByNotice,
}

/// Simulator-only supervisor owning omniscient truth, time-warp pacing, airport
/// repositioning and truth-to-belief radio projection.
#[derive(Debug)]
pub struct SimSupervisor {
    time_scale: SimTimeScale,
    viewport_mode: OperatorViewportMode,
    edition: Option<EditionManifest>,
    truth_world: MovingWorld,
    belief_world: MovingWorld,
    belief_surveillance_revision: u64,
    radio_policy: RadioBeliefPolicy,
}

impl SimSupervisor {
    /// Creates a new supervisor with default `1x` real-time pacing and standard radio policy.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError::World`] if either [`MovingWorld`] instance fails configuration.
    pub fn new() -> Result<Self, ScenarioError> {
        let truth_world = MovingWorld::new(MovingWorldConfig::new(
            ProducerInstanceId::new(1),
            world::airmass::ProducerInstanceId::new(1),
        ))?;
        let belief_world = MovingWorld::new(MovingWorldConfig::new(
            ProducerInstanceId::new(2),
            world::airmass::ProducerInstanceId::new(2),
        ))?;
        Ok(Self {
            time_scale: SimTimeScale::default(),
            viewport_mode: OperatorViewportMode::default(),
            edition: None,
            truth_world,
            belief_world,
            belief_surveillance_revision: 0,
            radio_policy: RadioBeliefPolicy::default(),
        })
    }

    /// Simulator time-warp / pause state.
    pub const fn time_scale(&self) -> SimTimeScale {
        self.time_scale
    }

    /// Sets the simulator time-warp / pause state.
    pub fn set_time_scale(&mut self, scale: SimTimeScale) {
        self.time_scale = scale;
    }

    /// Active shared viewport mode.
    pub const fn viewport_mode(&self) -> OperatorViewportMode {
        self.viewport_mode
    }

    /// Switches the shared operator viewport mode.
    pub fn set_viewport_mode(&mut self, mode: OperatorViewportMode) {
        self.viewport_mode = mode;
    }

    /// Attaches the active published `Flightbag` edition manifest.
    pub fn set_edition(&mut self, edition: Option<EditionManifest>) {
        self.edition = edition;
    }

    /// Updates the truth-to-belief radio reception policy.
    pub fn set_radio_policy(&mut self, policy: RadioBeliefPolicy) {
        self.radio_policy = policy;
    }

    /// Omniscient physical truth world (simulator god-mode only).
    pub const fn truth_world(&self) -> &MovingWorld {
        &self.truth_world
    }

    /// Mutable omniscient physical truth world for injecting weather, traffic and TFRs.
    pub fn truth_world_mut(&mut self) -> &mut MovingWorld {
        &mut self.truth_world
    }

    /// Monotonic counter incremented whenever operator `belief_world` traffic publishes a delta.
    pub const fn belief_surveillance_revision(&self) -> u64 {
        self.belief_surveillance_revision
    }

    /// Single-copy avionics/operator view safe to pass to `Indicate` and `sokoly.app` UI.
    pub fn operator_view(&self) -> OperatorBeliefView<'_> {
        OperatorBeliefView {
            belief_world: &self.belief_world,
            edition: self.edition.as_ref(),
            viewport_mode: self.viewport_mode,
        }
    }

    /// Synchronizes `belief_world` from `truth_world` at `ownship_ecef` and `now`,
    /// applying radio reception ranges and expiring stale tracks/notices.
    pub fn sync_belief(&mut self, ownship_ecef: DVec3, now: SimInstant, start_utc: UtcTime) {
        let elapsed_s = now.duration_since(SimInstant::START).as_seconds().max(0.0);
        let raw_now_us = (elapsed_s * 1_000_000.0) as u64;
        let now_us = self
            .truth_world
            .now()
            .map_or(raw_now_us, |prev| raw_now_us.max(prev.monotonic_micros));
        let utc = start_utc.advanced_by(elapsed_s);
        let utc_us = (utc.unix_seconds() * 1_000_000.0).round() as i64;
        let world_now = WorldInstant::new(now_us, utc_us);

        let _truth_adv = self.truth_world.advance(world_now, |_| {});
        if self.radio_policy.receive_weather {
            self.belief_world.aloft = self.truth_world.aloft.clone();
        }
        if self.radio_policy.receive_notices {
            self.belief_world.notices = self.truth_world.notices.clone();
        }

        let own_geo = ecef_to_geodetic(ownship_ecef);
        let own_world = GeodeticPosition::wgs84(
            own_geo.lat_rad.to_degrees(),
            own_geo.lon_rad.to_degrees(),
            own_geo.height_m,
        );
        self.project_surveillance_belief(own_world, now_us);
        let mut delta_seen = false;
        let _belief_adv = self.belief_world.advance(world_now, |_| {
            delta_seen = true;
        });
        if delta_seen {
            self.belief_surveillance_revision = self.belief_surveillance_revision.wrapping_add(1);
        }
    }

    /// Spawns a [`VehicleState`] aligned on a `Flightbag` runway threshold,
    /// refusing the spawn if an active notice in `truth_world` closes that runway.
    pub fn spawn_on_runway(
        &self,
        graph: &AirportSurfaceGraph,
        runway: RunwayDesignator,
        height_above_threshold_m: f64,
        speed_m_s: f64,
        at: DecimalYear,
    ) -> Result<VehicleState, RunwaySpawnError> {
        if let (Ok(fac), Ok(rwy)) = (
            FacilityKey::parse(graph.facility.as_str()),
            RunwayKey::parse(runway.as_str()),
        ) && self.truth_world.notices.is_runway_closed(fac, rwy, at)
        {
            return Err(RunwaySpawnError::ClosedByNotice);
        }
        let spec = graph
            .runway_end(runway)
            .ok_or(RunwaySpawnError::UnknownRunway)?;
        let start = Geodetic::from_degrees(
            spec.threshold.latitude_deg,
            spec.threshold.longitude_deg,
            spec.threshold.ellipsoidal_height_m + height_above_threshold_m,
        );
        Ok(VehicleState::level_flight(
            &start,
            spec.true_bearing_rad,
            speed_m_s,
        ))
    }

    /// Spawns a stationary [`VehicleState`] on a named `Flightbag` apron/gate stand.
    pub fn spawn_at_parking(
        &self,
        graph: &AirportSurfaceGraph,
        label: &str,
    ) -> Result<VehicleState, RunwaySpawnError> {
        let spot = graph
            .parking_spot(label)
            .ok_or(RunwaySpawnError::UnknownParkingSpot)?;
        let pos = Geodetic::from_degrees(
            spot.position.latitude_deg,
            spot.position.longitude_deg,
            spot.position.ellipsoidal_height_m,
        );
        Ok(VehicleState::level_flight(&pos, spot.heading_rad, 0.0))
    }

    fn project_surveillance_belief(&mut self, ownship: GeodeticPosition, now_us: u64) {
        let Ok(plane) = LocalTangentPlane::new(ownship) else {
            return;
        };
        let mut changed = false;
        for handle in self.truth_world.surveillance.snapshots() {
            let track = handle.snapshot();
            let Some(pos_field) = track.position.as_ref() else {
                continue;
            };
            let max_range = if is_short_range_track(track) {
                self.radio_policy.short_range_radio_m
            } else {
                self.radio_policy.adsb_range_m
            };
            let alt_m = track
                .geometric_altitude_ft
                .or(track.pressure_altitude_ft)
                .map_or(0.0, |a| f64::from(a.value) * METRES_PER_FOOT);
            let track_geo = GeodeticPosition::wgs84(
                pos_field.value.latitude_deg,
                pos_field.value.longitude_deg,
                alt_m,
            );
            let Ok(ned) = plane.to_ned(track_geo) else {
                continue;
            };
            if ned.north_m.hypot(ned.east_m) <= max_range {
                let obs = snapshot_to_observation(track, pos_field.provenance.origin, now_us);
                let _res = self.belief_world.surveillance.ingest(obs, |_| {
                    changed = true;
                });
            }
        }
        if changed {
            self.belief_surveillance_revision = self.belief_surveillance_revision.wrapping_add(1);
        }
    }
}

fn is_short_range_track(track: &TrackSnapshot) -> bool {
    matches!(
        track.key.namespace,
        AddressNamespace::SelfAssigned | AddressNamespace::AdsbNonIcao
    ) || track
        .emitter_category
        .is_some_and(|cat| cat.value == EmitterCategory::UncrewedAircraft)
}

fn snapshot_to_observation(
    track: &TrackSnapshot,
    origin: ObservationOrigin,
    now_us: u64,
) -> TrafficObservation {
    let observed_at_micros = track.last_observed_at_micros.min(now_us);
    let mut obs = TrafficObservation::new(
        ObservationTime {
            received_at_micros: now_us,
            observed_at_micros,
            basis: TimeBasis::Mapped {
                uncertainty_micros: 0,
            },
        },
        SourceRef::new(1, 0, DeliveryPath::Simulator),
        origin,
        track.key,
    );
    obs.position = track
        .position
        .as_ref()
        .map(|p| PositionObservation::Decoded(p.value));
    obs.pressure_altitude_ft = track.pressure_altitude_ft.map(|a| a.value);
    obs.geometric_altitude_ft = track.geometric_altitude_ft.map(|a| a.value);
    obs.velocity = track.velocity.map(|v| v.value);
    obs.callsign = track.callsign.clone().map(|c| c.value);
    obs.squawk = track.squawk.map(|s| s.value);
    obs.air_ground = track.air_ground.map(|ag| ag.value);
    obs.emergency = track.emergency.map(|em| em.value);
    obs.emitter_category = track.emitter_category.map(|ec| ec.value);
    obs.quality = track
        .position
        .as_ref()
        .map(|p| p.quality)
        .unwrap_or_default();
    obs
}

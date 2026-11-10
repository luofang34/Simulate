use std::any::Any;
use std::ops::{Deref, DerefMut};
use std::sync::Arc;

use sim_apollo::{CockpitInstrumentPanel, EarthVehicleSources};
use sim_core::{ControlInputs, SimInstant, SimSession, TickSpan, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, UtcTime, ecef_to_geodetic};
use sim_orbital::VesselId;
use sim_scenario::{FlightDynamics, ScenarioProfile, ScenarioRun, ScenarioSnapshot};
use sim_spacetime::{SimEpoch, TimeScaleCatalog};
use sim_universe::{LocalPlant, OrbitalBody, Universe, UniverseCheckpoint, UniverseError};

use super::free_world::{FreeWorld, SolarSystem};
use super::plant_block::PlantBlock;
use super::sensors::{OPTICAL_SENSOR_CHANNEL, OPTICAL_SENSOR_PERIOD};
use super::space_world::SpaceWorld;
use crate::error::AppSimError;
use crate::sky_rig::wall_clock_utc_s;
use crate::vehicles::{FreeFlightPlant, FreeFlightVehicle};
use crate::world_surface::WorldSurfaceHandle;

const SCENARIO_OWNSHIP: VesselId = VesselId(1);

struct ScenarioPlant {
    body: BodyEllipsoid,
    period: TickSpan,
    now: SimInstant,
    state: VehicleState,
}

impl LocalPlant for ScenarioPlant {
    fn body(&self) -> BodyEllipsoid {
        self.body
    }

    fn period(&self) -> TickSpan {
        self.period
    }

    fn now(&self) -> SimInstant {
        self.now
    }

    fn state(&self) -> VehicleState {
        self.state
    }

    fn orbital_body(&self) -> Option<OrbitalBody> {
        None
    }

    fn holds_local_tier(&self) -> bool {
        true
    }

    fn step(&mut self) -> Result<(), UniverseError> {
        self.now = self.now.advanced(self.period);
        Ok(())
    }

    fn advance_to(&mut self, until: SimInstant) -> Result<(), UniverseError> {
        self.now = until;
        Ok(())
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

/// Scripted [`ScenarioRun`] coupled with a [`Universe`] holding the active scenario
/// vehicle as `OWNSHIP` on the scenario's [`SimEpoch`].
pub(super) struct ScenarioWorldRun {
    run: ScenarioRun,
    universe: Universe,
}

impl ScenarioWorldRun {
    pub(super) fn new(run: ScenarioRun, solar: &SolarSystem) -> Result<Self, AppSimError> {
        let mut universe = Universe::new(run.sim_epoch().clone(), solar.context());
        let plant = ScenarioPlant {
            body: run.profile().world().body(),
            period: run.period(),
            now: run.now(),
            state: run.vehicle(),
        };
        universe
            .insert_local(
                SCENARIO_OWNSHIP,
                Box::new(plant),
                Box::new(|_, _, _, _| Err(UniverseError::LocalOnly(SCENARIO_OWNSHIP))),
            )
            .map_err(|e| AppSimError::Universe(Box::new(e)))?;
        Ok(Self { run, universe })
    }

    pub(super) fn step(&mut self) -> Result<(), AppSimError> {
        self.run.step()?;
        let now = self.run.now();
        let state = self.run.vehicle();
        if let Some(plant) = self.universe.local_mut::<ScenarioPlant>(SCENARIO_OWNSHIP) {
            plant.state = state;
        }
        self.universe
            .advance_to(now)
            .map_err(|e| AppSimError::Universe(Box::new(e)))
    }

    #[cfg(test)]
    pub(super) fn universe(&self) -> &Universe {
        &self.universe
    }

    pub(super) fn checkpoint_universe(&self) -> UniverseCheckpoint {
        self.universe.checkpoint()
    }

    pub(super) fn restore_with_universe(
        &mut self,
        saved_run: &ScenarioSnapshot,
        saved_universe: &UniverseCheckpoint,
    ) -> Result<(), AppSimError> {
        self.run.restore(saved_run)?;
        let now = self.run.now();
        let state = self.run.vehicle();
        if let Some(plant) = self.universe.local_mut::<ScenarioPlant>(SCENARIO_OWNSHIP) {
            plant.now = now;
            plant.state = state;
        }
        self.universe
            .restore(saved_universe)
            .map_err(|e| AppSimError::Universe(Box::new(e)))
    }
}

impl Deref for ScenarioWorldRun {
    type Target = ScenarioRun;

    fn deref(&self) -> &Self::Target {
        &self.run
    }
}

impl DerefMut for ScenarioWorldRun {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.run
    }
}

/// Where a free flight starts: the Delaware corridor default, or over an airport runway.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum FreeSpawn {
    /// [`FreeFlightPlant::new`]'s Delaware corridor spawn.
    Delaware,
    /// A runway threshold spawn.
    At {
        /// Spawn position.
        spawn: Geodetic,
        /// True heading at spawn, radians.
        heading_rad: f64,
        /// Field elevation under the spawn, metres.
        field_elev_m: f64,
    },
    /// At rest on the ground.
    Ground {
        /// Spawn position, at the terrain height under it.
        spawn: Geodetic,
        /// True heading at spawn, radians.
        heading_rad: f64,
        /// Whether the engine starts shut down.
        cold: bool,
    },
}

/// What the vehicle is doing: flown by hand on a selectable plant inside a deterministic
/// [`SimSession`] that a [`FreeWorld`] owns, or playing a scripted scenario.
pub(super) enum Mode {
    Free {
        vehicle: FreeFlightVehicle,
        world: Box<FreeWorld>,
        panel: Box<CockpitInstrumentPanel>,
    },
    Scenario(Box<ScenarioWorldRun>),
    /// Spaceflight on `Raketa`, from its launch pad.
    Space(Box<SpaceWorld>),
}

impl Mode {
    /// A scenario flies its own clock; free flight starts at the wall-clock
    /// second it begins, so the sky matches the pilot's day and then follows
    /// sim time.
    pub(super) fn sim_epoch(&self) -> SimEpoch {
        match self {
            Self::Scenario(run) => run.sim_epoch().clone(),
            Self::Free { world, .. } => world.epoch().clone(),
            Self::Space(world) => world.epoch().clone(),
        }
    }

    pub(super) fn free(
        vehicle: FreeFlightVehicle,
        spawn: FreeSpawn,
        surface: &WorldSurfaceHandle,
        scales: &Arc<TimeScaleCatalog>,
        solar: &SolarSystem,
    ) -> Result<Self, AppSimError> {
        Self::free_at(
            vehicle,
            spawn,
            surface,
            scales,
            solar,
            wall_clock_utc_s().floor(),
        )
    }

    pub(super) fn free_at(
        vehicle: FreeFlightVehicle,
        spawn: FreeSpawn,
        surface: &WorldSurfaceHandle,
        scales: &Arc<TimeScaleCatalog>,
        solar: &SolarSystem,
        utc_s: f64,
    ) -> Result<Self, AppSimError> {
        let plant = match spawn {
            FreeSpawn::Delaware => FreeFlightPlant::new(vehicle)?,
            FreeSpawn::At {
                spawn,
                heading_rad,
                field_elev_m,
            } => FreeFlightPlant::new_at(vehicle, spawn, heading_rad, field_elev_m)?,
            FreeSpawn::Ground {
                spawn,
                heading_rad,
                cold,
            } => FreeFlightPlant::new_on_ground(vehicle, spawn, heading_rad, cold)?,
        };
        let mut panel = Box::new(CockpitInstrumentPanel::new(SimInstant::START));
        observe(&mut panel, &plant, SimInstant::START);
        let mut session = SimSession::new(
            PlantBlock::new(plant, surface.clone()),
            ControlInputs::default(),
        );
        session.register_sensor(OPTICAL_SENSOR_CHANNEL, OPTICAL_SENSOR_PERIOD)?;
        let start = UtcTime::from_unix_seconds(utc_s.floor());
        let epoch = SimEpoch::from_utc(start, Arc::clone(scales))?;
        Ok(Self::Free {
            vehicle,
            world: Box::new(FreeWorld::new(session, epoch, solar)?),
            panel,
        })
    }

    /// `Raketa` on its pad, at the wall-clock second the flight begins (or the certified
    /// 2026 equinox default when the host clock lies outside the `2026..2028` kernel span,
    /// recording `epoch_clamped = true`).
    pub(super) fn space(
        surface: &WorldSurfaceHandle,
        scales: &Arc<TimeScaleCatalog>,
        solar: &Arc<SolarSystem>,
    ) -> Result<Self, AppSimError> {
        Self::space_at(surface, scales, solar, wall_clock_utc_s().floor())
    }

    pub(super) fn space_at(
        surface: &WorldSurfaceHandle,
        scales: &Arc<TimeScaleCatalog>,
        solar: &Arc<SolarSystem>,
        raw_utc_s: f64,
    ) -> Result<Self, AppSimError> {
        let (utc_s, clamped) = certified_space_utc_s(raw_utc_s);
        let start = UtcTime::from_unix_seconds(utc_s);
        let epoch = SimEpoch::from_utc(start, Arc::clone(scales))?;
        let ground_m = SpaceWorld::pad_ground_height_m(surface);
        Ok(Self::Space(Box::new(SpaceWorld::on_pad(
            epoch, solar, ground_m, clamped,
        )?)))
    }

    pub(super) fn scenario(
        profile: ScenarioProfile,
        solar: &SolarSystem,
    ) -> Result<Self, AppSimError> {
        let run = ScenarioRun::continuous_pattern(profile, FlightDynamics::Jsbsim)?;
        Ok(Self::Scenario(Box::new(ScenarioWorldRun::new(run, solar)?)))
    }

    pub(super) fn vehicle(&self) -> VehicleState {
        match self {
            Self::Free { world, .. } => world.truth_state(),
            Self::Scenario(run) => run.vehicle(),
            Self::Space(world) => world.vehicle(),
        }
    }

    pub(super) fn period(&self) -> TickSpan {
        match self {
            Self::Free { world, .. } => world.period(),
            Self::Scenario(run) => run.period(),
            Self::Space(_) => SpaceWorld::period(),
        }
    }

    /// Physics instant of the latest completed step.
    pub(super) fn now(&self) -> SimInstant {
        match self {
            Self::Free { world, .. } => world.physics_time(),
            Self::Scenario(run) => run.now(),
            Self::Space(world) => world.now(),
        }
    }
}

/// The step a spaceflight advances by.
pub(super) const fn space_period() -> TickSpan {
    SpaceWorld::period()
}

/// Returns `(utc_s, clamped)` where `utc_s` is `raw_utc_s.floor()` when inside the certified
/// `2026..2028` kernel window (`clamped = false`), or `2026-03-20T12:00:00Z` (`1_774_008_000.0`,
/// `clamped = true`) when outside.
fn certified_space_utc_s(raw_utc_s: f64) -> (f64, bool) {
    let raw = raw_utc_s.floor();
    if (1_767_225_600.0..=1_822_348_800.0).contains(&raw) {
        (raw, false)
    } else {
        (1_774_008_000.0, true)
    }
}

/// Shows the cockpit panel `plant`'s state at `timestamp`.
pub(super) fn observe(
    panel: &mut CockpitInstrumentPanel,
    plant: &FreeFlightPlant,
    timestamp: SimInstant,
) {
    let vehicle = plant.state();
    panel.observe_earth(&EarthVehicleSources {
        timestamp,
        vehicle,
        height_m: ecef_to_geodetic(vehicle.position_ecef).height_m,
    });
}

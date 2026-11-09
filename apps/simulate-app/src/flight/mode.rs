use std::sync::Arc;

use sim_apollo::{CockpitInstrumentPanel, EarthVehicleSources};
use sim_core::{ControlInputs, SimInstant, SimSession, TickSpan, VehicleState};
use sim_geo::{Geodetic, UtcTime, ecef_to_geodetic};
use sim_scenario::{FlightDynamics, ScenarioProfile, ScenarioRun};
use sim_spacetime::{SimEpoch, TimeScaleCatalog};

use super::plant_block::PlantBlock;
use super::sensors::{OPTICAL_SENSOR_CHANNEL, OPTICAL_SENSOR_PERIOD};
use crate::error::AppSimError;
use crate::sky_rig::wall_clock_utc_s;
use crate::vehicles::{FreeFlightPlant, FreeFlightVehicle};
use crate::world_surface::WorldSurfaceHandle;

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
/// [`SimSession`], or playing a scripted scenario.
pub(super) enum Mode {
    Free {
        vehicle: FreeFlightVehicle,
        session: Box<SimSession<PlantBlock>>,
        panel: Box<CockpitInstrumentPanel>,
    },
    Scenario(Box<ScenarioRun>),
}

impl Mode {
    /// A scenario flies its own clock; free flight starts at the wall-clock
    /// second it begins, so the sky matches the pilot's day and then follows
    /// sim time.
    pub(super) fn sim_epoch(
        &self,
        scales: &Arc<TimeScaleCatalog>,
    ) -> Result<SimEpoch, AppSimError> {
        match self {
            Self::Scenario(run) => Ok(run.sim_epoch().clone()),
            Self::Free { .. } => {
                let start = UtcTime::from_unix_seconds(wall_clock_utc_s().floor());
                Ok(SimEpoch::from_utc(start, Arc::clone(scales))?)
            }
        }
    }

    pub(super) fn free(
        vehicle: FreeFlightVehicle,
        spawn: FreeSpawn,
        surface: &WorldSurfaceHandle,
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
        Ok(Self::Free {
            vehicle,
            session: Box::new(session),
            panel,
        })
    }

    pub(super) fn scenario(profile: ScenarioProfile) -> Result<Self, AppSimError> {
        Ok(Self::Scenario(Box::new(ScenarioRun::continuous_pattern(
            profile,
            FlightDynamics::Jsbsim,
        )?)))
    }

    pub(super) fn vehicle(&self) -> VehicleState {
        match self {
            Self::Free { session, .. } => session.truth_state(),
            Self::Scenario(run) => run.vehicle(),
        }
    }

    pub(super) fn period(&self) -> TickSpan {
        match self {
            Self::Free { session, .. } => session.block().plant().period(),
            Self::Scenario(run) => run.period(),
        }
    }

    /// Physics instant of the latest completed step.
    pub(super) fn now(&self) -> SimInstant {
        match self {
            Self::Free { session, .. } => session.physics_time(),
            Self::Scenario(run) => run.now(),
        }
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

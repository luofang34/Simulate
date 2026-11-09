use glam::DVec3;
use sim_core::{
    Block, EngineStartMode, JsbsimFdmBlock, JsbsimFdmConfig, KinematicInput, KinematicModel,
    VehicleState,
};
use sim_geo::{Geodetic, ecef_to_geodetic};
use sim_pack::builtin_c172_package;
use sim_scenery::{AuthoritativeWorldSurface, WorldSurfaceQuery};

use super::PERIOD;
use crate::ScenarioError;

/// Angle of attack the C172 starts at, near its cruise trim, radians.
const JSBSIM_START_ALPHA_RAD: f64 = 0.05;

/// Which [`sim_core::Block`] flies a corridor scenario.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightDynamics {
    /// The stick-to-rate placeholder with no aerodynamics.
    Kinematic,
    /// The JSBSim C172.
    Jsbsim,
}

/// Either flight dynamics source, behind the one input and output they share.
pub(crate) enum Source {
    Kinematic(KinematicModel),
    Jsbsim(Box<JsbsimFdmBlock>),
}

impl Source {
    pub(crate) fn new(kind: FlightDynamics, start: &CorridorStart) -> Result<Self, ScenarioError> {
        Ok(match kind {
            FlightDynamics::Kinematic => Self::Kinematic(KinematicModel::new(
                VehicleState::level_flight(&start.position, start.heading_rad, start.speed_m_s),
            )),
            FlightDynamics::Jsbsim => {
                let c172 = builtin_c172_package()?;
                let files = c172.build_jsbsim_resolver()?;
                let (aircraft_path, model) = c172.jsbsim_model()?;
                Self::Jsbsim(Box::new(JsbsimFdmBlock::new(&JsbsimFdmConfig {
                    resolver: &files,
                    aircraft_path,
                    model,
                    period: PERIOD,
                    initial: start.position,
                    heading_rad: start.heading_rad,
                    speed_m_s: start.speed_m_s,
                    alpha_rad: JSBSIM_START_ALPHA_RAD,
                    engine_start: EngineStartMode::Running,
                })?))
            }
        })
    }

    /// A source flying on from `state`: exactly for the kinematic model, and
    /// for JSBSim trimmed to the same position, heading and speed.
    pub(crate) fn from_state(
        kind: FlightDynamics,
        state: &VehicleState,
    ) -> Result<Self, ScenarioError> {
        match kind {
            FlightDynamics::Kinematic => Ok(Self::Kinematic(KinematicModel::new(*state))),
            FlightDynamics::Jsbsim => {
                let position = ecef_to_geodetic(state.position_ecef);
                let frame = sim_geo::LocalFrame::at(&position);
                let forward = state.forward_ecef();
                let heading_rad = forward
                    .dot(frame.east())
                    .atan2(forward.dot(frame.north()))
                    .rem_euclid(std::f64::consts::TAU);
                Self::new(
                    kind,
                    &CorridorStart {
                        position,
                        heading_rad,
                        speed_m_s: state.velocity_ecef.length(),
                    },
                )
            }
        }
    }

    pub(crate) fn sync_ground_at(&mut self, position_ecef: DVec3) -> Result<(), ScenarioError> {
        if let Self::Jsbsim(model) = self {
            let geo = ecef_to_geodetic(position_ecef);
            if let Some(sample) =
                AuthoritativeWorldSurface::regional_earth_shared()?.sample_surface(&geo)?
            {
                model.sync_ground_elevation_m(sample.ellipsoidal_height_m.0)?;
            }
        }
        Ok(())
    }

    pub(crate) fn step(&mut self, input: &KinematicInput) -> Result<(), ScenarioError> {
        match self {
            Self::Kinematic(model) => model.step(input)?,
            Self::Jsbsim(model) => model.step(input)?,
        }
        Ok(())
    }

    pub(crate) fn output(&self) -> VehicleState {
        match self {
            Self::Kinematic(model) => model.output(),
            Self::Jsbsim(model) => model.output(),
        }
    }
}

pub(crate) struct CorridorStart {
    pub(crate) position: Geodetic,
    pub(crate) heading_rad: f64,
    pub(crate) speed_m_s: f64,
}

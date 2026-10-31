use sim_core::{
    Block, EngineStartMode, JsbsimFdmBlock, JsbsimFdmConfig, KinematicInput, KinematicModel,
    VehicleState,
};
use sim_geo::Geodetic;
use sim_pack::builtin_light_trainer_package;

use super::PERIOD;
use crate::ScenarioError;

/// Angle of attack the trainer starts at, near its cruise trim, radians.
const JSBSIM_START_ALPHA_RAD: f64 = 0.05;

/// Which [`sim_core::Block`] flies a corridor scenario.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightDynamics {
    /// The stick-to-rate placeholder with no aerodynamics.
    Kinematic,
    /// The JSBSim trainer.
    Jsbsim,
}

/// Either flight dynamics source, behind the one input and output they share.
pub(super) enum Source {
    Kinematic(KinematicModel),
    Jsbsim(Box<JsbsimFdmBlock>),
}

impl Source {
    pub(super) fn new(kind: FlightDynamics, start: &CorridorStart) -> Result<Self, ScenarioError> {
        Ok(match kind {
            FlightDynamics::Kinematic => Self::Kinematic(KinematicModel::new(
                VehicleState::level_flight(&start.position, start.heading_rad, start.speed_m_s),
            )),
            FlightDynamics::Jsbsim => {
                let trainer = builtin_light_trainer_package()?;
                let files = trainer.build_jsbsim_resolver()?;
                let (aircraft_path, model) = trainer.jsbsim_model()?;
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

    pub(super) fn step(&mut self, input: &KinematicInput) -> Result<(), ScenarioError> {
        match self {
            Self::Kinematic(model) => model.step(input)?,
            Self::Jsbsim(model) => model.step(input)?,
        }
        Ok(())
    }

    pub(super) fn output(&self) -> VehicleState {
        match self {
            Self::Kinematic(model) => model.output(),
            Self::Jsbsim(model) => model.output(),
        }
    }
}

pub(super) struct CorridorStart {
    pub(super) position: Geodetic,
    pub(super) heading_rad: f64,
    pub(super) speed_m_s: f64,
}

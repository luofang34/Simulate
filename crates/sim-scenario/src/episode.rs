//! Headless, pure-CPU episodes for batch and reinforcement-learning
//! rollouts: a fixed-wing scenario's start, flown on stick inputs instead
//! of its script, with no window and no GPU.

use glam::{DQuat, DVec3};
use sim_atmosphere::{AtmosphereField, AtmosphereSample, GeoidStandardAtmosphere};
use sim_core::{ControlInputs, KinematicInput, SimInstant, TickSpan, VehicleState};
use sim_devices::WhiteNoise;
use sim_geo::{Geodetic, LocalFrame, UtcTime, ecef_to_geodetic};
use sim_scenery::{AuthoritativeWorldSurface, WorldSurfaceQuery};

use crate::corridor::dynamics::Source;
use crate::run::snapshot::FdmSnapshot;
use crate::{FlightDynamics, ScenarioError, ScenarioProfile, ScenarioRun};

mod batch;
#[cfg(test)]
mod tests;

pub use batch::BatchScenarioRunner;

/// The step period of every episode, matching the scripted scenarios.
const PERIOD: TickSpan = sim_core::JSBSIM_DEFAULT_PERIOD;

/// What an [`EpisodeEnv`] flies.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EpisodeConfig {
    /// The scenario whose start the episode begins from; it must be a
    /// fixed-wing profile (a corridor or an airport pattern).
    pub profile: ScenarioProfile,
    /// The flight model the stick drives.
    pub dynamics: FlightDynamics,
    /// Steps after which an episode is truncated.
    pub max_steps: u64,
    /// One-sigma horizontal scatter of the start, per axis, metres; the
    /// reset seed draws it.
    pub start_jitter_m: f64,
    /// One-sigma scatter of the start heading, radians.
    pub start_heading_jitter_rad: f64,
}

impl Default for EpisodeConfig {
    /// A minute of the Delaware cruise start on the kinematic model, with no
    /// scatter.
    fn default() -> Self {
        Self {
            profile: ScenarioProfile::CorridorNominalCruise,
            dynamics: FlightDynamics::Kinematic,
            max_steps: 120 * 60,
            start_jitter_m: 0.0,
            start_heading_jitter_rad: 0.0,
        }
    }
}

/// What a policy sees after a reset or a step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EpisodeObservation {
    /// Steps taken since the reset.
    pub step: u64,
    /// Simulation time.
    pub now: SimInstant,
    /// The vehicle, body-fixed.
    pub vehicle: VehicleState,
    /// The vehicle's position; its height is ellipsoidal.
    pub position: Geodetic,
    /// Height above EGM96 mean sea level, metres.
    pub orthometric_msl_m: f64,
    /// Height above the regional terrain, where the terrain is known.
    pub agl_m: Option<f64>,
    /// Still ISA air at the orthometric height, which is what puts the
    /// geoid into pressure and density.
    pub atmosphere: Option<AtmosphereSample>,
    /// The controls applied on the last step; neutral after a reset.
    pub controls: ControlInputs,
}

/// One transition.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EpisodeStep {
    /// The state after the step.
    pub observation: EpisodeObservation,
    /// The vehicle reached the ground.
    pub terminated: bool,
    /// The step budget ran out.
    pub truncated: bool,
}

/// An [`EpisodeEnv`] at one instant, for [`EpisodeEnv::restore`].
#[derive(Debug, Clone)]
pub struct EpisodeSnapshot {
    source: FdmSnapshot,
    step: u64,
    now: SimInstant,
    controls: ControlInputs,
}

/// A stick-driven episode on a fixed-wing scenario's start.
pub struct EpisodeEnv {
    config: EpisodeConfig,
    start: VehicleState,
    source: Option<Source>,
    step: u64,
    now: SimInstant,
    controls: ControlInputs,
}

impl EpisodeEnv {
    /// An environment for `config`, not yet reset. Fails for the Apollo
    /// profiles, which take no stick.
    pub fn new(config: EpisodeConfig) -> Result<Self, ScenarioError> {
        if matches!(
            config.profile,
            ScenarioProfile::Apollo11LunarLanding | ScenarioProfile::Apollo11CapsuleReentry
        ) {
            return Err(ScenarioError::Unsupported {
                detail: "episodes fly fixed-wing profiles only",
            });
        }
        let (start, _) = ScenarioRun::with_dynamics(config.profile, FlightDynamics::Kinematic)?
            .take_manual_control_state();
        Ok(Self {
            config,
            start,
            source: None,
            step: 0,
            now: SimInstant::START,
            controls: ControlInputs::default(),
        })
    }

    /// The step period.
    #[must_use]
    pub fn period(&self) -> TickSpan {
        PERIOD
    }

    /// Starts a new episode at the profile's start, scattered by `seed`.
    /// The same seed gives the same episode.
    pub fn reset(&mut self, seed: u64) -> Result<EpisodeObservation, ScenarioError> {
        let start = scattered(&self.start, &self.config, seed);
        self.source = Some(Source::from_state(self.config.dynamics, &start)?);
        self.step = 0;
        self.now = SimInstant::START;
        self.controls = ControlInputs::default();
        self.observation()
    }

    /// Advances one period on `controls`.
    pub fn step(&mut self, controls: ControlInputs) -> Result<EpisodeStep, ScenarioError> {
        let source = self.source.as_mut().ok_or(ScenarioError::Unsupported {
            detail: "an episode must be reset before it steps",
        })?;
        let controls = controls.clamped();
        source.sync_ground_at(source.output().position_ecef)?;
        source.step(&KinematicInput { controls })?;
        self.step = self.step.wrapping_add(1);
        self.now = self.now.advanced(PERIOD);
        self.controls = controls;
        let observation = self.observation()?;
        Ok(EpisodeStep {
            observation,
            terminated: observation.agl_m.is_some_and(|agl| agl <= 0.0),
            truncated: self.step >= self.config.max_steps,
        })
    }

    /// The current observation.
    pub fn observation(&self) -> Result<EpisodeObservation, ScenarioError> {
        let vehicle = self.source.as_ref().map_or(self.start, Source::output);
        let position = ecef_to_geodetic(vehicle.position_ecef);
        let agl_m = AuthoritativeWorldSurface::regional_earth_shared()?
            .agl_clearance_m(&position)?
            .map(|agl| agl.0);
        let atmosphere = GeoidStandardAtmosphere::new()
            .sample(&position, UtcTime::from_unix_seconds(0.0))
            .ok();
        Ok(EpisodeObservation {
            step: self.step,
            now: self.now,
            vehicle,
            position,
            orthometric_msl_m: sim_geo::ellipsoidal_to_msl_egm96(&position).0,
            agl_m,
            atmosphere,
            controls: self.controls,
        })
    }

    /// Captures the episode, or `None` before the first reset.
    #[must_use]
    pub fn snapshot(&self) -> Option<EpisodeSnapshot> {
        let source = match self.source.as_ref()? {
            Source::Kinematic(model) => FdmSnapshot::of_kinematic(model),
            Source::Jsbsim(model) => FdmSnapshot::of_jsbsim(model),
        };
        Some(EpisodeSnapshot {
            source,
            step: self.step,
            now: self.now,
            controls: self.controls,
        })
    }

    /// Rewinds to `snapshot`, after which stepping replays the same bits.
    /// Fails, changing nothing, before a reset or for a snapshot of another
    /// flight model.
    pub fn restore(&mut self, snapshot: &EpisodeSnapshot) -> Result<(), ScenarioError> {
        match self.source.as_mut() {
            Some(Source::Kinematic(model)) => snapshot.source.restore_kinematic(model)?,
            Some(Source::Jsbsim(model)) => snapshot.source.restore_jsbsim(model)?,
            None => {
                return Err(ScenarioError::Unsupported {
                    detail: "an episode must be reset before it restores",
                });
            }
        }
        self.step = snapshot.step;
        self.now = snapshot.now;
        self.controls = snapshot.controls;
        Ok(())
    }
}

/// `start` moved and turned by the seed's draw of the configured scatter.
fn scattered(start: &VehicleState, config: &EpisodeConfig, seed: u64) -> VehicleState {
    let draw = WhiteNoise::from_seed(seed).next_vec3(DVec3::new(
        config.start_jitter_m,
        config.start_jitter_m,
        config.start_heading_jitter_rad,
    ));
    let position = ecef_to_geodetic(start.position_ecef);
    let frame = LocalFrame::at(&position);
    let shift = frame.east() * draw.x + frame.north() * draw.y;
    // A positive heading change turns clockwise seen from above.
    let turn = DQuat::from_axis_angle(frame.up(), -draw.z);
    VehicleState {
        position_ecef: start.position_ecef + shift,
        velocity_ecef: turn * start.velocity_ecef,
        attitude: (turn * start.attitude).normalize(),
        ..*start
    }
}

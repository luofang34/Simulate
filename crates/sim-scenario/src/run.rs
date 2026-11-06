use sim_apollo::CockpitInstrumentSnapshot;
use sim_core::{
    AirframeArticulation, CameraPoseEcef, ControlInputs, SimInstant, TickSpan, VehicleState,
};
use sim_scenery::AerodromeId;

use crate::{
    AirportPatternRun, CapsuleReentryRun, CorridorRun, FlightDynamics, LunarLandingConfig,
    LunarLandingRun, ScenarioError, ScenarioProfile,
};

pub(crate) mod snapshot;
#[cfg(test)]
mod tests;

use crate::corridor::snapshot::CorridorState;
use crate::lunar::snapshot::LunarState;
use crate::pattern::snapshot::PatternState;
use crate::reentry::snapshot::ReentryState;

/// The flight behind a [`ScenarioRun`].
enum Flight {
    Corridor(Box<CorridorRun>),
    Lunar(Box<LunarLandingRun>),
    Reentry(Box<CapsuleReentryRun>),
    Pattern(Box<AirportPatternRun>),
}

/// The captured state behind a [`ScenarioSnapshot`].
#[derive(Debug, Clone)]
enum FlightState {
    Corridor(Box<CorridorState>),
    Lunar(Box<LunarState>),
    Reentry(Box<ReentryState>),
    Pattern(Box<PatternState>),
}

/// Everything a [`ScenarioRun`] changes while it flies, captured at one
/// instant so the run can be rewound there and fly the same bits again.
#[derive(Debug, Clone)]
pub struct ScenarioSnapshot {
    profile: ScenarioProfile,
    flight: FlightState,
}

impl ScenarioSnapshot {
    /// The profile the snapshot was taken from.
    #[must_use]
    pub fn profile(&self) -> ScenarioProfile {
        self.profile
    }
}

/// Any scenario in flight, stepped the same way whether a test or a window is
/// driving it. Runs are fully determined by their profile: two runs of one
/// profile fly the same bits.
pub struct ScenarioRun {
    profile: ScenarioProfile,
    flight: Flight,
}

impl ScenarioRun {
    /// `profile` at its first instant, on JSBSim for fixed-wing profiles.
    pub fn new(profile: ScenarioProfile) -> Result<Self, ScenarioError> {
        Self::with_dynamics(profile, FlightDynamics::Jsbsim)
    }

    /// `profile` at its first instant, with corridor and airport-pattern profiles
    /// on `dynamics`. The Apollo profiles fly their own rigid-body vehicles either way.
    pub fn with_dynamics(
        profile: ScenarioProfile,
        dynamics: FlightDynamics,
    ) -> Result<Self, ScenarioError> {
        let flight = match profile {
            ScenarioProfile::Apollo11LunarLanding => Flight::Lunar(Box::new(LunarLandingRun::new(
                &LunarLandingConfig::default(),
            )?)),
            ScenarioProfile::Apollo11CapsuleReentry => {
                Flight::Reentry(Box::new(CapsuleReentryRun::new()?))
            }
            ScenarioProfile::Airport47NUncontrolledPattern => Flight::Pattern(Box::new(
                AirportPatternRun::new(AerodromeId::CentralJersey47N, dynamics)?,
            )),
            ScenarioProfile::AirportKewrToweredPattern => Flight::Pattern(Box::new(
                AirportPatternRun::new(AerodromeId::NewarkKewr, dynamics)?,
            )),
            corridor => Flight::Corridor(Box::new(CorridorRun::new(corridor, dynamics)?)),
        };
        Ok(Self { profile, flight })
    }

    /// Starts a continuous multi-circuit airport traffic pattern (`max_circuits = None`),
    /// ideal for interactive window sessions.
    pub fn continuous_pattern(
        profile: ScenarioProfile,
        dynamics: FlightDynamics,
    ) -> Result<Self, ScenarioError> {
        let aerodrome = match profile {
            ScenarioProfile::Airport47NUncontrolledPattern => AerodromeId::CentralJersey47N,
            ScenarioProfile::AirportKewrToweredPattern => AerodromeId::NewarkKewr,
            other => return Self::with_dynamics(other, dynamics),
        };
        Ok(Self {
            profile,
            flight: Flight::Pattern(Box::new(AirportPatternRun::with_max_circuits(
                aerodrome, dynamics, None,
            )?)),
        })
    }

    /// Borrows the underlying [`AirportPatternRun`] when running an airport pattern profile.
    #[must_use]
    pub fn pattern_run(&self) -> Option<&AirportPatternRun> {
        match &self.flight {
            Flight::Pattern(run) => Some(run),
            _ => None,
        }
    }

    /// The profile being flown.
    pub fn profile(&self) -> ScenarioProfile {
        self.profile
    }

    /// The step period.
    pub fn period(&self) -> TickSpan {
        match &self.flight {
            Flight::Corridor(run) => run.period(),
            Flight::Lunar(run) => run.period(),
            Flight::Reentry(run) => run.period(),
            Flight::Pattern(run) => run.period(),
        }
    }

    /// The instant the vehicle state describes.
    pub fn now(&self) -> SimInstant {
        match &self.flight {
            Flight::Corridor(run) => run.now(),
            Flight::Lunar(run) => run.now(),
            Flight::Reentry(run) => run.now(),
            Flight::Pattern(run) => run.now(),
        }
    }

    /// The true vehicle state, fixed to the central body.
    pub fn vehicle(&self) -> VehicleState {
        match &self.flight {
            Flight::Corridor(run) => run.vehicle(),
            Flight::Lunar(run) => run.vehicle(),
            Flight::Reentry(run) => run.vehicle(),
            Flight::Pattern(run) => run.vehicle(),
        }
    }

    /// Engine, propeller, control-surface, and landing-gear articulation at [`Self::now`].
    #[must_use]
    pub fn articulation(&self) -> AirframeArticulation {
        match &self.flight {
            Flight::Pattern(run) => run.articulation(),
            Flight::Corridor(run) => {
                let elapsed_s = run.now().duration_since(SimInstant::START).as_seconds();
                AirframeArticulation::from_kinematic(
                    &run.vehicle(),
                    ControlInputs {
                        roll: 0.0,
                        pitch: 0.0,
                        yaw: 0.0,
                        throttle: 0.72,
                    },
                    elapsed_s,
                )
            }
            Flight::Lunar(_) | Flight::Reentry(_) => AirframeArticulation::default(),
        }
    }

    /// The live vehicle state and the controls last applied to it, so a
    /// pilot can take the stick out of the script at this exact instant
    /// without a bump: seed a manual flight model with the state and start
    /// the stick, rudder and throttle from these controls. The Apollo
    /// vehicles take no stick inputs and report neutral controls.
    #[must_use]
    pub fn take_manual_control_state(&self) -> (VehicleState, ControlInputs) {
        let controls = match &self.flight {
            Flight::Corridor(run) => run.last_controls(),
            Flight::Pattern(run) => run.last_controls(),
            Flight::Lunar(_) | Flight::Reentry(_) => ControlInputs::default(),
        };
        (self.vehicle(), controls)
    }

    /// The cockpit at [`Self::now`].
    pub fn cockpit(&self) -> CockpitInstrumentSnapshot {
        match &self.flight {
            Flight::Corridor(run) => run.cockpit(),
            Flight::Lunar(run) => run.cockpit(),
            Flight::Reentry(run) => run.cockpit(),
            Flight::Pattern(run) => run.cockpit(),
        }
    }

    /// Where a person aboard looks from.
    pub fn eye_pose(&self) -> Result<CameraPoseEcef, ScenarioError> {
        match &self.flight {
            Flight::Corridor(run) => Ok(run.eye_pose()),
            Flight::Lunar(run) => run.eye_pose(),
            Flight::Reentry(run) => Ok(run.eye_pose()),
            Flight::Pattern(run) => Ok(run.eye_pose()),
        }
    }

    /// Whether the scenario has played out.
    pub fn finished(&self) -> bool {
        match &self.flight {
            Flight::Corridor(run) => run.finished(),
            Flight::Lunar(run) => run.finished(),
            Flight::Reentry(run) => run.finished(),
            Flight::Pattern(run) => run.finished(),
        }
    }

    /// Advances one period. A finished scenario holds still.
    pub fn step(&mut self) -> Result<(), ScenarioError> {
        match &mut self.flight {
            Flight::Corridor(run) => run.step(),
            Flight::Lunar(run) => run.step(),
            Flight::Reentry(run) => run.step(),
            Flight::Pattern(run) => run.step(),
        }
    }

    /// Captures the run at [`Self::now`] for a later [`Self::restore`].
    #[must_use]
    pub fn snapshot(&self) -> ScenarioSnapshot {
        let flight = match &self.flight {
            Flight::Corridor(run) => FlightState::Corridor(Box::new(run.save_state())),
            Flight::Lunar(run) => FlightState::Lunar(Box::new(run.save_state())),
            Flight::Reentry(run) => FlightState::Reentry(Box::new(run.save_state())),
            Flight::Pattern(run) => FlightState::Pattern(Box::new(run.save_state())),
        };
        ScenarioSnapshot {
            profile: self.profile,
            flight,
        }
    }

    /// Rewinds to `snapshot`, after which stepping replays the bits flown
    /// from that instant. Fails, leaving the run untouched, when the snapshot
    /// was taken from another profile or flight dynamics source.
    pub fn restore(&mut self, snapshot: &ScenarioSnapshot) -> Result<(), ScenarioError> {
        if snapshot.profile != self.profile {
            return Err(ScenarioError::SnapshotMismatch {
                detail: "the snapshot was taken from another profile",
            });
        }
        match (&mut self.flight, &snapshot.flight) {
            (Flight::Corridor(run), FlightState::Corridor(state)) => run.restore_state(state),
            (Flight::Lunar(run), FlightState::Lunar(state)) => run.restore_state(state),
            (Flight::Reentry(run), FlightState::Reentry(state)) => run.restore_state(state),
            (Flight::Pattern(run), FlightState::Pattern(state)) => run.restore_state(state),
            _ => Err(ScenarioError::SnapshotMismatch {
                detail: "the snapshot was taken from another kind of flight",
            }),
        }
    }
}

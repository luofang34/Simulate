use sim_apollo::CockpitInstrumentSnapshot;
use sim_core::{CameraPoseEcef, SimInstant, TickSpan, VehicleState};
use sim_scenery::AerodromeId;

use crate::{
    AirportPatternRun, CapsuleReentryRun, CorridorRun, FlightDynamics, LunarLandingConfig,
    LunarLandingRun, ScenarioError, ScenarioProfile,
};

#[cfg(test)]
mod tests;

/// The flight behind a [`ScenarioRun`].
enum Flight {
    Corridor(Box<CorridorRun>),
    Lunar(Box<LunarLandingRun>),
    Reentry(Box<CapsuleReentryRun>),
    Pattern(Box<AirportPatternRun>),
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
}

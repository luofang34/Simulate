//! The holding-burn search as a sequence of short legs, so a host can spread one
//! solve over many frames or run it on another thread and still get exactly the
//! burn a single uninterrupted solve gives.

use glam::DVec3;

use super::{L2HoldingDynamics, L2HoldingTarget};
use crate::{ContinuousPlantState, LagrangeError, TdbEpoch};

/// Steps allowed to solve for one burn.
const SOLVER_STEPS: u32 = 40;
/// A burn is found once its bracket is this narrow (`m/s`).
const BURN_RESOLUTION_M_S: f64 = 1.0e-5;
/// A burn is found once it leaves this little velocity along the line at the crossing
/// (`m/s`).
const CROSSING_RATE_TOLERANCE_M_S: f64 = 1.0e-3;
/// How often the lookahead checks the box and looks for a crossing (`s`); one leg of
/// the search.
pub const LOOKAHEAD_STEP_S: f64 = 2.0 * 86_400.0;

/// How a trajectory ends up, looking ahead from a maintenance epoch.
#[derive(Clone, Copy, PartialEq)]
enum Outlook {
    /// It left the box towards the secondary before the crossing.
    FellInward,
    /// It left the box away from the secondary before the crossing.
    DriftedOutward,
    /// It reached the crossing with this velocity along the line of the two bodies
    /// (`m/s`), positive away from the primary.
    Crossed(f64),
    /// It stayed in the box and did not reach the crossing in the time allowed.
    StillBounded,
}

impl Outlook {
    /// Negative when the spacecraft needs a push away from the primary.
    fn drift(self) -> Option<f64> {
        match self {
            Self::FellInward => Some(f64::NEG_INFINITY),
            Self::DriftedOutward => Some(f64::INFINITY),
            Self::Crossed(rate) => Some(rate),
            Self::StillBounded => None,
        }
    }
}

/// One trial burn's lookahead, part way flown.
#[derive(Clone, Copy)]
struct Lookahead {
    state: ContinuousPlantState,
    flown_s: f64,
    before: DVec3,
    before_rate: f64,
    crossings: u32,
}

/// Where the root-find stands.
#[derive(Clone, Copy)]
enum Stage {
    /// Flying the trajectory with no burn.
    Unburnt,
    /// Flying the largest burn that pushes against the drift found without one.
    Far { drift_now: f64, far: f64 },
    /// Closing in on the burn by false position between `a` and `b`.
    Closing {
        a: f64,
        drift_a: f64,
        b: f64,
        drift_b: f64,
        tried: u32,
    },
    /// The burn is known (`m/s`).
    Found(f64),
}

/// A holding-burn solve that advances one lookahead leg at a time; see
/// [`L2HoldingDynamics::holding_burn_m_s`] for what it finds.
#[derive(Clone, Copy)]
pub struct HoldingBurnSearch {
    target: L2HoldingTarget,
    state: ContinuousPlantState,
    epoch: TdbEpoch,
    stage: Stage,
    trial_m_s: f64,
    look: Option<Lookahead>,
}

impl HoldingBurnSearch {
    /// A search for the burn at `epoch` that holds `state` towards `target`.
    ///
    /// # Errors
    /// [`LagrangeError::InvalidHoldingTarget`] for an out-of-range target.
    pub fn new(
        target: L2HoldingTarget,
        state: ContinuousPlantState,
        epoch: TdbEpoch,
    ) -> Result<Self, LagrangeError> {
        if !target.is_valid() {
            return Err(LagrangeError::InvalidHoldingTarget);
        }
        Ok(Self {
            target,
            state,
            epoch,
            stage: Stage::Unburnt,
            trial_m_s: 0.0,
            look: None,
        })
    }

    /// The burn (`m/s`) once found, without flying anything.
    #[must_use]
    pub const fn found(&self) -> Option<f64> {
        match self.stage {
            Stage::Found(burn) => Some(burn),
            Stage::Unburnt | Stage::Far { .. } | Stage::Closing { .. } => None,
        }
    }

    /// Flies one lookahead leg of at most two days in `dynamics` and
    /// returns the burn once it is found. `dynamics` must be the same on every call.
    ///
    /// # Errors
    /// [`LagrangeError::NoHoldingBurn`] when no burn within the limit turns the drift
    /// around, and the frame, ephemeris and propagation errors of the prediction.
    pub fn advance(
        &mut self,
        dynamics: &L2HoldingDynamics<'_>,
    ) -> Result<Option<f64>, LagrangeError> {
        if let Some(burn) = self.found() {
            return Ok(Some(burn));
        }
        let mut look = match self.look {
            Some(look) => look,
            None => {
                let state = dynamics.burned(&self.state, self.epoch, self.trial_m_s)?;
                let (before, _) = dynamics.offset_from_l2(&state, self.epoch)?;
                Lookahead {
                    state,
                    flown_s: 0.0,
                    before,
                    before_rate: 0.0,
                    crossings: 0,
                }
            }
        };
        let outlook = self.fly_leg(dynamics, &mut look)?;
        self.look = Some(look);
        if let Some(outlook) = outlook {
            self.look = None;
            self.decide(outlook)?;
        }
        Ok(self.found())
    }

    /// Flies `look` one leg on; the outlook once the leg settles it.
    fn fly_leg(
        &self,
        dynamics: &L2HoldingDynamics<'_>,
        look: &mut Lookahead,
    ) -> Result<Option<Outlook>, LagrangeError> {
        let target = &self.target;
        if look.flown_s >= target.lookahead_s {
            return Ok(Some(Outlook::StillBounded));
        }
        let step_s = LOOKAHEAD_STEP_S.min(target.lookahead_s - look.flown_s);
        let from = self.epoch.advanced_seconds(look.flown_s);
        let to = self.epoch.advanced_seconds(look.flown_s + step_s);
        look.state = dynamics.coast(&look.state, from, to)?;
        look.flown_s += step_s;
        let (here, velocity) = dynamics.offset_from_l2(&look.state, to)?;
        if here.x > target.x_bound_m {
            return Ok(Some(Outlook::DriftedOutward));
        }
        if here.x < -target.x_bound_m {
            return Ok(Some(Outlook::FellInward));
        }
        if look.before.y * here.y < 0.0 {
            look.crossings = look.crossings.wrapping_add(1);
            if look.crossings >= target.crossings_ahead {
                let fraction = look.before.y / (look.before.y - here.y);
                return Ok(Some(Outlook::Crossed(
                    look.before_rate + fraction * (velocity.x - look.before_rate),
                )));
            }
        }
        look.before = here;
        look.before_rate = velocity.x;
        if look.flown_s >= target.lookahead_s {
            return Ok(Some(Outlook::StillBounded));
        }
        Ok(None)
    }

    /// Takes the outlook of the trial burn and picks the next trial, or the answer.
    fn decide(&mut self, outlook: Outlook) -> Result<(), LagrangeError> {
        let limit = self.target.max_burn_m_s;
        let cannot_hold = LagrangeError::NoHoldingBurn {
            epoch: self.epoch,
            max_burn_m_s: limit,
        };
        let Some(drift) = outlook.drift() else {
            let burn = match self.stage {
                Stage::Closing { .. } => self.worth_making(self.trial_m_s),
                Stage::Unburnt | Stage::Far { .. } | Stage::Found(_) => 0.0,
            };
            self.stage = Stage::Found(burn);
            return Ok(());
        };
        // Bracket the burn between one that leaves the spacecraft drifting inward and
        // one that leaves it drifting outward, then close in by false position.
        let (a, drift_a, b, drift_b, tried) = match self.stage {
            Stage::Unburnt => {
                let far = if drift < 0.0 { limit } else { -limit };
                self.stage = Stage::Far {
                    drift_now: drift,
                    far,
                };
                self.trial_m_s = far;
                return Ok(());
            }
            Stage::Far { drift_now, far } => {
                if drift_now * drift > 0.0 {
                    return Err(cannot_hold);
                }
                (0.0, drift_now, far, drift, 0)
            }
            Stage::Closing {
                a,
                drift_a,
                b,
                drift_b,
                tried,
            } => {
                let next = self.trial_m_s;
                let (a, drift_a, b, drift_b) = if drift * drift_a > 0.0 {
                    (next, drift, b, drift_b)
                } else {
                    (a, drift_a, next, drift)
                };
                if drift.abs() < CROSSING_RATE_TOLERANCE_M_S || (b - a).abs() < BURN_RESOLUTION_M_S
                {
                    self.stage = Stage::Found(self.worth_making(next));
                    return Ok(());
                }
                let tried = tried.wrapping_add(1);
                if tried >= SOLVER_STEPS {
                    return Err(cannot_hold);
                }
                (a, drift_a, b, drift_b, tried)
            }
            Stage::Found(_) => return Ok(()),
        };
        let both_reach_the_crossing = drift_a.is_finite() && drift_b.is_finite();
        self.trial_m_s = if both_reach_the_crossing {
            a - drift_a * (b - a) / (drift_b - drift_a)
        } else {
            0.5 * (a + b)
        };
        self.stage = Stage::Closing {
            a,
            drift_a,
            b,
            drift_b,
            tried,
        };
        Ok(())
    }

    fn worth_making(&self, delta_v: f64) -> f64 {
        if delta_v.abs() < self.target.min_burn_m_s {
            0.0
        } else {
            delta_v
        }
    }
}

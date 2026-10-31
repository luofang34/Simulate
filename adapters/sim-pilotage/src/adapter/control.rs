use glam::DVec3;
use pilotage_adapter_api::{
    ActionResult, ApplyOutcome, Disposition, LinkLossEnactError, LinkLossPolicy, RejectReason,
};
use pilotage_protocol::{ControlAction, ControlIntent, ScopedControlFrame};
use pilotage_timing::SimTick;
use sim_core::ControlInputs;

/// Largest commanded rates, rad/s. The kinematic model's full stick reaches
/// these, and the trainer flies well inside them.
pub(super) const MAX_ROLL_RATE_RAD_S: f64 = 1.0;
const MAX_RATES_RAD_S: DVec3 = DVec3::new(MAX_ROLL_RATE_RAD_S, 0.35, 0.2);
/// Rate-loop gains, stick per rad/s and per rad, the corridor autopilot's: the
/// proportional gain stays under 1 because the kinematic model's stick is a
/// rate, answered within one step.
const RATE_KP: f64 = 0.5;
const RATE_KI: f64 = 1.5;

/// One axis of a proportional-integral rate loop. The integrator finds the
/// airframe's trim and stops at the stick's travel.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct RateLoop {
    integral: f64,
}

impl RateLoop {
    fn update(&mut self, error: f64, dt_s: f64) -> f64 {
        self.integral = (self.integral + RATE_KI * error * dt_s).clamp(-1.0, 1.0);
        (RATE_KP * error + self.integral).clamp(-1.0, 1.0)
    }
}

/// A scope's link-loss state.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
enum LinkLoss {
    #[default]
    Clear,
    /// Keep the last command for this many more ticks, then neutralize.
    Holding(u32),
    Neutralized,
}

/// A command with its limits applied, and whether they changed it.
fn limited(value: f32, limit: f64) -> (f64, bool) {
    let value = f64::from(value);
    let safe = if value.is_nan() {
        0.0
    } else {
        value.clamp(-limit, limit)
    };
    (safe, safe != value)
}

fn limited_thrust(value: f32) -> (f64, bool) {
    let value = f64::from(value);
    let safe = if value.is_nan() {
        0.0
    } else {
        value.clamp(0.0, 1.0)
    };
    (safe, safe != value)
}

/// Turns the latest typed command into stick and throttle.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(super) struct ControlLaw {
    rates_rad_s: DVec3,
    thrust: f64,
    roll: RateLoop,
    pitch: RateLoop,
    yaw: RateLoop,
    link_loss: LinkLoss,
}

impl ControlLaw {
    /// A neutral command and an empty trim, with any latch left in place.
    pub(super) fn reset(&mut self) {
        *self = Self {
            link_loss: self.link_loss,
            ..Self::default()
        };
    }

    /// One period's stick and throttle. Disarmed, the motor stops and the rate
    /// loop still flies; neutralized, the command is level rates at idle.
    pub(super) fn controls(&mut self, rates: DVec3, dt_s: f64, armed: bool) -> ControlInputs {
        if let LinkLoss::Holding(remaining) = self.link_loss {
            self.link_loss = match remaining.checked_sub(1) {
                Some(left) => LinkLoss::Holding(left),
                None => LinkLoss::Neutralized,
            };
        }
        if self.link_loss == LinkLoss::Neutralized {
            self.rates_rad_s = DVec3::ZERO;
            self.thrust = 0.0;
        }
        let error = self.rates_rad_s - rates;
        ControlInputs {
            roll: self.roll.update(error.x, dt_s),
            pitch: self.pitch.update(error.y, dt_s),
            yaw: self.yaw.update(error.z, dt_s),
            throttle: if armed { self.thrust } else { 0.0 },
        }
    }

    /// Applies one motion-scope frame. The tick is the caller's to fill in.
    pub(super) fn apply(&mut self, frame: &ScopedControlFrame, armed: &mut bool) -> ApplyOutcome {
        let refuse = |reason| ApplyOutcome::new(SimTick::new(0), Disposition::Rejected(reason));
        if self.link_loss != LinkLoss::Clear {
            return refuse(RejectReason::LinkLossEngaged);
        }
        // The session host translates legacy input before an adapter sees it.
        if !frame.payload.axes.is_empty() || !frame.payload.edges.is_empty() {
            let detail = "the motion scope takes typed intents only".to_owned();
            return refuse(RejectReason::Other(detail));
        }
        let disposition = match frame.intent {
            None if frame.actions.is_empty() => {
                return refuse(RejectReason::Other(
                    "the frame carries no command".to_owned(),
                ));
            }
            None => Disposition::Accepted,
            Some(ControlIntent::BodyRate(intent)) => {
                let (roll, roll_limited) = limited(intent.roll_rate, MAX_RATES_RAD_S.x);
                let (pitch, pitch_limited) = limited(intent.pitch_rate, MAX_RATES_RAD_S.y);
                let (yaw, yaw_limited) = limited(intent.yaw_rate, MAX_RATES_RAD_S.z);
                let (thrust, thrust_limited) = limited_thrust(intent.thrust);
                self.rates_rad_s = DVec3::new(roll, pitch, yaw);
                self.thrust = thrust;
                if roll_limited || pitch_limited || yaw_limited || thrust_limited {
                    Disposition::Constrained
                } else {
                    Disposition::Accepted
                }
            }
            Some(_) => {
                let detail = "the motion scope takes body-rate intents only".to_owned();
                return refuse(RejectReason::Other(detail));
            }
        };
        ApplyOutcome {
            tick: SimTick::new(0),
            disposition,
            action_results: frame
                .actions
                .iter()
                .map(|action| match action {
                    ControlAction::Arm => {
                        *armed = true;
                        ActionResult::accepted(*action)
                    }
                    ControlAction::Disarm => {
                        *armed = false;
                        ActionResult::accepted(*action)
                    }
                    other => ActionResult::rejected(*other, "not on the motion scope"),
                })
                .collect(),
        }
    }

    /// Engages or clears the scope's latch. An unsupported policy still
    /// latches and neutralizes, because a fenced scope must never stay
    /// drivable, and the refusal reaches the host as a fault to count.
    pub(super) fn set_link_loss(
        &mut self,
        policy: Option<LinkLossPolicy>,
    ) -> Result<(), LinkLossEnactError> {
        match policy {
            None => self.link_loss = LinkLoss::Clear,
            Some(LinkLossPolicy::Neutralize) => self.link_loss = LinkLoss::Neutralized,
            Some(LinkLossPolicy::HoldBrief { ticks }) => self.link_loss = LinkLoss::Holding(ticks),
            Some(other) => {
                self.link_loss = LinkLoss::Neutralized;
                return Err(LinkLossEnactError::ChannelRejected {
                    detail: format!("{other:?} is not available; neutralized instead"),
                });
            }
        }
        Ok(())
    }
}

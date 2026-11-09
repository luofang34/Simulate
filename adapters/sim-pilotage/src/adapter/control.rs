use std::f64::consts::{PI, TAU};

use glam::{DQuat, DVec3, EulerRot};
use pilotage_adapter_api::{
    ActionResult, ApplyOutcome, Disposition, LinkLossEnactError, LinkLossPolicy, RejectReason,
    attitude_euler,
};
use pilotage_protocol::{
    AttitudeThrustIntent, ControlAction, ControlIntent, ReferenceFrame, ScopedControlFrame,
};
use pilotage_timing::SimTick;
use sim_core::ControlInputs;

/// Largest commanded rates, rad/s. The kinematic model's full stick reaches
/// these, and the C172 flies well inside them.
pub(super) const MAX_ROLL_RATE_RAD_S: f64 = 1.0;
pub(super) const MAX_YAW_RATE_RAD_S: f64 = 0.2;
const MAX_RATES_RAD_S: DVec3 = DVec3::new(MAX_ROLL_RATE_RAD_S, 0.35, MAX_YAW_RATE_RAD_S);
/// Largest commanded bank, rad: a 60-degree bank is the C172's steep-turn
/// limit. Pitch clamps tighter because the airframe stalls first.
pub(super) const MAX_ROLL_RAD: f64 = 1.05;
const MAX_PITCH_RAD: f64 = 0.60;
/// Attitude-loop gains, rad/s of rate demand per rad of error. Each sits well
/// under the rate loop's bandwidth so the two loops do not fight.
const KP_ROLL: f64 = 2.2;
const KP_PITCH: f64 = 2.0;
const KP_YAW: f64 = 1.2;
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

/// Wraps an angle into `[-pi, pi)`, so a heading error takes the short way.
fn wrapped(angle: f64) -> f64 {
    (angle + PI).rem_euclid(TAU) - PI
}

/// Roll, pitch, and yaw of a body-to-NED attitude, radians.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct Euler {
    roll: f64,
    pitch: f64,
    yaw: f64,
}

impl Euler {
    fn of(attitude_ned: DQuat) -> Self {
        let (yaw, pitch, roll) = attitude_ned.to_euler(EulerRot::ZYX);
        Self { roll, pitch, yaw }
    }

    /// Body rates that close the error to `target` proportionally. The
    /// Euler-angle rates map to body axes through the present attitude, so a
    /// pitch demand in a bank is not flown as a turn.
    fn rates_toward(self, target: Self) -> DVec3 {
        let roll_dot = KP_ROLL * wrapped(target.roll - self.roll);
        let pitch_dot = KP_PITCH * (target.pitch - self.pitch);
        let yaw_dot = KP_YAW * wrapped(target.yaw - self.yaw);
        let (sin_roll, cos_roll) = self.roll.sin_cos();
        let (sin_pitch, cos_pitch) = self.pitch.sin_cos();
        let rates = DVec3::new(
            roll_dot - sin_pitch * yaw_dot,
            cos_roll * pitch_dot + sin_roll * cos_pitch * yaw_dot,
            -sin_roll * pitch_dot + cos_roll * cos_pitch * yaw_dot,
        );
        rates.clamp(-MAX_RATES_RAD_S, MAX_RATES_RAD_S)
    }
}

/// What the rate loop chases: commanded rates, or rates that fly an attitude.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Setpoint {
    Rates(DVec3),
    Attitude(Euler),
}

impl Default for Setpoint {
    fn default() -> Self {
        Self::Rates(DVec3::ZERO)
    }
}

/// The attitude and thrust an intent asks for after the envelope, whether the
/// envelope changed them, or why the intent cannot be flown at all.
fn limited_attitude(intent: &AttitudeThrustIntent) -> Result<(Euler, f64, bool), String> {
    if intent.frame != ReferenceFrame::LocalNed {
        return Err(format!(
            "attitude must be against LocalNed, not {:?}",
            intent.frame
        ));
    }
    let components = [intent.qw, intent.qx, intent.qy, intent.qz, intent.thrust];
    if !components.iter().all(|value| value.is_finite()) {
        return Err("the attitude intent has a non-finite component".to_owned());
    }
    let norm = [intent.qw, intent.qx, intent.qy, intent.qz]
        .iter()
        .map(|value| value * value)
        .sum::<f32>()
        .sqrt();
    if norm < f32::EPSILON {
        return Err("the attitude quaternion has no length".to_owned());
    }
    let unit = AttitudeThrustIntent {
        qw: intent.qw / norm,
        qx: intent.qx / norm,
        qy: intent.qy / norm,
        qz: intent.qz / norm,
        ..*intent
    };
    let (roll, pitch, yaw) = attitude_euler(&unit);
    let (roll, roll_limited) = limited(roll, MAX_ROLL_RAD);
    let (pitch, pitch_limited) = limited(pitch, MAX_PITCH_RAD);
    let (thrust, thrust_limited) = limited_thrust(intent.thrust);
    let target = Euler {
        roll,
        pitch,
        yaw: f64::from(yaw),
    };
    Ok((
        target,
        thrust,
        roll_limited || pitch_limited || thrust_limited,
    ))
}

const fn disposition(constrained: bool) -> Disposition {
    if constrained {
        Disposition::Constrained
    } else {
        Disposition::Accepted
    }
}

/// Turns the latest typed command into stick and throttle.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(super) struct ControlLaw {
    setpoint: Setpoint,
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

    /// One period's stick and throttle from the body rates and body-to-NED
    /// attitude. Disarmed, the motor stops and the rate loop still flies;
    /// neutralized, the command is level rates at idle.
    pub(super) fn controls(
        &mut self,
        rates: DVec3,
        attitude_ned: DQuat,
        dt_s: f64,
        armed: bool,
    ) -> ControlInputs {
        if let LinkLoss::Holding(remaining) = self.link_loss {
            self.link_loss = match remaining.checked_sub(1) {
                Some(left) => LinkLoss::Holding(left),
                None => LinkLoss::Neutralized,
            };
        }
        if self.link_loss == LinkLoss::Neutralized {
            self.setpoint = Setpoint::Rates(DVec3::ZERO);
            self.thrust = 0.0;
        }
        let demand = match self.setpoint {
            Setpoint::Rates(demand) => demand,
            Setpoint::Attitude(target) => Euler::of(attitude_ned).rates_toward(target),
        };
        let error = demand - rates;
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
                self.setpoint = Setpoint::Rates(DVec3::new(roll, pitch, yaw));
                self.thrust = thrust;
                disposition(roll_limited || pitch_limited || yaw_limited || thrust_limited)
            }
            Some(ControlIntent::AttitudeThrust(intent)) => match limited_attitude(&intent) {
                Ok((target, thrust, constrained)) => {
                    self.setpoint = Setpoint::Attitude(target);
                    self.thrust = thrust;
                    disposition(constrained)
                }
                Err(detail) => return refuse(RejectReason::Other(detail)),
            },
            Some(_) => {
                let detail = "the motion scope takes body-rate or attitude intents only".to_owned();
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

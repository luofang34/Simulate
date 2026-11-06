use super::{EPSILON, InputMapper};
use crate::{Control, DeviceId, InputTarget, OutputRange, ThrottleBehavior};

/// The source a control currently follows.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ControlOwner {
    /// The keyboard, gamepad buttons, or [`InputMapper::set`].
    #[default]
    Keyboard,
    /// A connected device's axis.
    Device(DeviceId),
}

/// One value per pilot control.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct PerControl<T> {
    pub(crate) roll: T,
    pub(crate) pitch: T,
    pub(crate) yaw: T,
    pub(crate) throttle: T,
}

impl<T> PerControl<T> {
    pub(crate) fn get(&self, control: Control) -> &T {
        match control {
            Control::Roll => &self.roll,
            Control::Pitch => &self.pitch,
            Control::Yaw => &self.yaw,
            Control::Throttle => &self.throttle,
        }
    }

    pub(crate) fn get_mut(&mut self, control: Control) -> &mut T {
        match control {
            Control::Roll => &mut self.roll,
            Control::Pitch => &mut self.pitch,
            Control::Yaw => &mut self.yaw,
            Control::Throttle => &mut self.throttle,
        }
    }
}

/// Whether a new shaped reading is a real input rather than rest noise.
/// Centred axes count when they move or sit off centre, so a stick resting
/// in its deadzone never takes a control; levers count whenever they move,
/// including their first reading, because every lever position is a setting.
pub(super) fn is_meaningful(previous: Option<f64>, shaped: f64, output: OutputRange) -> bool {
    match output {
        OutputRange::Lever => previous.is_none_or(|p| (shaped - p).abs() > EPSILON),
        OutputRange::Centered => {
            (shaped - previous.unwrap_or(0.0)).abs() > EPSILON || shaped.abs() > EPSILON
        }
    }
}

impl InputMapper {
    pub(super) fn stick(&self, control: Control) -> f64 {
        match self.owners.get(control) {
            ControlOwner::Keyboard => self.keyboard.axis(control),
            ControlOwner::Device(d) => self.device_output(*d, InputTarget::Flight(control)),
        }
    }

    pub(super) fn throttle_output(&self) -> f64 {
        let (behavior, climb) = match self.owners.throttle {
            ControlOwner::Keyboard => (self.throttle_behavior(), self.keyboard.climb),
            ControlOwner::Device(d) => (
                self.device_throttle(d),
                self.device_output(d, InputTarget::Flight(Control::Throttle)),
            ),
        };
        match behavior {
            ThrottleBehavior::CenteredClimbRate => 0.5 + 0.5 * climb.clamp(-1.0, 1.0),
            ThrottleBehavior::DirectLever | ThrottleBehavior::LatchingIncremental { .. } => {
                self.latched_throttle
            }
        }
    }

    /// Integrates latched-lever rate commands from keys, buttons, and
    /// rate-command throttle sticks over `dt_s`.
    pub(super) fn integrate_throttle(&mut self, dt_s: f64) {
        use crate::FlightKeyAction as A;
        let keys = self.keyboard.direction(A::ThrottleUp, A::ThrottleDown);
        let key_rate = match self.throttle_behavior() {
            ThrottleBehavior::LatchingIncremental { rate_per_s } => rate_per_s,
            ThrottleBehavior::DirectLever => self.rates.throttle_rate_per_s,
            ThrottleBehavior::CenteredClimbRate => 0.0,
        };
        let mut delta = keys * key_rate * dt_s;
        let target = InputTarget::Flight(Control::Throttle);
        for (&id, state) in &self.devices {
            if let ThrottleBehavior::LatchingIncremental { rate_per_s } = self.device_throttle(id) {
                let command = state.last_shaped.get(&target).copied().unwrap_or(0.0);
                delta += command.clamp(-1.0, 1.0) * rate_per_s * dt_s;
            }
        }
        if delta.is_finite() {
            self.latched_throttle = (self.latched_throttle + delta).clamp(0.0, 1.0);
        }
    }

    /// Hands every control back to the keyboard and forgets device outputs,
    /// after which devices must move again to take a control. Used when the
    /// active bindings change.
    pub(super) fn reset_device_outputs(&mut self) {
        for state in self.devices.values_mut() {
            state.last_shaped.clear();
        }
        self.owners = PerControl::default();
        self.camera_pan = (0.0, None);
        self.camera_tilt = (0.0, None);
    }
}

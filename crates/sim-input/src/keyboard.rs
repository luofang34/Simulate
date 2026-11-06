use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

use crate::{Control, FlightKeyAction};

/// How fast held keys move the virtual keyboard stick.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct KeyboardRates {
    /// Deflection per second toward a held key's end stop.
    pub slew_rate_per_s: f64,
    /// Deflection per second back to centre once keys are released.
    pub center_return_rate_per_s: f64,
    /// Lever travel per second for throttle keys under
    /// [`ThrottleBehavior::DirectLever`](crate::ThrottleBehavior::DirectLever).
    pub throttle_rate_per_s: f64,
}

impl Default for KeyboardRates {
    fn default() -> Self {
        Self {
            slew_rate_per_s: 4.0,
            center_return_rate_per_s: 6.0,
            throttle_rate_per_s: 0.5,
        }
    }
}

impl KeyboardRates {
    /// The same rates with non-finite or negative values replaced by the defaults.
    #[must_use]
    pub fn sanitized(self) -> Self {
        let fallback = Self::default();
        let pick = |v: f64, d: f64| if v.is_finite() && v >= 0.0 { v } else { d };
        Self {
            slew_rate_per_s: pick(self.slew_rate_per_s, fallback.slew_rate_per_s),
            center_return_rate_per_s: pick(
                self.center_return_rate_per_s,
                fallback.center_return_rate_per_s,
            ),
            throttle_rate_per_s: pick(self.throttle_rate_per_s, fallback.throttle_rate_per_s),
        }
    }
}

/// The virtual stick the keyboard (and gamepad buttons) drive: held actions
/// slew each axis toward its end stop and release returns it to centre.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct KeyboardFlightAxes {
    /// Actions held by keys.
    keys: BTreeSet<FlightKeyAction>,
    /// Actions held by gamepad buttons; kept apart so releasing a key does
    /// not release a button holding the same action.
    buttons: BTreeSet<FlightKeyAction>,
    /// Axes last written by [`Self::set_axis`]; they hold their value
    /// instead of self-centring until a key for that axis is pressed.
    direct: BTreeSet<Control>,
    /// Roll deflection, −1..1.
    pub roll: f64,
    /// Pitch deflection, −1..1, positive nose up.
    pub pitch: f64,
    /// Yaw deflection, −1..1.
    pub yaw: f64,
    /// Climb-rate deflection for
    /// [`ThrottleBehavior::CenteredClimbRate`](crate::ThrottleBehavior::CenteredClimbRate), −1..1.
    pub climb: f64,
}

impl KeyboardFlightAxes {
    /// Records a key changing; returns whether the held set changed.
    pub fn set_key(&mut self, action: FlightKeyAction, pressed: bool) -> bool {
        let changed = toggle(&mut self.keys, action, pressed);
        if changed && pressed {
            self.release_direct(action);
        }
        changed
    }

    /// Records a gamepad button changing; returns whether the held set changed.
    pub fn set_button(&mut self, action: FlightKeyAction, pressed: bool) -> bool {
        let changed = toggle(&mut self.buttons, action, pressed);
        if changed && pressed {
            self.release_direct(action);
        }
        changed
    }

    fn release_direct(&mut self, action: FlightKeyAction) {
        if let Some(control) = action_control(action) {
            self.direct.remove(&control);
        }
    }

    /// Whether a key or button holds `action`.
    pub fn is_held(&self, action: FlightKeyAction) -> bool {
        self.keys.contains(&action) || self.buttons.contains(&action)
    }

    /// Every held action, in order.
    pub fn held(&self) -> Vec<FlightKeyAction> {
        self.keys.union(&self.buttons).copied().collect()
    }

    /// Releases every key and button.
    pub fn release_all(&mut self) {
        self.keys.clear();
        self.buttons.clear();
    }

    /// −1, 0, or 1 from a pair of opposing actions.
    pub fn direction(&self, positive: FlightKeyAction, negative: FlightKeyAction) -> f64 {
        f64::from(u8::from(self.is_held(positive))) - f64::from(u8::from(self.is_held(negative)))
    }

    /// Advances every axis `dt_s` seconds toward what the held actions command.
    pub fn slew(&mut self, rates: &KeyboardRates, dt_s: f64) {
        use FlightKeyAction as A;
        let roll = self.direction(A::RollRight, A::RollLeft);
        let pitch = self.direction(A::PitchUp, A::PitchDown);
        let yaw = self.direction(A::YawRight, A::YawLeft);
        let climb = self.direction(A::ThrottleUp, A::ThrottleDown);
        for (control, target) in [
            (Control::Roll, roll),
            (Control::Pitch, pitch),
            (Control::Yaw, yaw),
            (Control::Throttle, climb),
        ] {
            if !self.direct.contains(&control) {
                let next = slew_toward(self.axis(control), target, rates, dt_s);
                self.write_axis(control, next);
            }
        }
    }

    /// The deflection of a stick control; throttle reads as the climb axis.
    pub fn axis(&self, control: Control) -> f64 {
        match control {
            Control::Roll => self.roll,
            Control::Pitch => self.pitch,
            Control::Yaw => self.yaw,
            Control::Throttle => self.climb,
        }
    }

    /// Sets a stick control's deflection directly; it holds there until a
    /// key for that axis is pressed.
    pub fn set_axis(&mut self, control: Control, value: f64) {
        let value = if value.is_nan() {
            0.0
        } else {
            value.clamp(-1.0, 1.0)
        };
        self.direct.insert(control);
        self.write_axis(control, value);
    }

    fn write_axis(&mut self, control: Control, value: f64) {
        match control {
            Control::Roll => self.roll = value,
            Control::Pitch => self.pitch = value,
            Control::Yaw => self.yaw = value,
            Control::Throttle => self.climb = value,
        }
    }

    /// Centres every axis without releasing held actions.
    pub fn centre(&mut self) {
        self.direct.clear();
        self.roll = 0.0;
        self.pitch = 0.0;
        self.yaw = 0.0;
        self.climb = 0.0;
    }
}

/// The control a held action deflects, if any.
pub(crate) fn action_control(action: FlightKeyAction) -> Option<Control> {
    use FlightKeyAction as A;
    match action {
        A::RollLeft | A::RollRight => Some(Control::Roll),
        A::PitchUp | A::PitchDown => Some(Control::Pitch),
        A::YawLeft | A::YawRight => Some(Control::Yaw),
        A::ThrottleUp | A::ThrottleDown => Some(Control::Throttle),
        A::WheelBrake | A::ResetTrim => None,
    }
}

fn toggle(set: &mut BTreeSet<FlightKeyAction>, action: FlightKeyAction, pressed: bool) -> bool {
    if pressed {
        set.insert(action)
    } else {
        set.remove(&action)
    }
}

/// Moves `current` toward `target`, using the slew rate while a key holds
/// the axis and the centre-return rate once released.
fn slew_toward(current: f64, target: f64, rates: &KeyboardRates, dt_s: f64) -> f64 {
    let rate = if target == 0.0 {
        rates.center_return_rate_per_s
    } else {
        rates.slew_rate_per_s
    };
    let step = rate * dt_s;
    let delta = target - current;
    if delta.abs() <= step {
        target
    } else {
        current + step.copysign(delta)
    }
}

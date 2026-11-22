//! Pure touch-cockpit and mobile gyro-tilt shaping, plus the shared touch control state
//! drained by the flight loop.

use sim_input::{Control, InputMapper};

use super::ControlScheme;
use crate::flight::Flight;

const STICK_DEADZONE: f64 = 0.06;
const STICK_EXPO: f64 = 0.35;
const RUDDER_DEADZONE: f64 = 0.08;
const RUDDER_EXPO: f64 = 0.30;
const TILT_DEADZONE: f64 = 0.08;
const TILT_EXPO: f64 = 0.35;
const TILT_ROLL_FULL_DEG: f64 = 35.0;
const TILT_PITCH_FULL_DEG: f64 = 25.0;

/// Discrete actions triggered by the mobile `#touch-cockpit` overlay.
#[cfg_attr(
    not(target_arch = "wasm32"),
    allow(
        dead_code,
        reason = "variants are constructed by wasm32 touch controls and dispatched on wasm32"
    )
)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TouchAction {
    /// Open or close the Flight Hub.
    ToggleHub,
    /// Cycle cockpit/chase/sensor camera mode.
    CycleViewport,
    /// Pause or resume the simulation clock.
    TogglePause,
    /// Extend one flap notch.
    FlapsDown,
    /// Retract one flap notch.
    FlapsUp,
    /// Toggle landing gear up/down.
    ToggleGear,
    /// Toggle EFB tablet expanded/compact view.
    ToggleEfb,
    /// Reset cockpit/chase look offset to straight ahead.
    ResetLook,
}

/// Applies deadzone and cubic expo blending to a signed normalized axis (`-1.0..=1.0`).
#[must_use]
pub fn shape_stick_axis(raw: f64, deadzone: f64, expo: f64) -> f64 {
    if !raw.is_finite() {
        return 0.0;
    }
    let clamped = raw.clamp(-1.0, 1.0);
    let dz = deadzone.clamp(0.0, 0.95);
    let mag = clamped.abs();
    if mag <= dz {
        return 0.0;
    }
    let rescaled = ((mag - dz) / (1.0 - dz)).clamp(0.0, 1.0);
    let e = expo.clamp(0.0, 1.0);
    let blended = rescaled * (1.0 - e) + rescaled.powi(3) * e;
    (clamped.signum() * blended).clamp(-1.0, 1.0)
}

/// Maps circular pad pointer offsets `(dx, dy)` inside `radius` pixels into
/// `[roll, pitch]` axes matching aircraft stick conventions (`+dx` right roll, `+dy` pull up).
#[must_use]
pub fn map_stick_pad(dx: f64, dy: f64, radius: f64) -> [f64; 2] {
    if !(dx.is_finite() && dy.is_finite() && radius.is_finite() && radius > 0.0) {
        return [0.0, 0.0];
    }
    let dist = dx.hypot(dy);
    let scale = if dist > radius { radius / dist } else { 1.0 };
    let nx = (dx * scale) / radius;
    let ny = (dy * scale) / radius;
    [
        shape_stick_axis(nx, STICK_DEADZONE, STICK_EXPO),
        shape_stick_axis(ny, STICK_DEADZONE, STICK_EXPO),
    ]
}

/// Maps a vertical offset from the top of the throttle slider (`height` pixels) into `0.0..=1.0`.
#[must_use]
pub fn map_throttle_track(y_from_top: f64, height: f64) -> f64 {
    if !(y_from_top.is_finite() && height.is_finite() && height > 0.0) {
        return 0.0;
    }
    (1.0 - y_from_top / height).clamp(0.0, 1.0)
}

/// Maps a horizontal offset from the left of the rudder slider (`width` pixels) into `-1.0..=1.0`.
#[must_use]
pub fn map_rudder_track(x_from_left: f64, width: f64) -> f64 {
    if !(x_from_left.is_finite() && width.is_finite() && width > 0.0) {
        return 0.0;
    }
    let norm = ((x_from_left / width) * 2.0 - 1.0).clamp(-1.0, 1.0);
    shape_stick_axis(norm, RUDDER_DEADZONE, RUDDER_EXPO)
}

/// Computes shortest signed difference `a - b` wrapped into `[-180, 180]` degrees.
fn wrapped_deg_delta(a: f64, b: f64) -> f64 {
    if !(a.is_finite() && b.is_finite()) {
        return 0.0;
    }
    (a - b + 180.0).rem_euclid(360.0) - 180.0
}

/// Maps `DeviceOrientation` `(beta, gamma)` deltas from a neutral hold pose into
/// `[roll, pitch]` normalized stick deflection across portrait and landscape orientations.
#[must_use]
pub fn tilt_axes_for_orientation(
    beta_deg: f64,
    gamma_deg: f64,
    neutral_beta: f64,
    neutral_gamma: f64,
    screen_angle_deg: i32,
) -> [f64; 2] {
    if !(beta_deg.is_finite()
        && gamma_deg.is_finite()
        && neutral_beta.is_finite()
        && neutral_gamma.is_finite())
    {
        return [0.0, 0.0];
    }
    let d_beta = wrapped_deg_delta(beta_deg, neutral_beta);
    let d_gamma = wrapped_deg_delta(gamma_deg, neutral_gamma);
    let (raw_roll, raw_pitch) = match screen_angle_deg.rem_euclid(360) {
        90 => (d_beta / TILT_ROLL_FULL_DEG, -d_gamma / TILT_PITCH_FULL_DEG),
        180 => (-d_gamma / TILT_ROLL_FULL_DEG, -d_beta / TILT_PITCH_FULL_DEG),
        270 => (-d_beta / TILT_ROLL_FULL_DEG, d_gamma / TILT_PITCH_FULL_DEG),
        _ => (d_gamma / TILT_ROLL_FULL_DEG, d_beta / TILT_PITCH_FULL_DEG),
    };
    [
        shape_stick_axis(raw_roll, TILT_DEADZONE, TILT_EXPO),
        shape_stick_axis(raw_pitch, TILT_DEADZONE, TILT_EXPO),
    ]
}

/// Live touch cockpit + optional gyro-tilt state synchronized into [`InputMapper`].
#[derive(Debug, Clone, Default)]
pub struct TouchFlightState {
    /// Whether the on-screen virtual stick finger is currently down.
    pub stick_active: bool,
    /// Normalized `[roll, pitch]` axes from the virtual stick (`-1.0..=1.0`).
    pub stick_axes: [f64; 2],
    /// Whether optional gyro-tilt steering is enabled.
    pub tilt_enabled: bool,
    /// Normalized `[roll, pitch]` axes from `DeviceOrientation` (`-1.0..=1.0`).
    pub tilt_axes: [f64; 2],
    /// Whether the rudder slider finger is currently down.
    pub rudder_active: bool,
    /// Normalized yaw axis (`-1.0..=1.0`).
    pub rudder_axis: f64,
    /// Pending throttle slider write (`0.0..=1.0`), consumed when applied.
    pub throttle_value: Option<f64>,
    /// Whether the wheel-brake button is held down.
    pub brakes_held: bool,
    /// Discrete actions queued since the last frame.
    pub actions: Vec<TouchAction>,
    stick_was_active: bool,
    rudder_was_active: bool,
    brakes_applied: bool,
}

impl TouchFlightState {
    /// Stick axes shown on the virtual stick puck (finger overrides tilt while held).
    #[must_use]
    pub fn shown_stick_axes(&self) -> [f64; 2] {
        if self.stick_active {
            self.stick_axes
        } else if self.tilt_enabled {
            self.tilt_axes
        } else {
            [0.0, 0.0]
        }
    }

    /// Releases held flight controls (called when the Flight Hub opens).
    pub fn release_flight_inputs(&mut self) {
        self.stick_active = false;
        self.stick_axes = [0.0, 0.0];
        self.rudder_active = false;
        self.rudder_axis = 0.0;
        self.throttle_value = None;
        self.brakes_held = false;
    }

    /// Applies touch controls into `mapper`, updates `controls.live`, and returns queued actions.
    ///
    /// When `allow_flight` is `false` (for example while the Flight Hub is open), held stick/yaw
    /// inputs center cleanly without letting tilt or sliders move flight surfaces.
    pub fn apply_to(
        &mut self,
        mapper: &mut InputMapper,
        controls: &mut ControlScheme,
        flight: &mut Flight,
        allow_flight: bool,
    ) -> Vec<TouchAction> {
        let stick_driving = allow_flight && (self.stick_active || self.tilt_enabled);
        if stick_driving {
            let [roll, pitch] = self.shown_stick_axes();
            mapper.set(Control::Roll, roll);
            mapper.set(Control::Pitch, pitch);
            controls.note_touch();
            self.stick_was_active = true;
        } else if self.stick_was_active {
            mapper.set(Control::Roll, 0.0);
            mapper.set(Control::Pitch, 0.0);
            self.stick_was_active = false;
        }
        if allow_flight && self.rudder_active {
            mapper.set(Control::Yaw, self.rudder_axis);
            controls.note_touch();
            self.rudder_was_active = true;
        } else if self.rudder_was_active {
            mapper.set(Control::Yaw, 0.0);
            self.rudder_was_active = false;
        }
        if let Some(throttle) = self.throttle_value.take().filter(|_| allow_flight) {
            mapper.set(Control::Throttle, throttle.clamp(0.0, 1.0));
            controls.note_touch();
        }
        let target_brake = allow_flight && self.brakes_held;
        if target_brake != self.brakes_applied {
            flight.update_system_controls(|sys| {
                sys.wheel_brake_norm = if target_brake { 1.0 } else { 0.0 };
            });
            self.brakes_applied = target_brake;
            if target_brake {
                controls.note_touch();
            }
        }
        controls.sync_live(mapper.controls());
        std::mem::take(&mut self.actions)
    }
}

#[cfg(test)]
mod tests {
    use sim_input::{Control, InputMapper, ProfileLibrary};

    use super::{
        TouchFlightState, map_rudder_track, map_stick_pad, map_throttle_track, shape_stick_axis,
        tilt_axes_for_orientation,
    };
    use crate::controls::{ControlScheme, InputSource, VehicleClass};
    use crate::flight::Flight;

    #[test]
    fn shape_and_stick_pad_respect_deadzone_circular_clamp_and_signs() {
        assert_eq!(shape_stick_axis(0.04, 0.06, 0.35), 0.0);
        assert_eq!(shape_stick_axis(f64::NAN, 0.06, 0.35), 0.0);
        assert_eq!(map_stick_pad(3.0, 3.0, 100.0), [0.0, 0.0]);
        let [right_roll, up_pitch] = map_stick_pad(300.0, 300.0, 100.0);
        assert!((right_roll - up_pitch).abs() < 1e-6);
        assert!(right_roll > 0.4 && right_roll < 0.85, "{right_roll}");
        let [left_roll, nose_down] = map_stick_pad(-100.0, 0.0, 100.0);
        assert!((left_roll + 1.0).abs() < 1e-9);
        assert_eq!(nose_down, 0.0);
    }

    #[test]
    fn throttle_and_rudder_tracks_map_ranges_and_guard_bad_inputs() {
        assert_eq!(map_throttle_track(0.0, 150.0), 1.0);
        assert_eq!(map_throttle_track(150.0, 150.0), 0.0);
        assert_eq!(map_throttle_track(75.0, 150.0), 0.5);
        assert_eq!(map_throttle_track(f64::NAN, 150.0), 0.0);
        assert_eq!(map_rudder_track(64.0, 128.0), 0.0);
        assert_eq!(map_rudder_track(0.0, 128.0), -1.0);
        assert_eq!(map_rudder_track(128.0, 128.0), 1.0);
    }

    #[test]
    fn tilt_handles_portrait_landscape_and_angle_wrap() {
        assert_eq!(
            tilt_axes_for_orientation(45.0, 0.0, 45.0, 0.0, 0),
            [0.0, 0.0]
        );
        let [roll_p, pitch_p] = tilt_axes_for_orientation(70.0, 35.0, 45.0, 0.0, 0);
        assert!((roll_p - 1.0).abs() < 1e-9);
        assert!((pitch_p - 1.0).abs() < 1e-9);

        let [roll_l, pitch_l] = tilt_axes_for_orientation(80.0, -25.0, 45.0, 0.0, 90);
        assert!((roll_l - 1.0).abs() < 1e-9);
        assert!((pitch_l - 1.0).abs() < 1e-9);

        let [roll_r, pitch_r] = tilt_axes_for_orientation(10.0, 25.0, 45.0, 0.0, 270);
        assert!((roll_r - 1.0).abs() < 1e-9);
        assert!((pitch_r - 1.0).abs() < 1e-9);

        // Wrapping across +/-180 stays continuous.
        let [roll_w, _] = tilt_axes_for_orientation(45.0, -170.0, 45.0, 175.0, 0);
        assert!(roll_w > 0.1, "{roll_w}");
    }

    #[test]
    fn releasing_touch_centers_once_without_clobbering_later_inputs() {
        let mut mapper = InputMapper::new(ProfileLibrary::default());
        let mut controls = ControlScheme::new(VehicleClass::FixedWing);
        let mut flight = Flight::new().unwrap();
        let mut state = TouchFlightState {
            stick_active: true,
            stick_axes: [0.6, -0.4],
            rudder_active: true,
            rudder_axis: 0.5,
            throttle_value: Some(0.8),
            brakes_held: true,
            ..Default::default()
        };
        let _ = state.apply_to(&mut mapper, &mut controls, &mut flight, true);
        assert_eq!(controls.source(), InputSource::Touch);
        assert_eq!(mapper.controls().roll, 0.6);
        assert_eq!(mapper.controls().throttle, 0.8);

        state.release_flight_inputs();
        let _ = state.apply_to(&mut mapper, &mut controls, &mut flight, true);
        assert_eq!(mapper.controls().roll, 0.0);
        assert_eq!(mapper.controls().pitch, 0.0);
        assert_eq!(mapper.controls().yaw, 0.0);
        assert_eq!(mapper.controls().throttle, 0.8);

        // Simulate keyboard setting roll afterward; idle touch must leave it untouched.
        mapper.set(Control::Roll, 0.9);
        let _ = state.apply_to(&mut mapper, &mut controls, &mut flight, true);
        assert_eq!(mapper.controls().roll, 0.9);
    }
}

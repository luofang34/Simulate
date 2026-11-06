//! Pilot control presets, per-axis tuning, click-to-rebind, and the shaping layer between
//! raw gamepad / joystick events and the flight's [`ControlInputs`].
//!
//! `sim_input::InputMapper` stays the store of the four flight controls; this layer decides
//! what each raw axis writes into it, so presets and rebinding live in the app and a richer
//! `sim-input` binding model can replace [`ControlScheme`] without touching the menu.

use sim_core::ControlInputs;
use sim_input::{AxisShape, AxisSource, Control, GamepadAxis, InputRange};
use winit::keyboard::KeyCode;

mod keys;
mod labels;
mod preset;
#[cfg(test)]
mod tests;
mod tuning;

pub use keys::{KeyAction, KeyBindings, key_label};
use labels::set_control;
pub use labels::{control_index, control_title, control_value, source_label};
pub use preset::{ControlPreset, THROTTLE_RATE_PER_S, VehicleClass};

/// Raw deflection below which an axis reads as resting, so idle-stick noise never
/// overwrites a control the keyboard is holding.
const REST_EPS: f32 = 0.08;
/// Raw deflection that captures an axis during click-to-rebind.
const CAPTURE_THRESHOLD: f32 = 0.6;
const DEADZONE_STEP: f64 = 0.02;
/// Throttle step per throttle-key press (or key repeat) in the hub preview.
const PREVIEW_THROTTLE_STEP: f64 = 0.05;
const EXPO_STEP: f64 = 0.1;

/// The four flight controls in display order.
pub const CONTROLS: [Control; 4] = [
    Control::Roll,
    Control::Pitch,
    Control::Yaw,
    Control::Throttle,
];

/// How an axis contributes to its control.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BindKind {
    /// Writes the shaped value.
    Absolute,
    /// Moves the control at a rate proportional to the shaped deflection (latching throttle).
    Rate,
    /// Adds `sign ×` the shaped value to the other additive inputs of the control
    /// (left/right triggers on one rudder).
    Additive(f64),
}

/// One device axis bound to one control.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisBind {
    /// The device axis.
    pub source: AxisSource,
    /// The control it drives.
    pub control: Control,
    /// How it drives it.
    pub kind: BindKind,
    /// Invert, deadzone, expo, and ranges.
    pub shape: AxisShape,
}

/// A gamepad button a preset can bind.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PadButton {
    /// Left shoulder bumper (`LB`).
    LeftBumper,
    /// Right shoulder bumper (`RB`).
    RightBumper,
}

/// A button moving a control at a rate while held.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ButtonBind {
    /// The button.
    pub button: PadButton,
    /// The control it moves.
    pub control: Control,
    /// Signed fraction of [`THROTTLE_RATE_PER_S`] while held.
    pub rate: f64,
}

/// The device that last moved a control.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InputSource {
    /// Keyboard flight keys.
    #[default]
    Keyboard,
    /// A gamepad, joystick, or yoke.
    Gamepad,
}

/// What the next device input will be bound to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureTarget {
    /// The next strongly moved axis drives this control.
    Axis(Control),
    /// The next pressed key performs this action.
    Key(KeyAction),
}

/// One control's binding as the Controls tab shows it.
#[derive(Debug, Clone, PartialEq)]
pub struct AxisRow {
    /// The control.
    pub control: Control,
    /// Bound device inputs, e.g. `RS X` or `LT/RT`.
    pub sources: String,
    /// Whether the first bound axis is inverted.
    pub invert: bool,
    /// Its deadzone fraction.
    pub deadzone: f64,
    /// Its cubic expo blend.
    pub expo: f64,
}

/// Active preset, user tuning and rebinds, and live control telemetry.
#[derive(Debug, Clone)]
pub struct ControlScheme {
    class: VehicleClass,
    override_preset: Option<ControlPreset>,
    axes: Vec<AxisBind>,
    buttons: Vec<ButtonBind>,
    keys: KeyBindings,
    additive: Vec<(AxisSource, f64)>,
    rate_axis: f64,
    held_buttons: Vec<PadButton>,
    pad_active: [bool; 4],
    live: ControlInputs,
    source: InputSource,
    controller: Option<String>,
    capture: Option<CaptureTarget>,
    preview_keys: Vec<KeyAction>,
    camera_look_axes: [f64; 2],
    look_reset: bool,
}

impl ControlScheme {
    /// The automatic preset for `class`.
    #[must_use]
    pub fn new(class: VehicleClass) -> Self {
        let preset = ControlPreset::default_for(class);
        Self {
            class,
            override_preset: None,
            axes: preset.axis_binds(),
            buttons: preset.button_binds(),
            keys: KeyBindings::default(),
            additive: Vec::new(),
            rate_axis: 0.0,
            held_buttons: Vec::new(),
            pad_active: [false; 4],
            live: ControlInputs::default(),
            source: InputSource::Keyboard,
            controller: None,
            capture: None,
            preview_keys: Vec::new(),
            camera_look_axes: [0.0; 2],
            look_reset: false,
        }
    }

    /// The preset in force: the user's choice, else the vehicle class default.
    #[must_use]
    pub fn preset(&self) -> ControlPreset {
        self.override_preset
            .unwrap_or_else(|| ControlPreset::default_for(self.class))
    }

    /// Whether the preset follows the vehicle class.
    #[must_use]
    pub const fn is_automatic(&self) -> bool {
        self.override_preset.is_none()
    }

    /// Follows a new vehicle's class; an automatic preset switches with it.
    pub fn set_vehicle_class(&mut self, class: VehicleClass) {
        let before = self.preset();
        self.class = class;
        if self.preset() != before {
            self.load(self.preset());
        }
    }

    /// Chooses the neighbouring preset, overriding the automatic default.
    pub fn step_preset(&mut self, forward: bool) -> ControlPreset {
        let next = self.preset().step(forward);
        self.override_preset = Some(next);
        self.load(next);
        next
    }

    /// Returns to the vehicle class default.
    pub fn reset_automatic(&mut self) {
        self.override_preset = None;
        self.load(self.preset());
    }

    fn load(&mut self, preset: ControlPreset) {
        self.axes = preset.axis_binds();
        self.buttons = preset.button_binds();
        self.additive.clear();
        self.rate_axis = 0.0;
        self.pad_active = [false; 4];
        self.camera_look_axes = [0.0; 2];
    }

    /// Right-stick camera look `[pan right, tilt up]` in `-1..=1`, held at zero unless
    /// the preset leaves the right stick free of flight controls.
    #[must_use]
    pub const fn camera_look_axes(&self) -> [f64; 2] {
        self.camera_look_axes
    }

    /// Requests a camera look reset (e.g. right-stick click).
    pub fn request_look_reset(&mut self) {
        self.look_reset = true;
    }

    /// Consumes a pending camera look reset request.
    pub fn take_look_reset(&mut self) -> bool {
        std::mem::take(&mut self.look_reset)
    }

    /// Records `raw` as camera look when `source` is a right-stick axis no flight
    /// control is bound to under a preset that frees the right stick.
    fn track_camera_look(&mut self, source: AxisSource, raw: f32) {
        let slot = match source {
            AxisSource::Gamepad(GamepadAxis::RightStickX) => 0,
            AxisSource::Gamepad(GamepadAxis::RightStickY) => 1,
            _ => return,
        };
        let free =
            self.preset().frees_right_stick() && self.axes.iter().all(|b| b.source != source);
        if let Some(axis) = self.camera_look_axes.get_mut(slot) {
            *axis = if free {
                AxisShape::STICK.apply(raw)
            } else {
                0.0
            };
        }
    }

    /// Rebindable flight keys.
    #[must_use]
    pub const fn keys(&self) -> &KeyBindings {
        &self.keys
    }

    /// Controls as last commanded by any device, for the live telemetry bars.
    #[must_use]
    pub const fn live(&self) -> ControlInputs {
        self.live
    }

    /// Records the controls the flight actually received this frame.
    pub fn sync_live(&mut self, controls: ControlInputs) {
        self.live = controls;
    }

    /// Moves the live telemetry (not the flight) as a held flight key would, so keys can be
    /// tried out while the hub holds the simulation; throttle keys step it per press.
    pub fn preview_key(&mut self, action: KeyAction, pressed: bool) {
        self.preview_keys.retain(|a| *a != action);
        if pressed {
            self.preview_keys.push(action);
            self.source = InputSource::Keyboard;
        }
        let held = |a: KeyAction| f64::from(u8::from(self.preview_keys.contains(&a)));
        self.live.roll = held(KeyAction::RollRight) - held(KeyAction::RollLeft);
        self.live.pitch = held(KeyAction::PitchUp) - held(KeyAction::PitchDown);
        self.live.yaw = held(KeyAction::YawRight) - held(KeyAction::YawLeft);
        let step = match (action, pressed) {
            (KeyAction::ThrottleUp, true) => PREVIEW_THROTTLE_STEP,
            (KeyAction::ThrottleDown, true) => -PREVIEW_THROTTLE_STEP,
            _ => 0.0,
        };
        self.live.throttle = (self.live.throttle + step).clamp(0.0, 1.0);
    }

    /// Forgets keys held for the preview (the hub closed).
    pub fn clear_preview(&mut self) {
        self.preview_keys.clear();
    }

    /// Device that last moved a control.
    #[must_use]
    pub const fn source(&self) -> InputSource {
        self.source
    }

    /// Marks the keyboard as the active input.
    pub fn note_keyboard(&mut self) {
        self.source = InputSource::Keyboard;
    }

    /// Name of the connected controller, if any.
    #[must_use]
    pub fn controller(&self) -> Option<&str> {
        self.controller.as_deref()
    }

    /// Records the connected controller's name (`None` when it disconnects).
    pub fn set_controller(&mut self, name: Option<String>) {
        self.controller = name;
    }

    /// Pending click-to-rebind target.
    #[must_use]
    pub const fn capture(&self) -> Option<CaptureTarget> {
        self.capture
    }

    /// Waits for the next axis movement or key press to rebind `target`; the same target
    /// again cancels.
    pub fn begin_capture(&mut self, target: CaptureTarget) {
        self.capture = if self.capture == Some(target) {
            None
        } else {
            Some(target)
        };
    }

    /// Abandons a pending rebind.
    pub fn cancel_capture(&mut self) {
        self.capture = None;
    }

    /// Completes a pending key rebind with `code`; returns whether it was consumed.
    pub fn capture_key(&mut self, code: KeyCode) -> bool {
        let Some(CaptureTarget::Key(action)) = self.capture else {
            return false;
        };
        if code != KeyCode::Escape {
            self.keys.bind(action, code);
        }
        self.capture = None;
        true
    }

    /// Feeds one raw axis value; returns the control writes for the flight.
    pub fn handle_axis(&mut self, source: AxisSource, raw: f32) -> Vec<(Control, f64)> {
        let raw = if raw.is_finite() { raw } else { 0.0 };
        if let Some(CaptureTarget::Axis(control)) = self.capture {
            if raw.abs() >= CAPTURE_THRESHOLD {
                self.rebind_axis(control, source);
                self.capture = None;
            }
            return Vec::new();
        }
        self.track_camera_look(source, raw);
        let binds: Vec<AxisBind> = self
            .axes
            .iter()
            .filter(|b| b.source == source)
            .copied()
            .collect();
        let mut writes = Vec::new();
        for bind in binds {
            if let Some(value) = self.apply_bind(&bind, raw) {
                writes.push((bind.control, value));
            }
        }
        for &(control, value) in &writes {
            set_control(&mut self.live, control, value);
            self.source = InputSource::Gamepad;
        }
        writes
    }

    fn apply_bind(&mut self, bind: &AxisBind, raw: f32) -> Option<f64> {
        let shaped = bind.shape.apply(raw);
        let resting = match bind.shape.input {
            InputRange::Bipolar => raw.abs() < REST_EPS,
            InputRange::Unipolar => raw < REST_EPS,
        };
        let value = match bind.kind {
            BindKind::Absolute => shaped,
            BindKind::Rate => {
                self.rate_axis = if resting { 0.0 } else { shaped };
                return None;
            }
            BindKind::Additive(sign) => {
                upsert(&mut self.additive, bind.source, sign * shaped);
                let all_rest = self.additive.iter().all(|(_, v)| v.abs() < 0.05);
                return self.gate(bind.control, all_rest, self.additive_sum());
            }
        };
        self.gate(bind.control, resting, value)
    }

    /// Writes when the stick is deflected, or once as it returns to rest.
    fn gate(&mut self, control: Control, resting: bool, value: f64) -> Option<f64> {
        let slot = self.pad_active.get_mut(control_index(control))?;
        if resting && !*slot {
            return None;
        }
        *slot = !resting;
        Some(value)
    }

    fn additive_sum(&self) -> f64 {
        self.additive
            .iter()
            .map(|(_, v)| v)
            .sum::<f64>()
            .clamp(-1.0, 1.0)
    }

    /// Records a bumper press or release.
    pub fn handle_button(&mut self, button: PadButton, pressed: bool) {
        self.held_buttons.retain(|b| *b != button);
        if pressed {
            self.held_buttons.push(button);
            self.source = InputSource::Gamepad;
        }
    }

    /// The latched throttle after `elapsed_s` of rate inputs from `throttle`, when any
    /// rate input is held.
    #[must_use]
    pub fn integrate_throttle(&self, elapsed_s: f64, throttle: f64) -> Option<f64> {
        let buttons: f64 = self
            .buttons
            .iter()
            .filter(|b| b.control == Control::Throttle && self.held_buttons.contains(&b.button))
            .map(|b| b.rate)
            .sum();
        let rate = (self.rate_axis + buttons).clamp(-1.0, 1.0);
        (rate != 0.0 && elapsed_s.is_finite())
            .then(|| (throttle + rate * THROTTLE_RATE_PER_S * elapsed_s).clamp(0.0, 1.0))
    }

    fn rebind_axis(&mut self, control: Control, source: AxisSource) {
        let raw_device = matches!(source, AxisSource::Raw(_));
        let template = self
            .axes
            .iter()
            .find(|b| b.control == control && matches!(b.source, AxisSource::Raw(_)) == raw_device)
            .copied();
        self.axes.retain(|b| {
            b.source != source
                && !(b.control == control && matches!(b.source, AxisSource::Raw(_)) == raw_device)
        });
        let (kind, shape) = match template {
            Some(AxisBind {
                kind: BindKind::Additive(_),
                shape,
                ..
            }) => (
                BindKind::Absolute,
                AxisShape {
                    input: InputRange::Bipolar,
                    ..shape
                },
            ),
            Some(bind) => (bind.kind, bind.shape),
            None if control == Control::Throttle => (BindKind::Absolute, AxisShape::LEVER),
            None => (BindKind::Absolute, AxisShape::STICK),
        };
        self.axes.push(AxisBind {
            source,
            control,
            kind,
            shape,
        });
        self.additive.retain(|(s, _)| *s != source);
        self.camera_look_axes = [0.0; 2];
    }
}

fn upsert(entries: &mut Vec<(AxisSource, f64)>, source: AxisSource, value: f64) {
    match entries.iter_mut().find(|(s, _)| *s == source) {
        Some(entry) => entry.1 = value,
        None => entries.push((source, value)),
    }
}

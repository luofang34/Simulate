use std::collections::BTreeMap;

use sim_core::ControlInputs;

use crate::{
    AxisSource, Control, ControlPresetId, DeviceId, FlightKeyAction, InputTarget,
    KeyboardFlightAxes, KeyboardRates, ProfileLibrary, RawInput, VehicleControlClass,
};

mod arbitration;
mod capture;
mod device;
mod keyboard_input;
mod monitor;
mod persist;
mod presets;
mod table;
mod tuning;

pub use arbitration::ControlOwner;
pub use capture::{BindingCapture, CapturedBinding};
pub use monitor::{AxisReading, ControlMonitorSnapshot, DeviceMonitor, TargetReading};
pub use tuning::ControlTuning;

pub(crate) use arbitration::PerControl;
pub(crate) use table::PresetTable;

use capture::PendingCapture;
use device::DeviceState;

#[cfg(test)]
mod tests;

/// Below this, a shaped value or a change in it is noise, not a deflection.
const EPSILON: f64 = 1e-5;

/// Folds raw events from any number of devices and the keyboard into one set
/// of controls.
///
/// Each source keeps its own state, and a control follows whichever source
/// last moved it meaningfully: a device takes a control only when its shaped
/// output changes or sits off centre, so a gamepad resting inside its
/// deadzone never overrides held keys, and a gamepad and a yoke can both stay
/// plugged in.
#[derive(Debug, Clone)]
pub struct InputMapper {
    library: ProfileLibrary,
    devices: BTreeMap<DeviceId, DeviceState>,
    presets: PresetTable,
    vehicle_class: VehicleControlClass,
    preset: ControlPresetId,
    pinned: BTreeMap<VehicleControlClass, ControlPresetId>,
    tuning: PerControl<ControlTuning>,
    keyboard: KeyboardFlightAxes,
    rates: KeyboardRates,
    owners: PerControl<ControlOwner>,
    latched_throttle: f64,
    trim_throttle: f64,
    camera_pan: (f64, Option<DeviceId>),
    camera_tilt: (f64, Option<DeviceId>),
    capture: Option<PendingCapture>,
}

impl InputMapper {
    /// A fixed-wing mapper whose USB-specific profiles come from `library`;
    /// other devices use the [`VehicleControlClass::FixedWing`] presets.
    pub fn new(library: ProfileLibrary) -> Self {
        let vehicle_class = VehicleControlClass::FixedWing;
        Self {
            library,
            devices: BTreeMap::new(),
            presets: PresetTable::builtin(),
            vehicle_class,
            preset: vehicle_class.default_preset(),
            pinned: BTreeMap::new(),
            tuning: PerControl::default(),
            keyboard: KeyboardFlightAxes::default(),
            rates: KeyboardRates::default(),
            owners: PerControl::default(),
            latched_throttle: 0.0,
            trim_throttle: 0.0,
            camera_pan: (0.0, None),
            camera_tilt: (0.0, None),
            capture: None,
        }
    }

    /// Applies one event; a completed axis capture stays readable through
    /// [`Self::preset_table`].
    pub fn handle(&mut self, event: &RawInput) {
        self.handle_event(event);
    }

    /// Applies one event, returning the binding it made if an axis capture
    /// was pending and this event completed it.
    pub fn handle_event(&mut self, event: &RawInput) -> Option<CapturedBinding> {
        match event {
            RawInput::Connected(info) => {
                self.connect(info);
                None
            }
            RawInput::Disconnected(id) => {
                self.disconnect(*id);
                None
            }
            RawInput::Axis {
                device,
                source,
                value,
            } => self.on_axis(*device, *source, *value),
        }
    }

    /// Sets one control directly, for keyboard or on-screen input. A stick
    /// value off centre takes the control from any device; centring it only
    /// recentres the keyboard's own stick. Throttle always moves the lever.
    pub fn set(&mut self, control: Control, value: f64) {
        let value = if value.is_nan() { 0.0 } else { value };
        match control {
            Control::Throttle => {
                let lever = value.clamp(0.0, 1.0);
                self.latched_throttle = lever;
                self.keyboard.set_axis(Control::Throttle, lever * 2.0 - 1.0);
                *self.owners.get_mut(control) = ControlOwner::Keyboard;
            }
            Control::Roll | Control::Pitch | Control::Yaw => {
                self.keyboard.set_axis(control, value);
                if value.abs() > EPSILON {
                    *self.owners.get_mut(control) = ControlOwner::Keyboard;
                }
            }
        }
    }

    /// Current control positions.
    pub fn controls(&self) -> ControlInputs {
        ControlInputs {
            roll: self.stick(Control::Roll),
            pitch: self.stick(Control::Pitch),
            yaw: self.stick(Control::Yaw),
            throttle: self.throttle_output(),
        }
        .clamped()
    }

    /// Camera look, `[pan, tilt]`, each −1..1.
    pub fn camera_look(&self) -> [f64; 2] {
        [self.camera_pan.0, self.camera_tilt.0]
    }

    /// Whether `device` is connected and bound.
    pub fn is_bound(&self, device: DeviceId) -> bool {
        self.devices.contains_key(&device)
    }

    /// Whether a key or button holds [`FlightKeyAction::WheelBrake`].
    pub fn wheel_brake_held(&self) -> bool {
        self.keyboard.is_held(FlightKeyAction::WheelBrake)
    }

    /// The source currently driving `control`.
    pub fn owner(&self, control: Control) -> ControlOwner {
        *self.owners.get(control)
    }

    /// The latched throttle lever, 0..1, that throttle keys, buttons, and
    /// rate sticks move.
    pub fn latched_throttle(&self) -> f64 {
        self.latched_throttle
    }

    /// Sets the throttle trim, such as cruise power when spawning airborne or
    /// idle on the ramp, and moves the latched lever there.
    pub fn set_trim_throttle(&mut self, trim: f64) {
        let trim = if trim.is_finite() {
            trim.clamp(0.0, 1.0)
        } else {
            0.0
        };
        self.trim_throttle = trim;
        self.latched_throttle = trim;
    }

    /// The throttle trim [`FlightKeyAction::ResetTrim`] returns to.
    pub fn trim_throttle(&self) -> f64 {
        self.trim_throttle
    }

    /// The keyboard slew rates.
    pub fn keyboard_rates(&self) -> KeyboardRates {
        self.rates
    }

    /// Sets the keyboard slew rates; invalid values fall back to the defaults.
    pub fn set_keyboard_rates(&mut self, rates: KeyboardRates) {
        self.rates = rates.sanitized();
    }

    fn device_output(&self, device: DeviceId, target: InputTarget) -> f64 {
        self.devices
            .get(&device)
            .and_then(|d| d.last_shaped.get(&target).copied())
            .unwrap_or(0.0)
    }

    fn last_raw(&self, device: DeviceId, source: AxisSource) -> Option<f64> {
        self.devices
            .get(&device)
            .and_then(|d| d.last_raw.get(&source).copied())
    }
}

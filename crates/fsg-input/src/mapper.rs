use std::collections::BTreeMap;

use fsg_sim::ControlInputs;

use crate::{AxisBinding, Control, DeviceId, ProfileLibrary, RawInput};

#[cfg(test)]
mod tests;

/// Folds raw events from any number of devices into one set of controls.
/// When two devices drive the same control, the one that moved last wins, so
/// a gamepad and a yoke can both stay plugged in.
#[derive(Debug, Clone)]
pub struct InputMapper {
    library: ProfileLibrary,
    bindings: BTreeMap<DeviceId, Vec<AxisBinding>>,
    controls: ControlInputs,
}

impl InputMapper {
    /// A mapper selecting profiles from `library`.
    pub fn new(library: ProfileLibrary) -> Self {
        Self {
            library,
            bindings: BTreeMap::new(),
            controls: ControlInputs::default(),
        }
    }

    /// Applies one event.
    pub fn handle(&mut self, event: &RawInput) {
        match event {
            RawInput::Connected(info) => {
                let profile = self.library.select(info);
                self.bindings.insert(info.id, profile.bindings.clone());
            }
            RawInput::Disconnected(id) => {
                self.bindings.remove(id);
            }
            RawInput::Axis {
                device,
                source,
                value,
            } => {
                let Some(bindings) = self.bindings.get(device) else {
                    return;
                };
                for binding in bindings.iter().filter(|b| b.source == *source) {
                    let shaped = binding.shape.apply(*value);
                    match binding.control {
                        Control::Roll => self.controls.roll = shaped,
                        Control::Pitch => self.controls.pitch = shaped,
                        Control::Yaw => self.controls.yaw = shaped,
                        Control::Throttle => self.controls.throttle = shaped,
                    }
                }
            }
        }
    }

    /// Sets one control directly, for keyboard or on-screen input.
    pub fn set(&mut self, control: Control, value: f64) {
        match control {
            Control::Roll => self.controls.roll = value,
            Control::Pitch => self.controls.pitch = value,
            Control::Yaw => self.controls.yaw = value,
            Control::Throttle => self.controls.throttle = value,
        }
        self.controls = self.controls.clamped();
    }

    /// Current control positions.
    pub fn controls(&self) -> ControlInputs {
        self.controls.clamped()
    }

    /// Whether `device` is connected and bound.
    pub fn is_bound(&self, device: DeviceId) -> bool {
        self.bindings.contains_key(&device)
    }
}

use super::InputMapper;
use crate::{ControlPreset, ControlPresetId, ThrottleBehavior, VehicleControlClass};

impl InputMapper {
    /// The vehicle class being flown.
    pub fn vehicle_class(&self) -> VehicleControlClass {
        self.vehicle_class
    }

    /// Switches vehicle class and selects its pinned preset, or the class
    /// default when none is pinned.
    pub fn set_vehicle_class(&mut self, class: VehicleControlClass) {
        self.vehicle_class = class;
        let preset = self
            .pinned
            .get(&class)
            .copied()
            .unwrap_or_else(|| class.default_preset());
        self.switch_preset(preset);
    }

    /// The preset standard-layout gamepads and the keyboard follow.
    pub fn preset(&self) -> ControlPresetId {
        self.preset
    }

    /// Switches preset now without pinning it; the next vehicle-class change
    /// picks that class's preset again.
    pub fn set_preset(&mut self, preset: ControlPresetId) {
        self.switch_preset(preset);
    }

    /// Pins `preset` for `class` (or unpins with `None`), switching now if
    /// `class` is being flown.
    pub fn pin_preset(&mut self, class: VehicleControlClass, preset: Option<ControlPresetId>) {
        match preset {
            Some(id) => self.pinned.insert(class, id),
            None => self.pinned.remove(&class),
        };
        if class == self.vehicle_class {
            self.set_vehicle_class(class);
        }
    }

    /// The preset pinned for `class`, if any.
    pub fn pinned_preset(&self, class: VehicleControlClass) -> Option<ControlPresetId> {
        self.pinned.get(&class).copied()
    }

    /// The current, possibly rebound, contents of `id`.
    pub fn preset_table(&self, id: ControlPresetId) -> &ControlPreset {
        self.presets.get(id)
    }

    /// Restores `id` to its built-in bindings.
    pub fn reset_preset(&mut self, id: ControlPresetId) {
        self.presets.replace(id.builtin());
        self.reset_device_outputs();
    }

    /// The active preset's throttle behaviour, which the keyboard follows.
    pub fn throttle_behavior(&self) -> ThrottleBehavior {
        self.presets.get(self.preset).throttle
    }

    fn switch_preset(&mut self, preset: ControlPresetId) {
        if preset != self.preset {
            self.preset = preset;
            self.reset_device_outputs();
        }
    }
}

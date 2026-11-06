use super::InputMapper;
use crate::{Control, ControlConfig, ControlTuningEntry, PinnedPreset};

const CONTROLS: [Control; 4] = [
    Control::Roll,
    Control::Pitch,
    Control::Yaw,
    Control::Throttle,
];

impl InputMapper {
    /// Everything the user has customised.
    pub fn config(&self) -> ControlConfig {
        ControlConfig {
            vehicle_class: self.vehicle_class,
            preset: self.preset,
            pinned: self
                .pinned
                .iter()
                .map(|(&class, &preset)| PinnedPreset { class, preset })
                .collect(),
            presets: self.presets.all(),
            tuning: CONTROLS
                .iter()
                .map(|&control| ControlTuningEntry {
                    control,
                    tuning: *self.tuning.get(control),
                })
                .collect(),
            keyboard: self.rates,
            trim_throttle: self.trim_throttle,
        }
    }

    /// Applies a saved config; presets it omits keep their current bindings.
    pub fn apply_config(&mut self, config: &ControlConfig) {
        for preset in &config.presets {
            self.presets.replace(preset.clone());
        }
        self.pinned = config.pinned.iter().map(|p| (p.class, p.preset)).collect();
        for entry in &config.tuning {
            *self.tuning.get_mut(entry.control) = entry.tuning;
        }
        self.set_keyboard_rates(config.keyboard);
        self.set_trim_throttle(config.trim_throttle);
        self.vehicle_class = config.vehicle_class;
        self.preset = config.preset;
        self.reset_device_outputs();
    }
}

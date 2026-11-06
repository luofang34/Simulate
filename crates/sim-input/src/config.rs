use serde::{Deserialize, Serialize};

use crate::{
    AxisShape, Control, ControlPreset, ControlPresetId, ControlTuning, KeyboardRates,
    ThrottleBehavior, VehicleControlClass,
};

#[cfg(test)]
mod tests;

/// A failure saving or loading a [`ControlConfig`].
#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    /// The configuration could not be written as JSON.
    #[error("cannot serialise control config: {source}")]
    Serialize {
        /// The JSON error.
        #[source]
        source: serde_json::Error,
    },
    /// The text is not a valid control config.
    #[error("cannot parse control config at line {line}, column {column}: {source}", line = source.line(), column = source.column())]
    Parse {
        /// The JSON error.
        #[source]
        source: serde_json::Error,
    },
    /// A number in the config is NaN or infinite.
    #[error("control config field {field} is not finite: {value}")]
    NonFinite {
        /// Which field.
        field: String,
        /// Its value.
        value: f64,
    },
}

/// A preset pinned to a vehicle class.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PinnedPreset {
    /// The vehicle class.
    pub class: VehicleControlClass,
    /// The preset it always uses.
    pub preset: ControlPresetId,
}

/// Tuning for one control.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ControlTuningEntry {
    /// The control.
    pub control: Control,
    /// Its adjustments.
    pub tuning: ControlTuning,
}

/// Everything a user can customise, for saving and loading.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlConfig {
    /// Vehicle class being flown.
    pub vehicle_class: VehicleControlClass,
    /// Active preset.
    pub preset: ControlPresetId,
    /// Per-class preset pins.
    pub pinned: Vec<PinnedPreset>,
    /// Every preset's current bindings.
    pub presets: Vec<ControlPreset>,
    /// Per-control tuning.
    pub tuning: Vec<ControlTuningEntry>,
    /// Keyboard slew rates.
    pub keyboard: KeyboardRates,
    /// Throttle trim.
    pub trim_throttle: f64,
}

impl ControlConfig {
    /// The config as indented JSON.
    pub fn to_json_pretty(&self) -> Result<String, ConfigError> {
        serde_json::to_string_pretty(self).map_err(|source| ConfigError::Serialize { source })
    }

    /// Parses and validates a config written by [`Self::to_json_pretty`].
    pub fn from_json_str(text: &str) -> Result<Self, ConfigError> {
        let config: Self =
            serde_json::from_str(text).map_err(|source| ConfigError::Parse { source })?;
        config.validate()?;
        Ok(config)
    }

    /// Checks every number is finite.
    pub fn validate(&self) -> Result<(), ConfigError> {
        finite("trim_throttle", self.trim_throttle)?;
        finite("keyboard.slew_rate_per_s", self.keyboard.slew_rate_per_s)?;
        let rate = self.keyboard.center_return_rate_per_s;
        finite("keyboard.center_return_rate_per_s", rate)?;
        finite(
            "keyboard.throttle_rate_per_s",
            self.keyboard.throttle_rate_per_s,
        )?;
        for entry in &self.tuning {
            let name = format!("{:?}", entry.control);
            if let Some(d) = entry.tuning.deadzone {
                finite(&format!("tuning.{name}.deadzone"), d)?;
            }
            if let Some(e) = entry.tuning.expo {
                finite(&format!("tuning.{name}.expo"), e)?;
            }
        }
        self.presets.iter().try_for_each(validate_preset)
    }
}

fn validate_preset(preset: &ControlPreset) -> Result<(), ConfigError> {
    let name = format!("{:?}", preset.id);
    if let ThrottleBehavior::LatchingIncremental { rate_per_s } = preset.throttle {
        finite(&format!("presets.{name}.throttle.rate_per_s"), rate_per_s)?;
    }
    preset.bindings.iter().try_for_each(|b| {
        let AxisShape { deadzone, expo, .. } = b.shape;
        finite(&format!("presets.{name}.bindings.deadzone"), deadzone)?;
        finite(&format!("presets.{name}.bindings.expo"), expo)
    })
}

fn finite(field: &str, value: f64) -> Result<(), ConfigError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(ConfigError::NonFinite {
            field: field.to_owned(),
            value,
        })
    }
}

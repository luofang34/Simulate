use serde::{Deserialize, Serialize};

use super::InputMapper;
use crate::{AxisShape, Control, InputTarget, MappedBinding};

/// Largest deadzone a user can set; beyond it a stick would barely move.
const MAX_DEADZONE: f64 = 0.95;

/// User adjustments applied on top of every binding that drives a control.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct ControlTuning {
    /// Reverse the control relative to the preset's convention.
    pub inverted: bool,
    /// Deadzone replacing the binding's, for centred axes.
    pub deadzone: Option<f64>,
    /// Expo replacing the binding's.
    pub expo: Option<f64>,
}

impl ControlTuning {
    /// `shape` with these adjustments applied.
    pub fn apply(&self, shape: AxisShape) -> AxisShape {
        let mut tuned = if self.inverted {
            shape.inverted()
        } else {
            shape
        };
        if let Some(deadzone) = self.deadzone.filter(|d| d.is_finite()) {
            tuned.deadzone = deadzone.clamp(0.0, MAX_DEADZONE);
        }
        if let Some(expo) = self.expo.filter(|e| e.is_finite()) {
            tuned.expo = expo.clamp(0.0, 1.0);
        }
        tuned
    }
}

impl InputMapper {
    /// Reverses `control` relative to the preset when `inverted`.
    pub fn set_control_inverted(&mut self, control: Control, inverted: bool) {
        self.tuning.get_mut(control).inverted = inverted;
    }

    /// Overrides the deadzone of every centred axis driving `control`,
    /// clamped to 0..0.95; NaN is ignored.
    pub fn set_control_deadzone(&mut self, control: Control, deadzone: f64) {
        if deadzone.is_finite() {
            self.tuning.get_mut(control).deadzone = Some(deadzone.clamp(0.0, MAX_DEADZONE));
        }
    }

    /// Overrides the expo of every axis driving `control`, clamped to 0..1;
    /// NaN is ignored.
    pub fn set_control_expo(&mut self, control: Control, expo: f64) {
        if expo.is_finite() {
            self.tuning.get_mut(control).expo = Some(expo.clamp(0.0, 1.0));
        }
    }

    /// The adjustments applied to `control`.
    pub fn control_tuning(&self, control: Control) -> ControlTuning {
        *self.tuning.get(control)
    }

    /// Replaces every adjustment to `control`.
    pub fn set_control_tuning(&mut self, control: Control, tuning: ControlTuning) {
        *self.tuning.get_mut(control) = tuning;
    }

    pub(super) fn tuned_shape(&self, binding: &MappedBinding) -> AxisShape {
        match binding.target {
            InputTarget::Flight(control) => self.tuning.get(control).apply(binding.shape),
            InputTarget::CameraPan | InputTarget::CameraTilt => binding.shape,
        }
    }
}

use serde::{Deserialize, Serialize};

use crate::{AxisBinding, AxisShape, AxisSource, Control, FlightKeyAction, GamepadButton};

/// Anything an axis can drive: a pilot control or a camera look axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum InputTarget {
    /// A pilot control in [`sim_core::ControlInputs`].
    Flight(Control),
    /// Cockpit or chase camera pan, −1 (left) to 1 (right).
    CameraPan,
    /// Cockpit or chase camera tilt, −1 (down) to 1 (up).
    CameraTilt,
}

impl From<Control> for InputTarget {
    fn from(control: Control) -> Self {
        Self::Flight(control)
    }
}

/// The physical input behind a binding.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BindingSource {
    /// One axis.
    Single(AxisSource),
    /// Two unipolar axes read as `positive − negative`, such as
    /// `RightTrigger − LeftTrigger` for rudder. Each side is clamped to 0..1,
    /// so the pair spans −1..1 and is shaped as a bipolar axis.
    Differential {
        /// Axis that pushes the target positive.
        positive: AxisSource,
        /// Axis that pushes the target negative.
        negative: AxisSource,
    },
}

impl BindingSource {
    /// Whether an event on `source` changes this binding's value.
    pub fn uses(&self, source: AxisSource) -> bool {
        match *self {
            Self::Single(s) => s == source,
            Self::Differential { positive, negative } => positive == source || negative == source,
        }
    }
}

/// One physical input bound to one target, as used by presets.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MappedBinding {
    /// The device input.
    pub source: BindingSource,
    /// What it drives.
    pub target: InputTarget,
    /// How its value is shaped.
    pub shape: AxisShape,
}

impl MappedBinding {
    /// A single-axis binding.
    pub fn axis(source: AxisSource, target: impl Into<InputTarget>, shape: AxisShape) -> Self {
        Self {
            source: BindingSource::Single(source),
            target: target.into(),
            shape,
        }
    }

    /// A `positive − negative` differential binding.
    pub fn differential(
        positive: AxisSource,
        negative: AxisSource,
        target: impl Into<InputTarget>,
        shape: AxisShape,
    ) -> Self {
        Self {
            source: BindingSource::Differential { positive, negative },
            target: target.into(),
            shape,
        }
    }
}

impl From<AxisBinding> for MappedBinding {
    fn from(binding: AxisBinding) -> Self {
        Self::axis(binding.source, binding.control, binding.shape)
    }
}

/// A gamepad button that holds a flight action while pressed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ButtonBinding {
    /// The button.
    pub button: GamepadButton,
    /// The action it holds.
    pub action: FlightKeyAction,
}

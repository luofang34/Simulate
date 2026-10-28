#[cfg(test)]
mod tests;

/// Range a physical axis reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputRange {
    /// −1..1 (sticks, yoke axes, most lever axes).
    Bipolar,
    /// 0..1 (gamepad triggers).
    Unipolar,
}

/// What the axis controls physically.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputRange {
    /// A self-centring control producing −1..1, with a deadzone at centre.
    Centered,
    /// A lever that stays where it is set, producing 0..1. No centre deadzone:
    /// mid-travel is a real setting, not noise around rest.
    Lever,
}

/// How a raw axis value becomes a control value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisShape {
    /// Reverse the axis direction.
    pub invert: bool,
    /// Fraction of travel around rest that reads as zero (centred axes only).
    pub deadzone: f64,
    /// Cubic expo blend, 0 (linear) to 1 (fully cubic), for finer centre feel.
    pub expo: f64,
    /// Range the device reports.
    pub input: InputRange,
    /// Range the control expects.
    pub output: OutputRange,
}

impl AxisShape {
    /// A centred stick axis with a small deadzone and mild expo.
    pub const STICK: Self = Self {
        invert: false,
        deadzone: 0.05,
        expo: 0.3,
        input: InputRange::Bipolar,
        output: OutputRange::Centered,
    };

    /// A lever such as a throttle on a bipolar axis.
    pub const LEVER: Self = Self {
        invert: false,
        deadzone: 0.0,
        expo: 0.0,
        input: InputRange::Bipolar,
        output: OutputRange::Lever,
    };

    /// This shape with the direction reversed.
    #[must_use]
    pub const fn inverted(mut self) -> Self {
        self.invert = !self.invert;
        self
    }

    /// Shapes a raw device value; NaN reads as rest.
    pub fn apply(&self, raw: f32) -> f64 {
        let raw = if raw.is_nan() { 0.0 } else { f64::from(raw) };
        let unit = match self.input {
            InputRange::Bipolar => raw.clamp(-1.0, 1.0),
            InputRange::Unipolar => raw.clamp(0.0, 1.0) * 2.0 - 1.0,
        };
        let unit = if self.invert { -unit } else { unit };
        match self.output {
            OutputRange::Centered => {
                let magnitude = ((unit.abs() - self.deadzone) / (1.0 - self.deadzone).max(1e-6))
                    .clamp(0.0, 1.0);
                let curved = magnitude * (1.0 - self.expo) + magnitude.powi(3) * self.expo;
                curved.copysign(unit)
            }
            OutputRange::Lever => (unit + 1.0) / 2.0,
        }
    }
}

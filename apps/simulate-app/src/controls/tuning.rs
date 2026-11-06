//! Per-control invert / deadzone / expo tuning and the bindings table the hub shows.

use sim_input::{AxisShape, OutputRange};

use super::labels::button_label;
use super::{
    AxisBind, AxisRow, BindKind, CONTROLS, ControlScheme, DEADZONE_STEP, EXPO_STEP, source_label,
};
use sim_input::Control;

impl ControlScheme {
    /// Flips the direction of every axis driving `control`. Additive inputs (a trigger
    /// pair on one rudder) swap their sign instead, since inverting a `0..1` trigger's
    /// shape would make its released position read as full deflection.
    pub fn toggle_invert(&mut self, control: Control) {
        for bind in self.axes.iter_mut().filter(|b| b.control == control) {
            match &mut bind.kind {
                BindKind::Additive(sign) => *sign = -*sign,
                BindKind::Absolute | BindKind::Rate => bind.shape.invert = !bind.shape.invert,
            }
        }
        self.additive.clear();
    }

    /// Steps the deadzone of `control`'s centred axes by `±0.02` within `0..=0.4`.
    pub fn step_deadzone(&mut self, control: Control, forward: bool) {
        let delta = if forward {
            DEADZONE_STEP
        } else {
            -DEADZONE_STEP
        };
        self.tune(control, |shape| {
            if shape.output == OutputRange::Centered {
                shape.deadzone = (shape.deadzone + delta).clamp(0.0, 0.4);
            }
        });
    }

    /// Steps the expo of `control`'s centred axes by `±0.1` within `0..=1`.
    pub fn step_expo(&mut self, control: Control, forward: bool) {
        let delta = if forward { EXPO_STEP } else { -EXPO_STEP };
        self.tune(control, |shape| {
            if shape.output == OutputRange::Centered {
                shape.expo = (shape.expo + delta).clamp(0.0, 1.0);
            }
        });
    }

    fn tune(&mut self, control: Control, f: impl Fn(&mut AxisShape)) {
        for bind in self.axes.iter_mut().filter(|b| b.control == control) {
            f(&mut bind.shape);
        }
    }

    /// Bindings of each flight control, for display.
    #[must_use]
    pub fn axis_rows(&self) -> Vec<AxisRow> {
        CONTROLS
            .into_iter()
            .map(|control| {
                let binds: Vec<&AxisBind> =
                    self.axes.iter().filter(|b| b.control == control).collect();
                let first = binds.first();
                let mut sources: Vec<String> =
                    binds.iter().map(|b| source_label(b.source)).collect();
                sources.extend(
                    self.buttons
                        .iter()
                        .filter(|b| b.control == control)
                        .map(|b| button_label(b.button).to_owned()),
                );
                AxisRow {
                    control,
                    sources: sources.join(" "),
                    invert: first.is_some_and(|b| self.is_inverted(b)),
                    deadzone: first.map_or(0.0, |b| b.shape.deadzone),
                    expo: first.map_or(0.0, |b| b.shape.expo),
                }
            })
            .collect()
    }

    /// Whether `bind` runs opposite to the preset: an additive input whose sign differs
    /// from the preset's bind of the same axis, otherwise an inverted shape.
    fn is_inverted(&self, bind: &AxisBind) -> bool {
        let BindKind::Additive(sign) = bind.kind else {
            return bind.shape.invert;
        };
        let preset_sign = self
            .preset()
            .axis_binds()
            .into_iter()
            .find(|p| p.source == bind.source && p.control == bind.control)
            .and_then(|p| match p.kind {
                BindKind::Additive(s) => Some(s),
                BindKind::Absolute | BindKind::Rate => None,
            });
        preset_sign.is_some_and(|s| s.signum() != sign.signum()) != bind.shape.invert
    }
}

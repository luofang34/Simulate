//! Reading and posing a [`DialSource`]: each pointer gets its own value,
//! from Indicate's state or straight from its mechanism, and Indicate's own
//! law places it.

use indicate_instrument_analog::{
    AnalogDialSpec, Channel, DialPose, DialReading, MAX_POINTERS, pose, reading,
};
use indicate_instrument_state::{BesideSamples, PanelData};

use super::{DialSource, FacePaint, Lane, pointer_spec};

/// Every pointer's value and the drum window's, in dial units; `None` where
/// nothing is shown.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PointerValues {
    /// Pointer values, in pointer order.
    pub pointers: [Option<f32>; MAX_POINTERS],
    /// The drum window's value.
    pub window: Option<f32>,
}

impl DialSource {
    /// Each pointer's value: a state lane read from `data` and the records
    /// `beside` it, a direct lane from `direct(pointer)` in dial units.
    #[must_use]
    pub fn values(
        &self,
        data: &PanelData,
        beside: &BesideSamples<'_>,
        direct: impl Fn(usize) -> Option<f32>,
    ) -> PointerValues {
        let mut values = PointerValues::default();
        for (index, (slot, pointer)) in values.pointers.iter_mut().zip(&self.pointers).enumerate() {
            *slot = match pointer.lane {
                Lane::State(channel) => channel.read(data, beside),
                Lane::Direct => direct(index),
            };
        }
        values.window = self
            .window
            .and_then(|(_, channel)| channel.read(data, beside));
        values
    }

    /// The dial's values and its pose: a built-in gauge read and posed whole
    /// by Indicate from `data` and `beside`, a pack dial pointer by pointer
    /// as [`Self::values`] and [`Self::pose`] do.
    #[must_use]
    pub fn read_and_pose(
        &self,
        data: &PanelData,
        beside: &BesideSamples<'_>,
        direct: impl Fn(usize) -> Option<f32>,
    ) -> (PointerValues, DialPose) {
        let Some(spec) = self.builtin_spec() else {
            let values = self.values(data, beside, direct);
            return (values, self.pose(&values));
        };
        let reading = reading(&spec, data, beside);
        let mut values = PointerValues::default();
        for (slot, pointer) in values.pointers.iter_mut().zip(spec.pointers) {
            *slot = match pointer.channel {
                Channel::Primary => reading.primary,
                Channel::Secondary => reading.secondary,
                Channel::Extra(n) => reading.extra.get(usize::from(n)).copied().flatten(),
            };
        }
        (values, pose(&spec, &reading))
    }

    /// Every pointer angle, drum wheel and the failure flag for `values`.
    /// Each pointer is posed alone by Indicate on its own value, so a case
    /// holds as many independent quantities as it has pointers.
    #[must_use]
    pub fn pose(&self, values: &PointerValues) -> DialPose {
        let frame = self.frame(FacePaint::Printed);
        let mut out = DialPose::default();
        for ((slot, pointer), value) in out
            .pointers
            .iter_mut()
            .zip(&self.pointers)
            .zip(values.pointers)
        {
            let one = [pointer_spec(pointer, FacePaint::Printed)];
            let spec = AnalogDialSpec {
                pointers: &one,
                input_range: pointer.input_range,
                ..frame
            };
            let posed = pose(
                &spec,
                &DialReading {
                    primary: value,
                    ..DialReading::default()
                },
            );
            if let Some(&posed) = posed.pointers.first() {
                *slot = posed;
            }
            out.flag |= !slot.live;
        }
        if let Some((window, _)) = &self.window {
            let windows = [*window];
            let spec = AnalogDialSpec {
                windows: &windows,
                ..frame
            };
            let posed = pose(
                &spec,
                &DialReading {
                    secondary: values.window,
                    ..DialReading::default()
                },
            );
            out.windows = posed.windows;
            out.flag |= posed.flag;
        }
        out
    }
}

use std::collections::BTreeMap;

use super::InputMapper;
use crate::{
    AxisShape, AxisSource, Control, ControlPresetId, DeviceId, FlightKeyAction, InputTarget,
    KeyBinding, KeyId, MappedBinding, ThrottleBehavior,
};

/// How far an axis must move from where it rested when capture began.
const CAPTURE_EXCURSION: f64 = 0.40;

/// What binding capture is waiting for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BindingCapture {
    /// Not capturing.
    Idle,
    /// Waiting for an axis to drive this target.
    Axis(InputTarget),
    /// Waiting for a key to hold this action.
    Key(FlightKeyAction),
}

/// An axis bound by [`InputMapper::start_axis_capture`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CapturedBinding {
    /// Device whose axis moved.
    pub device: DeviceId,
    /// Preset that was rebound; `None` for a device on a USB-specific profile.
    pub preset: Option<ControlPresetId>,
    /// The new binding.
    pub binding: MappedBinding,
    /// Whether the axis moved negative, so it was bound reversed.
    pub inverted: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PendingKind {
    Axis(InputTarget),
    Key(FlightKeyAction),
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct PendingCapture {
    pub(super) kind: PendingKind,
    /// Raw positions when capture began, so a stick or trigger already held
    /// off rest does not bind itself.
    baselines: BTreeMap<(DeviceId, AxisSource), f64>,
}

impl InputMapper {
    /// Binds the next axis moved more than 0.40 from rest to `target`.
    /// Move it the way that should read positive (for pitch, pull back);
    /// moving it the other way binds it reversed.
    pub fn start_axis_capture(&mut self, target: impl Into<InputTarget>) {
        let baselines = self
            .devices
            .iter()
            .flat_map(|(&id, d)| d.last_raw.iter().map(move |(&s, &v)| ((id, s), v)))
            .collect();
        self.capture = Some(PendingCapture {
            kind: PendingKind::Axis(target.into()),
            baselines,
        });
    }

    /// Binds the next key pressed to `action` in the active preset.
    pub fn start_key_capture(&mut self, action: FlightKeyAction) {
        self.capture = Some(PendingCapture {
            kind: PendingKind::Key(action),
            baselines: BTreeMap::new(),
        });
    }

    /// Abandons a pending capture.
    pub fn cancel_capture(&mut self) {
        self.capture = None;
    }

    /// What capture is waiting for.
    pub fn capture_state(&self) -> BindingCapture {
        match self.capture.as_ref().map(|c| c.kind) {
            None => BindingCapture::Idle,
            Some(PendingKind::Axis(target)) => BindingCapture::Axis(target),
            Some(PendingKind::Key(action)) => BindingCapture::Key(action),
        }
    }

    pub(super) fn try_capture(
        &mut self,
        device: DeviceId,
        source: AxisSource,
        raw: f64,
    ) -> Option<CapturedBinding> {
        let pending = self.capture.as_ref()?;
        let PendingKind::Axis(target) = pending.kind else {
            return None;
        };
        let baseline = pending
            .baselines
            .get(&(device, source))
            .copied()
            .unwrap_or(0.0);
        let excursion = raw - baseline;
        if excursion.abs() <= CAPTURE_EXCURSION {
            return None;
        }
        self.capture = None;
        let inverted = excursion < 0.0;
        let mut shape = self.capture_shape(device, target);
        if inverted {
            shape = shape.inverted();
        }
        let binding = MappedBinding::axis(source, target, shape);
        let preset = self.install_binding(device, binding);
        Some(CapturedBinding {
            device,
            preset,
            binding,
            inverted,
        })
    }

    fn capture_shape(&self, device: DeviceId, target: InputTarget) -> AxisShape {
        let lever = target == InputTarget::Flight(Control::Throttle)
            && self.device_throttle(device) == ThrottleBehavior::DirectLever;
        if lever {
            AxisShape::LEVER
        } else {
            AxisShape::STICK
        }
    }

    /// Replaces whatever drove the binding's target, and whatever the axis
    /// drove, in the bindings `device` follows, so one axis drives one target.
    fn install_binding(
        &mut self,
        device: DeviceId,
        binding: MappedBinding,
    ) -> Option<ControlPresetId> {
        let crate::BindingSource::Single(source) = binding.source else {
            return None;
        };
        let replace = |bindings: &mut Vec<MappedBinding>| {
            bindings.retain(|b| b.target != binding.target && !b.source.uses(source));
            bindings.push(binding);
        };
        let preset = self.device_preset(device);
        match preset {
            Some(id) => replace(&mut self.presets.get_mut(id).bindings),
            None => {
                if let Some(own) = self.devices.get_mut(&device).and_then(|d| d.own.as_mut()) {
                    replace(&mut own.bindings);
                }
            }
        }
        self.reset_device_outputs();
        preset
    }

    pub(super) fn bind_key(&mut self, key: &KeyId, action: FlightKeyAction) {
        let keys = &mut self.presets.get_mut(self.preset).keys;
        keys.retain(|b| b.key != *key && b.action != action);
        keys.push(KeyBinding {
            key: key.clone(),
            action,
        });
    }
}

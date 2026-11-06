use std::collections::BTreeMap;

use super::arbitration::is_meaningful;
use super::{ControlOwner, InputMapper};
use crate::{
    AxisShape, AxisSource, BindingSource, Control, ControlPresetId, DeviceId, DeviceInfo,
    InputRange, InputTarget, Layout, MappedBinding, ThrottleBehavior,
};

/// What the mapper knows about one connected device.
#[derive(Debug, Clone)]
pub(super) struct DeviceState {
    pub(super) info: DeviceInfo,
    /// Bindings from a USB-specific profile; `None` follows a preset.
    pub(super) own: Option<OwnProfile>,
    pub(super) last_raw: BTreeMap<AxisSource, f64>,
    pub(super) last_shaped: BTreeMap<InputTarget, f64>,
}

/// A USB-specific profile copied at connect time.
#[derive(Debug, Clone)]
pub(super) struct OwnProfile {
    pub(super) name: String,
    pub(super) bindings: Vec<MappedBinding>,
}

impl InputMapper {
    pub(super) fn connect(&mut self, info: &DeviceInfo) {
        let profile = self.library.select(info);
        let own = profile.usb_id.is_some().then(|| OwnProfile {
            name: profile.name.clone(),
            bindings: profile.bindings.iter().copied().map(Into::into).collect(),
        });
        self.release_device(info.id);
        self.devices.insert(
            info.id,
            DeviceState {
                info: info.clone(),
                own,
                last_raw: BTreeMap::new(),
                last_shaped: BTreeMap::new(),
            },
        );
    }

    pub(super) fn disconnect(&mut self, id: DeviceId) {
        self.release_device(id);
        self.devices.remove(&id);
    }

    /// Hands everything `id` drives back to the keyboard; the latched
    /// throttle keeps its position.
    fn release_device(&mut self, id: DeviceId) {
        for control in [
            Control::Roll,
            Control::Pitch,
            Control::Yaw,
            Control::Throttle,
        ] {
            let owner = self.owners.get_mut(control);
            if *owner == ControlOwner::Device(id) {
                *owner = ControlOwner::Keyboard;
            }
        }
        for camera in [&mut self.camera_pan, &mut self.camera_tilt] {
            if camera.1 == Some(id) {
                *camera = (0.0, None);
            }
        }
    }

    /// The preset `device` follows; `None` for a USB-specific profile or an
    /// unknown device.
    pub(super) fn device_preset(&self, device: DeviceId) -> Option<ControlPresetId> {
        let state = self.devices.get(&device)?;
        if state.own.is_some() {
            return None;
        }
        Some(match state.info.layout {
            Layout::Standard => self.preset,
            Layout::NonStandard => ControlPresetId::FlightStickHotas,
        })
    }

    pub(super) fn device_bindings(&self, device: DeviceId) -> Vec<MappedBinding> {
        match self.device_preset(device) {
            Some(id) => self.presets.get(id).bindings.clone(),
            None => self
                .devices
                .get(&device)
                .and_then(|d| d.own.as_ref())
                .map(|own| own.bindings.clone())
                .unwrap_or_default(),
        }
    }

    /// USB-specific profiles hold absolute positions, so they act as levers.
    pub(super) fn device_throttle(&self, device: DeviceId) -> ThrottleBehavior {
        self.device_preset(device)
            .map_or(ThrottleBehavior::DirectLever, |id| {
                self.presets.get(id).throttle
            })
    }

    pub(super) fn on_axis(
        &mut self,
        device: DeviceId,
        source: AxisSource,
        value: f32,
    ) -> Option<crate::CapturedBinding> {
        let raw = if value.is_nan() {
            0.0
        } else {
            f64::from(value)
        };
        self.devices.get_mut(&device)?.last_raw.insert(source, raw);
        if let Some(captured) = self.try_capture(device, source, raw) {
            return Some(captured);
        }
        let behavior = self.device_throttle(device);
        for binding in self.device_bindings(device) {
            if binding.source.uses(source) {
                self.apply_binding(device, &binding, behavior);
            }
        }
        None
    }

    fn apply_binding(
        &mut self,
        device: DeviceId,
        binding: &MappedBinding,
        behavior: ThrottleBehavior,
    ) {
        let tuned = self.tuned_shape(binding);
        let (shape, input) = match binding.source {
            BindingSource::Single(s) => (tuned, self.last_raw(device, s).unwrap_or(0.0)),
            BindingSource::Differential { positive, negative } => {
                let side = |s| self.last_raw(device, s).unwrap_or(0.0).clamp(0.0, 1.0);
                // The pair's difference is centred on zero whatever shape a
                // saved preset carries; reading it as 0..1 would turn released
                // triggers into full deflection.
                let bipolar = AxisShape {
                    input: InputRange::Bipolar,
                    ..tuned
                };
                (bipolar, side(positive) - side(negative))
            }
        };
        let shaped = shape.apply_f64(input);
        let Some(state) = self.devices.get_mut(&device) else {
            return;
        };
        let previous = state.last_shaped.insert(binding.target, shaped);
        if !is_meaningful(previous, shaped, shape.output) {
            return;
        }
        match binding.target {
            InputTarget::Flight(control) => {
                *self.owners.get_mut(control) = ControlOwner::Device(device);
                if control == Control::Throttle && behavior == ThrottleBehavior::DirectLever {
                    self.latched_throttle = shaped.clamp(0.0, 1.0);
                }
            }
            InputTarget::CameraPan => self.camera_pan = (shaped, Some(device)),
            InputTarget::CameraTilt => self.camera_tilt = (shaped, Some(device)),
        }
    }
}

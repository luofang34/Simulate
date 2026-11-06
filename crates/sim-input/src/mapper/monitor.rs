use sim_core::ControlInputs;

use super::{BindingCapture, ControlOwner, InputMapper};
use crate::{
    AxisSource, Control, ControlPresetId, DeviceId, FlightKeyAction, InputTarget, Layout,
    ThrottleBehavior, VehicleControlClass,
};

/// Everything a controls menu needs to draw live input bars.
#[derive(Debug, Clone, PartialEq)]
pub struct ControlMonitorSnapshot {
    /// Preset for standard gamepads and the keyboard.
    pub preset: ControlPresetId,
    /// Vehicle class being flown.
    pub vehicle_class: VehicleControlClass,
    /// The active preset's throttle behaviour.
    pub throttle_behavior: ThrottleBehavior,
    /// Source driving roll, pitch, yaw, and throttle, in that order.
    pub active_owner: [ControlOwner; 4],
    /// Final control positions.
    pub controls: ControlInputs,
    /// Camera look, `[pan, tilt]`.
    pub camera_look: [f64; 2],
    /// Latched throttle lever.
    pub latched_throttle: f64,
    /// Throttle trim.
    pub trim_throttle: f64,
    /// Actions keys and buttons hold.
    pub held_actions: Vec<FlightKeyAction>,
    /// What binding capture is waiting for.
    pub capture: BindingCapture,
    /// Connected devices.
    pub devices: Vec<DeviceMonitor>,
}

/// One connected device's live state.
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceMonitor {
    /// Backend identifier.
    pub id: DeviceId,
    /// Name the device reports.
    pub name: String,
    /// Axis layout.
    pub layout: Layout,
    /// Preset it follows; `None` for a USB-specific profile.
    pub preset: Option<ControlPresetId>,
    /// Name of the preset or profile it follows.
    pub profile_name: String,
    /// How its throttle axis moves the throttle.
    pub throttle_behavior: ThrottleBehavior,
    /// Last raw reading of every axis it has reported.
    pub axes: Vec<AxisReading>,
    /// Last shaped output for every target it drives.
    pub outputs: Vec<TargetReading>,
}

/// A raw axis reading.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisReading {
    /// The axis.
    pub source: AxisSource,
    /// Its last raw value.
    pub raw: f64,
}

/// A shaped binding output.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TargetReading {
    /// What it drives.
    pub target: InputTarget,
    /// Its last shaped value.
    pub shaped: f64,
}

impl InputMapper {
    /// A snapshot of every source and the arbitrated result.
    pub fn monitor_snapshot(&self) -> ControlMonitorSnapshot {
        ControlMonitorSnapshot {
            preset: self.preset,
            vehicle_class: self.vehicle_class,
            throttle_behavior: self.throttle_behavior(),
            active_owner: [
                self.owner(Control::Roll),
                self.owner(Control::Pitch),
                self.owner(Control::Yaw),
                self.owner(Control::Throttle),
            ],
            controls: self.controls(),
            camera_look: self.camera_look(),
            latched_throttle: self.latched_throttle,
            trim_throttle: self.trim_throttle,
            held_actions: self.held_actions(),
            capture: self.capture_state(),
            devices: self
                .devices
                .keys()
                .map(|&id| self.device_monitor(id))
                .collect(),
        }
    }

    fn device_monitor(&self, id: DeviceId) -> DeviceMonitor {
        let state = self.devices.get(&id);
        let preset = self.device_preset(id);
        let profile_name = match (preset, state.and_then(|s| s.own.as_ref())) {
            (Some(p), _) => self.presets.get(p).name.clone(),
            (None, Some(own)) => own.name.clone(),
            (None, None) => String::new(),
        };
        DeviceMonitor {
            id,
            name: state.map(|s| s.info.name.clone()).unwrap_or_default(),
            layout: state.map_or(Layout::NonStandard, |s| s.info.layout),
            preset,
            profile_name,
            throttle_behavior: self.device_throttle(id),
            axes: state
                .map(|s| {
                    s.last_raw
                        .iter()
                        .map(|(&source, &raw)| AxisReading { source, raw })
                        .collect()
                })
                .unwrap_or_default(),
            outputs: state
                .map(|s| {
                    s.last_shaped
                        .iter()
                        .map(|(&target, &shaped)| TargetReading { target, shaped })
                        .collect()
                })
                .unwrap_or_default(),
        }
    }
}

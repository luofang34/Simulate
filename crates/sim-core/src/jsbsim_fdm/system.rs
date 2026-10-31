use jsbsim::{Fdm, NodeId};

use super::fdm_error;
use crate::BlockError;

/// The ignition switch of a piston engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MagnetoSwitch {
    /// Neither magneto: no spark.
    Off = 0,
    /// Left magneto only.
    Left = 1,
    /// Right magneto only.
    Right = 2,
    /// Both magnetos.
    Both = 3,
    /// Both magnetos with the starter engaged, as the key held past `Both`.
    Start,
}

impl MagnetoSwitch {
    /// The value of `propulsion/magneto_cmd`.
    pub(super) fn magneto_cmd(self) -> f64 {
        match self {
            Self::Start => f64::from(Self::Both as u8),
            other => f64::from(other as u8),
        }
    }
}

/// How the engine sits when the block first runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngineStartMode {
    /// Shut down, under [`SystemControls::cold_and_dark`]: a piston engine
    /// starts at 0 RPM and has to be started.
    ColdAndDark,
    /// Already running, under [`SystemControls::running_defaults`].
    Running,
}

/// What the block reads of the engine after each step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EngineStatus {
    /// Whether the first engine is making power.
    pub running: bool,
    /// Speed of its propeller, RPM; zero for an aircraft with none.
    pub propeller_rpm: f64,
}

/// The operator controls beyond the flight controls, latched by the block and
/// applied on every step. A control the loaded aircraft lacks does nothing, so
/// one set of controls fits a piston aeroplane and an electric trainer alike.
///
/// JSBSim's aliases reach the first engine only, which is what a single fits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SystemControls {
    /// The ignition switch.
    pub magnetos: MagnetoSwitch,
    /// Engages the starter independently of the ignition switch.
    pub starter: bool,
    /// Mixture lever, 0 (idle cutoff) to 1 (full rich).
    pub mixture_norm: f64,
    /// Propeller lever, 0 (coarse) to 1 (fine).
    pub prop_pitch_norm: f64,
    /// Shuts the fuel off. JSBSim has a cutoff property only for turbines, so
    /// on a piston engine this acts as the mixture at idle cutoff.
    pub fuel_cutoff: bool,
    /// Flap lever, 0 (up) to 1 (full).
    pub flaps_norm: f64,
    /// Gear lever.
    pub gear_down: bool,
    /// Both wheel brakes, 0 (off) to 1 (full).
    pub wheel_brake_norm: f64,
}

impl SystemControls {
    /// Everything set for flight: both magnetos, full rich, fine pitch, gear
    /// down, brakes and flaps off.
    pub const fn running_defaults() -> Self {
        Self {
            magnetos: MagnetoSwitch::Both,
            starter: false,
            mixture_norm: 1.0,
            prop_pitch_norm: 1.0,
            fuel_cutoff: false,
            flaps_norm: 0.0,
            gear_down: true,
            wheel_brake_norm: 0.0,
        }
    }

    /// An aircraft parked and shut down: ignition off, idle cutoff, fuel off,
    /// brakes set.
    pub const fn cold_and_dark() -> Self {
        Self {
            magnetos: MagnetoSwitch::Off,
            mixture_norm: 0.0,
            fuel_cutoff: true,
            wheel_brake_norm: 1.0,
            ..Self::running_defaults()
        }
    }

    /// The same controls limited to their valid ranges; NaN becomes the
    /// lever's lower stop.
    #[must_use]
    pub fn clamped(self) -> Self {
        let lever = |v: f64| if v.is_nan() { 0.0 } else { v.clamp(0.0, 1.0) };
        Self {
            mixture_norm: lever(self.mixture_norm),
            prop_pitch_norm: lever(self.prop_pitch_norm),
            flaps_norm: lever(self.flaps_norm),
            wheel_brake_norm: lever(self.wheel_brake_norm),
            ..self
        }
    }
}

/// The properties [`SystemControls`] drive. Each may be absent from a given
/// aircraft; resolved once so a step does no path lookups.
#[derive(Debug, Clone, Copy)]
pub(super) struct SystemNodes {
    magnetos: Option<NodeId>,
    starter: Option<NodeId>,
    mixture: Option<NodeId>,
    prop_pitch: Option<NodeId>,
    cutoff: Option<NodeId>,
    flaps: Option<NodeId>,
    gear: Option<NodeId>,
    brake_left: Option<NodeId>,
    brake_right: Option<NodeId>,
    set_running: Option<NodeId>,
    engine_running: Option<NodeId>,
    propeller_rpm: Option<NodeId>,
}

impl SystemNodes {
    pub(super) fn resolve(fdm: &Fdm) -> Result<Self, BlockError> {
        let node = |path: &str| fdm.node(path).map_err(fdm_error);
        Ok(Self {
            magnetos: node("propulsion/magneto_cmd")?,
            starter: node("propulsion/starter_cmd")?,
            mixture: node("fcs/mixture-cmd-norm")?,
            prop_pitch: node("fcs/advance-cmd-norm")?,
            cutoff: node("propulsion/cutoff_cmd")?,
            flaps: node("fcs/flap-cmd-norm")?,
            gear: node("gear/gear-cmd-norm")?,
            brake_left: node("fcs/left-brake-cmd-norm")?,
            brake_right: node("fcs/right-brake-cmd-norm")?,
            set_running: node("propulsion/set-running")?,
            engine_running: node("propulsion/engine[0]/set-running")?,
            propeller_rpm: node("propulsion/engine[0]/propeller-rpm")?,
        })
    }

    /// Writes `controls` to every property the aircraft has.
    pub(super) fn write(&self, fdm: &mut Fdm, controls: SystemControls) -> Result<(), BlockError> {
        let flag = |on: bool| if on { 1.0 } else { 0.0 };
        let cranking = controls.starter || controls.magnetos == MagnetoSwitch::Start;
        let mixture = if controls.fuel_cutoff && self.cutoff.is_none() {
            0.0
        } else {
            controls.mixture_norm
        };
        let commands = [
            (self.magnetos, controls.magnetos.magneto_cmd()),
            (self.starter, flag(cranking)),
            (self.mixture, mixture),
            (self.prop_pitch, controls.prop_pitch_norm),
            (self.cutoff, flag(controls.fuel_cutoff)),
            (self.flaps, controls.flaps_norm),
            (self.gear, flag(controls.gear_down)),
            (self.brake_left, controls.wheel_brake_norm),
            (self.brake_right, controls.wheel_brake_norm),
        ];
        for (node, value) in commands {
            if let Some(node) = node {
                fdm.write_node(node, value).map_err(fdm_error)?;
            }
        }
        Ok(())
    }

    /// Asks JSBSim to start every engine, on an aircraft that has any.
    pub(super) fn set_running(&self, fdm: &mut Fdm) -> Result<(), BlockError> {
        if let Some(node) = self.set_running {
            fdm.write_node(node, -1.0).map_err(fdm_error)?;
        }
        Ok(())
    }

    pub(super) fn engine_status(&self, fdm: &mut Fdm) -> Result<EngineStatus, BlockError> {
        let mut read = |node: Option<NodeId>| match node {
            Some(node) => fdm.read_node(node).map_err(fdm_error),
            None => Ok(0.0),
        };
        Ok(EngineStatus {
            running: read(self.engine_running)? != 0.0,
            propeller_rpm: read(self.propeller_rpm)?,
        })
    }
}

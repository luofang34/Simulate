use jsbsim::{Fdm, NodeId};

use super::fdm_error;
use crate::BlockError;
use crate::airframe::CanopyOpening;

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
    /// Closes the hydraulic flap selector valve (`CLOSED` detent on P-38G), holding
    /// the Fowler flaps at their current extension regardless of `flaps_norm`.
    pub flap_valve_closed: bool,
    /// Three-axis cockpit trim tabs `[aileron_trim, elevator_trim, rudder_trim]` in `-1..=1`.
    pub trim_norm: [f64; 3],
    /// Optional independent left/right throttle overrides `[left, right]` in `0..=1`.
    /// When `None`, both engines track the primary throttle input.
    pub throttle_split: Option<[f64; 2]>,
    /// Optional independent left/right mixture overrides `[left, right]` in `0..=1`.
    pub mixture_split: Option<[f64; 2]>,
    /// Optional independent left/right propeller governor overrides `[left, right]` in `0..=1`.
    pub prop_pitch_split: Option<[f64; 2]>,
    /// Curtiss Electric full-feathering switches `[left_feathered, right_feathered]`.
    pub prop_feather: [bool; 2],
    /// Left and right hydraulic coolant-radiator exit flap openings `[left, right]` in `0..=1`.
    pub coolant_flap_norm: [f64; 2],
    /// Left and right electric oil-cooler shutter openings `[left, right]` in `0..=1`.
    pub oil_shutter_norm: [f64; 2],
    /// Gear lever.
    pub gear_down: bool,
    /// Landing light switch (`true` on, `false` off).
    pub landing_light: bool,
    /// Both wheel brakes, 0 (off) to 1 (full).
    pub wheel_brake_norm: f64,
    /// The canopy's hatch and side windows. The flight model takes no drag or
    /// noise from an open canopy; this is what the airframe is drawn with.
    pub canopy: CanopyOpening,
}

impl SystemControls {
    /// Everything set for flight: both magnetos, full rich, fine pitch, gear
    /// down, landing light on, brakes and flaps off, normal automatic cooling flaps.
    pub const fn running_defaults() -> Self {
        Self {
            magnetos: MagnetoSwitch::Both,
            starter: false,
            mixture_norm: 1.0,
            prop_pitch_norm: 1.0,
            fuel_cutoff: false,
            flaps_norm: 0.0,
            flap_valve_closed: false,
            trim_norm: [0.0; 3],
            throttle_split: None,
            mixture_split: None,
            prop_pitch_split: None,
            prop_feather: [false; 2],
            coolant_flap_norm: [0.20; 2],
            oil_shutter_norm: [0.20; 2],
            gear_down: true,
            landing_light: true,
            wheel_brake_norm: 0.0,
            canopy: CanopyOpening {
                hatch: 0.0,
                side_windows: [0.0; 2],
            },
        }
    }

    /// An aircraft parked and shut down: ignition off, idle cutoff, fuel off,
    /// landing light off, cooling flaps closed, brakes set.
    pub const fn cold_and_dark() -> Self {
        Self {
            magnetos: MagnetoSwitch::Off,
            mixture_norm: 0.0,
            fuel_cutoff: true,
            coolant_flap_norm: [0.0; 2],
            oil_shutter_norm: [0.0; 2],
            landing_light: false,
            wheel_brake_norm: 1.0,
            canopy: CanopyOpening {
                hatch: 0.0,
                side_windows: [0.0; 2],
            },
            ..Self::running_defaults()
        }
    }

    /// The same controls limited to their valid ranges; NaN becomes the
    /// lever's lower stop.
    #[must_use]
    pub fn clamped(self) -> Self {
        let lever = |v: f64| if v.is_nan() { 0.0 } else { v.clamp(0.0, 1.0) };
        let sym = |v: f64| if v.is_nan() { 0.0 } else { v.clamp(-1.0, 1.0) };
        Self {
            mixture_norm: lever(self.mixture_norm),
            prop_pitch_norm: lever(self.prop_pitch_norm),
            flaps_norm: lever(self.flaps_norm),
            trim_norm: self.trim_norm.map(sym),
            throttle_split: self.throttle_split.map(|pair| pair.map(lever)),
            mixture_split: self.mixture_split.map(|pair| pair.map(lever)),
            prop_pitch_split: self.prop_pitch_split.map(|pair| pair.map(lever)),
            coolant_flap_norm: self.coolant_flap_norm.map(lever),
            oil_shutter_norm: self.oil_shutter_norm.map(lever),
            wheel_brake_norm: lever(self.wheel_brake_norm),
            canopy: CanopyOpening {
                hatch: 0.0,
                side_windows: [0.0; 2],
            },
            ..self
        }
    }
}

/// The properties [`SystemControls`] drive. Each may be absent from a given
/// aircraft; resolved once so a step does no path lookups.
#[derive(Debug, Clone, Copy)]
pub(super) struct SystemNodes {
    throttle_0: Option<NodeId>,
    throttle_aux: [Option<NodeId>; 3],
    magnetos: Option<NodeId>,
    starter: Option<NodeId>,
    mixture: Option<NodeId>,
    mixture_aux: [Option<NodeId>; 3],
    prop_pitch: Option<NodeId>,
    prop_pitch_aux: [Option<NodeId>; 3],
    feather: [Option<NodeId>; 2],
    coolant_flap: [Option<NodeId>; 2],
    oil_shutter: [Option<NodeId>; 2],
    trim: [Option<NodeId>; 3],
    cutoff: Option<NodeId>,
    flaps: Option<NodeId>,
    gear: Option<NodeId>,
    gear_pos: Option<NodeId>,
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
            throttle_0: node("fcs/throttle-cmd-norm")?,
            throttle_aux: [
                node("fcs/throttle-cmd-norm[1]")?,
                node("fcs/throttle-cmd-norm[2]")?,
                node("fcs/throttle-cmd-norm[3]")?,
            ],
            magnetos: node("propulsion/magneto_cmd")?,
            starter: node("propulsion/starter_cmd")?,
            mixture: node("fcs/mixture-cmd-norm")?,
            mixture_aux: [
                node("fcs/mixture-cmd-norm[1]")?,
                node("fcs/mixture-cmd-norm[2]")?,
                node("fcs/mixture-cmd-norm[3]")?,
            ],
            prop_pitch: node("fcs/advance-cmd-norm")?,
            prop_pitch_aux: [
                node("fcs/advance-cmd-norm[1]")?,
                node("fcs/advance-cmd-norm[2]")?,
                node("fcs/advance-cmd-norm[3]")?,
            ],
            feather: [
                node("fcs/feather-left-norm")?,
                node("fcs/feather-right-norm")?,
            ],
            coolant_flap: [
                node("fcs/coolant-flap-left-norm")?,
                node("fcs/coolant-flap-right-norm")?,
            ],
            oil_shutter: [
                node("fcs/oil-shutter-left-norm")?,
                node("fcs/oil-shutter-right-norm")?,
            ],
            trim: [
                node("fcs/roll-trim-cmd-norm")?,
                node("fcs/pitch-trim-cmd-norm")?,
                node("fcs/yaw-trim-cmd-norm")?,
            ],
            cutoff: node("propulsion/cutoff_cmd")?,
            flaps: node("fcs/flap-cmd-norm")?,
            gear: node("gear/gear-cmd-norm")?,
            gear_pos: node("gear/gear-pos-norm")?,
            brake_left: node("fcs/left-brake-cmd-norm")?,
            brake_right: node("fcs/right-brake-cmd-norm")?,
            set_running: node("propulsion/set-running")?,
            engine_running: node("propulsion/engine[0]/set-running")?,
            propeller_rpm: node("propulsion/engine[0]/propeller-rpm")?,
        })
    }

    /// Synchronizes `gear/gear-pos-norm` immediately with `controls.gear_down`
    /// during initial-condition setup or steady-state trim.
    pub(super) fn sync_gear_pos(
        &self,
        fdm: &mut Fdm,
        controls: SystemControls,
    ) -> Result<(), BlockError> {
        let pos = if controls.gear_down { 1.0 } else { 0.0 };
        if let Some(node) = self.gear_pos {
            fdm.write_node(node, pos).map_err(fdm_error)?;
        }
        Ok(())
    }

    /// Slews `gear/gear-pos-norm` toward `controls.gear_down` over a 5-second
    /// hydraulic retraction/extension cycle.
    pub(super) fn step_gear_pos(
        &self,
        fdm: &mut Fdm,
        controls: SystemControls,
        dt_s: f64,
    ) -> Result<(), BlockError> {
        let Some(node) = self.gear_pos else {
            return Ok(());
        };
        let target = if controls.gear_down { 1.0 } else { 0.0 };
        let current = fdm.read_node(node).map_err(fdm_error)?.clamp(0.0, 1.0);
        let max_delta = (dt_s.max(0.0) / 5.0).clamp(0.0, 1.0);
        let next = current + (target - current).clamp(-max_delta, max_delta);
        fdm.write_node(node, next).map(|_| ()).map_err(fdm_error)
    }

    /// Writes auxiliary throttles, latched system controls, and slews `gear/gear-pos-norm`.
    pub(super) fn write_step(
        &self,
        fdm: &mut Fdm,
        throttle: f64,
        controls: SystemControls,
        dt_s: f64,
    ) -> Result<(), BlockError> {
        self.write_throttle_with_controls(fdm, throttle, controls)?;
        self.write(fdm, controls)?;
        self.step_gear_pos(fdm, controls, dt_s)
    }

    /// Writes `controls` and immediately synchronizes `gear/gear-pos-norm`.
    pub(super) fn write_and_sync_gear(
        &self,
        fdm: &mut Fdm,
        controls: SystemControls,
    ) -> Result<(), BlockError> {
        self.write(fdm, controls)?;
        self.sync_gear_pos(fdm, controls)
    }

    fn write_throttle_with_controls(
        &self,
        fdm: &mut Fdm,
        throttle: f64,
        controls: SystemControls,
    ) -> Result<(), BlockError> {
        let [t_left, t_right] = controls.throttle_split.unwrap_or([throttle, throttle]);
        let [f_left, f_right] = controls.prop_feather;
        let eff_left = if f_left { 0.0 } else { t_left };
        let eff_right = if f_right { 0.0 } else { t_right };
        if let Some(node) = self.throttle_0 {
            fdm.write_node(node, eff_left).map_err(fdm_error)?;
        }
        if let Some(Some(node)) = self.throttle_aux.first() {
            fdm.write_node(*node, eff_right).map_err(fdm_error)?;
        }
        for node in self.throttle_aux.into_iter().skip(1).flatten() {
            fdm.write_node(node, throttle).map_err(fdm_error)?;
        }
        Ok(())
    }

    /// Writes `controls` to every property the aircraft has.
    pub(super) fn write(&self, fdm: &mut Fdm, controls: SystemControls) -> Result<(), BlockError> {
        let flag = |on: bool| if on { 1.0 } else { 0.0 };
        let cranking = controls.starter || controls.magnetos == MagnetoSwitch::Start;
        let base_mix = if controls.fuel_cutoff && self.cutoff.is_none() {
            0.0
        } else {
            controls.mixture_norm
        };
        let [m_left, m_right] = controls.mixture_split.unwrap_or([base_mix, base_mix]);
        let [p_left, p_right] = controls
            .prop_pitch_split
            .unwrap_or([controls.prop_pitch_norm, controls.prop_pitch_norm]);
        let [f_left, f_right] = controls.prop_feather;
        let [c_left, c_right] = controls.coolant_flap_norm;
        let [o_left, o_right] = controls.oil_shutter_norm;
        let [tr_roll, tr_pitch, tr_yaw] = controls.trim_norm;
        let [trim_r, trim_p, trim_y] = self.trim;
        let [feath_l, feath_r] = self.feather;
        let [cool_l, cool_r] = self.coolant_flap;
        let [oil_l, oil_r] = self.oil_shutter;
        let commands = [
            (self.magnetos, controls.magnetos.magneto_cmd()),
            (self.starter, flag(cranking)),
            (self.mixture, if f_left { 0.0 } else { m_left }),
            (self.prop_pitch, if f_left { 0.0 } else { p_left }),
            (self.cutoff, flag(controls.fuel_cutoff)),
            (
                self.flaps.filter(|_| !controls.flap_valve_closed),
                controls.flaps_norm,
            ),
            (self.gear, flag(controls.gear_down)),
            (self.brake_left, controls.wheel_brake_norm),
            (self.brake_right, controls.wheel_brake_norm),
            (trim_r.filter(|_| tr_roll.abs() > 1e-9), tr_roll),
            (trim_p.filter(|_| tr_pitch.abs() > 1e-9), tr_pitch),
            (trim_y.filter(|_| tr_yaw.abs() > 1e-9), tr_yaw),
            (feath_l, flag(f_left)),
            (feath_r, flag(f_right)),
            (cool_l, c_left),
            (cool_r, c_right),
            (oil_l, o_left),
            (oil_r, o_right),
        ];
        for (node, value) in commands {
            if let Some(node) = node {
                fdm.write_node(node, value).map_err(fdm_error)?;
            }
        }
        self.write_aux_engine_controls(fdm, base_mix, m_right, p_right, f_right, controls)
    }

    fn write_aux_engine_controls(
        &self,
        fdm: &mut Fdm,
        base_mix: f64,
        m_right: f64,
        p_right: f64,
        f_right: bool,
        controls: SystemControls,
    ) -> Result<(), BlockError> {
        if let Some(Some(node)) = self.mixture_aux.first() {
            fdm.write_node(*node, if f_right { 0.0 } else { m_right })
                .map_err(fdm_error)?;
        }
        for node in self.mixture_aux.into_iter().skip(1).flatten() {
            fdm.write_node(node, base_mix).map_err(fdm_error)?;
        }
        if let Some(Some(node)) = self.prop_pitch_aux.first() {
            fdm.write_node(*node, if f_right { 0.0 } else { p_right })
                .map_err(fdm_error)?;
        }
        for node in self.prop_pitch_aux.into_iter().skip(1).flatten() {
            fdm.write_node(node, controls.prop_pitch_norm)
                .map_err(fdm_error)?;
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

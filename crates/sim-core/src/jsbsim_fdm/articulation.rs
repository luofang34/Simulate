use std::f64::consts::{PI, TAU};

use jsbsim::{Fdm, NodeId};

use super::fdm_error;
use crate::{BlockError, ControlInputs, VehicleState};

const METRES_PER_FOOT: f64 = 0.3048;
const NEWTONS_PER_POUND_FORCE: f64 = 4.448_221_615_260_5;
const DEFAULT_BLADE_ANGLE_DEG: f64 = 22.0;
const DEFAULT_MAX_FLAP_DEG: f64 = 30.0;
const NOSE_WHEEL_RADIUS_FT: f64 = 0.58;
const MAIN_WHEEL_RADIUS_FT: f64 = 0.68;

/// Optical appearance of a spinning aircraft propeller for a given camera exposure time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropellerVisualState {
    /// Instantaneous hub rotation angle around body `+X` (radians in `[0, 2π)`).
    pub phase_rad: f32,
    /// Blade pitch angle relative to the plane of rotation (degrees).
    pub blade_angle_deg: f32,
    /// Opacity `[0, 1]` of the discrete solid 3D blades (`1.0` stopped/cranking, `0.0` at cruise RPM).
    pub blade_opacity: f32,
    /// Peak opacity `[0, 1]` of the motion-blurred translucent propeller disc (`0.0` stopped).
    pub disc_opacity: f32,
    /// Azimuthal sector swept by one blade during `shutter_exposure_s` (radians).
    pub shutter_blur_angle_rad: f32,
    /// Number of propeller blades (`2` on Cessna 172 McCauley fixed-pitch prop).
    pub blade_count: u32,
}

/// Complete engine, propeller, control-surface, and landing-gear articulation state of an airframe.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirframeArticulation {
    /// Whether the primary engine is running under its own combustion.
    pub engine_running: bool,
    /// Whether the electric starter motor is engaged.
    pub starter_engaged: bool,
    /// Crankshaft speed in revolutions per minute.
    pub engine_rpm: f64,
    /// Propeller shaft speed in revolutions per minute.
    pub propeller_rpm: f64,
    /// Direction of propeller rotation viewed from the cockpit (`+1.0` clockwise, `-1.0` CCW).
    pub propeller_sense: f64,
    /// Propeller blade geometric pitch at the reference station (degrees).
    pub propeller_blade_angle_deg: f64,
    /// Deterministically integrated propeller shaft angle in `[0, 2π)` radians.
    pub propeller_phase_rad: f64,
    /// Net propeller axial thrust in newtons.
    pub propeller_thrust_n: f64,
    /// Intake manifold absolute pressure in pascals.
    pub manifold_pressure_pa: f64,
    /// Fuel consumption rate in US gallons per hour.
    pub fuel_flow_gph: f64,
    /// Exhaust gas temperature (`°F`).
    pub egt_deg_f: f64,
    /// Cylinder head temperature (`°F`).
    pub cht_deg_f: f64,
    /// Engine oil temperature (`°F`).
    pub oil_temp_deg_f: f64,
    /// Engine oil pressure (`psi`).
    pub oil_pressure_psi: f64,
    /// Elastic engine-mount angular deflection `[roll, pitch, yaw]` in body radians.
    pub engine_vibration_rad: [f64; 3],
    /// Deployed trailing-edge flap deflection in degrees (`0..=40`).
    pub flap_deg: f64,
    /// Left aileron hinge deflection in radians.
    pub left_aileron_rad: f64,
    /// Right aileron hinge deflection in radians.
    pub right_aileron_rad: f64,
    /// Elevator hinge deflection in radians.
    pub elevator_rad: f64,
    /// Rudder hinge deflection in radians.
    pub rudder_rad: f64,
    /// Steerable nosewheel yaw deflection in radians.
    pub nosewheel_steer_rad: f64,
    /// Weight-on-wheels flags `[nose, left_main, right_main]`.
    pub gear_wow: [bool; 3],
    /// Landing-gear strut compression `[nose, left_main, right_main]` in metres.
    pub gear_compression_m: [f64; 3],
    /// Integrated rolling wheel angles `[nose, left_main, right_main]` in `[0, 2π)` radians.
    pub wheel_phase_rad: [f64; 3],
    /// Anti-collision red fin beacon normalized intensity `[0, 1]`.
    pub beacon_flash_norm: f64,
    /// Integrated electrical/beacon phase in `[0, 2π)` radians.
    pub beacon_phase_rad: f64,
}

impl Default for AirframeArticulation {
    fn default() -> Self {
        Self {
            engine_running: false,
            starter_engaged: false,
            engine_rpm: 0.0,
            propeller_rpm: 0.0,
            propeller_sense: 1.0,
            propeller_blade_angle_deg: DEFAULT_BLADE_ANGLE_DEG,
            propeller_phase_rad: 0.0,
            propeller_thrust_n: 0.0,
            manifold_pressure_pa: 101_325.0,
            fuel_flow_gph: 0.0,
            egt_deg_f: 70.0,
            cht_deg_f: 70.0,
            oil_temp_deg_f: 70.0,
            oil_pressure_psi: 0.0,
            engine_vibration_rad: [0.0; 3],
            flap_deg: 0.0,
            left_aileron_rad: 0.0,
            right_aileron_rad: 0.0,
            elevator_rad: 0.0,
            rudder_rad: 0.0,
            nosewheel_steer_rad: 0.0,
            gear_wow: [false; 3],
            gear_compression_m: [0.0; 3],
            wheel_phase_rad: [0.0; 3],
            beacon_flash_norm: 0.0,
            beacon_phase_rad: 0.0,
        }
    }
}

impl AirframeArticulation {
    /// Computes the optical blade vs. blurred-disc state for a camera with `shutter_exposure_s`.
    #[must_use]
    pub fn propeller_visual(&self, shutter_exposure_s: f64) -> PropellerVisualState {
        let abs_rpm = self.propeller_rpm.abs();
        let omega = abs_rpm * TAU / 60.0;
        let blade_t = smoothstep(350.0, 900.0, abs_rpm);
        let disc_t = smoothstep(240.0, 1_900.0, abs_rpm);
        let exposure = shutter_exposure_s.clamp(1.0 / 2_000.0, 1.0 / 24.0);
        let blur = (omega * exposure).clamp(0.06, PI);
        PropellerVisualState {
            phase_rad: self.propeller_phase_rad.rem_euclid(TAU) as f32,
            blade_angle_deg: self.propeller_blade_angle_deg as f32,
            blade_opacity: (1.0 - blade_t) as f32,
            disc_opacity: (0.34 * disc_t) as f32,
            shutter_blur_angle_rad: blur as f32,
            blade_count: 2,
        }
    }

    /// Synthesizes physically consistent articulation for kinematic (non-JSBSim) flight profiles.
    #[must_use]
    pub fn from_kinematic(state: &VehicleState, controls: ControlInputs, elapsed_s: f64) -> Self {
        let speed_mps = state.velocity_ecef.length();
        let throttle = if controls.throttle.is_finite() {
            controls.throttle.clamp(0.0, 1.0)
        } else {
            0.65
        };
        let rpm = if speed_mps > 1.0 || throttle > 0.05 {
            750.0 + throttle * 1_850.0
        } else {
            0.0
        };
        let omega = rpm * TAU / 60.0;
        let prop_phase = (omega * elapsed_s.max(0.0)).rem_euclid(TAU);
        let beacon_phase = (elapsed_s.max(0.0) * 1.2 * TAU).rem_euclid(TAU);
        let aileron = controls.roll.clamp(-1.0, 1.0) * 0.34;
        Self {
            engine_running: rpm > 100.0,
            propeller_rpm: rpm,
            engine_rpm: rpm,
            propeller_phase_rad: prop_phase,
            propeller_thrust_n: throttle * 1_650.0,
            fuel_flow_gph: 2.0 + throttle * 7.5,
            left_aileron_rad: aileron,
            right_aileron_rad: -aileron,
            elevator_rad: -controls.pitch.clamp(-1.0, 1.0) * 0.42,
            rudder_rad: -controls.yaw.clamp(-1.0, 1.0) * 0.28,
            beacon_phase_rad: beacon_phase,
            beacon_flash_norm: beacon_pulse(beacon_phase, rpm > 100.0),
            ..Self::default()
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(super) struct ArticulationNodes {
    engine_running: Option<NodeId>,
    starter_norm: Option<NodeId>,
    engine_rpm: Option<NodeId>,
    propeller_rpm: Option<NodeId>,
    propeller_sense: Option<NodeId>,
    blade_angle_deg: Option<NodeId>,
    thrust_lbs: Option<NodeId>,
    map_pa: Option<NodeId>,
    fuel_flow_gph: Option<NodeId>,
    egt_deg_f: Option<NodeId>,
    cht_deg_f: Option<NodeId>,
    oil_temp_deg_f: Option<NodeId>,
    oil_pressure_psi: Option<NodeId>,
    flap_pos_deg: Option<NodeId>,
    flap_pos_norm: Option<NodeId>,
    left_aileron_rad: Option<NodeId>,
    right_aileron_rad: Option<NodeId>,
    elevator_rad: Option<NodeId>,
    rudder_rad: Option<NodeId>,
    steer_deg: Option<NodeId>,
    gear_wow: [Option<NodeId>; 3],
    gear_comp_ft: [Option<NodeId>; 3],
    wheel_speed_fps: [Option<NodeId>; 3],
}

impl ArticulationNodes {
    pub(super) fn resolve(fdm: &Fdm) -> Result<Self, BlockError> {
        let opt = |path: &str| fdm.node(path).map_err(fdm_error);
        Ok(Self {
            engine_running: opt("propulsion/engine[0]/set-running")?,
            starter_norm: opt("propulsion/engine[0]/starter-norm")?,
            engine_rpm: opt("propulsion/engine[0]/engine-rpm")?,
            propeller_rpm: opt("propulsion/engine[0]/propeller-rpm")?,
            propeller_sense: opt("propulsion/engine[0]/propeller-sense")?,
            blade_angle_deg: opt("propulsion/engine[0]/blade-angle")?,
            thrust_lbs: opt("propulsion/engine[0]/thrust-lbs")?,
            map_pa: opt("propulsion/engine[0]/map-pa")?,
            fuel_flow_gph: opt("propulsion/engine[0]/fuel-flow-rate-gph")?,
            egt_deg_f: opt("propulsion/engine[0]/egt-degF")?,
            cht_deg_f: opt("propulsion/engine[0]/cht-degF")?,
            oil_temp_deg_f: opt("propulsion/engine[0]/oil-temperature-degF")?,
            oil_pressure_psi: opt("propulsion/engine[0]/oil-pressure-psi")?,
            flap_pos_deg: opt("fcs/flap-pos-deg")?,
            flap_pos_norm: opt("fcs/flap-pos-norm")?,
            left_aileron_rad: opt("fcs/left-aileron-pos-rad")?,
            right_aileron_rad: opt("fcs/right-aileron-pos-rad")?,
            elevator_rad: opt("fcs/elevator-pos-rad")?,
            rudder_rad: opt("fcs/rudder-pos-rad")?,
            steer_deg: opt("fcs/steer-pos-deg")?,
            gear_wow: [
                opt("gear/unit[0]/WOW")?,
                opt("gear/unit[1]/WOW")?,
                opt("gear/unit[2]/WOW")?,
            ],
            gear_comp_ft: [
                opt("gear/unit[0]/compression-ft")?,
                opt("gear/unit[1]/compression-ft")?,
                opt("gear/unit[2]/compression-ft")?,
            ],
            wheel_speed_fps: [
                opt("gear/unit[0]/wheel-speed-fps")?,
                opt("gear/unit[1]/wheel-speed-fps")?,
                opt("gear/unit[2]/wheel-speed-fps")?,
            ],
        })
    }

    pub(super) fn sample(
        &self,
        fdm: &mut Fdm,
        prev: &AirframeArticulation,
        dt_s: f64,
    ) -> Result<AirframeArticulation, BlockError> {
        let mut rd = |n: Option<NodeId>, fallback: f64| -> Result<f64, BlockError> {
            match n {
                Some(id) => fdm.read_node(id).map_err(fdm_error),
                None => Ok(fallback),
            }
        };
        let running = rd(self.engine_running, 0.0)? != 0.0;
        let starter = rd(self.starter_norm, 0.0)? > 0.1;
        let prop_rpm = rd(self.propeller_rpm, 0.0)?;
        let eng_rpm = rd(self.engine_rpm, prop_rpm)?;
        let raw_sense = rd(self.propeller_sense, 1.0)?;
        let sense = if raw_sense < 0.0 { -1.0 } else { 1.0 };
        let blade_deg = rd(self.propeller_blade_angle_deg(), DEFAULT_BLADE_ANGLE_DEG)?;
        let thrust_n = rd(self.thrust_lbs, 0.0)? * NEWTONS_PER_POUND_FORCE;
        let map_pa = rd(self.map_pa, 101_325.0)?;
        let ff_gph = rd(self.fuel_flow_gph, 0.0)?;
        let egt = rd(self.egt_deg_f, 70.0)?;
        let cht = rd(self.cht_deg_f, 70.0)?;
        let oil_t = rd(self.oil_temp_deg_f, 70.0)?;
        let oil_p = rd(self.oil_pressure_psi, 0.0)?;
        let step_dt = if dt_s.is_finite() && dt_s > 0.0 {
            dt_s
        } else {
            0.0
        };
        let omega = prop_rpm * TAU / 60.0;
        let prop_phase = (prev.propeller_phase_rad + sense * omega * step_dt).rem_euclid(TAU);
        let beacon_phase = (prev.beacon_phase_rad + 1.2 * TAU * step_dt).rem_euclid(TAU);
        let surfaces = self.sample_surfaces_and_gear(fdm, prev, step_dt)?;
        Ok(AirframeArticulation {
            engine_running: running,
            starter_engaged: starter,
            engine_rpm: eng_rpm,
            propeller_rpm: prop_rpm,
            propeller_sense: sense,
            propeller_blade_angle_deg: if blade_deg.abs() > 0.1 {
                blade_deg
            } else {
                DEFAULT_BLADE_ANGLE_DEG
            },
            propeller_phase_rad: prop_phase,
            propeller_thrust_n: thrust_n,
            manifold_pressure_pa: map_pa,
            fuel_flow_gph: ff_gph,
            egt_deg_f: egt,
            cht_deg_f: cht,
            oil_temp_deg_f: oil_t,
            oil_pressure_psi: oil_p,
            engine_vibration_rad: piston_mount_vibration(eng_rpm, prop_phase, running || starter),
            beacon_phase_rad: beacon_phase,
            beacon_flash_norm: beacon_pulse(beacon_phase, running || starter),
            ..surfaces
        })
    }

    const fn propeller_blade_angle_deg(&self) -> Option<NodeId> {
        self.blade_angle_deg
    }

    fn sample_surfaces_and_gear(
        &self,
        fdm: &mut Fdm,
        prev: &AirframeArticulation,
        dt_s: f64,
    ) -> Result<AirframeArticulation, BlockError> {
        let mut rd = |n: Option<NodeId>| -> Result<f64, BlockError> {
            match n {
                Some(id) => fdm.read_node(id).map_err(fdm_error),
                None => Ok(0.0),
            }
        };
        let flap_deg_raw = rd(self.flap_pos_deg)?;
        let flap_norm = rd(self.flap_pos_norm)?;
        let flap_deg = if flap_deg_raw.abs() > 1e-4 {
            flap_deg_raw
        } else {
            flap_norm * DEFAULT_MAX_FLAP_DEG
        };
        let left_ail = rd(self.left_aileron_rad)?;
        let right_raw = rd(self.right_aileron_rad)?;
        let right_ail = if self.right_aileron_rad.is_some() && right_raw.abs() > 1e-6 {
            right_raw
        } else {
            -left_ail
        };
        let elev = rd(self.elevator_rad)?;
        let rud = rd(self.rudder_rad)?;
        let [w0, w1, w2] = self.gear_wow;
        let wow = [rd(w0)? != 0.0, rd(w1)? != 0.0, rd(w2)? != 0.0];
        let [nose_wow, _, _] = wow;
        let [c0, c1, c2] = self.gear_comp_ft;
        let comp_m = [
            (rd(c0)? * METRES_PER_FOOT).clamp(0.0, 0.35),
            (rd(c1)? * METRES_PER_FOOT).clamp(0.0, 0.35),
            (rd(c2)? * METRES_PER_FOOT).clamp(0.0, 0.35),
        ];
        let [s0, s1, s2] = self.wheel_speed_fps;
        let [p0, p1, p2] = prev.wheel_phase_rad;
        let wheel_phase = [
            (p0 + (rd(s0)? / NOSE_WHEEL_RADIUS_FT) * dt_s).rem_euclid(TAU),
            (p1 + (rd(s1)? / MAIN_WHEEL_RADIUS_FT) * dt_s).rem_euclid(TAU),
            (p2 + (rd(s2)? / MAIN_WHEEL_RADIUS_FT) * dt_s).rem_euclid(TAU),
        ];
        let steer_rad = if let Some(st) = self.steer_deg {
            rd(Some(st))?.to_radians()
        } else if nose_wow {
            0.35 * rud
        } else {
            0.0
        };
        Ok(AirframeArticulation {
            flap_deg,
            left_aileron_rad: left_ail,
            right_aileron_rad: right_ail,
            elevator_rad: elev,
            rudder_rad: rud,
            nosewheel_steer_rad: steer_rad,
            gear_wow: wow,
            gear_compression_m: comp_m,
            wheel_phase_rad: wheel_phase,
            ..AirframeArticulation::default()
        })
    }
}

fn piston_mount_vibration(engine_rpm: f64, crank_phase: f64, active: bool) -> [f64; 3] {
    if !active || engine_rpm.abs() < 15.0 {
        return [0.0; 3];
    }
    let abs_rpm = engine_rpm.abs();
    // 4-cylinder 4-stroke flat-four fires twice per crank revolution; rough below 650 RPM.
    let cranking_gain = 1.0 - smoothstep(280.0, 950.0, abs_rpm);
    let amp_roll = 0.001_0 + 0.014 * cranking_gain;
    let firing = (2.0 * crank_phase).sin();
    let secondary = (4.0 * crank_phase + 0.7).cos() * 0.35;
    [
        amp_roll * (firing + secondary),
        amp_roll * 0.25 * (2.0 * crank_phase + 1.2).cos(),
        0.0,
    ]
}

fn beacon_pulse(phase_rad: f64, powered: bool) -> f64 {
    if !powered {
        return 0.0;
    }
    let p = (phase_rad.cos() * 0.5 + 0.5).powi(8);
    (0.12 + 0.88 * p).clamp(0.0, 1.0)
}

fn smoothstep(edge0: f64, edge1: f64, x: f64) -> f64 {
    let span = (edge1 - edge0).max(1e-9);
    let t = ((x - edge0) / span).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

use std::f64::consts::{PI, TAU};

use crate::{ControlInputs, VehicleState};

mod nodes;

pub(super) use nodes::ArticulationNodes;

use crate::instrument_panel::{InstrumentNeedles, SecondEngineGauges};

const METRES_PER_FOOT: f64 = 0.3048;
const KILOGRAMS_PER_POUND: f64 = 0.453_592_37;
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
    /// Starboard/right engine propeller shaft speed in RPM (mirrors `propeller_rpm` on single-engine aircraft).
    pub propeller_rpm_right: f64,
    /// Starboard/right propeller rotation direction (`+1.0` CW, `-1.0` CCW viewed from rear).
    pub propeller_sense_right: f64,
    /// Starboard/right propeller blade geometric pitch at the reference station (degrees).
    pub propeller_blade_angle_right_deg: f64,
    /// Deterministically integrated starboard/right propeller shaft angle in `[0, 2π)` radians.
    pub propeller_phase_right_rad: f64,
    /// Whether each propeller is feathered edge-on into the airstream `[left, right]`.
    pub propeller_feathered: [bool; 2],
    /// Left and right hydraulic coolant-radiator exit flap openings `[left, right]` in `0..=1`.
    pub coolant_flap_norm: [f64; 2],
    /// Left and right electric oil-cooler shutter openings `[left, right]` in `0..=1`.
    pub oil_shutter_norm: [f64; 2],
    /// Three-axis aerodynamic trim tab deflections `[aileron_trim, elevator_trim, rudder_trim]` in radians.
    pub trim_tab_rad: [f64; 3],
    /// Net propeller axial thrust in newtons.
    pub propeller_thrust_n: f64,
    /// Intake manifold absolute pressure in pascals.
    pub manifold_pressure_pa: f64,
    /// The second engine's gauges on a twin (mirrors the first on a single).
    pub engine_right_gauges: SecondEngineGauges,
    /// Instrument panel needles, drawn on airframes that model their panel.
    pub instrument_needles: InstrumentNeedles,
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
    /// Normalized landing-gear deployment (`1.0` fully extended, `0.0` fully retracted).
    pub gear_pos_norm: f64,
    /// Landing-gear strut compression `[nose, left_main, right_main]` in metres.
    pub gear_compression_m: [f64; 3],
    /// Integrated rolling wheel angles `[nose, left_main, right_main]` in `[0, 2π)` radians.
    pub wheel_phase_rad: [f64; 3],
    /// Anti-collision red fin beacon normalized intensity `[0, 1]`.
    pub beacon_flash_norm: f64,
    /// Integrated electrical/beacon phase in `[0, 2π)` radians.
    pub beacon_phase_rad: f64,
    /// Normalized landing light illumination `[0, 1]` (`1.0` extended and lit, `0.0` off/stowed).
    pub landing_light_norm: f64,
    /// Current gross mass including remaining fuel and payload in kilograms (`0.0` if unknown).
    pub gross_mass_kg: f64,
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
            propeller_rpm_right: 0.0,
            propeller_sense_right: 1.0,
            propeller_blade_angle_right_deg: DEFAULT_BLADE_ANGLE_DEG,
            propeller_phase_right_rad: 0.0,
            propeller_feathered: [false; 2],
            coolant_flap_norm: [0.20; 2],
            oil_shutter_norm: [0.20; 2],
            trim_tab_rad: [0.0; 3],
            propeller_thrust_n: 0.0,
            manifold_pressure_pa: 101_325.0,
            engine_right_gauges: SecondEngineGauges::default(),
            instrument_needles: InstrumentNeedles::default(),
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
            gear_pos_norm: 1.0,
            gear_compression_m: [0.0; 3],
            wheel_phase_rad: [0.0; 3],
            beacon_flash_norm: 0.0,
            beacon_phase_rad: 0.0,
            landing_light_norm: 0.0,
            gross_mass_kg: 0.0,
        }
    }
}

impl AirframeArticulation {
    /// Computes the optical blade vs. blurred-disc state for the primary (or port) propeller.
    #[must_use]
    pub fn propeller_visual(&self, shutter_exposure_s: f64) -> PropellerVisualState {
        self.propeller_visual_for_engine(0, shutter_exposure_s)
    }

    /// Computes the optical blade vs. blurred-disc state for `engine_index` (`0` = port/single,
    /// `1` = starboard), handling stopped, running, and full-feathered (`85°`) states.
    #[must_use]
    pub fn propeller_visual_for_engine(
        &self,
        engine_index: usize,
        shutter_exposure_s: f64,
    ) -> PropellerVisualState {
        let [feath_0, feath_1] = self.propeller_feathered;
        let (rpm, phase, blade_deg, feathered) = if engine_index == 1 {
            (
                self.propeller_rpm_right,
                self.propeller_phase_right_rad,
                self.propeller_blade_angle_right_deg,
                feath_1,
            )
        } else {
            (
                self.propeller_rpm,
                self.propeller_phase_rad,
                self.propeller_blade_angle_deg,
                feath_0,
            )
        };
        if feathered {
            return PropellerVisualState {
                phase_rad: phase.rem_euclid(TAU) as f32,
                blade_angle_deg: 85.0,
                blade_opacity: 1.0,
                disc_opacity: 0.0,
                shutter_blur_angle_rad: 0.06,
                blade_count: 3,
            };
        }
        let abs_rpm = rpm.abs();
        // The disc fades in exactly as the solid blades fade out, so a turning
        // propeller is always drawn one way or the other.
        let blade_t = smoothstep(350.0, 900.0, abs_rpm);
        let disc_t = blade_t;
        let blur = ((abs_rpm * TAU / 60.0) * shutter_exposure_s.clamp(1.0 / 2_000.0, 1.0 / 24.0))
            .clamp(0.06, PI);
        PropellerVisualState {
            phase_rad: phase.rem_euclid(TAU) as f32,
            blade_angle_deg: blade_deg as f32,
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
        let prop_phase = ((rpm * TAU / 60.0) * elapsed_s.max(0.0)).rem_euclid(TAU);
        let beacon_phase = (elapsed_s.max(0.0) * 1.2 * TAU).rem_euclid(TAU);
        let aileron = controls.roll.clamp(-1.0, 1.0) * 0.34;
        Self {
            engine_running: rpm > 100.0,
            propeller_rpm: rpm,
            propeller_rpm_right: rpm,
            engine_rpm: rpm,
            propeller_sense: -1.0,
            propeller_sense_right: 1.0,
            propeller_phase_rad: prop_phase,
            propeller_phase_right_rad: (TAU - prop_phase).rem_euclid(TAU),
            propeller_thrust_n: throttle * 1_650.0,
            fuel_flow_gph: 2.0 + throttle * 7.5,
            left_aileron_rad: aileron,
            right_aileron_rad: -aileron,
            elevator_rad: -controls.pitch.clamp(-1.0, 1.0) * 0.42,
            rudder_rad: -controls.yaw.clamp(-1.0, 1.0) * 0.28,
            beacon_phase_rad: beacon_phase,
            beacon_flash_norm: beacon_pulse(beacon_phase, rpm > 100.0),
            landing_light_norm: f64::from(u8::from(rpm > 100.0)),
            ..Self::default()
        }
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
    (0.12 + 0.88 * (phase_rad.cos() * 0.5 + 0.5).powi(8)).clamp(0.0, 1.0)
}

fn smoothstep(edge0: f64, edge1: f64, x: f64) -> f64 {
    let t = ((x - edge0) / (edge1 - edge0).max(1e-9)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

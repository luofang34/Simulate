//! JSBSim property nodes an airframe's articulation is sampled from.

use std::f64::consts::TAU;

use jsbsim::{Fdm, NodeId};

use super::{
    AirframeArticulation, DEFAULT_BLADE_ANGLE_DEG, DEFAULT_MAX_FLAP_DEG, KILOGRAMS_PER_POUND,
    MAIN_WHEEL_RADIUS_FT, METRES_PER_FOOT, NEWTONS_PER_POUND_FORCE, NOSE_WHEEL_RADIUS_FT,
    beacon_pulse, piston_mount_vibration,
};
use crate::BlockError;
use crate::instrument_panel::SecondEngineGauges;
use crate::jsbsim_fdm::fdm_error;
use crate::jsbsim_fdm::system::SystemControls;

#[derive(Debug, Clone, Copy)]
pub(in crate::jsbsim_fdm) struct ArticulationNodes {
    engine_running: Option<NodeId>,
    starter_norm: Option<NodeId>,
    engine_rpm: Option<NodeId>,
    propeller_rpm: Option<NodeId>,
    propeller_sense: Option<NodeId>,
    blade_angle_deg: Option<NodeId>,
    propeller_1_rpm: Option<NodeId>,
    propeller_1_sense: Option<NodeId>,
    blade_1_angle_deg: Option<NodeId>,
    thrust_lbs: Option<NodeId>,
    engine_1_thrust_lbs: Option<NodeId>,
    map_pa: Option<NodeId>,
    map_1_pa: Option<NodeId>,
    engine_1_rpm: Option<NodeId>,
    fuel_flow_gph: Option<NodeId>,
    engine_1_fuel_flow_gph: Option<NodeId>,
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
    weight_lbs: Option<NodeId>,
    gear_pos_norm: Option<NodeId>,
    gear_wow: [Option<NodeId>; 3],
    gear_comp_ft: [Option<NodeId>; 3],
    wheel_speed_fps: [Option<NodeId>; 3],
}

impl ArticulationNodes {
    pub(in crate::jsbsim_fdm) fn resolve(fdm: &Fdm) -> Result<Self, BlockError> {
        let opt = |path: &str| fdm.node(path).map_err(fdm_error);
        let tri = |a: &str, b: &str, c: &str| -> Result<[Option<NodeId>; 3], BlockError> {
            Ok([opt(a)?, opt(b)?, opt(c)?])
        };
        Ok(Self {
            engine_running: opt("propulsion/engine[0]/set-running")?,
            starter_norm: opt("propulsion/engine[0]/starter-norm")?,
            engine_rpm: opt("propulsion/engine[0]/engine-rpm")?,
            propeller_rpm: opt("propulsion/engine[0]/propeller-rpm")?,
            propeller_sense: opt("propulsion/engine[0]/propeller-sense")?,
            blade_angle_deg: opt("propulsion/engine[0]/blade-angle")?,
            propeller_1_rpm: opt("propulsion/engine[1]/propeller-rpm")?,
            propeller_1_sense: opt("propulsion/engine[1]/propeller-sense")?,
            blade_1_angle_deg: opt("propulsion/engine[1]/blade-angle")?,
            thrust_lbs: opt("propulsion/engine[0]/thrust-lbs")?,
            engine_1_thrust_lbs: opt("propulsion/engine[1]/thrust-lbs")?,
            map_pa: opt("propulsion/engine[0]/map-pa")?,
            map_1_pa: opt("propulsion/engine[1]/map-pa")?,
            engine_1_rpm: opt("propulsion/engine[1]/engine-rpm")?,
            fuel_flow_gph: opt("propulsion/engine[0]/fuel-flow-rate-gph")?,
            engine_1_fuel_flow_gph: opt("propulsion/engine[1]/fuel-flow-rate-gph")?,
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
            weight_lbs: opt("inertia/weight-lbs")?,
            gear_pos_norm: opt("gear/gear-pos-norm")?,
            gear_wow: tri("gear/unit[0]/WOW", "gear/unit[1]/WOW", "gear/unit[2]/WOW")?,
            gear_comp_ft: tri(
                "gear/unit[0]/compression-ft",
                "gear/unit[1]/compression-ft",
                "gear/unit[2]/compression-ft",
            )?,
            wheel_speed_fps: tri(
                "gear/unit[0]/wheel-speed-fps",
                "gear/unit[1]/wheel-speed-fps",
                "gear/unit[2]/wheel-speed-fps",
            )?,
        })
    }

    pub(in crate::jsbsim_fdm) fn sample(
        &self,
        fdm: &mut Fdm,
        prev: &AirframeArticulation,
        controls: SystemControls,
        dt_s: f64,
    ) -> Result<AirframeArticulation, BlockError> {
        let step_dt = if dt_s.is_finite() && dt_s > 0.0 {
            dt_s
        } else {
            0.0
        };
        let mut out = self.sample_surfaces_and_gear(fdm, prev, step_dt)?;
        let mut rd = |n: Option<NodeId>, fallback: f64| -> Result<f64, BlockError> {
            n.map_or(Ok(fallback), |id| fdm.read_node(id).map_err(fdm_error))
        };
        let (running, starter) = (
            rd(self.engine_running, 0.0)? != 0.0,
            rd(self.starter_norm, 0.0)? > 0.1,
        );
        let [feath_l, feath_r] = controls.prop_feather;
        let prop_rpm = if feath_l {
            0.0
        } else {
            rd(self.propeller_rpm, 0.0)?
        };
        let prop_rpm_r = if feath_r {
            0.0
        } else {
            rd(self.propeller_1_rpm, prop_rpm)?
        };
        let eng_rpm = rd(self.engine_rpm, prop_rpm)?;
        let sign = |v: f64| if v < 0.0 { -1.0 } else { 1.0 };
        let (sense, sense_r) = (
            sign(rd(self.propeller_sense, 1.0)?),
            sign(rd(self.propeller_1_sense, -1.0)?),
        );
        let raw_blade_l = rd(self.blade_angle_deg, DEFAULT_BLADE_ANGLE_DEG)?.max(16.0);
        let raw_blade_r = rd(self.blade_1_angle_deg, raw_blade_l)?.max(16.0);
        let step_phase =
            |p: f64, s: f64, rpm: f64| (p + s * (rpm * TAU / 60.0) * step_dt).rem_euclid(TAU);
        let prop_phase = step_phase(prev.propeller_phase_rad, sense, prop_rpm);
        let prop_phase_r = step_phase(prev.propeller_phase_right_rad, sense_r, prop_rpm_r);
        let beacon_phase = (prev.beacon_phase_rad + 1.2 * TAU * step_dt).rem_euclid(TAU);
        out.engine_running = running;
        out.starter_engaged = starter;
        out.engine_rpm = eng_rpm;
        out.propeller_rpm = prop_rpm;
        out.propeller_sense = sense;
        out.propeller_blade_angle_deg = if feath_l { 85.0 } else { raw_blade_l };
        out.propeller_phase_rad = prop_phase;
        out.propeller_rpm_right = prop_rpm_r;
        out.propeller_sense_right = sense_r;
        out.propeller_blade_angle_right_deg = if feath_r { 85.0 } else { raw_blade_r };
        out.propeller_phase_right_rad = prop_phase_r;
        out.propeller_feathered = controls.prop_feather;
        out.coolant_flap_norm = controls.coolant_flap_norm;
        out.oil_shutter_norm = controls.oil_shutter_norm;
        out.trim_tab_rad = controls.trim_norm.map(|t| t * 0.28);
        self.sample_engine_readouts(&mut rd, &mut out, eng_rpm)?;
        out.engine_vibration_rad = piston_mount_vibration(eng_rpm, prop_phase, running || starter);
        out.beacon_phase_rad = beacon_phase;
        out.beacon_flash_norm = beacon_pulse(beacon_phase, running || starter);
        out.landing_light_norm = if controls.landing_light && (running || starter) {
            out.gear_pos_norm.clamp(0.0, 1.0)
        } else {
            0.0
        };
        Ok(out)
    }

    /// Thrust, fuel flow, temperatures, pressures and both engines' gauges.
    fn sample_engine_readouts(
        &self,
        rd: &mut impl FnMut(Option<NodeId>, f64) -> Result<f64, BlockError>,
        out: &mut AirframeArticulation,
        eng_rpm: f64,
    ) -> Result<(), BlockError> {
        out.propeller_thrust_n = (rd(self.thrust_lbs, 0.0)? + rd(self.engine_1_thrust_lbs, 0.0)?)
            * NEWTONS_PER_POUND_FORCE;
        out.manifold_pressure_pa = rd(self.map_pa, 101_325.0)?;
        let (rpm_1, map_1) = (
            rd(self.engine_1_rpm, eng_rpm)?,
            rd(self.map_1_pa, out.manifold_pressure_pa)?,
        );
        out.engine_right_gauges = SecondEngineGauges {
            engine_rpm: rpm_1,
            manifold_pressure_pa: map_1,
        };
        out.fuel_flow_gph = rd(self.fuel_flow_gph, 0.0)? + rd(self.engine_1_fuel_flow_gph, 0.0)?;
        out.egt_deg_f = rd(self.egt_deg_f, 70.0)?;
        out.cht_deg_f = rd(self.cht_deg_f, 70.0)?;
        out.oil_temp_deg_f = rd(self.oil_temp_deg_f, 70.0)?;
        out.oil_pressure_psi = rd(self.oil_pressure_psi, 0.0)?;
        out.gross_mass_kg = rd(self.weight_lbs, 0.0)?.max(0.0) * KILOGRAMS_PER_POUND;
        Ok(())
    }

    fn sample_surfaces_and_gear(
        &self,
        fdm: &mut Fdm,
        prev: &AirframeArticulation,
        dt_s: f64,
    ) -> Result<AirframeArticulation, BlockError> {
        let mut rd = |n: Option<NodeId>| -> Result<f64, BlockError> {
            n.map_or(Ok(0.0), |id| fdm.read_node(id).map_err(fdm_error))
        };
        let (flap_deg_raw, flap_norm) = (rd(self.flap_pos_deg)?, rd(self.flap_pos_norm)?);
        let flap_deg = if flap_deg_raw.abs() > 1e-4 {
            flap_deg_raw
        } else {
            flap_norm * DEFAULT_MAX_FLAP_DEG
        };
        let (left_ail, right_raw) = (rd(self.left_aileron_rad)?, rd(self.right_aileron_rad)?);
        let right_ail = if self.right_aileron_rad.is_some() && right_raw.abs() > 1e-6 {
            right_raw
        } else {
            -left_ail
        };
        let (elev, rud) = (rd(self.elevator_rad)?, rd(self.rudder_rad)?);
        let [w0, w1, w2] = self.gear_wow;
        let wow = [rd(w0)? != 0.0, rd(w1)? != 0.0, rd(w2)? != 0.0];
        let gear_pos_norm = match self.gear_pos_norm {
            Some(id) if !wow.into_iter().any(|w| w) => rd(Some(id))?.clamp(0.0, 1.0),
            _ => 1.0,
        };
        let [c0, c1, c2] = self.gear_comp_ft;
        let comp_m =
            [c0, c1, c2].map(|c| (rd(c).unwrap_or(0.0) * METRES_PER_FOOT).clamp(0.0, 0.35));
        let [s0, s1, s2] = self.wheel_speed_fps;
        let [p0, p1, p2] = prev.wheel_phase_rad;
        let wheel_phase = [
            (p0 + (rd(s0)? / NOSE_WHEEL_RADIUS_FT) * dt_s).rem_euclid(TAU),
            (p1 + (rd(s1)? / MAIN_WHEEL_RADIUS_FT) * dt_s).rem_euclid(TAU),
            (p2 + (rd(s2)? / MAIN_WHEEL_RADIUS_FT) * dt_s).rem_euclid(TAU),
        ];
        let steer_rad = match (self.steer_deg, wow[0]) {
            (Some(st), _) => rd(Some(st))?.to_radians(),
            (None, true) => 0.35 * rud,
            (None, false) => 0.0,
        };
        Ok(AirframeArticulation {
            flap_deg,
            left_aileron_rad: left_ail,
            right_aileron_rad: right_ail,
            elevator_rad: elev,
            rudder_rad: rud,
            nosewheel_steer_rad: steer_rad,
            gear_wow: wow,
            gear_pos_norm,
            gear_compression_m: comp_m,
            wheel_phase_rad: wheel_phase,
            ..AirframeArticulation::default()
        })
    }
}

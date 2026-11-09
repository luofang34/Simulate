//! Power-off glide polar sweep and the reference geometry that turns the
//! sweep into lift and drag coefficients.

use jsbsim::Fdm;

use crate::PackError;
use crate::calibration::cruise::{
    configure_mass_and_overrides, fdm_err, set_if_exists, step_wings_level,
};
use crate::calibration::results::{GlideSample, ReferenceGeometry};
use crate::calibration::solver::{DT_S, FPS_PER_KT, ParameterOverrides};

const M2_PER_FT2: f64 = 0.092_903_04;
const M_PER_FT: f64 = 0.3048;

/// Flies the idle-power glide at four speeds between `1.21` and `1.43 Vs1`.
pub(super) fn sweep_glide_polar(
    fdm: &mut Fdm,
    gross_mass_kg: f64,
    vs1_kcas: f64,
    base_non_payload_lbs: f64,
) -> Result<Vec<GlideSample>, PackError> {
    configure_mass_and_overrides(
        fdm,
        gross_mass_kg,
        base_non_payload_lbs,
        ParameterOverrides::default(),
    )?;
    let mut samples = Vec::with_capacity(4);
    for ratio in [1.21, 1.28, 1.35, 1.43] {
        let target_kcas = (vs1_kcas * ratio).max(45.0);
        let (kcas, lift_to_drag) = fly_glide_point(fdm, target_kcas)?;
        samples.push(GlideSample { kcas, lift_to_drag });
    }
    Ok(samples)
}

/// `(KCAS, L/D)` of the flattest glide in `samples`. The airspeed envelope
/// needs a best-glide speed even when no sample beats `L/D = 1`, so that case
/// keeps `1.28 Vs1` and `L/D = 1`.
pub(super) fn best_glide(samples: &[GlideSample], vs1_kcas: f64) -> (f64, f64) {
    samples
        .iter()
        .fold((vs1_kcas * 1.28, 1.0_f64), |best, sample| {
            if sample.lift_to_drag > best.1 {
                (sample.kcas, sample.lift_to_drag)
            } else {
                best
            }
        })
}

/// Reads the wing reference area and span the flight model normalises its
/// coefficients with.
pub(super) fn measure_reference_geometry(fdm: &mut Fdm) -> Result<ReferenceGeometry, PackError> {
    Ok(ReferenceGeometry {
        wing_area_m2: fdm.get_property("metrics/Sw-sqft").map_err(fdm_err)? * M2_PER_FT2,
        wing_span_m: fdm.get_property("metrics/bw-ft").map_err(fdm_err)? * M_PER_FT,
        engine_count: crate::calibration::engines::engine_count(fdm)?,
    })
}

fn fly_glide_point(fdm: &mut Fdm, target_kcas: f64) -> Result<(f64, f64), PackError> {
    for (prop, val) in [
        ("ic/h-sl-ft", 1_500.0),
        ("ic/terrain-elevation-ft", -1_000.0),
        ("ic/vt-fps", target_kcas * FPS_PER_KT),
        ("ic/alpha-deg", 4.5),
        ("ic/theta-deg", -2.0),
        ("ic/beta-deg", 0.0),
        ("fcs/throttle-cmd-norm", 0.0),
    ] {
        fdm.set_property(prop, val).map_err(fdm_err)?;
    }
    set_if_exists(fdm, "fcs/flap-cmd-norm", 0.0)?;
    set_if_exists(fdm, "fcs/flap-pos-norm", 0.0)?;
    set_if_exists(fdm, "propulsion/magneto_cmd", 3.0)?;
    set_if_exists(fdm, "propulsion/set-running", -1.0)?;
    fdm.run_ic().map_err(fdm_err)?;
    let mut prev_vc = fdm.get_property("velocities/vc-kts").map_err(fdm_err)?;
    let mut int_err = 0.0_f64;
    let mut h_start = 1_500.0_f64;
    let (mut vc_sum, mut vt_sum) = (0.0, 0.0);
    let mut samples = 0_u32;
    for step in 0..3_600_u32 {
        let vc = fdm.get_property("velocities/vc-kts").map_err(fdm_err)?;
        let vdot = (vc - prev_vc) / DT_S;
        prev_vc = vc;
        let err = vc - target_kcas;
        int_err = (int_err + err * DT_S).clamp(-35.0, 35.0);
        let theta = fdm.get_property("attitude/theta-rad").map_err(fdm_err)?;
        let q = fdm.get_property("velocities/q-rad_sec").map_err(fdm_err)?;
        let wanted = (-0.06 + 0.018 * err + 0.012 * int_err + 0.008 * vdot).clamp(-0.30, 0.25);
        let nose_up = (wanted - theta) * 2.5 - 0.55 * q;
        set_if_exists(fdm, "fcs/flap-pos-norm", 0.0)?;
        step_wings_level(fdm, nose_up, 0.0, 1.0)?;
        if step == 2_880 {
            h_start = fdm.get_property("position/h-sl-ft").map_err(fdm_err)?;
        }
        if step > 2_880 {
            vc_sum += vc;
            vt_sum += fdm.get_property("velocities/vt-fps").map_err(fdm_err)?;
            samples = samples.wrapping_add(1);
        }
    }
    let h_end = fdm.get_property("position/h-sl-ft").map_err(fdm_err)?;
    let n = f64::from(samples.max(1));
    let h_dot_fps = (h_end - h_start) / (n * DT_S);
    let mean_vt_fps = (vt_sum / n).max(30.0);
    let gamma_rad = (h_dot_fps / mean_vt_fps).clamp(-0.8, -1e-4).asin();
    let ld = (-gamma_rad).cos() / (-gamma_rad).sin();
    Ok((vc_sum / n, ld))
}

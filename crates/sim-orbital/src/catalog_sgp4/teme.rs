//! Pure-`libm` `TEME` (True Equator, Mean Equinox) to `ICRF` / `J2000` coordinate transforms.
//!
//! Uses IAU-1976 Lieske precession, IAU-1980 106-term nutation, and the IAU-1994
//! equation of the equinoxes so SGP4 states enter `sim-spacetime` in SI `IcrfInertial`.

use glam::{DMat3, DVec3};
use sim_spacetime::TdbEpoch;

use super::nutation::{ARCSEC_TO_RAD, mean_obliquity_iau1980, nutation_iau1980};

const SEC_PER_JULIAN_CENTURY: f64 = 36_525.0 * 86_400.0;
const KM_TO_M: f64 = 1_000.0;

fn rot1(angle_rad: f64) -> DMat3 {
    let (s, c) = libm::sincos(angle_rad);
    DMat3::from_cols(
        DVec3::new(1.0, 0.0, 0.0),
        DVec3::new(0.0, c, -s),
        DVec3::new(0.0, s, c),
    )
}

fn rot2(angle_rad: f64) -> DMat3 {
    let (s, c) = libm::sincos(angle_rad);
    DMat3::from_cols(
        DVec3::new(c, 0.0, s),
        DVec3::new(0.0, 1.0, 0.0),
        DVec3::new(-s, 0.0, c),
    )
}

fn rot3(angle_rad: f64) -> DMat3 {
    let (s, c) = libm::sincos(angle_rad);
    DMat3::from_cols(
        DVec3::new(c, -s, 0.0),
        DVec3::new(s, c, 0.0),
        DVec3::new(0.0, 0.0, 1.0),
    )
}

fn precession_j2000_to_mod(t_cy: f64) -> DMat3 {
    let zeta = (2306.2181 + (0.30188 + 0.017998 * t_cy) * t_cy) * t_cy * ARCSEC_TO_RAD;
    let theta = (2004.3109 + (-0.42665 - 0.041833 * t_cy) * t_cy) * t_cy * ARCSEC_TO_RAD;
    let z = (2306.2181 + (1.09468 + 0.018203 * t_cy) * t_cy) * t_cy * ARCSEC_TO_RAD;
    rot3(-z) * rot2(theta) * rot3(-zeta)
}

/// Computes the orthonormal rotation matrix $Q_{\text{TEME}\to\text{ICRF}}$ at `epoch`.
///
/// Maps position/velocity column vectors expressed in True Equator, Mean Equinox (`TEME`)
/// axes into Earth-centered `J2000` / `ICRF` inertial axes (`r_icrf = Q * r_teme`).
#[must_use]
pub fn teme_to_icrf_rotation(epoch: TdbEpoch) -> DMat3 {
    let (whole_s, sub_s) = epoch.split_j2000_seconds();
    let t_cy = (whole_s + sub_s) / SEC_PER_JULIAN_CENTURY;
    let prec = precession_j2000_to_mod(t_cy);
    let eps_mean = mean_obliquity_iau1980(t_cy);
    let (dpsi, deps, omega_moon) = nutation_iau1980(t_cy);
    let eps_true = eps_mean + deps;
    let nut = rot1(-eps_true) * rot3(-dpsi) * rot1(eps_mean);
    let eq_eq = dpsi * libm::cos(eps_mean)
        + ARCSEC_TO_RAD
            * (0.00264 * libm::sin(omega_moon) + 0.000063 * libm::sin(2.0 * omega_moon));
    prec.transpose() * nut.transpose() * rot3(-eq_eq)
}

pub(super) fn teme_km_to_icrf_si(
    epoch: TdbEpoch,
    pos_km: DVec3,
    vel_km_s: DVec3,
) -> (DVec3, DVec3) {
    let q = teme_to_icrf_rotation(epoch);
    (q * (pos_km * KM_TO_M), q * (vel_km_s * KM_TO_M))
}

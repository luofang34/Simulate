use core::f64::consts::PI;

use glam::DVec3;
use sim_analysis_contract::{AntennaSpec, CosinePowerPatchSpec};
use sim_spacetime::SPEED_OF_LIGHT_M_S;

use super::dish;
use crate::signal::AntennaPattern;

fn direction_at(theta_rad: f64) -> DVec3 {
    DVec3::new(theta_rad.sin(), 0.0, theta_rad.cos())
}

#[test]
fn parabolic_boresight_gain_is_eta_pi_d_over_lambda_squared() {
    let f = 8.42e9;
    let lambda = SPEED_OF_LIGHT_M_S / f;
    let antenna = AntennaSpec::ParabolicDish(dish(34.0, 0.7));
    let expected_db = 10.0 * (0.7 * (PI * 34.0 / lambda).powi(2)).log10();
    assert!((antenna.gain_dbi(DVec3::Z, f) - expected_db).abs() < 1e-9);
    assert!((antenna.peak_gain_dbi(f) - expected_db).abs() < 1e-9);
}

#[test]
fn parabolic_half_power_beamwidth_is_seventy_lambda_over_d_degrees() {
    let f = 2.2e9;
    let diameter_m = 3.0;
    let antenna = AntennaSpec::ParabolicDish(dish(diameter_m, 0.6));
    let peak = antenna.peak_gain_dbi(f);
    let (mut lo, mut hi) = (0.0_f64, 0.5_f64);
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if antenna.gain_dbi(direction_at(mid), f) > peak - 3.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let full_width_deg = (2.0 * lo).to_degrees();
    let expected_deg = 70.0 * (SPEED_OF_LIGHT_M_S / f) / diameter_m;
    assert!((full_width_deg - expected_deg).abs() < 1e-9 * expected_deg);
}

#[test]
fn parabolic_pattern_rolls_off_then_holds_the_stated_floor() {
    let f = 8.42e9;
    let antenna = AntennaSpec::ParabolicDish(dish(34.0, 0.7));
    let peak = antenna.peak_gain_dbi(f);
    let mut previous = peak;
    for k in 1..60 {
        let g = antenna.gain_dbi(direction_at(f64::from(k) * 1.0e-4), f);
        assert!(g <= previous + 1e-12);
        previous = g;
    }
    assert!((antenna.gain_dbi(direction_at(1.0), f) - (peak - 30.0)).abs() < 1e-12);
    assert!((antenna.gain_dbi(-DVec3::Z, f) - (peak - 30.0)).abs() < 1e-12);
}

#[test]
fn patch_follows_cosine_power_and_floors_behind_the_plane() {
    let patch = AntennaSpec::CosinePowerPatch(CosinePowerPatchSpec {
        peak_gain_dbi: 7.0,
        cosine_exponent: 2.0,
        back_hemisphere_floor_dbi: -15.0,
    });
    let theta = 40.0_f64.to_radians();
    let expected = 7.0 + 20.0 * theta.cos().log10();
    assert!((patch.gain_dbi(direction_at(theta), 2.2e9) - expected).abs() < 1e-9);
    assert!((patch.gain_dbi(DVec3::Z, 2.2e9) - 7.0).abs() < 1e-12);
    assert_eq!(patch.gain_dbi(-DVec3::Z, 2.2e9), -15.0);
    assert_eq!(
        patch.gain_dbi(direction_at(89.9_f64.to_radians()), 2.2e9),
        -15.0
    );
}

#[test]
fn isotropic_needs_no_orientation_and_directional_antennas_do() {
    assert!(!AntennaSpec::Isotropic.requires_orientation());
    assert!(AntennaSpec::ParabolicDish(dish(1.0, 0.5)).requires_orientation());
    assert_eq!(AntennaSpec::Isotropic.gain_dbi(DVec3::X, 1.0e9), 0.0);
}

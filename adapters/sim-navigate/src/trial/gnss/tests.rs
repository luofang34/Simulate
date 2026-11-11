use glam::DVec3;
use sim_core::VehicleState;
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic};

use super::{BaroAltimeter, GnssReceiver};
use crate::trial::{BaroSpec, GnssSpec};

fn parked() -> VehicleState {
    let ground = Geodetic {
        lat_rad: 39.0_f64.to_radians(),
        lon_rad: (-75.5_f64).to_radians(),
        height_m: 100.0,
    };
    VehicleState::level_flight(&ground, 0.0, 0.0)
}

fn north_east_down_error(truth: &VehicleState, fix: &super::GnssFix) -> DVec3 {
    let p = fix.position;
    let measured = sim_geo::geodetic_to_ecef(&Geodetic {
        lat_rad: p.latitude_rad,
        lon_rad: p.longitude_rad,
        height_m: p.altitude_m,
    });
    LocalFrame::at(&ecef_to_geodetic(truth.position_ecef))
        .ecef_to_ned_vector(measured - truth.position_ecef)
}

#[test]
fn the_declared_covariance_is_the_simulated_spread() {
    // A short correlation time makes successive fixes nearly independent,
    // so the sample spread estimates the marginal variance.
    let spec = GnssSpec {
        correlation_time_s: 0.5,
        ..GnssSpec::default()
    };
    let truth = parked();
    let mut receiver = GnssReceiver::new(spec, 11).unwrap();
    let count = 20_000;
    let mut squares = DVec3::ZERO;
    let mut declared = DVec3::ZERO;
    for _ in 0..count {
        let fix = receiver.measure(&truth);
        let error = north_east_down_error(&truth, &fix);
        squares += error * error;
        let (n, e, d) = fix.position_cov.diagonal();
        declared = DVec3::new(n, e, d);
    }
    let measured = squares / f64::from(count);
    let ratio = measured / declared;
    assert!(
        ratio.min_element() > 0.95 && ratio.max_element() < 1.05,
        "{measured} vs {declared}"
    );
}

#[test]
fn the_correlated_error_decays_over_its_correlation_time() {
    let spec = GnssSpec {
        horizontal_white_sigma_m: 1e-3,
        vertical_white_sigma_m: 1e-3,
        correlation_time_s: 20.0,
        ..GnssSpec::default()
    };
    let truth = parked();
    let mut receiver = GnssReceiver::new(spec, 5).unwrap();
    let north: Vec<f64> = (0..40_000)
        .map(|_| north_east_down_error(&truth, &receiver.measure(&truth)).x)
        .collect();
    let lag = 20;
    let product: f64 = north.iter().zip(&north[lag..]).map(|(a, b)| a * b).sum();
    let variance: f64 = north.iter().map(|a| a * a).sum();
    let correlation = product / variance;
    assert!(
        (correlation - (-1.0_f64).exp()).abs() < 0.08,
        "{correlation}"
    );
}

#[test]
fn the_baro_bias_and_noise_have_their_declared_spread() {
    let spec = BaroSpec {
        bias_correlation_time_s: 0.1,
        ..BaroSpec::default()
    };
    let truth = parked();
    let mut altimeter = BaroAltimeter::new(spec, 3).unwrap();
    let count = 20_000;
    let squares: f64 = (0..count)
        .map(|_| (altimeter.measure(&truth).altitude_m - 100.0).powi(2))
        .sum();
    let expected = spec.white_sigma_m.powi(2) + spec.bias_sigma_m.powi(2);
    let ratio = squares / f64::from(count) / expected;
    assert!((0.95..1.05).contains(&ratio), "{ratio}");
}

use glam::DVec3;
use navigate_onboard::{ErrorBudget, SensorData};
use sim_core::VehicleState;
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};

use super::{StaticPort, receiver};
use crate::trial::onboard::{altimeter_budget, baro_altitude_data, gnss_fix_data};
use crate::trial::{BaroSpec, GnssSpec, RoutePoint, TrialRequest, request_from_route};

const HEIGHT_M: f64 = 1_000.0;

fn request() -> TrialRequest {
    let point = |lon_deg| RoutePoint {
        ident: None,
        lat_deg: 39.0,
        lon_deg,
        altitude_m: HEIGHT_M,
        speed_m_s: None,
    };
    request_from_route(&[point(-75.6), point(-75.5)], None).unwrap()
}

const LAT_RAD: f64 = 0.680_678_408_277_788_5; // 39.0°
const LON_RAD: f64 = -1.317_723_585_255_718_8; // -75.5°

fn parked() -> VehicleState {
    parked_at(LAT_RAD, LON_RAD)
}

fn parked_at(lat_rad: f64, lon_rad: f64) -> VehicleState {
    let ground = Geodetic {
        lat_rad,
        lon_rad,
        height_m: HEIGHT_M,
    };
    VehicleState::level_flight(&ground, 0.0, 0.0)
}

#[test]
fn the_host_reads_the_static_port_with_the_declared_metre_spread() {
    // A short correlation time makes successive offsets nearly
    // independent, so the sample spread estimates the marginal variance.
    let mut request = request();
    request.baro = BaroSpec {
        bias_correlation_time_s: 0.1,
        ..BaroSpec::default()
    };
    let truth = parked();
    let mut port = StaticPort::new(&request).unwrap();
    let count = 20_000;
    let squares: f64 = (0..count)
        .map(|k| {
            let sample = port.sense(&truth, f64::from(k) * 0.2, 0.2).unwrap();
            let SensorData::BaroAltitude { altitude_m } =
                baro_altitude_data(&sample, LAT_RAD, LON_RAD).unwrap()
            else {
                panic!("not a barometric altitude");
            };
            (altitude_m - HEIGHT_M).powi(2)
        })
        .sum();
    let ErrorBudget::Barometer {
        white_sigma_m,
        bias,
    } = altimeter_budget(&request.baro, HEIGHT_M).unwrap()
    else {
        panic!("not a barometer budget");
    };
    let expected = white_sigma_m.powi(2) + bias.sigma().powi(2);
    let ratio = squares / f64::from(count) / expected;
    assert!((0.95..1.05).contains(&ratio), "{ratio}");
}

#[test]
fn an_ideal_static_port_reads_the_true_height() {
    let mut request = request();
    request.baro.white_sigma_pa = 1e-9;
    request.baro.bias_sigma_pa = 0.0;
    let mut port = StaticPort::new(&request).unwrap();
    let sample = port.sense(&parked(), 0.0, 0.2).unwrap();
    let SensorData::BaroAltitude { altitude_m } =
        baro_altitude_data(&sample, LAT_RAD, LON_RAD).unwrap()
    else {
        panic!("not a barometric altitude");
    };
    assert!((altitude_m - HEIGHT_M).abs() < 1e-6, "{altitude_m}");
}

#[test]
fn the_host_recovers_ellipsoidal_height_where_the_geoid_is_far_from_the_ellipsoid() {
    // South of Sri Lanka the geoid lies about 100 m below the ellipsoid, so
    // a conversion that dropped the undulation on either side would miss by
    // that much.
    let (lat_rad, lon_rad) = (4.0_f64.to_radians(), 79.0_f64.to_radians());
    let undulation_m = sim_geo::egm96_undulation_m(lat_rad, lon_rad);
    assert!(undulation_m < -80.0, "{undulation_m}");
    let mut request = request();
    request.baro.white_sigma_pa = 1e-9;
    request.baro.bias_sigma_pa = 0.0;
    let mut port = StaticPort::new(&request).unwrap();
    let sample = port.sense(&parked_at(lat_rad, lon_rad), 0.0, 0.2).unwrap();
    let SensorData::BaroAltitude { altitude_m } =
        baro_altitude_data(&sample, lat_rad, lon_rad).unwrap()
    else {
        panic!("not a barometric altitude");
    };
    assert!((altitude_m - HEIGHT_M).abs() < 1e-6, "{altitude_m}");
}

#[test]
fn the_declared_fix_covariance_is_the_simulated_spread() {
    let mut request = request();
    request.gnss = GnssSpec {
        correlation_time_s: 0.5,
        ..GnssSpec::default()
    };
    let truth = parked();
    let mut gnss = receiver(&request).unwrap();
    let count = 20_000;
    let mut squares = DVec3::ZERO;
    let mut declared = DVec3::ZERO;
    for _ in 0..count {
        let fix = gnss.measure(&truth, 1.0).unwrap();
        let SensorData::GnssFix {
            position,
            position_cov: Some(cov),
            ..
        } = gnss_fix_data(&fix)
        else {
            panic!("not a fix with covariance");
        };
        let measured = geodetic_to_ecef(&Geodetic {
            lat_rad: position.latitude_rad,
            lon_rad: position.longitude_rad,
            height_m: position.altitude_m,
        });
        let error = LocalFrame::at(&ecef_to_geodetic(truth.position_ecef))
            .ecef_to_ned_vector(measured - truth.position_ecef);
        squares += error * error;
        let (n, e, d) = cov.diagonal();
        declared = DVec3::new(n, e, d);
    }
    let ratio = squares / f64::from(count) / declared;
    assert!(
        ratio.min_element() > 0.95 && ratio.max_element() < 1.05,
        "{ratio}"
    );
}

#[test]
fn a_fix_the_filter_could_not_weigh_is_refused() {
    let mut request = request();
    request.gnss.velocity_sigma_m_s = 0.0;
    assert!(receiver(&request).is_err());
}

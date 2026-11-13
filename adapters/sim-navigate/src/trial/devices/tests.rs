use glam::DVec3;
use navigate_onboard::{ErrorBudget, SensorData};
use sim_core::VehicleState;
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};

use super::{PitotProbe, StaticPort, receiver};
use crate::trial::onboard::{
    GNSS_SOURCE, altimeter_budget, baro_altitude_data, gnss_fix_data, sensor_suite,
};
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

/// Fixes drawn for the measured budget: at 1 Hz and a 2 s correlation
/// time, enough that the white and correlated spreads and the correlation
/// time come out within a few percent.
const FIXES: usize = 200_000;

/// The receiver's position and velocity errors, north-east-down, for a
/// parked vehicle fixing once a second.
fn fix_errors(request: &TrialRequest) -> Vec<(DVec3, DVec3)> {
    let truth = parked();
    let frame = LocalFrame::at(&ecef_to_geodetic(truth.position_ecef));
    let mut gnss = receiver(request).unwrap();
    (0..FIXES)
        .map(|_| {
            let fix = gnss.measure(&truth, 1.0).unwrap();
            let SensorData::GnssFix {
                position,
                velocity: Some(velocity),
                position_cov: None,
                velocity_cov: None,
            } = gnss_fix_data(&fix)
            else {
                panic!("a fix without velocity or carrying its own covariance");
            };
            let measured = geodetic_to_ecef(&Geodetic {
                lat_rad: position.latitude_rad,
                lon_rad: position.longitude_rad,
                height_m: position.altitude_m,
            });
            let velocity = DVec3::new(velocity.north_mps, velocity.east_mps, velocity.down_mps);
            (
                frame.ecef_to_ned_vector(measured - truth.position_ecef),
                velocity - frame.ecef_to_ned_vector(truth.velocity_ecef),
            )
        })
        .collect()
}

/// The white sigma, the correlated sigma and the correlation time of a
/// white-plus-first-order-Gauss–Markov series from its autocovariances at
/// lags 0, 1 and 2 (`c0 = w² + c²`, `ck = c²·φᵏ`), with a 1 s spacing.
fn white_and_correlated(c0: f64, c1: f64, c2: f64) -> (f64, f64, f64) {
    let correlated_variance = c1 * c1 / c2;
    (
        (c0 - correlated_variance).sqrt(),
        correlated_variance.sqrt(),
        -1.0 / (c2 / c1).ln(),
    )
}

fn assert_within(what: &str, measured: f64, declared: f64, relative: f64) {
    assert!(
        (measured / declared - 1.0).abs() < relative,
        "{what}: measured {measured}, declared {declared}"
    );
}

/// The suite declares the receiver the trial flies: the white, correlated
/// and velocity statistics measured from its fixes are the GNSS budget.
#[test]
fn the_declared_gnss_budget_is_the_simulated_receiver() {
    let mut request = request();
    request.gnss = GnssSpec {
        correlation_time_s: 2.0,
        ..GnssSpec::default()
    };
    let suite = sensor_suite(&request, request.sensor_sets.first().unwrap()).unwrap();
    let ErrorBudget::Gnss {
        horizontal_sigma_m,
        vertical_sigma_m,
        velocity_sigma_m_s,
        correlated: Some(correlated),
    } = suite.get(GNSS_SOURCE).unwrap().budget
    else {
        panic!("no correlated GNSS budget");
    };
    let errors = fix_errors(&request);
    let lag = |k: usize| -> DVec3 {
        let pairs = errors.iter().zip(errors.iter().skip(k));
        pairs.map(|(a, b)| a.0 * b.0).sum::<DVec3>() / (FIXES - k) as f64
    };
    let (c0, c1, c2) = (lag(0), lag(1), lag(2));
    let horizontal = |c: DVec3| 0.5 * (c.x + c.y);
    let axes = [
        (
            "horizontal",
            white_and_correlated(horizontal(c0), horizontal(c1), horizontal(c2)),
            (horizontal_sigma_m, correlated.horizontal_sigma_m),
        ),
        (
            "vertical",
            white_and_correlated(c0.z, c1.z, c2.z),
            (vertical_sigma_m, correlated.vertical_sigma_m),
        ),
    ];
    for (axis, (white, correlated_sigma, tau), declared) in axes {
        assert_within(&format!("{axis} white"), white, declared.0, 0.05);
        assert_within(
            &format!("{axis} correlated"),
            correlated_sigma,
            declared.1,
            0.03,
        );
        assert_within(&format!("{axis} tau"), tau, correlated.tau_s, 0.05);
    }
    let velocity_squares = errors.iter().map(|e| e.1 * e.1).sum::<DVec3>() / FIXES as f64;
    for variance in velocity_squares.to_array() {
        assert_within("velocity", variance.sqrt(), velocity_sigma_m_s, 0.02);
    }
}

#[test]
fn a_receiver_without_a_correlated_error_declares_none() {
    let mut request = request();
    request.gnss.horizontal_correlated_sigma_m = 0.0;
    request.gnss.vertical_correlated_sigma_m = 0.0;
    let suite = sensor_suite(&request, request.sensor_sets.first().unwrap()).unwrap();
    assert!(
        suite
            .get(GNSS_SOURCE)
            .unwrap()
            .budget
            .gnss_correlated()
            .is_none()
    );
    assert!(!suite.has_correlated_error());
}

#[test]
fn a_fix_the_filter_could_not_weigh_is_refused() {
    let mut no_velocity = request();
    no_velocity.gnss.velocity_sigma_m_s = 0.0;
    assert!(receiver(&no_velocity).is_err());
    let mut correlated_only = request();
    correlated_only.gnss.vertical_white_sigma_m = 0.0;
    assert!(receiver(&correlated_only).is_err());
}

#[test]
fn pitot_probe_reports_differential_pressure_and_true_airspeed() {
    let mut req = request();
    req.baro.white_sigma_pa = 1e-6;
    req.baro.bias_sigma_pa = 0.0;
    let mut pitot = PitotProbe::new(&req).unwrap();
    let ground = Geodetic {
        lat_rad: LAT_RAD,
        lon_rad: LON_RAD,
        height_m: HEIGHT_M,
    };
    let flying = VehicleState::level_flight(&ground, 0.0, 52.0);
    let sample = pitot.sense(&flying, 0.0, 1.0 / 24.0).unwrap();
    let dp_pa = sample
        .differential_pressure_pa
        .expect("pitot differential pressure");
    assert!(
        dp_pa > 1_200.0,
        "expected positive impact pressure, got {dp_pa} Pa"
    );
    let tas = sim_devices::sensed_true_airspeed_m_s(&sample).unwrap();
    assert!((tas - 52.0).abs() < 0.05, "expected ~52 m/s TAS, got {tas}");
}

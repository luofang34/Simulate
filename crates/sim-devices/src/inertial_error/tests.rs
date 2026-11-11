use super::*;

const DT_S: f64 = 0.01;

fn increment(theta: DVec3, velocity: DVec3) -> InertialIncrement {
    InertialIncrement {
        dt_s: DT_S,
        delta_theta_rad: theta,
        delta_velocity_m_s: velocity,
    }
}

fn only_gyro(gyro: TriadErrorSpec) -> ImuErrorSpec {
    ImuErrorSpec {
        accel: TriadErrorSpec::IDEAL,
        gyro,
    }
}

#[test]
fn an_ideal_unit_reports_the_motion_exactly() {
    let mut unit = ImuErrorModel::new(ImuErrorSpec::IDEAL, 3).unwrap();
    let ideal = increment(DVec3::new(1e-3, -2e-3, 3e-4), DVec3::new(0.1, 0.0, -0.098));
    for _ in 0..10 {
        assert_eq!(unit.sense(&ideal).unwrap(), ideal);
    }
}

#[test]
fn a_copy_resumes_the_error_stream_exactly() {
    let mut unit = ImuErrorModel::new(ImuGrade::ConsumerMems.spec(), 11).unwrap();
    let ideal = increment(DVec3::ZERO, DVec3::new(0.0, 0.0, -0.098));
    unit.sense(&ideal).unwrap();
    let mut resumed = unit;
    for _ in 0..5 {
        assert_eq!(unit.sense(&ideal).unwrap(), resumed.sense(&ideal).unwrap());
    }
    let mut other = ImuErrorModel::new(ImuGrade::ConsumerMems.spec(), 12).unwrap();
    assert_ne!(other.sense(&ideal).unwrap(), unit.sense(&ideal).unwrap());
}

#[test]
fn white_noise_per_increment_matches_the_continuous_density() {
    let density = 1e-3;
    let spec = only_gyro(TriadErrorSpec {
        noise_density: density,
        ..TriadErrorSpec::IDEAL
    });
    let mut unit = ImuErrorModel::new(spec, 5).unwrap();
    let samples = 60_000;
    let mut squares = 0.0;
    for _ in 0..samples {
        let sensed = unit.sense(&increment(DVec3::ZERO, DVec3::ZERO)).unwrap();
        squares += sensed.delta_theta_rad.length_squared();
    }
    let variance = squares / f64::from(3 * samples);
    let expected = density * density * DT_S;
    assert!(
        (variance / expected - 1.0).abs() < 0.02,
        "{variance} vs {expected}"
    );
}

#[test]
fn turn_on_biases_follow_the_stationary_distribution() {
    let sigma = 2e-3;
    let spec = only_gyro(TriadErrorSpec {
        bias: GaussMarkov {
            tau_s: 100.0,
            sigma,
        },
        ..TriadErrorSpec::IDEAL
    });
    let units = 20_000_u64;
    let mut squares = 0.0;
    for seed in 0..units {
        squares += ImuErrorModel::new(spec, seed)
            .unwrap()
            .gyro_bias_rad_s()
            .length_squared();
    }
    let variance = squares / (3 * units) as f64;
    assert!(
        (variance / (sigma * sigma) - 1.0).abs() < 0.03,
        "{variance}"
    );
}

#[test]
fn a_gauss_markov_bias_decorrelates_over_its_time_constant() {
    let tau_s = 0.5;
    let sigma = 1.0;
    let spec = only_gyro(TriadErrorSpec {
        bias: GaussMarkov { tau_s, sigma },
        ..TriadErrorSpec::IDEAL
    });
    let steps = (tau_s / DT_S) as usize;
    let units = 8_000_u64;
    let (mut product, mut start_squares, mut end_squares) = (0.0, 0.0, 0.0);
    for seed in 0..units {
        let mut unit = ImuErrorModel::new(spec, seed).unwrap();
        let start = unit.gyro_bias_rad_s();
        for _ in 0..steps {
            unit.sense(&increment(DVec3::ZERO, DVec3::ZERO)).unwrap();
        }
        let end = unit.gyro_bias_rad_s();
        product += start.dot(end);
        start_squares += start.length_squared();
        end_squares += end.length_squared();
    }
    let correlation = product / (start_squares * end_squares).sqrt();
    assert!(
        (correlation - (-1.0_f64).exp()).abs() < 0.03,
        "{correlation}"
    );
    let stationary = end_squares / (3 * units) as f64;
    assert!((stationary - 1.0).abs() < 0.05, "{stationary}");
}

#[test]
fn a_constant_bias_adds_its_integral_to_each_increment() {
    let spec = only_gyro(TriadErrorSpec {
        bias: GaussMarkov {
            tau_s: f64::INFINITY,
            sigma: 1e-2,
        },
        ..TriadErrorSpec::IDEAL
    });
    let mut unit = ImuErrorModel::new(spec, 9).unwrap();
    let bias = unit.gyro_bias_rad_s();
    assert_ne!(bias, DVec3::ZERO);
    for _ in 0..100 {
        let sensed = unit.sense(&increment(DVec3::ZERO, DVec3::ZERO)).unwrap();
        assert!((sensed.delta_theta_rad - bias * DT_S).length() < 1e-18);
    }
    assert_eq!(unit.gyro_bias_rad_s(), bias);
}

#[test]
fn scale_factor_and_misalignment_mix_the_axes() {
    let spec = only_gyro(TriadErrorSpec {
        scale_factor_sigma: 1e-2,
        misalignment_sigma_rad: 1e-3,
        ..TriadErrorSpec::IDEAL
    });
    let mut unit = ImuErrorModel::new(spec, 21).unwrap();
    let sensed = unit.sense(&increment(DVec3::X, DVec3::ZERO)).unwrap();
    let gain = sensed.delta_theta_rad;
    assert!((gain.x - 1.0).abs() > 0.0 && (gain.x - 1.0).abs() < 0.05);
    assert!(gain.y != 0.0 && gain.y.abs() < 5e-3);
    assert!(gain.z != 0.0 && gain.z.abs() < 5e-3);
}

#[test]
fn grades_order_every_statistic() {
    let [consumer, tactical, navigation] = ImuGrade::ALL.map(ImuGrade::spec);
    // Accelerometer white noise does not set the grade: a tactical unit's
    // velocity random walk is no better than a good MEMS part's.
    assert!(tactical.accel.noise_density > navigation.accel.noise_density);
    let total = |triad: TriadErrorSpec| triad.turn_on_bias_sigma.hypot(triad.bias.sigma);
    for (worse, better) in [(consumer, tactical), (tactical, navigation)] {
        assert!(worse.gyro.noise_density > better.gyro.noise_density);
        assert!(total(worse.accel) > total(better.accel));
        assert!(total(worse.gyro) > total(better.gyro));
    }
    let consumer_gyro_turn_on_deg_s = consumer.gyro.turn_on_bias_sigma.to_degrees();
    assert!((consumer_gyro_turn_on_deg_s - 0.5).abs() < 1e-12);
    let tactical_gyro_bias_deg_h = tactical.gyro.bias.sigma.to_degrees() * 3_600.0;
    assert!((tactical_gyro_bias_deg_h - 1.0).abs() < 1e-12);
    let navigation_gyro_bias_deg_h = navigation.gyro.bias.sigma.to_degrees() * 3_600.0;
    assert!((navigation_gyro_bias_deg_h - 0.01).abs() < 1e-4);
}

/// A consumer part's large turn-on bias holds for the power-up; only its
/// small in-run instability wanders.
#[test]
fn the_turn_on_bias_is_a_constant_for_the_power_up() {
    let spec = ImuGrade::ConsumerMems.spec();
    let mut unit = ImuErrorModel::new(spec, 7).unwrap();
    let turn_on = unit.gyro_turn_on_bias_rad_s();
    let first = unit.gyro_bias_rad_s();
    assert!(turn_on.length() > 0.0);
    for _ in 0..6_000 {
        unit.sense(&increment(DVec3::ZERO, DVec3::ZERO)).unwrap();
    }
    assert_eq!(unit.gyro_turn_on_bias_rad_s(), turn_on);
    let wandered = (unit.gyro_bias_rad_s() - first).length();
    assert!(
        wandered < 6.0 * spec.gyro.bias.sigma,
        "a minute wandered {wandered} rad/s"
    );
    let other = ImuErrorModel::new(spec, 8).unwrap();
    assert_ne!(other.gyro_turn_on_bias_rad_s(), turn_on);
}

#[test]
fn unusable_statistics_and_intervals_are_refused() {
    let negative = only_gyro(TriadErrorSpec {
        noise_density: -1.0,
        ..TriadErrorSpec::IDEAL
    });
    assert!(matches!(
        ImuErrorModel::new(negative, 0),
        Err(DeviceError::InvalidErrorModel {
            triad: "gyro",
            field: "noise_density",
            ..
        })
    ));
    let mut unit = ImuErrorModel::new(ImuErrorSpec::IDEAL, 0).unwrap();
    let mut bad = increment(DVec3::ZERO, DVec3::ZERO);
    bad.dt_s = 0.0;
    assert!(matches!(
        unit.sense(&bad),
        Err(DeviceError::InvalidIncrement { .. })
    ));
}

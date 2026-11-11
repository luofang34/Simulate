use super::*;

const FIELD_UT: DVec3 = DVec3::new(20.5, -4.1, 46.8);

#[test]
fn an_ideal_magnetometer_reports_the_field() {
    let mut sensor = Magnetometer::new(MagnetometerSpec::IDEAL, 3).unwrap();
    assert_eq!(sensor.sense(FIELD_UT), FIELD_UT);
    assert_eq!(sensor.hard_iron_ut(), DVec3::ZERO);
}

#[test]
fn the_hard_iron_offset_is_constant_and_the_noise_is_not() {
    let spec = MagnetometerSpec {
        white_sigma_ut: 0.2,
        hard_iron_sigma_ut: 1.5,
        scale_factor_sigma: 0.0,
    };
    let mut sensor = Magnetometer::new(spec, 3).unwrap();
    let offset = sensor.hard_iron_ut();
    assert!(offset.length() > 0.0);
    let readings: Vec<DVec3> = (0..2_000).map(|_| sensor.sense(FIELD_UT)).collect();
    let mean = readings.iter().copied().sum::<DVec3>() / readings.len() as f64;
    assert!((mean - FIELD_UT - offset).length() < 0.03, "mean {mean:?}");
    let variance = readings
        .iter()
        .map(|r| (*r - mean).length_squared())
        .sum::<f64>()
        / (3 * readings.len()) as f64;
    assert!(
        (variance.sqrt() - 0.2).abs() < 0.02,
        "noise {} uT",
        variance.sqrt()
    );
    assert_eq!(sensor.hard_iron_ut(), offset);
}

#[test]
fn a_seed_fixes_the_sensor_and_bad_statistics_are_refused() {
    let spec = MagnetometerSpec {
        white_sigma_ut: 0.2,
        hard_iron_sigma_ut: 1.5,
        scale_factor_sigma: 0.01,
    };
    let read = |seed| Magnetometer::new(spec, seed).unwrap().sense(FIELD_UT);
    assert_eq!(read(5), read(5));
    assert_ne!(read(5), read(6));
    let bad = MagnetometerSpec {
        white_sigma_ut: f64::NAN,
        ..spec
    };
    assert!(Magnetometer::new(bad, 1).is_err());
}

use sim_atmosphere::{geometric_to_geopotential_m, isa_at_geopotential};

use super::*;

#[test]
fn pressure_altitude_inverts_the_standard_atmosphere() {
    for geometric_m in [0.0, 1_500.0, 6_000.0, 9_000.0] {
        let geopotential = geometric_to_geopotential_m(geometric_m);
        let isa = isa_at_geopotential(geopotential).unwrap();
        let altitude = pressure_altitude_m(isa.pressure_pa).unwrap();
        assert!((altitude - geopotential).abs() < 1.0, "{altitude}");
    }
}

#[test]
fn the_stratosphere_and_nonsense_have_no_pressure_altitude() {
    assert_eq!(pressure_altitude_m(10_000.0), None);
    assert_eq!(pressure_altitude_m(f64::NAN), None);
}

#[test]
fn indicated_falls_below_true_with_height() {
    let sample = |density_kg_m3| AirDataSample {
        true_airspeed_m_s: 100.0,
        density_kg_m3,
        static_pressure_pa: None,
        altimeter_setting_hpa: None,
        pec_indicated_airspeed_m_s: None,
    };
    assert_eq!(sample(1.225).indicated_airspeed_m_s(), 100.0);
    let at_height = sample(1.225 / 4.0).indicated_airspeed_m_s();
    assert!((at_height - 50.0).abs() < 1e-12);
}

#[test]
fn pec_override_shifts_indicated_while_preserving_calibrated_airspeed() {
    let base = AirDataSample {
        true_airspeed_m_s: 27.265_555_555_6,
        density_kg_m3: 1.225,
        static_pressure_pa: Some(101_325.0),
        altimeter_setting_hpa: Some(1_013.25),
        pec_indicated_airspeed_m_s: None,
    };
    let with_pec = base.with_pec_indicated_airspeed_m_s(24.693_333_333_3);
    assert!((with_pec.calibrated_airspeed_m_s() - 27.265_555_555_6).abs() < 1e-12);
    assert!((with_pec.indicated_airspeed_m_s() - 24.693_333_333_3).abs() < 1e-12);
}

use navigate_onboard::ErrorBudget;
use sim_atmosphere::{geometric_to_geopotential_m, isa_at_geopotential};

use super::{altimeter_budget, pressure_altitude_m};
use crate::trial::BaroSpec;

#[test]
fn pressure_altitude_inverts_the_standard_atmosphere() {
    for height_m in [-400.0, 0.0, 1_000.0, 2_000.0, 11_500.0, 25_000.0] {
        let pressure_pa = isa_at_geopotential(geometric_to_geopotential_m(height_m))
            .unwrap()
            .pressure_pa;
        let altitude_m = pressure_altitude_m(pressure_pa).unwrap();
        assert!(
            (altitude_m - height_m).abs() < 1e-6,
            "{height_m}: {altitude_m}"
        );
    }
}

#[test]
fn an_unreadable_pressure_is_refused() {
    for pressure_pa in [0.0, -1.0, f64::NAN, 1e-3] {
        assert!(pressure_altitude_m(pressure_pa).is_err(), "{pressure_pa}");
    }
}

#[test]
fn the_budget_follows_the_pressure_gradient() {
    // At sea level a pascal is 1 / (1.225 kg/m³ × 9.80665 m/s²) metres.
    let baro = BaroSpec {
        white_sigma_pa: 1.0,
        bias_sigma_pa: 10.0,
        ..BaroSpec::default()
    };
    let ErrorBudget::Barometer {
        white_sigma_m,
        bias,
    } = altimeter_budget(&baro, 0.0).unwrap()
    else {
        panic!("not a barometer budget");
    };
    assert!((white_sigma_m - 0.083_24).abs() < 1e-4, "{white_sigma_m}");
    assert!((bias.sigma() - 0.832_4).abs() < 1e-3, "{}", bias.sigma());
    assert_eq!(bias.tau_s(), baro.bias_correlation_time_s);
}

use super::*;

fn assert_close(actual: f64, expected: f64, relative: f64, what: &str) {
    assert!(
        ((actual - expected) / expected).abs() < relative,
        "{what}: {actual} vs {expected}"
    );
}

// Expected values are the ICAO Doc 7488 standard-atmosphere table entries.
#[test]
fn sea_level_matches_icao_table() {
    let s = isa_at_geopotential(0.0).unwrap();
    assert_close(s.temperature_k, 288.15, 1e-12, "T");
    assert_close(s.pressure_pa, 101_325.0, 1e-12, "p");
    assert_close(s.density_kg_m3, 1.2250, 1e-4, "rho");
    assert_close(s.speed_of_sound_m_s, 340.294, 1e-5, "a");
}

#[test]
fn layer_bases_are_continuous() {
    let tropopause = isa_at_geopotential(11_000.0).unwrap();
    assert_close(tropopause.temperature_k, 216.65, 1e-12, "T 11 km");
    assert_close(tropopause.pressure_pa, 22_632.06, 1e-5, "p 11 km");
    let below = isa_at_geopotential(10_999.999).unwrap();
    assert_close(
        below.pressure_pa,
        tropopause.pressure_pa,
        1e-6,
        "p continuity 11 km",
    );
    let stratosphere = isa_at_geopotential(20_000.0).unwrap();
    assert_close(stratosphere.pressure_pa, 5_474.889, 1e-5, "p 20 km");
    let top = isa_at_geopotential(32_000.0).unwrap();
    assert_close(top.temperature_k, 228.65, 1e-12, "T 32 km");
    assert_close(top.pressure_pa, 868.019, 1e-4, "p 32 km");
}

#[test]
fn rejects_heights_outside_the_model() {
    assert!(matches!(
        isa_at_geopotential(32_000.1),
        Err(AtmosphereError::OutOfRange { .. })
    ));
    assert!(isa_at_geopotential(-5_000.1).is_err());
    assert!(isa_at_geopotential(-400.0).unwrap().pressure_pa > 101_325.0);
}

#[test]
fn geopotential_is_below_geometric_above_sea_level() {
    let h = geometric_to_geopotential_m(10_000.0);
    assert!(h < 10_000.0 && h > 9_980.0, "{h}");
}

#[test]
fn ussa76_layers_chain_into_each_other() {
    for pair in LAYERS.windows(2) {
        let [lower, upper] = pair else {
            panic!("windows(2) yields pairs");
        };
        let below = ussa76_at_geopotential(upper.base_m - 1e-6).unwrap();
        let above = ussa76_at_geopotential(upper.base_m).unwrap();
        assert_close(below.temperature_k, above.temperature_k, 1e-9, "T seam");
        assert_close(below.pressure_pa, above.pressure_pa, 3e-6, "p seam");
        assert!(lower.base_m < upper.base_m);
    }
}

// Expected values are the 1976 U.S. Standard Atmosphere table entries.
#[test]
fn ussa76_matches_the_published_table_above_the_icao_ceiling() {
    let z47 = ussa76_at_geopotential(geometric_to_geopotential_m(50_000.0)).unwrap();
    assert_close(z47.temperature_k, 270.65, 1e-5, "T 50 km");
    assert_close(z47.pressure_pa, 79.779, 1e-4, "p 50 km");
    assert_close(z47.density_kg_m3, 1.0269e-3, 1e-4, "rho 50 km");
    let z70 = ussa76_at_geopotential(geometric_to_geopotential_m(70_000.0)).unwrap();
    assert_close(z70.temperature_k, 219.585, 1e-5, "T 70 km");
    assert_close(z70.pressure_pa, 5.2209, 1e-4, "p 70 km");
}

#[test]
fn ussa76_keeps_the_error_contract() {
    assert!(ussa76_at_geopotential(32_000.1).is_ok());
    assert!(matches!(
        ussa76_at_geopotential(84_853.0),
        Err(AtmosphereError::OutOfRange { .. })
    ));
    assert!(ussa76_at_geopotential(f64::NAN).is_err());
}

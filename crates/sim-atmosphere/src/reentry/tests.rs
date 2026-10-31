use super::*;
use crate::StandardAtmosphere;

const UTC_UNIX_SECONDS: f64 = 0.0;
/// Apollo's Entry Interface, 400,000 ft.
const ENTRY_INTERFACE_M: f64 = 121_920.0;

fn at(height_m: f64) -> AtmosphereSample {
    ReentryAtmosphere
        .sample(
            &Geodetic::from_degrees(-13.3, -169.2, height_m),
            UtcTime::from_unix_seconds(UTC_UNIX_SECONDS),
        )
        .unwrap()
}

fn assert_close(actual: f64, expected: f64, relative: f64, what: &str) {
    assert!(
        ((actual - expected) / expected).abs() < relative,
        "{what}: {actual} vs {expected}"
    );
}

#[test]
fn matches_the_standard_atmosphere_where_both_are_defined() {
    for height_m in [-400.0, 0.0, 5_000.0, 11_000.0, 20_000.0, 31_000.0] {
        let position = Geodetic::from_degrees(0.0, 0.0, height_m);
        let standard = StandardAtmosphere
            .sample(&position, UtcTime::from_unix_seconds(UTC_UNIX_SECONDS))
            .unwrap();
        assert_eq!(at(height_m), standard, "{height_m}");
    }
}

// Expected values are the 1976 U.S. Standard Atmosphere table entries.
#[test]
fn matches_the_published_thermosphere() {
    let table = [
        (100_000.0, 195.081, 3.2012e-2, 5.6044e-7, 280.00),
        (120_000.0, 360.0, 2.5366e-3, 2.2199e-8, 380.36),
        (150_000.0, 634.392, 4.5422e-4, 2.0752e-9, 504.92),
    ];
    for (height_m, temperature_k, pressure_pa, density_kg_m3, sound_m_s) in table {
        let sample = at(height_m);
        assert_close(sample.temperature_k, temperature_k, 1e-5, "T");
        assert_close(sample.pressure_pa, pressure_pa, 1e-4, "p");
        assert_close(sample.density_kg_m3, density_kg_m3, 1e-4, "rho");
        assert_close(sample.speed_of_sound_m_s, sound_m_s, 1e-3, "a");
    }
}

// 135 km and 165 km are published rows the table leaves out, so they measure
// how closely each interval's exponential follows the real decay.
#[test]
fn interpolation_tracks_published_rows_between_nodes() {
    let sample = at(135_000.0);
    assert_close(sample.pressure_pa, 9.3569e-4, 0.03, "p 135 km");
    assert_close(sample.density_kg_m3, 5.4647e-9, 0.03, "rho 135 km");
    let sample = at(165_000.0);
    assert_close(sample.pressure_pa, 2.5277e-4, 0.03, "p 165 km");
    assert_close(sample.density_kg_m3, 9.7526e-10, 0.03, "rho 165 km");
}

#[test]
fn entry_interface_is_inside_the_model() {
    let sample = at(ENTRY_INTERFACE_M);
    assert!(sample.density_kg_m3 > 1.2918e-8 && sample.density_kg_m3 < 2.2199e-8);
    assert!(sample.temperature_k > 360.0 && sample.temperature_k < 417.231);
}

#[test]
fn seam_with_the_layers_has_no_step() {
    let below = at(LAYERS_TOP_M - 1e-3);
    let above = at(LAYERS_TOP_M);
    assert_close(below.temperature_k, above.temperature_k, 1e-6, "T");
    assert_close(below.pressure_pa, above.pressure_pa, 1e-5, "p");
    assert_close(below.density_kg_m3, above.density_kg_m3, 1e-5, "rho");
    assert_close(
        below.speed_of_sound_m_s,
        above.speed_of_sound_m_s,
        1e-6,
        "a",
    );
}

/// Largest allowed change per step: a share of the current value, except in
/// the fade, whose values head to zero and so are held to a share of the value
/// where the fade began.
fn step_limit(value: f64, at_fade_start: f64, height_m: f64) -> f64 {
    let reference = if height_m > FADE_START_M {
        at_fade_start
    } else {
        value
    };
    0.02 * reference
}

#[test]
fn every_field_is_continuous_from_sea_level_to_orbit() {
    let fade_start = at(FADE_START_M);
    let step_m = 50.0;
    let mut previous = at(0.0);
    let mut height_m = 0.0;
    while height_m < 250_000.0 {
        height_m += step_m;
        let sample = at(height_m);
        let pressure_step = previous.pressure_pa - sample.pressure_pa;
        let density_step = previous.density_kg_m3 - sample.density_kg_m3;
        assert!(pressure_step >= 0.0, "p rises at {height_m}");
        assert!(density_step >= 0.0, "rho rises at {height_m}");
        assert!(
            pressure_step <= step_limit(previous.pressure_pa, fade_start.pressure_pa, height_m),
            "p step at {height_m}"
        );
        assert!(
            density_step <= step_limit(previous.density_kg_m3, fade_start.density_kg_m3, height_m),
            "rho step at {height_m}"
        );
        assert!(
            (sample.temperature_k - previous.temperature_k).abs() < 15.0,
            "T step at {height_m}"
        );
        assert!(
            (sample.speed_of_sound_m_s - previous.speed_of_sound_m_s).abs() < 15.0,
            "a step at {height_m}"
        );
        previous = sample;
    }
}

#[test]
fn fade_starts_flat_and_ends_at_vacuum() {
    assert_eq!(fade_to_vacuum(FADE_START_M), 1.0);
    assert_eq!(fade_to_vacuum(VACUUM_FLOOR_M), 0.0);
    assert!((fade_to_vacuum(190_000.0) - 0.5).abs() < 1e-12);
}

#[test]
fn above_the_fade_is_exactly_vacuum() {
    for height_m in [200_000.0, 250_000.0, 384_400_000.0] {
        assert_eq!(at(height_m), VacuumAtmosphere::SAMPLE);
    }
}

#[test]
fn below_the_floor_and_nan_are_out_of_range() {
    let sample = |height_m| {
        ReentryAtmosphere.sample(
            &Geodetic::from_degrees(0.0, 0.0, height_m),
            UtcTime::from_unix_seconds(UTC_UNIX_SECONDS),
        )
    };
    assert!(matches!(
        sample(-6_000.0),
        Err(AtmosphereError::OutOfRange { .. })
    ));
    assert!(matches!(
        sample(f64::NAN),
        Err(AtmosphereError::OutOfRange { .. })
    ));
}

#[test]
fn air_is_still_at_every_height() {
    for height_m in [0.0, 50_000.0, 121_920.0, 190_000.0, 300_000.0] {
        assert_eq!(at(height_m).wind_ned_m_s, DVec3::ZERO);
    }
}

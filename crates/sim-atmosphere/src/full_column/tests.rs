use glam::DVec3;
use sim_geo::{Geodetic, UtcTime};

use super::*;
use crate::{GeoidStandardAtmosphere, VacuumAtmosphere};

/// Where the geoid sits far from the ellipsoid (about −100 m, south of India).
const GEOID_LOW: (f64, f64) = (5.0, 78.0);

fn utc() -> UtcTime {
    UtcTime::from_unix_seconds(0.0)
}

fn at_msl(field: &FullColumnAtmosphere, msl_m: f64) -> AtmosphereSample {
    let (lat, lon) = GEOID_LOW;
    let position = Geodetic::from_degrees(lat, lon, 0.0);
    let undulation = sim_geo::egm96_undulation_m(position.lat_rad, position.lon_rad);
    field
        .sample(
            &Geodetic {
                height_m: msl_m + undulation,
                ..position
            },
            utc(),
        )
        .unwrap()
}

fn close(a: f64, b: f64, relative: f64) -> bool {
    let scale = a.abs().max(b.abs());
    scale == 0.0 || (a - b).abs() / scale < relative
}

#[test]
fn reads_sea_level_isa_on_the_geoid_not_the_ellipsoid() {
    let (lat, lon) = GEOID_LOW;
    let undulation = sim_geo::egm96_undulation_m(lat.to_radians(), lon.to_radians());
    assert!(undulation < -50.0, "{undulation}");
    let sea = at_msl(&FullColumnAtmosphere::new(), 0.0);
    assert!(
        (sea.pressure_pa - 101_325.0).abs() < 1e-6,
        "{}",
        sea.pressure_pa
    );
    assert!((sea.temperature_k - 288.15).abs() < 1e-9);
    let on_ellipsoid = FullColumnAtmosphere::new()
        .sample(&Geodetic::from_degrees(lat, lon, 0.0), utc())
        .unwrap();
    assert!(on_ellipsoid.pressure_pa < 101_325.0 - 500.0);
    let geoid_isa = GeoidStandardAtmosphere::new()
        .sample(&Geodetic::from_degrees(lat, lon, 9_000.0), utc())
        .unwrap();
    let full = FullColumnAtmosphere::new()
        .sample(&Geodetic::from_degrees(lat, lon, 9_000.0), utc())
        .unwrap();
    assert!(close(full.pressure_pa, geoid_isa.pressure_pa, 1e-9));
}

#[test]
fn every_layer_handover_is_continuous() {
    let field = FullColumnAtmosphere::new();
    for boundary_m in [11_000.0, 32_000.0, 86_000.0, 180_000.0] {
        let below = at_msl(&field, boundary_m - 1.0);
        let above = at_msl(&field, boundary_m + 1.0);
        for (what, a, b) in [
            ("T", below.temperature_k, above.temperature_k),
            ("p", below.pressure_pa, above.pressure_pa),
            ("rho", below.density_kg_m3, above.density_kg_m3),
        ] {
            assert!(close(a, b, 1e-3), "{what} at {boundary_m}: {a} vs {b}");
        }
    }
    let top = at_msl(&field, 199_999.0);
    assert!(
        top.pressure_pa < 1e-9 && top.density_kg_m3 < 1e-15,
        "{top:?}"
    );
    assert_eq!(at_msl(&field, 200_000.0), VacuumAtmosphere::SAMPLE);
    assert_eq!(at_msl(&field, 400_000.0), VacuumAtmosphere::SAMPLE);
}

#[test]
fn wind_holds_low_and_dies_out_before_the_thermosphere() {
    let wind = DVec3::new(12.0, -5.0, 0.0);
    let field = FullColumnAtmosphere::with_wind(wind);
    assert_eq!(at_msl(&field, 1_000.0).wind_ned_m_s, wind);
    assert_eq!(at_msl(&field, 32_000.0).wind_ned_m_s, wind);
    let mid = at_msl(&field, 59_000.0).wind_ned_m_s;
    assert!(mid.length() > 0.0 && mid.length() < wind.length());
    for msl_m in [86_000.0, 120_000.0, 250_000.0] {
        assert_eq!(at_msl(&field, msl_m).wind_ned_m_s, DVec3::ZERO, "{msl_m}");
    }
}

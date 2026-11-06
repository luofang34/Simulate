use glam::DVec3;
use sim_geo::{Geodetic, OrthometricHeightM, UtcTime, msl_egm96_to_ellipsoidal};

use super::*;

const SEA_LEVEL_PA: f64 = 101_325.0;

fn utc() -> UtcTime {
    UtcTime::from_unix_seconds(0.0)
}

/// A point charted `msl_m` above mean sea level, with its ellipsoidal height.
fn at_msl(lat_deg: f64, lon_deg: f64, msl_m: f64) -> Geodetic {
    let mut position = Geodetic::from_degrees(lat_deg, lon_deg, 0.0);
    position.height_m = msl_egm96_to_ellipsoidal(
        position.lat_rad,
        position.lon_rad,
        OrthometricHeightM(msl_m),
    )
    .0;
    position
}

#[test]
fn mean_sea_level_reads_sea_level_isa_wherever_the_geoid_lies() {
    for (lat, lon) in [(39.13, -75.47), (4.5, 78.5), (-6.0, 147.0), (60.0, -25.0)] {
        let sample = GeoidStandardAtmosphere::new()
            .sample(&at_msl(lat, lon, 0.0), utc())
            .unwrap();
        assert!(
            (sample.pressure_pa - SEA_LEVEL_PA).abs() < 1e-6,
            "{lat},{lon}"
        );
        assert!((sample.temperature_k - 288.15).abs() < 1e-9);
    }
}

#[test]
fn it_matches_the_ellipsoidal_model_shifted_by_the_undulation() {
    let charted = at_msl(39.13, -75.47, 1_500.0);
    let geoid = GeoidStandardAtmosphere::new()
        .sample(&charted, utc())
        .unwrap();
    let as_if_msl = Geodetic {
        height_m: 1_500.0,
        ..charted
    };
    let reference = StandardAtmosphere.sample(&as_if_msl, utc()).unwrap();
    assert!((geoid.pressure_pa - reference.pressure_pa).abs() < 1e-6);
    assert!((geoid.density_kg_m3 - reference.density_kg_m3).abs() < 1e-12);
    // Over Delaware MSL is about 35 m below the ellipsoid, so the plain model
    // reads the same ellipsoidal height as higher, thinner air.
    let ellipsoidal = StandardAtmosphere.sample(&charted, utc()).unwrap();
    assert!(ellipsoidal.pressure_pa > geoid.pressure_pa + 300.0);
}

#[test]
fn the_wind_is_uniform_and_still_air_is_the_default() {
    let wind = DVec3::new(5.0, -3.0, 0.5);
    let position = at_msl(40.69, -74.17, 300.0);
    let windy = GeoidStandardAtmosphere::with_wind(wind)
        .sample(&position, utc())
        .unwrap();
    let still = GeoidStandardAtmosphere::new()
        .sample(&position, utc())
        .unwrap();
    assert_eq!(windy.wind_ned_m_s, wind);
    assert_eq!(still.wind_ned_m_s, DVec3::ZERO);
    assert_eq!(windy.pressure_pa, still.pressure_pa);
}

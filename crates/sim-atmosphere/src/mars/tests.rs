use sim_geo::{Geodetic, UtcTime};

use crate::{AtmosphereError, AtmosphereField, VacuumAtmosphere};

use super::MarsAtmosphere;

fn geo(height_m: f64) -> Geodetic {
    Geodetic {
        lat_rad: 0.0,
        lon_rad: 0.0,
        height_m,
    }
}

#[test]
fn datum_and_hellas_basin_bounds_verified() {
    let atm = MarsAtmosphere::new();
    let utc = UtcTime::from_unix_seconds(0.0);

    let datum = atm.sample(&geo(0.0), utc).expect("datum");
    assert!((datum.pressure_pa - 610.0).abs() < 1e-9);
    assert!((datum.temperature_k - 214.0).abs() < 1e-9);
    assert!(datum.density_kg_m3 > 0.014 && datum.density_kg_m3 < 0.016);
    assert!(datum.speed_of_sound_m_s > 225.0 && datum.speed_of_sound_m_s < 245.0);

    let hellas = atm.sample(&geo(-8_200.0), utc).expect("hellas");
    assert!(hellas.pressure_pa > 1_100.0);
    assert!(hellas.density_kg_m3 > datum.density_kg_m3);

    assert!(matches!(
        atm.sample(&geo(-9_500.0), utc),
        Err(AtmosphereError::OutOfRange { .. })
    ));
}

#[test]
fn thermosphere_seam_continuity_and_vacuum_fade_verified() {
    let atm = MarsAtmosphere::new();
    let utc = UtcTime::from_unix_seconds(0.0);

    let below_110 = atm.sample(&geo(109_999.99), utc).expect("below 110 km");
    let at_110 = atm.sample(&geo(110_000.0), utc).expect("at 110 km");
    assert!(((at_110.pressure_pa - below_110.pressure_pa) / at_110.pressure_pa).abs() < 1e-5);
    assert!(((at_110.density_kg_m3 - below_110.density_kg_m3) / at_110.density_kg_m3).abs() < 1e-5);

    let space = atm.sample(&geo(200_000.0), utc).expect("200 km");
    let vac = VacuumAtmosphere.sample(&geo(200_000.0), utc).expect("vac");
    assert_eq!(space, vac);
}

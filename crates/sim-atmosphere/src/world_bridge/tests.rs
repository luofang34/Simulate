use sim_geo::{Geodetic, UtcTime};
use world_airmass::{AirmassStore, AloftLevel, CloudGenus, CloudLayer, SoundingProfile};
use world_atmosphere::{Altitude, Isa};
use world_core::{DecimalYear, GeodeticPosition, Ned, ValidityWindow};

use crate::{AtmosphereField, CloudField, CloudKind, StandardAtmosphere, utc_to_decimal_year};

#[test]
fn world_isa_matches_sim_standard_atmosphere() {
    let pos = Geodetic::from_degrees(39.18, -75.53, 1_200.0);
    let utc = UtcTime::from_unix_seconds(1_780_000_000.0);

    let from_sim = StandardAtmosphere.sample(&pos, utc).unwrap();
    let from_world = Isa.sample(&pos, utc).unwrap();

    assert!((from_sim.temperature_k - from_world.temperature_k).abs() < 1e-6);
    assert!((from_sim.pressure_pa - from_world.pressure_pa).abs() < 1e-2);
    assert!((from_sim.density_kg_m3 - from_world.density_kg_m3).abs() < 1e-6);
}

#[test]
fn airmass_store_serves_both_3d_wind_and_cloud_fields() {
    let mut store = AirmassStore::default();
    let utc = UtcTime::from_unix_seconds(1_780_000_000.0);
    let now = utc_to_decimal_year(utc);
    let valid = ValidityWindow {
        from: DecimalYear(now.value() - 0.1),
        until: DecimalYear(now.value() + 0.1),
    };

    let sounding = SoundingProfile::new(
        1,
        Some(GeodeticPosition::wgs84(39.18, -75.53, 0.0)),
        40_000.0,
        valid,
        None,
        vec![AloftLevel {
            altitude: Altitude::GeometricMsl(1_000.0),
            temperature_k: 286.0,
            pressure_pa: None,
            wind_ned_m_s: Ned {
                north_m: 11.5,
                east_m: -4.25,
                down_m: 0.2,
            },
            gust_m_s: 2.0,
            turbulence_edr: 0.1,
        }],
    )
    .unwrap();
    store.upsert_sounding(sounding).unwrap();

    store
        .upsert_cloud_layer(CloudLayer {
            id: 9,
            center: None,
            radius_m: 0.0,
            coverage: 0.75,
            base_msl_m: 800.0,
            top_msl_m: 2_200.0,
            genus: CloudGenus::Cumuliform,
            valid,
        })
        .unwrap();

    let pos = Geodetic::from_degrees(39.18, -75.53, 1_000.0);
    let air = AtmosphereField::sample(&store, &pos, utc).unwrap();
    assert!((air.wind_ned_m_s.x - 11.5).abs() < 1e-6);
    assert!((air.wind_ned_m_s.y + 4.25).abs() < 1e-6);
    assert!((air.wind_ned_m_s.z - 0.2).abs() < 1e-6);

    let cloud = CloudField::sample(&store, &pos, utc).unwrap();
    assert_eq!(cloud.kind, CloudKind::Cumuliform);
    assert!((cloud.coverage - 0.75).abs() < 1e-5);
}

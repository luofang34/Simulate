//! Real MOLA terrain around Jezero crater: physical sanity against published values and
//! terrain line-of-sight occultation on the measured surface.

use glam::DVec3;
use sim_analysis::{
    AccessRestrictionReason, DemElevationGrid, MARS_AREOID_ABOVE_IAU_ELLIPSOID_M,
    TerrainElevationSource, TerrainLosConfig, check_terrain_line_of_sight_fixed,
};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_spacetime::TdbEpoch;

/// Perseverance landing site as given for this task, planetocentric degrees.
const SITE_LAT_CENTRIC_DEG: f64 = 18.4447;
const SITE_LON_DEG: f64 = 77.4508;

#[cfg(test)]
fn epoch() -> TdbEpoch {
    TdbEpoch::new(820_497_600, 0)
}

#[cfg(test)]
fn site_geodetic_lat_rad() -> f64 {
    let e2 = BodyEllipsoid::MARS_IAU.eccentricity_squared();
    (SITE_LAT_CENTRIC_DEG.to_radians().tan() / (1.0 - e2)).atan()
}

#[cfg(test)]
/// Terrain height above the areoid (metres), i.e. in the datum of the published MOLA values.
fn areoid_height_m(dem: &DemElevationGrid, lat_rad: f64, lon_deg: f64) -> f64 {
    dem.elevation_above_ellipsoid_m(dem.body(), lat_rad, lon_deg.to_radians(), epoch())
        .expect("inside window")
        - MARS_AREOID_ABOVE_IAU_ELLIPSOID_M
}

#[test]
fn jezero_floor_and_western_rim_match_published_relief() {
    let dem = DemElevationGrid::mars_jezero_mola().expect("load");
    let lat = site_geodetic_lat_rad();

    // DLR/HRSC topographic map of Jezero (areoid-referenced): "the floor of Jezero Crater lies
    // at an elevation of approximately minus 2600 metres"
    // (https://www.dlr.de/en/images/2021/1/topographic-map-of-the-surroundings-of-jezero-crater).
    let floor_m = areoid_height_m(&dem, lat, SITE_LON_DEG);
    assert!((floor_m + 2600.0).abs() <= 150.0, "floor {floor_m} m");

    // Crater diameter about 45 km (https://en.wikipedia.org/wiki/Jezero_(crater)), so the
    // western rim lies within 30 km of the centre-ish landing site. 1 degree of longitude is
    // about 56 km here.
    let rim_west_m = (0..=60)
        .map(|i| areoid_height_m(&dem, lat, SITE_LON_DEG - 0.53 * f64::from(i) / 60.0))
        .fold(f64::MIN, f64::max);
    assert!(rim_west_m >= floor_m + 300.0, "west rim {rim_west_m} m");
}

#[cfg(test)]
fn fixed_m(lat_rad: f64, lon_deg: f64, height_m: f64) -> DVec3 {
    BodyEllipsoid::MARS_IAU.geodetic_to_fixed(&Geodetic {
        lat_rad,
        lon_rad: lon_deg.to_radians(),
        height_m,
    })
}

#[test]
fn low_ray_to_western_rim_is_blocked_and_high_ray_is_clear() {
    let dem = DemElevationGrid::mars_jezero_mola().expect("load");
    let lat = site_geodetic_lat_rad();
    let config = TerrainLosConfig {
        sample_step_m: 100.0,
        min_clearance_m: 2.0,
        max_terrain_interaction_altitude_m: 8_000.0,
        max_ray_samples: 50_000,
    };
    let site_h_m = dem
        .elevation_above_ellipsoid_m(dem.body(), lat, SITE_LON_DEG.to_radians(), epoch())
        .expect("site")
        + 2.0;
    let site = fixed_m(lat, SITE_LON_DEG, site_h_m);
    let check = |target_h_m: f64| {
        check_terrain_line_of_sight_fixed(
            epoch(),
            site,
            fixed_m(lat, 76.6, target_h_m),
            BodyEllipsoid::MARS_IAU,
            &dem,
            &config,
        )
        .expect("coverage")
    };

    // About 48 km west, a couple of hundred metres above the local floor: the crater wall
    // stands in between.
    let blocked = check(site_h_m + 600.0);
    assert!(matches!(
        blocked,
        Some(AccessRestrictionReason::OccultedByTerrain { .. })
    ));
    // Four kilometres above the rim: nothing in between.
    assert!(check(4_000.0).is_none());
}

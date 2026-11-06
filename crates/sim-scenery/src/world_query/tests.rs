use sim_geo::{egm96_undulation_m, geodetic_to_ecef};

use super::*;
use crate::{AerodromeId, TerrariumTile, TileId};

const TILE: TileId = TileId {
    zoom: 12,
    x: 1189,
    y: 1565,
};

fn flat_catalog(height_m: f64, datum: CatalogVerticalDatum) -> CatalogScenery {
    let mut catalog = CatalogScenery::new().with_vertical_datum(datum);
    let tile = TerrariumTile::from_heights(8, 8, vec![height_m; 64]).unwrap();
    catalog.insert_elevation(TILE, tile).unwrap();
    catalog
}

fn tile_centre() -> Geodetic {
    crate::tile_point_to_geodetic(TILE, 0.5, 0.5).unwrap()
}

/// `position` moved `east_m` and `north_m` on a sphere, close enough for
/// tests that only compare neighbouring samples.
fn offset(position: &Geodetic, east_m: f64, north_m: f64) -> Geodetic {
    Geodetic {
        lat_rad: position.lat_rad + north_m / 6_371_000.0,
        lon_rad: position.lon_rad + east_m / (6_371_000.0 * position.lat_rad.cos()),
        height_m: position.height_m,
    }
}

fn dover(
    datum: CatalogVerticalDatum,
    runway_m: Option<f64>,
) -> (AuthoritativeWorldSurface, Geodetic, f64) {
    let mut diagram = OsmAerodromeDiagram::builtin(AerodromeId::DoverKdov).unwrap();
    if let Some(height) = runway_m {
        for end in &mut diagram.surface_graph.runway_ends {
            end.threshold.ellipsoidal_height_m = height;
        }
    }
    let end = *diagram.primary_runway_end().unwrap();
    let catalog = CatalogScenery::synthetic_aerodrome(AerodromeId::DoverKdov)
        .unwrap()
        .with_vertical_datum(datum);
    let surface = AuthoritativeWorldSurface::earth_with_catalog(catalog).with_aerodrome(&diagram);
    let threshold =
        Geodetic::from_degrees(end.threshold.latitude_deg, end.threshold.longitude_deg, 0.0);
    let (sin, cos) = end.true_bearing_rad.sin_cos();
    let half = end.length_m * 0.5;
    let middle = offset(&threshold, half * sin, half * cos);
    (surface, middle, end.true_bearing_rad)
}

/// Ellipsoidal heights every metre from the centreline out across the
/// shoulder and skirt.
fn transect(surface: &AuthoritativeWorldSurface, middle: &Geodetic, bearing: f64) -> Vec<f64> {
    let (sin, cos) = bearing.sin_cos();
    (0..200)
        .map(|metre| {
            let across = f64::from(metre);
            let point = offset(middle, across * cos, -across * sin);
            let sample = surface.sample_surface(&point).unwrap().unwrap();
            sample.ellipsoidal_height_m.0
        })
        .collect()
}

#[test]
fn a_runway_meets_its_dem_without_a_datum_step_in_either_datum() {
    for datum in [
        CatalogVerticalDatum::EllipsoidWgs84,
        CatalogVerticalDatum::OrthometricMslEgm96,
    ] {
        let (surface, middle, bearing) = dover(datum, None);
        let on_runway = surface.sample_surface(&middle).unwrap().unwrap();
        assert_eq!(on_runway.source, SurfaceSampleSource::RunwaySurface);
        let heights = transect(&surface, &middle, bearing);
        let worst = heights
            .windows(2)
            .map(|pair| (pair[1] - pair[0]).abs())
            .fold(0.0, f64::max);
        assert!(worst < 0.01, "{datum:?}: {worst} m step");

        let (ellipsoidal, msl) = (
            on_runway.ellipsoidal_height_m.0,
            on_runway.orthometric_msl_m.0,
        );
        let undulation = egm96_undulation_m(middle.lat_rad, middle.lon_rad);
        assert!((ellipsoidal - msl - undulation).abs() < 1e-9);
        let raw = match datum {
            CatalogVerticalDatum::EllipsoidWgs84 => ellipsoidal,
            CatalogVerticalDatum::OrthometricMslEgm96 => msl,
        };
        assert!((raw - 8.81).abs() < 0.05, "{datum:?}: {raw}");
    }
}

#[test]
fn a_runway_above_its_terrain_is_feathered_in_over_the_skirt() {
    let (surface, middle, bearing) = dover(CatalogVerticalDatum::EllipsoidWgs84, Some(20.0));
    let heights = transect(&surface, &middle, bearing);
    assert!((heights[0] - 20.0).abs() < 1e-9);
    let edge = heights[199];
    assert!((edge - 8.81).abs() < 0.05, "{edge}");
    let steepest = heights
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).abs())
        .fold(0.0, f64::max);
    // Smoothstep over 40 m peaks at 1.5 / 40 of the 11.2 m rise per metre.
    assert!(steepest < 0.45, "{steepest}");
    assert!(heights.windows(2).all(|pair| pair[1] <= pair[0] + 1e-9));
}

#[test]
fn heights_convert_by_the_geoid_in_the_named_datum() {
    let centre = tile_centre();
    let undulation = egm96_undulation_m(centre.lat_rad, centre.lon_rad);
    let msl = AuthoritativeWorldSurface::earth_with_catalog(flat_catalog(
        10.0,
        CatalogVerticalDatum::OrthometricMslEgm96,
    ));
    let sample = msl.sample_surface(&centre).unwrap().unwrap();
    assert_eq!(sample.orthometric_msl_m.0, 10.0);
    assert!((sample.ellipsoidal_height_m.0 - (10.0 + undulation)).abs() < 1e-9);
    assert_eq!(sample.source, SurfaceSampleSource::ResidentDem { zoom: 12 });
    assert_eq!(
        msl.ground_height_m(&centre).unwrap(),
        Some(sample.ellipsoidal_height_m.0)
    );

    let ellipsoidal = AuthoritativeWorldSurface::earth_with_catalog(flat_catalog(
        10.0,
        CatalogVerticalDatum::EllipsoidWgs84,
    ));
    let sample = ellipsoidal.sample_surface(&centre).unwrap().unwrap();
    assert_eq!(sample.ellipsoidal_height_m.0, 10.0);
    assert!((sample.orthometric_msl_m.0 - (10.0 - undulation)).abs() < 1e-9);
}

#[test]
fn level_ground_points_straight_up_and_a_slope_tilts_away_from_the_rise() {
    let centre = tile_centre();
    let flat = AuthoritativeWorldSurface::earth_with_catalog(flat_catalog(
        50.0,
        CatalogVerticalDatum::EllipsoidWgs84,
    ));
    let sample = flat.sample_surface(&centre).unwrap().unwrap();
    assert!((sample.normal_enu - DVec3::Z).length() < 1e-9);
    assert!((sample.normal_ecef - LocalFrame::at(&centre).up()).length() < 1e-9);

    // Columns rise 10 m each going east across an 8 px tile.
    let heights = (0..64).map(|i| f64::from(i % 8) * 10.0).collect();
    let mut catalog =
        CatalogScenery::new().with_vertical_datum(CatalogVerticalDatum::EllipsoidWgs84);
    catalog
        .insert_elevation(TILE, TerrariumTile::from_heights(8, 8, heights).unwrap())
        .unwrap();
    let sloped = AuthoritativeWorldSurface::earth_with_catalog(catalog);
    let normal = sloped.sample_surface(&centre).unwrap().unwrap().normal_enu;
    assert!(normal.x < -1e-3 && normal.y.abs() < 1e-6, "{normal}");
    assert!((normal.length() - 1.0).abs() < 1e-12);
}

#[test]
fn uncovered_ground_is_unknown_until_a_fallback_is_given() {
    let empty = AuthoritativeWorldSurface::earth(CatalogVerticalDatum::OrthometricMslEgm96);
    let point = Geodetic::from_degrees(10.0, 10.0, 500.0);
    assert_eq!(empty.sample_surface(&point), Ok(None));
    assert_eq!(empty.agl_clearance_m(&point), Ok(None));

    let fallback = empty.with_fallback_elevation_m(100.0);
    let sample = fallback.sample_surface(&point).unwrap().unwrap();
    assert_eq!(sample.source, SurfaceSampleSource::FallbackElevation);
    let agl = fallback.agl_clearance_m(&point).unwrap().unwrap();
    assert!((agl.0 - (500.0 - sample.ellipsoidal_height_m.0)).abs() < 1e-9);
}

#[test]
fn every_change_moves_the_world_version() {
    let mut versions = Vec::new();
    let surface = AuthoritativeWorldSurface::earth(CatalogVerticalDatum::EllipsoidWgs84);
    versions.push(surface.world_version());
    let surface = surface.with_fallback_elevation_m(3.0);
    versions.push(surface.world_version());
    let surface = surface.with_catalog(flat_catalog(1.0, CatalogVerticalDatum::EllipsoidWgs84));
    versions.push(surface.world_version());
    let diagram = OsmAerodromeDiagram::builtin(AerodromeId::DoverKdov).unwrap();
    let surface = surface.with_aerodrome(&diagram);
    versions.push(surface.world_version());
    assert!(
        versions.windows(2).all(|pair| pair[1] != pair[0]),
        "{versions:?}"
    );
    let sample = surface.sample_surface(&tile_centre()).unwrap().unwrap();
    assert_eq!(sample.world_version, surface.world_version());
}

#[test]
fn a_ray_finds_the_terrain_to_well_under_a_metre() {
    let centre = tile_centre();
    let surface = AuthoritativeWorldSurface::earth_with_catalog(flat_catalog(
        100.0,
        CatalogVerticalDatum::EllipsoidWgs84,
    ));
    let above = geodetic_to_ecef(&Geodetic {
        height_m: 1_000.0,
        ..centre
    });
    let below = geodetic_to_ecef(&Geodetic {
        height_m: -50.0,
        ..centre
    });
    let hit = surface
        .raycast_segment_ecef(above, below, 16)
        .unwrap()
        .unwrap();
    assert!((hit.distance_m - 900.0).abs() < 0.5, "{}", hit.distance_m);
    assert!((hit.geodetic.height_m - 100.0).abs() < 0.5);
    assert_eq!(
        hit.surface.source,
        SurfaceSampleSource::ResidentDem { zoom: 12 }
    );

    let slant_end = geodetic_to_ecef(&Geodetic {
        height_m: 0.0,
        ..offset(&centre, 1_500.0, 0.0)
    });
    let slant = surface
        .raycast_segment_ecef(above, slant_end, 64)
        .unwrap()
        .unwrap();
    assert!(
        (slant.geodetic.height_m - 100.0).abs() < 0.5,
        "{:?}",
        slant.geodetic
    );

    let up = geodetic_to_ecef(&Geodetic {
        height_m: 5_000.0,
        ..centre
    });
    assert_eq!(surface.raycast_segment_ecef(above, up, 4), Ok(None));
    let started_below = surface
        .raycast_segment_ecef(below, above, 4)
        .unwrap()
        .unwrap();
    assert_eq!(started_below.distance_m, 0.0);
}

#[test]
fn a_ray_over_unknown_ground_stops_at_the_ellipsoid() {
    let surface = AuthoritativeWorldSurface::earth(CatalogVerticalDatum::OrthometricMslEgm96);
    let site = Geodetic::from_degrees(-20.0, 60.0, 0.0);
    let above = geodetic_to_ecef(&Geodetic {
        height_m: 2_000.0,
        ..site
    });
    let below = geodetic_to_ecef(&Geodetic {
        height_m: -2_000.0,
        ..site
    });
    let hit = surface
        .raycast_segment_ecef(above, below, 4)
        .unwrap()
        .unwrap();
    assert!(
        hit.geodetic.height_m.abs() < 0.5,
        "{}",
        hit.geodetic.height_m
    );
    assert_eq!(hit.surface.source, SurfaceSampleSource::ReferenceEllipsoid);
}

#[test]
fn the_moon_answers_from_the_apollo_site_on_its_own_sphere() {
    let scenery = LunarTranquilityScenery::apollo_11_landing();
    let surface = AuthoritativeWorldSurface::moon(scenery);
    assert_eq!(surface.vertical_datum(), None);
    let site = scenery.at_local(120.0, -40.0);
    let sample = surface.sample_surface(&site).unwrap().unwrap();
    assert_eq!(sample.source, SurfaceSampleSource::LunarProcedural);
    assert_eq!(sample.ellipsoidal_height_m.0, sample.orthometric_msl_m.0);
    assert!(sample.normal_enu.z > 0.9);

    let moon = BodyEllipsoid::MOON_IAU;
    let above = moon.geodetic_to_fixed(&Geodetic {
        height_m: 500.0,
        ..site
    });
    let below = moon.geodetic_to_fixed(&Geodetic {
        height_m: -500.0,
        ..site
    });
    let hit = surface
        .raycast_segment_ecef(above, below, 32)
        .unwrap()
        .unwrap();
    let ground = sample.ellipsoidal_height_m.0;
    assert!(
        (hit.geodetic.height_m - ground).abs() < 0.5,
        "{hit:?} vs {ground}"
    );
    assert!((hit.distance_m - (500.0 - ground)).abs() < 0.5);
}

#[test]
fn nzsp_runway_strip_and_south_pole_normals_stay_flat_and_finite() {
    let (_, nzsp) = crate::AirportDatabase::builtin().by_code("NZSP").unwrap();
    let r02t = &nzsp.runways[0].ends[0];
    let origin = Geodetic::from_degrees(r02t.lat_deg, r02t.lon_deg, 0.0);
    let frame = LocalFrame::at(&origin);
    let (sin_c, cos_c) = f64::from(r02t.true_heading_deg).to_radians().sin_cos();
    let along_dir = frame.north() * cos_c + frame.east() * sin_c;
    let across_dir = frame.east() * cos_c - frame.north() * sin_c;
    let point_at = |along_m: f64, across_m: f64| {
        sim_geo::ecef_to_geodetic(
            geodetic_to_ecef(&origin) + along_dir * along_m + across_dir * across_m,
        )
    };

    for datum in [
        CatalogVerticalDatum::EllipsoidWgs84,
        CatalogVerticalDatum::OrthometricMslEgm96,
    ] {
        let mut surface =
            AuthoritativeWorldSurface::earth(datum).with_fallback_elevation_m(2_800.0);
        surface.set_extra_runways(nzsp.runway_strips(datum));

        for along_m in [0.0, 1_829.0, 3_658.0] {
            let pt = point_at(along_m, 0.0);
            let sample = surface.sample_surface(&pt).unwrap().unwrap();
            assert_eq!(sample.source, SurfaceSampleSource::RunwaySurface);
            let charted = match datum {
                CatalogVerticalDatum::EllipsoidWgs84 => sample.ellipsoidal_height_m.0,
                CatalogVerticalDatum::OrthometricMslEgm96 => sample.orthometric_msl_m.0,
            };
            assert!(
                (charted - 2_835.0).abs() < 1e-4,
                "{datum:?} {along_m}: {charted}"
            );
            assert!(sample.normal_enu.is_finite());
            assert!(
                (sample.normal_enu - DVec3::Z).length() < 1e-4,
                "{}",
                sample.normal_enu
            );
        }

        // Far end stays uncurved: ±20 m across is on pavement; ±120 m is outside the skirt.
        let edge_in = surface
            .sample_surface(&point_at(3_658.0, 20.0))
            .unwrap()
            .unwrap();
        assert_eq!(edge_in.source, SurfaceSampleSource::RunwaySurface);
        let edge_out = surface
            .sample_surface(&point_at(3_658.0, 120.0))
            .unwrap()
            .unwrap();
        assert_eq!(edge_out.source, SurfaceSampleSource::FallbackElevation);

        for lon_deg in [0.0, 139.2729, -45.0] {
            let pole = Geodetic::from_degrees(-90.0, lon_deg, 0.0);
            let sample = surface.sample_surface(&pole).unwrap().unwrap();
            assert!(sample.normal_enu.is_finite());
            assert!(
                (sample.normal_enu - DVec3::Z).length() < 1e-4,
                "{}",
                sample.normal_enu
            );
        }
    }
}

mod datum;

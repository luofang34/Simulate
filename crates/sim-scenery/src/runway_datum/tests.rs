use sim_geo::{Geodetic, ecef_to_geodetic, geodetic_to_ecef};

use super::{MIN_STAMP_ZOOM, RunwayDatum};
use crate::{AirportDatabase, AirportRecord, TerrariumTile, project_to_tile};

fn airport(code: &str) -> &'static AirportRecord {
    AirportDatabase::builtin().by_code(code).unwrap().1
}

/// Each runway end's point `along` of the way to the other end, with the charted height
/// there.
fn runway_points(record: &AirportRecord, along: f64) -> Vec<(Geodetic, f64)> {
    record
        .runways
        .iter()
        .flat_map(|runway| {
            let [a, b] = &runway.ends;
            [(a, b), (b, a)]
        })
        .map(|(end, other)| {
            // Interpolated in ECEF so a runway across a pole stays straight.
            let from = geodetic_to_ecef(&Geodetic::from_degrees(end.lat_deg, end.lon_deg, 0.0));
            let to = geodetic_to_ecef(&Geodetic::from_degrees(other.lat_deg, other.lon_deg, 0.0));
            let point = Geodetic {
                height_m: 0.0,
                ..ecef_to_geodetic(from + (to - from) * along)
            };
            let charted = f64::from(end.elevation_msl_m)
                + f64::from(other.elevation_msl_m - end.elevation_msl_m) * along;
            (point, charted)
        })
        .collect()
}

fn flat_tile(height_m: f64) -> TerrariumTile {
    TerrariumTile::from_heights(256, 256, vec![height_m; 256 * 256]).unwrap()
}

#[test]
fn every_runway_point_reads_its_charted_height_at_full_weight() {
    let datum = RunwayDatum::builtin();
    for code in ["KEWR", "47N", "KDEN", "VCBI", "EGLL", "NZSP", "RJTT"] {
        for along in [0.0, 0.3, 0.5, 0.9] {
            for (point, charted) in runway_points(airport(code), along) {
                let (height, weight) = datum
                    .height_and_weight(&point)
                    .unwrap_or_else(|| panic!("{code} {along} {point:?} off every strip"));
                assert_eq!(weight, 1.0, "{code} {along}");
                assert!(
                    (height - charted).abs() < 0.02,
                    "{code} {along}: {height} vs {charted}"
                );
            }
        }
    }
}

#[test]
fn ground_far_from_any_airport_is_left_alone() {
    let datum = RunwayDatum::builtin();
    let pacific = Geodetic::from_degrees(-30.0, -140.0, 0.0);
    assert_eq!(datum.height_and_weight(&pacific), None);
    assert_eq!(datum.blend(&pacific, 12.5), 12.5);
    let tile = project_to_tile(&pacific, 12).unwrap().tile;
    assert!(!datum.touches_tile(tile));
    let mut dem = flat_tile(-4_000.0);
    assert!(!datum.stamp_terrarium(tile, &mut dem));
}

/// A stamped tile reads the charted height on the runway whether a reader puts a
/// pixel's height at its centre (the scenery crate) or at its corner (maplibre's GL JS
/// convention): pavement and shoulders are flat across more than a pixel.
#[test]
fn stamped_tiles_carry_the_charted_runway_in_either_pixel_convention() {
    let datum = RunwayDatum::builtin();
    for code in ["KEWR", "KDEN", "VCBI"] {
        for (point, charted) in runway_points(airport(code), 0.5)
            .into_iter()
            .chain(runway_points(airport(code), 0.05))
        {
            for zoom in [MIN_STAMP_ZOOM + 4, 15] {
                let coord = project_to_tile(&point, zoom).unwrap();
                let mut dem = flat_tile(charted + 3.0);
                assert!(datum.touches_tile(coord.tile), "{code} z{zoom}");
                assert!(
                    datum.stamp_terrarium(coord.tile, &mut dem),
                    "{code} z{zoom}"
                );
                let centre = dem.sample_bilinear(coord.u, coord.v).unwrap();
                let corner_u = (coord.u * 256.0 + 0.5) / 256.0;
                let corner_v = (coord.v * 256.0 + 0.5) / 256.0;
                let corner = dem.sample_bilinear(corner_u, corner_v).unwrap();
                for (convention, height) in [("centre", centre), ("corner", corner)] {
                    assert!(
                        (height - charted).abs() < 0.05,
                        "{code} z{zoom} {convention}: {height} vs {charted}"
                    );
                }
            }
        }
    }
}

#[test]
fn coarse_tiles_are_not_stamped() {
    let datum = RunwayDatum::builtin();
    let (point, _) = runway_points(airport("KEWR"), 0.5)[0];
    let tile = project_to_tile(&point, MIN_STAMP_ZOOM - 1).unwrap().tile;
    assert!(!datum.touches_tile(tile));
}

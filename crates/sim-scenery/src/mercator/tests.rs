use super::*;

fn at(lat_deg: f64, lon_deg: f64) -> Geodetic {
    Geodetic::from_degrees(lat_deg, lon_deg, 0.0)
}

#[test]
fn zoom_zero_is_one_tile_centred_on_the_origin() {
    let coord = project_to_tile(&at(0.0, 0.0), 0).unwrap();
    assert_eq!((coord.tile.x, coord.tile.y), (0, 0));
    assert!((coord.u - 0.5).abs() < 1e-12 && (coord.v - 0.5).abs() < 1e-12);
}

#[test]
fn matches_the_openstreetmap_slippy_map_formula() {
    // Addresses computed separately from the wiki's ln(tan + sec) form.
    let dover = project_to_tile(&at(39.1582, -75.5244), 12).unwrap();
    assert_eq!((dover.tile.x, dover.tile.y), (1188, 1563));
    let berlin = project_to_tile(&at(52.51628, 13.37771), 16).unwrap();
    assert_eq!((berlin.tile.x, berlin.tile.y), (35203, 21493));
}

#[test]
fn round_trips_through_the_tile_point() {
    for (lat, lon) in [(39.1, -75.5), (-33.9, 151.2), (84.9, 179.9), (0.5, -0.5)] {
        let coord = project_to_tile(&at(lat, lon), 14).unwrap();
        let back = tile_point_to_geodetic(coord.tile, coord.u, coord.v).unwrap();
        assert!((back.lat_deg() - lat).abs() < 1e-9, "{lat}");
        assert!((back.lon_deg() - lon).abs() < 1e-9, "{lon}");
    }
}

#[test]
fn latitudes_beyond_the_projection_have_no_tile() {
    assert!(project_to_tile(&at(85.06, 0.0), 3).is_none());
    assert!(project_to_tile(&at(-90.0, 0.0), 3).is_none());
    assert!(project_to_tile(&at(f64::NAN, 0.0), 3).is_none());
    assert!(project_to_tile(&at(85.05, 0.0), 3).is_some());
}

#[test]
fn the_map_edges_belong_to_the_outermost_tiles() {
    let top = project_to_tile(&at(MAX_MERCATOR_LAT_DEG, 0.0), 4).unwrap();
    assert_eq!(top.tile.y, 0);
    let bottom = project_to_tile(&at(-MAX_MERCATOR_LAT_DEG, 0.0), 4).unwrap();
    assert_eq!(bottom.tile.y, 15);
}

#[test]
fn longitude_wraps_at_the_antimeridian() {
    let west = project_to_tile(&at(10.0, -180.0), 4).unwrap();
    let east = project_to_tile(&at(10.0, 180.0), 4).unwrap();
    assert_eq!(west.tile, east.tile);
    assert_eq!(west.tile.x, 0);
}

#[test]
fn unaddressable_tiles_are_rejected() {
    let beyond = TileId {
        zoom: 2,
        x: 4,
        y: 0,
    };
    assert!(!beyond.is_valid());
    assert!(tile_point_to_geodetic(beyond, 0.0, 0.0).is_none());
    assert!(project_to_tile(&at(0.0, 0.0), MAX_TILE_ZOOM + 1).is_none());
    assert!(
        !TileId {
            zoom: 40,
            x: 0,
            y: 0
        }
        .is_valid()
    );
}

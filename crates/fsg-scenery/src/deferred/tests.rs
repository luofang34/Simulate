use super::*;
use crate::{ElevationSource, EllipsoidSurface};
use fsg_geo::Geodetic;

#[test]
fn deferred_sources_fail_naming_the_capability() {
    let tile = TileId {
        zoom: 3,
        x: 1,
        y: 2,
    };
    let imagery = Deferred::IMAGERY.imagery(tile).unwrap_err();
    assert_eq!(
        imagery,
        SceneryError::Deferred {
            capability: "global imagery"
        }
    );
    assert!(imagery.to_string().contains("global imagery"));
    let mesh = Deferred::PHOTOGRAMMETRY.node(&[0, 3]).unwrap_err();
    assert_eq!(
        mesh,
        SceneryError::Deferred {
            capability: "photogrammetry"
        }
    );
}

#[test]
fn ellipsoid_surface_reports_known_zero_height() {
    let height = EllipsoidSurface.ground_height_m(&Geodetic::from_degrees(10.0, 20.0, 0.0));
    assert_eq!(height, Ok(Some(0.0)));
}

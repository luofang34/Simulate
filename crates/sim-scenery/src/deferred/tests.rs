use super::*;
use crate::{ElevationSource, EllipsoidSurface};
use sim_geo::Geodetic;

#[test]
fn deferred_source_fails_naming_the_capability() {
    let mesh = Deferred::PHOTOGRAMMETRY.node(&[0, 3]).unwrap_err();
    assert_eq!(
        mesh,
        SceneryError::Deferred {
            capability: "photogrammetry"
        }
    );
    assert!(mesh.to_string().contains("photogrammetry"));
}

#[test]
fn ellipsoid_surface_reports_known_zero_height() {
    let height = EllipsoidSurface.ground_height_m(&Geodetic::from_degrees(10.0, 20.0, 0.0));
    assert_eq!(height, Ok(Some(0.0)));
}

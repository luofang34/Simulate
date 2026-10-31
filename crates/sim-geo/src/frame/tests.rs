use super::*;
use crate::geodetic_to_ecef;

#[test]
fn axes_are_orthonormal_and_right_handed() {
    let frame = LocalFrame::at(&Geodetic::from_degrees(47.45, -122.31, 130.0));
    assert!((frame.east().length() - 1.0).abs() < 1e-12);
    assert!(frame.east().dot(frame.north()).abs() < 1e-12);
    assert!((frame.east().cross(frame.north()) - frame.up()).length() < 1e-12);
}

#[test]
fn north_points_toward_increasing_latitude() {
    let here = Geodetic::from_degrees(10.0, 20.0, 0.0);
    let frame = LocalFrame::at(&here);
    let step =
        geodetic_to_ecef(&Geodetic::from_degrees(10.001, 20.0, 0.0)) - geodetic_to_ecef(&here);
    let enu = frame.ecef_to_enu_vector(step);
    assert!(
        enu.y > 100.0 && enu.x.abs() < 1e-6 && enu.z.abs() < 0.1,
        "{enu:?}"
    );
}

#[test]
fn ned_down_is_negative_up() {
    let frame = LocalFrame::at(&Geodetic::from_degrees(-33.9, 151.2, 0.0));
    let down = frame.ecef_to_ned_vector(-frame.up());
    assert!((down - DVec3::Z).length() < 1e-12);
}

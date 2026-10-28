use super::*;
use crate::{Geodetic, geodetic_to_ecef};

#[test]
fn local_offsets_keep_millimetre_precision_far_from_the_earth_centre() {
    let anchor = geodetic_to_ecef(&Geodetic::from_degrees(51.47, -0.45, 25.0));
    let origin = FloatingOrigin::new(anchor);
    let offset = DVec3::new(1_234.567_8, -0.001, 15.25);
    let local = origin.to_local(anchor + offset);
    assert!((local.as_dvec3() - offset).length() < 1e-4, "{local:?}");

    let absolute = (anchor + offset).as_vec3().as_dvec3() - anchor.as_vec3().as_dvec3();
    assert!(
        (absolute - offset).length() > 1e-2,
        "f32 absolute coordinates unexpectedly precise"
    );
}

#[test]
fn rebases_only_beyond_threshold() {
    let mut origin = FloatingOrigin::new(DVec3::ZERO);
    assert!(!origin.rebase_if_far(DVec3::new(999.0, 0.0, 0.0), 1_000.0));
    assert_eq!(origin.anchor(), DVec3::ZERO);
    assert!(origin.rebase_if_far(DVec3::new(1_001.0, 0.0, 0.0), 1_000.0));
    assert_eq!(origin.anchor(), DVec3::new(1_001.0, 0.0, 0.0));
}

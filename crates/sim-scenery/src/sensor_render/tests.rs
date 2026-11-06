use glam::DMat3;
use sim_geo::{Geodetic, geodetic_to_ecef};

use super::*;
use crate::{AuthoritativeWorldSurface, SyntheticCorridorSpec, WorldSurfaceQuery};

const INTRINSICS: SensorIntrinsics = SensorIntrinsics {
    width: 33,
    height: 25,
    fx: 30.0,
    fy: 30.0,
    cx: 16.0,
    cy: 12.0,
};

/// Looking straight down with the image top towards north.
fn nadir(at: &Geodetic) -> DQuat {
    let frame = LocalFrame::at(at);
    DQuat::from_mat3(&DMat3::from_cols(frame.east(), frame.north(), frame.up()))
}

fn scene() -> (CatalogScenery, Geodetic, DVec3) {
    let spec = SyntheticCorridorSpec::delaware();
    let catalog = CatalogScenery::synthetic_corridor(&spec).unwrap();
    let below = Geodetic::from_degrees(39.07, -75.66, 0.0);
    let ground = catalog
        .ellipsoidal_ground_height_m(&below)
        .unwrap()
        .unwrap()
        .0;
    let eye = Geodetic {
        height_m: ground + 600.0,
        ..below
    };
    (catalog, eye, geodetic_to_ecef(&eye))
}

#[test]
fn every_pixel_shows_the_imagery_where_its_ray_meets_the_dem() {
    let (catalog, eye, eye_ecef) = scene();
    let orientation = nadir(&eye);
    let image = render_luma_view(&catalog, eye_ecef, orientation, &INTRINSICS).unwrap();
    let surface = AuthoritativeWorldSurface::earth_with_catalog(catalog.clone());
    let mut worst = 0.0_f64;
    for (x, y) in [(16, 12), (0, 0), (32, 24), (5, 20), (30, 3)] {
        let ray = orientation
            * DVec3::new(
                (f64::from(x) - INTRINSICS.cx) / INTRINSICS.fx,
                (INTRINSICS.cy - f64::from(y)) / INTRINSICS.fy,
                -1.0,
            );
        let hit = surface
            .raycast_segment_ecef(eye_ecef, eye_ecef + ray.normalize() * 3_000.0, 256)
            .unwrap()
            .unwrap();
        let expected = catalog.imagery_luma_at(&hit.geodetic).unwrap();
        let got = f64::from(image.at(x, y).unwrap());
        worst = worst.max((got - expected).abs());
    }
    // Luma is rounded to a level, and bilinear imagery changes little over
    // the sub-metre left between the two intersections.
    assert!(worst <= 3.0, "{worst}");
}

#[test]
fn renders_repeat_bit_for_bit_and_the_sky_is_black() {
    let (catalog, eye, eye_ecef) = scene();
    let down = render_luma_view(&catalog, eye_ecef, nadir(&eye), &INTRINSICS).unwrap();
    let again = render_luma_view(&catalog, eye_ecef, nadir(&eye), &INTRINSICS).unwrap();
    assert_eq!(down, again);
    assert!(down.pixels().iter().any(|&luma| luma > 0));
    let up = nadir(&eye) * DQuat::from_rotation_x(std::f64::consts::PI);
    let sky = render_luma_view(&catalog, eye_ecef, up, &INTRINSICS).unwrap();
    assert!(sky.pixels().iter().all(|&luma| luma == 0));
}

#[test]
fn bilinear_sampling_interpolates_between_pixel_centres() {
    let image = LumaImage::new(2, 2, vec![0, 100, 50, 150]).unwrap();
    assert_eq!(image.sample_bilinear(0.5, 0.0), Some(50.0));
    assert_eq!(image.sample_bilinear(0.5, 0.5), Some(75.0));
    assert_eq!(image.sample_bilinear(1.0, 1.0), Some(150.0));
    assert_eq!(image.sample_bilinear(1.01, 0.0), None);
    assert!(LumaImage::new(2, 2, vec![0; 3]).is_err());
}

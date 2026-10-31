use super::*;
use glam::Vec4;

fn camera_looking_down_x() -> Camera {
    Camera {
        pose: CameraPose::looking(DVec3::ZERO, DVec3::X, DVec3::Z),
        fov_y_rad: 60_f64.to_radians(),
        near_m: 0.05,
    }
}

fn clip(camera: &Camera, relative: glam::Vec3) -> glam::Vec3 {
    let clip =
        camera.view_projection(16.0 / 9.0) * Vec4::new(relative.x, relative.y, relative.z, 1.0);
    clip.truncate() / clip.w
}

#[test]
fn forward_point_projects_to_screen_centre() {
    let ndc = clip(&camera_looking_down_x(), glam::Vec3::new(100.0, 0.0, 0.0));
    assert!(ndc.x.abs() < 1e-6 && ndc.y.abs() < 1e-6, "{ndc:?}");
}

#[test]
fn up_vector_projects_upward() {
    let ndc = clip(&camera_looking_down_x(), glam::Vec3::new(100.0, 0.0, 10.0));
    assert!(ndc.y > 0.0, "{ndc:?}");
}

#[test]
fn depth_is_reversed_and_stays_positive_at_planet_distances() {
    let camera = camera_looking_down_x();
    let near = clip(&camera, glam::Vec3::new(1.0, 0.0, 0.0)).z;
    let far = clip(&camera, glam::Vec3::new(5_000_000.0, 0.0, 0.0)).z;
    assert!(near > far, "near {near} far {far}");
    assert!(far > 0.0 && near <= 1.0, "near {near} far {far}");
}

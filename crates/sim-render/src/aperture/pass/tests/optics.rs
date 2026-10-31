use glam::DVec3;
use sim_geo::{BodyEllipsoid, InertialTransform, UtcTime, nav_star_by_octal_id};

use super::{HEIGHT, WIDTH, gpu, render};
use crate::aperture::{ApertureFrame, ApertureWaveband, OcclusionOverlayConfig, OpticalReticle};
use crate::{CameraPose, GlobeMesh, SceneLighting, TerrainRegion, TerrainSources};

fn inertial() -> InertialTransform {
    InertialTransform::moon_at(UtcTime::from_unix_seconds(-14_182_940.0))
}

/// Looks along the fixed-frame direction of a star from `eye`.
fn looking_at_star(eye: DVec3, octal_id: u8, inertial: &InertialTransform) -> CameraPose {
    let star = nav_star_by_octal_id(octal_id).unwrap();
    let forward = inertial.j2000_to_fixed(star.direction_j2000);
    CameraPose::looking(eye, forward, forward.cross(DVec3::Z).cross(forward))
}

fn starfield(pose: CameraPose, reticle: Option<OpticalReticle>) -> ApertureFrame {
    ApertureFrame {
        pose,
        lighting: SceneLighting {
            sun_direction_ecef: -pose.orientation * DVec3::NEG_Z,
        },
        waveband: ApertureWaveband::StarfieldOptical {
            inertial: inertial(),
            reticle,
        },
        occlusion: OcclusionOverlayConfig::default(),
    }
}

fn srgb_byte_to_linear(byte: u8) -> f32 {
    let encoded = f32::from(byte) / 255.0;
    if encoded <= 0.04045 {
        encoded / 12.92
    } else {
        ((encoded + 0.055) / 1.055).powf(2.4)
    }
}

fn moon() -> GlobeMesh {
    let region = TerrainRegion::whole_body(BodyEllipsoid::MOON_IAU, 12, 24, 8);
    GlobeMesh::build_region(&region, TerrainSources::default()).unwrap()
}

#[test]
fn a_star_lands_on_its_projected_pixel_against_black_space() {
    let Some(gpu) = gpu() else { return };
    let inertial = inertial();
    let forward = inertial.j2000_to_fixed(nav_star_by_octal_id(0o17).unwrap().direction_j2000);
    // Out beyond the Moon, looking away from it.
    let pose = looking_at_star(forward * 3_000_000.0, 0o17, &inertial);
    let image = render(&gpu, &moon(), None, 2_000.0, &starfield(pose, None));
    let magnitude = nav_star_by_octal_id(0o17).unwrap().visual_magnitude;
    let at_centre = 10.0_f32.powf(-0.4 * magnitude);
    // The star sits on the corner of four pixels, half a pixel from each
    // centre, with the optic's 1.3 px spread.
    let expected = at_centre * (-0.5_f32 / (2.0 * 1.3 * 1.3)).exp();
    let peak = image.pixel(WIDTH / 2, HEIGHT / 2).unwrap();
    let measured = srgb_byte_to_linear(peak[0]);
    assert!(
        (measured - expected).abs() < 0.01,
        "{measured} vs {expected}"
    );
    assert_eq!(peak[0], peak[2], "white");
    let tail = image.pixel(WIDTH / 2 + 3, HEIGHT / 2).unwrap();
    assert!(tail[0] < peak[0], "the image of a star spreads and fades");
    assert_eq!(image.pixel(10, 10).unwrap(), [0, 0, 0, 255], "black space");
}

#[test]
fn the_limb_hides_the_stars_behind_it() {
    let Some(gpu) = gpu() else { return };
    let inertial = inertial();
    let forward = inertial.j2000_to_fixed(nav_star_by_octal_id(0o17).unwrap().direction_j2000);
    // The Moon's centre is on the line of sight to the star.
    let pose = looking_at_star(forward * -3_000_000.0, 0o17, &inertial);
    let image = render(&gpu, &moon(), None, 2_000.0, &starfield(pose, None));
    assert!(image.depth_at(WIDTH / 2, HEIGHT / 2).unwrap() > 0.0);
    let hidden = image.pixel(WIDTH / 2, HEIGHT / 2).unwrap();
    assert!(hidden[0] < 120, "no star through the Moon {hidden:?}");
}

#[test]
fn the_lpd_scale_marks_its_angles_below_the_boresight() {
    let Some(gpu) = gpu() else { return };
    let inertial = inertial();
    let pose = looking_at_star(DVec3::new(0.0, 0.0, 9e6), 0o17, &inertial);
    let reticle = OpticalReticle::Lpd {
        mark_spacing_deg: 10.0,
    };
    let fx = 150.0;
    let image = render(&gpu, &moon(), None, fx, &starfield(pose, Some(reticle)));
    let row_at = |deg: f64| (119.5 + fx * deg.to_radians().tan()).round() as u32;
    let beside_scale = WIDTH / 2 + 6;
    let is_amber = |pixel: [u8; 4]| pixel[0] > 200 && pixel[2] < 120;
    for deg in [10.0, 20.0, 30.0] {
        let mark = image.pixel(beside_scale, row_at(deg)).unwrap();
        assert!(is_amber(mark), "{deg}° {mark:?}");
    }
    let between = image.pixel(beside_scale, row_at(15.0)).unwrap();
    assert!(!is_amber(between), "{between:?}");
    let above = image.pixel(WIDTH / 2, 40).unwrap();
    assert!(!is_amber(above), "the scale runs below the boresight only");
}

#[test]
fn each_reticle_draws_on_the_boresight_region() {
    let Some(gpu) = gpu() else { return };
    let inertial = inertial();
    let pose = looking_at_star(DVec3::new(0.0, 0.0, 9e6), 0o17, &inertial);
    let amber_pixels = |reticle| {
        let image = render(&gpu, &moon(), None, 150.0, &starfield(pose, Some(reticle)));
        image
            .rgba8
            .as_chunks::<4>()
            .0
            .iter()
            .filter(|pixel| pixel[0] > 200 && pixel[2] < 120)
            .count()
    };
    let sxt = amber_pixels(OpticalReticle::Sxt);
    let aot = amber_pixels(OpticalReticle::Aot {
        detent: 1,
        reticle_rotation_rad: 0.3,
    });
    assert!(sxt > 1000, "{sxt}");
    assert!(aot > sxt / 2, "{aot}");
}

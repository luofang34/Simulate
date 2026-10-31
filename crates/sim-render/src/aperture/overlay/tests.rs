use sim_geo::SensorIntrinsics;

use super::*;

const WIDTH: u32 = 64;
const HEIGHT: u32 = 48;

fn bezel() -> WindowBezel {
    WindowBezel {
        semi_axes_px: [28.0, 20.0],
        vignette_px: 3.0,
        colour: [0.1, 0.1, 0.1],
    }
}

fn propeller() -> PropellerDisk {
    PropellerDisk {
        hub_px: [32.0, 24.0],
        tip_radius_px: 20.0,
        hub_radius_px: 3.0,
        blade_count: 3,
        blade_half_width_rad: 0.2,
        phase_rad: 0.0,
        colour: [0.0, 0.0, 0.0],
    }
}

#[test]
fn nothing_configured_hides_nothing() {
    let mask = OcclusionOverlayConfig::default().ground_truth_mask(WIDTH, HEIGHT);
    assert_eq!(mask.len(), (WIDTH * HEIGHT) as usize);
    assert!(mask.iter().all(|hidden| !hidden));
}

#[test]
fn the_bezel_hides_the_corners_and_not_the_centre() {
    let config = OcclusionOverlayConfig {
        bezel: Some(bezel()),
        propeller: None,
    };
    assert!(config.occludes(WIDTH, HEIGHT, 0.5, 0.5));
    assert!(config.occludes(WIDTH, HEIGHT, 63.5, 47.5));
    assert!(!config.occludes(WIDTH, HEIGHT, 32.0, 24.0));
    assert!(!config.occludes(WIDTH, HEIGHT, 59.0, 24.0));
    assert!(config.occludes(WIDTH, HEIGHT, 61.0, 24.0));
}

#[test]
fn blades_sweep_with_the_phase_and_the_hub_does_not() {
    let at_zero = OcclusionOverlayConfig {
        bezel: None,
        propeller: Some(propeller()),
    };
    let on_blade = (32.0 + 12.0, 24.0);
    let between = (32.0 - 12.0, 24.0);
    assert!(at_zero.occludes(WIDTH, HEIGHT, on_blade.0, on_blade.1));
    assert!(!at_zero.occludes(WIDTH, HEIGHT, between.0, between.1));
    let turned = OcclusionOverlayConfig {
        bezel: None,
        propeller: Some(PropellerDisk {
            phase_rad: TAU / 6.0,
            ..propeller()
        }),
    };
    assert!(!turned.occludes(WIDTH, HEIGHT, on_blade.0, on_blade.1));
    assert!(turned.occludes(WIDTH, HEIGHT, between.0, between.1));
    assert!(turned.occludes(WIDTH, HEIGHT, 32.5, 24.5), "hub");
    assert!(
        !turned.occludes(WIDTH, HEIGHT, 32.0 + 21.0, 24.0),
        "past the tips"
    );
}

#[test]
fn a_bladeless_hub_is_still_solid() {
    let disk = PropellerDisk {
        blade_count: 0,
        ..propeller()
    };
    assert!(disk.occludes(32.0, 25.0));
    assert!(!disk.occludes(32.0 + 12.0, 24.0));
}

#[test]
fn the_mask_is_row_major_at_pixel_centres() {
    let config = OcclusionOverlayConfig {
        bezel: Some(bezel()),
        propeller: None,
    };
    let mask = config.ground_truth_mask(WIDTH, HEIGHT);
    let index = |x: u32, y: u32| (y * WIDTH + x) as usize;
    assert!(mask[index(0, 0)]);
    assert!(!mask[index(32, 24)]);
    assert!(mask[index(32, 2)], "above the ellipse's top");
}

#[test]
fn the_uniform_carries_every_field_in_shader_order() {
    let projection = PinholeProjection::new(
        SensorIntrinsics {
            width: WIDTH,
            height: HEIGHT,
            fx: 50.0,
            fy: 51.0,
            cx: 31.5,
            cy: 23.5,
        },
        1.0,
    )
    .unwrap();
    let config = OcclusionOverlayConfig {
        bezel: Some(bezel()),
        propeller: Some(propeller()),
    };
    let reticle = OpticalReticle::Lpd {
        mark_spacing_deg: 10.0,
    };
    let mut stars = [[0.0; 4]; STAR_SLOTS];
    stars[36] = [1.0, 2.0, 3.0, 4.0];
    let block = overlay_uniform(&projection, &config, Some(&reticle), &stars);
    assert_eq!(block[..2], [64.0, 48.0]);
    assert_eq!(block[4..8], [50.0, 51.0, 32.0, 24.0]);
    assert_eq!(block[8], RETICLE_LPD);
    assert!((block[10] - 10.0_f32.to_radians()).abs() < 1e-7);
    assert_eq!(block[12..16], [28.0, 20.0, 3.0, 1.0]);
    assert_eq!(block[20..24], [32.0, 24.0, 20.0, 1.0]);
    assert_eq!(block[24..28], [3.0, 0.2, 0.0, 3.0]);
    assert_eq!(block[OVERLAY_UNIFORM_FLOATS - 4..], [1.0, 2.0, 3.0, 4.0]);
}

#[test]
fn the_aot_detent_turns_its_pattern_by_sixths() {
    let uniform = OpticalReticle::uniform(Some(&OpticalReticle::Aot {
        detent: 2,
        reticle_rotation_rad: 0.25,
    }));
    assert_eq!(uniform[0], RETICLE_AOT);
    assert!((uniform[1] - (2.0 * TAU / 6.0 + 0.25)).abs() < 1e-6);
    assert_eq!(OpticalReticle::uniform(None)[0], RETICLE_NONE);
    assert_eq!(
        OpticalReticle::uniform(Some(&OpticalReticle::Sxt))[0],
        RETICLE_SXT
    );
}

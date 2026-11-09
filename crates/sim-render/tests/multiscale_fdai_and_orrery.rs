//! Gate 6 verification: 4-Pass Multi-Scale Rendering Contract, cross-pass planetary
//! ridge occlusion vs LEO `0.25 m` docking visibility, Mach/pressure-adaptive Aerospike
//! plume, gimbal-free quaternion `FDAI/Navball`, and 3D `Orrery` projection.

use glam::{DQuat, DVec3};
use sim_render::{
    CelestialBodySample, NEAR_FIELD_USD_Z_FAR_M, NEAR_FIELD_USD_Z_NEAR_M, OrreryBodyNode,
    compute_dynamic_planetary_frustum, encode_finite_reversed_z, evaluate_aerospike_plume_visual,
    evaluate_eclipse_shadow_factor, evaluate_fdai_navball, evaluate_planet_impostor_to_terrain_lod,
    near_field_occluded_by_planet, polar_cap_blend_weight, project_orrery_scene,
    reconstruct_planetary_eye_depth_m, sort_celestial_bodies_back_to_front,
};

#[test]
fn pass1_finite_distance_celestial_back_to_front_sorting_and_eclipse_shadow_verified() {
    // Camera in cislunar space: Moon at 15,000 km, Earth at 360,000 km, Mars at 1.8e11 m, Sun at 1.496e11 m
    let samples = [
        CelestialBodySample {
            body_id: 301,
            pos_rel_eye_m: DVec3::new(15_000_000.0, 0.0, 0.0),
            radius_m: 1_737_400.0,
        },
        CelestialBodySample {
            body_id: 10,
            pos_rel_eye_m: DVec3::new(-1.496e11, 0.0, 0.0),
            radius_m: 695_700_000.0,
        },
        CelestialBodySample {
            body_id: 399,
            pos_rel_eye_m: DVec3::new(-360_000_000.0, 0.0, 0.0),
            radius_m: 6_378_137.0,
        },
        CelestialBodySample {
            body_id: 499,
            pos_rel_eye_m: DVec3::new(0.0, 1.80e11, 0.0),
            radius_m: 3_389_500.0,
        },
    ];
    let sorted = sort_celestial_bodies_back_to_front(&samples, 1080, 50.0_f64.to_radians());
    assert_eq!(sorted.len(), 4);
    let ids: Vec<i32> = sorted.iter().map(|s| s.body_id).collect();
    // Mars (1.80e11 m) -> Sun (1.496e11 m) -> Earth (3.6e8 m) -> Moon (1.5e7 m)
    assert_eq!(ids, vec![499, 10, 399, 301]);

    // Total solar eclipse umbra when Moon sits directly between sample and Sun:
    let sun_pos = DVec3::new(1.496e11, 0.0, 0.0);
    let moon_pos = DVec3::new(370_000_000.0, 0.0, 0.0);
    let f_umbra =
        evaluate_eclipse_shadow_factor(DVec3::ZERO, sun_pos, 695_700_000.0, moon_pos, 1_737_400.0);
    assert_eq!(f_umbra, 0.0);

    // Partial penumbra at lateral offset:
    let f_penumbra = evaluate_eclipse_shadow_factor(
        DVec3::new(0.0, 1_500_000.0, 0.0),
        sun_pos,
        695_700_000.0,
        moon_pos,
        1_737_400.0,
    );
    assert!(f_penumbra > 0.0 && f_penumbra < 1.0);

    // Unoccluded outside penumbra cone:
    let f_lit = evaluate_eclipse_shadow_factor(
        DVec3::new(0.0, 10_000_000.0, 0.0),
        sun_pos,
        695_700_000.0,
        moon_pos,
        1_737_400.0,
    );
    assert_eq!(f_lit, 1.0);
}

#[test]
fn pass2_dynamic_planetary_frustum_pixel_radius_lod_and_polar_cap_verified() {
    // Launch pad (h_agl = 2.0 m): z_near_p clamps to 0.5 m
    let pad_f = compute_dynamic_planetary_frustum(2.0, 6_378_139.0, 6_378_137.0);
    assert_eq!(pad_f.z_near_m, 0.5);
    assert!(pad_f.z_far_m > 2.0e7);

    // LEO (h_agl = 200 km): z_near_p clamps to 5,000 m
    let leo_f = compute_dynamic_planetary_frustum(200_000.0, 6_578_137.0, 6_378_137.0);
    assert_eq!(leo_f.z_near_m, 5_000.0);

    // Apparent-pixel-radius [48 px, 96 px] smoothstep transition decoupled from SOI:
    assert_eq!(evaluate_planet_impostor_to_terrain_lod(32.0), 0.0);
    assert!((evaluate_planet_impostor_to_terrain_lod(72.0) - 0.5).abs() < 1.0e-12);
    assert_eq!(evaluate_planet_impostor_to_terrain_lod(120.0), 1.0);

    // Polar cap blend weight above |lat| > 84.5..85.05112878 deg:
    assert_eq!(polar_cap_blend_weight(80.0), 0.0);
    let mid_polar = polar_cap_blend_weight(84.8);
    assert!(mid_polar > 0.0 && mid_polar < 1.0);
    assert_eq!(polar_cap_blend_weight(88.5), 1.0);
    assert_eq!(polar_cap_blend_weight(-89.2), 1.0);
}

#[test]
fn pass3_cross_pass_planetary_ridge_occlusion_and_leo_docking_visibility_verified() {
    // Case 1: Mountain ridge at 2,400 m eye depth on a low-altitude flight (h_agl = 300 m -> z_near_p = 30 m):
    let frustum_low = compute_dynamic_planetary_frustum(300.0, 6_378_437.0, 6_378_137.0);
    assert_eq!(frustum_low.z_near_m, 30.0);
    let d_ridge_rev_z =
        encode_finite_reversed_z(2_400.0, frustum_low.z_near_m, frustum_low.z_far_m);
    let z_reconstructed = reconstruct_planetary_eye_depth_m(d_ridge_rev_z, frustum_low);
    assert!((z_reconstructed - 2_400.0).abs() < 0.05);

    // Traffic spacecraft at 3,100 m behind the 2,400 m terrain ridge IS occluded:
    assert!(near_field_occluded_by_planet(
        3_100.0,
        d_ridge_rev_z,
        frustum_low
    ));
    // Ownship / wingman at 15.0 m in front of the 2,400 m ridge is NOT occluded:
    assert!(!near_field_occluded_by_planet(
        15.0,
        d_ridge_rev_z,
        frustum_low
    ));
    // Sky pixel (d_p == 0.0 => +infinity) never occludes any near-field object:
    assert_eq!(
        reconstruct_planetary_eye_depth_m(0.0, frustum_low),
        f64::INFINITY
    );
    assert!(!near_field_occluded_by_planet(95_000.0, 0.0, frustum_low));

    // Case 2: LEO docking at 0.25 m range when planetary z_near_p = 5,000 m!
    // Pass 3's dedicated [0.10 m, 100 km] frustum keeps 0.25 m well inside [z_near_local, z_far_local],
    // while the planet surface below is at 200,000 m:
    let frustum_leo = compute_dynamic_planetary_frustum(200_000.0, 6_578_137.0, 6_378_137.0);
    assert_eq!(frustum_leo.z_near_m, 5_000.0);
    let dock_range_m = std::hint::black_box(0.25);
    assert!(dock_range_m > NEAR_FIELD_USD_Z_NEAR_M && dock_range_m < NEAR_FIELD_USD_Z_FAR_M);
    let d_local_dock = encode_finite_reversed_z(
        dock_range_m,
        NEAR_FIELD_USD_Z_NEAR_M,
        NEAR_FIELD_USD_Z_FAR_M,
    );
    assert!(d_local_dock > 0.35 && d_local_dock < 0.45);
    let d_planet_below =
        encode_finite_reversed_z(200_000.0, frustum_leo.z_near_m, frustum_leo.z_far_m);
    assert!(!near_field_occluded_by_planet(
        0.25,
        d_planet_below,
        frustum_leo
    ));

    // Case 3: Aerospike plume adapts from sea-level Mach diamonds to vacuum expansion:
    let plume_sl = evaluate_aerospike_plume_visual(1.0, 101_325.0);
    let plume_vac = evaluate_aerospike_plume_visual(1.0, 0.0);
    assert!(plume_sl.visible && plume_vac.visible);
    assert!(plume_vac.expansion_ratio > 2.5 * plume_sl.expansion_ratio);
    assert!(plume_sl.shock_diamond_spacing_m > 0.5);
    assert_eq!(plume_vac.shock_diamond_spacing_m, 0.0);
}

#[test]
fn pass4_gimbal_free_quaternion_fdai_navball_and_3d_orrery_projection_verified() {
    // Test exact +90 deg vertical pitch (where classic Euler angles hit gimbal lock):
    let q_vert_90 = DQuat::from_rotation_x(core::f64::consts::FRAC_PI_2);
    let fdai = evaluate_fdai_navball(
        q_vert_90,
        DVec3::new(6_563_137.0, 0.0, 0.0),
        DVec3::new(0.0, 7_790.0, 0.0),
        DVec3::new(0.0, 1.0, 0.0),
        DVec3::new(0.0, 0.0, 1.0),
    );
    assert!((fdai.prograde.dir_body.length() - 1.0).abs() < 1.0e-12);
    assert!((fdai.retrograde.dir_body + fdai.prograde.dir_body).length() < 1.0e-12);
    assert!(fdai.prograde.dir_body.dot(fdai.radial_out.dir_body).abs() < 1.0e-12);
    assert!(fdai.prograde.dir_body.dot(fdai.normal.dir_body).abs() < 1.0e-12);

    // Project 3D Orrery scene (Earth, Moon, spacecraft):
    let nodes = [
        OrreryBodyNode {
            id: 399,
            pos_rel_focus_icrf_m: DVec3::ZERO,
            soi_radius_m: 924_600_000.0,
        },
        OrreryBodyNode {
            id: 301,
            pos_rel_focus_icrf_m: DVec3::new(384_400_000.0, 0.0, 0.0),
            soi_radius_m: 66_183_000.0,
        },
    ];
    let proj = project_orrery_scene(
        &nodes,
        DVec3::new(0.0, 0.0, 1.2e9),
        DQuat::IDENTITY,
        60.0_f64.to_radians(),
        16.0 / 9.0,
    );
    assert_eq!(proj.len(), 2);
    let earth_node = proj.first().expect("earth node");
    assert!(earth_node.ndc_xy == [0.0, 0.0]);
    assert!(earth_node.soi_radius_ndc > 0.5);
}

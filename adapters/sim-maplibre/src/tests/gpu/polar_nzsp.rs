//! Polar `NZSP RWY 02T` Short Final 3 NM GPU regression tests for multi-view OTW + sensor
//! (`TruthSceneMap`) and PFD synthetic vision (`AvionicsMapSet`).
//!
//! Reproduces the exact failure mode from spawning at `NZSP` after flying over a mid-latitude
//! `Catalog` dataset (Delaware `z = 10`, zero DEM/ortho tiles in the southern hemisphere) while
//! holding level pitch (`0.0°`) at `10,299 ft` (`3,139.1 m` MSL, `304 m` above the `2,835 m`
//! South Pole plateau):
//! 1. Polar-cap wedges (`z = 12`, `y = 4095`) must expand their frustum-culling bounds by
//!    `PolarPoleElevations` (`2,835 m`) even when `loaded_dem_for(coords)` is `None`, whether
//!    `TerrainCoverageIndex` is freshly defaulted (`exaggeration == 0.0`) or populated with
//!    leftover mid-latitude DEM tiles (`exaggeration == 1.0`).
//! 2. Simultaneous `otw_slot` + `sensor_slot` rendering on `TruthSceneMap` must share the
//!    64-texture `DrapeCache` without starving OTW polar wedges into `DrapeState::Withheld`.
//! 3. Polar-cap fills (`antarctic-ice-fill` `#dce6f0`) must composite independently of 2D drape
//!    availability so no `#0b1626` `sky-bg` wedges ever appear in the Antarctic snowfield.

use std::{sync::Arc, time::Duration};

use glam::{DQuat, DVec3, EulerRot};
use sim_core::{CameraMount, SimInstant};
use sim_scenery::{CatalogScenery, ImageryRaster, TerrariumTile, TileId};

use super::{c172, try_host_gpu};
use crate::{
    AvionicsMapSet, CatalogTileLoader, CockpitDeviceTier, ConformalFrameStamp, ConformalMapFrame,
    InstrumentMapMode, InstrumentMapSpec, SceneryEndpoints, SharedMapRuntime, TruthSceneConfig,
    TruthSceneFrame, TruthSceneMap, TruthSlotMask, belief_real_svs_style,
    conformal_stamp_from_vehicle, symmetric_frustum, tests::sample_vehicle_at,
    truth_real_world_style,
};

const OTW_W: usize = 240;
const OTW_H: usize = 135;
const SEN_W: usize = 160;
const SEN_H: usize = 120;
const SVS_W: usize = 200;
const SVS_H: usize = 150;

const NZSP_SHORT_FINAL_LAT_DEG: f64 = -89.9502;
const NZSP_SHORT_FINAL_LON_DEG: f64 = -19.69;
const NZSP_SHORT_FINAL_MSL_M: f64 = 3_139.1;

fn delaware_only_scenery_and_runtime(
    gpu: maplibre::render::host_gpu::HostGpu,
) -> (CatalogScenery, SharedMapRuntime) {
    let mut scenery = CatalogScenery::new();
    let delaware_z10 = TileId {
        zoom: 10,
        x: 295,
        y: 394,
    };
    let dem = TerrariumTile::from_heights(16, 16, vec![15.0; 16 * 16]).expect("dem");
    let ortho = ImageryRaster::new(16, 16, vec![110_u8; 16 * 16 * 4]).expect("ortho");
    scenery
        .insert_elevation(delaware_z10, dem)
        .expect("insert dem");
    scenery
        .insert_imagery(delaware_z10, ortho)
        .expect("insert ortho");
    let loader = CatalogTileLoader::new(Arc::new(scenery.clone()));
    let runtime = SharedMapRuntime::new(
        gpu,
        CockpitDeviceTier::Desktop,
        wgpu::TextureFormat::Rgba8Unorm,
        Some(loader.into_shared_loader()),
    )
    .expect("shared runtime");
    (scenery, runtime)
}

fn stamp_at(
    lat_deg: f64,
    lon_deg: f64,
    alt_msl_m: f64,
    [heading_deg, pitch_deg, roll_deg]: [f64; 3],
    mount: CameraMount,
    fov_y_rad: f64,
    [w, h]: [u32; 2],
) -> ConformalFrameStamp {
    let ned_body = DQuat::from_euler(
        EulerRot::ZYX,
        heading_deg.to_radians(),
        pitch_deg.to_radians(),
        roll_deg.to_radians(),
    );
    let veh = sample_vehicle_at(lat_deg, lon_deg, alt_msl_m, ned_body);
    let frustum = symmetric_frustum(fov_y_rad, w, h, 0.5, 250_000.0).expect("frustum");
    conformal_stamp_from_vehicle(SimInstant::START, &veh, &mount, DVec3::ZERO, frustum, 0.0)
        .expect("conformal stamp")
}

/// Most frames a phase may draw before its views must have drawn every drape.
const SETTLE_FRAMES: u64 = 120;

fn assert_otw_snowfield_gap_free(rgba: &[u8], context: &str) {
    let y_start = OTW_H * 62 / 100;
    for y in y_start..OTW_H {
        for x in 0..OTW_W {
            let idx = (y * OTW_W + x) * 4;
            let [r, g, b, a] = [rgba[idx], rgba[idx + 1], rgba[idx + 2], rgba[idx + 3]];
            assert!(
                a == 255 && r >= 150 && g >= 160 && b >= 175 && b >= r.saturating_sub(4),
                "{context}: non-snow pixel [{r}, {g}, {b}, {a}] at ({x}, {y}) in OTW snowfield \
                 (either #0b1626 withheld-drape wedge or frustum-culled limb leak)"
            );
        }
    }
}

fn assert_svs_polar_terrain_gap_free(rgba: &[u8], context: &str) {
    let y_start = SVS_H * 62 / 100;
    for y in y_start..SVS_H {
        for x in 0..SVS_W {
            let idx = (y * SVS_W + x) * 4;
            let [r, g, b, a] = [rgba[idx], rgba[idx + 1], rgba[idx + 2], rgba[idx + 3]];
            let is_svs_bg_gap = r < 95 && g < 75 && b < 60;
            let is_sky_leak = b > r.saturating_add(25);
            assert!(
                a == 255 && !is_svs_bg_gap && !is_sky_leak && r >= 140 && g >= 90,
                "{context}: cracked SVS pixel [{r}, {g}, {b}, {a}] at ({x}, {y}) \
                 (expected continuous TAWS terrain over 2,835 m South Pole plateau)"
            );
        }
    }
}

#[test]
fn nzsp_short_final_3nm_level_pitch_has_zero_polar_cracks_in_otw_and_svs() {
    let Some(gpu) = try_host_gpu() else {
        return;
    };
    let (scenery, runtime) = delaware_only_scenery_and_runtime(gpu);
    let endpoints = SceneryEndpoints::catalog(10, 10);
    let mut truth = TruthSceneMap::new(
        &runtime,
        truth_real_world_style(&endpoints).expect("truth style"),
        TruthSceneConfig {
            otw_size: [OTW_W as u32, OTW_H as u32],
            stereo_otw: false,
            otw_depth_readout: false,
            sensor_size: [SEN_W as u32, SEN_H as u32],
            request_overscan: 1.05,
        },
        c172(),
    )
    .expect("truth map");
    truth
        .set_sun_direction_ecef(DVec3::new(0.45, 0.35, -0.55).normalize())
        .expect("sun");

    let mut svs_set = AvionicsMapSet::new(
        &runtime,
        vec![InstrumentMapSpec {
            id: "pfd_svs".to_owned(),
            style: belief_real_svs_style(&endpoints).expect("svs style"),
            mode: InstrumentMapMode::Conformal3d {
                initial_reference_altitude_m: Some(NZSP_SHORT_FINAL_MSL_M as f32),
            },
            size: [SVS_W as u32, SVS_H as u32],
        }],
    )
    .expect("svs set");

    // Phase 1: Cold-start at NZSP Short Final BEFORE any DEM tiles exist in the coverage index.
    exercise_nzsp_frames(&mut truth, &mut svs_set, 16, "cold_start_empty_dem_index");

    // Phase 2: Populate mid-latitude Delaware DEM (matching default Catalog flight), step there,
    // then spawn back at NZSP Short Final 3 NM with both OTW + nadir sensor active.
    assert_eq!(truth.supply_dem_synchronously(&scenery).expect("dem"), 1);
    assert_eq!(svs_set.supply_dem_synchronously(&scenery).expect("dem"), 1);
    warm_up_delaware_frames(&mut truth, &mut svs_set, 200);
    exercise_nzsp_frames(&mut truth, &mut svs_set, 400, "post_delaware_teleport");
}

fn warm_up_delaware_frames(truth: &mut TruthSceneMap, svs_set: &mut AvionicsMapSet, base_ms: u64) {
    let otw = stamp_at(
        39.13,
        -75.47,
        600.0,
        [180.0, -3.0, 0.0],
        CameraMount::forward_nose(DVec3::ZERO),
        0.75,
        [OTW_W as u32, OTW_H as u32],
    );
    let sen = stamp_at(
        39.13,
        -75.47,
        600.0,
        [180.0, 0.0, 0.0],
        CameraMount::nadir_belly(DVec3::ZERO),
        0.80,
        [SEN_W as u32, SEN_H as u32],
    );
    let svs_stamp = stamp_at(
        39.13,
        -75.47,
        600.0,
        [180.0, -3.0, 0.0],
        CameraMount::forward_nose(DVec3::ZERO),
        40.0_f64.to_radians(),
        [SVS_W as u32, SVS_H as u32],
    );
    for step in 0_u64..2 {
        let timestamp = Duration::from_millis(base_ms + step * 16);
        truth
            .render_frame(TruthSceneFrame {
                timestamp,
                otw_stamp: Some(otw),
                stereo_right_stamp: None,
                sensor_stamp: Some(sen),
                extra_camera_slots: &[],
                entities: &[],
                aircraft: &[],
                ownship_camera_roles: &[],
                polar_runway: None,
            })
            .expect("delaware truth frame");
        let svs = svs_set.get_mut("pfd_svs").expect("svs");
        svs.invalidate();
        svs.render_conformal(ConformalMapFrame {
            timestamp,
            stamp: svs_stamp,
            ownship_msl_m: 600.0,
            runway: None,
            entities: &[],
        })
        .expect("delaware svs frame");
    }
}

/// The window's, the sensor's and the synthetic vision's stamps on the short final.
fn nzsp_stamps() -> [ConformalFrameStamp; 3] {
    let otw = stamp_at(
        NZSP_SHORT_FINAL_LAT_DEG,
        NZSP_SHORT_FINAL_LON_DEG,
        NZSP_SHORT_FINAL_MSL_M,
        [179.0, 0.0, 0.0],
        CameraMount::forward_nose(DVec3::ZERO),
        0.75,
        [OTW_W as u32, OTW_H as u32],
    );
    let sen = stamp_at(
        NZSP_SHORT_FINAL_LAT_DEG,
        NZSP_SHORT_FINAL_LON_DEG,
        NZSP_SHORT_FINAL_MSL_M,
        [179.0, 0.0, 0.0],
        CameraMount::nadir_belly(DVec3::ZERO),
        0.80,
        [SEN_W as u32, SEN_H as u32],
    );
    let svs_stamp = stamp_at(
        NZSP_SHORT_FINAL_LAT_DEG,
        NZSP_SHORT_FINAL_LON_DEG,
        NZSP_SHORT_FINAL_MSL_M,
        [179.0, 0.0, 0.0],
        CameraMount::forward_nose(DVec3::ZERO),
        40.0_f64.to_radians(),
        [SVS_W as u32, SVS_H as u32],
    );
    [otw, sen, svs_stamp]
}

fn exercise_nzsp_frames(
    truth: &mut TruthSceneMap,
    svs_set: &mut AvionicsMapSet,
    base_ms: u64,
    phase: &str,
) {
    let [otw, sen, svs_stamp] = nzsp_stamps();

    // A view draws only a few of its drapes a frame, and a frame of both slots may
    // draw none of the window's, so the views are whole once a frame of each kind in
    // a row redrew none; the count of frames that takes is the map's to choose.
    let mut quiet_frames = 0_u32;
    for step in 0_u64..SETTLE_FRAMES {
        if quiet_frames >= 2 {
            break;
        }
        let timestamp = Duration::from_millis(base_ms + step * 16);
        let mask = if step % 2 == 0 {
            TruthSlotMask::BOTH
        } else {
            TruthSlotMask::OTW_ONLY
        };
        truth
            .render_frame_slots(
                TruthSceneFrame {
                    timestamp,
                    otw_stamp: Some(otw),
                    stereo_right_stamp: None,
                    sensor_stamp: Some(sen),
                    extra_camera_slots: &[],
                    entities: &[],
                    aircraft: &[],
                    ownship_camera_roles: &[],
                    polar_runway: None,
                },
                mask,
            )
            .expect("nzsp truth frame");
        let svs = svs_set.get_mut("pfd_svs").expect("svs");
        svs.invalidate();
        svs.render_conformal(ConformalMapFrame {
            timestamp,
            stamp: svs_stamp,
            ownship_msl_m: NZSP_SHORT_FINAL_MSL_M as f32,
            runway: None,
            entities: &[],
        })
        .expect("nzsp svs frame");
        quiet_frames = if truth.drape_redraws_last_frame() == 0 {
            quiet_frames.saturating_add(1)
        } else {
            0
        };
    }
    assert!(
        quiet_frames >= 2,
        "{phase}: the views still redrew drapes after {SETTLE_FRAMES} frames"
    );

    let otw_rgba = truth.read_slot_rgba(truth.otw_slot()).expect("otw rgba");
    assert_otw_snowfield_gap_free(&otw_rgba, phase);
    let svs_rgba = svs_set
        .get("pfd_svs")
        .expect("svs")
        .read_rgba()
        .expect("svs rgba");
    assert_svs_polar_terrain_gap_free(&svs_rgba, phase);
}

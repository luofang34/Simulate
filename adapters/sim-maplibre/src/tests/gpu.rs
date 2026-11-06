use std::{sync::Arc, time::Duration};

use super::{sample_panel_data, sample_vehicle_at};
use glam::{DQuat, DVec3};
use indicate_instrument_volumes::{CANONICAL_EXTENT, TerrainPatchAux};
use maplibre::render::host_gpu::HostGpu;
use sim_core::{CameraMount, SimInstant};
use sim_indicate::VOLUME_SCENE_BYTES;
use sim_scenery::{CatalogScenery, ImageryRaster, TerrariumTile, TileId};

use crate::{
    CatalogTileLoader, CockpitDeviceTier, InstrumentMapSurface, SceneryEndpoints, SharedMapRuntime,
    TruthSceneConfig, TruthSceneFrame, TruthSceneMap, UsdAircraftAsset, conformal_stamp_from_pose,
    conformal_stamp_from_vehicle, emit_terrain_volume_with_patch, encode_terrain_patch_aux,
    extract_volumetric_mesh_patch, frustum_from_intrinsics, symmetric_frustum,
    tile_id_to_world_coords, truth_real_world_style,
};

mod aircraft;
mod efb_map;
mod globe_picker;
mod instrument_maps;
mod slots;
mod svs_sky;

pub(super) fn c172() -> Arc<UsdAircraftAsset> {
    Arc::new(UsdAircraftAsset::embedded_c172().expect("embedded c172"))
}

pub(crate) fn try_host_gpu() -> Option<HostGpu> {
    pollster::block_on(async {
        let instance = wgpu::Instance::default();
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions::default())
            .await
            .ok()?;
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor::default())
            .await
            .ok()?;
        Some(HostGpu {
            instance,
            adapter,
            device,
            queue,
        })
    })
}

pub(crate) fn sample_scenery_and_runtime(
    gpu: HostGpu,
) -> (CatalogScenery, TileId, SharedMapRuntime) {
    sample_scenery_and_runtime_at(gpu, 320.0)
}

/// A flat z0 world at `ground_m`; KEWR fixtures need ground below the tower eye (~55 m).
pub(super) fn sample_scenery_and_runtime_at(
    gpu: HostGpu,
    ground_m: f64,
) -> (CatalogScenery, TileId, SharedMapRuntime) {
    let mut scenery = CatalogScenery::new();
    let tile_z0 = TileId {
        zoom: 0,
        x: 0,
        y: 0,
    };
    let dem = TerrariumTile::from_heights(16, 16, vec![ground_m; 16 * 16]).expect("dem");
    let ortho = ImageryRaster::new(16, 16, vec![120_u8; 16 * 16 * 4]).expect("ortho");
    scenery.insert_elevation(tile_z0, dem).expect("insert dem");
    scenery
        .insert_imagery(tile_z0, ortho)
        .expect("insert ortho");
    let loader = CatalogTileLoader::new(Arc::new(scenery.clone()));
    let runtime = SharedMapRuntime::new(
        gpu,
        CockpitDeviceTier::Desktop,
        wgpu::TextureFormat::Rgba8Unorm,
        Some(loader.into_shared_loader()),
    )
    .expect("shared runtime");
    (scenery, tile_z0, runtime)
}

#[test]
fn truth_scene_renders_otw_and_sensor_zero_copy() {
    let Some(gpu) = try_host_gpu() else {
        return;
    };
    let (scenery, _, runtime) = sample_scenery_and_runtime(gpu);
    let mut truth = TruthSceneMap::new(
        &runtime,
        truth_real_world_style(&SceneryEndpoints::catalog(0, 0)).expect("truth style"),
        TruthSceneConfig {
            otw_size: [160, 90],
            stereo_otw: false,
            otw_depth_readout: false,
            sensor_size: [80, 60],
            request_overscan: 1.1,
        },
        c172(),
    )
    .expect("truth map");
    assert_eq!(truth.supply_dem_synchronously(&scenery).expect("dem"), 1);
    truth
        .set_sun_direction_ecef(DVec3::new(0.8, 0.2, 0.5))
        .expect("sun");

    let otw_frustum = symmetric_frustum(1.0, 160, 90, 0.5, 250_000.0).expect("otw frustum");
    let sen_frustum = symmetric_frustum(0.8, 80, 60, 0.5, 250_000.0).expect("sen frustum");
    let veh = sample_vehicle_at(39.0, -75.5, 600.0, DQuat::IDENTITY);
    let otw_stamp = conformal_stamp_from_vehicle(
        SimInstant::START,
        &veh,
        &CameraMount::forward_nose(DVec3::ZERO),
        DVec3::ZERO,
        otw_frustum,
        0.0,
    )
    .expect("otw stamp");
    let sen_stamp = conformal_stamp_from_vehicle(
        SimInstant::START,
        &veh,
        &CameraMount::nadir_belly(DVec3::ZERO),
        DVec3::ZERO,
        sen_frustum,
        0.0,
    )
    .expect("sen stamp");

    truth
        .render_frame(TruthSceneFrame {
            timestamp: Duration::from_millis(16),
            otw_stamp: Some(otw_stamp),
            stereo_right_stamp: None,
            sensor_stamp: Some(sen_stamp),
            extra_camera_slots: &[],
            entities: &[],
            aircraft: &[],
            ownship_camera_roles: &[],
            polar_runway: None,
        })
        .expect("truth frame 1");
    assert!(truth.otw_color_view().is_some());
    assert!(truth.sensor_color_view().is_some());
    assert!(truth.sensor_depth_view().is_some());
    assert_eq!(truth.otw_has_readback_buffer(), Some(false));
    assert_eq!(truth.sensor_has_readback_buffer(), Some(false));

    truth
        .render_frame(TruthSceneFrame {
            timestamp: Duration::from_millis(33),
            otw_stamp: Some(otw_stamp),
            stereo_right_stamp: None,
            sensor_stamp: None,
            extra_camera_slots: &[],
            entities: &[],
            aircraft: &[],
            ownship_camera_roles: &[],
            polar_runway: None,
        })
        .expect("truth frame 2");
    assert!(truth.sensor_depth_view().is_some());
}

pub(super) fn verify_volumetric_patch_export(svs: &InstrumentMapSurface, tile_z0: TileId) {
    let patch = svs
        .resident_terrain_patch(tile_id_to_world_coords(tile_z0).expect("z0"))
        .expect("resident z0 patch");
    assert_eq!(patch.grid_side, 129);
    assert!(patch.drape_texture.is_none());
    let aux_bytes = encode_terrain_patch_aux(&patch, 8, 1_000.0, 300.0).expect("aux 8x8");
    assert_eq!(TerrainPatchAux::decode(&aux_bytes).expect("decode").cols, 8);

    let panel = sample_panel_data();
    let mut vol_scene = vec![0_u8; VOLUME_SCENE_BYTES];
    let vol_render = emit_terrain_volume_with_patch(
        &crate::TerrainVolumeRequest {
            snapshot: &patch,
            grid_side: 8,
            meters_per_unit: 1_000.0,
            reference_msl_m: 300.0,
            data: &panel,
            view: indicate_interaction::ViewPose3D::IDENTITY,
            alerts: None,
            extent_m: Some(CANONICAL_EXTENT),
        },
        &mut vol_scene,
    )
    .expect("emit terrain volume");
    assert!(vol_render.scene_bytes > 0);

    let mesh_patch = extract_volumetric_mesh_patch(&patch, CANONICAL_EXTENT, 12_000.0, 300.0)
        .expect("volumetric mesh patch");
    assert_eq!(mesh_patch.positions_local_m.len(), 129 * 129);
    assert_eq!(mesh_patch.indices.len(), 128 * 128 * 6);
}

pub(super) fn sample_kewr_otw_and_tower_stamps(
    kewr: &sim_scenery::OsmAerodromeDiagram,
    rwy_22l: &flightbag::RunwayEndSpec,
) -> (crate::ConformalFrameStamp, crate::ConformalFrameStamp) {
    let c172 = sample_vehicle_at(
        rwy_22l.threshold.latitude_deg,
        rwy_22l.threshold.longitude_deg,
        120.0,
        DQuat::IDENTITY,
    );
    let rig_spec = sim_core::VirtualCameraSpec {
        id: "kewr_tower",
        title: "Newark Tower PTZ",
        placement: sim_core::VirtualCameraPlacement::WorldAutoTrack {
            position: kewr.tower_viewpoint,
            // The traffic drawn here is static, so aim at it rather than 21 m ahead of the
            // 60 m/s fixture vehicle; a tight auto-zoom span leaves no margin for a lead, and
            // a coarse FOV floor would shrink the aircraft to a few pixels at tower range.
            lead_time_s: 0.0,
            auto_zoom_span_m: Some(12.0),
            min_fov_y_rad: 0.1_f64.to_radians(),
            max_fov_y_rad: 50.0_f64.to_radians(),
        },
        base_intrinsics: sim_geo::SensorIntrinsics {
            width: 96,
            height: 72,
            fx: 96.0,
            fy: 96.0,
            cx: 48.0,
            cy: 36.0,
        },
        period: sim_core::CAMERA_DEFAULT_PERIOD,
        depth_readout: true,
    };
    let (tower_pose, tower_intrinsics) = rig_spec
        .pose_and_intrinsics(&c172, DVec3::ZERO)
        .expect("tower pose");
    let tower_frustum =
        frustum_from_intrinsics(&tower_intrinsics, 0.5, 250_000.0).expect("tower frustum");
    let tower_stamp = conformal_stamp_from_pose(SimInstant::START, &tower_pose, tower_frustum, 0.0)
        .expect("tower stamp");
    let otw_frustum = symmetric_frustum(1.0, 128, 72, 0.5, 250_000.0).expect("otw frustum");
    let otw_stamp = conformal_stamp_from_vehicle(
        SimInstant::START,
        &c172,
        &CameraMount::forward_nose(DVec3::ZERO),
        DVec3::ZERO,
        otw_frustum,
        0.0,
    )
    .expect("otw stamp");
    (otw_stamp, tower_stamp)
}

use std::{sync::Arc, time::Duration};

use super::{sample_panel_data, sample_vehicle_at};
use glam::{DQuat, DVec3};
use indicate_instrument_volumes::{CANONICAL_EXTENT, TerrainPatchAux};
use maplibre::{headless::TerrainDrapeMode, render::host_gpu::HostGpu};
use sim_core::{CameraMount, SimInstant};
use sim_indicate::VOLUME_SCENE_BYTES;
use sim_scenery::{CatalogScenery, ImageryRaster, TerrariumTile, TileId};

use crate::{
    BeliefAvionicsConfig, BeliefAvionicsMap, CatalogTileLoader, ChartCameraSpec, ChartFrameOutcome,
    CockpitDeviceTier, SharedMapRuntime, TruthSceneConfig, TruthSceneFrame, TruthSceneMap,
    belief_chart_style, belief_svs_relief_style, conformal_stamp_from_vehicle,
    emit_terrain_volume_with_patch, encode_terrain_patch_aux, extract_volumetric_mesh_patch,
    symmetric_frustum, tile_id_to_world_coords, truth_otw_style,
};

fn try_host_gpu() -> Option<HostGpu> {
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

fn sample_scenery_and_runtime(gpu: HostGpu) -> (CatalogScenery, TileId, SharedMapRuntime) {
    let mut scenery = CatalogScenery::new();
    let tile_z0 = TileId {
        zoom: 0,
        x: 0,
        y: 0,
    };
    let dem = TerrariumTile::from_heights(16, 16, vec![320.0; 16 * 16]).expect("dem");
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
fn dual_world_truth_scene_renders_otw_and_sensor_zero_copy() {
    let Some(gpu) = try_host_gpu() else {
        return;
    };
    let (scenery, _, runtime) = sample_scenery_and_runtime(gpu);
    let mut truth = TruthSceneMap::new(
        &runtime,
        truth_otw_style(0, 0).expect("truth style"),
        TruthSceneConfig {
            otw_size: [160, 90],
            stereo_otw: false,
            otw_depth_readout: false,
            sensor_size: [80, 60],
            request_overscan: 1.1,
        },
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
            otw_stamp,
            stereo_right_stamp: None,
            sensor_stamp: Some(sen_stamp),
            runway: None,
            entities: &[],
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
            otw_stamp,
            stereo_right_stamp: None,
            sensor_stamp: None,
            runway: None,
            entities: &[],
        })
        .expect("truth frame 2");
    assert!(truth.sensor_depth_view().is_some());
}

#[test]
fn belief_avionics_renders_procedural_svs_and_dirty_gated_chart() {
    let Some(gpu) = try_host_gpu() else {
        return;
    };
    let (scenery, tile_z0, runtime) = sample_scenery_and_runtime(gpu);
    let mut belief = BeliefAvionicsMap::new(
        &runtime,
        belief_svs_relief_style(0).expect("svs style"),
        belief_chart_style(0, 0).expect("chart style"),
        BeliefAvionicsConfig {
            svs_size: [120, 90],
            chart_size: [100, 100],
            chart_min_interval: Duration::from_millis(66),
            initial_reference_altitude_m: Some(600.0),
        },
    )
    .expect("belief map");
    assert!(matches!(
        belief.svs_terrain_drape_mode(),
        TerrainDrapeMode::ProceduralRelief(_)
    ));
    assert_eq!(belief.supply_dem_synchronously(&scenery).expect("dem"), 1);

    let svs_frustum = symmetric_frustum(1.0, 120, 90, 0.5, 250_000.0).expect("svs frustum");
    let veh = sample_vehicle_at(39.0, -75.5, 600.0, DQuat::IDENTITY);
    let svs_stamp = conformal_stamp_from_vehicle(
        SimInstant::START,
        &veh,
        &CameraMount::forward_nose(DVec3::ZERO),
        DVec3::ZERO,
        svs_frustum,
        0.0,
    )
    .expect("svs stamp");
    belief
        .render_svs_frame(Duration::from_millis(16), svs_stamp, 600.0, None, &[])
        .expect("svs frame");
    assert!(belief.svs_color_view().is_some());
    assert_eq!(belief.svs_has_readback_buffer(), Some(false));

    let spec = ChartCameraSpec {
        center_lat_deg: 39.0,
        center_lon_deg: -75.5,
        zoom: 4.0,
        bearing_deg: 0.0,
        pitch_deg: 0.0,
        surveillance_revision: 1,
        notices_revision: 0,
    };
    assert_eq!(
        belief
            .render_chart_if_dirty(Duration::from_millis(16), spec)
            .expect("c1"),
        ChartFrameOutcome::Rendered
    );
    assert_eq!(
        belief
            .render_chart_if_dirty(Duration::from_millis(32), spec)
            .expect("c2"),
        ChartFrameOutcome::ReusedCachedTexture
    );
    assert_eq!(belief.chart_render_count(), 1);
    verify_volumetric_patch_export(&belief, tile_z0);
}

fn verify_volumetric_patch_export(belief: &BeliefAvionicsMap, tile_z0: TileId) {
    let patch = belief
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

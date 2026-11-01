use std::sync::Arc;
use std::time::Duration;

use glam::DVec3;
use sim_core::{SimInstant, TICKS_PER_SECOND, VehicleState};
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_indicate::{PANEL_HEIGHT_PX, PANEL_WIDTH_PX};
use sim_maplibre::{
    BeliefAvionicsConfig, BeliefAvionicsMap, CatalogTileLoader, ChartCameraSpec, CockpitDeviceTier,
    ConformalFrameStamp, DEFAULT_FAR_CLIP_M, DEFAULT_NEAR_CLIP_M, HostGpu, SharedMapRuntime,
    TerrainPatchSnapshot, TruthSceneConfig, TruthSceneFrame, TruthSceneMap, ViewSlotId,
    ViewSlotMask, belief_chart_style_with_aerodromes, belief_svs_relief_style,
    conformal_stamp_from_pose, conformal_stamp_from_vehicle, frustum_from_intrinsics,
    symmetric_frustum, tile_id_to_world_coords, truth_otw_style_with_aerodromes,
};
use sim_render::Camera;
use sim_scenario::{OperatorViewportMode, ScenarioProfile};
use sim_scenery::{CatalogScenery, OsmAerodromeDiagram, TileId};

use crate::camera_view::{COCKPIT_FOV_Y_DEG, SVS_FOV_Y_DEG, svs_nose_mount};
use crate::error::AppSimError;
use crate::supervisor_rig::SupervisorRig;

const SENSOR_WIDTH_PX: u32 = 800;
const SENSOR_HEIGHT_PX: u32 = 600;
const SENSOR_RATE_HZ: u64 = 30;
const SENSOR_INTERVAL_TICKS: u64 = TICKS_PER_SECOND / SENSOR_RATE_HZ;

/// Inputs driving one dual-world `TruthSceneMap` + `BeliefAvionicsMap` frame.
pub struct MapFrameInput<'a> {
    /// Window swapchain `[width, height]` in physical pixels.
    pub surface_size: [u32; 2],
    /// Current simulation clock instant.
    pub timestamp: SimInstant,
    /// Active scripted scenario profile (`None` in free flight).
    pub profile: Option<ScenarioProfile>,
    /// True vehicle state (`TruthSceneMap` + virtual camera rig).
    pub vehicle: VehicleState,
    /// Active camera in `sim-render` convention (for `Cockpit` / `Chase` / `VolumetricScene` OTW).
    pub otw_camera: Camera,
    /// Solar illumination direction in ECEF.
    pub sun_direction_ecef: DVec3,
    /// Supervisor holding OSM airport diagrams, virtual camera presets, and surveillance belief.
    pub supervisor: &'a SupervisorRig,
}

/// Dual-World `maplibre-rs` rig sharing a single [`HostGpu`], compiled pipeline cache, and
/// `CatalogScenery` static tile cache between [`TruthSceneMap`] and [`BeliefAvionicsMap`].
pub struct DualWorldMapRig {
    truth: TruthSceneMap,
    belief: BeliefAvionicsMap,
    aux_virtual_camera_slot: ViewSlotId,
    primary_dem_tile: Option<TileId>,
    last_sensor_tick: Option<u64>,
}

impl DualWorldMapRig {
    /// Initializes `TruthSceneMap` and `BeliefAvionicsMap` over `gpu` using the regional
    /// multi-airport `CatalogScenery` (`Delaware`, `KDOV`, `47N`, `KEWR`) and conflated
    /// OpenStreetMap `aeroway=*` airport diagrams.
    pub fn new(gpu: HostGpu, initial_size: [u32; 2]) -> Result<Self, AppSimError> {
        let scenery = CatalogScenery::synthetic_regional_network()
            .map_err(sim_scenario::ScenarioError::from)?;
        let aerodrome_geojson = OsmAerodromeDiagram::builtin_all_merged_geojson()
            .map_err(sim_scenario::ScenarioError::from)?;
        let primary_dem_tile = scenery.elevation_tiles().next();
        let max_dem_z = scenery
            .elevation_tiles()
            .map(|t| t.zoom)
            .max()
            .unwrap_or(14);
        let max_ortho_z = scenery.imagery_tiles().map(|t| t.zoom).max().unwrap_or(15);

        let loader = CatalogTileLoader::new(Arc::new(scenery.clone()));
        let runtime = SharedMapRuntime::new(
            gpu,
            default_device_tier(),
            wgpu::TextureFormat::Rgba8Unorm,
            Some(loader.into_shared_loader()),
        )?;

        let [w_in, h_in] = initial_size;
        let [w, h] = [w_in.max(16), h_in.max(16)];
        let mut truth = TruthSceneMap::new(
            &runtime,
            truth_otw_style_with_aerodromes(max_dem_z, max_ortho_z, aerodrome_geojson.clone())?,
            TruthSceneConfig {
                otw_size: [w, h],
                stereo_otw: false,
                otw_depth_readout: false,
                sensor_size: [SENSOR_WIDTH_PX, SENSOR_HEIGHT_PX],
                request_overscan: 1.15,
            },
        )?;
        let aux_virtual_camera_slot =
            truth.register_virtual_camera(SENSOR_WIDTH_PX, SENSOR_HEIGHT_PX, true)?;
        let _supplied_truth = truth.supply_dem_synchronously(&scenery)?;

        let mut belief = BeliefAvionicsMap::new(
            &runtime,
            belief_svs_relief_style(max_dem_z)?,
            belief_chart_style_with_aerodromes(max_dem_z, max_ortho_z, aerodrome_geojson)?,
            BeliefAvionicsConfig {
                svs_size: [PANEL_WIDTH_PX, PANEL_HEIGHT_PX],
                chart_size: [PANEL_WIDTH_PX, PANEL_HEIGHT_PX],
                chart_min_interval: Duration::from_millis(66),
                initial_reference_altitude_m: Some(450.0),
            },
        )?;
        let _supplied_belief = belief.supply_dem_synchronously(&scenery)?;

        Ok(Self {
            truth,
            belief,
            aux_virtual_camera_slot,
            primary_dem_tile,
            last_sensor_tick: None,
        })
    }

    /// Simulator-only [`TruthSceneMap`].
    #[must_use]
    pub const fn truth(&self) -> &TruthSceneMap {
        &self.truth
    }

    /// Operational-belief [`BeliefAvionicsMap`].
    #[must_use]
    pub const fn belief(&self) -> &BeliefAvionicsMap {
        &self.belief
    }

    /// Secondary registered virtual camera [`ViewSlotId`] ready for multi-camera sensor feeds.
    #[cfg(test)]
    #[must_use]
    pub const fn aux_virtual_camera_slot(&self) -> ViewSlotId {
        self.aux_virtual_camera_slot
    }

    /// Exports a stitched `129×129` resident DEM patch from `BeliefAvionicsMap` for `terrain-volume`.
    #[must_use]
    pub fn sample_belief_terrain_patch(&self) -> Option<TerrainPatchSnapshot> {
        let tile = self.primary_dem_tile?;
        let coords = tile_id_to_world_coords(tile)?;
        self.belief.resident_terrain_patch(coords)
    }

    /// Steps `TruthSceneMap` (OTW + 30 Hz virtual camera) and `BeliefAvionicsMap` (SVS + 15 Hz Chart).
    pub fn render_earth_frame(&mut self, input: &MapFrameInput<'_>) -> Result<(), AppSimError> {
        let _ = self.aux_virtual_camera_slot;
        let [w_in, h_in] = input.surface_size;
        let [w, h] = [w_in.max(16), h_in.max(16)];
        self.truth.resize_otw(w, h)?;
        self.truth
            .set_sun_direction_ecef(input.sun_direction_ecef)?;

        let sim_dur = Duration::from_secs_f64(
            input
                .timestamp
                .duration_since(SimInstant::START)
                .as_seconds()
                .max(0.0),
        );
        self.step_truth_world(input, sim_dur, [w, h])?;
        self.step_belief_world(input, sim_dur)
    }

    fn step_truth_world(
        &mut self,
        input: &MapFrameInput<'_>,
        sim_dur: Duration,
        otw_size: [u32; 2],
    ) -> Result<(), AppSimError> {
        let mode = input.supervisor.viewport_mode();
        let otw_stamp = build_camera_stamp(input.timestamp, &input.otw_camera, otw_size)?;
        let sensor_stamp = if self.should_step_sensor(input.timestamp, mode) {
            let cam_spec = input.supervisor.active_virtual_camera(input.profile);
            let (pose, intrinsics) = cam_spec
                .pose_and_intrinsics(&input.vehicle, DVec3::ZERO)
                .map_err(sim_scenario::ScenarioError::from)?;
            let frustum =
                frustum_from_intrinsics(&intrinsics, DEFAULT_NEAR_CLIP_M, DEFAULT_FAR_CLIP_M)?;
            Some(conformal_stamp_from_pose(
                input.timestamp,
                &pose,
                frustum,
                0.0,
            )?)
        } else {
            None
        };
        if mode != OperatorViewportMode::ChartMap || sensor_stamp.is_some() {
            let truth_entities = input.supervisor.truth_entities(ViewSlotMask::ALL);
            self.truth.render_frame(TruthSceneFrame {
                timestamp: sim_dur,
                otw_stamp,
                stereo_right_stamp: None,
                sensor_stamp,
                extra_camera_slots: &[],
                runway: input.supervisor.active_runway(input.profile),
                entities: &truth_entities,
            })?;
        }
        Ok(())
    }

    fn step_belief_world(
        &mut self,
        input: &MapFrameInput<'_>,
        sim_dur: Duration,
    ) -> Result<(), AppSimError> {
        let mode = input.supervisor.viewport_mode();
        let svs_frustum = symmetric_frustum(
            SVS_FOV_Y_DEG.to_radians(),
            PANEL_WIDTH_PX,
            PANEL_HEIGHT_PX,
            DEFAULT_NEAR_CLIP_M,
            DEFAULT_FAR_CLIP_M,
        )?;
        let svs_stamp = conformal_stamp_from_vehicle(
            input.timestamp,
            &input.vehicle,
            &svs_nose_mount(),
            DVec3::ZERO,
            svs_frustum,
            0.0,
        )?;
        let own_geo = ecef_to_geodetic(input.vehicle.position_ecef);
        let belief_entities = input.supervisor.belief_entities(ViewSlotMask::ALL);
        self.belief.render_svs_frame(
            sim_dur,
            svs_stamp,
            own_geo.height_m as f32,
            input.supervisor.active_runway(input.profile),
            &belief_entities,
        )?;
        let chart_zoom = if mode == OperatorViewportMode::ChartMap {
            13.6
        } else {
            12.6
        };
        let _chart_redrawn = self.belief.render_chart_if_dirty(
            sim_dur,
            ChartCameraSpec {
                center_lat_deg: own_geo.lat_rad.to_degrees(),
                center_lon_deg: own_geo.lon_rad.to_degrees(),
                zoom: chart_zoom,
                bearing_deg: heading_deg(&input.vehicle),
                pitch_deg: 0.0,
                surveillance_revision: input.supervisor.belief_surveillance_revision(),
                notices_revision: 0,
            },
        )?;
        Ok(())
    }

    fn should_step_sensor(&mut self, timestamp: SimInstant, mode: OperatorViewportMode) -> bool {
        if mode == OperatorViewportMode::SensorAperture {
            return true;
        }
        let ticks = u64::try_from(timestamp.ticks_u128()).unwrap_or(u64::MAX);
        let due = self
            .last_sensor_tick
            .is_none_or(|prev| ticks.saturating_sub(prev) >= SENSOR_INTERVAL_TICKS);
        if due {
            self.last_sensor_tick = Some(ticks);
        }
        due
    }
}

#[cfg(target_arch = "wasm32")]
const fn default_device_tier() -> CockpitDeviceTier {
    CockpitDeviceTier::Handheld
}

#[cfg(not(target_arch = "wasm32"))]
const fn default_device_tier() -> CockpitDeviceTier {
    CockpitDeviceTier::Desktop
}

fn build_camera_stamp(
    timestamp: SimInstant,
    camera: &Camera,
    size_px: [u32; 2],
) -> Result<ConformalFrameStamp, AppSimError> {
    let [w_px, h_px] = size_px;
    let fov_y = if camera.fov_y_rad.is_finite() && camera.fov_y_rad > 0.01 {
        camera.fov_y_rad
    } else {
        COCKPIT_FOV_Y_DEG.to_radians()
    };
    let pose_ecef = sim_core::CameraPoseEcef {
        position_ecef: camera.pose.position_ecef,
        orientation: camera.pose.orientation,
    };
    let frustum = symmetric_frustum(
        fov_y,
        w_px.max(1),
        h_px.max(1),
        DEFAULT_NEAR_CLIP_M,
        DEFAULT_FAR_CLIP_M,
    )?;
    Ok(conformal_stamp_from_pose(
        timestamp, &pose_ecef, frustum, 0.0,
    )?)
}

fn heading_deg(vehicle: &VehicleState) -> f64 {
    let geo = ecef_to_geodetic(vehicle.position_ecef);
    let local = LocalFrame::at(&geo);
    let nose_ecef = vehicle.attitude * DVec3::X;
    let north = nose_ecef.dot(local.north());
    let east = nose_ecef.dot(local.east());
    east.atan2(north).to_degrees().rem_euclid(360.0)
}

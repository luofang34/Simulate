//! Real-flight & simulator `BeliefAvionicsMap`: PFD `ConformalSvs3D` procedural DEM
//! relief (`0` drapes, `0` symbols) + MFD `Map2D` dirty-gated chart world.
//!
//! Contains zero reference to `TruthSceneMap` or truth-tier simulator types so the
//! exact same code path executes on real aircraft hardware and in simulation.

use std::time::Duration;

use flightbag::RunwayEndSpec;
use glam::DVec3;
use maplibre::{
    background::BackgroundPlugin,
    celestial::EcefDirection,
    coords::{LatLon, WorldTileCoords, Zoom},
    headless::{
        CoveringPolicy, MultiViewFrame, SlotView, TerrainDrapeMode, TerrainPatchSnapshot,
        ViewSlotConfig, ViewSlotEntry, ViewSlotId, environment::HeadlessEnvironment,
        map::HeadlessMap,
    },
    overlay::OverlayPlugin,
    plugin::Plugin,
    raster::{DefaultRasterTransferables, RasterPlugin},
    render::RenderPlugin,
    style::Style,
    terrain::{TerrainPlugin, transferables::DefaultDemTransferables},
    vector::{DefaultVectorTransferables, VectorPlugin},
};
use sim_scenery::CatalogScenery;

use super::{SharedMapRuntime, validate_size};
use crate::{
    ConformalFrameStamp, DynamicEntity3D, MapAdapterError, build_conformal_overlay,
    supply_catalog_dem_synchronously,
};

/// Viewport and rate-gating configuration for [`BeliefAvionicsMap`].
#[derive(Clone, Copy, Debug)]
pub struct BeliefAvionicsConfig {
    /// PFD `ConformalSvs3D` physical pixel dimensions `[width, height]`.
    pub svs_size: [u32; 2],
    /// MFD `Map2D` chart physical pixel dimensions `[width, height]`.
    pub chart_size: [u32; 2],
    /// Minimum interval between MFD `Map2D` GPU redraws (default `66 ms` / `15 Hz`).
    pub chart_min_interval: Duration,
    /// Initial TAWS ownship reference altitude (metres MSL) for procedural relief coloring.
    pub initial_reference_altitude_m: Option<f32>,
}

impl Default for BeliefAvionicsConfig {
    fn default() -> Self {
        Self {
            svs_size: [800, 600],
            chart_size: [800, 600],
            chart_min_interval: Duration::from_millis(66),
            initial_reference_altitude_m: Some(0.0),
        }
    }
}

/// Camera and belief-layer revisions driving dirty-gated MFD `Map2D` chart rendering.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChartCameraSpec {
    /// Center latitude in degrees.
    pub center_lat_deg: f64,
    /// Center longitude in degrees.
    pub center_lon_deg: f64,
    /// Map zoom level.
    pub zoom: f64,
    /// Bearing in degrees clockwise from true north.
    pub bearing_deg: f64,
    /// Pitch in degrees from nadir (`0.0` for north-up / track-up 2D charts).
    pub pitch_deg: f64,
    /// Monotonic `SurveillanceStore::revision()` of received belief traffic.
    pub surveillance_revision: u64,
    /// Monotonic revision of active aeronautical notices / TFR overlays.
    pub notices_revision: u64,
}

/// Result of a dirty-gated [`BeliefAvionicsMap::render_chart_if_dirty`] call.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChartFrameOutcome {
    /// Camera or belief revisions changed and the minimum interval elapsed; GPU frame executed.
    Rendered,
    /// Nothing changed or rate-limit held; cached `wgpu::TextureView` reused at `0.0 ms` GPU cost.
    ReusedCachedTexture,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ChartDirtyKey {
    lat_q: i64,
    lon_q: i64,
    zoom_q: i64,
    bearing_q: i64,
    pitch_q: i64,
    surveillance_revision: u64,
    notices_revision: u64,
}

impl ChartDirtyKey {
    fn from_spec(spec: ChartCameraSpec) -> Self {
        Self {
            lat_q: (spec.center_lat_deg * 1e6).round() as i64,
            lon_q: (spec.center_lon_deg * 1e6).round() as i64,
            zoom_q: (spec.zoom * 1e4).round() as i64,
            bearing_q: (spec.bearing_deg * 1e3).round() as i64,
            pitch_q: (spec.pitch_deg * 1e3).round() as i64,
            surveillance_revision: spec.surveillance_revision,
            notices_revision: spec.notices_revision,
        }
    }
}

/// Avionics underlay renderer strictly fed by operational belief state (`Indicate` / `Aviate`).
pub struct BeliefAvionicsMap {
    runtime: SharedMapRuntime,
    svs_map: HeadlessMap,
    svs_slot: ViewSlotId,
    svs_size: [u32; 2],
    last_svs_stamp: Option<ConformalFrameStamp>,
    chart_map: HeadlessMap,
    chart_slot: ViewSlotId,
    chart_min_interval: Duration,
    last_chart_key: Option<ChartDirtyKey>,
    last_chart_timestamp: Option<Duration>,
    chart_render_count: u64,
}

impl BeliefAvionicsMap {
    /// Assembles isolated `svs_map` (`ProceduralRelief`, zero drapes/symbols) and
    /// `chart_map` (`Map2D`, dirty-gated) sharing `runtime`'s GPU, pipelines, and static cache.
    pub fn new(
        runtime: &SharedMapRuntime,
        svs_style: Style,
        chart_style: Style,
        config: BeliefAvionicsConfig,
    ) -> Result<Self, MapAdapterError> {
        let [svs_w, svs_h] = config.svs_size;
        let [cht_w, cht_h] = config.chart_size;
        let svs_phys = validate_size(svs_w, svs_h)?;
        let cht_phys = validate_size(cht_w, cht_h)?;
        let mem = runtime.device_tier().available_memory_bytes();

        let svs_settings = runtime
            .device_tier()
            .belief_svs_settings(runtime.color_format(), config.initial_reference_altitude_m);
        let (svs_kernel, svs_renderer) = runtime.create_renderer(16, 16, svs_settings)?;
        let svs_plugins: Vec<Box<dyn Plugin<HeadlessEnvironment>>> = vec![
            Box::new(RenderPlugin),
            Box::new(BackgroundPlugin),
            Box::new(TerrainPlugin::<DefaultDemTransferables>::default()),
            Box::new(OverlayPlugin),
        ];
        let mut svs_map = runtime
            .with_runtime_context(|| {
                HeadlessMap::new(svs_style, svs_renderer, svs_kernel, svs_plugins)
            })
            .map_err(|source| MapAdapterError::Assemble { source })?;
        svs_map.set_available_memory(mem);
        let svs_slot = svs_map.register_view_slot(ViewSlotConfig {
            size: svs_phys,
            depth_readout: false,
        });

        let chart_settings = runtime
            .device_tier()
            .belief_chart_settings(runtime.color_format());
        let (cht_kernel, cht_renderer) = runtime.create_renderer(16, 16, chart_settings)?;
        let cht_plugins: Vec<Box<dyn Plugin<HeadlessEnvironment>>> = vec![
            Box::new(RenderPlugin),
            Box::new(BackgroundPlugin),
            Box::new(RasterPlugin::<DefaultRasterTransferables>::default()),
            Box::new(VectorPlugin::<DefaultVectorTransferables>::default()),
            Box::new(TerrainPlugin::<DefaultDemTransferables>::default()),
            Box::new(OverlayPlugin),
        ];
        let mut chart_map = runtime
            .with_runtime_context(|| {
                HeadlessMap::new(chart_style, cht_renderer, cht_kernel, cht_plugins)
            })
            .map_err(|source| MapAdapterError::Assemble { source })?;
        chart_map.set_available_memory(mem);
        let chart_slot = chart_map.register_view_slot(ViewSlotConfig {
            size: cht_phys,
            depth_readout: false,
        });

        Ok(Self {
            runtime: runtime.clone(),
            svs_map,
            svs_slot,
            svs_size: config.svs_size,
            last_svs_stamp: None,
            chart_map,
            chart_slot,
            chart_min_interval: config.chart_min_interval,
            last_chart_key: None,
            last_chart_timestamp: None,
            chart_render_count: 0,
        })
    }

    /// Synchronously populates resident DEM tiles into both belief maps for offline / lockstep tests.
    pub fn supply_dem_synchronously(
        &mut self,
        scenery: &CatalogScenery,
    ) -> Result<usize, MapAdapterError> {
        let count = self.runtime.with_runtime_context(|| {
            supply_catalog_dem_synchronously(&mut self.svs_map, scenery)
        })?;
        let _ = self.runtime.with_runtime_context(|| {
            supply_catalog_dem_synchronously(&mut self.chart_map, scenery)
        })?;
        Ok(count)
    }

    /// Updates the deterministic ephemeris solar direction illuminating `ConformalSvs3D` relief.
    pub fn set_sun_direction_ecef(&mut self, sun_ecef: DVec3) -> Result<(), MapAdapterError> {
        let dir = EcefDirection::new(sun_ecef.x, sun_ecef.y, sun_ecef.z)
            .map_err(|source| MapAdapterError::Celestial { source })?;
        self.svs_map.set_sun_direction(Some(dir));
        Ok(())
    }

    /// Renders one PFD `ConformalSvs3D` procedural DEM frame (`0` drape passes, `0` symbol collision)
    /// from operational belief kinematics and updates the TAWS reference altitude.
    pub fn render_svs_frame(
        &mut self,
        timestamp: Duration,
        stamp: ConformalFrameStamp,
        ownship_msl_m: f32,
        runway: Option<&RunwayEndSpec>,
        belief_entities: &[DynamicEntity3D],
    ) -> Result<(), MapAdapterError> {
        self.svs_map
            .set_procedural_relief_reference_altitude(Some(ownship_msl_m));
        let (mesh, probes) = build_conformal_overlay(
            &stamp,
            self.svs_size,
            runway,
            belief_entities,
            Some(self.svs_slot),
        );
        self.svs_map
            .set_view_slot_overlay(self.svs_slot, mesh)
            .map_err(|source| MapAdapterError::MultiView { source })?;
        if !probes.is_empty() {
            let _ = self
                .svs_map
                .set_view_slot_depth_probes(self.svs_slot, probes)
                .map_err(|source| MapAdapterError::MultiView { source })?;
        }
        let mv = MultiViewFrame {
            timestamp,
            opaque_environment: true,
            request_overscan: 1.1,
            slots: vec![ViewSlotEntry {
                slot: self.svs_slot,
                view: SlotView::External(stamp.external_view),
                policy: CoveringPolicy::Independent,
            }],
        };
        self.runtime
            .with_runtime_context(|| self.svs_map.run_multi_view_frame(mv))
            .map_err(|source| MapAdapterError::MultiView { source })?;
        self.last_svs_stamp = Some(stamp);
        Ok(())
    }

    /// Renders the MFD `Map2D` chart only when `spec` changed and `chart_min_interval` has elapsed;
    /// otherwise reuses [`Self::chart_color_view`] at zero GPU cost.
    pub fn render_chart_if_dirty(
        &mut self,
        timestamp: Duration,
        spec: ChartCameraSpec,
    ) -> Result<ChartFrameOutcome, MapAdapterError> {
        let key = ChartDirtyKey::from_spec(spec);
        let chart_pending = self.chart_map.needs_redraw();
        if self.last_chart_key == Some(key) && !chart_pending {
            return Ok(ChartFrameOutcome::ReusedCachedTexture);
        }
        if let Some(prev_ts) = self.last_chart_timestamp
            && timestamp >= prev_ts
            && timestamp.saturating_sub(prev_ts) < self.chart_min_interval
        {
            return Ok(ChartFrameOutcome::ReusedCachedTexture);
        }
        let mv = MultiViewFrame {
            timestamp,
            opaque_environment: true,
            request_overscan: 1.0,
            slots: vec![ViewSlotEntry {
                slot: self.chart_slot,
                view: SlotView::Camera {
                    center: LatLon::new(spec.center_lat_deg, spec.center_lon_deg),
                    zoom: Zoom::new(spec.zoom),
                    bearing_deg: spec.bearing_deg,
                    pitch_deg: spec.pitch_deg,
                },
                policy: CoveringPolicy::Independent,
            }],
        };
        self.runtime
            .with_runtime_context(|| self.chart_map.run_multi_view_frame(mv))
            .map_err(|source| MapAdapterError::MultiView { source })?;
        self.last_chart_key = Some(key);
        self.last_chart_timestamp = Some(timestamp);
        self.chart_render_count = self.chart_render_count.wrapping_add(1);
        Ok(ChartFrameOutcome::Rendered)
    }

    /// Marks the MFD `Map2D` chart dirty so the next call redraws regardless of camera state.
    pub fn invalidate_chart(&mut self) {
        self.last_chart_key = None;
        self.last_chart_timestamp = None;
    }

    /// Active [`TerrainDrapeMode`] on the PFD `ConformalSvs3D` map (always `ProceduralRelief`).
    #[must_use]
    pub fn svs_terrain_drape_mode(&self) -> TerrainDrapeMode {
        self.svs_map.terrain_drape_mode()
    }

    /// Exports a stitched `129 × 129` resident DEM [`TerrainPatchSnapshot`] from the belief SVS world.
    #[must_use]
    pub fn resident_terrain_patch(&self, coords: WorldTileCoords) -> Option<TerrainPatchSnapshot> {
        self.svs_map.resident_terrain_patch(coords)
    }

    /// Zero-copy [`wgpu::TextureView`] of the PFD `ConformalSvs3D` color attachment.
    #[must_use]
    pub fn svs_color_view(&self) -> Option<&wgpu::TextureView> {
        self.svs_map.view_slot_color_view(self.svs_slot)
    }

    /// Zero-copy [`wgpu::Texture`] of the PFD `ConformalSvs3D` color attachment.
    #[must_use]
    pub fn svs_color_texture(&self) -> Option<&wgpu::Texture> {
        self.svs_map.view_slot_color_texture(self.svs_slot)
    }

    /// Whether the SVS slot has allocated a CPU `MAP_READ` staging buffer (`false` in zero-copy operation).
    #[must_use]
    pub fn svs_has_readback_buffer(&self) -> Option<bool> {
        self.svs_map.view_slot_has_readback_buffer(self.svs_slot)
    }

    /// Latest [`ConformalFrameStamp`] rendered on the PFD `ConformalSvs3D` slot.
    #[must_use]
    pub const fn last_svs_stamp(&self) -> Option<ConformalFrameStamp> {
        self.last_svs_stamp
    }

    /// Zero-copy [`wgpu::TextureView`] of the MFD `Map2D` chart color attachment.
    #[must_use]
    pub fn chart_color_view(&self) -> Option<&wgpu::TextureView> {
        self.chart_map.view_slot_color_view(self.chart_slot)
    }

    /// Zero-copy [`wgpu::Texture`] of the MFD `Map2D` chart color attachment.
    #[must_use]
    pub fn chart_color_texture(&self) -> Option<&wgpu::Texture> {
        self.chart_map.view_slot_color_texture(self.chart_slot)
    }

    /// Whether the chart slot has allocated a CPU `MAP_READ` staging buffer.
    #[must_use]
    pub fn chart_has_readback_buffer(&self) -> Option<bool> {
        self.chart_map
            .view_slot_has_readback_buffer(self.chart_slot)
    }

    /// Total number of GPU frames actually executed on the MFD `Map2D` chart slot.
    #[must_use]
    pub const fn chart_render_count(&self) -> u64 {
        self.chart_render_count
    }

    /// Whether either the SVS or MFD chart world has pending async tile arrivals or redraw signals.
    #[must_use]
    pub fn needs_redraw(&self) -> bool {
        self.svs_map.needs_redraw() || self.chart_map.needs_redraw()
    }
}

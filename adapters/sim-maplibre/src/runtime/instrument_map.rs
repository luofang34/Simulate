//! One `maplibre-rs` instrument map surface: a conformal 3D relief view or a
//! dirty-gated 2D chart, each with its own independent [`Style`].
//!
//! Instrument surfaces are fed by operational belief state alone, so the same
//! code path runs on aircraft hardware and in simulation.

use std::time::Duration;

use glam::DVec3;
use maplibre::{
    celestial::EcefDirection,
    coords::{LatLon, WorldTileCoords, Zoom},
    headless::{
        CoveringPolicy, MultiViewFrame, SlotView, TerrainDrapeMode, TerrainPatchSnapshot,
        ViewSlotConfig, ViewSlotEntry, ViewSlotId, map::HeadlessMap,
    },
    overlay::OverlayMesh,
    projection::ProjectionType,
    render::view_state::NavigationMode,
    style::Style,
};
use sim_scenery::{CatalogScenery, LandingRunwayGeometry};

use super::{
    SharedMapRuntime, instrument_plugins_for,
    pacing::interval_elapsed,
    pixel_ratio::pixel_ratio_update,
    svs_dem::{ResidentDemSampler, sample_point_msl},
    tile_json::resolve_or_strip_tile_json,
    validate_size,
};
use crate::{
    ConformalFrameStamp, DynamicEntity3D, MapAdapterError, efb_map::effective_globe_zoom,
    entities::build_conformal_overlay_with_dem, geodesy::wrap_lon_deg,
    supply_catalog_dem_synchronously,
};

/// How an instrument map surface is drawn.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum InstrumentMapMode {
    /// Conformal 3D terrain through an [`ConformalFrameStamp`]'s external eye (PFD SVS, HUD
    /// underlays), shaded by `TerrainDrapeMode::ProceduralRelief` when the style has no drapes.
    Conformal3d {
        /// Initial TAWS ownship reference altitude (metres MSL) for procedural relief colouring.
        initial_reference_altitude_m: Option<f32>,
    },
    /// Top-down 2D chart from a [`ChartCameraSpec`], redrawn only when it changes.
    Chart2d {
        /// Minimum interval between GPU redraws.
        min_interval: Duration,
    },
}

/// Everything that defines one instrument map surface.
#[derive(Clone, Debug)]
pub struct InstrumentMapSpec {
    /// Stable key the instrument preset uses to find the surface (e.g. `"pfd_svs"`).
    pub id: String,
    /// The surface's own style and layer stack.
    pub style: Style,
    /// Drawing mode.
    pub mode: InstrumentMapMode,
    /// Physical pixel dimensions `[width, height]`.
    pub size: [u32; 2],
}

/// Camera and data revisions driving dirty-gated 2D chart rendering.
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
    /// Revision of the host symbology overlay (ownship mark, predictors, range ring).
    pub symbology_revision: u64,
}

/// Result of a dirty-gated [`InstrumentMapSurface::render_chart_if_dirty`] call.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChartFrameOutcome {
    /// Camera or data revisions changed and the minimum interval elapsed; GPU frame executed.
    Rendered,
    /// Nothing changed or rate-limit held; cached texture reused at zero GPU cost.
    ReusedCachedTexture,
}

/// Per-frame inputs of a [`InstrumentMapMode::Conformal3d`] surface.
#[derive(Clone, Copy, Debug)]
pub struct ConformalMapFrame<'a> {
    /// Simulation timestamp for tile motion prefetch and animated paint properties.
    pub timestamp: Duration,
    /// Exact camera that draws this frame.
    pub stamp: ConformalFrameStamp,
    /// Ownship altitude (metres MSL) for procedural relief colouring.
    pub ownship_msl_m: f32,
    /// Selected landing runway geometry, outlined conformally.
    pub runway: Option<&'a LandingRunwayGeometry>,
    /// Belief traffic marked conformally; ownship is never drawn on its own view.
    pub entities: &'a [DynamicEntity3D],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ChartDirtyKey([i64; 5], [u64; 3]);

impl ChartDirtyKey {
    fn from_spec(spec: &ChartCameraSpec) -> Self {
        Self(
            [
                (spec.center_lat_deg * 1e6).round() as i64,
                (spec.center_lon_deg * 1e6).round() as i64,
                (spec.zoom * 1e4).round() as i64,
                (spec.bearing_deg * 1e3).round() as i64,
                (spec.pitch_deg * 1e3).round() as i64,
            ],
            [
                spec.surveillance_revision,
                spec.notices_revision,
                spec.symbology_revision,
            ],
        )
    }
}

/// A `maplibre-rs` instrument map with one view slot, sharing [`SharedMapRuntime`]'s GPU,
/// pipelines, and static tile cache with every other map in the process.
pub struct InstrumentMapSurface {
    id: String,
    mode: InstrumentMapMode,
    runtime: SharedMapRuntime,
    map: HeadlessMap,
    slot: ViewSlotId,
    size: [u32; 2],
    last_stamp: Option<ConformalFrameStamp>,
    last_chart_key: Option<ChartDirtyKey>,
    last_chart_timestamp: Option<Duration>,
    conformal_min_interval: Duration,
    last_conformal_timestamp: Option<Duration>,
    pixel_ratio: Option<f64>,
    render_count: u64,
}

impl InstrumentMapSurface {
    /// Assembles the surface's `HeadlessMap`, resolving TileJSON sources first and dropping
    /// the layers of any source that cannot be resolved (offline hosts keep every other layer).
    pub fn new(
        runtime: &SharedMapRuntime,
        spec: InstrumentMapSpec,
    ) -> Result<Self, MapAdapterError> {
        let [width, height] = spec.size;
        let phys = validate_size(width, height)?;
        let tier = runtime.device_tier();
        let settings = match spec.mode {
            InstrumentMapMode::Conformal3d {
                initial_reference_altitude_m,
            } => tier.belief_svs_settings(runtime.color_format(), initial_reference_altitude_m),
            InstrumentMapMode::Chart2d { .. } => tier.belief_chart_settings(runtime.color_format()),
        };
        let (kernel, renderer) = runtime.create_renderer(16, 16, settings)?;
        let mut style = spec.style;
        resolve_or_strip_tile_json(runtime, &mut style, kernel.source_client());
        let free_globe = matches!(spec.mode, InstrumentMapMode::Chart2d { .. })
            && style
                .projection
                .as_ref()
                .is_some_and(|p| p.projection_type == ProjectionType::VerticalPerspective);
        let plugins = instrument_plugins_for(spec.mode);
        let mut map = runtime
            .with_runtime_context(|| HeadlessMap::new(style, renderer, kernel, plugins))
            .map_err(|source| MapAdapterError::Assemble { source })?;
        // Conformal surfaces leave the atmosphere and sun unset: the style's sky is then drawn
        // as an analytic elevation gradient (the same at every heading, altitude and time of
        // day) and procedural relief lights itself.
        map.set_available_memory(tier.available_memory_bytes());
        if free_globe {
            map.set_navigation_mode(NavigationMode::FreeGlobe)
                .map_err(|source| MapAdapterError::Navigation { source })?;
        }
        let slot = map.register_view_slot(ViewSlotConfig {
            size: phys,
            depth_readout: false,
        });
        Ok(Self {
            id: spec.id,
            mode: spec.mode,
            runtime: runtime.clone(),
            map,
            slot,
            size: spec.size,
            last_stamp: None,
            last_chart_key: None,
            last_chart_timestamp: None,
            conformal_min_interval: tier.conformal_min_interval(),
            last_conformal_timestamp: None,
            pixel_ratio: None,
            render_count: 0,
        })
    }

    /// Stable key of this surface.
    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }

    /// Drawing mode.
    #[must_use]
    pub const fn mode(&self) -> InstrumentMapMode {
        self.mode
    }

    /// Whether this surface draws conformal 3D views.
    #[must_use]
    pub const fn is_conformal(&self) -> bool {
        matches!(self.mode, InstrumentMapMode::Conformal3d { .. })
    }

    /// Physical `[width, height]` pixel dimensions.
    #[must_use]
    pub const fn size(&self) -> [u32; 2] {
        self.size
    }

    /// Resizes the surface; zero or unchanged dimensions are ignored.
    pub fn resize(&mut self, width: u32, height: u32) -> Result<(), MapAdapterError> {
        if width == 0 || height == 0 || self.size == [width, height] {
            return Ok(());
        }
        let phys = validate_size(width, height)?;
        self.map
            .resize_view_slot(self.slot, phys)
            .map_err(|source| MapAdapterError::MultiView { source })?;
        self.size = [width, height];
        self.invalidate();
        Ok(())
    }

    /// Synchronously populates resident DEM tiles for offline / lockstep tests.
    pub fn supply_dem_synchronously(
        &mut self,
        scenery: &CatalogScenery,
    ) -> Result<usize, MapAdapterError> {
        self.runtime
            .with_runtime_context(|| supply_catalog_dem_synchronously(&mut self.map, scenery))
    }

    /// Updates the solar direction lighting a 2D chart. Conformal surfaces ignore it: their
    /// sky and relief never depend on the sun, so a night outside never darkens the SVS.
    pub fn set_sun_direction_ecef(&mut self, sun_ecef: DVec3) -> Result<(), MapAdapterError> {
        if self.is_conformal() {
            return Ok(());
        }
        let dir = EcefDirection::new(sun_ecef.x, sun_ecef.y, sun_ecef.z)
            .map_err(|source| MapAdapterError::Celestial { source })?;
        self.map.set_sun_direction(Some(dir));
        Ok(())
    }

    /// Draws at `ratio` physical pixels per logical pixel (Retina `2.0`). Non-finite ratios
    /// are ignored, others clamped to `0.5..=4.0`, and unchanged ratios not reapplied.
    pub fn set_pixel_ratio(&mut self, ratio: f64) {
        if let Some(applied) = pixel_ratio_update(self.pixel_ratio, ratio) {
            self.map.set_pixel_ratio(applied);
            self.pixel_ratio = Some(applied);
            self.invalidate();
        }
    }

    /// Active device scale factor (`1.0` until [`Self::set_pixel_ratio`] is called).
    #[must_use]
    pub fn pixel_ratio(&self) -> f64 {
        self.pixel_ratio.unwrap_or(1.0)
    }

    /// Whether this surface's minimum redraw interval has elapsed at `at` (the timeline of
    /// [`Self::render_conformal`] and [`Self::render_chart_if_dirty`]). Hosts check it before
    /// building per-frame camera stamps or symbology; it has no side effects.
    #[must_use]
    pub fn is_due(&self, at: Duration) -> bool {
        match self.mode {
            InstrumentMapMode::Conformal3d { .. } => interval_elapsed(
                self.last_conformal_timestamp,
                at,
                self.conformal_min_interval,
            ),
            InstrumentMapMode::Chart2d { min_interval } => {
                interval_elapsed(self.last_chart_timestamp, at, min_interval)
            }
        }
    }

    /// Renders one conformal frame; ownship-exterior entities are never marked on it.
    /// Draws unconditionally: hosts pace it with [`Self::is_due`].
    pub fn render_conformal(
        &mut self,
        frame: ConformalMapFrame<'_>,
    ) -> Result<(), MapAdapterError> {
        self.require(true)?;
        self.map
            .set_procedural_relief_reference_altitude(Some(frame.ownship_msl_m));
        let (mesh, probes) = {
            let mut dem = ResidentDemSampler::new(&self.map);
            build_conformal_overlay_with_dem(
                &frame.stamp,
                self.size,
                frame.runway,
                frame.entities,
                Some(self.slot),
                &mut |lat, lon| dem.sample_msl_and_cell_max(lat, lon),
            )
        };
        self.map
            .set_view_slot_overlay(self.slot, mesh)
            .map_err(|source| MapAdapterError::MultiView { source })?;
        if !probes.is_empty() {
            let _request = self
                .map
                .set_view_slot_depth_probes(self.slot, probes)
                .map_err(|source| MapAdapterError::MultiView { source })?;
        }
        let view = SlotView::External(frame.stamp.external_view);
        self.run(frame.timestamp, 1.1, view)?;
        self.last_stamp = Some(frame.stamp);
        self.last_conformal_timestamp = Some(frame.timestamp);
        Ok(())
    }

    /// Renders the chart only when `spec` changed and the minimum interval has elapsed;
    /// `symbology` replaces the chart's overlay whenever a frame is drawn.
    pub fn render_chart_if_dirty(
        &mut self,
        timestamp: Duration,
        spec: ChartCameraSpec,
        symbology: Option<OverlayMesh>,
    ) -> Result<ChartFrameOutcome, MapAdapterError> {
        self.require(false)?;
        let InstrumentMapMode::Chart2d { min_interval } = self.mode else {
            return Ok(ChartFrameOutcome::ReusedCachedTexture);
        };
        let key = ChartDirtyKey::from_spec(&spec);
        if self.last_chart_key == Some(key) && !self.map.needs_redraw() {
            return Ok(ChartFrameOutcome::ReusedCachedTexture);
        }
        if !interval_elapsed(self.last_chart_timestamp, timestamp, min_interval) {
            return Ok(ChartFrameOutcome::ReusedCachedTexture);
        }
        if let Some(mesh) = symbology {
            self.map
                .set_view_slot_overlay(self.slot, mesh)
                .map_err(|source| MapAdapterError::MultiView { source })?;
        }
        let center_lat = spec.center_lat_deg.clamp(-90.0, 90.0);
        let view = SlotView::Camera {
            center: LatLon::new(center_lat, wrap_lon_deg(spec.center_lon_deg)),
            zoom: Zoom::new(effective_globe_zoom(spec.zoom, center_lat)),
            bearing_deg: spec.bearing_deg,
            pitch_deg: spec.pitch_deg,
        };
        self.run(timestamp, 1.0, view)?;
        self.last_chart_key = Some(key);
        self.last_chart_timestamp = Some(timestamp);
        Ok(ChartFrameOutcome::Rendered)
    }

    /// Marks the surface dirty so the next call redraws regardless of camera state or interval.
    pub fn invalidate(&mut self) {
        self.last_chart_key = None;
        self.last_chart_timestamp = None;
        self.last_conformal_timestamp = None;
    }

    fn require(&self, conformal: bool) -> Result<(), MapAdapterError> {
        if self.is_conformal() == conformal {
            return Ok(());
        }
        Err(MapAdapterError::InstrumentModeMismatch {
            id: self.id.clone(),
            requested: if conformal { "Conformal3d" } else { "Chart2d" },
        })
    }

    fn run(
        &mut self,
        timestamp: Duration,
        request_overscan: f64,
        view: SlotView,
    ) -> Result<(), MapAdapterError> {
        let frame = MultiViewFrame {
            timestamp,
            opaque_environment: true,
            request_overscan,
            slots: vec![ViewSlotEntry {
                slot: self.slot,
                view,
                policy: CoveringPolicy::Independent,
            }],
        };
        self.runtime
            .with_runtime_context(|| self.map.run_multi_view_frame(frame))
            .map_err(|source| MapAdapterError::MultiView { source })?;
        self.render_count = self.render_count.wrapping_add(1);
        Ok(())
    }

    /// Active [`TerrainDrapeMode`] (`ProceduralRelief` for drape-free conformal styles).
    #[must_use]
    pub fn terrain_drape_mode(&self) -> TerrainDrapeMode {
        self.map.terrain_drape_mode()
    }

    /// Samples bilinear terrain elevation (metres MSL) from the surface's resident z14 DEM.
    #[must_use]
    pub fn sample_resident_dem_msl(&self, lat_deg: f64, lon_deg: f64) -> Option<f64> {
        sample_point_msl(&self.map, lat_deg, lon_deg)
    }

    /// Exports a stitched `129 × 129` resident DEM [`TerrainPatchSnapshot`].
    #[must_use]
    pub fn resident_terrain_patch(&self, coords: WorldTileCoords) -> Option<TerrainPatchSnapshot> {
        self.map.resident_terrain_patch(coords)
    }

    /// Zero-copy [`wgpu::TextureView`] of the color attachment.
    #[must_use]
    pub fn color_view(&self) -> Option<&wgpu::TextureView> {
        self.map.view_slot_color_view(self.slot)
    }

    /// Zero-copy [`wgpu::Texture`] of the color attachment.
    #[must_use]
    pub fn color_texture(&self) -> Option<&wgpu::Texture> {
        self.map.view_slot_color_texture(self.slot)
    }

    /// Whether the slot has allocated a CPU `MAP_READ` staging buffer.
    #[must_use]
    pub fn has_readback_buffer(&self) -> Option<bool> {
        self.map.view_slot_has_readback_buffer(self.slot)
    }

    /// Latest conformal [`ConformalFrameStamp`] drawn.
    #[must_use]
    pub const fn last_stamp(&self) -> Option<ConformalFrameStamp> {
        self.last_stamp
    }

    /// Total number of GPU frames executed.
    #[must_use]
    pub const fn render_count(&self) -> u64 {
        self.render_count
    }

    /// Whether background tile workers delivered arrivals that still need drawing.
    #[must_use]
    pub fn needs_redraw(&self) -> bool {
        self.map.needs_redraw()
    }

    /// Lazily copies the color attachment into tight RGBA8 bytes (`width * height * 4`).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn read_rgba(&self) -> Result<Vec<u8>, MapAdapterError> {
        let texture = self
            .color_texture()
            .ok_or_else(|| MapAdapterError::Readback {
                detail: format!("instrument map {} has no color attachment", self.id),
            })?;
        let gpu = self.runtime.host_gpu();
        super::readback::read_texture_rgba_blocking(&gpu.device, &gpu.queue, texture)
    }
}

//! The electronic flight bag map shared by the simulator cockpit and a practical EFB: one
//! moving map with a compact corner view and an expanded EFB page, a georeferenced
//! approach plate, the airport surface diagram, airspace, and ownship, route and traffic
//! symbology.
//!
//! Both views are slots of one `maplibre` map, so they share every loaded tile: switching
//! between them renders the other slot at once with nothing to reload. Only the shown view
//! is drawn each frame.

use std::{sync::Arc, time::Duration};

use maplibre::{
    background::BackgroundPlugin,
    coords::{LatLon, Zoom},
    headless::{
        CoveringPolicy, MultiViewFrame, SlotView, ViewSlotConfig, ViewSlotEntry, ViewSlotId,
        environment::HeadlessEnvironment, map::HeadlessMap,
    },
    hillshade::HillshadePlugin,
    overlay::OverlayPlugin,
    plugin::Plugin,
    raster::{DefaultRasterTransferables, RasterPlugin},
    render::{RenderPlugin, view_state::NavigationMode},
    terrain::{TerrainPlugin, transferables::DefaultDemTransferables},
    vector::{DefaultVectorTransferables, VectorPlugin},
};
use serde_json::{Value, json};

use crate::{
    ChartAssetStore, ChartSymbologyInput, CockpitDeviceTier, MapAdapterError, SceneryEndpoints,
    SharedMapRuntime, build_chart_symbology_scaled, chart_url,
    geodesy::wrap_lon_deg,
    runtime::{
        DrapeBudgetPlugin, EARTH_POLAR_ELEVATIONS, PolarElevationsPlugin, pixel_ratio_update,
        resolve_or_strip_tile_json, validate_size,
    },
};

mod camera;
mod pacing;
mod plate;
mod style;
#[cfg(test)]
mod tests;

pub(crate) use camera::effective_globe_zoom;
pub use camera::{
    DEFAULT_COMPACT_ZOOM, DEFAULT_EXPANDED_ZOOM, EFB_ZOOM_RANGE, EfbDisplayMode, EfbFollowMode,
    EfbLayerFlags, EfbRefreshPolicy,
};
pub use pacing::EfbPacer;
pub use plate::{ApproachPlateOverlay, ProceduralPlateSpec, generate_procedural_approach_plate};

use camera::EfbCamera;
use style::{
    AERODROME_SOURCE, AIRSPACE_LAYER, AIRSPACE_SOURCE, NO_PLATE_CHART_ID, PLATE_LAYER, PLATE_SOURCE,
};

/// Frame interval of the EFB's own frame clock.
const FRAME_INTERVAL: Duration = Duration::from_millis(33);

/// Viewport sizes of the EFB map.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EfbMapConfig {
    /// Compact corner view, physical pixels.
    pub compact_size: [u32; 2],
    /// Expanded EFB page, physical pixels.
    pub expanded_size: [u32; 2],
    /// Initial centre `[lat, lon]` before ownship is known, degrees.
    pub initial_center: [f64; 2],
}

impl Default for EfbMapConfig {
    fn default() -> Self {
        Self {
            compact_size: [384, 288],
            expanded_size: [1024, 768],
            initial_center: [40.69, -74.17],
        }
    }
}

/// Moving map with compact and expanded views over one tile world.
pub struct EfbMapSurface {
    runtime: SharedMapRuntime,
    map: HeadlessMap,
    charts: ChartAssetStore,
    slots: [ViewSlotId; 2],
    sizes: [[u32; 2]; 2],
    camera: EfbCamera,
    layers: EfbLayerFlags,
    aerodrome_layers: Vec<String>,
    plate: Option<ApproachPlateOverlay>,
    plate_opacity: f32,
    pixel_ratio: Option<f64>,
    frames: u64,
}

impl EfbMapSurface {
    /// Builds the EFB map over `endpoints` with the airport surface diagram
    /// `aerodrome_geojson`. Plates are served from `charts`, which must be the store of
    /// the runtime's [`crate::HybridTileLoader`].
    pub fn new(
        runtime: &SharedMapRuntime,
        charts: ChartAssetStore,
        endpoints: &SceneryEndpoints,
        aerodrome_geojson: Value,
        config: EfbMapConfig,
    ) -> Result<Self, MapAdapterError> {
        let compact = size(config.compact_size)?;
        let expanded = size(config.expanded_size)?;
        charts.put_chart_png(NO_PLATE_CHART_ID, transparent_pixel_png()?);
        let (kernel, renderer) = runtime.create_renderer(
            16,
            16,
            runtime
                .device_tier()
                .belief_chart_settings(runtime.color_format()),
        )?;
        let mut style = style::efb_style(endpoints, aerodrome_geojson)?;
        resolve_or_strip_tile_json(runtime, &mut style, kernel.source_client());
        let aerodrome_layers = style::aerodrome_layer_ids(&style);
        let tier = runtime.device_tier();
        let mut map = runtime
            .with_runtime_context(|| HeadlessMap::new(style, renderer, kernel, plugins(tier)))
            .map_err(|source| MapAdapterError::Assemble { source })?;
        map.set_available_memory(runtime.device_tier().available_memory_bytes());
        map.set_navigation_mode(NavigationMode::FreeGlobe)
            .map_err(|source| MapAdapterError::Navigation { source })?;
        let mut register = |size| {
            map.register_view_slot(ViewSlotConfig {
                size,
                depth_readout: false,
            })
        };
        let slots = [register(compact), register(expanded)];
        let [lat, lon] = config.initial_center;
        Ok(Self {
            runtime: runtime.clone(),
            map,
            charts,
            slots,
            sizes: [config.compact_size, config.expanded_size],
            camera: EfbCamera::new(lat, lon),
            layers: EfbLayerFlags::default(),
            aerodrome_layers,
            plate: None,
            plate_opacity: style::DEFAULT_PLATE_OPACITY,
            pixel_ratio: None,
            frames: 0,
        })
    }

    /// Draws the EFB at `ratio` physical pixels per logical pixel (`2.0` on Retina).
    /// Non-finite ratios are ignored, others clamped to `0.5..=4.0`.
    pub fn set_pixel_ratio(&mut self, ratio: f64) {
        if let Some(applied) = pixel_ratio_update(self.pixel_ratio, ratio) {
            self.map.set_pixel_ratio(applied);
            self.pixel_ratio = Some(applied);
        }
    }

    /// Active device scale factor (`1.0` until [`Self::set_pixel_ratio`] is called).
    #[must_use]
    pub fn pixel_ratio(&self) -> f64 {
        self.pixel_ratio.unwrap_or(1.0)
    }

    /// The shown view.
    #[must_use]
    pub const fn display_mode(&self) -> EfbDisplayMode {
        self.camera.mode
    }

    /// Shows `mode`'s view from the next frame; its tiles are already resident.
    pub fn set_display_mode(&mut self, mode: EfbDisplayMode) {
        self.camera.mode = mode;
    }

    /// Switches between the compact and expanded views; returns the view now shown.
    pub fn toggle_expanded(&mut self) -> EfbDisplayMode {
        let next = match self.camera.mode {
            EfbDisplayMode::CompactCorner => EfbDisplayMode::ExpandedEfb,
            EfbDisplayMode::ExpandedEfb => EfbDisplayMode::CompactCorner,
        };
        self.set_display_mode(next);
        next
    }

    /// How the camera follows ownship.
    #[must_use]
    pub const fn follow_mode(&self) -> EfbFollowMode {
        self.camera.follow
    }

    /// Whether the map is rotated to a custom gesture bearing (either centred or panned).
    #[must_use]
    pub const fn has_custom_bearing(&self) -> bool {
        self.camera.has_custom_bearing()
    }

    /// Sets how the camera follows ownship.
    pub fn set_follow_mode(&mut self, mode: EfbFollowMode) {
        self.camera.set_follow(mode);
    }

    /// Follows ownship again, in the follow mode last chosen.
    pub fn recenter_ownship(&mut self) {
        self.camera.recenter();
    }

    /// The shown view's zoom.
    #[must_use]
    pub fn zoom(&self) -> f64 {
        self.camera.zoom()
    }

    /// Changes the shown view's zoom by `delta`, within [`EFB_ZOOM_RANGE`].
    pub fn zoom_by(&mut self, delta: f64) {
        self.camera.zoom_by(delta);
    }

    /// Rotates the shown view by `delta_deg` (`> 0` rotates map features counter-clockwise),
    /// staying centred on ownship unless panned free.
    pub fn rotate_by_deg(&mut self, delta_deg: f64) {
        self.camera.rotate_by_deg(delta_deg);
    }

    /// Pans the shown view by physical texture pixels (`dx` right, `dy` down) and stops
    /// following ownship until [`Self::recenter_ownship`].
    pub fn pan_by_pixels(&mut self, dx_px: f32, dy_px: f32) {
        let ratio = self.pixel_ratio();
        self.camera
            .pan_by_pixels(f64::from(dx_px) / ratio, f64::from(dy_px) / ratio);
    }

    /// `(lat_deg, lon_deg, bearing_deg)` the shown view is centred on and rotated to.
    #[must_use]
    pub fn camera_center_and_bearing(&self) -> (f64, f64, f64) {
        let spec = self.camera.spec();
        (spec.center_lat_deg, spec.center_lon_deg, spec.bearing_deg)
    }

    /// The overlays drawn.
    #[must_use]
    pub const fn layers(&self) -> EfbLayerFlags {
        self.layers
    }

    /// Chooses the overlays drawn.
    pub fn set_layers(&mut self, layers: EfbLayerFlags) -> Result<(), MapAdapterError> {
        self.layers = layers;
        self.apply_visibility()
    }

    /// Opacity of the approach plate over the moving map.
    #[must_use]
    pub const fn plate_opacity(&self) -> f32 {
        self.plate_opacity
    }

    /// Sets the approach plate's opacity, clamped to `[0, 1]`.
    pub fn set_plate_opacity(&mut self, opacity: f32) -> Result<(), MapAdapterError> {
        let opacity = if opacity.is_finite() {
            opacity.clamp(0.0, 1.0)
        } else {
            style::DEFAULT_PLATE_OPACITY
        };
        self.mutate(PLATE_LAYER, |style| {
            style.set_paint_property(PLATE_LAYER, "raster-opacity", json!(opacity))
        })?;
        self.plate_opacity = opacity;
        Ok(())
    }

    /// The approach plate shown, if any.
    #[must_use]
    pub const fn approach_plate(&self) -> Option<&ApproachPlateOverlay> {
        self.plate.as_ref()
    }

    /// Shows `plate` georeferenced over the map, or removes the plate.
    pub fn set_approach_plate(
        &mut self,
        plate: Option<ApproachPlateOverlay>,
    ) -> Result<(), MapAdapterError> {
        if let Some(old) = self.plate.take()
            && plate
                .as_ref()
                .is_none_or(|new| new.chart_id != old.chart_id)
        {
            let _was_registered = self.charts.remove_chart(&old.chart_id);
        }
        if let Some(plate) = &plate {
            self.charts
                .put_chart_png(&plate.chart_id, Arc::clone(&plate.png_bytes));
            let url = chart_url(&plate.chart_id);
            let corners = plate.corners_lon_lat;
            self.mutate(PLATE_SOURCE, |style| {
                style.update_image_source(PLATE_SOURCE, url, Some(corners))
            })?;
        }
        self.plate = plate;
        self.apply_visibility()
    }

    /// Replaces the airport surface diagram with the GeoJSON document `geojson`.
    pub fn set_surface_diagram_geojson(&mut self, geojson: &str) -> Result<(), MapAdapterError> {
        self.set_geojson(AERODROME_SOURCE, geojson)
    }

    /// Replaces the airspace boundaries with the GeoJSON document `geojson`.
    pub fn set_airspace_geojson(&mut self, geojson: &str) -> Result<(), MapAdapterError> {
        self.set_geojson(AIRSPACE_SOURCE, geojson)
    }

    fn set_geojson(&mut self, source_name: &str, geojson: &str) -> Result<(), MapAdapterError> {
        let document: Value = serde_json::from_str(geojson)
            .map_err(|source| MapAdapterError::StyleParse { source })?;
        self.map
            .set_geojson_source(source_name, document)
            .map_err(|source| MapAdapterError::GeoJsonSource {
                source_name: source_name.to_owned(),
                source,
            })
    }

    /// Draws the shown view: follows ownship per the follow mode and draws the symbology
    /// of `symbology` (its camera and viewport are replaced by the EFB's own).
    pub fn render(
        &mut self,
        symbology: &ChartSymbologyInput<'_>,
    ) -> Result<Option<&wgpu::TextureView>, MapAdapterError> {
        self.camera.follow_ownship(symbology.ownship.as_ref());
        let spec = self.camera.spec();
        let route = self.layers.route_and_traffic;
        let mesh = build_chart_symbology_scaled(
            &ChartSymbologyInput {
                camera: spec,
                viewport_px: self.active_size(),
                ownship: symbology.ownship,
                runway: symbology.runway.filter(|_| route),
                traffic: if route { symbology.traffic } else { &[] },
            },
            self.pixel_ratio(),
        );
        let slot = self.active_slot();
        self.map
            .set_view_slot_overlay(slot, mesh)
            .map_err(|source| MapAdapterError::MultiView { source })?;
        self.frames = self.frames.wrapping_add(1);
        let center_lat = spec.center_lat_deg.clamp(-90.0, 90.0);
        let frame = MultiViewFrame {
            timestamp: FRAME_INTERVAL
                .saturating_mul(u32::try_from(self.frames).unwrap_or(u32::MAX)),
            opaque_environment: true,
            request_overscan: 1.0,
            slots: vec![ViewSlotEntry {
                slot,
                view: SlotView::Camera {
                    center: LatLon::new(center_lat, wrap_lon_deg(spec.center_lon_deg)),
                    zoom: Zoom::new(effective_globe_zoom(spec.zoom, center_lat)),
                    bearing_deg: spec.bearing_deg,
                    pitch_deg: 0.0,
                },
                policy: CoveringPolicy::Independent,
            }],
        };
        self.runtime
            .with_runtime_context(|| self.map.run_multi_view_frame(frame))
            .map_err(|source| MapAdapterError::MultiView { source })?;
        Ok(self.map.view_slot_color_view(slot))
    }

    /// Resizes the expanded EFB page; zero or unchanged sizes are ignored.
    pub fn resize_expanded(&mut self, size_px: [u32; 2]) -> Result<(), MapAdapterError> {
        let [compact, expanded] = self.sizes;
        if size_px == expanded || size_px.contains(&0) {
            return Ok(());
        }
        let [_, slot] = self.slots;
        self.map
            .resize_view_slot(slot, size(size_px)?)
            .map_err(|source| MapAdapterError::MultiView { source })?;
        self.sizes = [compact, size_px];
        Ok(())
    }

    /// The shown view's size, physical pixels.
    #[must_use]
    pub fn active_size(&self) -> [u32; 2] {
        let [compact, expanded] = self.sizes;
        match self.camera.mode {
            EfbDisplayMode::CompactCorner => compact,
            EfbDisplayMode::ExpandedEfb => expanded,
        }
    }

    /// Whether background requests or arrived tiles remain unsettled.
    #[must_use]
    pub fn needs_redraw(&self) -> bool {
        self.map.needs_redraw()
    }

    /// Whether dirty state, animations, or arrived worker tiles are ready to draw right now.
    #[must_use]
    pub fn has_ready_work(&self) -> bool {
        self.map.has_ready_work()
    }

    /// Copies the shown view's last frame into tight RGBA8 bytes.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn read_active_rgba(&self) -> Result<Vec<u8>, MapAdapterError> {
        let texture = self
            .map
            .view_slot_color_texture(self.active_slot())
            .ok_or_else(|| MapAdapterError::Readback {
                detail: "EFB view has no colour attachment".to_owned(),
            })?;
        let gpu = self.runtime.host_gpu();
        crate::runtime::read_texture_rgba_blocking(&gpu.device, &gpu.queue, texture)
    }

    fn active_slot(&self) -> ViewSlotId {
        let [compact, expanded] = self.slots;
        match self.camera.mode {
            EfbDisplayMode::CompactCorner => compact,
            EfbDisplayMode::ExpandedEfb => expanded,
        }
    }

    fn apply_visibility(&mut self) -> Result<(), MapAdapterError> {
        let plate = self.layers.approach_plate && self.plate.is_some();
        let mut targets = vec![
            (PLATE_LAYER.to_owned(), plate),
            (AIRSPACE_LAYER.to_owned(), self.layers.airspace),
        ];
        let diagram = self.layers.airport_diagram;
        targets.extend(self.aerodrome_layers.iter().map(|id| (id.clone(), diagram)));
        for (layer, visible) in targets {
            let value = json!(if visible { "visible" } else { "none" });
            self.mutate(&layer, |style| {
                style.set_layout_property(&layer, "visibility", value)
            })?;
        }
        Ok(())
    }

    fn mutate(
        &mut self,
        target: &str,
        apply: impl FnOnce(
            &mut maplibre::style::Style,
        ) -> Result<
            maplibre::style::mutation::StyleChange,
            maplibre::style::mutation::StyleMutationError,
        >,
    ) -> Result<(), MapAdapterError> {
        self.map
            .mutate_style(apply)
            .map(|_change| ())
            .map_err(|source| MapAdapterError::StyleMutation {
                target: target.to_owned(),
                source,
            })
    }
}

fn size([width, height]: [u32; 2]) -> Result<maplibre::window::PhysicalSize, MapAdapterError> {
    validate_size(width, height)
}

fn plugins(tier: CockpitDeviceTier) -> Vec<Box<dyn Plugin<HeadlessEnvironment>>> {
    vec![
        Box::new(RenderPlugin),
        Box::new(BackgroundPlugin),
        Box::new(RasterPlugin::<DefaultRasterTransferables>::default()),
        Box::new(VectorPlugin::<DefaultVectorTransferables>::default()),
        Box::new(HillshadePlugin),
        Box::new(TerrainPlugin::<DefaultDemTransferables>::default()),
        Box::new(DrapeBudgetPlugin(tier.otw_drape_budget())),
        Box::new(PolarElevationsPlugin(EARTH_POLAR_ELEVATIONS)),
        Box::new(OverlayPlugin),
    ]
}

/// A 1 × 1 transparent PNG for the plate source while no plate is shown.
fn transparent_pixel_png() -> Result<Arc<[u8]>, MapAdapterError> {
    let mut cursor = std::io::Cursor::new(Vec::new());
    image::RgbaImage::new(1, 1)
        .write_to(&mut cursor, image::ImageFormat::Png)
        .map_err(|error| MapAdapterError::ChartRaster {
            chart_id: NO_PLATE_CHART_ID.to_owned(),
            detail: error.to_string(),
        })?;
    Ok(Arc::from(cursor.into_inner()))
}

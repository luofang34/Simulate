//! An interactive 3D globe for choosing an airport and runway: free-globe drag and zoom,
//! pixel picking, tiered aerodrome markers, and the selected airport's runways.
//!
//! The globe is one `maplibre` map with its own offscreen texture, sharing the GPU,
//! pipelines and tile cache of a [`SharedMapRuntime`]. Drawn over a thin atmospheric rim,
//! with satellite imagery when real-world tiles are allowed and a dark ocean with a
//! graticule otherwise.

use std::time::Duration;

use cgmath::Point2;
use maplibre::{
    background::BackgroundPlugin,
    coords::LatLon,
    headless::{environment::HeadlessEnvironment, map::HeadlessMap},
    overlay::{OverlayMesh, OverlayPlugin},
    plugin::Plugin,
    raster::{DefaultRasterTransferables, RasterPlugin},
    render::{
        RenderPlugin,
        projection::globe_camera_for_view,
        view_state::{CameraOptions, FlightCurve, FlyOptions, NavigationMode},
    },
    vector::{DefaultVectorTransferables, VectorPlugin},
};
use serde_json::Value;
use web_time::Instant;

use crate::{
    MapAdapterError, SharedMapRuntime, efb_map::effective_globe_zoom, geodesy::wrap_lon_deg,
    runtime::validate_size,
};

mod features;
mod polar_overlay;
mod style;
#[cfg(test)]
mod tests;

pub use features::{GlobeAirportMarker, GlobeRunwayEndOverlay};
use style::{AIRPORTS_SOURCE, RUNWAYS_SOURCE};

/// Lowest and highest zoom the picker flies to.
const ZOOM_RANGE: (f64, f64) = (0.5, 16.0);
/// Default nominal step when no prior frame timestamp exists, and clamp bounds for animation.
const DEFAULT_FRAME_STEP: Duration = Duration::from_millis(16);
const MIN_FRAME_STEP: Duration = Duration::from_millis(1);
const MAX_FRAME_STEP: Duration = Duration::from_millis(50);

/// Interactive airport-selection globe rendering into its own offscreen texture.
pub struct GlobePickerSurface {
    runtime: SharedMapRuntime,
    map: HeadlessMap,
    size_px: [u32; 2],
    last_render: Option<Instant>,
    polar_markers: Vec<GlobeAirportMarker>,
    polar_runways: Vec<GlobeRunwayEndOverlay>,
    last_overlay: OverlayMesh,
}

impl GlobePickerSurface {
    /// Builds the globe over North America and the Atlantic, with satellite imagery when
    /// `real_world_tiles` is set and an offline dark-ocean graticule otherwise.
    pub fn new(
        runtime: &SharedMapRuntime,
        size_px: [u32; 2],
        real_world_tiles: bool,
    ) -> Result<Self, MapAdapterError> {
        let [width, height] = size_px;
        let _valid = validate_size(width, height)?;
        let settings = runtime
            .device_tier()
            .belief_chart_settings(runtime.color_format());
        let (kernel, renderer) = runtime.create_renderer(width, height, settings)?;
        let style = style::picker_style(real_world_tiles)?;
        let plugins: Vec<Box<dyn Plugin<HeadlessEnvironment>>> = vec![
            Box::new(RenderPlugin),
            Box::new(BackgroundPlugin),
            Box::new(RasterPlugin::<DefaultRasterTransferables>::default()),
            Box::new(VectorPlugin::<DefaultVectorTransferables>::default()),
            Box::new(OverlayPlugin),
        ];
        let mut map = runtime
            .with_runtime_context(|| HeadlessMap::new(style, renderer, kernel, plugins))
            .map_err(|source| MapAdapterError::Assemble { source })?;
        map.set_available_memory(runtime.device_tier().available_memory_bytes());
        map.set_navigation_mode(NavigationMode::FreeGlobe)
            .map_err(|source| MapAdapterError::Navigation { source })?;
        map.set_atmosphere(true);
        Ok(Self {
            runtime: runtime.clone(),
            map,
            size_px,
            last_render: None,
            polar_markers: Vec::new(),
            polar_runways: Vec::new(),
            last_overlay: OverlayMesh::default(),
        })
    }

    /// Resizes the globe's texture; zero or unchanged sizes are ignored.
    pub fn resize(&mut self, size_px: [u32; 2]) {
        let [width, height] = size_px;
        if size_px == self.size_px {
            return;
        }
        if let Ok(size) = validate_size(width, height) {
            self.map.resize(size);
            self.size_px = size_px;
        }
    }

    /// Texture size in physical pixels.
    #[must_use]
    pub const fn size_px(&self) -> [u32; 2] {
        self.size_px
    }

    /// Turns the globe so the ground under `from_px` moves under `to_px` (a pixel in space
    /// grabs the nearest point of the limb); returns whether it moved.
    pub fn drag_pixels(&mut self, from_px: [f32; 2], to_px: [f32; 2]) -> bool {
        self.map.drag_globe(point(from_px), point(to_px))
    }

    /// Zooms by `zoom_delta` keeping the ground under `pixel_px` in place; returns whether
    /// the camera moved.
    pub fn zoom_at_pixel(&mut self, pixel_px: [f32; 2], zoom_delta: f64) -> bool {
        self.map.zoom_globe(point(pixel_px), zoom_delta)
    }

    /// Centres the globe on a location at once, at `zoom` (clamped to the picker's range)
    /// or the current zoom.
    pub fn jump_or_fly_to(
        &mut self,
        lat_deg: f64,
        lon_deg: f64,
        zoom: Option<f64>,
    ) -> Result<(), MapAdapterError> {
        self.map
            .jump_to(camera_options(lat_deg, lon_deg, zoom))
            .map_err(|source| MapAdapterError::Navigation { source })
    }

    /// Smoothly flies the globe camera to `(lat_deg, lon_deg)` along `maplibre`'s
    /// van Wijk–Nuij [`FlightCurve::Fly`] arc, zooming out across long distances and
    /// settling at `zoom` (clamped to the picker's range) or the current zoom.
    pub fn fly_to(
        &mut self,
        lat_deg: f64,
        lon_deg: f64,
        zoom: Option<f64>,
    ) -> Result<(), MapAdapterError> {
        let _flight_id = self.map.fly_to(
            camera_options(lat_deg, lon_deg, zoom),
            FlightCurve::Fly(FlyOptions {
                speed: 1.8,
                ..FlyOptions::default()
            }),
        );
        Ok(())
    }

    /// `(lat_deg, lon_deg, zoom)` of the point the camera looks at; latitude and longitude
    /// are NaN if the view has no globe camera.
    #[must_use]
    pub fn camera_center_and_zoom(&self) -> (f64, f64, f64) {
        let view = self.map.view_state();
        let zoom = view.zoom().value();
        globe_camera_for_view(view).map_or((f64::NAN, f64::NAN, zoom), |camera| {
            let center = camera.center();
            (center.latitude, center.longitude, zoom)
        })
    }

    /// The sea-level location under a pixel, or `None` where the pixel shows space.
    #[must_use]
    pub fn screen_to_lat_lon(&self, pixel_px: [f32; 2]) -> Option<(f64, f64)> {
        self.map
            .screen_to_globe_lat_lon(point(pixel_px))
            .map(|location| (location.latitude, location.longitude))
    }

    /// Where a location is drawn, in pixels, and whether the planet hides it.
    #[must_use]
    pub fn project_lat_lon(&self, lat_deg: f64, lon_deg: f64) -> Option<([f32; 2], bool)> {
        self.map
            .globe_lat_lon_to_screen(LatLon::new(lat_deg, lon_deg), 0.0)
            .map(|(pixel, occluded)| ([pixel.x as f32, pixel.y as f32], occluded))
    }

    /// Replaces the aerodrome markers.
    pub fn set_airport_markers(
        &mut self,
        markers: &[GlobeAirportMarker],
    ) -> Result<(), MapAdapterError> {
        self.polar_markers = markers
            .iter()
            .filter(|m| polar_overlay::is_polar_marker(m))
            .cloned()
            .collect();
        self.set_source(AIRPORTS_SOURCE, features::airport_features(markers))
    }

    /// Replaces the selected airport's runway overlay.
    pub fn set_selected_runways(
        &mut self,
        runways: &[GlobeRunwayEndOverlay],
    ) -> Result<(), MapAdapterError> {
        self.polar_runways = runways
            .iter()
            .filter(|r| polar_overlay::is_polar_runway(r))
            .cloned()
            .collect();
        self.set_source(RUNWAYS_SOURCE, features::runway_features(runways))
    }

    fn set_source(&mut self, source_name: &str, data: Value) -> Result<(), MapAdapterError> {
        self.map
            .set_geojson_source(source_name, data)
            .map_err(|source| MapAdapterError::GeoJsonSource {
                source_name: source_name.to_owned(),
                source,
            })
    }

    /// Draws one frame, advancing the camera animation clock by wall-clock elapsed time and
    /// taking in whatever tiles have arrived; returns the texture.
    pub fn render(&mut self) -> Result<Option<&wgpu::TextureView>, MapAdapterError> {
        let now = Instant::now();
        let dt = self
            .last_render
            .map_or(DEFAULT_FRAME_STEP, |prev| {
                now.saturating_duration_since(prev)
            })
            .clamp(MIN_FRAME_STEP, MAX_FRAME_STEP);
        self.last_render = Some(now);
        self.render_step(dt)
    }

    /// Draws one frame after advancing the map's frame clock by `dt`.
    pub fn render_step(
        &mut self,
        dt: Duration,
    ) -> Result<Option<&wgpu::TextureView>, MapAdapterError> {
        let overlay =
            polar_overlay::build_polar_overlay(self, &self.polar_markers, &self.polar_runways);
        if overlay != self.last_overlay {
            self.last_overlay = overlay.clone();
            self.map.set_overlay(overlay);
        }
        self.map.frame_input_mut().advance(dt);
        self.runtime
            .with_runtime_context(|| self.map.run_frame())
            .map_err(|source| MapAdapterError::MapOperation { source })?;
        Ok(self.map.head_texture_view())
    }

    /// Whether requests, arrived tiles, or an active camera flight remain unsettled.
    #[must_use]
    pub fn needs_redraw(&self) -> bool {
        self.map.needs_redraw()
    }

    /// Whether dirty state, an active flight, or arrived worker tiles are ready to draw now.
    #[must_use]
    pub fn has_ready_work(&self) -> bool {
        self.map.has_ready_work()
    }

    /// Copies the last frame into tight RGBA8 bytes (`width * height * 4`).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn read_rgba(&self) -> Result<Vec<u8>, MapAdapterError> {
        let texture = self
            .map
            .head_texture()
            .ok_or_else(|| MapAdapterError::Readback {
                detail: "globe picker has no colour texture".to_owned(),
            })?;
        let gpu = self.runtime.host_gpu();
        crate::runtime::read_texture_rgba_blocking(&gpu.device, &gpu.queue, texture)
    }
}

fn camera_options(lat_deg: f64, lon_deg: f64, zoom: Option<f64>) -> CameraOptions {
    let clamped_lat = lat_deg.clamp(-90.0, 90.0);
    CameraOptions {
        center: Some(LatLon::new(clamped_lat, wrap_lon_deg(lon_deg))),
        zoom: zoom.filter(|z| z.is_finite()).map(|z| {
            effective_globe_zoom(z.clamp(ZOOM_RANGE.0, ZOOM_RANGE.1), clamped_lat)
                .clamp(ZOOM_RANGE.0, ZOOM_RANGE.1)
        }),
        ..CameraOptions::default()
    }
}

fn point([x, y]: [f32; 2]) -> Point2<f64> {
    Point2::new(f64::from(x), f64::from(y))
}

//! The Flight Hub's interactive 3D airport globe: built on first use over the rig's shared
//! runtime, centred on the hub's selection, with the database's visible airports as
//! markers, the selected airport's runways, drag, zoom, and pixel picking.

use sim_maplibre::{GlobeAirportMarker, GlobePickerSurface, GlobeRunwayEndOverlay};
use sim_scenery::{AerodromeTier, AirportDatabase, AirportIdx, AirportRecord};

use super::MapLibreRig;
use crate::error::AppSimError;

/// Most airport markers drawn at once.
const MAX_MARKERS: usize = 180;
/// Least zoom the globe flies to for a selected airport, so its runways read.
const SELECTION_MIN_ZOOM: f64 = 7.8;
/// How near a marker a click must land to pick it, logical pixels.
const PICK_RADIUS_PX: f64 = 18.0;
/// Camera travel that refreshes the markers, degrees of arc and zoom levels.
const MARKER_REFRESH: (f64, f64) = (0.25, 0.1);
/// Earth's equatorial circumference, metres, for the ground size of a pixel.
const EQUATOR_M: f64 = 40_075_016.7;
/// Pixels across a zoom-0 world in the picker's tiles.
const WORLD_PX_AT_ZOOM_0: f64 = 512.0;

/// What the hub has selected, for the globe to centre on and outline.
#[derive(Debug, Clone, PartialEq)]
pub struct GlobeSelection {
    /// Reference point latitude, degrees.
    pub lat_deg: f64,
    /// Reference point longitude, degrees.
    pub lon_deg: f64,
    /// The selected database airport, whose runways the globe draws.
    pub airport: Option<AirportIdx>,
    /// The selected runway end's designator.
    pub runway: Option<String>,
}

/// The picker surface and what it last drew.
pub struct GlobeRig {
    surface: GlobePickerSurface,
    view: Option<wgpu::TextureView>,
    selection: Option<GlobeSelection>,
    markers: Vec<(AirportIdx, f64, f64)>,
    markers_at: Option<([f64; 3], Option<AirportIdx>)>,
    dirty: bool,
}

impl MapLibreRig {
    /// Draws the globe at `size_px`, building it on first use; a new `selection` flies the
    /// globe to it.
    pub fn render_globe_picker(
        &mut self,
        size_px: [u32; 2],
        selection: Option<GlobeSelection>,
    ) -> Result<(), AppSimError> {
        let globe = match self.globe.as_mut() {
            Some(globe) => globe,
            None => {
                let real_world = self.catalog.is_none();
                let surface = GlobePickerSurface::new(&self.runtime, size_px, real_world)?;
                self.globe.insert(GlobeRig {
                    surface,
                    view: None,
                    selection: None,
                    markers: Vec::new(),
                    markers_at: None,
                    dirty: true,
                })
            }
        };
        if globe.surface.size_px() != size_px {
            globe.surface.resize(size_px);
            globe.dirty = true;
        }
        if selection != globe.selection {
            globe.select(selection)?;
        }
        globe.refresh_markers()?;
        if globe.dirty || globe.view.is_none() || globe.surface.needs_redraw() {
            globe.view = globe.surface.render()?.cloned();
            globe.dirty = false;
        }
        Ok(())
    }

    /// The globe's last frame.
    #[must_use]
    pub fn globe_view(&self) -> Option<&wgpu::TextureView> {
        self.globe.as_ref().and_then(|g| g.view.as_ref())
    }

    /// Whether the globe has arrived tiles or an active camera flight waiting to be drawn.
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub fn globe_needs_redraw(&self) -> bool {
        self.globe
            .as_ref()
            .is_some_and(|g| g.dirty || g.surface.needs_redraw())
    }

    /// Turns the globe so the ground under `from_px` follows the cursor to `to_px`
    /// (texture pixels).
    pub fn drag_globe(&mut self, from_px: [f32; 2], to_px: [f32; 2]) {
        if let Some(globe) = self.globe.as_mut() {
            globe.dirty |= globe.surface.drag_pixels(from_px, to_px);
        }
    }

    /// Zooms the globe by `delta` levels about `pixel_px` (texture pixels).
    pub fn zoom_globe(&mut self, pixel_px: [f32; 2], delta: f64) {
        if let Some(globe) = self.globe.as_mut() {
            globe.dirty |= globe.surface.zoom_at_pixel(pixel_px, delta);
        }
    }

    /// The airport a click at `pixel_px` (texture pixels) picks: the nearest drawn marker
    /// within reach, else the nearest airport the zoom shows around the clicked ground.
    #[must_use]
    pub fn pick_airport_at_pixel(&self, pixel_px: [f32; 2]) -> Option<AirportIdx> {
        let globe = self.globe.as_ref()?;
        let reach = PICK_RADIUS_PX * self.pixel_ratio;
        let near_marker = globe
            .markers
            .iter()
            .filter_map(|&(idx, lat, lon)| {
                let (at, occluded) = globe.surface.project_lat_lon(lat, lon)?;
                let d = f64::from(at[0] - pixel_px[0]).hypot(f64::from(at[1] - pixel_px[1]));
                (!occluded && d <= reach).then_some((d, idx))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, idx)| idx);
        near_marker.or_else(|| {
            let (lat, lon) = globe.surface.screen_to_lat_lon(pixel_px)?;
            let (_, _, zoom) = globe.surface.camera_center_and_zoom();
            let radius_km = pick_radius_km(lat, zoom, reach / self.pixel_ratio);
            let tier = AerodromeTier::max_visible_at_zoom(zoom);
            AirportDatabase::builtin().nearest(lat, lon, radius_km, tier)
        })
    }
}

impl GlobeRig {
    fn select(&mut self, selection: Option<GlobeSelection>) -> Result<(), AppSimError> {
        self.dirty = true;
        let database = AirportDatabase::builtin();
        if let Some(sel) = &selection {
            let first = self.selection.is_none();
            let moved = self
                .selection
                .as_ref()
                .is_none_or(|old| (old.lat_deg, old.lon_deg) != (sel.lat_deg, sel.lon_deg));
            if moved {
                let (_, _, zoom) = self.surface.camera_center_and_zoom();
                let target_zoom =
                    Some((if zoom.is_finite() { zoom } else { 0.0 }).max(SELECTION_MIN_ZOOM));
                if first {
                    self.surface
                        .jump_or_fly_to(sel.lat_deg, sel.lon_deg, target_zoom)?;
                } else {
                    self.surface.fly_to(sel.lat_deg, sel.lon_deg, target_zoom)?;
                }
            }
        }
        let runways = selection
            .as_ref()
            .and_then(|sel| {
                let record = database.get(sel.airport?)?;
                Some(runway_overlays(record, sel.runway.as_deref()))
            })
            .unwrap_or_default();
        self.surface.set_selected_runways(&runways)?;
        self.selection = selection;
        Ok(())
    }

    /// Re-queries the visible airports when the camera or the selection moved.
    fn refresh_markers(&mut self) -> Result<(), AppSimError> {
        let (lat, lon, zoom) = self.surface.camera_center_and_zoom();
        let (lat, lon) = if lat.is_finite() && lon.is_finite() {
            (lat, lon)
        } else {
            self.selection
                .as_ref()
                .map_or((40.0, -74.0), |s| (s.lat_deg, s.lon_deg))
        };
        let pinned = self.selection.as_ref().and_then(|s| s.airport);
        let (arc, levels) = MARKER_REFRESH;
        let fresh = self.markers_at.is_some_and(|([la, lo, z], p)| {
            p == pinned
                && (la - lat).abs() < arc
                && (lo - lon).abs() < arc
                && (z - zoom).abs() < levels
        });
        if fresh {
            return Ok(());
        }
        let database = AirportDatabase::builtin();
        let shown = database.query_globe_visible_pinned(lat, lon, zoom, MAX_MARKERS, pinned);
        let markers: Vec<GlobeAirportMarker> = shown
            .iter()
            .filter_map(|&idx| {
                let record = database.get(idx)?;
                Some(GlobeAirportMarker {
                    code: record.ids.primary.as_str().to_owned(),
                    label: record.ids.primary.as_str().to_owned(),
                    lat_deg: record.lat_deg,
                    lon_deg: record.lon_deg,
                    tier: record.tier as u8,
                    towered: record.towered,
                    selected: Some(idx) == pinned,
                })
            })
            .collect();
        self.surface.set_airport_markers(&markers)?;
        self.markers = shown
            .iter()
            .filter_map(|&idx| database.get(idx).map(|r| (idx, r.lat_deg, r.lon_deg)))
            .collect();
        self.markers_at = Some(([lat, lon, zoom], pinned));
        self.dirty = true;
        Ok(())
    }
}

/// Both ends of every runway of `record`, the end named `selected` highlighted.
#[must_use]
pub fn runway_overlays(
    record: &AirportRecord,
    selected: Option<&str>,
) -> Vec<GlobeRunwayEndOverlay> {
    record
        .runways
        .iter()
        .flat_map(|runway| {
            let [a, b] = runway.ends;
            [(a, b), (b, a)].map(|(end, other)| GlobeRunwayEndOverlay {
                designator: end.designator.as_str().to_owned(),
                threshold_lat_deg: end.lat_deg,
                threshold_lon_deg: end.lon_deg,
                opposite_lat_deg: other.lat_deg,
                opposite_lon_deg: other.lon_deg,
                width_m: runway.width_m,
                selected: selected == Some(end.designator.as_str()),
            })
        })
        .collect()
}

/// Ground distance `reach_px` logical pixels cover at `lat_deg` and `zoom`, kilometres.
#[must_use]
pub fn pick_radius_km(lat_deg: f64, zoom: f64, reach_px: f64) -> f64 {
    let clamped_lat = lat_deg.abs().min(85.051_129);
    let metres_per_px =
        EQUATOR_M * clamped_lat.to_radians().cos() / (WORLD_PX_AT_ZOOM_0 * zoom.exp2());
    (reach_px * metres_per_px / 1_000.0).clamp(0.5, 400.0)
}

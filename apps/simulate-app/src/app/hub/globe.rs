//! The World tab's live 3D globe in the app: drawn into the hub's preview card for the
//! selected aerodrome, turned by drag, zoomed by the wheel about the cursor, and clicked
//! to pick an airport.

use tracing::{info, warn};

use super::super::FlightApp;
use crate::cockpit_rig::CockpitRig;
use crate::map_rig::GlobeSelection;

/// Zoom levels one mouse-wheel row moves the globe.
const ZOOM_PER_ROW: f64 = 0.35;
/// Most pointer travel, logical pixels, that still counts as a click rather than a drag.
const CLICK_SLOP_PX: f64 = 5.0;

impl FlightApp {
    /// The hub's selected aerodrome as the globe centres on and outlines it.
    fn globe_selection(&self) -> Option<GlobeSelection> {
        let entry = self.catalog.aerodromes.get(self.esc_menu.aerodrome())?;
        Some(GlobeSelection {
            lat_deg: entry.lat_deg?,
            lon_deg: entry.lon_deg?,
            airport: entry.airport_idx,
            runway: entry
                .runways
                .get(self.esc_menu.runway())
                .map(|r| r.designator.clone()),
        })
    }

    /// Draws the globe at `size_px` for the current selection.
    pub(in crate::app) fn render_globe(&mut self, size_px: [u32; 2]) {
        let selection = self.globe_selection();
        let Some(maps) = self.cockpit_rig.as_mut().and_then(CockpitRig::maps_mut) else {
            return;
        };
        if let Err(err) = maps.render_globe_picker(size_px, selection) {
            warn!(error = %err, "airport globe unavailable");
        }
    }

    /// `cursor_px` (physical window pixels) in the globe texture's pixels.
    fn globe_pixel(&self, cursor_px: [f64; 2]) -> Option<[f32; 2]> {
        let rect = self.hub_preview?;
        let [sw, sh] = self.surface_size().map(|v| f64::from(v.max(1)));
        Some([
            (cursor_px[0] - f64::from(rect.x) * sw) as f32,
            (cursor_px[1] - f64::from(rect.y) * sh) as f32,
        ])
    }

    /// Turns the globe as the cursor drags from `last` to `position`.
    pub(in crate::app) fn drag_globe(&mut self, last: [f64; 2], position: [f64; 2]) {
        let (Some(from), Some(to)) = (self.globe_pixel(last), self.globe_pixel(position)) else {
            return;
        };
        if let Some(maps) = self.cockpit_rig.as_mut().and_then(CockpitRig::maps_mut) {
            maps.drag_globe(from, to);
        }
    }

    /// Zooms the globe about the cursor by `rows` wheel rows (positive zooms out).
    pub(in crate::app) fn zoom_globe(&mut self, rows: f64) {
        let Some(at) = self.globe_pixel(self.cursor_px) else {
            return;
        };
        if let Some(maps) = self.cockpit_rig.as_mut().and_then(CockpitRig::maps_mut) {
            maps.zoom_globe(at, -rows * ZOOM_PER_ROW);
        }
    }

    /// A release at `position` after pressing at `origin`: barely moved, it picks the
    /// airport under the cursor and selects it in the hub.
    pub(in crate::app) fn release_globe(&mut self, origin: [f64; 2], position: [f64; 2]) {
        let scale = self.window.as_ref().map_or(1.0, |w| w.scale_factor());
        let travel = (position[0] - origin[0]).hypot(position[1] - origin[1]) / scale;
        if travel >= CLICK_SLOP_PX {
            return;
        }
        let Some(at) = self.globe_pixel(position) else {
            return;
        };
        let picked = self
            .cockpit_rig
            .as_ref()
            .and_then(CockpitRig::maps)
            .and_then(|maps| maps.pick_airport_at_pixel(at));
        if let Some(index) = picked.and_then(|idx| self.catalog.index_of_airport(idx)) {
            self.esc_menu.select_aerodrome(index);
            if let Some(entry) = self.catalog.aerodromes.get(index) {
                info!(airport = entry.code, "picked airport on the globe");
            }
        }
    }
}

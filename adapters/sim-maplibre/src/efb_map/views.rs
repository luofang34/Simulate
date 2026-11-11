//! Drawing the EFB's views: each shown view follows ownship with its own camera, carries
//! its own symbology, and every shown view draws in one frame of the shared map.

use maplibre::{
    coords::{LatLon, Zoom},
    headless::{CoveringPolicy, MultiViewFrame, SlotView, ViewSlotEntry},
};

use super::{EfbDisplayMode, EfbMapSurface, FRAME_INTERVAL, effective_globe_zoom, mode_index};
use crate::{
    ChartCameraSpec, ChartSymbologyInput, MapAdapterError, build_chart_symbology_scaled,
    chart_symbology::append_sigma_rings, geodesy::wrap_lon_deg,
};

/// Both views, in the order their slots draw.
const VIEWS: [EfbDisplayMode; 2] = [EfbDisplayMode::CompactCorner, EfbDisplayMode::ExpandedEfb];

impl EfbMapSurface {
    /// Draws every view as well as the focused one each frame (`true`), as a host with
    /// room for both does, or only the focused one (`false`), as a cockpit does.
    pub fn set_all_views_shown(&mut self, all: bool) {
        self.all_views = all;
    }

    /// Whether every view is drawn each frame.
    #[must_use]
    pub const fn all_views_shown(&self) -> bool {
        self.all_views
    }

    /// The views drawn each frame.
    fn shown_views(&self) -> impl Iterator<Item = EfbDisplayMode> + use<> {
        let (all, focus) = (self.all_views, self.focus);
        VIEWS.into_iter().filter(move |mode| all || *mode == focus)
    }

    /// The camera `mode`'s view drew with last.
    #[must_use]
    pub fn camera_spec(&self, mode: EfbDisplayMode) -> Option<ChartCameraSpec> {
        self.cameras
            .get(mode_index(mode))
            .map(super::camera::EfbCamera::spec)
    }

    /// Draws the shown views: each follows ownship per its follow mode and draws the
    /// symbology of `symbology` (its camera and viewport are replaced by the view's own)
    /// and the plan's rings. Returns the focused view's frame.
    pub fn render(
        &mut self,
        symbology: &ChartSymbologyInput<'_>,
    ) -> Result<Option<&wgpu::TextureView>, MapAdapterError> {
        let route = self.layers.route_and_traffic;
        let ratio = self.pixel_ratio();
        let mut slots = Vec::with_capacity(VIEWS.len());
        for mode in self.shown_views() {
            let Some(camera) = self.cameras.get_mut(mode_index(mode)) else {
                continue;
            };
            camera.follow_ownship(symbology.ownship.as_ref());
            let spec = camera.spec();
            let viewport_px = self.size_of(mode);
            let mut mesh = build_chart_symbology_scaled(
                &ChartSymbologyInput {
                    camera: spec,
                    viewport_px,
                    ownship: symbology.ownship,
                    runway: symbology.runway.filter(|_| route),
                    traffic: if route { symbology.traffic } else { &[] },
                },
                ratio,
            );
            append_sigma_rings(
                &mut mesh,
                (&spec, viewport_px, ratio),
                &self.plan.tracks.rings,
            );
            let slot = self.slot_of(mode);
            self.map
                .set_view_slot_overlay(slot, mesh)
                .map_err(|source| MapAdapterError::MultiView { source })?;
            let center_lat = spec.center_lat_deg.clamp(-90.0, 90.0);
            slots.push(ViewSlotEntry {
                slot,
                view: SlotView::Camera {
                    center: LatLon::new(center_lat, wrap_lon_deg(spec.center_lon_deg)),
                    zoom: Zoom::new(effective_globe_zoom(spec.zoom, center_lat)),
                    bearing_deg: spec.bearing_deg,
                    pitch_deg: 0.0,
                },
                policy: CoveringPolicy::Independent,
            });
        }
        self.frames = self.frames.wrapping_add(1);
        let frame = MultiViewFrame {
            timestamp: FRAME_INTERVAL
                .saturating_mul(u32::try_from(self.frames).unwrap_or(u32::MAX)),
            opaque_environment: true,
            request_overscan: 1.0,
            slots,
        };
        self.runtime
            .with_runtime_context(|| self.map.run_multi_view_frame(frame))
            .map_err(|source| MapAdapterError::MultiView { source })?;
        Ok(self.map.view_slot_color_view(self.active_slot()))
    }

    /// `mode`'s last frame, once it has drawn one.
    #[must_use]
    pub fn view_of(&self, mode: EfbDisplayMode) -> Option<&wgpu::TextureView> {
        self.map.view_slot_color_view(self.slot_of(mode))
    }
}

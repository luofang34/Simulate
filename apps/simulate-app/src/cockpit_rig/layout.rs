//! Instrument dock geometry: `4:3` panels whose normalized rectangles keep their pixel
//! aspect on any window, so round gauges stay round.

use glam::DVec3;
use sim_core::VehicleState;
use sim_geo::{BodyEllipsoid, LocalFrame, ecef_to_geodetic};
use sim_maplibre::{DynamicEntity3D, HostGpu, default_render_body};
use sim_render::{Camera, OverlayQuad, ViewportRect};
use sim_scenery::CatalogVerticalDatum;
use tracing::warn;

use super::efb_chrome::rect_px;
use crate::avionics::{ConformalSurface, HudView};
use crate::map_rig::{AvionicsPreset, MapLibreRig, ScenerySource};
use crate::world_surface::WorldSurfaceHandle;

/// Where the flight's displays get what they draw this frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct FlightSources {
    /// Whether the onboard instruments have a source to draw.
    pub avionics: bool,
    /// Whether the flight is a link's page's, whose route the app's own aerodrome and
    /// moving map do not show.
    pub page: bool,
}

impl Default for FlightSources {
    fn default() -> Self {
        Self {
            avionics: true,
            page: false,
        }
    }
}

impl FlightSources {
    /// The sources of `flight` now.
    pub(super) fn of(flight: &crate::flight::Flight) -> Self {
        Self {
            avionics: flight.onboard_avionics_shown(),
            page: flight.shows_page(),
        }
    }
}

/// Attempts to create [`MapLibreRig`], logging and returning `None` on initialization failure.
pub(super) fn try_build_maps(
    host_gpu: HostGpu,
    size: [u32; 2],
    source: &ScenerySource,
    preset: AvionicsPreset,
    surface: WorldSurfaceHandle,
) -> Option<MapLibreRig> {
    MapLibreRig::new(host_gpu, size, source, preset, surface)
        .inspect_err(|err| {
            warn!(error = %err, "MapLibre maps unavailable; using GlobeScene fallback");
        })
        .ok()
}

/// Panel width over height (`480 × 360`).
const PANEL_ASPECT: f32 = 4.0 / 3.0;
/// Panel height as a fraction of the window when the width allows it.
const PANEL_HEIGHT_FRACTION: f32 = 0.34;
/// Gap between panels and window edges, as a fraction of the window width.
const GAP_X: f32 = 0.015;
/// Gap under the dock, as a fraction of the window height.
const GAP_BOTTOM: f32 = 0.008;

/// `count` panels docked along the bottom: first left, last right, any middle centred.
pub(super) fn dock_rects([sw, sh]: [u32; 2], count: usize) -> Vec<ViewportRect> {
    let (sw, sh) = (sw.max(1) as f32, sh.max(1) as f32);
    let n = count.max(1) as f32;
    let mut height = PANEL_HEIGHT_FRACTION;
    let mut width = height * PANEL_ASPECT * sh / sw;
    let room = 1.0 - (n + 1.0) * GAP_X;
    if width * n > room {
        width = room / n;
        height = width * sw / (sh * PANEL_ASPECT);
    }
    let y = 1.0 - height - GAP_BOTTOM;
    (0..count)
        .map(|i| {
            let x = if i == 0 {
                GAP_X
            } else if i + 1 == count {
                1.0 - GAP_X - width
            } else {
                let span = 1.0 - 2.0 * GAP_X - width;
                GAP_X + span * i as f32 / (n - 1.0)
            };
            ViewportRect {
                x,
                y,
                width,
                height,
            }
        })
        .collect()
}

/// True heading of the nose, radians.
pub(super) fn heading_rad(vehicle: &VehicleState) -> f64 {
    let local = LocalFrame::at(&ecef_to_geodetic(vehicle.position_ecef));
    let nose = vehicle.attitude * DVec3::X;
    nose.dot(local.east()).atan2(nose.dot(local.north()))
}

/// Physical pixel dimensions of a docked panel for `surface_size`.
pub(super) fn dock_panel_px(surface_size: [u32; 2], has_dock_chart: bool) -> Option<[u32; 2]> {
    let rects = dock_rects(surface_size, if has_dock_chart { 3 } else { 2 });
    rects.first().map(|&r| rect_px(r, surface_size))
}

/// Appends the docked PFD (with optional SVS underlay), optional MFD chart, and HSI quads.
pub(super) fn push_dock_quads<'a>(
    quads: &mut Vec<OverlayQuad<'a>>,
    surface_size: [u32; 2],
    svs_view: Option<&'a wgpu::TextureView>,
    pfd_view: &'a wgpu::TextureView,
    chart_view: Option<&'a wgpu::TextureView>,
    hsi_view: &'a wgpu::TextureView,
) {
    let rects = dock_rects(surface_size, if chart_view.is_some() { 3 } else { 2 });
    let (Some(&pfd_rect), Some(&hsi_rect)) = (rects.first(), rects.last()) else {
        return;
    };
    if let Some(svs) = svs_view {
        quads.push(OverlayQuad::inset_map(pfd_rect, svs, false));
    }
    quads.push(OverlayQuad::vector_panel(
        pfd_rect,
        pfd_view,
        svs_view.is_none(),
        true,
    ));
    if let (Some(chart), Some(&mfd_rect)) = (chart_view, rects.get(1)) {
        quads.push(OverlayQuad::inset_map(mfd_rect, chart, true));
    }
    quads.push(OverlayQuad::vector_panel(hsi_rect, hsi_view, true, true));
}

/// Conformal HUD camera and surface projection parameters for `camera`.
pub(super) fn hud_view<'a>(
    surface_size: [u32; 2],
    body: BodyEllipsoid,
    map_sphere_active: bool,
    camera: &Camera,
    truth_vehicle: &VehicleState,
    traffic: &'a [DynamicEntity3D],
    vertical_datum: CatalogVerticalDatum,
) -> HudView<'a> {
    let eye_body_lever_m = truth_vehicle.attitude.inverse()
        * (camera.pose.position_ecef - truth_vehicle.position_ecef);
    let conformal_surface = if map_sphere_active {
        ConformalSurface::GlobeSphere {
            physics: body,
            render_body: default_render_body(body),
        }
    } else {
        ConformalSurface::Ellipsoid { physics: body }
    };
    HudView {
        surface_size,
        camera: *camera,
        traffic,
        airframe_attitude: truth_vehicle.attitude,
        eye_body_lever_m,
        conformal_surface,
        vertical_datum: (body == BodyEllipsoid::WGS84).then_some(vertical_datum),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn docked_panels_keep_four_by_three_pixels_on_a_wide_window() {
        let size = [1280_u32, 720_u32];
        let rects = dock_rects(size, 3);
        assert_eq!(rects.len(), 3);
        for rect in &rects {
            let aspect = (rect.width * 1280.0) / (rect.height * 720.0);
            assert!((aspect - PANEL_ASPECT).abs() < 1e-4, "{aspect}");
            assert!((rect.y + rect.height + GAP_BOTTOM - 1.0).abs() < 1e-6);
        }
        assert!((rects[0].width - 0.255).abs() < 1e-3);
        assert!((rects[1].x + rects[1].width * 0.5 - 0.5).abs() < 1e-4);
    }

    #[test]
    fn a_narrow_window_shrinks_the_dock_instead_of_overlapping() {
        let rects = dock_rects([600, 900], 3);
        for pair in rects.windows(2) {
            assert!(pair[0].x + pair[0].width <= pair[1].x + 1e-5);
        }
        let r = rects[2];
        assert!(r.x + r.width <= 1.0 + 1e-5);
        assert!(((r.width * 600.0) / (r.height * 900.0) - PANEL_ASPECT).abs() < 1e-4);
    }
}

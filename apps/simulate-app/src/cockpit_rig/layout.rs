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
use super::safe_area::SafeAreaInsets;
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
    page_view: bool,
) -> Option<MapLibreRig> {
    let built = if page_view {
        MapLibreRig::new_for_page(host_gpu, size, source, preset, surface)
    } else {
        MapLibreRig::new(host_gpu, size, source, preset, surface)
    };
    built
        .inspect_err(|err| {
            warn!(error = %err, "MapLibre maps unavailable; using GlobeScene fallback");
        })
        .ok()
}

/// Preferred arrangement of docked primary flight instruments (`PFD` + `HSI`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InstrumentDockStyle {
    /// Traditional side-by-side strip docked along the bottom edge (desktop default).
    #[default]
    BottomRow,
    /// Vertical `PFD` (top) + `HSI` (bottom) stack on the left edge so the center windshield
    /// stays clear on compact landscape displays (handheld default).
    LeftRail,
}

impl InstrumentDockStyle {
    /// Toggles between vertical left-rail and bottom-row docks.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "used by wasm32 touch controls and tests")
    )]
    #[must_use]
    pub const fn toggled(self) -> Self {
        match self {
            Self::BottomRow => Self::LeftRail,
            Self::LeftRail => Self::BottomRow,
        }
    }
}

/// Integer-aligned dock rectangles and panel pixel resolution for one cockpit frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PanelDockLayout {
    pub pfd: ViewportRect,
    pub mfd: Option<ViewportRect>,
    pub hsi: ViewportRect,
    pub panel_px: [u32; 2],
    pub right_edge_norm: f32,
}

/// Panel width over height (`480 × 360`).
const PANEL_ASPECT: f32 = 4.0 / 3.0;
/// Panel height as a fraction of the window when the width allows it.
const PANEL_HEIGHT_FRACTION: f32 = 0.34;
/// Gap between panels and window edges, as a fraction of the window width.
const GAP_X: f32 = 0.015;
/// Gap under the dock, as a fraction of the window height.
const GAP_BOTTOM: f32 = 0.008;

fn px_to_viewport_rect([x, y, w, h]: [u32; 4], [sw, sh]: [u32; 2]) -> ViewportRect {
    let sw = sw.max(1) as f32;
    let sh = sh.max(1) as f32;
    ViewportRect {
        x: x as f32 / sw,
        y: y as f32 / sh,
        width: w as f32 / sw,
        height: h as f32 / sh,
    }
}

/// Converts a normalized `rect` into integer swapchain pixels `[x, y, width, height]` clamped
/// strictly inside `surface_size`.
#[must_use]
pub(super) fn viewport_to_px_rect(rect: ViewportRect, [sw, sh]: [u32; 2]) -> [u32; 4] {
    let sw_i = sw.max(1);
    let sh_i = sh.max(1);
    let max_x = sw_i.saturating_sub(1);
    let max_y = sh_i.saturating_sub(1);
    let x = (rect.x * sw_i as f32).round().clamp(0.0, max_x as f32) as u32;
    let y = (rect.y * sh_i as f32).round().clamp(0.0, max_y as f32) as u32;
    let w = (rect.width * sw_i as f32)
        .round()
        .clamp(1.0, (sw_i - x) as f32) as u32;
    let h = (rect.height * sh_i as f32)
        .round()
        .clamp(1.0, (sh_i - y) as f32) as u32;
    [x, y, w, h]
}

fn snap_rect(rect: ViewportRect, [pw, ph]: [u32; 2], [sw, sh]: [u32; 2]) -> ViewportRect {
    let sw_i = sw.max(1);
    let sh_i = sh.max(1);
    let w = pw.clamp(1, sw_i);
    let h = ph.clamp(1, sh_i);
    let x = (rect.x * sw_i as f32)
        .round()
        .clamp(0.0, sw_i.saturating_sub(w) as f32) as u32;
    let y = (rect.y * sh_i as f32)
        .round()
        .clamp(0.0, sh_i.saturating_sub(h) as f32) as u32;
    px_to_viewport_rect([x, y, w, h], [sw_i, sh_i])
}

/// Computes integer-snapped dock geometry for `style` and `has_dock_chart`, clear of the
/// window edges `insets` says the system covers.
pub(super) fn dock_layout(
    surface_size: [u32; 2],
    style: InstrumentDockStyle,
    has_dock_chart: bool,
    insets: SafeAreaInsets,
) -> PanelDockLayout {
    match style {
        InstrumentDockStyle::BottomRow => bottom_row_layout(surface_size, has_dock_chart, insets),
        InstrumentDockStyle::LeftRail => left_rail_layout(surface_size, has_dock_chart, insets),
    }
}

fn bottom_row_layout(
    surface_size: [u32; 2],
    has_dock_chart: bool,
    insets: SafeAreaInsets,
) -> PanelDockLayout {
    let rects = dock_rects(surface_size, if has_dock_chart { 3 } else { 2 }, insets);
    let panel_px = rects
        .first()
        .map_or([64, 48], |&r| rect_px(r, surface_size));
    let pfd = rects.first().map_or(ViewportRect::FULL, |&r| {
        snap_rect(r, panel_px, surface_size)
    });
    let mfd = has_dock_chart
        .then(|| rects.get(1).copied())
        .flatten()
        .map(|r| snap_rect(r, panel_px, surface_size));
    let hsi = rects.last().map_or(ViewportRect::FULL, |&r| {
        snap_rect(r, panel_px, surface_size)
    });
    PanelDockLayout {
        pfd,
        mfd,
        hsi,
        panel_px,
        right_edge_norm: 0.0,
    }
}

/// The rail's share of the window height, the rest split above and below it.
const RAIL_HEIGHT_FRACTION: f32 = 0.88;

fn left_rail_layout(
    surface_size: [u32; 2],
    has_dock_chart: bool,
    insets: SafeAreaInsets,
) -> PanelDockLayout {
    let sw_i = surface_size[0].max(1);
    let sh_i = surface_size[1].max(1);
    let (sw, sh) = (sw_i as f32, sh_i as f32);
    let edge_px = (sw * 0.014).round().clamp(4.0, 24.0);
    let band_px = sh * (1.0 - RAIL_HEIGHT_FRACTION) * 0.5;
    let [left_px, top_px, right_px, bottom_px] =
        insets.margins_px([sw, sh], [edge_px, band_px, edge_px, band_px]);
    let gap_y_px = (sh * 0.014).round().clamp(6.0, 14.0);
    let room_w_px = (sw - left_px - right_px).max(1.0);
    let room_h_px = (sh - top_px - bottom_px).max(1.0);
    let max_w_px = (sw * if sh_i > sw_i { 0.36 } else { 0.30 }).min(room_w_px);
    let max_h_px = ((room_h_px - gap_y_px) * 0.5).floor().max(1.0);
    let w_px = (max_w_px.min(max_h_px * PANEL_ASPECT).floor().max(1.0) as u32).min(sw_i);
    let h_px = ((w_px as f32 / PANEL_ASPECT).round().max(1.0) as u32).min(sh_i);
    let gap_y = (gap_y_px as u32).min(sh_i.saturating_sub(h_px.saturating_mul(2)));
    let total_h = h_px.saturating_mul(2).saturating_add(gap_y).min(sh_i);
    let slack_px = (room_h_px - total_h as f32).max(0.0) * 0.5;
    let top_y = ((top_px + slack_px).round() as u32).min(sh_i - total_h);
    let x = (left_px.round() as u32).min(sw_i - w_px);
    let pfd = px_to_viewport_rect([x, top_y, w_px, h_px], [sw_i, sh_i]);
    let hsi_y = top_y
        .saturating_add(h_px)
        .saturating_add(gap_y)
        .min(sh_i - h_px);
    let hsi = px_to_viewport_rect([x, hsi_y, w_px, h_px], [sw_i, sh_i]);
    let right_edge_px = x.saturating_add(w_px);
    let mfd = has_dock_chart.then(|| {
        let gap_x = (sw * 0.015).round().max(8.0) as u32;
        let mfd_x = right_edge_px.saturating_add(gap_x).min(sw_i - w_px);
        let mfd_y = (sh_i - h_px).saturating_sub(bottom_px.max(sh * 0.012).round() as u32);
        px_to_viewport_rect([mfd_x, mfd_y, w_px, h_px], [sw_i, sh_i])
    });
    PanelDockLayout {
        pfd,
        mfd,
        hsi,
        panel_px: [w_px, h_px],
        right_edge_norm: right_edge_px as f32 / sw,
    }
}

/// `count` panels docked along the bottom: first left, last right, any middle centred.
pub(super) fn dock_rects(
    [sw, sh]: [u32; 2],
    count: usize,
    insets: SafeAreaInsets,
) -> Vec<ViewportRect> {
    let (sw, sh) = (sw.max(1) as f32, sh.max(1) as f32);
    let n = count.max(1) as f32;
    let [left, _, right, bottom] = insets.margins_px([1.0, 1.0], [GAP_X, 0.0, GAP_X, GAP_BOTTOM]);
    let mut height = PANEL_HEIGHT_FRACTION;
    let mut width = height * PANEL_ASPECT * sh / sw;
    let room = (1.0 - left - right - (n - 1.0) * GAP_X).max(0.0);
    if width * n > room {
        width = room / n;
        height = width * sw / (sh * PANEL_ASPECT);
    }
    let y = 1.0 - height - bottom;
    (0..count)
        .map(|i| {
            let x = if i == 0 {
                left
            } else if i + 1 == count {
                1.0 - right - width
            } else {
                let span = 1.0 - left - right - width;
                left + span * i as f32 / (n - 1.0)
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

/// Physical pixel dimensions of a docked panel for `surface_size` and `style`.
pub(super) fn dock_panel_px(
    surface_size: [u32; 2],
    style: InstrumentDockStyle,
    has_dock_chart: bool,
    insets: SafeAreaInsets,
) -> Option<[u32; 2]> {
    Some(dock_layout(surface_size, style, has_dock_chart, insets).panel_px)
}

/// Appends the docked PFD (with optional SVS underlay), optional MFD chart, and HSI quads.
pub(super) fn push_dock_quads<'a>(
    quads: &mut Vec<OverlayQuad<'a>>,
    dock: &PanelDockLayout,
    svs_view: Option<&'a wgpu::TextureView>,
    pfd_view: Option<&'a wgpu::TextureView>,
    chart_view: Option<&'a wgpu::TextureView>,
    hsi_view: Option<&'a wgpu::TextureView>,
) {
    if let Some(svs) = svs_view {
        quads.push(OverlayQuad::inset_map(dock.pfd, svs, false));
    }
    if let Some(pfd) = pfd_view {
        quads.push(OverlayQuad::vector_panel(
            dock.pfd,
            pfd,
            svs_view.is_none(),
            true,
        ));
    }
    if let (Some(chart), Some(mfd_rect)) = (chart_view, dock.mfd) {
        quads.push(OverlayQuad::inset_map(mfd_rect, chart, true));
    }
    if let Some(hsi) = hsi_view {
        quads.push(OverlayQuad::vector_panel(dock.hsi, hsi, true, true));
    }
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
        let rects = dock_rects(size, 3, SafeAreaInsets::NONE);
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
        let rects = dock_rects([600, 900], 3, SafeAreaInsets::NONE);
        for pair in rects.windows(2) {
            assert!(pair[0].x + pair[0].width <= pair[1].x + 1e-5);
        }
        let r = rects[2];
        assert!(r.x + r.width <= 1.0 + 1e-5);
        assert!(((r.width * 600.0) / (r.height * 900.0) - PANEL_ASPECT).abs() < 1e-4);
    }

    #[test]
    fn left_rail_stacks_pfd_above_hsi_without_overlap_or_aspect_distortion() {
        for size in [[1704_u32, 786_u32], [786_u32, 1704_u32]] {
            let layout = dock_layout(
                size,
                InstrumentDockStyle::LeftRail,
                false,
                SafeAreaInsets::NONE,
            );
            let pfd = viewport_to_px_rect(layout.pfd, size);
            let hsi = viewport_to_px_rect(layout.hsi, size);
            assert_eq!([pfd[2], pfd[3]], layout.panel_px);
            assert_eq!([hsi[2], hsi[3]], layout.panel_px);
            assert_eq!(pfd[0], hsi[0]);
            assert!(pfd[1] + pfd[3] < hsi[1], "PFD must sit cleanly above HSI");
            assert!(hsi[1] + hsi[3] <= size[1]);
            assert!(layout.right_edge_norm > 0.15 && layout.right_edge_norm < 0.45);
            let aspect = pfd[2] as f32 / pfd[3] as f32;
            assert!((aspect - PANEL_ASPECT).abs() < 0.02, "{aspect}");
        }
    }

    #[test]
    fn docks_stay_inside_a_surface_of_any_size() {
        let insets = SafeAreaInsets::from_lengths([59.0, 59.0, 59.0, 21.0], [402.0, 402.0]);
        for size in [
            [1_u32, 1_u32],
            [60, 1],
            [1, 60],
            [47, 47],
            [300, 20],
            [20, 300],
        ] {
            for style in [
                InstrumentDockStyle::LeftRail,
                InstrumentDockStyle::BottomRow,
            ] {
                for insets in [SafeAreaInsets::NONE, insets] {
                    let layout = dock_layout(size, style, true, insets);
                    for rect in [Some(layout.pfd), layout.mfd, Some(layout.hsi)]
                        .into_iter()
                        .flatten()
                    {
                        let [x, y, w, h] = viewport_to_px_rect(rect, size);
                        assert!(w >= 1 && h >= 1, "{size:?} {style:?}");
                        assert!(x + w <= size[0].max(1), "{size:?} {style:?}");
                        assert!(y + h <= size[1].max(1), "{size:?} {style:?}");
                    }
                }
            }
        }
    }

    #[test]
    fn the_rail_clears_a_landscape_phone_sensor_housing() {
        let size = [1748_u32, 804_u32];
        let housing = SafeAreaInsets::from_lengths([59.0, 0.0, 59.0, 21.0], [874.0, 402.0]);
        let bare = dock_layout(
            size,
            InstrumentDockStyle::LeftRail,
            false,
            SafeAreaInsets::NONE,
        );
        let clear = dock_layout(size, InstrumentDockStyle::LeftRail, false, housing);
        let [bare_x, ..] = viewport_to_px_rect(bare.pfd, size);
        let [x, y, w, h] = viewport_to_px_rect(clear.pfd, size);
        let [_, hsi_y, _, hsi_h] = viewport_to_px_rect(clear.hsi, size);
        assert!(
            bare_x < 118,
            "an uncovered window keeps the standard margin"
        );
        assert_eq!(x, 118, "the larger of margin and inset, not their sum");
        assert!(y + h < hsi_y);
        assert!(hsi_y + hsi_h <= 804 - 42);
        assert!((w as f32 / h as f32 - PANEL_ASPECT).abs() < 0.02);
    }

    #[test]
    fn the_bottom_row_clears_both_side_insets_and_the_home_indicator() {
        let size = [1748_u32, 804_u32];
        let housing = SafeAreaInsets::from_lengths([59.0, 0.0, 59.0, 21.0], [874.0, 402.0]);
        let rects = dock_rects(size, 2, housing);
        let first = viewport_to_px_rect(rects[0], size);
        let last = viewport_to_px_rect(rects[1], size);
        assert!(first[0] >= 118);
        assert!(last[0] + last[2] <= 1748 - 118);
        assert!(last[1] + last[3] <= 804 - 42);
    }
}

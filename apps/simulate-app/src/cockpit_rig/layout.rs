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
use super::safe_area::{DockClearance, SafeAreaInsets};
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

/// The arrangement drawn on a `surface_size` window when `style` is asked for: a window
/// taller than wide has no room for a row beside the thumb controls, so it takes the rail.
#[must_use]
pub(super) const fn drawn_style(
    style: InstrumentDockStyle,
    [width, height]: [u32; 2],
) -> InstrumentDockStyle {
    if height > width {
        InstrumentDockStyle::LeftRail
    } else {
        style
    }
}

/// Computes integer-snapped dock geometry for `style` and `has_dock_chart`, clear of
/// everything `clearance` names.
pub(super) fn dock_layout(
    surface_size: [u32; 2],
    style: InstrumentDockStyle,
    has_dock_chart: bool,
    clearance: DockClearance,
) -> PanelDockLayout {
    match drawn_style(style, surface_size) {
        InstrumentDockStyle::BottomRow => bottom_row_layout(
            surface_size,
            has_dock_chart,
            clearance.for_bottom_row(GAP_X),
        ),
        InstrumentDockStyle::LeftRail => {
            left_rail_layout(surface_size, has_dock_chart, clearance.system)
        }
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
    clearance: DockClearance,
) -> Option<[u32; 2]> {
    Some(dock_layout(surface_size, style, has_dock_chart, clearance).panel_px)
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
mod tests;

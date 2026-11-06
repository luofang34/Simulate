//! Instrument dock geometry: `4:3` panels whose normalized rectangles keep their pixel
//! aspect on any window, so round gauges stay round.

use glam::DVec3;
use sim_core::VehicleState;
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_render::ViewportRect;

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

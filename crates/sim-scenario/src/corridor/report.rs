use std::collections::BTreeSet;

use glam::DVec3;
use sim_core::{CameraFrameCapture, MapCell};
use sim_geo::{LocalFrame, ecef_to_geodetic};

/// Whether the horizon crosses a capture's image: some corner ray above the
/// local horizontal and some below.
pub(super) fn horizon_in_view(capture: &CameraFrameCapture) -> bool {
    let intrinsics = capture.intrinsics;
    let up = LocalFrame::at(&ecef_to_geodetic(capture.pose.position_ecef)).up();
    let [right, bottom] = [intrinsics.width, intrinsics.height].map(f64::from);
    let elevation = |u: f64, v: f64| {
        let ray = DVec3::new(
            (u - intrinsics.cx) / intrinsics.fx,
            (intrinsics.cy - v) / intrinsics.fy,
            -1.0,
        );
        (capture.pose.orientation * ray).dot(up)
    };
    let corners = [
        elevation(0.0, 0.0),
        elevation(right, 0.0),
        elevation(0.0, bottom),
        elevation(right, bottom),
    ];
    corners.iter().any(|e| *e > 0.0) && corners.iter().any(|e| *e < 0.0)
}

/// What a corridor flight has shown so far.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CorridorReport {
    /// Camera exposures taken.
    pub frames: u64,
    /// Warm-track fixes delivered.
    pub warm_fixes: u64,
    /// Anchors delivered that are the first in their map cell.
    pub independent_anchor_fixes: u64,
    /// Anchors delivered in a cell that already has one.
    pub shared_map_cell_fixes: u64,
    /// Map cells with a delivered anchor.
    pub anchor_cells: BTreeSet<MapCell>,
    /// 600 m navigation tiles the camera has been over.
    pub nav_tiles_crossed: BTreeSet<MapCell>,
    /// Largest bank, radians.
    pub peak_bank_rad: f64,
    /// Largest body rate, rad/s.
    pub peak_body_rate_rad_s: f64,
    /// Exposures with no horizon in the image.
    pub horizon_loss_frames: u64,
    /// Latest distance right of the nominal track, metres.
    pub cross_track_m: f64,
    /// Latest height above the nominal cruise height, metres.
    pub height_error_m: f64,
}

//! Surface quad fills, solid/dashed perimeter rails, displaced-threshold zones, aiming-point
//! bars, and extended approach lead-in dashes for SVS / XVS conformal runways.

use maplibre::overlay::OverlayMesh;
use sim_geo::LocalFrame;
use sim_scenery::{LandingRunwayGeometry, RunwaySurfaceKind};

use super::{
    CHORD_DEPTH_CLEARANCE_M, StationProjection, StripAxis, VISUAL_LIFT_M, project_with_clearance,
    sample_station, sea_ecef,
};
use crate::{
    entities::{push_quad, push_segment},
    pose::ConformalFrameStamp,
};

const PAVED_SURFACE_FILL: [f32; 4] = [0.12, 0.16, 0.21, 0.56];
const DISPLACED_PAVED_FILL: [f32; 4] = [0.12, 0.16, 0.21, 0.30];
const TURF_SURFACE_FILL: [f32; 4] = [0.14, 0.30, 0.18, 0.42];
const RUNWAY_GREEN: [f32; 4] = [0.15, 0.95, 0.45, 0.92];
const THRESHOLD_GREEN: [f32; 4] = [0.22, 1.00, 0.54, 0.98];
const CENTRELINE_GREEN: [f32; 4] = [0.82, 0.98, 0.88, 0.86];
const DISPLACED_LEAD_GREEN: [f32; 4] = [0.15, 0.95, 0.45, 0.50];
const EXTENDED_LEAD_GREEN: [f32; 4] = [0.15, 0.95, 0.45, 0.62];

#[allow(clippy::too_many_arguments)]
pub(super) fn append_primary_zone(
    mesh: &mut OverlayMesh,
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    axis: &StripAxis,
    stations: &[StationProjection],
    primary_len_m: f64,
    surface: RunwaySurfaceKind,
    has_recip_displaced: bool,
    sample_dem_msl: &mut dyn FnMut(f64, f64) -> Option<(f64, f64)>,
) {
    let unpaved = surface.is_unpaved();
    let fill = if unpaved {
        TURF_SURFACE_FILL
    } else {
        PAVED_SURFACE_FILL
    };
    append_surface_fill(mesh, stations, fill);

    for (idx, pair) in stations.windows(2).enumerate() {
        if unpaved && idx % 2 != 0 {
            continue;
        }
        let [s0, s1] = pair else {
            continue;
        };
        let w = if unpaved { 2.0 } else { 2.2 };
        if let (Some(a), Some(b)) = (s0.left, s1.left) {
            push_segment(mesh, a.0, a.1, b.0, b.1, w, RUNWAY_GREEN);
        }
        if let (Some(a), Some(b)) = (s0.right, s1.right) {
            push_segment(mesh, a.0, a.1, b.0, b.1, w, RUNWAY_GREEN);
        }
    }

    if let Some(thr) = stations.first()
        && let (Some(l), Some(r)) = (thr.left, thr.right)
    {
        push_segment(mesh, l.0, l.1, r.0, r.1, 2.8, THRESHOLD_GREEN);
    }
    if let Some(far) = stations.last()
        && let (Some(l), Some(r)) = (far.left, far.right)
    {
        let far_w = if has_recip_displaced { 1.8 } else { 2.0 };
        push_segment(mesh, l.0, l.1, r.0, r.1, far_w, RUNWAY_GREEN);
    }

    if has_recip_displaced {
        append_inward_ticks(mesh, stamp, viewport_px, axis, 1.0, sample_dem_msl);
    }
    if !unpaved {
        append_interior_centreline(mesh, stations, primary_len_m);
        if primary_len_m >= 750.0 {
            append_aiming_point_bars(
                mesh,
                stamp,
                viewport_px,
                axis,
                primary_len_m,
                sample_dem_msl,
            );
        }
    }
}

fn append_surface_fill(mesh: &mut OverlayMesh, stations: &[StationProjection], color: [f32; 4]) {
    for pair in stations.windows(2) {
        let [s0, s1] = pair else {
            continue;
        };
        let (Some(al), Some(ar), Some(br), Some(bl)) = (s0.left, s0.right, s1.right, s1.left)
        else {
            continue;
        };
        push_quad(mesh, al, ar, br, bl, color);
    }
}

fn append_interior_centreline(
    mesh: &mut OverlayMesh,
    stations: &[StationProjection],
    primary_len_m: f64,
) {
    let seg_count = stations.len().saturating_sub(1);
    if seg_count < 3 || primary_len_m < 140.0 {
        return;
    }
    for (idx, pair) in stations.windows(2).enumerate() {
        if idx == 0 || idx.wrapping_add(1) >= seg_count || idx % 2 != 1 {
            continue;
        }
        let [s0, s1] = pair else {
            continue;
        };
        if let (Some(a), Some(b)) = (s0.center, s1.center) {
            push_segment(mesh, a.0, a.1, b.0, b.1, 1.5, CENTRELINE_GREEN);
        }
    }
}

fn append_aiming_point_bars(
    mesh: &mut OverlayMesh,
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    axis: &StripAxis,
    primary_len_m: f64,
    sample_dem_msl: &mut dyn FnMut(f64, f64) -> Option<(f64, f64)>,
) {
    let f0 = (285.0 / primary_len_m).clamp(0.05, 0.85);
    let f1 = (330.0 / primary_len_m).clamp(f0 + 0.02, 0.90);
    let aim_axis = StripAxis {
        half_width_m: (axis.half_width_m * 0.42).max(2.0),
        ..*axis
    };
    let s0 = sample_station(stamp, viewport_px, &aim_axis, f0, sample_dem_msl);
    let s1 = sample_station(stamp, viewport_px, &aim_axis, f1, sample_dem_msl);
    if let (Some(a), Some(b)) = (s0.left, s1.left) {
        push_segment(mesh, a.0, a.1, b.0, b.1, 2.4, CENTRELINE_GREEN);
    }
    if let (Some(a), Some(b)) = (s0.right, s1.right) {
        push_segment(mesh, a.0, a.1, b.0, b.1, 2.4, CENTRELINE_GREEN);
    }
}

fn append_inward_ticks(
    mesh: &mut OverlayMesh,
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    axis: &StripAxis,
    frac: f64,
    sample_dem_msl: &mut dyn FnMut(f64, f64) -> Option<(f64, f64)>,
) {
    let outer = sample_station(stamp, viewport_px, axis, frac, sample_dem_msl);
    let inner_axis = StripAxis {
        half_width_m: axis.half_width_m * 0.55,
        ..*axis
    };
    let inner = sample_station(stamp, viewport_px, &inner_axis, frac, sample_dem_msl);
    if let (Some(ol), Some(il)) = (outer.left, inner.left) {
        push_segment(mesh, ol.0, ol.1, il.0, il.1, 2.0, THRESHOLD_GREEN);
    }
    if let (Some(or_), Some(ir)) = (outer.right, inner.right) {
        push_segment(mesh, or_.0, or_.1, ir.0, ir.1, 2.0, THRESHOLD_GREEN);
    }
}

pub(super) fn append_displaced_zone(
    mesh: &mut OverlayMesh,
    stations: &[StationProjection],
    turf_tint: bool,
    pre_threshold: bool,
) {
    let fill = if turf_tint {
        TURF_SURFACE_FILL
    } else {
        DISPLACED_PAVED_FILL
    };
    append_surface_fill(mesh, stations, fill);

    for (idx, pair) in stations.windows(2).enumerate() {
        if idx % 2 != 0 {
            continue;
        }
        let [s0, s1] = pair else {
            continue;
        };
        if let (Some(a), Some(b)) = (s0.left, s1.left) {
            push_segment(mesh, a.0, a.1, b.0, b.1, 1.6, DISPLACED_LEAD_GREEN);
        }
        if let (Some(a), Some(b)) = (s0.right, s1.right) {
            push_segment(mesh, a.0, a.1, b.0, b.1, 1.6, DISPLACED_LEAD_GREEN);
        }
        if pre_threshold && let (Some(a), Some(b)) = (s0.center, s1.center) {
            push_segment(mesh, a.0, a.1, b.0, b.1, 1.5, DISPLACED_LEAD_GREEN);
        }
    }

    let cap = if pre_threshold {
        stations.first()
    } else {
        stations.last()
    };
    if let Some(end) = cap
        && let (Some(l), Some(r)) = (end.left, end.right)
    {
        push_segment(mesh, l.0, l.1, r.0, r.1, 1.5, DISPLACED_LEAD_GREEN);
    }
}

pub(super) fn append_extended_lead_in(
    mesh: &mut OverlayMesh,
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    rwy: &LandingRunwayGeometry,
) {
    let body = stamp.physics_ellipsoid;
    let origin_sea = sea_ecef(body, &rwy.pavement_start);
    let far_sea = sea_ecef(body, &rwy.far_end);
    let origin_zero = body.fixed_to_geodetic(origin_sea);
    let local = LocalFrame::at(&origin_zero);
    let up = local.up();
    let chord = far_sea - origin_sea;
    let fwd = (chord - up * chord.dot(up))
        .try_normalize()
        .unwrap_or_else(|| {
            let (sin_b, cos_b) = rwy.true_bearing_rad.sin_cos();
            local.north() * cos_b + local.east() * sin_b
        });
    let vis_h = rwy.threshold.height_m + VISUAL_LIFT_M;
    let clear_h = rwy.threshold.height_m + CHORD_DEPTH_CLEARANCE_M;

    for k in 0..6_u32 {
        let s0 = 90.0 + f64::from(k) * 180.0;
        let s1 = s0 + 75.0;
        let g0 = body.fixed_to_geodetic(origin_sea - fwd * s0);
        let g1 = body.fixed_to_geodetic(origin_sea - fwd * s1);
        if let (Some(p0), Some(p1)) = (
            project_with_clearance(stamp, viewport_px, &g0, vis_h, clear_h),
            project_with_clearance(stamp, viewport_px, &g1, vis_h, clear_h),
        ) {
            push_segment(mesh, p0.0, p0.1, p1.0, p1.1, 1.5, EXTENDED_LEAD_GREEN);
        }
    }
}

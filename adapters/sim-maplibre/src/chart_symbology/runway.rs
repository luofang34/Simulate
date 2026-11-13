//! Runway pavement strip, displaced landing threshold bar, and 5 NM extended approach
//! centreline feather projected through [`ChartProjection`] across all latitudes
//! including `±90°`.

use maplibre::overlay::OverlayMesh;
use sim_scenery::{LandingRunwayGeometry, RunwaySurfaceKind};

use super::{ChartProjection, FEATHER_CYAN, FEATHER_WHITE, HALO, push_segment};
use crate::geodesy::{METRES_PER_NM, destination, initial_bearing_deg};

const RUNWAY_PAVEMENT: [f32; 4] = [0.80, 0.85, 0.92, 0.96];
const DISPLACED_OVERRUN: [f32; 4] = [0.44, 0.52, 0.62, 0.88];
const RUNWAY_TURF: [f32; 4] = [0.28, 0.58, 0.38, 0.92];
const THRESHOLD_AMBER: [f32; 4] = [0.98, 0.80, 0.08, 0.96];

pub(super) fn append_approach_feather(
    mesh: &mut OverlayMesh,
    projection: &ChartProjection<'_>,
    runway: &LandingRunwayGeometry,
) {
    let [thr_lon, thr_lat] = [runway.threshold.lon_deg(), runway.threshold.lat_deg()];
    let [far_lon, far_lat] = [runway.far_end.lon_deg(), runway.far_end.lat_deg()];
    let thr = [thr_lon, thr_lat];
    let course = if (thr_lon - far_lon).abs() > 1e-7 || (thr_lat - far_lat).abs() > 1e-7 {
        initial_bearing_deg(thr, [far_lon, far_lat])
    } else {
        runway.true_bearing_rad.to_degrees().rem_euclid(360.0)
    };
    let outbound = (course + 180.0).rem_euclid(360.0);
    append_runway_strip(mesh, projection, runway, thr, course);
    let s = projection.scale;
    let dash_m = 0.25 * METRES_PER_NM;
    for i in (0..20_u32).step_by(2) {
        let [lon_a, lat_a] = destination(thr, outbound, f64::from(i) * dash_m);
        let [lon_b, lat_b] = destination(thr, outbound, f64::from(i.wrapping_add(1)) * dash_m);
        let (Some(pa), Some(pb)) = (
            projection.project(lat_a, lon_a),
            projection.project(lat_b, lon_b),
        ) else {
            continue;
        };
        let color = if (i / 2) % 2 == 0 {
            FEATHER_CYAN
        } else {
            FEATHER_WHITE
        };
        push_segment(mesh, pa, pb, 2.2 * s, color);
    }
    for nm in 1_u32..=5 {
        append_feather_tick(mesh, projection, thr, outbound, nm);
    }
}

fn append_runway_strip(
    mesh: &mut OverlayMesh,
    projection: &ChartProjection<'_>,
    runway: &LandingRunwayGeometry,
    thr: [f64; 2],
    course: f64,
) {
    let [thr_lon, thr_lat] = thr;
    let (Some(p_start), Some(p_thr), Some(p_far)) = (
        projection.project(
            runway.pavement_start.lat_deg(),
            runway.pavement_start.lon_deg(),
        ),
        projection.project(thr_lat, thr_lon),
        projection.project(runway.far_end.lat_deg(), runway.far_end.lon_deg()),
    ) else {
        return;
    };
    let half_w = runway.width_m.max(12.0) * 0.5;
    let [llon, llat] = destination(thr, course - 90.0, half_w);
    let [rlon, rlat] = destination(thr, course + 90.0, half_w);
    let ground_px = match (
        projection.project(llat, llon),
        projection.project(rlat, rlon),
    ) {
        (Some([lx, ly]), Some([rx, ry])) => (rx - lx).hypot(ry - ly),
        _ => 0.0,
    };
    let s = projection.scale;
    let pave_px = ground_px.max(4.4 * s);
    let [ax, ay] = p_thr;
    let [bx, by] = p_far;
    push_segment(mesh, p_start, p_far, pave_px + 2.4 * s, HALO);
    if runway.surface.is_unpaved() {
        push_segment(mesh, p_start, p_far, pave_px, RUNWAY_TURF);
    } else if runway.displaced_threshold_m > 1.0 || runway.reciprocal_displaced_m > 1.0 {
        let end_tint = if runway.surface == RunwaySurfaceKind::PavedTurf {
            RUNWAY_TURF
        } else {
            DISPLACED_OVERRUN
        };
        push_segment(mesh, p_start, p_far, pave_px, end_tint);
        let recip_frac = ((runway.landing_length_m - runway.reciprocal_displaced_m.max(0.0))
            / runway.landing_length_m.max(1.0))
        .clamp(0.05, 1.0) as f32;
        let p_recip = [ax + (bx - ax) * recip_frac, ay + (by - ay) * recip_frac];
        push_segment(mesh, p_thr, p_recip, pave_px, RUNWAY_PAVEMENT);
    } else {
        push_segment(mesh, p_start, p_far, pave_px, RUNWAY_PAVEMENT);
    }

    let (tx, ty) = (bx - ax, by - ay);
    let len = tx.hypot(ty);
    if len > 1e-3 {
        let half_bar = (pave_px * 0.85).max(4.5 * s);
        let (nx, ny) = (-ty / len * half_bar, tx / len * half_bar);
        push_segment(
            mesh,
            [ax - nx, ay - ny],
            [ax + nx, ay + ny],
            2.4 * s,
            THRESHOLD_AMBER,
        );
    }
}

fn append_feather_tick(
    mesh: &mut OverlayMesh,
    projection: &ChartProjection<'_>,
    thr: [f64; 2],
    outbound: f64,
    nm: u32,
) {
    let dist_m = f64::from(nm) * METRES_PER_NM;
    let [lon_c, lat_c] = destination(thr, outbound, dist_m);
    let [lon_prev, lat_prev] = destination(thr, outbound, dist_m - 50.0);
    let [lon_next, lat_next] = destination(thr, outbound, dist_m + 50.0);
    let (Some([cx, cy]), Some([px, py]), Some([nx, ny])) = (
        projection.project(lat_c, lon_c),
        projection.project(lat_prev, lon_prev),
        projection.project(lat_next, lon_next),
    ) else {
        return;
    };
    let (tx, ty) = (nx - px, ny - py);
    let len = tx.hypot(ty);
    if len <= 1e-4 {
        return;
    }
    let s = projection.scale;
    let half = (if nm == 5 { 9.0 } else { 5.0 }) * s;
    let (dx, dy) = (-ty / len * half, tx / len * half);
    push_segment(
        mesh,
        [cx - dx, cy - dy],
        [cx + dx, cy + dy],
        1.6 * s,
        FEATHER_WHITE,
    );
}

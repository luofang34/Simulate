//! Ownship on the chart: its ground-track predictor, heading lubber line, and a haloed
//! top-down silhouette.

use maplibre::overlay::OverlayMesh;

use super::{
    ChartOwnship, ChartProjection, HALO, LUBBER_CYAN, OWNSHIP_WHITE, TRACK_VECTOR, WINGTIP_CYAN,
    push_segment, push_triangle, rotate,
};
use crate::geodesy::destination;

pub(super) fn append_ownship(
    mesh: &mut OverlayMesh,
    projection: &ChartProjection<'_>,
    own: &ChartOwnship,
    px: [f32; 2],
) {
    let s = projection.scale;
    let [east, north] = own.ground_velocity_en_mps;
    let speed = east.hypot(north);
    if speed > 2.0 {
        let track = east.atan2(north).to_degrees();
        let [lon2, lat2] = destination([own.lon_deg, own.lat_deg], track, speed * 60.0);
        if let Some(tip) = projection.project(lat2, lon2) {
            push_segment(mesh, px, tip, 3.0 * s, HALO);
            push_segment(mesh, px, tip, 2.0 * s, TRACK_VECTOR);
        }
    }
    let angle = projection.screen_angle_at(own.lat_deg, own.lon_deg, own.true_heading_deg);
    let nose = rotate([0.0, -18.0 * s], angle, px);
    let lubber_end = rotate([0.0, -62.0 * s], angle, px);
    push_segment(mesh, nose, lubber_end, 1.8 * s, LUBBER_CYAN);
    for (polygon, _) in silhouette(s) {
        push_convex(mesh, &inflate(&polygon, 2.5 * s), angle, px, HALO);
    }
    for (polygon, color) in silhouette(s) {
        push_convex(mesh, &polygon, angle, px, color);
    }
}

/// Top-down high-wing trainer, nose up, pixels about its centre of gravity.
fn silhouette(s: f32) -> [(Vec<[f32; 2]>, [f32; 4]); 6] {
    let sp = |pts: &[[f32; 2]]| pts.iter().map(|&[x, y]| [x * s, y * s]).collect();
    [
        (
            sp(&[
                [0.0, -15.0],
                [2.3, -11.5],
                [2.2, 7.0],
                [1.0, 13.5],
                [-1.0, 13.5],
                [-2.2, 7.0],
                [-2.3, -11.5],
            ]),
            OWNSHIP_WHITE,
        ),
        (
            sp(&[[-15.0, -4.6], [15.0, -4.6], [15.0, -1.0], [-15.0, -1.0]]),
            OWNSHIP_WHITE,
        ),
        (
            sp(&[[2.0, -5.4], [12.0, -4.6], [12.0, -0.8], [2.0, 0.2]]),
            OWNSHIP_WHITE,
        ),
        (
            sp(&[[-6.5, 9.5], [6.5, 9.5], [6.0, 12.4], [-6.0, 12.4]]),
            OWNSHIP_WHITE,
        ),
        (
            sp(&[[12.8, -4.6], [15.0, -4.6], [15.0, -1.0], [12.8, -1.0]]),
            WINGTIP_CYAN,
        ),
        (
            sp(&[[-15.0, -4.6], [-12.8, -4.6], [-12.8, -1.0], [-15.0, -1.0]]),
            WINGTIP_CYAN,
        ),
    ]
}

/// Grows a convex polygon outward by about `by` pixels.
fn inflate(polygon: &[[f32; 2]], by: f32) -> Vec<[f32; 2]> {
    let n = polygon.len().max(1) as f32;
    let (sx, sy) = polygon
        .iter()
        .fold((0.0, 0.0), |(x, y), [px, py]| (x + px, y + py));
    let (cx, cy) = (sx / n, sy / n);
    polygon
        .iter()
        .map(|&[x, y]| {
            let (dx, dy) = (x - cx, y - cy);
            let len = dx.hypot(dy).max(1e-3);
            [x + dx / len * by, y + dy / len * by]
        })
        .collect()
}

fn push_convex(
    mesh: &mut OverlayMesh,
    polygon: &[[f32; 2]],
    angle: f32,
    origin: [f32; 2],
    color: [f32; 4],
) {
    let Some((&first, rest)) = polygon.split_first() else {
        return;
    };
    let a = rotate(first, angle, origin);
    for pair in rest.windows(2) {
        if let [b, c] = pair {
            push_triangle(
                mesh,
                [a, rotate(*b, angle, origin), rotate(*c, angle, origin)],
                color,
            );
        }
    }
}

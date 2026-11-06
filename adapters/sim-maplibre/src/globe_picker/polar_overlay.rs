//! Screen-space [`OverlayMesh`] rendering for polar (`|lat| > 85°`) airports, runways,
//! and converging polar meridians that lie poleward of Web Mercator's `geojson-vt` bounds.

use maplibre::overlay::OverlayMesh;

use super::{
    GlobeAirportMarker, GlobePickerSurface, GlobeRunwayEndOverlay,
    features::POLAR_GEOJSON_LIMIT_DEG,
};
use crate::{
    chart_symbology::{push_segment, push_triangle},
    geodesy::{METRES_PER_NM, destination, initial_bearing_deg},
};

const FEATHER_NM: f64 = 5.0;
const THRESHOLD_BAR_WIDTHS: f64 = 1.8;
const CASING_NAVY: [f32; 4] = [0.04, 0.09, 0.17, 0.90];
const PAVEMENT: [f32; 4] = [0.89, 0.91, 0.94, 0.98];
const FEATHER_CYAN: [f32; 4] = [0.06, 0.76, 0.91, 0.98];
const AMBER: [f32; 4] = [0.98, 0.80, 0.08, 1.0];
const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
const TOWERED_BLUE: [f32; 4] = [0.22, 0.74, 0.97, 0.96];
const UNTOWERED_VIOLET: [f32; 4] = [0.75, 0.52, 0.99, 0.96];
const POLAR_GRID: [f32; 4] = [0.22, 0.40, 0.60, 0.34];

pub(super) fn build_polar_overlay(
    surface: &GlobePickerSurface,
    markers: &[GlobeAirportMarker],
    runways: &[GlobeRunwayEndOverlay],
) -> OverlayMesh {
    let mut mesh = OverlayMesh::default();
    let [w, h] = surface.size_px();
    let bounds = [w as f32, h as f32];
    for pole_sign in [-1.0_f64, 1.0_f64] {
        if let Some((pole_px, false)) = surface.project_lat_lon(pole_sign * 90.0, 0.0)
            && inside(pole_px, bounds, 240.0)
        {
            append_polar_graticule(&mut mesh, surface, pole_sign, pole_px);
        }
    }
    for runway in runways {
        append_polar_runway(&mut mesh, surface, runway);
    }
    let mut ordered: Vec<&GlobeAirportMarker> = markers.iter().collect();
    ordered.sort_by_key(|m| m.selected);
    for marker in ordered {
        if let Some((px, false)) = surface.project_lat_lon(marker.lat_deg, marker.lon_deg)
            && inside(px, bounds, 32.0)
        {
            append_polar_marker(&mut mesh, px, marker);
        }
    }
    mesh
}

fn append_polar_graticule(
    mesh: &mut OverlayMesh,
    surface: &GlobePickerSurface,
    pole_sign: f64,
    pole_px: [f32; 2],
) {
    for lon_step in (0_u32..360).step_by(30) {
        let lon = f64::from(lon_step);
        for lat_abs in 85_u32..90 {
            let lat_a = pole_sign * f64::from(lat_abs);
            let lat_b = pole_sign * f64::from(lat_abs.wrapping_add(1));
            if let (Some((pa, false)), Some((pb, false))) = (
                surface.project_lat_lon(lat_a, lon),
                surface.project_lat_lon(lat_b, lon),
            ) {
                push_segment(mesh, pa, pb, 1.1, POLAR_GRID);
            }
        }
    }
    for ring_lat in [89.5_f64, 89.95_f64] {
        let lat = pole_sign * ring_lat;
        for step in (0_u32..360).step_by(10) {
            let lon_a = f64::from(step);
            let lon_b = f64::from(step.wrapping_add(10));
            if let (Some((pa, false)), Some((pb, false))) = (
                surface.project_lat_lon(lat, lon_a),
                surface.project_lat_lon(lat, lon_b),
            ) {
                push_segment(mesh, pa, pb, 1.0, POLAR_GRID);
            }
        }
    }
    let [px, py] = pole_px;
    push_segment(mesh, [px - 5.0, py], [px + 5.0, py], 1.4, POLAR_GRID);
    push_segment(mesh, [px, py - 5.0], [px, py + 5.0], 1.4, POLAR_GRID);
}

fn append_polar_runway(
    mesh: &mut OverlayMesh,
    surface: &GlobePickerSurface,
    runway: &GlobeRunwayEndOverlay,
) {
    let thr = [runway.threshold_lon_deg, runway.threshold_lat_deg];
    let opp = [runway.opposite_lon_deg, runway.opposite_lat_deg];
    let course = initial_bearing_deg(thr, opp);
    if runway.selected {
        let step_m = FEATHER_NM * METRES_PER_NM / 8.0;
        for i in 0_u32..8 {
            let [la, phia] = destination(thr, course + 180.0, f64::from(i) * step_m);
            let [lb, phib] =
                destination(thr, course + 180.0, f64::from(i.wrapping_add(1)) * step_m);
            if let (Some((pa, false)), Some((pb, false))) = (
                surface.project_lat_lon(phia, la),
                surface.project_lat_lon(phib, lb),
            ) {
                push_segment(mesh, pa, pb, 4.4, CASING_NAVY);
                push_segment(mesh, pa, pb, 2.4, FEATHER_CYAN);
            }
        }
    }
    let (Some((pa, false)), Some((pb, false))) = (
        surface.project_lat_lon(runway.threshold_lat_deg, runway.threshold_lon_deg),
        surface.project_lat_lon(runway.opposite_lat_deg, runway.opposite_lon_deg),
    ) else {
        return;
    };
    push_segment(mesh, pa, pb, 6.4, CASING_NAVY);
    push_segment(mesh, pa, pb, 4.2, PAVEMENT);
    if runway.selected {
        let half_bar = f64::from(runway.width_m.max(10.0)) * THRESHOLD_BAR_WIDTHS * 0.5;
        let [bl_lon, bl_lat] = destination(thr, course - 90.0, half_bar);
        let [br_lon, br_lat] = destination(thr, course + 90.0, half_bar);
        let bar_pts = match (
            surface.project_lat_lon(bl_lat, bl_lon),
            surface.project_lat_lon(br_lat, br_lon),
        ) {
            (Some((pl, false)), Some((pr, false))) => {
                let [lx, ly] = pl;
                let [rx, ry] = pr;
                if (rx - lx).hypot(ry - ly) >= 10.0 {
                    Some((pl, pr))
                } else {
                    screen_perp_bar(pa, pb, 5.5)
                }
            }
            _ => screen_perp_bar(pa, pb, 5.5),
        };
        if let Some((pl, pr)) = bar_pts {
            push_segment(mesh, pl, pr, 5.2, CASING_NAVY);
            push_segment(mesh, pl, pr, 3.2, AMBER);
        }
    }
}

fn screen_perp_bar(pa: [f32; 2], pb: [f32; 2], half_px: f32) -> Option<([f32; 2], [f32; 2])> {
    let [ax, ay] = pa;
    let [bx, by] = pb;
    let (tx, ty) = (bx - ax, by - ay);
    let len = tx.hypot(ty);
    (len > 1e-3).then(|| {
        let (nx, ny) = (-ty / len * half_px, tx / len * half_px);
        ([ax - nx, ay - ny], [ax + nx, ay + ny])
    })
}

fn append_polar_marker(mesh: &mut OverlayMesh, center: [f32; 2], marker: &GlobeAirportMarker) {
    if marker.selected {
        let [r, g, b, _] = AMBER;
        push_disc(mesh, center, 13.0, [r, g, b, 0.24]);
        push_ring(mesh, center, 13.0, 1.3, [r, g, b, 0.72]);
        push_disc(mesh, center, 9.6, CASING_NAVY);
        push_disc(mesh, center, 8.2, WHITE);
        push_disc(mesh, center, 6.2, AMBER);
    } else {
        let (fill, radius) = if marker.towered {
            (TOWERED_BLUE, 4.4)
        } else {
            (UNTOWERED_VIOLET, 3.6)
        };
        push_disc(mesh, center, radius + 1.2, CASING_NAVY);
        push_disc(mesh, center, radius, fill);
    }
}

fn push_disc(mesh: &mut OverlayMesh, [cx, cy]: [f32; 2], radius: f32, color: [f32; 4]) {
    const STEPS: u32 = 24;
    for i in 0..STEPS {
        let a0 = i as f32 / STEPS as f32 * std::f32::consts::TAU;
        let a1 = i.wrapping_add(1) as f32 / STEPS as f32 * std::f32::consts::TAU;
        push_triangle(
            mesh,
            [
                [cx, cy],
                [cx + radius * a0.cos(), cy + radius * a0.sin()],
                [cx + radius * a1.cos(), cy + radius * a1.sin()],
            ],
            color,
        );
    }
}

fn push_ring(
    mesh: &mut OverlayMesh,
    [cx, cy]: [f32; 2],
    radius: f32,
    width_px: f32,
    color: [f32; 4],
) {
    const STEPS: u32 = 28;
    for i in 0..STEPS {
        let a0 = i as f32 / STEPS as f32 * std::f32::consts::TAU;
        let a1 = i.wrapping_add(1) as f32 / STEPS as f32 * std::f32::consts::TAU;
        push_segment(
            mesh,
            [cx + radius * a0.cos(), cy + radius * a0.sin()],
            [cx + radius * a1.cos(), cy + radius * a1.sin()],
            width_px,
            color,
        );
    }
}

fn inside([x, y]: [f32; 2], [w, h]: [f32; 2], margin: f32) -> bool {
    x >= -margin && y >= -margin && x <= w + margin && y <= h + margin
}

pub(super) fn is_polar_marker(marker: &GlobeAirportMarker) -> bool {
    marker.lat_deg.is_finite()
        && marker.lon_deg.is_finite()
        && marker.lat_deg.abs() > POLAR_GEOJSON_LIMIT_DEG
}

pub(super) fn is_polar_runway(runway: &GlobeRunwayEndOverlay) -> bool {
    [
        runway.threshold_lat_deg,
        runway.threshold_lon_deg,
        runway.opposite_lat_deg,
        runway.opposite_lon_deg,
    ]
    .iter()
    .all(|v| v.is_finite())
        && (runway.threshold_lat_deg.abs() > POLAR_GEOJSON_LIMIT_DEG
            || runway.opposite_lat_deg.abs() > POLAR_GEOJSON_LIMIT_DEG)
}

//! Pixel-sized marks a host lays over a chart besides its own symbology, and where points
//! fall on the chart's screen, for labels and picking drawn by the host.

use maplibre::overlay::OverlayMesh;

use super::{ChartProjection, push_segment};
use crate::{ChartCameraSpec, SigmaRing, efb_map::MIN_RING_PX, geodesy::destination};

/// Segments of a drawn ring.
const RING_STEPS: u32 = 64;

/// Adds a ring of each one-sigma in `rings` to `mesh`, at least [`MIN_RING_PX`] in
/// radius, registered to `camera` over a `viewport_px` drawn at `pixel_ratio`.
pub(crate) fn append_sigma_rings(
    mesh: &mut OverlayMesh,
    (camera, viewport_px, pixel_ratio): (&ChartCameraSpec, [u32; 2], f64),
    rings: &[SigmaRing],
) {
    let Some(projection) = ChartProjection::new(camera, viewport_px, pixel_ratio) else {
        return;
    };
    let s = projection.scale;
    for ring in rings {
        let [lat, lon] = ring.centre_deg;
        let Some(centre) = projection.project(lat, lon) else {
            continue;
        };
        let [edge_lon, edge_lat] = destination([lon, lat], 90.0, ring.sigma_m.max(0.0));
        let radius = projection
            .project(edge_lat, edge_lon)
            .map_or(0.0, |[x, y]| (x - centre[0]).hypot(y - centre[1]))
            .max(MIN_RING_PX * s);
        if !projection.contains(centre, radius / s) {
            continue;
        }
        let [r, g, b] = ring.colour.map(|v| f32::from(v) / 255.0);
        let colour = [r, g, b, 0.95];
        let [cx, cy] = centre;
        let point = |step: u32| {
            let angle = step as f32 / RING_STEPS as f32 * std::f32::consts::TAU;
            [cx + radius * angle.cos(), cy + radius * angle.sin()]
        };
        for step in 0..RING_STEPS {
            push_segment(
                mesh,
                point(step),
                point(step.wrapping_add(1)),
                1.5 * s,
                colour,
            );
        }
    }
}

/// Where each `[lat, lon]` of `points` falls on a `viewport_px` chart drawn by `camera`
/// at `pixel_ratio`, physical pixels from its top left; `None` behind the globe.
pub(crate) fn screen_positions(
    (camera, viewport_px, pixel_ratio): (&ChartCameraSpec, [u32; 2], f64),
    points: &[[f64; 2]],
) -> Vec<Option<[f32; 2]>> {
    let Some(projection) = ChartProjection::new(camera, viewport_px, pixel_ratio) else {
        return vec![None; points.len()];
    };
    points
        .iter()
        .map(|&[lat, lon]| projection.project(lat, lon))
        .collect()
}

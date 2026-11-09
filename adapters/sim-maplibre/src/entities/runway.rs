//! 3D geodesic, surface-aware, displaced-threshold, terrain-conformed conformal runway
//! [`OverlayMesh`] builder for Synthetic / Combined Vision (`SVS` / `XVS`).
//!
//! Portrayal follows certified avionics SVS/CVS conventions (Honeywell SmartView / Dassault
//! EASy + FalconEye, Collins Pro Line Fusion, Garmin SVT, and RTCA DO-315B):
//! - Hard-surfaced runways render with a translucent dark-asphalt fill, solid perimeter rails,
//!   bold landing threshold bar, painted centreline dashes, and `1,000 ft` (`305 m`) aiming
//!   bars (`>= 750 m` landing length).
//! - Unpaved strips (`Turf`, `Gravel`, `Dirt`) render with a muted turf tint, dashed perimeter
//!   rails, and threshold/end bars without painted centreline or aiming-point markings.
//! - Pre-threshold and reciprocal displaced zones render with dashed sidelines (and turf tint
//!   on `PavedTurf` composite strips) so the solid landing box stays strictly between the
//!   surveyed landing thresholds.
//! - Every station and sideline offset is evaluated directly on `stamp.render_body`'s sphere
//!   using datum/MSL elevations (`point_geoid_undulation_m = 0.0`) and clipped in 3D camera
//!   eye space against the near clip plane and frustum guardband before screen projection, so
//!   surface quads, sidelines, and centreline stripes never truncate mid-viewport during flare,
//!   rollout, or low overflight.

mod clip;
mod markings;
#[cfg(test)]
mod tests;

use clip::{ClipVertex, project_in_viewport, vertex_with_clearance};
use glam::DVec3;
use maplibre::{
    overlay::{DepthProbe, MAX_PROBES, OverlayMesh},
    projection::body::Body,
};
use sim_geo::Geodetic;
use sim_scenery::{LandingRunwayGeometry, RunwaySurfaceKind};

use crate::pose::{
    ConformalFrameStamp, geodetic_zero_from_globe_vec, sphere_enu_basis, unit_sphere_from_geodetic,
};

const TARGET_STATION_STEP_M: f64 = 40.0;
const MIN_PRIMARY_STEPS: usize = 20;
const MAX_PRIMARY_STEPS: usize = 128;
const VISUAL_LIFT_M: f64 = 0.35;
const DEM_DEPTH_CLEARANCE_M: f64 = 0.90;
const CHORD_DEPTH_CLEARANCE_M: f64 = 0.65;
const DEM_REJECT_BAND_M: f64 = 12.0;

#[derive(Clone, Copy)]
struct StationProjection {
    left: Option<ClipVertex>,
    right: Option<ClipVertex>,
    center: Option<ClipVertex>,
}

struct StripAxis {
    start_globe_m: DVec3,
    end_globe_m: DVec3,
    start_msl_m: f64,
    end_msl_m: f64,
    half_width_m: f64,
    fallback_bearing_rad: f64,
    valid_msl_range: (f64, f64),
}

fn strip_steps(length_m: f64, min_steps: usize, max_steps: usize) -> usize {
    if !length_m.is_finite() || length_m <= 0.0 {
        return min_steps;
    }
    ((length_m / TARGET_STATION_STEP_M).ceil() as usize).clamp(min_steps, max_steps)
}

pub(super) fn append_runway_outline(
    mesh: &mut OverlayMesh,
    probes: &mut Vec<DepthProbe>,
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    rwy: &LandingRunwayGeometry,
    sample_dem_msl: &mut dyn FnMut(f64, f64) -> Option<(f64, f64)>,
) {
    markings::append_extended_lead_in(mesh, stamp, viewport_px, rwy);
    if rwy.displaced_threshold_m > 10.0 {
        append_zone_segment(
            mesh,
            stamp,
            viewport_px,
            rwy,
            (&rwy.pavement_start, rwy.start_elevation_msl_m),
            (&rwy.threshold, rwy.threshold_elevation_msl_m),
            rwy.displaced_threshold_m,
            true,
            sample_dem_msl,
        );
    }

    let (primary_axis, primary_len_m, has_recip, recip_zero, recip_msl_m) =
        primary_strip_axis(stamp.render_body, rwy);
    let steps = strip_steps(primary_len_m, MIN_PRIMARY_STEPS, MAX_PRIMARY_STEPS);
    let primary_stations = sample_axis_stations(stamp, &primary_axis, steps, sample_dem_msl);
    markings::append_primary_zone(
        mesh,
        stamp,
        viewport_px,
        &primary_axis,
        &primary_stations,
        primary_len_m,
        rwy.surface,
        has_recip,
        sample_dem_msl,
    );
    if has_recip {
        append_zone_segment(
            mesh,
            stamp,
            viewport_px,
            rwy,
            (&recip_zero, recip_msl_m),
            (&rwy.far_end, rwy.far_elevation_msl_m),
            rwy.reciprocal_displaced_m,
            false,
            sample_dem_msl,
        );
    }
    if probes.len() < MAX_PROBES
        && let Some((px, depth)) = primary_stations.iter().find_map(|s| {
            s.center
                .as_ref()
                .and_then(|c| project_in_viewport(stamp, viewport_px, c))
        })
    {
        probes.push(DepthProbe {
            position: px,
            depth,
        });
    }
}

fn primary_strip_axis(
    render_body: Body,
    rwy: &LandingRunwayGeometry,
) -> (StripAxis, f64, bool, Geodetic, f64) {
    let half_w = (rwy.width_m * 0.5).max(4.0);
    let total_landing_m = rwy.landing_length_m.max(1.0);
    let primary_len_m = (total_landing_m - rwy.reciprocal_displaced_m.max(0.0))
        .clamp(60.0_f64.min(total_landing_m), total_landing_m);
    let recip_frac = (primary_len_m / total_landing_m).clamp(0.05, 1.0);
    let thr_globe = sea_globe(render_body, &rwy.threshold);
    let far_globe = sea_globe(render_body, &rwy.far_end);
    let recip_globe = (thr_globe + (far_globe - thr_globe) * recip_frac)
        .try_normalize()
        .unwrap_or(DVec3::Y)
        * render_body.radius_meters;
    let recip_msl_m = rwy.threshold_elevation_msl_m
        + (rwy.far_elevation_msl_m - rwy.threshold_elevation_msl_m) * recip_frac;
    let recip_zero = geodetic_zero_from_globe_vec(recip_globe);
    let has_recip = rwy.reciprocal_displaced_m > 10.0 && recip_frac < 0.995;
    let axis = StripAxis {
        start_globe_m: thr_globe,
        end_globe_m: recip_globe,
        start_msl_m: rwy.threshold_elevation_msl_m,
        end_msl_m: recip_msl_m,
        half_width_m: half_w,
        fallback_bearing_rad: rwy.true_bearing_rad,
        valid_msl_range: rwy.valid_surface_msl_range(),
    };
    (axis, primary_len_m, has_recip, recip_zero, recip_msl_m)
}

#[allow(clippy::too_many_arguments)]
fn append_zone_segment(
    mesh: &mut OverlayMesh,
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    rwy: &LandingRunwayGeometry,
    (start, start_msl_m): (&Geodetic, f64),
    (end, end_msl_m): (&Geodetic, f64),
    length_m: f64,
    pre_threshold: bool,
    sample_dem_msl: &mut dyn FnMut(f64, f64) -> Option<(f64, f64)>,
) {
    let displaced_turf = rwy.surface.is_unpaved() || rwy.surface == RunwaySurfaceKind::PavedTurf;
    let axis = StripAxis {
        start_globe_m: sea_globe(stamp.render_body, start),
        end_globe_m: sea_globe(stamp.render_body, end),
        start_msl_m,
        end_msl_m,
        half_width_m: (rwy.width_m * 0.5).max(4.0),
        fallback_bearing_rad: rwy.true_bearing_rad,
        valid_msl_range: rwy.valid_surface_msl_range(),
    };
    let steps = strip_steps(length_m, 2, 36);
    let stations = sample_axis_stations(stamp, &axis, steps, sample_dem_msl);
    markings::append_displaced_zone(
        mesh,
        stamp,
        viewport_px,
        &stations,
        displaced_turf,
        pre_threshold,
    );
}

fn sample_axis_stations(
    stamp: &ConformalFrameStamp,
    axis: &StripAxis,
    steps: usize,
    sample_dem_msl: &mut dyn FnMut(f64, f64) -> Option<(f64, f64)>,
) -> Vec<StationProjection> {
    let denom = steps.max(1) as f64;
    let mut out = Vec::with_capacity(steps.wrapping_add(1));
    for i in 0..=steps {
        let frac = i as f64 / denom;
        out.push(sample_station(stamp, axis, frac, sample_dem_msl));
    }
    out
}

fn sample_station(
    stamp: &ConformalFrameStamp,
    axis: &StripAxis,
    frac: f64,
    sample_dem_msl: &mut dyn FnMut(f64, f64) -> Option<(f64, f64)>,
) -> StationProjection {
    let chord_vec = axis.end_globe_m - axis.start_globe_m;
    let center_zero = geodetic_zero_from_globe_vec(axis.start_globe_m + chord_vec * frac);
    let [east, north, up] = sphere_enu_basis(center_zero.lat_deg(), center_zero.lon_deg());
    let center_surf = up * stamp.render_body.radius_meters;
    let tangent_fwd = (chord_vec - up * chord_vec.dot(up))
        .try_normalize()
        .unwrap_or_else(|| {
            let (sin_b, cos_b) = axis.fallback_bearing_rad.sin_cos();
            north * cos_b + east * sin_b
        });
    let right_globe = tangent_fwd.cross(up).try_normalize().unwrap_or(east);

    let left_zero = geodetic_zero_from_globe_vec(center_surf - right_globe * axis.half_width_m);
    let right_zero = geodetic_zero_from_globe_vec(center_surf + right_globe * axis.half_width_m);

    let (lo, hi) = axis.valid_msl_range;
    let dem_c = sample_dem_msl(center_zero.lat_deg(), center_zero.lon_deg());
    let dem_l = sample_dem_msl(left_zero.lat_deg(), left_zero.lon_deg());
    let dem_r = sample_dem_msl(right_zero.lat_deg(), right_zero.lon_deg());

    let (vis_msl, depth_c_msl, depth_l_msl, depth_r_msl) = match dem_c {
        Some((bil_c, max_c))
            if bil_c.is_finite()
                && bil_c >= lo - DEM_REJECT_BAND_M
                && bil_c <= hi + DEM_REJECT_BAND_M =>
        {
            let vis = bil_c.clamp(lo, hi) + VISUAL_LIFT_M;
            let max_c_clamped = max_c.clamp(lo - 2.0, hi + 6.0);
            let max_l =
                max_c_clamped.max(dem_l.map_or(bil_c, |(_, m)| m.clamp(lo - 2.0, hi + 6.0)));
            let max_r =
                max_c_clamped.max(dem_r.map_or(bil_c, |(_, m)| m.clamp(lo - 2.0, hi + 6.0)));
            (
                vis,
                vis.max(max_c_clamped + DEM_DEPTH_CLEARANCE_M),
                vis.max(max_l + DEM_DEPTH_CLEARANCE_M),
                vis.max(max_r + DEM_DEPTH_CLEARANCE_M),
            )
        }
        _ => {
            let chord_msl = axis.start_msl_m + (axis.end_msl_m - axis.start_msl_m) * frac;
            let vis = chord_msl + VISUAL_LIFT_M;
            let clear = chord_msl + CHORD_DEPTH_CLEARANCE_M;
            (vis, clear, clear, clear)
        }
    };

    StationProjection {
        left: vertex_with_clearance(stamp, &left_zero, vis_msl, depth_l_msl),
        right: vertex_with_clearance(stamp, &right_zero, vis_msl, depth_r_msl),
        center: vertex_with_clearance(stamp, &center_zero, vis_msl, depth_c_msl),
    }
}

fn sea_globe(render_body: Body, geo: &Geodetic) -> DVec3 {
    unit_sphere_from_geodetic(geo) * render_body.radius_meters
}

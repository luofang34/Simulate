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

mod markings;

use glam::DVec3;
use maplibre::overlay::{DepthProbe, MAX_PROBES, OverlayMesh};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_scenery::{LandingRunwayGeometry, RunwaySurfaceKind};

use crate::pose::{ConformalFrameStamp, local_enu_from_geodetic, project_geodetic_to_screen};

const RAIL_STEPS: usize = 20;
pub(super) const VISUAL_LIFT_M: f64 = 0.35;
const DEM_DEPTH_CLEARANCE_M: f64 = 0.90;
pub(super) const CHORD_DEPTH_CLEARANCE_M: f64 = 0.65;
const DEM_REJECT_BAND_M: f64 = 12.0;

#[derive(Clone, Copy)]
pub(super) struct StationProjection {
    pub(super) left: Option<([f32; 2], f32)>,
    pub(super) right: Option<([f32; 2], f32)>,
    pub(super) center: Option<([f32; 2], f32)>,
}

pub(super) struct StripAxis {
    pub(super) start_sea_ecef: DVec3,
    pub(super) end_sea_ecef: DVec3,
    pub(super) start_ellip_m: f64,
    pub(super) end_ellip_m: f64,
    pub(super) half_width_m: f64,
    pub(super) fallback_bearing_rad: f64,
    pub(super) valid_msl_range: (f64, f64),
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
            &rwy.pavement_start,
            &rwy.threshold,
            rwy.displaced_threshold_m,
            true,
            sample_dem_msl,
        );
    }

    let (primary_axis, primary_len_m, has_recip, recip_geo) =
        primary_strip_axis(stamp.physics_ellipsoid, rwy);
    let primary_stations = sample_axis_stations(
        stamp,
        viewport_px,
        &primary_axis,
        RAIL_STEPS,
        sample_dem_msl,
    );
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
            &recip_geo,
            &rwy.far_end,
            rwy.reciprocal_displaced_m,
            false,
            sample_dem_msl,
        );
    }
    if probes.len() < MAX_PROBES
        && let Some((px, depth)) = primary_stations.first().and_then(|s| s.center)
    {
        probes.push(DepthProbe {
            position: px,
            depth,
        });
    }
}

fn primary_strip_axis(
    body: BodyEllipsoid,
    rwy: &LandingRunwayGeometry,
) -> (StripAxis, f64, bool, Geodetic) {
    let half_w = (rwy.width_m * 0.5).max(4.0);
    let total_landing_m = rwy.landing_length_m.max(1.0);
    let primary_len_m = (total_landing_m - rwy.reciprocal_displaced_m.max(0.0))
        .clamp(60.0_f64.min(total_landing_m), total_landing_m);
    let recip_frac = (primary_len_m / total_landing_m).clamp(0.05, 1.0);
    let thr_sea = sea_ecef(body, &rwy.threshold);
    let far_sea = sea_ecef(body, &rwy.far_end);
    let recip_sea = thr_sea + (far_sea - thr_sea) * recip_frac;
    let recip_ellip =
        rwy.threshold.height_m + (rwy.far_end.height_m - rwy.threshold.height_m) * recip_frac;
    let mut recip_geo = body.fixed_to_geodetic(recip_sea);
    recip_geo.height_m = recip_ellip;
    let has_recip = rwy.reciprocal_displaced_m > 10.0 && recip_frac < 0.995;
    let axis = StripAxis {
        start_sea_ecef: thr_sea,
        end_sea_ecef: recip_sea,
        start_ellip_m: rwy.threshold.height_m,
        end_ellip_m: recip_ellip,
        half_width_m: half_w,
        fallback_bearing_rad: rwy.true_bearing_rad,
        valid_msl_range: rwy.valid_surface_msl_range(),
    };
    (axis, primary_len_m, has_recip, recip_geo)
}

#[allow(clippy::too_many_arguments)]
fn append_zone_segment(
    mesh: &mut OverlayMesh,
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    rwy: &LandingRunwayGeometry,
    start: &Geodetic,
    end: &Geodetic,
    length_m: f64,
    pre_threshold: bool,
    sample_dem_msl: &mut dyn FnMut(f64, f64) -> Option<(f64, f64)>,
) {
    let displaced_turf = rwy.surface.is_unpaved() || rwy.surface == RunwaySurfaceKind::PavedTurf;
    let axis = StripAxis {
        start_sea_ecef: sea_ecef(stamp.physics_ellipsoid, start),
        end_sea_ecef: sea_ecef(stamp.physics_ellipsoid, end),
        start_ellip_m: start.height_m,
        end_ellip_m: end.height_m,
        half_width_m: (rwy.width_m * 0.5).max(4.0),
        fallback_bearing_rad: rwy.true_bearing_rad,
        valid_msl_range: rwy.valid_surface_msl_range(),
    };
    let steps = ((length_m / 45.0).ceil() as usize).clamp(2, 12);
    let stations = sample_axis_stations(stamp, viewport_px, &axis, steps, sample_dem_msl);
    markings::append_displaced_zone(mesh, &stations, displaced_turf, pre_threshold);
}

fn sample_axis_stations(
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    axis: &StripAxis,
    steps: usize,
    sample_dem_msl: &mut dyn FnMut(f64, f64) -> Option<(f64, f64)>,
) -> Vec<StationProjection> {
    let denom = steps.max(1) as f64;
    let mut out = Vec::with_capacity(steps.wrapping_add(1));
    for i in 0..=steps {
        let frac = i as f64 / denom;
        out.push(sample_station(
            stamp,
            viewport_px,
            axis,
            frac,
            sample_dem_msl,
        ));
    }
    out
}

pub(super) fn sample_station(
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    axis: &StripAxis,
    frac: f64,
    sample_dem_msl: &mut dyn FnMut(f64, f64) -> Option<(f64, f64)>,
) -> StationProjection {
    let chord_vec = axis.end_sea_ecef - axis.start_sea_ecef;
    let center_sea = axis.start_sea_ecef + chord_vec * frac;
    let center_zero = stamp.physics_ellipsoid.fixed_to_geodetic(center_sea);
    let local = LocalFrame::at(&center_zero);
    let up = local.up();
    let tangent_fwd = (chord_vec - up * chord_vec.dot(up))
        .try_normalize()
        .unwrap_or_else(|| {
            let (sin_b, cos_b) = axis.fallback_bearing_rad.sin_cos();
            local.north() * cos_b + local.east() * sin_b
        });
    let right_ecef = tangent_fwd
        .cross(up)
        .try_normalize()
        .unwrap_or_else(|| local.east());

    let left_zero = stamp
        .physics_ellipsoid
        .fixed_to_geodetic(center_sea - right_ecef * axis.half_width_m);
    let right_zero = stamp
        .physics_ellipsoid
        .fixed_to_geodetic(center_sea + right_ecef * axis.half_width_m);

    let (lo, hi) = axis.valid_msl_range;
    let dem_c = sample_dem_msl(center_zero.lat_deg(), center_zero.lon_deg());
    let dem_l = sample_dem_msl(left_zero.lat_deg(), left_zero.lon_deg());
    let dem_r = sample_dem_msl(right_zero.lat_deg(), right_zero.lon_deg());

    let (vis_ellip, depth_c_ellip, depth_l_ellip, depth_r_ellip) = match dem_c {
        Some((bil_c, max_c))
            if bil_c.is_finite()
                && bil_c >= lo - DEM_REJECT_BAND_M
                && bil_c <= hi + DEM_REJECT_BAND_M =>
        {
            let undul = stamp.geoid_undulation_m;
            let vis = bil_c.clamp(lo, hi) + undul + VISUAL_LIFT_M;
            let max_c_clamped = max_c.clamp(lo - 2.0, hi + 6.0);
            let max_l =
                max_c_clamped.max(dem_l.map_or(bil_c, |(_, m)| m.clamp(lo - 2.0, hi + 6.0)));
            let max_r =
                max_c_clamped.max(dem_r.map_or(bil_c, |(_, m)| m.clamp(lo - 2.0, hi + 6.0)));
            (
                vis,
                vis.max(max_c_clamped + undul + DEM_DEPTH_CLEARANCE_M),
                vis.max(max_l + undul + DEM_DEPTH_CLEARANCE_M),
                vis.max(max_r + undul + DEM_DEPTH_CLEARANCE_M),
            )
        }
        _ => {
            let chord_ellip = axis.start_ellip_m + (axis.end_ellip_m - axis.start_ellip_m) * frac;
            let vis = chord_ellip + VISUAL_LIFT_M;
            let clear = chord_ellip + CHORD_DEPTH_CLEARANCE_M;
            (vis, clear, clear, clear)
        }
    };

    StationProjection {
        left: project_with_clearance(stamp, viewport_px, &left_zero, vis_ellip, depth_l_ellip),
        right: project_with_clearance(stamp, viewport_px, &right_zero, vis_ellip, depth_r_ellip),
        center: project_with_clearance(stamp, viewport_px, &center_zero, vis_ellip, depth_c_ellip),
    }
}

pub(super) fn project_with_clearance(
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    zero_geo: &Geodetic,
    vis_height_m: f64,
    depth_height_m: f64,
) -> Option<([f32; 2], f32)> {
    let vis_geo = Geodetic {
        height_m: vis_height_m,
        ..*zero_geo
    };
    let (px, vis_depth) =
        project_geodetic_to_screen(stamp, &vis_geo, stamp.geoid_undulation_m, viewport_px)?;
    let Some(local) = local_enu_from_geodetic(stamp, &vis_geo, stamp.geoid_undulation_m) else {
        return Some((px, vis_depth));
    };
    let range_m = local.length();
    let near = stamp.external_view.frustum.near;
    if range_m <= near || vis_depth <= 0.0 {
        return Some((px, vis_depth));
    }
    let dh = (depth_height_m - vis_height_m).max(0.5);
    let sin_grazing = (local.z.abs() / range_m).max(0.015);
    let forward_m = near / f64::from(vis_depth);
    let ray_pull_m = (dh / sin_grazing).min(120.0).min(forward_m * 0.45);
    let clear_forward_m = (forward_m - ray_pull_m).max(near * 1.001);
    let clear_depth = (near / clear_forward_m).clamp(0.0, 1.0) as f32;
    Some((px, clear_depth.max(vis_depth)))
}

pub(super) fn sea_ecef(body: BodyEllipsoid, geo: &Geodetic) -> DVec3 {
    body.geodetic_to_fixed(&Geodetic {
        height_m: 0.0,
        ..*geo
    })
}

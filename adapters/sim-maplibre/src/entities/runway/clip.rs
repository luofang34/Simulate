//! 3D camera-eye frustum and near-plane clipping for SVS / XVS conformal runway surfaces
//! and markings.
//!
//! Clips 3D line segments (Liang-Barsky) and 3D convex surface quads (Sutherland-Hodgman)
//! against the camera near clip plane and a generous `±4.0` NDC guardband before perspective
//! division, preventing abrupt whole-station truncation when runway thresholds, sidelines,
//! centreline stripes, or pavement quads pass beneath or behind the aircraft during flare,
//! rollout, or low overflight.

use glam::{DVec3, DVec4};
use maplibre::overlay::OverlayMesh;
use sim_geo::Geodetic;

use crate::{
    entities::{push_quad, push_segment, push_triangle},
    pose::{ConformalFrameStamp, local_enu_from_geodetic},
};

/// Homogeneous NDC guardband (`4×` viewport half-extent), keeping clipped vertices well
/// outside the visible `[0, w] × [0, h]` bezel while bounding screen-space coordinates.
const GUARD_NDC: f64 = 4.0;

/// Unprojected 3D runway point in local ENU, camera-eye, and homogeneous clip coordinates,
/// paired with its vertical DEM grazing-clearance margin `dh_m`.
#[derive(Clone, Copy, Debug)]
pub(super) struct ClipVertex {
    local_enu: DVec3,
    eye: DVec3,
    clip: DVec4,
    dh_m: f64,
}

impl ClipVertex {
    pub(super) const ZERO: Self = Self {
        local_enu: DVec3::ZERO,
        eye: DVec3::ZERO,
        clip: DVec4::ZERO,
        dh_m: 0.5,
    };

    fn lerp(&self, other: &Self, t: f64) -> Self {
        Self {
            local_enu: self.local_enu + (other.local_enu - self.local_enu) * t,
            eye: self.eye + (other.eye - self.eye) * t,
            clip: self.clip + (other.clip - self.clip) * t,
            dh_m: self.dh_m + (other.dh_m - self.dh_m) * t,
        }
    }
}

#[derive(Clone, Copy)]
enum ClipPlane {
    Near(f64),
    Left,
    Right,
    Bottom,
    Top,
}

impl ClipPlane {
    fn dist(self, v: &ClipVertex) -> f64 {
        match self {
            Self::Near(near_clip) => -v.eye.z - near_clip,
            Self::Left => v.clip.x + GUARD_NDC * v.clip.w,
            Self::Right => -v.clip.x + GUARD_NDC * v.clip.w,
            Self::Bottom => v.clip.y + GUARD_NDC * v.clip.w,
            Self::Top => -v.clip.y + GUARD_NDC * v.clip.w,
        }
    }
}

fn clip_planes(stamp: &ConformalFrameStamp) -> [ClipPlane; 5] {
    let near = stamp.external_view.frustum.near;
    let near_clip = (near * 1.02).max(near + 1e-4);
    [
        ClipPlane::Near(near_clip),
        ClipPlane::Left,
        ClipPlane::Right,
        ClipPlane::Bottom,
        ClipPlane::Top,
    ]
}

/// Builds an unclipped 3D [`ClipVertex`] at datum/MSL elevation `vis_msl_m`
/// (`point_geoid_undulation_m = 0.0`) with DEM depth-clearance ceiling `depth_msl_m`.
pub(super) fn vertex_with_clearance(
    stamp: &ConformalFrameStamp,
    zero_geo: &Geodetic,
    vis_msl_m: f64,
    depth_msl_m: f64,
) -> Option<ClipVertex> {
    let vis_geo = Geodetic {
        height_m: vis_msl_m,
        ..*zero_geo
    };
    let local_enu = local_enu_from_geodetic(stamp, &vis_geo, 0.0)?;
    let v4 = cgmath::Vector4::new(local_enu.x, local_enu.y, local_enu.z, 1.0);
    let eye_cg = stamp.external_view.view * v4;
    let clip_cg = stamp.external_view.frustum.projection() * eye_cg;
    let eye = DVec3::new(eye_cg.x, eye_cg.y, eye_cg.z);
    let clip = DVec4::new(clip_cg.x, clip_cg.y, clip_cg.z, clip_cg.w);
    if !eye.is_finite() || !clip.is_finite() {
        return None;
    }
    Some(ClipVertex {
        local_enu,
        eye,
        clip,
        dh_m: (depth_msl_m - vis_msl_m).max(0.5),
    })
}

/// Projects `v` to `([x_px, y_px], clear_depth)` only when it lies strictly in front of the
/// near plane and within `[0, viewport_px[0]) × [0, viewport_px[1])` (for `DepthProbe`s).
pub(super) fn project_in_viewport(
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    v: &ClipVertex,
) -> Option<([f32; 2], f32)> {
    let (px, depth) = project_clipped_vertex(stamp, viewport_px, v)?;
    let [w, h] = [viewport_px[0] as f32, viewport_px[1] as f32];
    if (0.0..w).contains(&px[0]) && (0.0..h).contains(&px[1]) {
        Some((px, depth))
    } else {
        None
    }
}

fn project_clipped_vertex(
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    v: &ClipVertex,
) -> Option<([f32; 2], f32)> {
    let [w_px, h_px] = viewport_px;
    if w_px == 0 || h_px == 0 {
        return None;
    }
    let near = stamp.external_view.frustum.near;
    let forward_m = -v.eye.z;
    if !forward_m.is_finite() || forward_m <= near || v.clip.w.abs() <= f64::EPSILON {
        return None;
    }
    let ndc_x = v.clip.x / v.clip.w;
    let ndc_y = v.clip.y / v.clip.w;
    let x_px = ((ndc_x + 1.0) * 0.5 * f64::from(w_px)) as f32;
    let y_px = ((1.0 - ndc_y) * 0.5 * f64::from(h_px)) as f32;
    let vis_depth = (near / forward_m).clamp(0.0, 1.0) as f32;
    let range_m = v.local_enu.length();
    if range_m <= near || vis_depth <= 0.0 {
        return Some(([x_px, y_px], vis_depth));
    }
    let sin_grazing = (v.local_enu.z.abs() / range_m).max(0.015);
    let ray_pull_m = (v.dh_m.max(0.5) / sin_grazing)
        .min(120.0)
        .min(forward_m * 0.45);
    let clear_forward_m = (forward_m - ray_pull_m).max(near * 1.001);
    let clear_depth = (near / clear_forward_m).clamp(0.0, 1.0) as f32;
    Some(([x_px, y_px], clear_depth.max(vis_depth)))
}

/// Clips 3D segment `(a, b)` against `stamp`'s near plane and `±4.0` NDC guardband, then
/// appends the projected screen-space stroke to `mesh`.
pub(super) fn push_clipped_segment(
    mesh: &mut OverlayMesh,
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    a: Option<ClipVertex>,
    b: Option<ClipVertex>,
    width_px: f32,
    color: [f32; 4],
) {
    let (Some(va), Some(vb)) = (a, b) else {
        return;
    };
    let (mut t0, mut t1) = (0.0_f64, 1.0_f64);
    for plane in clip_planes(stamp) {
        let f0 = plane.dist(&va);
        let f1 = plane.dist(&vb);
        if f0 < 0.0 && f1 < 0.0 {
            return;
        }
        let df = f1 - f0;
        if f0 < 0.0 {
            if df.abs() <= 1e-12 {
                return;
            }
            t0 = t0.max((-f0 / df).clamp(0.0, 1.0));
        } else if f1 < 0.0 {
            if df.abs() <= 1e-12 {
                return;
            }
            t1 = t1.min((-f0 / df).clamp(0.0, 1.0));
        }
        if t0 > t1 {
            return;
        }
    }
    let ca = if t0 == 0.0 { va } else { va.lerp(&vb, t0) };
    let cb = if t1 == 1.0 { vb } else { va.lerp(&vb, t1) };
    if let (Some(pa), Some(pb)) = (
        project_clipped_vertex(stamp, viewport_px, &ca),
        project_clipped_vertex(stamp, viewport_px, &cb),
    ) {
        push_segment(mesh, pa.0, pa.1, pb.0, pb.1, width_px, color);
    }
}

struct PolyBuf {
    verts: [ClipVertex; 12],
    len: usize,
}

impl PolyBuf {
    const fn empty() -> Self {
        Self {
            verts: [ClipVertex::ZERO; 12],
            len: 0,
        }
    }

    fn from_quad(quad: [ClipVertex; 4]) -> Self {
        let mut out = Self::empty();
        for v in quad {
            out.push(v);
        }
        out
    }

    fn as_slice(&self) -> &[ClipVertex] {
        self.verts.get(..self.len).unwrap_or(&[])
    }

    fn push(&mut self, v: ClipVertex) {
        if let Some(slot) = self.verts.get_mut(self.len) {
            *slot = v;
            self.len = self.len.wrapping_add(1);
        }
    }
}

fn clip_poly_plane(src: &PolyBuf, plane: ClipPlane) -> PolyBuf {
    let mut dst = PolyBuf::empty();
    let slice = src.as_slice();
    let Some(&last) = slice.last() else {
        return dst;
    };
    let mut prev = last;
    let mut prev_d = plane.dist(&prev);
    for &curr in slice {
        let curr_d = plane.dist(&curr);
        if (prev_d >= 0.0) != (curr_d >= 0.0) {
            let denom = prev_d - curr_d;
            if denom.abs() > 1e-12 {
                let t = (prev_d / denom).clamp(0.0, 1.0);
                dst.push(prev.lerp(&curr, t));
            }
        }
        if curr_d >= 0.0 {
            dst.push(curr);
        }
        prev = curr;
        prev_d = curr_d;
    }
    dst
}

/// Clips 3D quad `[al, ar, br, bl]` against `stamp`'s near plane and `±4.0` NDC guardband
/// via Sutherland-Hodgman, preserving the 4-vertex `push_quad` fast path when unclipped.
pub(super) fn push_clipped_quad(
    mesh: &mut OverlayMesh,
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    corners: [Option<ClipVertex>; 4],
    color: [f32; 4],
) {
    let [Some(al), Some(ar), Some(br), Some(bl)] = corners else {
        return;
    };
    let quad = [al, ar, br, bl];
    let planes = clip_planes(stamp);
    let mut all_inside = true;
    for plane in planes {
        let mut any_inside = false;
        for v in &quad {
            if plane.dist(v) >= 0.0 {
                any_inside = true;
            } else {
                all_inside = false;
            }
        }
        if !any_inside {
            return;
        }
    }
    if all_inside {
        if let (Some(pal), Some(par), Some(pbr), Some(pbl)) = (
            project_clipped_vertex(stamp, viewport_px, &al),
            project_clipped_vertex(stamp, viewport_px, &ar),
            project_clipped_vertex(stamp, viewport_px, &br),
            project_clipped_vertex(stamp, viewport_px, &bl),
        ) {
            push_quad(mesh, pal, par, pbr, pbl, color);
        }
        return;
    }

    let mut poly = PolyBuf::from_quad(quad);
    for plane in planes {
        poly = clip_poly_plane(&poly, plane);
        if poly.len < 3 {
            return;
        }
    }
    let slice = poly.as_slice();
    if let Some((&v0, rest)) = slice.split_first()
        && let Some(p0) = project_clipped_vertex(stamp, viewport_px, &v0)
    {
        for pair in rest.windows(2) {
            if let [v1, v2] = pair
                && let (Some(p1), Some(p2)) = (
                    project_clipped_vertex(stamp, viewport_px, v1),
                    project_clipped_vertex(stamp, viewport_px, v2),
                )
            {
                push_triangle(mesh, p0, p1, p2, color);
            }
        }
    }
}

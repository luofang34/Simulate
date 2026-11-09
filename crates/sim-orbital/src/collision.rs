//! Certified conservative proximity, rotating-OBB, and discontinuous-terrain
//! collision root isolation (`Gate 4`).
//!
//! Combines degree-7 Bernstein convex-hull bounds, rotational sweep bounds,
//! exact 15-axis `SAT` OBB separation, and quadtree/region [`ElevationSlab`]s
//! so interior double-crossings, grazing contacts, rotating booms, and vertical
//! terrain cliffs are never missed.

pub mod bernstein;
pub mod roots;
pub mod tube;

use glam::{DQuat, DVec3};
use sim_spacetime::{CollisionTerrain, ElevationSlab, TICKS_PER_SECOND, TickSpan};

use crate::{
    dense_segment::PlantTrajectorySegment, orientation::BodyOrientationCatalog,
    plant::quat_rotate_vec_portable, scalar_math::sqrt, time::TdbEpoch,
};

pub use bernstein::{
    certified_hull_min_norm_3d, eval_bernstein_scalar, eval_bernstein_vec3, monomial_to_bernstein,
    split_bernstein_vec3_mid,
};
pub use roots::{ContactRootKind, IsolatedContactRoot, isolate_scalar_bernstein_roots};
pub use tube::{
    OrientedCollisionShape, RendezvousGroup, SweptReachabilityTube, TubeUncertaintyBudget,
    form_rendezvous_groups, obb_signed_separation_m,
};

/// Kinematic & interpolation bounds for one vessel in narrow-phase rotating-OBB detection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SweptObbKinematics {
    /// Rigid OBB collision geometry in body coordinates.
    pub shape: OrientedCollisionShape,
    /// Upper bound on body angular speed $\|\boldsymbol{\omega}\|$ across the segment ($\text{rad}/\text{s}$).
    pub omega_max_rad_s: f64,
    /// Upper bound on body angular acceleration $\|\dot{\boldsymbol{\omega}}\|$ across the segment ($\text{rad}/\text{s}^2$).
    pub alpha_max_rad_s2: f64,
    /// Certified dense-output CG position remainder $\varepsilon_{\text{interp}}$ (`m`).
    pub interp_rem_m: f64,
}

/// Evaluates pose `(pos_icrf_m, q_body_to_icrf)` of a swept rigid body at segment-local offset `tau_s`.
pub trait SweptPoseTrajectory {
    /// Segment duration $T > 0$ ($\text{s}$).
    fn duration_s(&self) -> f64;
    /// Degree-7 Bernstein CG position control points on $[0, T]$ in common `ICRF` coordinates (`m`).
    fn cg_bernstein_8(&self) -> [DVec3; 8];
    /// Evaluates `(pos_icrf_m, q_body_to_icrf)` at `tau_s` $\in [0, T]$.
    fn sample_pose(&self, tau_s: f64) -> (DVec3, DQuat);
}

impl SweptPoseTrajectory for PlantTrajectorySegment {
    fn duration_s(&self) -> f64 {
        self.duration_s
    }

    fn cg_bernstein_8(&self) -> [DVec3; 8] {
        let dense = self.as_dense();
        let bx = dense.component_bernstein_8(0);
        let by = dense.component_bernstein_8(1);
        let bz = dense.component_bernstein_8(2);
        let mut out = [DVec3::ZERO; 8];
        for (((slot, &x), &y), &z) in out.iter_mut().zip(bx.iter()).zip(by.iter()).zip(bz.iter()) {
            *slot = DVec3::new(x, y, z);
        }
        out
    }

    fn sample_pose(&self, tau_s: f64) -> (DVec3, DQuat) {
        let y = self
            .as_dense()
            .evaluate_tau(tau_s.clamp(0.0, self.duration_s));
        let st = crate::plant::ContinuousPlantState::from_ode_array(self.primary_body, &y, true);
        (st.pos_rel_icrf_m, st.q_body_to_icrf)
    }
}

/// Isolates all initial overlaps, entries, exits, and grazing contacts between two rotating 3D OBBs
/// along `traj_a` and `traj_b` over `[0, min(T_a, T_b)]` to `tol_tau_s` precision.
pub fn detect_swept_obb_contacts<A: SweptPoseTrajectory, B: SweptPoseTrajectory>(
    traj_a: &A,
    kin_a: &SweptObbKinematics,
    traj_b: &B,
    kin_b: &SweptObbKinematics,
    tol_tau_s: f64,
) -> Vec<IsolatedContactRoot> {
    let dur = traj_a.duration_s().min(traj_b.duration_s());
    if dur <= 0.0 {
        return Vec::new();
    }
    let ba = traj_a.cg_bernstein_8();
    let bb = traj_b.cg_bernstein_8();
    let mut rel_ctrl = [DVec3::ZERO; 8];
    for ((slot, &pa), &pb) in rel_ctrl.iter_mut().zip(ba.iter()).zip(bb.iter()) {
        *slot = pb - pa;
    }

    let gap_fn = |tau: f64| -> f64 {
        let (pa, qa) = traj_a.sample_pose(tau);
        let (pb, qb) = traj_b.sample_pose(tau);
        obb_signed_separation_m(pa, qa, &kin_a.shape, pb, qb, &kin_b.shape)
    };

    let mut out = Vec::new();
    let g0 = gap_fn(0.0);
    if g0 <= 0.0 {
        out.push(IsolatedContactRoot {
            tau_s: 0.0,
            value: g0,
            kind: ContactRootKind::InitialOverlap,
        });
    }

    let ctx = ObbSearchCtx {
        kin_a,
        kin_b,
        outer_sum_m: kin_a.shape.outer_cg_radius_m()
            + kin_b.shape.outer_cg_radius_m()
            + kin_a.interp_rem_m
            + kin_b.interp_rem_m,
        tol_tau_s: tol_tau_s.max(1.0e-11),
        gap_fn: &gap_fn,
    };
    subdivide_obb_pair(&rel_ctrl, 0.0, dur, g0, gap_fn(dur), &ctx, &mut out, 0);
    out
}

struct ObbSearchCtx<'a, G> {
    kin_a: &'a SweptObbKinematics,
    kin_b: &'a SweptObbKinematics,
    outer_sum_m: f64,
    tol_tau_s: f64,
    gap_fn: &'a G,
}

#[allow(clippy::too_many_arguments)]
fn subdivide_obb_pair<G: Fn(f64) -> f64>(
    rel_ctrl: &[DVec3; 8],
    a: f64,
    b: f64,
    ga: f64,
    gb: f64,
    ctx: &ObbSearchCtx<'_, G>,
    out: &mut Vec<IsolatedContactRoot>,
    depth: u32,
) {
    let span = b - a;
    if span <= 0.0 {
        return;
    }
    if certified_hull_min_norm_3d(rel_ctrl) > ctx.outer_sum_m {
        return;
    }

    let mid = 0.5 * (a + b);
    let gm = (ctx.gap_fn)(mid);
    let half_dt = 0.5 * span;
    let rel_mid = eval_bernstein_vec3(rel_ctrl, 0.5);
    let mut cg_exc = 0.0_f64;
    for &p in rel_ctrl {
        cg_exc = cg_exc.max(sqrt((p - rel_mid).length_squared()));
    }
    let total_exc = cg_exc
        + ctx.kin_a.interp_rem_m
        + ctx.kin_b.interp_rem_m
        + ctx.kin_a.shape.rotational_excursion_m(
            ctx.kin_a.omega_max_rad_s,
            ctx.kin_a.alpha_max_rad_s2,
            half_dt,
        )
        + ctx.kin_b.shape.rotational_excursion_m(
            ctx.kin_b.omega_max_rad_s,
            ctx.kin_b.alpha_max_rad_s2,
            half_dt,
        );
    if gm > total_exc || gm < -total_exc {
        return;
    }

    if span <= 1.0e-3 && ((ga > 0.0 && gb <= 0.0) || (ga < 0.0 && gb >= 0.0)) {
        let tau_star = refine_bracket_bisect_secant(a, b, ga, gb, ctx.tol_tau_s, ctx.gap_fn);
        let kind = if ga > 0.0 {
            ContactRootKind::Entry
        } else {
            ContactRootKind::Exit
        };
        out.push(IsolatedContactRoot {
            tau_s: tau_star,
            value: (ctx.gap_fn)(tau_star),
            kind,
        });
        return;
    }

    if span <= ctx.tol_tau_s || depth >= 50 {
        if gm.abs() <= total_exc + 1.0e-9 {
            out.push(IsolatedContactRoot {
                tau_s: mid,
                value: gm,
                kind: ContactRootKind::Grazing,
            });
        }
        return;
    }

    let (left, right) = split_bernstein_vec3_mid(rel_ctrl);
    subdivide_obb_pair(&left, a, mid, ga, gm, ctx, out, depth.wrapping_add(1));
    subdivide_obb_pair(&right, mid, b, gm, gb, ctx, out, depth.wrapping_add(1));
}

/// Isolates all terrain/cliff contacts along a primary-centered `PlantTrajectorySegment`
/// against a [`CollisionTerrain`] using conservative geodetic [`ElevationSlab`] bounds
/// (capturing discontinuous vertical cliffs as well as smooth slopes to `tol_tau_s` precision).
pub fn detect_terrain_contacts(
    seg: &PlantTrajectorySegment,
    probe_radius_m: f64,
    interp_rem_m: f64,
    terrain: &dyn CollisionTerrain,
    orientation: &dyn BodyOrientationCatalog,
    tol_tau_s: f64,
) -> Vec<IsolatedContactRoot> {
    if seg.duration_s <= 0.0 {
        return Vec::new();
    }
    let cg_ctrl = seg.cg_bernstein_8();
    let clearance_fn = |tau: f64| -> f64 {
        let (pos_icrf, _) = seg.sample_pose(tau);
        let ep = epoch_plus_seconds(seg.start_epoch, tau);
        let p_fixed = icrf_to_body_fixed(seg.primary_body, pos_icrf, ep, orientation);
        let (geo, ground) = terrain.sample_fixed(p_fixed);
        geo.height_m - probe_radius_m - ground.ellipsoidal_height_m
    };

    let mut out = Vec::new();
    let c0 = clearance_fn(0.0);
    if c0 <= 0.0 {
        out.push(IsolatedContactRoot {
            tau_s: 0.0,
            value: c0,
            kind: ContactRootKind::InitialOverlap,
        });
    }

    let ctx = TerrainSearchCtx {
        seg,
        probe_radius_m: probe_radius_m.max(0.0),
        interp_rem_m: interp_rem_m.max(0.0),
        terrain,
        orientation,
        tol_tau_s: tol_tau_s.max(1.0e-10),
        clearance_fn: &clearance_fn,
    };
    let c1 = clearance_fn(seg.duration_s);
    subdivide_terrain(&cg_ctrl, 0.0, seg.duration_s, c0, c1, &ctx, &mut out, 0);
    out
}

struct TerrainSearchCtx<'a, C> {
    seg: &'a PlantTrajectorySegment,
    probe_radius_m: f64,
    interp_rem_m: f64,
    terrain: &'a dyn CollisionTerrain,
    orientation: &'a dyn BodyOrientationCatalog,
    tol_tau_s: f64,
    clearance_fn: &'a C,
}

#[allow(clippy::too_many_arguments)]
fn subdivide_terrain<C: Fn(f64) -> f64>(
    cg_ctrl: &[DVec3; 8],
    a: f64,
    b: f64,
    ca: f64,
    cb: f64,
    ctx: &TerrainSearchCtx<'_, C>,
    out: &mut Vec<IsolatedContactRoot>,
    depth: u32,
) {
    let span = b - a;
    if span <= 0.0 {
        return;
    }
    let (h_sc_min, h_sc_max, slab) = conservative_terrain_bounds(cg_ctrl, a, b, ctx);
    let margin = ctx.probe_radius_m + ctx.interp_rem_m + 0.05;
    if h_sc_min - margin > slab.max_m || h_sc_max + margin < slab.min_m {
        return;
    }

    let mid = 0.5 * (a + b);
    let cm = (ctx.clearance_fn)(mid);
    let slab_jump = (slab.max_m - slab.min_m).abs();

    if span <= 1.0e-2 && slab_jump <= 0.05 && ((ca > 0.0 && cb <= 0.0) || (ca < 0.0 && cb >= 0.0)) {
        let tau_star = refine_bracket_bisect_secant(a, b, ca, cb, ctx.tol_tau_s, ctx.clearance_fn);
        let kind = if ca > 0.0 {
            ContactRootKind::Entry
        } else {
            ContactRootKind::Exit
        };
        out.push(IsolatedContactRoot {
            tau_s: tau_star,
            value: (ctx.clearance_fn)(tau_star),
            kind,
        });
        return;
    }

    if span <= ctx.tol_tau_s || depth >= 52 {
        if (ca > 0.0 && cb <= 0.0) || (ca <= 0.0 && cb > 0.0) || cm.abs() <= 1.0e-4 {
            let kind = if ca > 0.0 && cb <= 0.0 {
                ContactRootKind::Entry
            } else if ca < 0.0 && cb >= 0.0 {
                ContactRootKind::Exit
            } else {
                ContactRootKind::Grazing
            };
            out.push(IsolatedContactRoot {
                tau_s: mid,
                value: cm,
                kind,
            });
        }
        return;
    }

    let (left, right) = split_bernstein_vec3_mid(cg_ctrl);
    subdivide_terrain(&left, a, mid, ca, cm, ctx, out, depth.wrapping_add(1));
    subdivide_terrain(&right, mid, b, cm, cb, ctx, out, depth.wrapping_add(1));
}

fn conservative_terrain_bounds<C>(
    cg_ctrl: &[DVec3; 8],
    a: f64,
    b: f64,
    ctx: &TerrainSearchCtx<'_, C>,
) -> (f64, f64, ElevationSlab) {
    let mid = 0.5 * (a + b);
    let ep_mid = epoch_plus_seconds(ctx.seg.start_epoch, mid);
    let ell = ctx.terrain.ellipsoid();
    let rot_pad_m = ell.omega_rad_s * (0.5 * (b - a)) * (ell.semi_major_m + 1.0e6);

    let mut fixed_lo = DVec3::splat(f64::INFINITY);
    let mut fixed_hi = DVec3::splat(f64::NEG_INFINITY);
    let mut lat_min = f64::INFINITY;
    let mut lat_max = f64::NEG_INFINITY;
    let mut lon_min = f64::INFINITY;
    let mut lon_max = f64::NEG_INFINITY;
    let mut h_min = f64::INFINITY;
    let mut h_max = f64::NEG_INFINITY;

    for &p_icrf in cg_ctrl {
        let pf = icrf_to_body_fixed(ctx.seg.primary_body, p_icrf, ep_mid, ctx.orientation);
        fixed_lo = fixed_lo.min(pf);
        fixed_hi = fixed_hi.max(pf);
        let g = ell.fixed_to_geodetic(pf);
        lat_min = lat_min.min(g.lat_rad);
        lat_max = lat_max.max(g.lat_rad);
        lon_min = lon_min.min(g.lon_rad);
        lon_max = lon_max.max(g.lon_rad);
        h_min = h_min.min(g.height_m);
        h_max = h_max.max(g.height_m);
    }

    let chord = sqrt((fixed_hi - fixed_lo).length_squared()) + 2.0 * rot_pad_m;
    let sagitta_m = 0.25 * chord * chord / ell.semi_minor_m().max(1.0);
    let ang_pad = ((sagitta_m + rot_pad_m + ctx.interp_rem_m) / ell.semi_minor_m().max(1.0))
        .min(std::f64::consts::PI);
    let slab = ctx.terrain.elevation_slab(
        (lat_min - ang_pad).max(-std::f64::consts::FRAC_PI_2),
        (lat_max + ang_pad).min(std::f64::consts::FRAC_PI_2),
        lon_min - ang_pad,
        lon_max + ang_pad,
    );
    (
        h_min - sagitta_m - rot_pad_m,
        h_max + sagitta_m + rot_pad_m,
        slab,
    )
}

fn icrf_to_body_fixed(
    body: crate::ids::NaifId,
    pos_icrf: DVec3,
    epoch: TdbEpoch,
    orientation: &dyn BodyOrientationCatalog,
) -> DVec3 {
    if let Ok(rot) = orientation.body_rotation(body, epoch) {
        rot.r_icrf_to_fixed * pos_icrf
    } else {
        let _ = quat_rotate_vec_portable;
        pos_icrf
    }
}

fn epoch_plus_seconds(base: TdbEpoch, tau_s: f64) -> TdbEpoch {
    let ticks = (tau_s.max(0.0) * (TICKS_PER_SECOND as f64)).round() as u128;
    base.add_span(TickSpan::from_ticks_u128(ticks))
}

fn refine_bracket_bisect_secant<F: Fn(f64) -> f64>(
    mut lo: f64,
    mut hi: f64,
    mut flo: f64,
    mut fhi: f64,
    tol_s: f64,
    f: &F,
) -> f64 {
    for _ in 0..56_u32 {
        if (hi - lo).abs() <= tol_s {
            break;
        }
        let sec = if (fhi - flo).abs() > 1.0e-20 {
            lo - flo * (hi - lo) / (fhi - flo)
        } else {
            0.5 * (lo + hi)
        };
        let margin = 0.1 * (hi - lo);
        let x = if sec > lo + margin && sec < hi - margin {
            sec
        } else {
            0.5 * (lo + hi)
        };
        let fx = f(x);
        if fx == 0.0 {
            return x;
        }
        if (flo > 0.0 && fx > 0.0) || (flo < 0.0 && fx < 0.0) {
            lo = x;
            flo = fx;
        } else {
            hi = x;
            fhi = fx;
        }
    }
    0.5 * (lo + hi)
}

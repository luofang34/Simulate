//! Conservative `SweptReachabilityTube` envelopes, exact 15-axis 3D OBB separation,
//! and deterministic `RendezvousGroup` formation/splitting.

use std::collections::{BTreeMap, BTreeSet};

use glam::{DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};
use sim_spacetime::SimInstant;

use crate::{
    dense_segment::PlantTrajectorySegment, ids::VesselId, plant::quat_rotate_vec_portable,
    scalar_math::sqrt,
};

/// Oriented 3D bounding-box (`OBB`) collision geometry attached to a vessel's rigid body.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct OrientedCollisionShape {
    /// Box center offset from the vessel's center of mass in body axes (`m`).
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub center_body_m: DVec3,
    /// Positive half-extents $(e_x, e_y, e_z)$ along body $X, Y, Z$ axes (`m`).
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub half_extents_body_m: DVec3,
}

impl OrientedCollisionShape {
    /// Symmetric CG-centered box of half-extents `(hx, hy, hz)` in metres.
    #[must_use]
    pub const fn centered_box(hx: f64, hy: f64, hz: f64) -> Self {
        Self {
            center_body_m: DVec3::ZERO,
            half_extents_body_m: DVec3::new(hx, hy, hz),
        }
    }

    /// Tight outer sphere radius $R_{\text{bound}}$ from the vessel CG enclosing all 8 OBB corners (`m`).
    #[must_use]
    pub fn outer_cg_radius_m(&self) -> f64 {
        let c = self.center_body_m.abs() + self.half_extents_body_m.abs();
        sqrt(c.length_squared())
    }

    /// Inscribed core half-thickness $\min(e_x, e_y, e_z)$ (`m`).
    #[must_use]
    pub fn min_half_extent_m(&self) -> f64 {
        let e = self.half_extents_body_m.abs();
        e.x.min(e.y).min(e.z)
    }

    /// Conservative rotational excursion bound $L_{\text{arm}} \min(2, \omega_{\max}\tau + \frac{1}{2}\alpha_{\max}\tau^2)$
    /// relative to endpoint orientation at `tau = 0` (`m`).
    #[must_use]
    pub fn rotational_excursion_m(
        &self,
        omega_max_rad_s: f64,
        alpha_max_rad_s2: f64,
        dt_s: f64,
    ) -> f64 {
        let dt = dt_s.max(0.0);
        let theta_bound =
            (omega_max_rad_s.max(0.0) * dt + 0.5 * alpha_max_rad_s2.max(0.0) * dt * dt).min(2.0);
        self.outer_cg_radius_m() * theta_bound
    }

    /// Evaluates the 8 world-space (`ICRF` or body-fixed) OBB corner vertices given CG `pos_m` and `q_body_to_frame`.
    #[must_use]
    pub fn world_corners(&self, pos_m: DVec3, q_body_to_frame: DQuat) -> [DVec3; 8] {
        let e = self.half_extents_body_m.abs();
        let signs = [
            DVec3::new(-1.0, -1.0, -1.0),
            DVec3::new(1.0, -1.0, -1.0),
            DVec3::new(-1.0, 1.0, -1.0),
            DVec3::new(1.0, 1.0, -1.0),
            DVec3::new(-1.0, -1.0, 1.0),
            DVec3::new(1.0, -1.0, 1.0),
            DVec3::new(-1.0, 1.0, 1.0),
            DVec3::new(1.0, 1.0, 1.0),
        ];
        let mut out = [DVec3::ZERO; 8];
        for (slot, s) in out.iter_mut().zip(signs.iter()) {
            let local = self.center_body_m + e * (*s);
            *slot = pos_m + quat_rotate_vec_portable(q_body_to_frame, local);
        }
        out
    }
}

/// Exact 15-axis Separating Axis Theorem (`SAT`) signed separation between two 3D OBBs (`m`).
///
/// Returns `<= 0.0` (negative penetration depth along the minimum-overlap axis) when the two OBBs
/// intersect or touch, and a strictly positive separating-plane gap (`> 0.0`) when disjoint.
#[must_use]
pub fn obb_signed_separation_m(
    pos_a: DVec3,
    q_a: DQuat,
    shape_a: &OrientedCollisionShape,
    pos_b: DVec3,
    q_b: DQuat,
    shape_b: &OrientedCollisionShape,
) -> f64 {
    let ca = pos_a + quat_rotate_vec_portable(q_a, shape_a.center_body_m);
    let cb = pos_b + quat_rotate_vec_portable(q_b, shape_b.center_body_m);
    let axes_a = [
        quat_rotate_vec_portable(q_a, DVec3::X),
        quat_rotate_vec_portable(q_a, DVec3::Y),
        quat_rotate_vec_portable(q_a, DVec3::Z),
    ];
    let axes_b = [
        quat_rotate_vec_portable(q_b, DVec3::X),
        quat_rotate_vec_portable(q_b, DVec3::Y),
        quat_rotate_vec_portable(q_b, DVec3::Z),
    ];
    let ea = shape_a.half_extents_body_m.abs().to_array();
    let eb = shape_b.half_extents_body_m.abs().to_array();
    let d = cb - ca;

    let mut max_sep = f64::NEG_INFINITY;
    for axis in axes_a.iter().chain(axes_b.iter()) {
        let sep = axis_gap(d, *axis, &axes_a, &ea, &axes_b, &eb);
        max_sep = max_sep.max(sep);
        if max_sep > 0.0 {
            return max_sep;
        }
    }
    for &ua in &axes_a {
        for &ub in &axes_b {
            let cross = ua.cross(ub);
            let len_sq = cross.length_squared();
            if len_sq > 1.0e-20 {
                let unit = cross / sqrt(len_sq);
                let sep = axis_gap(d, unit, &axes_a, &ea, &axes_b, &eb);
                max_sep = max_sep.max(sep);
                if max_sep > 0.0 {
                    return max_sep;
                }
            }
        }
    }
    let _ = DMat3::IDENTITY;
    max_sep
}

fn axis_gap(
    center_delta: DVec3,
    unit_axis: DVec3,
    axes_a: &[DVec3; 3],
    ea: &[f64; 3],
    axes_b: &[DVec3; 3],
    eb: &[f64; 3],
) -> f64 {
    let mut ra = 0.0_f64;
    let mut rb = 0.0_f64;
    for (u, &e) in axes_a.iter().zip(ea.iter()) {
        ra += e * unit_axis.dot(*u).abs();
    }
    for (u, &e) in axes_b.iter().zip(eb.iter()) {
        rb += e * unit_axis.dot(*u).abs();
    }
    center_delta.dot(unit_axis).abs() - (ra + rb)
}

/// Uncertainty & geometry budget inflating a trajectory polynomial into a conservative reachability tube.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TubeUncertaintyBudget {
    /// Outer rigid-body CG bounding radius $R_{\text{bound}}$ (`m`).
    pub rigid_radius_m: f64,
    /// Certified dense-output polynomial interpolation remainder $\varepsilon_{\text{interp}}$ (`m`).
    pub interp_error_m: f64,
    /// Central-body gravitational parameter $\mu_P$ used for tidal-gradient bound ($\text{m}^3/\text{s}^2$).
    pub primary_mu_m3_s2: f64,
    /// Maximum permitted unmodeled control-acceleration change $a_{\text{ctrl,max}}$ ($\text{m}/\text{s}^2$).
    pub max_control_accel_delta_m_s2: f64,
}

impl TubeUncertaintyBudget {
    /// Total scalar inflation margin (`m`) over sub-interval duration `dt_s` at minimum primary radius `r_min_m`.
    #[must_use]
    pub fn total_margin_m(&self, dt_s: f64, r_min_m: f64) -> f64 {
        let dt = dt_s.max(0.0);
        let r_safe = r_min_m.max(1_000.0);
        let chord_bound = self.rigid_radius_m.max(1.0);
        let tidal_grad = 2.0 * self.primary_mu_m3_s2.max(0.0) / (r_safe * r_safe * r_safe);
        let grav_term = 0.5 * tidal_grad * chord_bound * dt * dt;
        let ctrl_term = 0.5 * self.max_control_accel_delta_m_s2.max(0.0) * dt * dt;
        self.rigid_radius_m.max(0.0) + self.interp_error_m.max(0.0) + grav_term + ctrl_term
    }
}

/// Broad-phase conservative swept volume enclosing a vessel's reachable space over `[start_instant, end_instant]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SweptReachabilityTube {
    /// Vessel owning this reachability tube.
    pub vessel_id: VesselId,
    /// Coordinate origin (`NaifId`) of `aabb_min_m` / `aabb_max_m`.
    pub primary_body: crate::ids::NaifId,
    /// Inclusive start timestamp.
    pub start_instant: SimInstant,
    /// Inclusive end timestamp.
    pub end_instant: SimInstant,
    /// Inflated minimum `ICRF` coordinate corner (`m`).
    pub aabb_min_m: DVec3,
    /// Inflated maximum `ICRF` coordinate corner (`m`).
    pub aabb_max_m: DVec3,
    /// Total scalar inflation margin applied around the degree-7 Bernstein hull (`m`).
    pub margin_m: f64,
}

impl SweptReachabilityTube {
    /// Builds a conservative [`SweptReachabilityTube`] from a degree-7 [`PlantTrajectorySegment`]
    /// using its exact Bernstein position hull plus [`TubeUncertaintyBudget`].
    #[must_use]
    pub fn from_plant_segment(
        vessel_id: VesselId,
        start_instant: SimInstant,
        end_instant: SimInstant,
        seg: &PlantTrajectorySegment,
        budget: &TubeUncertaintyBudget,
    ) -> Self {
        let dense = seg.as_dense();
        let bx = dense.component_bernstein_8(0);
        let by = dense.component_bernstein_8(1);
        let bz = dense.component_bernstein_8(2);

        let mut lo = DVec3::splat(f64::INFINITY);
        let mut hi = DVec3::splat(f64::NEG_INFINITY);
        let mut r_min = f64::INFINITY;
        for ((&px, &py), &pz) in bx.iter().zip(by.iter()).zip(bz.iter()) {
            let p = DVec3::new(px, py, pz);
            lo = lo.min(p);
            hi = hi.max(p);
            r_min = r_min.min(sqrt(p.length_squared()));
        }
        let margin = budget.total_margin_m(seg.duration_s, r_min);
        let pad = DVec3::splat(margin);
        Self {
            vessel_id,
            primary_body: seg.primary_body,
            start_instant,
            end_instant,
            aabb_min_m: lo - pad,
            aabb_max_m: hi + pad,
            margin_m: margin,
        }
    }

    /// Returns `true` if `self` and `other` share common primary coordinates, overlap in time,
    /// and have intersecting conservative 3D envelopes.
    #[must_use]
    pub fn overlaps(&self, other: &Self) -> bool {
        if self.primary_body != other.primary_body {
            return false;
        }
        if self.end_instant < other.start_instant || other.end_instant < self.start_instant {
            return false;
        }
        self.aabb_min_m.x <= other.aabb_max_m.x
            && self.aabb_max_m.x >= other.aabb_min_m.x
            && self.aabb_min_m.y <= other.aabb_max_m.y
            && self.aabb_max_m.y >= other.aabb_min_m.y
            && self.aabb_min_m.z <= other.aabb_max_m.z
            && self.aabb_max_m.z >= other.aabb_min_m.z
    }
}

/// Deterministically ordered connected component of vessels with overlapping reachability envelopes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RendezvousGroup {
    /// Canonical lowest `VesselId` in this group (`leader == *members.first()`).
    pub leader: VesselId,
    /// Sorted member vessels (`len >= 2`).
    pub members: BTreeSet<VesselId>,
}

/// Forms deterministic `RendezvousGroup`s from a set of `SweptReachabilityTube`s via canonical
/// connected-component union-find (independent of input slice permutation!).
#[must_use]
pub fn form_rendezvous_groups(tubes: &[SweptReachabilityTube]) -> Vec<RendezvousGroup> {
    let mut parent: BTreeMap<VesselId, VesselId> = BTreeMap::new();
    for t in tubes {
        parent.entry(t.vessel_id).or_insert(t.vessel_id);
    }
    let mut idx_a: usize = 0;
    while idx_a < tubes.len() {
        let mut idx_b = idx_a.wrapping_add(1);
        while idx_b < tubes.len() {
            if let (Some(ta), Some(tb)) = (tubes.get(idx_a), tubes.get(idx_b))
                && ta.vessel_id != tb.vessel_id
                && ta.overlaps(tb)
            {
                union_vessels(&mut parent, ta.vessel_id, tb.vessel_id);
            }
            idx_b = idx_b.wrapping_add(1);
        }
        idx_a = idx_a.wrapping_add(1);
    }

    let vessels: Vec<VesselId> = parent.keys().copied().collect();
    let mut comps: BTreeMap<VesselId, BTreeSet<VesselId>> = BTreeMap::new();
    for v in vessels {
        let r = find_root(&mut parent, v);
        comps.entry(r).or_default().insert(v);
    }
    comps
        .into_iter()
        .filter(|(_, members)| members.len() >= 2)
        .map(|(leader, members)| RendezvousGroup { leader, members })
        .collect()
}

fn find_root(parent: &mut BTreeMap<VesselId, VesselId>, mut v: VesselId) -> VesselId {
    while let Some(&p) = parent.get(&v) {
        if p == v {
            return v;
        }
        v = p;
    }
    v
}

fn union_vessels(parent: &mut BTreeMap<VesselId, VesselId>, a: VesselId, b: VesselId) {
    let ra = find_root(parent, a);
    let rb = find_root(parent, b);
    if ra != rb {
        let lo = ra.min(rb);
        let hi = ra.max(rb);
        parent.insert(hi, lo);
    }
}

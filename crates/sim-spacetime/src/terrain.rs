//! Planetary surface collision/contact terrain trait (`CollisionTerrain`), elevation bounds
//! (`ElevationSlab`), surface sample (`CollisionGroundSample`), discontinuous cliff/pad patches
//! (`DiscontinuousStepPatch`), and pure ellipsoidal + step-patch terrain (`PatchedEllipsoidTerrain`).

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};

use crate::scalar_math::{cos, sin};

/// Conservative vertical elevation interval `[min_m, max_m]` above the body's
/// reference ellipsoid over a geodetic region.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ElevationSlab {
    /// Guaranteed lower bound on ellipsoidal terrain height in the queried region (`m`).
    pub min_m: f64,
    /// Guaranteed upper bound on ellipsoidal terrain height in the queried region (`m`).
    pub max_m: f64,
}

impl ElevationSlab {
    /// Constructs a valid slab (`min_m <= max_m`).
    #[must_use]
    pub const fn new(a: f64, b: f64) -> Self {
        if a <= b {
            Self { min_m: a, max_m: b }
        } else {
            Self { min_m: b, max_m: a }
        }
    }

    /// Expands `self` to enclose `other`.
    #[must_use]
    pub const fn union(self, other: Self) -> Self {
        Self {
            min_m: self.min_m.min(other.min_m),
            max_m: self.max_m.max(other.max_m),
        }
    }
}

/// Authoritative ground height, unit normals, and contact material parameters at a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CollisionGroundSample {
    /// Terrain height above the body's reference ellipsoid (`m`).
    pub ellipsoidal_height_m: f64,
    /// Upward unit surface normal in local East-North-Up (`ENU`) axes.
    pub normal_enu: DVec3,
    /// Upward unit surface normal in body-fixed (`ECEF` / `IAU_BODY`) axes.
    pub normal_fixed: DVec3,
    /// Static Coulomb friction coefficient $\mu_s$.
    pub mu_static: f64,
    /// Kinetic Coulomb friction coefficient $\mu_k \le \mu_s$.
    pub mu_kinetic: f64,
    /// Normal compliant contact stiffness $k_n$ ($\text{N}/\text{m}$).
    pub normal_stiffness_n_m: f64,
    /// Normal compliant contact damping $c_n$ ($\text{N}\cdot\text{s}/\text{m}$).
    pub normal_damping_n_s_m: f64,
    /// Tangential elastic-bristle stiffness $k_t$ ($\text{N}/\text{m}$).
    pub tangential_stiffness_n_m: f64,
    /// Tangential bristle damping $c_t$ ($\text{N}\cdot\text{s}/\text{m}$).
    pub tangential_damping_n_s_m: f64,
}

/// Authoritative raised/recessed plateau, launch pad, sloped ramp, or vertical cliff step
/// anchored in local tangent East/North coordinates around `(anchor_lat_rad, anchor_lon_rad)`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DiscontinuousStepPatch {
    /// Geodetic latitude of the patch anchor (`rad`).
    pub anchor_lat_rad: f64,
    /// Geodetic longitude of the patch anchor (`rad`).
    pub anchor_lon_rad: f64,
    /// Inclusive minimum east offset from anchor (`m`).
    pub east_min_m: f64,
    /// Inclusive maximum east offset from anchor (`m`).
    pub east_max_m: f64,
    /// Inclusive minimum north offset from anchor (`m`).
    pub north_min_m: f64,
    /// Inclusive maximum north offset from anchor (`m`).
    pub north_max_m: f64,
    /// Ellipsoidal surface height at the patch anchor `(0, 0)` (`m`).
    pub base_height_m: f64,
    /// Constant east slope $\partial h / \partial e$ (`dimensionless`, `tan(theta_e)`).
    pub slope_east: f64,
    /// Constant north slope $\partial h / \partial n$ (`dimensionless`, `tan(theta_n)`).
    pub slope_north: f64,
    /// Static friction coefficient $\mu_s$ on this patch.
    pub mu_static: f64,
    /// Kinetic friction coefficient $\mu_k$ on this patch.
    pub mu_kinetic: f64,
}

impl DiscontinuousStepPatch {
    /// Creates a flat horizontal launch/landing pad or vertical step plateau.
    #[must_use]
    pub const fn flat_pad(
        anchor_lat_rad: f64,
        anchor_lon_rad: f64,
        half_width_m: f64,
        height_m: f64,
    ) -> Self {
        Self {
            anchor_lat_rad,
            anchor_lon_rad,
            east_min_m: -half_width_m,
            east_max_m: half_width_m,
            north_min_m: -half_width_m,
            north_max_m: half_width_m,
            base_height_m: height_m,
            slope_east: 0.0,
            slope_north: 0.0,
            mu_static: 0.85,
            mu_kinetic: 0.65,
        }
    }

    /// Computes local `(east_m, north_m)` offsets of `pos` relative to this patch's anchor.
    #[must_use]
    pub fn local_en(&self, ellipsoid: &BodyEllipsoid, pos: &Geodetic) -> (f64, f64) {
        let (m_rad, pv_rad) = ellipsoid.radii_of_curvature_m(self.anchor_lat_rad);
        let dlon = wrap_lon_diff(pos.lon_rad - self.anchor_lon_rad);
        let east = dlon * pv_rad * cos(self.anchor_lat_rad);
        let north = (pos.lat_rad - self.anchor_lat_rad) * m_rad;
        (east, north)
    }

    /// Evaluates `(height_m, normal_enu)` if `pos` lies inside this patch's horizontal box.
    #[must_use]
    pub fn evaluate_point(
        &self,
        ellipsoid: &BodyEllipsoid,
        pos: &Geodetic,
    ) -> Option<(f64, DVec3)> {
        let (e, n) = self.local_en(ellipsoid, pos);
        if e >= self.east_min_m
            && e <= self.east_max_m
            && n >= self.north_min_m
            && n <= self.north_max_m
        {
            let h = self.base_height_m + self.slope_east * e + self.slope_north * n;
            let n_enu = DVec3::new(-self.slope_east, -self.slope_north, 1.0).normalize();
            Some((h, n_enu))
        } else {
            None
        }
    }

    /// Evaluates the conservative [`ElevationSlab`] and whether the geodetic box is completely
    /// covered by this patch.
    #[must_use]
    pub fn slab_if_overlapping(
        &self,
        ellipsoid: &BodyEllipsoid,
        lat_min_rad: f64,
        lat_max_rad: f64,
        lon_min_rad: f64,
        lon_max_rad: f64,
    ) -> Option<(ElevationSlab, bool)> {
        let (m_rad, pv_rad) = ellipsoid.radii_of_curvature_m(self.anchor_lat_rad);
        let cos_lat = cos(self.anchor_lat_rad).abs().max(1.0e-6);
        let n0 = (lat_min_rad - self.anchor_lat_rad) * m_rad - 1.0e-4;
        let n1 = (lat_max_rad - self.anchor_lat_rad) * m_rad + 1.0e-4;
        if n1 < self.north_min_m || n0 > self.north_max_m {
            return None;
        }
        let half_span = 0.5 * (lon_max_rad - lon_min_rad).abs();
        let mid_lon = 0.5 * (lon_min_rad + lon_max_rad);
        let d_mid = wrap_lon_diff(mid_lon - self.anchor_lon_rad);
        let e0 = (d_mid - half_span) * pv_rad * cos_lat - 1.0e-4;
        let e1 = (d_mid + half_span) * pv_rad * cos_lat + 1.0e-4;
        if half_span < core::f64::consts::PI && (e1 < self.east_min_m || e0 > self.east_max_m) {
            return None;
        }
        let fully_covered = half_span < core::f64::consts::PI
            && e0 >= self.east_min_m
            && e1 <= self.east_max_m
            && n0 >= self.north_min_m
            && n1 <= self.north_max_m;
        let ce0 = e0.clamp(self.east_min_m, self.east_max_m);
        let ce1 = e1.clamp(self.east_min_m, self.east_max_m);
        let cn0 = n0.clamp(self.north_min_m, self.north_max_m);
        let cn1 = n1.clamp(self.north_min_m, self.north_max_m);
        let corners = [
            self.base_height_m + self.slope_east * ce0 + self.slope_north * cn0,
            self.base_height_m + self.slope_east * ce0 + self.slope_north * cn1,
            self.base_height_m + self.slope_east * ce1 + self.slope_north * cn0,
            self.base_height_m + self.slope_east * ce1 + self.slope_north * cn1,
        ];
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for c in corners {
            lo = lo.min(c);
            hi = hi.max(c);
        }
        Some((ElevationSlab::new(lo, hi), fully_covered))
    }
}

/// Wraps a longitude difference into $[-\pi, \pi)$.
#[must_use]
pub fn wrap_lon_diff(dlon: f64) -> f64 {
    let two_pi = core::f64::consts::TAU;
    (dlon + core::f64::consts::PI).rem_euclid(two_pi) - core::f64::consts::PI
}

/// Abstract planetary collision and contact terrain surface.
///
/// Allows `sim-orbital` to resolve continuous impact times and surface contact normals
/// without depending on `sim-scenery`, `flightbag`, or raster image decoders.
pub trait CollisionTerrain: Send + Sync {
    /// Reference ellipsoid of this celestial body.
    fn ellipsoid(&self) -> BodyEllipsoid;

    /// Canonical SHA-256 digest pinning this collision terrain for replay manifests.
    fn content_hash(&self) -> [u8; 32];

    /// Samples exact ground height, unit normals, and contact parameters under `position`.
    fn sample_ground(&self, position: &Geodetic) -> CollisionGroundSample;

    /// Converts body-fixed position `pos_fixed_m` to [`Geodetic`] and samples the ground below it.
    fn sample_fixed(&self, pos_fixed_m: DVec3) -> (Geodetic, CollisionGroundSample) {
        let geo = self.ellipsoid().fixed_to_geodetic(pos_fixed_m);
        let samp = self.sample_ground(&geo);
        (geo, samp)
    }

    /// Returns a conservative `[min_m, max_m]` ellipsoidal elevation slab over
    /// `[lat_min_rad, lat_max_rad] x [lon_min_rad, lon_max_rad]`.
    fn elevation_slab(
        &self,
        lat_min_rad: f64,
        lat_max_rad: f64,
        lon_min_rad: f64,
        lon_max_rad: f64,
    ) -> ElevationSlab;
}

/// Pure ellipsoidal/procedural collision terrain with optional [`DiscontinuousStepPatch`] overlays
/// and zero raster/scenery dependencies.
#[derive(Debug, Clone, PartialEq)]
pub struct PatchedEllipsoidTerrain {
    ellipsoid: BodyEllipsoid,
    base_elevation_m: f64,
    undulation_seed: Option<u64>,
    patches: Vec<DiscontinuousStepPatch>,
    mu_static: f64,
    mu_kinetic: f64,
    normal_stiffness_n_m: f64,
    normal_damping_n_s_m: f64,
    tangential_stiffness_n_m: f64,
    tangential_damping_n_s_m: f64,
    hash: [u8; 32],
}

impl PatchedEllipsoidTerrain {
    /// Constructs a pinned Earth `WGS84` collision terrain with optional pad/cliff patches.
    #[must_use]
    pub fn earth_wgs84(base_elevation_m: f64, patches: Vec<DiscontinuousStepPatch>) -> Self {
        Self::build(
            BodyEllipsoid::WGS84,
            base_elevation_m,
            None,
            patches,
            0.85,
            0.65,
        )
    }

    /// Constructs a pinned Lunar (`MOON_IAU`) collision terrain with optional pad/cliff patches.
    #[must_use]
    pub fn moon_iau(seed: u64, patches: Vec<DiscontinuousStepPatch>) -> Self {
        Self::build(
            BodyEllipsoid::MOON_IAU,
            0.0,
            Some(seed),
            patches,
            0.80,
            0.60,
        )
    }

    fn build(
        ellipsoid: BodyEllipsoid,
        base_elevation_m: f64,
        undulation_seed: Option<u64>,
        patches: Vec<DiscontinuousStepPatch>,
        mu_static: f64,
        mu_kinetic: f64,
    ) -> Self {
        let mut s = Self {
            ellipsoid,
            base_elevation_m,
            undulation_seed,
            patches,
            mu_static,
            mu_kinetic,
            normal_stiffness_n_m: 1.8e5,
            normal_damping_n_s_m: 6.5e4,
            tangential_stiffness_n_m: 1.5e5,
            tangential_damping_n_s_m: 5.5e4,
            hash: [0; 32],
        };
        s.hash = s.compute_sha256();
        s
    }

    fn base_height_at(&self, pos: &Geodetic) -> f64 {
        if let Some(seed) = self.undulation_seed {
            sin(pos.lat_rad * 37.0 + (seed as f64) * 1.0e-6) * cos(pos.lon_rad * 41.0) * 1.5
        } else {
            self.base_elevation_m
        }
    }

    fn compute_sha256(&self) -> [u8; 32] {
        let mut h = Sha256::new();
        h.update(b"sim-spacetime:patched-ellipsoid-terrain:v1");
        if let Ok(json) = serde_json::to_vec(&(
            self.base_elevation_m.to_bits(),
            self.undulation_seed,
            &self.patches,
        )) {
            h.update(&json);
        }
        h.finalize().into()
    }
}

impl CollisionTerrain for PatchedEllipsoidTerrain {
    fn ellipsoid(&self) -> BodyEllipsoid {
        self.ellipsoid
    }

    fn content_hash(&self) -> [u8; 32] {
        self.hash
    }

    fn sample_ground(&self, position: &Geodetic) -> CollisionGroundSample {
        let frame = LocalFrame::at(position);
        let mut best: Option<(&DiscontinuousStepPatch, f64, DVec3)> = None;
        for p in &self.patches {
            if let Some((h, n)) = p.evaluate_point(&self.ellipsoid, position)
                && best.is_none_or(|(_, bh, _)| h >= bh)
            {
                best = Some((p, h, n));
            }
        }
        if let Some((patch, h, n_enu)) = best {
            return CollisionGroundSample {
                ellipsoidal_height_m: h,
                normal_enu: n_enu,
                normal_fixed: (frame.enu_to_ecef() * n_enu).normalize(),
                mu_static: patch.mu_static,
                mu_kinetic: patch.mu_kinetic,
                normal_stiffness_n_m: self.normal_stiffness_n_m,
                normal_damping_n_s_m: self.normal_damping_n_s_m,
                tangential_stiffness_n_m: self.tangential_stiffness_n_m,
                tangential_damping_n_s_m: self.tangential_damping_n_s_m,
            };
        }
        let h = self.base_height_at(position);
        let n_enu = DVec3::Z;
        CollisionGroundSample {
            ellipsoidal_height_m: h,
            normal_enu: n_enu,
            normal_fixed: (frame.enu_to_ecef() * n_enu).normalize(),
            mu_static: self.mu_static,
            mu_kinetic: self.mu_kinetic,
            normal_stiffness_n_m: self.normal_stiffness_n_m,
            normal_damping_n_s_m: self.normal_damping_n_s_m,
            tangential_stiffness_n_m: self.tangential_stiffness_n_m,
            tangential_damping_n_s_m: self.tangential_damping_n_s_m,
        }
    }

    fn elevation_slab(
        &self,
        lat_min_rad: f64,
        lat_max_rad: f64,
        lon_min_rad: f64,
        lon_max_rad: f64,
    ) -> ElevationSlab {
        let base = if self.undulation_seed.is_some() {
            ElevationSlab::new(-2.0, 2.0)
        } else {
            ElevationSlab::new(self.base_elevation_m, self.base_elevation_m)
        };
        let mut any_full_cover = false;
        let mut merged_patches: Option<ElevationSlab> = None;
        for patch in &self.patches {
            if let Some((p_slab, full_cover)) = patch.slab_if_overlapping(
                &self.ellipsoid,
                lat_min_rad,
                lat_max_rad,
                lon_min_rad,
                lon_max_rad,
            ) {
                any_full_cover |= full_cover;
                merged_patches = Some(merged_patches.map_or(p_slab, |acc| acc.union(p_slab)));
            }
        }
        match (merged_patches, any_full_cover) {
            (Some(p_slab), true) => p_slab,
            (Some(p_slab), false) => base.union(p_slab),
            (None, _) => base,
        }
    }
}

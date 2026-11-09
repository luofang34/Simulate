//! Immutable, SHA-256-hashed authoritative collision terrain for orbital,
//! atmospheric, and powered-landing physics (`PinnedCollisionTerrain`).
//!
//! Decouples deterministic physics ground contact and terrain-avoidance root
//! isolation from asynchronous visual tile arrival (`LiveDemBridge`).

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_spacetime::wrap_lon_diff;
pub use sim_spacetime::{
    CollisionGroundSample, CollisionTerrain, DiscontinuousStepPatch, ElevationSlab,
};

use crate::{LunarTranquilityScenery, SceneryError, sources::ElevationSource};

/// Baseline for finite-difference surface slope evaluation away from cliff edges (`m`).
const NORMAL_BASELINE_M: f64 = 0.5;

/// Base planetary relief model for [`PinnedCollisionTerrain`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CollisionTerrainBody {
    /// Earth `WGS84` reference ellipsoid with optional uniform base elevation (`m`).
    EarthWgs84 {
        /// Uniform base ellipsoidal elevation away from surveyed pads (`m`).
        base_elevation_m: f64,
    },
    /// Moon `MOON_IAU` (`1,737.4 km` sphere) combining [`LunarTranquilityScenery`] with
    /// global deterministic mare relief.
    MoonIau {
        /// Seed controlling deterministic global mare undulation outside Tranquility Base.
        seed: u64,
    },
    /// Mars `MARS_IAU` (`a = 3,396,190 m`, `b = 3,376,200 m`) with deterministic cratered
    /// plain relief around `base_elevation_m` (`MOLA` / spheroid offset).
    MarsIau {
        /// Base elevation relative to `MARS_IAU` spheroid (`m`).
        base_elevation_m: f64,
        /// Deterministic procedural relief amplitude (`m`, `>= 0`).
        relief_amplitude_m: f64,
    },
}

/// Immutable, manifest-hashed authoritative collision terrain for one celestial body.
#[derive(Debug, Clone, PartialEq)]
pub struct PinnedCollisionTerrain {
    body: CollisionTerrainBody,
    ellipsoid: BodyEllipsoid,
    lunar_site: Option<LunarTranquilityScenery>,
    patches: Vec<DiscontinuousStepPatch>,
    mu_static: f64,
    mu_kinetic: f64,
    normal_stiffness_n_m: f64,
    normal_damping_n_s_m: f64,
    tangential_stiffness_n_m: f64,
    tangential_damping_n_s_m: f64,
    hash: [u8; 32],
}

impl PinnedCollisionTerrain {
    /// Constructs a pinned Earth `WGS84` collision terrain with optional pad/cliff patches.
    #[must_use]
    pub fn earth_wgs84(base_elevation_m: f64, patches: Vec<DiscontinuousStepPatch>) -> Self {
        Self::build(
            CollisionTerrainBody::EarthWgs84 { base_elevation_m },
            BodyEllipsoid::WGS84,
            None,
            patches,
            0.85,
            0.65,
        )
    }

    /// Constructs a pinned Lunar (`MOON_IAU`) collision terrain including Apollo 11 Tranquility Base.
    #[must_use]
    pub fn moon_iau(seed: u64, patches: Vec<DiscontinuousStepPatch>) -> Self {
        Self::build(
            CollisionTerrainBody::MoonIau { seed },
            BodyEllipsoid::MOON_IAU,
            Some(LunarTranquilityScenery::apollo_11_landing()),
            patches,
            0.80,
            0.60,
        )
    }

    /// Constructs a pinned Martian (`MARS_IAU`) collision terrain with deterministic relief.
    #[must_use]
    pub fn mars_iau(
        base_elevation_m: f64,
        relief_amplitude_m: f64,
        patches: Vec<DiscontinuousStepPatch>,
    ) -> Self {
        Self::build(
            CollisionTerrainBody::MarsIau {
                base_elevation_m,
                relief_amplitude_m: relief_amplitude_m.abs(),
            },
            BodyEllipsoid::MARS_IAU,
            None,
            patches,
            0.78,
            0.58,
        )
    }

    fn build(
        body: CollisionTerrainBody,
        ellipsoid: BodyEllipsoid,
        lunar_site: Option<LunarTranquilityScenery>,
        patches: Vec<DiscontinuousStepPatch>,
        mu_static: f64,
        mu_kinetic: f64,
    ) -> Self {
        let mut s = Self {
            body,
            ellipsoid,
            lunar_site,
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

    /// Overrides compliant contact stiffness and damping coefficients, updating `content_hash`.
    #[must_use]
    pub fn with_compliance(
        mut self,
        normal_stiffness_n_m: f64,
        normal_damping_n_s_m: f64,
        tangential_stiffness_n_m: f64,
        tangential_damping_n_s_m: f64,
    ) -> Self {
        self.normal_stiffness_n_m = normal_stiffness_n_m;
        self.normal_damping_n_s_m = normal_damping_n_s_m;
        self.tangential_stiffness_n_m = tangential_stiffness_n_m;
        self.tangential_damping_n_s_m = tangential_damping_n_s_m;
        self.hash = self.compute_sha256();
        self
    }

    /// Reference ellipsoid of this celestial body.
    #[must_use]
    pub const fn ellipsoid(&self) -> BodyEllipsoid {
        self.ellipsoid
    }

    /// Canonical SHA-256 digest pinning this collision terrain for replay manifests.
    #[must_use]
    pub const fn content_hash(&self) -> [u8; 32] {
        self.hash
    }

    /// Samples exact ground height, unit normals, and contact parameters under `position`.
    #[must_use]
    pub fn sample_ground(&self, position: &Geodetic) -> CollisionGroundSample {
        let frame = LocalFrame::at(position);
        if let Some((patch, h, n_enu)) = self.top_patch_at(position) {
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
        let n_enu = self.finite_diff_normal_enu(position);
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

    /// Converts body-fixed position `pos_fixed_m` to [`Geodetic`] and samples the ground below it.
    #[must_use]
    pub fn sample_fixed(&self, pos_fixed_m: DVec3) -> (Geodetic, CollisionGroundSample) {
        let geo = self.ellipsoid.fixed_to_geodetic(pos_fixed_m);
        let samp = self.sample_ground(&geo);
        (geo, samp)
    }

    /// Returns a **provably conservative** `[min_m, max_m]` ellipsoidal elevation slab enclosing
    /// all continuous relief, craters, boulders, and discontinuous step/cliff patches over the
    /// geodetic box `[lat_min_rad, lat_max_rad] x [lon_min_rad, lon_max_rad]`.
    #[must_use]
    pub fn elevation_slab(
        &self,
        lat_min_rad: f64,
        lat_max_rad: f64,
        lon_min_rad: f64,
        lon_max_rad: f64,
    ) -> ElevationSlab {
        let base = self.base_slab(lat_min_rad, lat_max_rad, lon_min_rad, lon_max_rad);
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

    fn top_patch_at(&self, pos: &Geodetic) -> Option<(&DiscontinuousStepPatch, f64, DVec3)> {
        let mut best: Option<(&DiscontinuousStepPatch, f64, DVec3)> = None;
        for p in &self.patches {
            if let Some((h, n)) = p.evaluate_point(&self.ellipsoid, pos)
                && best.is_none_or(|(_, bh, _)| h >= bh)
            {
                best = Some((p, h, n));
            }
        }
        best
    }

    fn base_height_at(&self, pos: &Geodetic) -> f64 {
        match self.body {
            CollisionTerrainBody::EarthWgs84 { base_elevation_m } => base_elevation_m,
            CollisionTerrainBody::MoonIau { seed } => {
                if let Some(site) = &self.lunar_site
                    && let Ok(Some(h)) = site.ground_height_m(pos)
                {
                    return h;
                }
                ((pos.lat_rad * 37.0 + (seed as f64) * 1.0e-6).sin() * (pos.lon_rad * 41.0).cos())
                    * 1.5
            }
            CollisionTerrainBody::MarsIau {
                base_elevation_m,
                relief_amplitude_m,
            } => {
                let und = (pos.lat_rad * 53.0).sin() * (pos.lon_rad * 47.0).cos() * 0.6
                    + (pos.lat_rad * 211.0 + pos.lon_rad * 197.0).sin() * 0.4;
                base_elevation_m + relief_amplitude_m * und
            }
        }
    }

    fn base_slab(
        &self,
        lat_min_rad: f64,
        lat_max_rad: f64,
        lon_min_rad: f64,
        lon_max_rad: f64,
    ) -> ElevationSlab {
        match self.body {
            CollisionTerrainBody::EarthWgs84 { base_elevation_m } => {
                ElevationSlab::new(base_elevation_m, base_elevation_m)
            }
            CollisionTerrainBody::MoonIau { .. } => {
                let pad = Geodetic::from_degrees(
                    crate::APOLLO_11_SITE_LAT_DEG,
                    crate::APOLLO_11_SITE_LON_DEG,
                    0.0,
                );
                let r = self.ellipsoid.semi_major_m;
                let dlat = (0.5 * (lat_min_rad + lat_max_rad) - pad.lat_rad).abs()
                    - 0.5 * (lat_max_rad - lat_min_rad).abs();
                let dlon = wrap_lon_diff(0.5 * (lon_min_rad + lon_max_rad) - pad.lon_rad).abs()
                    - 0.5 * (lon_max_rad - lon_min_rad).abs();
                if dlat.max(0.0) * r <= 4_200.0 && dlon.max(0.0) * r <= 4_200.0 {
                    ElevationSlab::new(-38.0, 12.5)
                } else {
                    ElevationSlab::new(-2.0, 2.0)
                }
            }
            CollisionTerrainBody::MarsIau {
                base_elevation_m,
                relief_amplitude_m,
            } => ElevationSlab::new(
                base_elevation_m - relief_amplitude_m,
                base_elevation_m + relief_amplitude_m,
            ),
        }
    }

    fn finite_diff_normal_enu(&self, pos: &Geodetic) -> DVec3 {
        let (m_rad, pv_rad) = self.ellipsoid.radii_of_curvature_m(pos.lat_rad);
        let dlat = NORMAL_BASELINE_M / m_rad;
        let cos_lat = pos.lat_rad.cos().abs().max(1.0e-4);
        let dlon = NORMAL_BASELINE_M / (pv_rad * cos_lat);
        let he_p = self.base_height_at(&Geodetic {
            lon_rad: pos.lon_rad + dlon,
            ..*pos
        });
        let he_m = self.base_height_at(&Geodetic {
            lon_rad: pos.lon_rad - dlon,
            ..*pos
        });
        let max_lat = std::f64::consts::FRAC_PI_2 - 1.0e-7;
        let hn_p = self.base_height_at(&Geodetic {
            lat_rad: (pos.lat_rad + dlat).clamp(-max_lat, max_lat),
            ..*pos
        });
        let hn_m = self.base_height_at(&Geodetic {
            lat_rad: (pos.lat_rad - dlat).clamp(-max_lat, max_lat),
            ..*pos
        });
        let se = (he_p - he_m) / (2.0 * NORMAL_BASELINE_M);
        let sn = (hn_p - hn_m) / (2.0 * NORMAL_BASELINE_M);
        DVec3::new(-se, -sn, 1.0).normalize()
    }

    fn compute_sha256(&self) -> [u8; 32] {
        let mut h = Sha256::new();
        h.update(b"sim-scenery:pinned-collision-terrain:v1");
        if let Ok(json) = serde_json::to_vec(&(
            &self.body,
            &self.patches,
            canonical_f64_bits(self.mu_static),
            canonical_f64_bits(self.mu_kinetic),
            canonical_f64_bits(self.normal_stiffness_n_m),
            canonical_f64_bits(self.normal_damping_n_s_m),
            canonical_f64_bits(self.tangential_stiffness_n_m),
            canonical_f64_bits(self.tangential_damping_n_s_m),
        )) {
            h.update(&json);
        }
        h.finalize().into()
    }
}

impl ElevationSource for PinnedCollisionTerrain {
    fn ground_height_m(&self, position: &Geodetic) -> Result<Option<f64>, SceneryError> {
        Ok(Some(self.sample_ground(position).ellipsoidal_height_m))
    }
}

impl CollisionTerrain for PinnedCollisionTerrain {
    fn ellipsoid(&self) -> BodyEllipsoid {
        self.ellipsoid
    }

    fn content_hash(&self) -> [u8; 32] {
        self.hash
    }

    fn sample_ground(&self, position: &Geodetic) -> CollisionGroundSample {
        Self::sample_ground(self, position)
    }

    fn elevation_slab(
        &self,
        lat_min_rad: f64,
        lat_max_rad: f64,
        lon_min_rad: f64,
        lon_max_rad: f64,
    ) -> ElevationSlab {
        Self::elevation_slab(self, lat_min_rad, lat_max_rad, lon_min_rad, lon_max_rad)
    }
}

fn canonical_f64_bits(v: f64) -> u64 {
    if v == 0.0 {
        0
    } else if v.is_nan() {
        0x7ff8_0000_0000_0000
    } else {
        v.to_bits()
    }
}

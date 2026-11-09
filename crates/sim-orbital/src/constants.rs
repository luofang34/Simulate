//! Versioned gravitational parameters, body reference radii, spherical harmonics,
//! and disjoint barycenter-hierarchy validation.
//!
//! Keeps `GM` and harmonic models strictly separated from kinematic SPK segment files.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use sim_geo::BodyEllipsoid;

use crate::{
    hash::{ContentHash, DomainHasher},
    ids::NaifId,
    kpl::{KplError, KplPool},
    scalar_math::sqrt,
};

const KM_TO_M: f64 = 1_000.0;
const KM3_TO_M3: f64 = 1.0e9;

/// Unnormalized lunar second zonal harmonic $J_2$: the GRAIL GRGM1200A value at
/// the lunar reference radius that the simulated world and `Navigate` share.
pub const MOON_J2_UNNORMALIZED: f64 = BodyEllipsoid::MOON_IAU.j2;

/// Why building or validating a [`GravityConstantsCatalog`] failed.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ConstantsError {
    /// Failed to parse a NAIF text constants kernel (`gm_de440.tpc` or `pck00011.tpc`).
    #[error("invalid NAIF constants kernel: {0}")]
    InvalidKpl(#[from] KplError),
    /// Required gravitational parameter `BODY<id>_GM` was missing or non-positive.
    #[error("missing or non-positive gravitational parameter for {0}")]
    MissingGm(NaifId),
    /// Active point-mass attracting set contained duplicate body identifiers.
    #[error("duplicate body {0} in attracting set")]
    DuplicateAttractingBody(NaifId),
    /// Active attracting set included `SSB (0)` as a gravitating mass alongside solar-system bodies.
    #[error("SSB (0) cannot be included as an independent point-mass attractor")]
    SsbInAttractingSet,
    /// Active attracting set double-counted a planetary system barycenter alongside a constituent body.
    #[error(
        "attracting set double-counts system barycenter {barycenter} alongside constituent {child}"
    )]
    OverlappingBarycenterHierarchy {
        /// Planetary system barycenter (`1..=9`).
        barycenter: NaifId,
        /// Constituent planet or satellite (`100..=999`) belonging to `barycenter`.
        child: NaifId,
    },
}

/// Fully normalized spherical-harmonic Stokes coefficient pair $(\bar{C}_{nm}, \bar{S}_{nm})$.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct HarmonicTerm {
    /// Spherical-harmonic degree $n \ge 2$.
    pub degree: u32,
    /// Spherical-harmonic order $0 \le m \le n$.
    pub order: u32,
    /// Fully normalized cosine coefficient $\bar{C}_{nm}$.
    pub c_bar: f64,
    /// Fully normalized sine coefficient $\bar{S}_{nm}$ (`0.0` when $m = 0$).
    pub s_bar: f64,
}

impl HarmonicTerm {
    /// Converts an unnormalized second zonal harmonic $J_2 = -C_{20}$ into normalized
    /// $\bar{C}_{20} = -J_2 / \sqrt{5}$.
    #[must_use]
    pub fn from_unnormalized_j2(j2: f64) -> Self {
        Self {
            degree: 2,
            order: 0,
            c_bar: -j2 / sqrt(5.0),
            s_bar: 0.0,
        }
    }
}

/// Gravitational parameter, reference radii, and optional spherical-harmonic expansion for one body.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BodyGravitySpec {
    /// NAIF body or barycenter identifier.
    pub naif_id: NaifId,
    /// Standard gravitational parameter $\mu = GM$ in $\text{m}^3/\text{s}^2$.
    pub mu_m3_s2: f64,
    /// Reference equatorial radius $R_{\text{eq}}$ in meters (`0.0` for system barycenters).
    pub equatorial_radius_m: f64,
    /// Reference polar radius $R_{\text{polar}}$ in meters (`0.0` for system barycenters).
    pub polar_radius_m: f64,
    /// Fully normalized spherical-harmonic coefficients anchored to this body's fixed frame.
    pub harmonics: Vec<HarmonicTerm>,
}

impl BodyGravitySpec {
    /// Flattening $f = (a - b) / a$, or `0.0` when $a \le 0$.
    #[must_use]
    pub fn flattening(&self) -> f64 {
        if self.equatorial_radius_m > 0.0 {
            (self.equatorial_radius_m - self.polar_radius_m) / self.equatorial_radius_m
        } else {
            0.0
        }
    }
}

/// Versioned gravitational constants and harmonic coefficients catalog.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GravityConstantsCatalog {
    hash: ContentHash,
    bodies: BTreeMap<NaifId, BodyGravitySpec>,
}

impl GravityConstantsCatalog {
    /// Assembles a catalog from explicit [`BodyGravitySpec`] entries and hashes it canonically.
    #[must_use]
    pub fn from_specs(specs: impl IntoIterator<Item = BodyGravitySpec>) -> Self {
        let mut bodies = BTreeMap::new();
        for mut spec in specs {
            spec.harmonics.sort_by_key(|h| (h.degree, h.order));
            bodies.insert(spec.naif_id, spec);
        }
        let hash = hash_specs(&bodies);
        Self { hash, bodies }
    }

    /// Parses all `BODY<id>_GM` values from `gm_de440.tpc` (`km³/s² -> m³/s²`), merges
    /// `BODY<id>_RADII` from `pck_bytes` (`km -> m`), and attaches default normalized $J_2$
    /// harmonics for Earth (`399`), Moon (`301`), and Mars (`499`).
    pub fn from_naif_gm_and_pck_bytes(
        gm_bytes: &[u8],
        pck_bytes: &[u8],
    ) -> Result<Self, ConstantsError> {
        let gm_pool = KplPool::parse_bytes(gm_bytes)?;
        let pck_pool = KplPool::parse_bytes(pck_bytes)?;
        let mut specs: Vec<BodyGravitySpec> = Vec::new();

        for (var_name, _) in gm_pool.iter() {
            let Some(id) = parse_body_gm_var(var_name) else {
                continue;
            };
            let mu_km3_s2 = gm_pool.require_scalar(var_name)?;
            let mu_m3_s2 = mu_km3_s2 * KM3_TO_M3;
            if !mu_m3_s2.is_finite() || mu_m3_s2 < 0.0 {
                return Err(ConstantsError::MissingGm(id));
            }
            if mu_m3_s2 == 0.0 {
                continue;
            }
            let (r_eq_m, r_pol_m) = lookup_radii_m(&pck_pool, id);
            let harmonics = default_harmonics_for(id);
            specs.push(BodyGravitySpec {
                naif_id: id,
                mu_m3_s2,
                equatorial_radius_m: r_eq_m,
                polar_radius_m: r_pol_m,
                harmonics,
            });
        }

        // Ensure core bodies exist before returning.
        let catalog = Self::from_specs(specs);
        for req in [
            NaifId::SUN,
            NaifId::EARTH_MOON_BARYCENTER,
            NaifId::MARS_BARYCENTER,
            NaifId::EARTH,
            NaifId::MOON,
            NaifId::MARS,
        ] {
            if catalog.get(req).is_none() {
                return Err(ConstantsError::MissingGm(req));
            }
        }
        Ok(catalog)
    }

    /// Canonical SHA-256 content hash of all gravitational parameters, radii, and harmonics.
    #[must_use]
    pub const fn content_hash(&self) -> ContentHash {
        self.hash
    }

    /// Looks up the gravity specification for `body`.
    #[must_use]
    pub fn get(&self, body: NaifId) -> Option<&BodyGravitySpec> {
        self.bodies.get(&body)
    }

    /// Returns $\mu = GM > 0$ in $\text{m}^3/\text{s}^2$, failing closed if `body` is absent or zero-mass.
    pub fn require_mu(&self, body: NaifId) -> Result<f64, ConstantsError> {
        self.bodies
            .get(&body)
            .map(|s| s.mu_m3_s2)
            .filter(|&mu| mu.is_finite() && mu > 0.0)
            .ok_or(ConstantsError::MissingGm(body))
    }

    /// Borrowing map over all loaded body specifications.
    #[must_use]
    pub const fn bodies(&self) -> &BTreeMap<NaifId, BodyGravitySpec> {
        &self.bodies
    }

    /// Validates that `attracting_bodies` exist in this catalog, contain no duplicates or `SSB (0)`,
    /// and never double-count a planetary system barycenter (`1..=9`) alongside one of its child bodies
    /// (e.g., `EMB (3)` alongside `Earth (399)` or `Moon (301)`).
    pub fn validate_attracting_hierarchy(
        &self,
        attracting_bodies: &[NaifId],
    ) -> Result<(), ConstantsError> {
        validate_disjoint_hierarchy_ids(attracting_bodies)?;
        for &id in attracting_bodies {
            let _ = self.require_mu(id)?;
        }
        Ok(())
    }
}

/// Validates disjoint barycenter-vs-child mass hierarchy rules on a slice of `NaifId`s.
pub fn validate_disjoint_hierarchy_ids(attracting_bodies: &[NaifId]) -> Result<(), ConstantsError> {
    let mut seen = BTreeSet::new();
    for &id in attracting_bodies {
        if id == NaifId::SSB {
            return Err(ConstantsError::SsbInAttractingSet);
        }
        if !seen.insert(id) {
            return Err(ConstantsError::DuplicateAttractingBody(id));
        }
    }
    for &id in attracting_bodies {
        if let Some(parent_bary) = id.parent_planetary_barycenter()
            && seen.contains(&parent_bary)
        {
            return Err(ConstantsError::OverlappingBarycenterHierarchy {
                barycenter: parent_bary,
                child: id,
            });
        }
    }
    Ok(())
}

fn parse_body_gm_var(var_name: &str) -> Option<NaifId> {
    let inner = var_name.strip_prefix("BODY")?.strip_suffix("_GM")?;
    let code: i32 = inner.parse().ok()?;
    Some(NaifId(code))
}

fn lookup_radii_m(pck_pool: &KplPool, id: NaifId) -> (f64, f64) {
    match id {
        NaifId::EARTH => {
            return (
                BodyEllipsoid::WGS84.semi_major_m,
                BodyEllipsoid::WGS84.semi_minor_m(),
            );
        }
        NaifId::MOON => {
            return (
                BodyEllipsoid::MOON_IAU.semi_major_m,
                BodyEllipsoid::MOON_IAU.semi_minor_m(),
            );
        }
        NaifId::MARS => {
            return (
                BodyEllipsoid::MARS_IAU.semi_major_m,
                BodyEllipsoid::MARS_IAU.semi_minor_m(),
            );
        }
        _ => {}
    }
    let key = format!("BODY{}_RADII", id.0);
    if let Some(vals) = pck_pool.floats(&key) {
        let a = vals.first().copied().unwrap_or(0.0) * KM_TO_M;
        let c = vals.get(2).or_else(|| vals.first()).copied().unwrap_or(0.0) * KM_TO_M;
        (a, c)
    } else {
        (0.0, 0.0)
    }
}

fn default_harmonics_for(id: NaifId) -> Vec<HarmonicTerm> {
    match id {
        NaifId::EARTH => vec![HarmonicTerm::from_unnormalized_j2(BodyEllipsoid::WGS84.j2)],
        NaifId::MOON => vec![HarmonicTerm::from_unnormalized_j2(MOON_J2_UNNORMALIZED)],
        NaifId::MARS => vec![HarmonicTerm::from_unnormalized_j2(
            BodyEllipsoid::MARS_IAU.j2,
        )],
        _ => Vec::new(),
    }
}

fn hash_specs(bodies: &BTreeMap<NaifId, BodyGravitySpec>) -> ContentHash {
    let mut h = DomainHasher::new(b"sim-orbital:gravity-constants:v1");
    h.write_u64(bodies.len() as u64);
    for (id, spec) in bodies {
        h.write_i32(id.0);
        h.write_f64_canonical(spec.mu_m3_s2);
        h.write_f64_canonical(spec.equatorial_radius_m);
        h.write_f64_canonical(spec.polar_radius_m);
        h.write_u64(spec.harmonics.len() as u64);
        for term in &spec.harmonics {
            h.write_u64(u64::from(term.degree));
            h.write_u64(u64::from(term.order));
            h.write_f64_canonical(term.c_bar);
            h.write_f64_canonical(term.s_bar);
        }
    }
    h.finish()
}

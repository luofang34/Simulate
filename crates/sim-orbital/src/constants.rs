//! Versioned gravitational parameters, body reference radii, spherical harmonics,
//! and disjoint barycenter-hierarchy validation.
//!
//! Keeps `GM` and harmonic models strictly separated from kinematic SPK segment files.

use std::{
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
};

use sim_bodies::{BodyCatalog, HarmonicGravity};

use crate::{
    hash::{ContentHash, DomainHasher},
    ids::NaifId,
    kpl::{KplError, KplPool},
};

const KM_TO_M: f64 = 1_000.0;
const KM3_TO_M3: f64 = 1.0e9;

/// Why building or validating a [`GravityConstantsCatalog`] failed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
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
    /// A body's harmonic coefficients do not make a gravity model with its constants.
    #[error("harmonic gravity of {body} was refused")]
    InvalidHarmonics {
        /// The body whose field was being built.
        body: NaifId,
        /// What the model refused.
        #[source]
        source: sim_bodies::FieldBuildError,
    },
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

/// Gravitational parameter, reference radii, and optional spherical-harmonic expansion for one body.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyGravitySpec {
    /// NAIF body or barycenter identifier.
    pub naif_id: NaifId,
    /// Standard gravitational parameter $\mu = GM$ in $\text{m}^3/\text{s}^2$.
    pub mu_m3_s2: f64,
    /// Reference equatorial radius $R_{\text{eq}}$ in meters (`0.0` for system barycenters).
    pub equatorial_radius_m: f64,
    /// Reference polar radius $R_{\text{polar}}$ in meters (`0.0` for system barycenters).
    pub polar_radius_m: f64,
    /// The body's gravitation beyond its central term, anchored to its fixed frame and
    /// stated with its own `GM` and reference radius; `None` for a point mass. The
    /// central term is always [`Self::mu_m3_s2`], so only degrees one and up are read.
    pub harmonics: Option<HarmonicGravity>,
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
#[derive(Clone, Debug)]
pub struct GravityConstantsCatalog {
    hash: ContentHash,
    bodies: BTreeMap<NaifId, BodyGravitySpec>,
    catalog: Arc<BodyCatalog>,
}

impl PartialEq for GravityConstantsCatalog {
    /// Two catalogs are the same when their content is: the hash covers every
    /// constant, coefficient and body descriptor.
    fn eq(&self, other: &Self) -> bool {
        self.hash == other.hash && self.bodies == other.bodies
    }
}

impl GravityConstantsCatalog {
    /// Assembles a catalog from explicit [`BodyGravitySpec`] entries and hashes it
    /// canonically. It describes no body beyond its gravity: see
    /// [`Self::with_body_catalog`].
    #[must_use]
    pub fn from_specs(specs: impl IntoIterator<Item = BodyGravitySpec>) -> Self {
        let mut bodies = BTreeMap::new();
        for spec in specs {
            bodies.insert(spec.naif_id, spec);
        }
        let catalog = Arc::new(BodyCatalog::new());
        let hash = hash_specs(&bodies, &catalog);
        Self {
            hash,
            bodies,
            catalog,
        }
    }

    /// These constants with `catalog` describing the bodies: their spheres of
    /// influence, atmospheres and magnetic fields, which the orbital tier reads
    /// beside their gravity. The gravity specifications stay as given.
    #[must_use]
    pub fn with_body_catalog(mut self, catalog: Arc<BodyCatalog>) -> Self {
        self.hash = hash_specs(&self.bodies, &catalog);
        self.catalog = catalog;
        self
    }

    /// The bodies these constants were built for.
    #[must_use]
    pub const fn body_catalog(&self) -> &Arc<BodyCatalog> {
        &self.catalog
    }

    /// Parses all `BODY<id>_GM` values from `gm_de440.tpc` (`km³/s² -> m³/s²`) and merges
    /// `BODY<id>_RADII` from `pck_bytes` (`km -> m`). A body `catalog` describes takes
    /// its radii and its gravity beyond the central term from its descriptor, the field
    /// a flight-tier host is handed for the same body; every other attractor is a point
    /// mass. The central term is the kernel's `GM`.
    pub fn from_naif_gm_and_pck_bytes(
        gm_bytes: &[u8],
        pck_bytes: &[u8],
        catalog: &Arc<BodyCatalog>,
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
            let (r_eq_m, r_pol_m) = match catalog.get(id) {
                Some(body) => (body.ellipsoid.semi_major_m, body.ellipsoid.semi_minor_m()),
                None => lookup_radii_m(&pck_pool, id),
            };
            let harmonics = catalog.get(id).map(|body| body.gravity().clone());
            specs.push(BodyGravitySpec {
                naif_id: id,
                mu_m3_s2,
                equatorial_radius_m: r_eq_m,
                polar_radius_m: r_pol_m,
                harmonics,
            });
        }

        // The Sun and every described body must attract, with the barycentre each
        // described body's ephemeris chain passes through.
        let constants = Self::from_specs(specs).with_body_catalog(catalog.clone());
        let described = catalog
            .iter()
            .flat_map(|body| [Some(body.naif), body.naif.parent_planetary_barycenter()])
            .flatten();
        for required in std::iter::once(NaifId::SUN).chain(described) {
            if constants.get(required).is_none() {
                return Err(ConstantsError::MissingGm(required));
            }
        }
        Ok(constants)
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
    let key = format!("BODY{}_RADII", id.0);
    if let Some(vals) = pck_pool.floats(&key) {
        let a = vals.first().copied().unwrap_or(0.0) * KM_TO_M;
        let c = vals.get(2).or_else(|| vals.first()).copied().unwrap_or(0.0) * KM_TO_M;
        (a, c)
    } else {
        (0.0, 0.0)
    }
}

fn hash_specs(bodies: &BTreeMap<NaifId, BodyGravitySpec>, catalog: &BodyCatalog) -> ContentHash {
    let mut h = DomainHasher::new(b"sim-orbital:gravity-constants:v2");
    h.write_hash(catalog.content_hash());
    h.write_u64(bodies.len() as u64);
    for (id, spec) in bodies {
        h.write_i32(id.0);
        h.write_f64_canonical(spec.mu_m3_s2);
        h.write_f64_canonical(spec.equatorial_radius_m);
        h.write_f64_canonical(spec.polar_radius_m);
        match &spec.harmonics {
            Some(field) => {
                h.write_u8(1);
                field.write_to_hasher(&mut h);
            }
            None => h.write_u8(0),
        }
    }
    h.finish()
}

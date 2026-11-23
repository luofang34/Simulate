//! The bodies of one run.

use std::collections::BTreeMap;
use std::sync::Arc;

use sim_geo::BodyEllipsoid;
use sim_spacetime::{
    ContentHash, DomainHasher, NaifId, OccultingSphere, SOLAR_IRRADIANCE_1AU_W_M2, SolarFluxModel,
};

use crate::{BodyDescriptor, FieldBuildError};

#[cfg(test)]
mod tests;

/// Every body a run models beyond a point mass, by NAIF identifier.
///
/// Handoffs, the tier policy, sphere-of-influence switching, the Sun's
/// occulters and the orbital force model all look bodies up here, so a body
/// registered once exists for all of them.
#[derive(Debug, Clone)]
pub struct BodyCatalog {
    bodies: BTreeMap<NaifId, Arc<BodyDescriptor>>,
    /// Sunlight past the registered occulters, kept in step with `bodies`.
    sunlight: SolarFluxModel,
}

impl Default for BodyCatalog {
    fn default() -> Self {
        Self::new()
    }
}

impl BodyCatalog {
    /// A catalog with no bodies: every attractor is a point mass.
    #[must_use]
    pub fn new() -> Self {
        Self {
            bodies: BTreeMap::new(),
            sunlight: SolarFluxModel {
                irradiance_1au_w_m2: SOLAR_IRRADIANCE_1AU_W_M2,
                occulters: Vec::new(),
            },
        }
    }

    /// The Earth, the Moon and Mars.
    ///
    /// # Errors
    ///
    /// None for the embedded data; the `Result` is the readers'.
    pub fn standard_solar_system() -> Result<Self, FieldBuildError> {
        Ok(Self::new()
            .with(BodyDescriptor::earth()?)
            .with(BodyDescriptor::moon()?)
            .with(BodyDescriptor::mars()?))
    }

    /// This catalog with `body` registered, replacing any body of the same
    /// identifier.
    #[must_use]
    pub fn with(mut self, body: BodyDescriptor) -> Self {
        self.bodies.insert(body.naif, Arc::new(body));
        self.sunlight.occulters = self
            .iter()
            .filter_map(|body| {
                Some(OccultingSphere {
                    body: body.naif,
                    radius_m: body.occulter_radius_m?,
                })
            })
            .collect();
        self
    }

    /// The body `naif`, if registered.
    #[must_use]
    pub fn get(&self, naif: NaifId) -> Option<&Arc<BodyDescriptor>> {
        self.bodies.get(&naif)
    }

    /// The registered bodies in identifier order.
    pub fn iter(&self) -> impl Iterator<Item = &Arc<BodyDescriptor>> {
        self.bodies.values()
    }

    /// The body whose fixed frame `ellipsoid` describes, or `None` for an
    /// ellipsoid no registered body flies on.
    #[must_use]
    pub fn by_ellipsoid(&self, ellipsoid: &BodyEllipsoid) -> Option<&Arc<BodyDescriptor>> {
        self.iter().find(|body| body.ellipsoid == *ellipsoid)
    }

    /// The identifier of the body whose fixed frame `ellipsoid` describes.
    #[must_use]
    pub fn naif_of(&self, ellipsoid: &BodyEllipsoid) -> Option<NaifId> {
        self.by_ellipsoid(ellipsoid).map(|body| body.naif)
    }

    /// The reference ellipsoid flight dynamics use about `naif`, or `None`
    /// for an origin nothing can fly a body-fixed state about, such as the
    /// Sun or a barycentre.
    #[must_use]
    pub fn ellipsoid_of(&self, naif: NaifId) -> Option<BodyEllipsoid> {
        self.get(naif).map(|body| body.ellipsoid)
    }

    /// The bodies whose sphere of influence lies directly inside `parent`'s,
    /// in identifier order.
    pub fn children_of(&self, parent: NaifId) -> impl Iterator<Item = &Arc<BodyDescriptor>> {
        self.iter().filter(move |body| {
            body.sphere_of_influence
                .is_some_and(|soi| soi.parent == parent)
        })
    }

    /// Nominal sunlight, eclipsed by every registered body that hides the
    /// Sun: the one model radiation pressure, solar arrays and carried
    /// plants read.
    #[must_use]
    pub const fn solar_flux_model(&self) -> &SolarFluxModel {
        &self.sunlight
    }

    /// SHA-256 of what the bodies contribute to a trajectory: shapes, gravity
    /// coefficients and anomaly grids, spheres of influence, tier limits and
    /// occulters by value; magnetic fields and atmospheres by the names their
    /// descriptors give them, which must therefore differ when they do.
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-bodies:catalog:v1");
        h.write_u64(self.bodies.len() as u64);
        for body in self.iter() {
            body.write_to_hasher(&mut h);
        }
        h.finish()
    }
}

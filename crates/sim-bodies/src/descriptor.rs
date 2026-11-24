//! Everything the simulator knows about one body, as data.

use std::sync::Arc;

use sim_atmosphere::{
    AtmosphereContent, AtmosphereField, MarsAtmosphere, VacuumAtmosphere,
    VleoThermosphereAtmosphere,
};
use sim_geo::{BodyEllipsoid, GravityField, MagneticField};
use sim_spacetime::{CollisionTerrain, ContentHash, DomainHasher, NaifId};

use crate::{
    AnomalyOverlay, BodyFields, FieldBuildError, GravityWithAnomaly, HarmonicGravity,
    HarmonicMagnetism, NoMagneticField,
};

/// Where a body's orientation in inertial space comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RotationSource {
    /// A high-precision binary orientation kernel, required: outside its
    /// coverage the body has no orientation.
    PrecisionKernel,
    /// The IAU trigonometric rotation model, exact enough where nothing
    /// better is published.
    IauModel,
}

/// The region about a body inside which a vessel's motion is integrated
/// about that body rather than its parent.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SphereOfInfluence {
    /// Radius of the region, m.
    pub radius_m: f64,
    /// The origin a vessel leaving the region is integrated about.
    pub parent: NaifId,
}

/// The heights about a body between which a vessel keeps the fidelity tier
/// it has, so one skimming a threshold does not flip.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RegimeLimits {
    /// Above this height on the reference ellipsoid, m, flight dynamics have
    /// nothing left to model and orbital propagation takes a vessel.
    pub promote_above_m: f64,
    /// Below this height, m, a vessel needs flight dynamics. Lower than
    /// [`Self::promote_above_m`].
    pub demote_below_m: f64,
}

/// What the air about a catalog body must be: sampled from any thread, and
/// able to state its content for the run's configuration hash.
pub trait BodyAtmosphere: AtmosphereField + AtmosphereContent + Send + Sync {}

impl<T: AtmosphereField + AtmosphereContent + Send + Sync> BodyAtmosphere for T {}

/// The air about a body, shared by everything that samples it.
pub type AtmosphereProvider = Arc<dyn BodyAtmosphere>;

/// One body: its identity, shape, rotation, fields, air, ground and the
/// limits the tiers and the Sun's light read. Every consumer asks the same
/// descriptor, so adding a body is writing one of these.
#[derive(Clone)]
pub struct BodyDescriptor {
    /// The body's NAIF identifier.
    pub naif: NaifId,
    /// Its name, for reports.
    pub name: String,
    /// Its reference ellipsoid, spin rate and reference gravity constants;
    /// the frame flight dynamics fly in.
    pub ellipsoid: BodyEllipsoid,
    /// Where its orientation comes from.
    pub rotation: RotationSource,
    /// The region its gravity dominates, or `None` for a body a vessel is
    /// never integrated about by itself.
    pub sphere_of_influence: Option<SphereOfInfluence>,
    /// The tier limits about it, or `None` where vessels keep their tier.
    pub regime: Option<RegimeLimits>,
    /// Radius of the sphere that hides the Sun, m, or `None` for a body
    /// that casts no shadow worth modelling.
    pub occulter_radius_m: Option<f64>,
    /// Its surface for contact, where one is modelled.
    pub terrain: Option<Arc<dyn CollisionTerrain>>,
    gravity: HarmonicGravity,
    gravity_anomalies: Vec<AnomalyOverlay>,
    gravity_field: Arc<dyn GravityField>,
    magnetic_id: String,
    magnetic_field: Arc<dyn MagneticField>,
    atmosphere_id: String,
    atmosphere: AtmosphereProvider,
}

impl core::fmt::Debug for BodyDescriptor {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("BodyDescriptor")
            .field("naif", &self.naif)
            .field("name", &self.name)
            .field("ellipsoid", &self.ellipsoid)
            .field("rotation", &self.rotation)
            .field("gravity", &self.gravity)
            .field("magnetic", &self.magnetic_id)
            .field("atmosphere", &self.atmosphere_id)
            .field("terrain", &self.terrain.is_some())
            .field("sphere_of_influence", &self.sphere_of_influence)
            .field("regime", &self.regime)
            .field("occulter_radius_m", &self.occulter_radius_m)
            .finish()
    }
}

impl BodyDescriptor {
    /// A body of `ellipsoid` with its reference gravity (point mass and
    /// `J2`), the IAU rotation model, no magnetic field, no air, no terrain,
    /// no sphere of influence, no tier limits, and its mean sphere as the
    /// Sun's occulter.
    ///
    /// # Errors
    ///
    /// [`FieldBuildError`] for an ellipsoid whose constants make no field.
    pub fn new(
        naif: NaifId,
        name: impl Into<String>,
        ellipsoid: BodyEllipsoid,
    ) -> Result<Self, FieldBuildError> {
        let gravity = HarmonicGravity::reference(&ellipsoid)?;
        Ok(Self {
            naif,
            name: name.into(),
            ellipsoid,
            rotation: RotationSource::IauModel,
            sphere_of_influence: None,
            regime: None,
            occulter_radius_m: Some(ellipsoid.mean_radius_m()),
            terrain: None,
            gravity_field: Arc::new(gravity.clone()),
            gravity,
            gravity_anomalies: Vec::new(),
            magnetic_id: "none".to_owned(),
            magnetic_field: Arc::new(NoMagneticField),
            atmosphere_id: "vacuum".to_owned(),
            atmosphere: Arc::new(VacuumAtmosphere),
        })
    }

    /// The Earth: WGS84, ITRF93 orientation, `J2` reference gravity, IGRF-14,
    /// and the standard atmosphere from the ground up through a thermosphere
    /// at moderate solar activity, so a low orbit decays.
    ///
    /// # Errors
    ///
    /// None for the embedded data; the `Result` is the readers'.
    pub fn earth() -> Result<Self, FieldBuildError> {
        Ok(Self::new(NaifId::EARTH, "Earth", BodyEllipsoid::WGS84)?
            .with_rotation(RotationSource::PrecisionKernel)
            .with_magnetic_field("igrf-14", Arc::new(HarmonicMagnetism::igrf14()?))
            .with_atmosphere(
                "ussa76-bates-walker-f107-140",
                Arc::new(VleoThermosphereAtmosphere::new()),
            )
            .with_sphere_of_influence(924_600_000.0, NaifId::SSB)
            .with_regime(160_000.0, 140_000.0)
            .with_occulter_radius_m(6_371_008.8))
    }

    /// The Moon: the IAU mean sphere in the mean-Earth frame, `J2` reference
    /// gravity, no global field, no air. Its limits clear its highest
    /// terrain.
    ///
    /// # Errors
    ///
    /// As [`Self::new`].
    pub fn moon() -> Result<Self, FieldBuildError> {
        Ok(Self::new(NaifId::MOON, "Moon", BodyEllipsoid::MOON_IAU)?
            .with_rotation(RotationSource::PrecisionKernel)
            .with_sphere_of_influence(66_183_000.0, NaifId::EARTH)
            .with_regime(20_000.0, 15_000.0)
            .with_occulter_radius_m(1_737_400.0))
    }

    /// Mars: the IAU spheroid and rotation model, `J2` reference gravity, no
    /// global field, and its carbon-dioxide atmosphere.
    ///
    /// # Errors
    ///
    /// As [`Self::new`].
    pub fn mars() -> Result<Self, FieldBuildError> {
        Ok(Self::new(NaifId::MARS, "Mars", BodyEllipsoid::MARS_IAU)?
            .with_atmosphere("mars-co2", Arc::new(MarsAtmosphere::new()))
            .with_sphere_of_influence(577_200_000.0, NaifId::SSB)
            .with_regime(130_000.0, 110_000.0)
            .with_occulter_radius_m(3_389_500.0))
    }

    /// This body with its orientation from `rotation`.
    #[must_use]
    pub fn with_rotation(mut self, rotation: RotationSource) -> Self {
        self.rotation = rotation;
        self
    }

    /// This body with `gravity` as its global field: flight dynamics and
    /// sensors read it whole, with any surveyed anomalies; orbital propagation
    /// reads its terms beyond the central one.
    #[must_use]
    pub fn with_gravity(mut self, gravity: HarmonicGravity) -> Self {
        self.gravity = gravity;
        self.rebuilt_gravity_field()
    }

    /// This body with surveyed local anomalies, m/s², added to what flight
    /// dynamics and sensors read near the ground. Orbital propagation keeps
    /// the global model: an anomaly grid is not continued upward.
    #[must_use]
    pub fn with_gravity_anomalies(mut self, overlays: Vec<AnomalyOverlay>) -> Self {
        self.gravity_anomalies = overlays;
        self.rebuilt_gravity_field()
    }

    /// The one field instance every consumer shares, from the global model
    /// and the anomalies as they now stand.
    fn rebuilt_gravity_field(mut self) -> Self {
        let global: Arc<dyn GravityField> = Arc::new(self.gravity.clone());
        self.gravity_field = match self.gravity_anomalies.is_empty() {
            true => global,
            false => Arc::new(GravityWithAnomaly {
                base: global,
                overlays: self.gravity_anomalies.clone(),
            }),
        };
        self
    }

    /// This body with the magnetic field `field`, called `id` in reports.
    /// The catalog's hash covers the field's content, not the name.
    #[must_use]
    pub fn with_magnetic_field(
        mut self,
        id: impl Into<String>,
        field: Arc<dyn MagneticField>,
    ) -> Self {
        self.magnetic_id = id.into();
        self.magnetic_field = field;
        self
    }

    /// This body with the atmosphere `atmosphere`, called `id` in reports.
    /// The catalog's hash covers the atmosphere's content, not the name.
    #[must_use]
    pub fn with_atmosphere(
        mut self,
        id: impl Into<String>,
        atmosphere: AtmosphereProvider,
    ) -> Self {
        self.atmosphere_id = id.into();
        self.atmosphere = atmosphere;
        self
    }

    /// This body with `terrain` as its contact surface.
    #[must_use]
    pub fn with_terrain(mut self, terrain: Arc<dyn CollisionTerrain>) -> Self {
        self.terrain = Some(terrain);
        self
    }

    /// This body dominating out to `radius_m`, inside `parent`'s region.
    #[must_use]
    pub fn with_sphere_of_influence(mut self, radius_m: f64, parent: NaifId) -> Self {
        self.sphere_of_influence = Some(SphereOfInfluence { radius_m, parent });
        self
    }

    /// This body with tier limits at `promote_above_m` and `demote_below_m`.
    #[must_use]
    pub fn with_regime(mut self, promote_above_m: f64, demote_below_m: f64) -> Self {
        self.regime = Some(RegimeLimits {
            promote_above_m,
            demote_below_m,
        });
        self
    }

    /// This body hiding the Sun as a sphere of `radius_m`.
    #[must_use]
    pub fn with_occulter_radius_m(mut self, radius_m: f64) -> Self {
        self.occulter_radius_m = Some(radius_m);
        self
    }

    /// The global gravity model: what orbital propagation sums and what the
    /// run's manifest hashes.
    #[must_use]
    pub fn gravity(&self) -> &HarmonicGravity {
        &self.gravity
    }

    /// The truth gravity of this body, the one instance its dynamics and
    /// every inertial or gravimetric sensor share.
    #[must_use]
    pub fn gravity_field(&self) -> &Arc<dyn GravityField> {
        &self.gravity_field
    }

    /// The truth magnetic field of this body, the one instance every
    /// magnetometer and magnetic torque share.
    #[must_use]
    pub fn magnetic_field(&self) -> &Arc<dyn MagneticField> {
        &self.magnetic_field
    }

    /// Both truth fields, for a host that builds a vehicle's dynamics and
    /// its devices on this body.
    #[must_use]
    pub fn fields(&self) -> BodyFields {
        BodyFields {
            gravity: Arc::clone(&self.gravity_field),
            magnetic: Arc::clone(&self.magnetic_field),
        }
    }

    /// The air about this body; vacuum where it has none.
    #[must_use]
    pub fn atmosphere(&self) -> &AtmosphereProvider {
        &self.atmosphere
    }

    pub(crate) fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_i32(self.naif.0);
        let e = self.ellipsoid;
        for value in [
            e.semi_major_m,
            e.flattening,
            e.mu_m3_s2,
            e.omega_rad_s,
            e.j2,
        ] {
            h.write_f64_canonical(value);
        }
        h.write_u8(match self.rotation {
            RotationSource::PrecisionKernel => 1,
            RotationSource::IauModel => 2,
        });
        self.gravity.write_to_hasher(h);
        h.write_u64(self.gravity_anomalies.len() as u64);
        for overlay in &self.gravity_anomalies {
            overlay.write_to_hasher(h);
        }
        h.write_hash(content_hash(b"sim-bodies:magnetic-content:v1", |f| {
            self.magnetic_field.visit_content(f);
        }));
        h.write_hash(content_hash(b"sim-bodies:atmosphere-content:v1", |f| {
            self.atmosphere.visit_content(f);
        }));
        h.write_u8(u8::from(self.terrain.is_some()));
        h.write_opt_f64_canonical(self.sphere_of_influence.map(|soi| soi.radius_m));
        h.write_i32(
            self.sphere_of_influence
                .map_or(i32::MIN, |soi| soi.parent.0),
        );
        h.write_opt_f64_canonical(self.regime.map(|regime| regime.promote_above_m));
        h.write_opt_f64_canonical(self.regime.map(|regime| regime.demote_below_m));
        h.write_opt_f64_canonical(self.occulter_radius_m);
    }
}

/// The digest of a provider's content feed under `domain`. The feed's
/// slices are hashed unframed, so only their concatenation counts.
fn content_hash(domain: &[u8], feed: impl FnOnce(&mut dyn FnMut(&[u8]))) -> ContentHash {
    let mut h = DomainHasher::new(domain);
    feed(&mut |bytes| h.write_unframed(bytes));
    h.finish()
}

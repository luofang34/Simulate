//! Loads a vessel's surroundings put on it in the orbital tier.
//!
//! Gravity moves every vessel the same way; sunlight, thin air, the gravity gradient
//! and the body's magnetic field act on each according to what it is. A vessel carries a
//! set of [`EnvironmentLoadSpec`]s in its checkpoint, covered by the checkpoint's hash,
//! and [`EnvironmentLaw`] sums them at every integrator stage from the bodies the force
//! model describes. A contributor is immutable data and a pure function of the stage, so
//! a proposal is the same bits on whichever worker computes it.

mod drag;
mod gravity_gradient;
mod magnetic;
mod srp;

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};
use sim_bodies::{BodyCatalog, BodyDescriptor};
use sim_spacetime::{BodyRotationState, SolarFluxModel};

use crate::{
    ephemeris::EphemerisCatalog,
    gravity::{ForceModel, GravityError},
    hash::DomainHasher,
    orientation::BodyOrientationCatalog,
    plant::{
        ContinuousControlLaw, ContinuousPlantState, MassPropertiesModel, PlantStageWrench,
        quat_normalize_portable,
    },
    time::TdbEpoch,
};

pub use drag::DragSurface;
pub use gravity_gradient::GravityGradient;
pub use magnetic::ResidualDipole;
pub use srp::SrpSurface;

/// What every contributor of one propagation reads besides the vessel.
#[derive(Clone, Copy)]
pub struct EnvironmentContext<'a> {
    /// The bodies: their atmospheres, gravity and magnetic fields.
    pub bodies: &'a BodyCatalog,
    /// The Sun and what eclipses it.
    pub sunlight: &'a SolarFluxModel,
    /// Body positions; the same catalog the propagation integrates in.
    pub ephemeris: &'a dyn EphemerisCatalog,
    /// Body orientations; the same catalog the propagation integrates in.
    pub orientation: &'a dyn BodyOrientationCatalog,
    /// The mass and inertia the propagation integrates the vessel with.
    pub mass_model: &'a MassPropertiesModel,
}

impl<'a> EnvironmentContext<'a> {
    /// The context of a propagation in `force_model`'s bodies and their sunlight.
    #[must_use]
    pub fn of(
        force_model: &'a ForceModel,
        ephemeris: &'a dyn EphemerisCatalog,
        orientation: &'a dyn BodyOrientationCatalog,
        mass_model: &'a MassPropertiesModel,
    ) -> Self {
        let bodies: &BodyCatalog = force_model.body_catalog();
        Self {
            bodies,
            sunlight: bodies.solar_flux_model(),
            ephemeris,
            orientation,
            mass_model,
        }
    }
}

/// A vessel in the fixed frame of the body it is integrated about.
#[derive(Clone, Copy, Debug)]
pub struct BodyFixedState<'a> {
    /// The body's catalog entry: its fields, its air, its shape.
    pub body: &'a BodyDescriptor,
    /// The body's orientation and spin at the stage's epoch.
    pub rotation: BodyRotationState,
    /// Position in the body's fixed axes, m.
    pub position_fixed_m: DVec3,
    /// Velocity relative to the body's rotating frame, in its fixed axes, m/s.
    pub velocity_fixed_m_s: DVec3,
    /// Rotation taking the body's fixed axes to the vessel's body axes.
    pub fixed_to_vessel: DMat3,
}

/// One integrator stage as an environment-load contributor sees it.
#[derive(Clone, Copy)]
pub struct EnvironmentStage<'a> {
    /// The stage's epoch.
    pub epoch: TdbEpoch,
    /// The vessel's inertial state about its origin.
    pub state: &'a ContinuousPlantState,
    /// The vessel's mass at this stage, kg.
    pub mass_kg: f64,
    /// Its inertia about the centre of mass, body axes, kg·m².
    pub inertia_body: DMat3,
    /// The vessel in its origin's fixed frame, or `None` about an origin the catalog
    /// does not describe (a barycentre): a load that needs a body is then zero.
    pub about: Option<BodyFixedState<'a>>,
    /// The bodies, the Sun and the catalogs of this propagation.
    pub context: &'a EnvironmentContext<'a>,
}

/// A force and a torque about the centre of mass, in the vessel's body axes.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EnvironmentWrench {
    /// Force, N.
    pub force_body_n: DVec3,
    /// Torque about the centre of mass, N·m.
    pub torque_body_nm: DVec3,
}

/// One environment load on a vessel: immutable, and a pure function of the stage.
pub trait EnvironmentLoad: Send + Sync {
    /// Whether the load reads [`EnvironmentStage::about`]. The body-fixed state costs a
    /// body orientation per stage, found only for a set that has such a load.
    fn reads_body(&self) -> bool;

    /// The load at `stage`.
    ///
    /// # Errors
    ///
    /// [`GravityError`] when an ephemeris, orientation or field it reads has no value.
    fn wrench(&self, stage: &EnvironmentStage<'_>) -> Result<EnvironmentWrench, GravityError>;

    /// Writes everything that shapes the load.
    fn write_to_hasher(&self, h: &mut DomainHasher);
}

/// An environment load a vessel carries, as data a checkpoint stores.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum EnvironmentLoadSpec {
    /// Solar radiation pressure on a cannonball.
    SolarRadiationPressure(SrpSurface),
    /// Drag in the origin body's atmosphere.
    Drag(DragSurface),
    /// Gravity-gradient torque from the origin body's field.
    GravityGradient(GravityGradient),
    /// Torque of a residual magnetic dipole in the origin body's field.
    MagneticDipole(ResidualDipole),
}

impl EnvironmentLoadSpec {
    /// The contributor this specification names.
    #[must_use]
    pub fn load(&self) -> &dyn EnvironmentLoad {
        match self {
            Self::SolarRadiationPressure(load) => load,
            Self::Drag(load) => load,
            Self::GravityGradient(load) => load,
            Self::MagneticDipole(load) => load,
        }
    }

    const fn tag(&self) -> u8 {
        match self {
            Self::SolarRadiationPressure(_) => 1,
            Self::Drag(_) => 2,
            Self::GravityGradient(_) => 3,
            Self::MagneticDipole(_) => 4,
        }
    }
}

/// The environment loads on one vessel, summed in the order they were added.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct EnvironmentLoads {
    loads: Vec<EnvironmentLoadSpec>,
}

impl EnvironmentLoads {
    /// These loads and `load`.
    #[must_use]
    pub fn with(mut self, load: EnvironmentLoadSpec) -> Self {
        self.loads.push(load);
        self
    }

    /// The loads, in summing order.
    #[must_use]
    pub fn specs(&self) -> &[EnvironmentLoadSpec] {
        &self.loads
    }

    /// Whether the vessel feels nothing but gravity.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.loads.is_empty()
    }

    /// The first radiation-pressure surface among the loads.
    #[must_use]
    pub fn srp_surface(&self) -> Option<SrpSurface> {
        self.loads.iter().find_map(|load| match load {
            EnvironmentLoadSpec::SolarRadiationPressure(surface) => Some(*surface),
            _ => None,
        })
    }

    pub(crate) fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_u64(self.loads.len() as u64);
        for load in &self.loads {
            h.write_u8(load.tag());
            load.load().write_to_hasher(h);
        }
    }
}

impl FromIterator<EnvironmentLoadSpec> for EnvironmentLoads {
    fn from_iter<I: IntoIterator<Item = EnvironmentLoadSpec>>(iter: I) -> Self {
        Self {
            loads: iter.into_iter().collect(),
        }
    }
}

/// Another control law with a vessel's environment loads added to what it produces.
pub struct EnvironmentLaw<'a> {
    /// The law that flies the vessel.
    pub inner: &'a dyn ContinuousControlLaw,
    /// The loads its surroundings put on it.
    pub loads: &'a EnvironmentLoads,
    /// What the loads read.
    pub context: EnvironmentContext<'a>,
}

impl EnvironmentLaw<'_> {
    /// The vessel about its origin body, when a load asks and the catalog describes it.
    fn about(
        &self,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> Result<Option<BodyFixedState<'_>>, GravityError> {
        let reads_body = self.loads.specs().iter().any(|l| l.load().reads_body());
        let Some(body) = self.context.bodies.get(state.primary_body) else {
            return Ok(None);
        };
        if !reads_body {
            return Ok(None);
        }
        let rotation = self
            .context
            .orientation
            .body_rotation(state.primary_body, epoch)?;
        let position_fixed_m = rotation.r_icrf_to_fixed * state.pos_rel_icrf_m;
        let vessel_from_icrf =
            DMat3::from_quat(quat_normalize_portable(state.q_body_to_icrf).conjugate());
        Ok(Some(BodyFixedState {
            body,
            rotation,
            position_fixed_m,
            velocity_fixed_m_s: rotation.r_icrf_to_fixed * state.vel_rel_icrf_m_s
                - rotation.omega_body_rad_s.cross(position_fixed_m),
            fixed_to_vessel: vessel_from_icrf * rotation.r_icrf_to_fixed.transpose(),
        }))
    }

    /// Adds every load to `wrench`. A load that is exactly zero leaves the wrench's
    /// bits alone, so a vessel keeps the trajectory it has without that load.
    fn add_to(
        &self,
        wrench: &mut PlantStageWrench,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> Result<(), GravityError> {
        let mass_model = self.context.mass_model;
        let stage = EnvironmentStage {
            epoch,
            state,
            mass_kg: mass_model.total_mass_kg(state.propellant_mass_kg),
            inertia_body: mass_model.inertia_tensor(state.propellant_mass_kg),
            about: self.about(epoch, state)?,
            context: &self.context,
        };
        for load in self.loads.specs() {
            let added = load.load().wrench(&stage)?;
            if added.force_body_n != DVec3::ZERO {
                wrench.env_force_body_n += added.force_body_n;
            }
            if added.torque_body_nm != DVec3::ZERO {
                wrench.ext_torque_body_nm += added.torque_body_nm;
            }
        }
        Ok(())
    }
}

impl ContinuousControlLaw for EnvironmentLaw<'_> {
    /// Where a load cannot be evaluated the loads are left out; the checked form
    /// reports that instead, and is the one propagation calls.
    fn evaluate_stage(
        &self,
        tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> PlantStageWrench {
        let bare = self.inner.evaluate_stage(tau_s, epoch, state);
        let mut wrench = bare;
        match self.add_to(&mut wrench, epoch, state) {
            Ok(()) => wrench,
            Err(_) => bare,
        }
    }

    fn evaluate_stage_checked(
        &self,
        tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> Result<PlantStageWrench, GravityError> {
        let mut wrench = self.inner.evaluate_stage_checked(tau_s, epoch, state)?;
        self.add_to(&mut wrench, epoch, state)?;
        Ok(wrench)
    }
}

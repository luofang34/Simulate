//! Loads and mass properties a vessel's owner outside the orbital tier holds on it.
//!
//! A vessel whose internals are integrated elsewhere (a carried plant with its own
//! tanks, power bus and thrusters) reports the force and torque it generates and the
//! mass it has at an instant, and the orbital tier holds them constant until the
//! owner reports again. A magnetic dipole the owner commands is held as the moment,
//! not as a torque: the field it acts in turns with the vessel and changes along the
//! orbit faster than an owner reports, so the orbital tier works the torque out from
//! the held moment at every integrator stage. They live beside the guidance law rather than in it, so a
//! mission plan's burn and the owner's loads act together.
//!
//! The mass model integrates about the centre of mass and has no offset for it, so an
//! owner whose centre of mass moves in body axes reports its force and torque about
//! the centre of mass at each instant; the orbital tier does not model the shift.

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    gravity::GravityError,
    hash::DomainHasher,
    plant::{ContinuousControlLaw, ContinuousPlantState, MassPropertiesModel, PlantStageWrench},
    time::TdbEpoch,
};

/// Mass and inertia an external owner holds on a vessel.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExternalMassProperties {
    /// Total mass outside any propellant the orbital tier integrates itself, kg.
    pub mass_kg: f64,
    /// Inertia tensor about the centre of mass in body axes, as columns, kg·m².
    pub inertia_cg_body_cols: [[f64; 3]; 3],
}

impl ExternalMassProperties {
    /// Mass `mass_kg` with inertia `inertia_cg_body` about the centre of mass.
    #[must_use]
    pub fn new(mass_kg: f64, inertia_cg_body: DMat3) -> Self {
        Self {
            mass_kg,
            inertia_cg_body_cols: inertia_cg_body.to_cols_array_2d(),
        }
    }

    fn is_valid(&self) -> bool {
        self.mass_kg.is_finite()
            && self.mass_kg > 0.0
            && self
                .inertia_cg_body_cols
                .iter()
                .flatten()
                .all(|v| v.is_finite())
    }
}

/// The force, torque and mass an external owner holds on a vessel until it reports again.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ExternalLoads {
    /// Force the vessel generates on itself, body axes, N.
    pub force_body_n: [f64; 3],
    /// Torque about the centre of mass, body axes, N·m.
    pub torque_body_nm: [f64; 3],
    /// Magnetic dipole moment the vessel's torquers hold, body axes, A·m². Its torque
    /// in the origin body's field is evaluated at every integrator stage.
    #[serde(default)]
    pub dipole_body_a_m2: [f64; 3],
    /// Mass properties replacing the configured dry mass and dry inertia, or `None` to
    /// fly the vessel's configured mass model.
    pub mass: Option<ExternalMassProperties>,
}

impl ExternalLoads {
    /// `force_body_n` and `torque_body_nm` with the vessel's own mass `mass`.
    #[must_use]
    pub fn new(force_body_n: DVec3, torque_body_nm: DVec3, mass: ExternalMassProperties) -> Self {
        Self {
            force_body_n: force_body_n.to_array(),
            torque_body_nm: torque_body_nm.to_array(),
            dipole_body_a_m2: [0.0; 3],
            mass: Some(mass),
        }
    }

    /// These loads with the magnetic dipole `moment_body_a_m2` held by the vessel's
    /// torquers.
    #[must_use]
    pub fn with_dipole(mut self, moment_body_a_m2: DVec3) -> Self {
        self.dipole_body_a_m2 = moment_body_a_m2.to_array();
        self
    }

    /// The held dipole moment, body axes, A·m².
    #[must_use]
    pub fn dipole_body_a_m2(&self) -> DVec3 {
        DVec3::from_array(self.dipole_body_a_m2)
    }

    /// Whether every component is finite and any mass positive.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.force_body_n.iter().all(|v| v.is_finite())
            && self.torque_body_nm.iter().all(|v| v.is_finite())
            && self.dipole_body_a_m2.iter().all(|v| v.is_finite())
            && self.mass.is_none_or(|mass| mass.is_valid())
    }

    /// Whether `other` holds the same bits, so replacing one with the other changes no
    /// future step. `PartialEq` would equate `-0.0` with `0.0`.
    #[must_use]
    pub fn same_bits(&self, other: &Self) -> bool {
        self.bit_pattern() == other.bit_pattern()
    }

    fn bit_pattern(&self) -> Vec<u64> {
        let mut bits: Vec<u64> = self
            .force_body_n
            .iter()
            .chain(&self.torque_body_nm)
            .chain(&self.dipole_body_a_m2)
            .map(|v| v.to_bits())
            .collect();
        if let Some(mass) = self.mass {
            bits.push(1);
            bits.push(mass.mass_kg.to_bits());
            bits.extend(
                mass.inertia_cg_body_cols
                    .iter()
                    .flatten()
                    .map(|v| v.to_bits()),
            );
        }
        bits
    }

    pub(crate) fn write_to_hasher(&self, h: &mut DomainHasher) {
        let held = self
            .force_body_n
            .iter()
            .chain(&self.torque_body_nm)
            .chain(&self.dipole_body_a_m2);
        for &v in held {
            h.write_f64_canonical(v);
        }
        match self.mass {
            Some(mass) => {
                h.write_u8(1);
                h.write_f64_canonical(mass.mass_kg);
                for &v in mass.inertia_cg_body_cols.iter().flatten() {
                    h.write_f64_canonical(v);
                }
            }
            None => h.write_u8(0),
        }
    }

    fn has_wrench(&self) -> bool {
        self.force_body_n
            .iter()
            .chain(&self.torque_body_nm)
            .any(|&v| v != 0.0)
    }
}

/// Externally owned loads with the count of times their owner has replaced them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ExternalInputs {
    /// The loads held from the last time they were set.
    pub loads: ExternalLoads,
    /// Incremented each time [`crate::OrbitalUniverse::set_external_loads`] changes the loads.
    pub generation: u64,
}

impl ExternalInputs {
    /// `base` with the external mass, when held, as its dry mass and dry inertia. Propellant
    /// the orbital tier integrates for the guidance law keeps its configured contribution.
    #[must_use]
    pub fn mass_model(&self, base: &MassPropertiesModel) -> MassPropertiesModel {
        match self.loads.mass {
            Some(mass) => MassPropertiesModel {
                dry_mass_kg: mass.mass_kg,
                dry_inertia_cols: mass.inertia_cg_body_cols,
                ..*base
            },
            None => *base,
        }
    }
}

/// Another control law with an external owner's force and torque added to what it produces.
pub(crate) struct ExternalWrenchLaw<'a> {
    pub(crate) inner: &'a dyn ContinuousControlLaw,
    pub(crate) loads: &'a ExternalLoads,
}

impl ExternalWrenchLaw<'_> {
    /// Leaves the wrench untouched when nothing is held, so a vessel with no external owner
    /// keeps the exact bits it would have without this law.
    fn add_to(&self, wrench: &mut PlantStageWrench) {
        if self.loads.has_wrench() {
            wrench.env_force_body_n += DVec3::from_array(self.loads.force_body_n);
            wrench.ext_torque_body_nm += DVec3::from_array(self.loads.torque_body_nm);
        }
    }
}

impl ContinuousControlLaw for ExternalWrenchLaw<'_> {
    fn evaluate_stage(
        &self,
        tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> PlantStageWrench {
        let mut wrench = self.inner.evaluate_stage(tau_s, epoch, state);
        self.add_to(&mut wrench);
        wrench
    }

    fn evaluate_stage_checked(
        &self,
        tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> Result<PlantStageWrench, GravityError> {
        let mut wrench = self.inner.evaluate_stage_checked(tau_s, epoch, state)?;
        self.add_to(&mut wrench);
        Ok(wrench)
    }
}

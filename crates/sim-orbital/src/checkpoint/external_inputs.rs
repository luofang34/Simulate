//! Loads and mass properties a vessel's owner outside the orbital tier holds on it.
//!
//! A vessel whose internals are integrated elsewhere (a carried plant with its own
//! tanks, power bus and thrusters) reports the force and torque it generates and the
//! mass it has at an instant, and the orbital tier holds them constant until the
//! owner reports again. A magnetic dipole the owner commands is held as the moment,
//! not as a torque: the field it acts in turns with the vessel and changes along the
//! orbit faster than an owner reports, so the orbital tier works the torque out from
//! the held moment at every integrator stage. Parts the owner moves inside the vessel (a
//! slewing antenna, a spinning rotor) carry angular momentum relative to its body axes;
//! the owner reports that momentum and the orbital tier holds it in the vessel's gyroscopic
//! coupling, as it does a wheel's. They live beside the guidance law rather than in it, so a
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

/// Smooth linear ramp of internal angular momentum over `[start_epoch, end_epoch]`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MomentumRamp {
    /// Rate of change of internal angular momentum in body axes, N·m.
    pub rate_body_nm: [f64; 3],
    /// Epoch where the ramp starts (`internal_momentum_body_nms`).
    pub start_epoch: TdbEpoch,
    /// Epoch where the ramp ends.
    pub end_epoch: TdbEpoch,
}

impl MomentumRamp {
    fn is_valid(&self) -> bool {
        self.rate_body_nm.iter().all(|v| v.is_finite()) && self.end_epoch >= self.start_epoch
    }

    fn is_nonzero(&self) -> bool {
        self.end_epoch > self.start_epoch && self.rate_body_nm.iter().any(|&v| v != 0.0)
    }

    fn write_bits(&self, bits: &mut Vec<u64>) {
        bits.push(1);
        bits.extend(self.rate_body_nm.iter().map(|v| v.to_bits()));
        bits.push(self.start_epoch.j2000_seconds() as u64);
        bits.push(self.start_epoch.subsec_ticks());
        bits.push(self.end_epoch.j2000_seconds() as u64);
        bits.push(self.end_epoch.subsec_ticks());
    }

    fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_u8(1);
        for v in self.rate_body_nm {
            h.write_f64_canonical(v);
        }
        h.write_i64(self.start_epoch.j2000_seconds());
        h.write_u64(self.start_epoch.subsec_ticks());
        h.write_i64(self.end_epoch.j2000_seconds());
        h.write_u64(self.end_epoch.subsec_ticks());
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
    /// Angular momentum of the vessel's internal parts moving relative to its body axes,
    /// about the centre of mass, body axes, N·m·s. The vessel's angular momentum is
    /// `I ω` plus this, and it enters the gyroscopic coupling at every integrator stage.
    #[serde(default)]
    pub internal_momentum_body_nms: [f64; 3],
    /// Optional linear ramp applied to [`Self::internal_momentum_body_nms`] over an
    /// exchange interval so continuous internal angular momentum transfer reacts cleanly
    /// against rigid-body nutation.
    #[serde(default)]
    pub momentum_ramp: Option<MomentumRamp>,
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
            internal_momentum_body_nms: [0.0; 3],
            momentum_ramp: None,
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

    /// These loads with `momentum_body_nms` carried by the vessel's internal parts
    /// relative to its body axes.
    #[must_use]
    pub fn with_internal_momentum(mut self, momentum_body_nms: DVec3) -> Self {
        self.internal_momentum_body_nms = momentum_body_nms.to_array();
        self
    }

    /// The held internal angular momentum, body axes, N·m·s.
    #[must_use]
    pub fn internal_momentum_body_nms(&self) -> DVec3 {
        DVec3::from_array(self.internal_momentum_body_nms)
    }

    /// These loads with a linear ramp `rate_body_nm` on internal momentum
    /// from `start_epoch` to `end_epoch`.
    #[must_use]
    pub fn with_momentum_ramp(
        mut self,
        rate_body_nm: DVec3,
        start_epoch: TdbEpoch,
        end_epoch: TdbEpoch,
    ) -> Self {
        self.momentum_ramp = Some(MomentumRamp {
            rate_body_nm: rate_body_nm.to_array(),
            start_epoch,
            end_epoch,
        });
        self
    }

    /// Held internal angular momentum at `epoch`, body axes, N·m·s.
    #[must_use]
    pub fn internal_momentum_at(&self, epoch: TdbEpoch) -> DVec3 {
        let mut held = self.internal_momentum_body_nms();
        if let Some(ramp) = self.momentum_ramp {
            let duration_s = ramp.end_epoch.signed_offset_seconds_from(ramp.start_epoch);
            if duration_s > 0.0 {
                let elapsed_s = epoch
                    .signed_offset_seconds_from(ramp.start_epoch)
                    .clamp(0.0, duration_s);
                held += DVec3::from_array(ramp.rate_body_nm) * elapsed_s;
            }
        }
        held
    }

    /// Whether anything held acts on the vessel's motion: a force, a torque, a dipole or
    /// internal momentum. Held mass alone does not.
    #[must_use]
    pub fn acts_on_motion(&self) -> bool {
        self.held_vectors().any(|&v| v != 0.0) || self.momentum_ramp.is_some_and(|r| r.is_nonzero())
    }

    fn held_vectors(&self) -> impl Iterator<Item = &f64> {
        self.force_body_n
            .iter()
            .chain(&self.torque_body_nm)
            .chain(&self.dipole_body_a_m2)
            .chain(&self.internal_momentum_body_nms)
    }

    /// Whether every component is finite and any mass positive.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.held_vectors().all(|v| v.is_finite())
            && self.mass.is_none_or(|mass| mass.is_valid())
            && self.momentum_ramp.is_none_or(|r| r.is_valid())
    }

    /// Whether `other` holds the same bits, so replacing one with the other changes no
    /// future step. `PartialEq` would equate `-0.0` with `0.0`.
    #[must_use]
    pub fn same_bits(&self, other: &Self) -> bool {
        self.bit_pattern() == other.bit_pattern()
    }

    fn bit_pattern(&self) -> Vec<u64> {
        let mut bits: Vec<u64> = self.held_vectors().map(|v| v.to_bits()).collect();
        match self.mass {
            Some(mass) => {
                bits.push(1);
                bits.push(mass.mass_kg.to_bits());
                bits.extend(
                    mass.inertia_cg_body_cols
                        .iter()
                        .flatten()
                        .map(|v| v.to_bits()),
                );
            }
            None => bits.push(0),
        }
        match self.momentum_ramp {
            Some(ramp) => ramp.write_bits(&mut bits),
            None => bits.push(0),
        }
        bits
    }

    pub(crate) fn write_to_hasher(&self, h: &mut DomainHasher) {
        for &v in self.held_vectors() {
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
        match self.momentum_ramp {
            Some(ramp) => ramp.write_to_hasher(h),
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
    fn add_to(&self, epoch: TdbEpoch, state: &ContinuousPlantState, wrench: &mut PlantStageWrench) {
        if self.loads.has_wrench() {
            wrench.env_force_body_n += DVec3::from_array(self.loads.force_body_n);
            wrench.ext_torque_body_nm += DVec3::from_array(self.loads.torque_body_nm);
        }
        // In body axes the held momentum `h` turns with the vessel: the rate of
        // `I ω + h(t)` is the applied torque less `ω × (I ω + h(t)) + dh/dt`.
        let held = self.loads.internal_momentum_at(epoch);
        if held != DVec3::ZERO {
            wrench.ext_torque_body_nm -= state.omega_body_rad_s.cross(held);
        }
        if let Some(ramp) = self.loads.momentum_ramp {
            let duration_s = ramp.end_epoch.signed_offset_seconds_from(ramp.start_epoch);
            if duration_s > 0.0 {
                let elapsed_s = epoch.signed_offset_seconds_from(ramp.start_epoch);
                if elapsed_s >= -1.0e-4 && elapsed_s <= duration_s + 1.0e-4 {
                    wrench.ext_torque_body_nm -= DVec3::from_array(ramp.rate_body_nm);
                }
            }
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
        self.add_to(epoch, state, &mut wrench);
        wrench
    }

    fn evaluate_stage_checked(
        &self,
        tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> Result<PlantStageWrench, GravityError> {
        let mut wrench = self.inner.evaluate_stage_checked(tau_s, epoch, state)?;
        self.add_to(epoch, state, &mut wrench);
        Ok(wrench)
    }
}

//! Solar radiation pressure on a vessel, read from the shared
//! [`SolarFluxModel`] so it sees the same Sun, distance and eclipses as
//! everything else that feels sunlight.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_spacetime::{SolarFluxModel, SolarFluxSample};

use crate::{
    ephemeris::{DerivativeOrder, EphemerisCatalog},
    gravity::GravityError,
    ids::NaifId,
    plant::{
        ContinuousControlLaw, ContinuousPlantState, PlantStageWrench, quat_normalize_portable,
        quat_rotate_vec_portable,
    },
    time::TdbEpoch,
};

/// A vessel as sunlight pushes on it: the cannonball model, a sphere of one
/// cross-section and one reflectivity whatever its attitude.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SrpSurface {
    /// Cross-section facing the Sun, m².
    pub area_m2: f64,
    /// Radiation pressure coefficient: `1` for a surface that absorbs all
    /// the light, up to `2` for one that reflects it straight back.
    pub reflectivity_coefficient: f64,
}

impl SrpSurface {
    /// Force of the sunlight in `sample` on this surface, N, in ICRF axes:
    /// directly away from the Sun, and zero in an umbra.
    #[must_use]
    pub fn force_icrf_n(&self, sample: &SolarFluxSample) -> DVec3 {
        -sample.to_sun_icrf
            * (sample.radiation_pressure_n_m2() * self.reflectivity_coefficient * self.area_m2)
    }
}

/// Another control law with solar radiation pressure added to its
/// environment force.
pub struct SolarRadiationPressureLaw<'a, L: ContinuousControlLaw + ?Sized> {
    /// The law that flies the vessel.
    pub inner: &'a L,
    /// The vessel as sunlight sees it.
    pub surface: SrpSurface,
    /// The Sun and what eclipses it.
    pub flux: &'a SolarFluxModel,
    /// Body positions; the same catalog the propagation integrates in.
    pub ephemeris: &'a dyn EphemerisCatalog,
}

impl<L: ContinuousControlLaw + ?Sized> SolarRadiationPressureLaw<'_, L> {
    fn pressure_force_body_n(
        &self,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> Result<DVec3, GravityError> {
        let origin_ssb_m = if state.primary_body == NaifId::SSB {
            DVec3::ZERO
        } else {
            self.ephemeris
                .geometric_state_icrf(
                    state.primary_body,
                    NaifId::SSB,
                    epoch,
                    DerivativeOrder::Position,
                )?
                .pos_m
        };
        let sample =
            self.flux
                .sample(origin_ssb_m + state.pos_rel_icrf_m, epoch, self.ephemeris)?;
        let body_from_icrf = quat_normalize_portable(state.q_body_to_icrf).conjugate();
        Ok(quat_rotate_vec_portable(
            body_from_icrf,
            self.surface.force_icrf_n(&sample),
        ))
    }
}

impl<L: ContinuousControlLaw + ?Sized> ContinuousControlLaw for SolarRadiationPressureLaw<'_, L> {
    /// Where the Sun cannot be found the pressure is left out; the checked
    /// form reports that instead, and is the one propagation calls.
    fn evaluate_stage(
        &self,
        tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> PlantStageWrench {
        let mut wrench = self.inner.evaluate_stage(tau_s, epoch, state);
        wrench.env_force_body_n += self
            .pressure_force_body_n(epoch, state)
            .unwrap_or(DVec3::ZERO);
        wrench
    }

    fn evaluate_stage_checked(
        &self,
        tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> Result<PlantStageWrench, GravityError> {
        let mut wrench = self.inner.evaluate_stage_checked(tau_s, epoch, state)?;
        wrench.env_force_body_n += self.pressure_force_body_n(epoch, state)?;
        Ok(wrench)
    }
}

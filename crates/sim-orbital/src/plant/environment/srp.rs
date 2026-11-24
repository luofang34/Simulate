//! Solar radiation pressure on a vessel, read from the shared
//! [`SolarFluxModel`](sim_spacetime::SolarFluxModel) so it sees the same Sun, distance
//! and eclipses as everything else that feels sunlight.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_spacetime::SolarFluxSample;

use super::{EnvironmentLoad, EnvironmentStage, EnvironmentWrench};
use crate::{
    ephemeris::DerivativeOrder,
    gravity::GravityError,
    hash::DomainHasher,
    ids::NaifId,
    plant::{quat_normalize_portable, quat_rotate_vec_portable},
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

impl EnvironmentLoad for SrpSurface {
    fn reads_body(&self) -> bool {
        false
    }

    fn wrench(&self, stage: &EnvironmentStage<'_>) -> Result<EnvironmentWrench, GravityError> {
        let (state, context) = (stage.state, stage.context);
        let origin_ssb_m = if state.primary_body == NaifId::SSB {
            DVec3::ZERO
        } else {
            context
                .ephemeris
                .geometric_state_icrf(
                    state.primary_body,
                    NaifId::SSB,
                    stage.epoch,
                    DerivativeOrder::Position,
                )?
                .pos_m
        };
        let sample = context.sunlight.sample(
            origin_ssb_m + state.pos_rel_icrf_m,
            stage.epoch,
            context.ephemeris,
        )?;
        let body_from_icrf = quat_normalize_portable(state.q_body_to_icrf).conjugate();
        Ok(EnvironmentWrench {
            force_body_n: quat_rotate_vec_portable(body_from_icrf, self.force_icrf_n(&sample)),
            torque_body_nm: DVec3::ZERO,
        })
    }

    fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_f64_canonical(self.area_m2);
        h.write_f64_canonical(self.reflectivity_coefficient);
    }
}

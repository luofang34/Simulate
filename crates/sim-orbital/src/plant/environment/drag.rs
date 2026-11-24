//! Drag in the thin air of the body a vessel orbits.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_geo::LocalFrame;

use super::{EnvironmentLoad, EnvironmentStage, EnvironmentWrench};
use crate::{gravity::GravityError, hash::DomainHasher};

/// A vessel as the air it flies through slows it: one ballistic coefficient whatever
/// its attitude.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DragSurface {
    /// Ballistic coefficient `m / (C_d A)`, kg/m²: the deceleration is
    /// `½ ρ v² / β` against the air-relative velocity.
    pub ballistic_coefficient_kg_m2: f64,
}

impl EnvironmentLoad for DragSurface {
    fn reads_body(&self) -> bool {
        true
    }

    /// Zero about an origin that is no described body, where the body's atmosphere has
    /// no value (above or below its model), and for a non-positive coefficient.
    fn wrench(&self, stage: &EnvironmentStage<'_>) -> Result<EnvironmentWrench, GravityError> {
        let beta = self.ballistic_coefficient_kg_m2;
        let Some(about) = stage.about.filter(|_| beta.is_finite() && beta > 0.0) else {
            return Ok(EnvironmentWrench::default());
        };
        let position = about
            .body
            .ellipsoid
            .fixed_to_geodetic(about.position_fixed_m);
        let Ok(air) = about
            .body
            .atmosphere()
            .sample(&position, stage.epoch.approximate_utc())
        else {
            return Ok(EnvironmentWrench::default());
        };
        if air.density_kg_m3.is_nan() || air.density_kg_m3 <= 0.0 {
            return Ok(EnvironmentWrench::default());
        }
        // The air turns with the body, plus whatever wind the model states.
        let wind_fixed = match air.wind_ned_m_s == DVec3::ZERO {
            true => DVec3::ZERO,
            false => LocalFrame::at(&position).ned_to_ecef() * air.wind_ned_m_s,
        };
        let through_air = about.velocity_fixed_m_s - wind_fixed;
        let deceleration = -0.5 * air.density_kg_m3 * through_air.length() * through_air / beta;
        Ok(EnvironmentWrench {
            force_body_n: about.fixed_to_vessel * (deceleration * stage.mass_kg),
            torque_body_nm: DVec3::ZERO,
        })
    }

    fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_f64_canonical(self.ballistic_coefficient_kg_m2);
    }
}

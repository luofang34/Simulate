//! The torque a body's magnetic field puts on a vessel's own magnetism.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use super::{EnvironmentLoad, EnvironmentStage, EnvironmentWrench};
use crate::{gravity::GravityError, hash::DomainHasher};

const TESLA_PER_NANOTESLA: f64 = 1.0e-9;

/// The magnetic dipole a vessel is left with by its harness currents and its
/// magnetised parts, fixed in its body axes. In a field `B` it feels `m × B`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResidualDipole {
    /// The dipole moment in body axes, A·m².
    pub moment_body_a_m2: [f64; 3],
}

impl EnvironmentLoad for ResidualDipole {
    fn reads_body(&self) -> bool {
        true
    }

    fn wrench(&self, stage: &EnvironmentStage<'_>) -> Result<EnvironmentWrench, GravityError> {
        let Some(about) = stage.about else {
            return Ok(EnvironmentWrench::default());
        };
        let field_fixed_nt = about
            .body
            .magnetic_field()
            .field_fixed_nt(
                about.position_fixed_m,
                stage.epoch.approximate_decimal_year(),
            )
            .map_err(|source| GravityError::MagneticField {
                body: about.body.naif,
                source,
            })?;
        let field_body_t = about.fixed_to_vessel * field_fixed_nt * TESLA_PER_NANOTESLA;
        Ok(EnvironmentWrench {
            force_body_n: DVec3::ZERO,
            torque_body_nm: DVec3::from_array(self.moment_body_a_m2).cross(field_body_t),
        })
    }

    fn write_to_hasher(&self, h: &mut DomainHasher) {
        for component in self.moment_body_a_m2 {
            h.write_f64_canonical(component);
        }
    }
}

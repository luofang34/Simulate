//! The torque a body's gravity puts on an extended vessel.

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};

use super::{EnvironmentLoad, EnvironmentStage, EnvironmentWrench};
use crate::{gravity::GravityError, hash::DomainHasher};

/// Gravity-gradient torque from the gradient tensor of the origin body's field: the
/// whole field the catalog states, not its central term alone.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct GravityGradient;

/// The torque about the centre of mass of a body of inertia `inertia` in the gravity
/// gradient `gradient` (`∂g_i/∂x_j`), both in the same axes.
///
/// With the second moment `M = ∫ r rᵀ dm = ½ tr(I) 1 − I`, the torque
/// `∫ r × (G r) dm` is the axial vector of `G M`, and the isotropic part of `M` drops
/// out because `G` is symmetric: `τ_i = −ε_ijk (G I)_kj`.
#[must_use]
pub(crate) fn gravity_gradient_torque(gradient: DMat3, inertia: DMat3) -> DVec3 {
    let gi = gradient * inertia;
    // glam stores columns: `gi.col(j)[i]` is the element in row `i`, column `j`.
    let (x, y, z) = (gi.x_axis, gi.y_axis, gi.z_axis);
    DVec3::new(z.y - y.z, x.z - z.x, y.x - x.y)
}

impl EnvironmentLoad for GravityGradient {
    fn reads_body(&self) -> bool {
        true
    }

    fn wrench(&self, stage: &EnvironmentStage<'_>) -> Result<EnvironmentWrench, GravityError> {
        let Some(about) = stage.about else {
            return Ok(EnvironmentWrench::default());
        };
        let gradient_fixed = about
            .body
            .gravity()
            .sample(about.position_fixed_m, true)
            .ok()
            .and_then(|sample| sample.gradient_per_s2)
            .ok_or(GravityError::SingularRadius(about.body.naif))?;
        let to_vessel = about.fixed_to_vessel;
        let gradient_body = to_vessel * gradient_fixed * to_vessel.transpose();
        Ok(EnvironmentWrench {
            force_body_n: DVec3::ZERO,
            torque_body_nm: gravity_gradient_torque(gradient_body, stage.inertia_body),
        })
    }

    fn write_to_hasher(&self, _h: &mut DomainHasher) {}
}

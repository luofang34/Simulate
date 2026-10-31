use glam::{DQuat, DVec3};
use mbd_core::{ComponentId, LocalSiteKinematics, MbdError};
use sim_core::{Block, BlockError, RigidBody6DofBlock, TickSpan};
use sim_geo::{BodyEllipsoid, InertialTransform};

/// What inertial sensors at a mounting site respond to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InertialSiteSample {
    /// Site-to-J2000 rotation.
    pub site_to_inertial: DQuat,
    /// Specific force in site axes, m/s².
    pub specific_force_site_m_s2: DVec3,
    /// Inertial angular rate in site axes, rad/s.
    pub angular_rate_site_rad_s: DVec3,
}

impl InertialSiteSample {
    /// The sample at `site` at `vehicle`'s output instant, with `frame` the
    /// central body's orientation at that instant.
    pub fn of(
        vehicle: &RigidBody6DofBlock,
        site: ComponentId,
        frame: &InertialTransform,
    ) -> Result<Self, MbdError> {
        let kinematics = vehicle.site_kinematics(site)?;
        let body_to_fixed = vehicle.output().attitude;
        Ok(Self {
            site_to_inertial: (frame.fixed_to_j2000_rotation()
                * body_to_fixed
                * kinematics.attitude)
                .normalize(),
            specific_force_site_m_s2: kinematics.specific_force_site_m_s2,
            angular_rate_site_rad_s: kinematics.angular_rate_site_rad_s,
        })
    }

    pub(crate) fn validate(&self, block: &'static str) -> Result<(), BlockError> {
        if self.site_to_inertial.is_finite()
            && self.specific_force_site_m_s2.is_finite()
            && self.angular_rate_site_rad_s.is_finite()
        {
            return Ok(());
        }
        Err(BlockError::InputOutOfRange {
            block,
            input: "site",
            detail: format!("{self:?} is not finite"),
        })
    }
}

/// `site` in the central body's rotating frame, at `vehicle`'s output instant.
pub fn site_in_fixed_frame(
    vehicle: &RigidBody6DofBlock,
    site: ComponentId,
) -> Result<LocalSiteKinematics, MbdError> {
    let state = vehicle.output();
    Ok(vehicle.site_kinematics(site)?.in_ecef(
        state.position_ecef,
        state.velocity_ecef,
        state.attitude,
    ))
}

/// The local surface as a plane in the central body's frame: what a landing
/// sensor needs of the terrain beneath it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfacePlane {
    /// A point on the surface, metres.
    pub point_fixed_m: DVec3,
    /// Unit upward surface normal.
    pub normal_fixed: DVec3,
}

impl SurfacePlane {
    /// The level surface `terrain_height_m` above the reference ellipsoid,
    /// directly beneath `position_fixed_m`.
    pub fn level_beneath(
        body: &BodyEllipsoid,
        position_fixed_m: DVec3,
        terrain_height_m: f64,
    ) -> Self {
        let mut point = body.fixed_to_geodetic(position_fixed_m);
        point.height_m = terrain_height_m;
        let point_fixed_m = body.geodetic_to_fixed(&point);
        point.height_m += 1.0;
        let normal_fixed = (body.geodetic_to_fixed(&point) - point_fixed_m).normalize();
        Self {
            point_fixed_m,
            normal_fixed,
        }
    }

    /// Height of `point_fixed_m` above the plane, metres.
    pub fn clearance_m(&self, point_fixed_m: DVec3) -> f64 {
        (point_fixed_m - self.point_fixed_m).dot(self.normal_fixed)
    }

    pub(crate) fn validate(&self, block: &'static str) -> Result<(), BlockError> {
        if self.point_fixed_m.is_finite() && self.normal_fixed.is_normalized() {
            return Ok(());
        }
        Err(BlockError::InputOutOfRange {
            block,
            input: "surface",
            detail: format!("{self:?} is not a plane"),
        })
    }
}

/// Rejects a period that would never advance.
pub(crate) fn require_period(block: &'static str, period: TickSpan) -> Result<(), BlockError> {
    if period != TickSpan::ZERO {
        return Ok(());
    }
    Err(BlockError::InputOutOfRange {
        block,
        input: "period",
        detail: "a zero period never advances".to_owned(),
    })
}

/// The rotation a body turning at `rate_rad_s` in its own axes makes in `dt_s`.
pub(crate) fn rotation_over(rate_rad_s: DVec3, dt_s: f64) -> DQuat {
    DQuat::from_scaled_axis(rate_rad_s * dt_s)
}

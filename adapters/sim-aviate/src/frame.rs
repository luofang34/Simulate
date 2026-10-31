use glam::{DQuat, DVec3};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};

use crate::AviateStimulus;

#[cfg(test)]
mod tests;

/// Half turn about the shared forward axis, written exactly: turns
/// forward-left-up components into forward-right-down ones and back.
const FRD_FROM_FLU: DQuat = DQuat::from_xyzw(1.0, 0.0, 0.0, 0.0);

/// The flat world a flight controller believes in: an Earth-fixed
/// east-north-up tangent frame at a chosen origin, standing in for the
/// contract's Gazebo world.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnuWorld {
    origin_ecef: DVec3,
    frame: LocalFrame,
    enu_from_ecef: DQuat,
}

/// One model snapshot in the contract's conventions, before it becomes bit
/// patterns.
///
/// These describe the mounting site, as Gazebo's lanes describe a link origin
/// rather than a centre of gravity. All motion is relative to the Earth-fixed
/// world, so the angular rate is what successive attitudes show and omits the
/// Earth rate an inertial gyroscope would add.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModelLanes {
    /// Site position, world ENU metres.
    pub position_enu_m: [f64; 3],
    /// Site-to-world rotation `[w, x, y, z]`, ENU world and site axes turned
    /// from forward-right-down to forward-left-up.
    pub attitude_wxyz: [f64; 4],
    /// Site velocity, world ENU m/s.
    pub velocity_enu_m_s: [f64; 3],
    /// Angular rate in world ENU axes, rad/s: not a body rate, as the contract
    /// defines this lane.
    pub angular_rate_enu_rad_s: [f64; 3],
}

impl ModelLanes {
    pub(crate) fn is_finite(&self) -> bool {
        self.position_enu_m
            .iter()
            .chain(&self.attitude_wxyz)
            .chain(&self.velocity_enu_m_s)
            .chain(&self.angular_rate_enu_rad_s)
            .all(|lane| lane.is_finite())
    }
}

impl EnuWorld {
    /// The tangent frame at `origin` on `body`.
    pub fn new(body: &BodyEllipsoid, origin: &Geodetic) -> Self {
        let frame = LocalFrame::at(origin);
        Self {
            origin_ecef: body.geodetic_to_fixed(origin),
            frame,
            enu_from_ecef: DQuat::from_mat3(&frame.enu_to_ecef().transpose()).normalize(),
        }
    }

    /// `stimulus` in the contract's frames and units.
    pub fn model_lanes(&self, stimulus: &AviateStimulus) -> ModelLanes {
        let vehicle = &stimulus.vehicle;
        let site = stimulus.site.in_ecef(
            vehicle.position_ecef,
            vehicle.velocity_ecef,
            vehicle.attitude,
        );
        // The site's own velocity carries the inertial rate, which is not
        // motion over this Earth-fixed world.
        let rate_ecef = vehicle.attitude * vehicle.angular_rate_body;
        let lever_ecef = site.position_m - vehicle.position_ecef;
        let velocity_ecef = vehicle.velocity_ecef + rate_ecef.cross(lever_ecef);
        let attitude = self.enu_from_ecef * site.attitude * FRD_FROM_FLU;
        ModelLanes {
            position_enu_m: self.vector(site.position_m - self.origin_ecef),
            attitude_wxyz: [attitude.w, attitude.x, attitude.y, attitude.z],
            velocity_enu_m_s: self.vector(velocity_ecef),
            angular_rate_enu_rad_s: self.vector(rate_ecef),
        }
    }

    fn vector(&self, ecef: DVec3) -> [f64; 3] {
        self.frame.ecef_to_enu_vector(ecef).to_array()
    }
}

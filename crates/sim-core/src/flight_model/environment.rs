//! What a vehicle's force elements ask of the world while a step is being
//! integrated: air, ground, water and gravity around a rotating body, in the
//! inertial axes the step is integrated in.

use glam::{DMat3, DVec3};
use mbd_aero::{AeroInputs, AirSample};
use mbd_multibody::{
    ElementContext, EnvironmentError, ForceElement, Loads, MultibodyError, SurfaceMedium,
    SurfaceQuery, SurfaceSample, WaterQuery, WaterState,
};
use sim_atmosphere::AtmosphereField;
use sim_geo::{BodyEllipsoid, GravityField, LocalFrame, UtcTime};

use crate::flight_model::planet_frame::{fixed_from_inertial, spin};
use crate::multibody_vehicle::SurfaceContactProvider;

/// The world around a vehicle for one step.
///
/// World axes are inertial, centred on the central body, and coincide with
/// its fixed axes at the start of the step; time is counted from that
/// instant. The body turns under them, so every answer names the motion of
/// what it describes: the ground and the water move at the body's spin, and
/// the air moves and turns with it.
pub(crate) struct FlightEnvironment<'a> {
    /// The central body.
    pub(crate) body: BodyEllipsoid,
    /// Truth gravity.
    pub(crate) gravity: &'a dyn GravityField,
    /// Air and wind.
    pub(crate) atmosphere: &'a dyn AtmosphereField,
    /// Ground and water under the vehicle, if it can touch any.
    pub(crate) surface: Option<&'a dyn SurfaceContactProvider>,
    /// UTC at the start of the step.
    pub(crate) utc: UtcTime,
    /// Aerodynamic control channels held over the step.
    pub(crate) controls: &'a [f64],
    /// Acceleration the far ends of coupled mechanisms are carried on with,
    /// m/s².
    far_end_acceleration_m_s2: DVec3,
}

impl<'a> FlightEnvironment<'a> {
    /// The environment of a step that starts at `utc`.
    pub(crate) fn new(
        body: BodyEllipsoid,
        gravity: &'a dyn GravityField,
        atmosphere: &'a dyn AtmosphereField,
        utc: UtcTime,
        controls: &'a [f64],
    ) -> Self {
        Self {
            body,
            gravity,
            atmosphere,
            surface: None,
            utc,
            controls,
            far_end_acceleration_m_s2: DVec3::ZERO,
        }
    }

    /// The same environment over `surface`.
    pub(crate) fn over(mut self, surface: &'a dyn SurfaceContactProvider) -> Self {
        self.surface = Some(surface);
        self
    }

    /// The same environment with the far ends of coupled mechanisms taken
    /// to accelerate at `acceleration_m_s2`, world axes.
    pub(crate) fn carrying_far_ends_at(mut self, acceleration_m_s2: DVec3) -> Self {
        self.far_end_acceleration_m_s2 = acceleration_m_s2;
        self
    }

    /// Fixed-from-world rotation `time_s` into the step.
    fn turn(&self, time_s: f64) -> DMat3 {
        DMat3::from_quat(fixed_from_inertial(&self.body, time_s))
    }

    /// Inertial velocity of the body's own material at the world point
    /// `point_m`: what stands still on the ground moves like this.
    fn carried(&self, point_m: DVec3) -> DVec3 {
        spin(&self.body).cross(point_m)
    }

    /// The field's attraction at the world point `point_m`, `time_s` into
    /// the step, in world axes.
    pub(crate) fn attraction(&self, point_m: DVec3, time_s: f64) -> DVec3 {
        let turn = self.turn(time_s);
        turn.transpose() * self.gravity.attraction_fixed(turn * point_m)
    }

    /// The far end of a link to another vehicle, given as it is at the
    /// start of the step, `time_s` later: its position and velocity.
    ///
    /// It is taken to accelerate as this vehicle does at the start of the
    /// step, which is not known to be so. But when two linked vehicles each
    /// take this of the other, the stretch each sees is wrong by equal and
    /// opposite amounts, and that part of the error leaves the momentum of
    /// the pair alone; carrying the far end on at its velocity alone does
    /// not have this.
    pub(crate) fn far_end(
        &self,
        position_m: DVec3,
        velocity_m_s: DVec3,
        time_s: f64,
    ) -> (DVec3, DVec3) {
        let gained = self.far_end_acceleration_m_s2 * time_s;
        (
            position_m + (velocity_m_s + gained * 0.5) * time_s,
            velocity_m_s + gained,
        )
    }
}

impl SurfaceQuery for FlightEnvironment<'_> {
    fn surface_at(
        &self,
        point_world_m: DVec3,
        time_s: f64,
    ) -> Result<SurfaceSample, EnvironmentError> {
        let turn = self.turn(time_s);
        let fixed = turn * point_world_m;
        let Some(sample) = self.surface.and_then(|s| s.sample_surface(fixed)) else {
            // Nothing solid to touch: a sample no contact engages.
            return Ok(SurfaceSample {
                surface_point_world_m: point_world_m,
                outward_normal_world: point_world_m.normalize_or(DVec3::Z),
                velocity_world_m_s: DVec3::ZERO,
                medium: SurfaceMedium::Liquid,
                mu_static_override: None,
                mu_dynamic_override: None,
            });
        };
        let back = turn.transpose();
        Ok(SurfaceSample {
            // Placed relative to the queried point, so the penetration
            // keeps the digits it has in the provider's axes.
            surface_point_world_m: point_world_m + back * (sample.surface_point_world_m - fixed),
            outward_normal_world: back * sample.outward_normal_world,
            // Slip and closing rate are measured in the ground's own frame:
            // against the velocity that frame has at the queried point.
            velocity_world_m_s: self.carried(point_world_m) + back * sample.velocity_world_m_s,
            ..sample
        })
    }
}

impl WaterQuery for FlightEnvironment<'_> {
    fn water_at(
        &self,
        position_world_m: DVec3,
        time_s: f64,
    ) -> Result<Option<WaterState>, EnvironmentError> {
        let turn = self.turn(time_s);
        let fixed = turn * position_world_m;
        let Some(water) = self.surface.and_then(|s| s.sample_water(fixed)) else {
            return Ok(None);
        };
        let back = turn.transpose();
        Ok(Some(WaterState {
            surface_origin_world_m: position_world_m
                + back * (water.surface_origin_world_m - fixed),
            outward_up_world: back * water.outward_up_world,
            fluid_vel_world_m_s: self.carried(position_world_m) + back * water.fluid_vel_world_m_s,
            ..water
        }))
    }
}

impl AeroInputs for FlightEnvironment<'_> {
    fn air_at(&self, position_world_m: DVec3, time_s: f64) -> Result<AirSample, EnvironmentError> {
        let turn = self.turn(time_s);
        let geodetic = self.body.fixed_to_geodetic(turn * position_world_m);
        let air = self
            .atmosphere
            .sample(&geodetic, self.utc.advanced_by(time_s))
            .map_err(EnvironmentError::new)?;
        let wind_fixed = LocalFrame::at(&geodetic).ned_to_ecef() * air.wind_ned_m_s;
        Ok(AirSample {
            ambient: air.ambient_gas(),
            wind_world_m_s: self.carried(position_world_m) + turn.transpose() * wind_fixed,
            // The atmosphere is carried round by the body.
            angular_velocity_world_rad_s: spin(&self.body),
        })
    }

    fn controls(&self) -> &[f64] {
        self.controls
    }
}

/// The field's attraction at the vehicle's centre of gravity, applied as a
/// uniform acceleration: the gradient across a vehicle is not modelled.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CgGravity {
    /// Centre of gravity in the root link's axes, held over the step, m.
    pub(crate) cg_root_link_m: DVec3,
}

impl ForceElement<FlightEnvironment<'_>> for CgGravity {
    fn evaluate(
        &self,
        ctx: &ElementContext<'_, FlightEnvironment<'_>>,
        _state: &[f64],
        _modes: &[f64],
        loads: &mut Loads<'_>,
        _state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let base = ctx.kinematics.base();
        let cg = base.position_world_m + base.rotation_world_from_root * self.cg_root_link_m;
        loads.add_gravity(ctx.inputs.attraction(cg, ctx.time_s));
        Ok(())
    }
}

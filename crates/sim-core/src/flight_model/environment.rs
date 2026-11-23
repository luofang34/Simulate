//! What a vehicle's force elements ask of the world while a step is being
//! integrated: air, ground, water and gravity around a rotating body, seen
//! from the frame the step is integrated in.

use std::cell::RefCell;

use glam::{DMat3, DVec3};
use mbd_aero::{AeroInputs, AirSample};
use mbd_core::AmbientGas;
use mbd_multibody::{
    ElementContext, FloatingBaseState, ForceElement, Loads, MultibodyError, PointKinematics,
    SurfaceSample, WaterQuery, WaterState,
};
use sim_atmosphere::{AtmosphereError, AtmosphereField};
use sim_geo::{BodyEllipsoid, GravityField, LocalFrame, UtcTime};

use crate::flight_model::planet_frame::{
    fixed_from_inertial, fixed_to_instantaneous_inertial, inertial_to_fixed_after_step,
    point_in_fixed_axes, spin,
};
use crate::multibody_vehicle::SurfaceContactProvider;

/// The frame a step is integrated in. Its axes are those of the central
/// body's fixed frame at the start of the step and do not turn; its origin
/// starts at the vehicle's root with the root's velocity and keeps the
/// acceleration the vehicle has at that instant: the gravity there and the
/// specific force last read at its centre of gravity.
///
/// A uniformly accelerating frame is exact once the same acceleration is
/// taken off the field the vehicle feels, which
/// [`FlightEnvironment::attraction`] does. A vehicle therefore starts every
/// step at rest at the origin and stays near it while its loads stay as they
/// were, whether it is in free fall, under thrust or standing on the ground.
/// Its orbital speed and its acceleration stay out of the model's
/// velocity-product terms, where they would cost digits and couple the
/// truncation of a tumble into the trajectory; and the ground under a
/// standing vehicle moves in the frame as the vehicle does, so stiff contact
/// sees no motion that the integrator would have to resolve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct StepFrame {
    /// Frame origin at the start of the step, in the inertial axes that
    /// coincide with the fixed ones then, m.
    origin_m: DVec3,
    /// Velocity of the frame at the start of the step, m/s.
    velocity_m_s: DVec3,
    /// Acceleration of the frame, m/s².
    acceleration_m_s2: DVec3,
    /// The root at the start of the step, in the frame.
    start: FloatingBaseState,
}

impl StepFrame {
    /// The frame riding with a root that is at `base_fixed` in fixed axes
    /// and accelerating at `acceleration_m_s2`.
    fn riding(
        base_fixed: &FloatingBaseState,
        body: &BodyEllipsoid,
        acceleration_m_s2: DVec3,
    ) -> Self {
        let inertial = fixed_to_instantaneous_inertial(base_fixed, body);
        Self {
            origin_m: inertial.position_world_m,
            velocity_m_s: inertial.linear_vel_world_m_s(),
            acceleration_m_s2,
            start: FloatingBaseState {
                position_world_m: DVec3::ZERO,
                linear_vel_root_m_s: DVec3::ZERO,
                ..inertial
            },
        }
    }

    /// The root at the start of the step, in the frame.
    pub(crate) const fn start(&self) -> &FloatingBaseState {
        &self.start
    }

    /// Velocity of the frame `elapsed_s` into the step, in the inertial
    /// axes that coincided with the fixed ones at its start, m/s.
    fn velocity(&self, elapsed_s: f64) -> DVec3 {
        self.velocity_m_s + self.acceleration_m_s2 * elapsed_s
    }

    /// A frame position `elapsed_s` into the step, in those inertial axes.
    fn position_inertial(&self, position_m: DVec3, elapsed_s: f64) -> DVec3 {
        self.origin_m
            + self.velocity_m_s * elapsed_s
            + self.acceleration_m_s2 * (0.5 * elapsed_s * elapsed_s)
            + position_m
    }

    /// A point given in those inertial axes at the start of the step, as
    /// the frame sees it `elapsed_s` later: the far end of a link to another
    /// vehicle. It is taken to accelerate as the frame does, which is not
    /// known to be so. But when two linked vehicles each take this of the
    /// other, the stretch each sees is wrong by equal and opposite amounts,
    /// and that part of the error leaves the momentum of the pair alone;
    /// carrying the far end on at its velocity alone does not have this.
    pub(crate) fn seen(
        &self,
        position_m: DVec3,
        velocity_m_s: DVec3,
        elapsed_s: f64,
    ) -> (DVec3, DVec3) {
        let relative = velocity_m_s - self.velocity_m_s;
        (position_m - self.origin_m + relative * elapsed_s, relative)
    }

    /// A root that is at `base` in the frame `dt_s` after the start of the
    /// step, in the fixed axes of that instant.
    pub(crate) fn base_in_fixed_axes(
        &self,
        base: &FloatingBaseState,
        body: &BodyEllipsoid,
        dt_s: f64,
    ) -> FloatingBaseState {
        let turn = base.rotation_world_from_root.conjugate();
        let inertial = FloatingBaseState {
            position_world_m: self.position_inertial(base.position_world_m, dt_s),
            linear_vel_root_m_s: base.linear_vel_root_m_s + turn * self.velocity(dt_s),
            ..*base
        };
        inertial_to_fixed_after_step(&inertial, body, dt_s)
    }
}

/// The world around a vehicle for one step.
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
    /// The frame the step is integrated in.
    pub(crate) frame: StepFrame,
    /// The first atmosphere failure met while sampling: the element
    /// interface cannot return one, so it is kept here for the block.
    failure: RefCell<Option<AtmosphereError>>,
}

impl<'a> FlightEnvironment<'a> {
    /// The environment of a step that starts at `utc` with the vehicle's
    /// root at `base_fixed` in fixed axes.
    pub(crate) fn new(
        body: BodyEllipsoid,
        gravity: &'a dyn GravityField,
        atmosphere: &'a dyn AtmosphereField,
        utc: UtcTime,
        controls: &'a [f64],
        base_fixed: &FloatingBaseState,
    ) -> Self {
        Self {
            body,
            gravity,
            atmosphere,
            surface: None,
            utc,
            controls,
            frame: StepFrame::riding(
                base_fixed,
                &body,
                gravity.attraction_fixed(base_fixed.position_world_m),
            ),
            failure: RefCell::new(None),
        }
    }

    /// The same environment in a frame that also keeps the specific force
    /// `specific_force_fixed_m_s2` the vehicle is under at the start of the
    /// step, given in fixed axes.
    pub(crate) fn pushed_by(mut self, specific_force_fixed_m_s2: DVec3) -> Self {
        self.frame.acceleration_m_s2 += specific_force_fixed_m_s2;
        self
    }

    /// The same environment over `surface`.
    pub(crate) fn over(mut self, surface: &'a dyn SurfaceContactProvider) -> Self {
        self.surface = Some(surface);
        self
    }

    /// The atmosphere failure met since the last call, if any.
    pub(crate) fn take_failure(&self) -> Option<AtmosphereError> {
        self.failure.borrow_mut().take()
    }

    /// The environment as an element evaluated `elapsed_s` into the step
    /// samples it.
    pub(crate) const fn at(&self, elapsed_s: f64) -> StageWorld<'_, 'a> {
        StageWorld {
            environment: self,
            elapsed_s,
        }
    }

    /// Fixed-from-frame rotation `elapsed_s` into the step.
    fn turn(&self, elapsed_s: f64) -> DMat3 {
        DMat3::from_quat(fixed_from_inertial(&self.body, elapsed_s))
    }

    /// Gravitational acceleration felt in the frame at its point `point_m`,
    /// `elapsed_s` into the step: the field's attraction there less the
    /// frame's own fall, in frame axes.
    pub(crate) fn attraction(&self, point_m: DVec3, elapsed_s: f64) -> DVec3 {
        let turn = self.turn(elapsed_s);
        let fixed = turn * self.frame.position_inertial(point_m, elapsed_s);
        turn.transpose() * self.gravity.attraction_fixed(fixed) - self.frame.acceleration_m_s2
    }

    /// The link point `point`, given in the frame `elapsed_s` into the
    /// step, as the ground sees it: position, velocity and rates relative to
    /// the rotating body, in its fixed axes.
    fn on_the_body(&self, point: &PointKinematics, elapsed_s: f64) -> PointKinematics {
        let inertial = PointKinematics {
            position_world_m: self
                .frame
                .position_inertial(point.position_world_m, elapsed_s),
            velocity_world_m_s: point.velocity_world_m_s + self.frame.velocity(elapsed_s),
            ..*point
        };
        point_in_fixed_axes(&inertial, &self.body, elapsed_s)
    }

    /// The ground under the link point `point`, given in the frame
    /// `elapsed_s` into the step: the point as the ground sees it and the
    /// surface beneath, both in the body's fixed axes. `None` away from any
    /// surface.
    pub(crate) fn ground_under(
        &self,
        point: &PointKinematics,
        elapsed_s: f64,
    ) -> Option<(PointKinematics, SurfaceSample)> {
        let on_ground = self.on_the_body(point, elapsed_s);
        let sample = self.surface?.sample_surface(on_ground.position_world_m)?;
        Some((on_ground, sample))
    }
}

/// A [`FlightEnvironment`] at one instant of its step: what elements that
/// ask by position alone are answered from, each answer given in the frame
/// of the step.
pub(crate) struct StageWorld<'s, 'a> {
    environment: &'s FlightEnvironment<'a>,
    elapsed_s: f64,
}

impl StageWorld<'_, '_> {
    /// A frame position in the body's fixed axes, with the fixed-from-frame
    /// rotation and the velocity in frame axes of the body's own material
    /// at that point.
    fn on_the_body(&self, position_m: DVec3) -> (DVec3, DMat3, DVec3) {
        let env = self.environment;
        let inertial = env.frame.position_inertial(position_m, self.elapsed_s);
        let turn = env.turn(self.elapsed_s);
        let carried = spin(&env.body).cross(inertial) - env.frame.velocity(self.elapsed_s);
        (turn * inertial, turn, carried)
    }
}

impl AeroInputs for StageWorld<'_, '_> {
    fn air_at(&self, position_world_m: DVec3) -> AirSample {
        let env = self.environment;
        let (fixed, turn, carried) = self.on_the_body(position_world_m);
        let geodetic = env.body.fixed_to_geodetic(fixed);
        let utc = env.utc.advanced_by(self.elapsed_s);
        match env.atmosphere.sample(&geodetic, utc) {
            Ok(air) => {
                let wind_fixed = LocalFrame::at(&geodetic).ned_to_ecef() * air.wind_ned_m_s;
                AirSample {
                    ambient: air.ambient_gas(),
                    wind_world_m_s: carried + turn.transpose() * wind_fixed,
                }
            }
            Err(error) => {
                env.failure.borrow_mut().get_or_insert(error);
                AirSample {
                    ambient: AmbientGas::VACUUM,
                    wind_world_m_s: carried,
                }
            }
        }
    }

    fn controls(&self) -> &[f64] {
        self.environment.controls
    }
}

impl WaterQuery for StageWorld<'_, '_> {
    fn water_at(&self, position_world_m: DVec3) -> Option<WaterState> {
        let env = self.environment;
        let (fixed, turn, carried) = self.on_the_body(position_world_m);
        let water = env.surface?.sample_water(fixed)?;
        let back = turn.transpose();
        // The surface point is placed relative to the queried one, which is
        // exact in either frame.
        let surface_m = position_world_m + back * (water.surface_origin_world_m - fixed);
        Some(WaterState {
            surface_origin_world_m: surface_m,
            outward_up_world: back * water.outward_up_world,
            fluid_vel_world_m_s: carried + back * water.fluid_vel_world_m_s,
            ..water
        })
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

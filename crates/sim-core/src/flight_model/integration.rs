//! Advancing a vehicle's multibody model one block period around a rotating
//! body: the choice of integrator, the scale of the model, and the step
//! itself.

use mbd_core::{ComponentTree, DynamicMassContribution};
use mbd_multibody::{
    DormandPrince54, ForceElement, ModelState, MultibodyError, MultibodyModel, Rk4, Rosenbrock23,
    TimeScale, Tolerances, Workspace,
};
use sim_atmosphere::AtmosphereError;
use sim_geo::BodyEllipsoid;

use crate::BlockError;
use crate::flight_model::environment::FlightEnvironment;
use crate::flight_model::planet_frame::{
    fixed_to_instantaneous_inertial, inertial_to_fixed_after_step,
};

/// How a vehicle's equations of motion are integrated across a block period.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VehicleIntegrator {
    /// Classical fourth-order Runge-Kutta in `substeps` equal steps per
    /// period: a fixed cost per step, for real-time flight whose period
    /// resolves the vehicle's fastest dynamics.
    FixedRk4 {
        /// Equal steps per block period, at least one.
        substeps: u32,
    },
    /// Dormand-Prince 5(4) with error control inside the period, for motion
    /// whose time scale changes along the flight.
    Adaptive(Tolerances),
    /// Rosenbrock 2(3), for stiff contact, tethers and very light links that
    /// an explicit scheme would need many steps to keep stable.
    Stiff(Tolerances),
}

impl VehicleIntegrator {
    /// One fourth-order step per block period.
    pub const REAL_TIME: Self = Self::FixedRk4 { substeps: 1 };

    /// Adaptive Dormand–Prince 5(4) with uniform relative and absolute tolerances.
    #[must_use]
    pub fn adaptive(rtol: f64, atol: f64) -> Self {
        Self::Adaptive(Tolerances::uniform(rtol, atol))
    }
}

/// Where the characteristic time of a vehicle's motion comes from. With the
/// vehicle's own extent and mass it sets the magnitudes that integration
/// tolerances are measured against.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotionTimeScale {
    /// The time to fall the vehicle's own length under the central body's
    /// surface gravity. A body without gravity has none and is an error.
    SurfaceGravity,
    /// A period or time constant of the motion, s.
    Seconds(f64),
    /// A characteristic acceleration, m/s², such as the peak specific force.
    Acceleration(f64),
}

impl MotionTimeScale {
    fn resolve(self, body: &BodyEllipsoid) -> TimeScale {
        match self {
            Self::SurfaceGravity => {
                let equator = glam::DVec3::X * body.semi_major_m;
                TimeScale::Acceleration(body.gravity_fixed(equator).length())
            }
            Self::Seconds(seconds) => TimeScale::Seconds(seconds),
            Self::Acceleration(accel) => TimeScale::Acceleration(accel),
        }
    }
}

/// The multibody model of `tree`, its scale taken from the tree's extent and
/// mass and from `time`.
pub(crate) fn scaled_model(
    tree: &ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
    time: MotionTimeScale,
    body: &BodyEllipsoid,
) -> Result<MultibodyModel, MultibodyError> {
    MultibodyModel::with_derived_scale(tree, dynamic_masses, time.resolve(body))
}

/// The integrator a block keeps between steps. It holds scratch storage and
/// its configuration only, so it is not part of any snapshot.
#[derive(Debug, Clone)]
pub(crate) enum Stepper {
    FixedRk4(Rk4),
    Adaptive(DormandPrince54),
    Stiff(Rosenbrock23),
}

impl Stepper {
    pub(crate) fn new(choice: VehicleIntegrator) -> Self {
        match choice {
            VehicleIntegrator::FixedRk4 { substeps } => {
                Self::FixedRk4(Rk4::with_substeps(substeps.max(1)))
            }
            VehicleIntegrator::Adaptive(tolerances) => {
                Self::Adaptive(DormandPrince54::new(tolerances))
            }
            VehicleIntegrator::Stiff(tolerances) => Self::Stiff(Rosenbrock23::new(tolerances)),
        }
    }

    pub(crate) fn carried_for_spin(
        &self,
        omega_inertial_rad_s: glam::DVec3,
        dt_s: f64,
        near_stop: bool,
    ) -> Option<Self> {
        const MAX_RK4_PHASE_RAD: f64 = 0.25;
        let Self::FixedRk4(rk4) = self else {
            return None;
        };
        let substeps = rk4.substeps.max(1);
        let h = dt_s / f64::from(substeps);
        let spin = omega_inertial_rad_s.length();
        if !near_stop && spin * h <= MAX_RK4_PHASE_RAD {
            return None;
        }
        Some(Self::Adaptive(DormandPrince54::new(Tolerances::uniform(
            1e-11, 1e-11,
        ))))
    }
}

/// A failure of a step or a load read as the error of `block`. A failed
/// atmosphere query comes back as the atmosphere's own error.
pub(crate) fn step_error(block: &'static str, source: MultibodyError) -> BlockError {
    if let MultibodyError::Environment(failed) = &source
        && let Some(atmosphere) = failed.downcast_ref::<AtmosphereError>()
    {
        return BlockError::Environment {
            block,
            source: atmosphere.clone(),
        };
    }
    BlockError::Multibody { block, source }
}

/// Everything fixed over one step of a model.
pub(crate) struct StepSetup<'a, 'e, E: ?Sized> {
    pub(crate) model: &'a MultibodyModel,
    pub(crate) elements: &'a E,
    pub(crate) environment: &'a FlightEnvironment<'e>,
    pub(crate) dt_s: f64,
}

impl<'e, E: ForceElement<FlightEnvironment<'e>> + ?Sized> StepSetup<'_, 'e, E> {
    /// Advances `state` by one period. Its base is in the body's fixed axes
    /// before and after; in between the model is integrated in the
    /// environment's world axes, with time counted from the start of the
    /// step. On error `state` is as it was.
    pub(crate) fn advance(
        &self,
        stepper: &mut Stepper,
        state: &mut ModelState,
        work: &mut Workspace,
    ) -> Result<(), MultibodyError> {
        self.advance_then(stepper, state, work, |_, _| Ok(()))
    }

    /// As [`Self::advance`], calling `at_end` with the state the step
    /// reached while it is still in world axes, one period in.
    pub(crate) fn advance_then<T>(
        &self,
        stepper: &mut Stepper,
        state: &mut ModelState,
        work: &mut Workspace,
        at_end: impl FnOnce(&ModelState, &mut Workspace) -> Result<T, MultibodyError>,
    ) -> Result<T, MultibodyError> {
        let start = state.clone();
        let (model, elements, env) = (self.model, self.elements, self.environment);
        state.set_base(&fixed_to_instantaneous_inertial(&start.base(), &env.body));
        let advanced = match stepper {
            Stepper::FixedRk4(rk4) => {
                model.advance(rk4, elements, env, state, 0.0, self.dt_s, work)
            }
            Stepper::Adaptive(dp54) => {
                model.advance(dp54, elements, env, state, 0.0, self.dt_s, work)
            }
            Stepper::Stiff(ros23) => {
                model.advance(ros23, elements, env, state, 0.0, self.dt_s, work)
            }
        };
        match advanced.and_then(|_| at_end(state, work)) {
            Ok(read) => {
                let ended = inertial_to_fixed_after_step(&state.base(), &env.body, self.dt_s);
                state.set_base(&ended);
                Ok(read)
            }
            Err(error) => {
                // The base was re-expressed before the integrator saw it and
                // the read at the end may fail after it finished, so the
                // whole state goes back.
                *state = start;
                Err(error)
            }
        }
    }
}

//! Vessels flown by `sim-core` blocks in their central body's rotating frame.

use glam::DMat3;
use sim_core::{Block, RigidBody6DofBlock, RigidBodyInput, SimInstant, TickSpan, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_spacetime::SimEpoch;

use crate::UniverseError;

/// A vessel's flight dynamics in its central body's rotating frame: the
/// fidelity tier for atmospheric flight, surface contact and close
/// proximity. Steps on its own period, on the universe's master clock.
pub trait LocalPlant {
    /// The body whose fixed frame the state is in.
    fn body(&self) -> BodyEllipsoid;
    /// The native step.
    fn period(&self) -> TickSpan;
    /// The master-clock instant the state describes.
    fn now(&self) -> SimInstant;
    /// The vehicle state at [`Self::now`].
    fn state(&self) -> VehicleState;
    /// Total mass, kg, and inertia about the centre of gravity in body axes,
    /// kg·m², which an inertial tier needs to carry the attitude on.
    fn mass_and_inertia(&self) -> (f64, DMat3);
    /// Advances one period.
    fn step(&mut self) -> Result<(), UniverseError>;
}

/// Builds a vessel's local tier from a state handed down from orbit, at
/// `start` on the universe's clock.
pub type LocalFactory =
    Box<dyn Fn(&VehicleState, SimInstant, &SimEpoch) -> Result<Box<dyn LocalPlant>, UniverseError>>;

/// A [`RigidBody6DofBlock`] coasting with no thrust, started at `start`.
///
/// The block counts time from its own first tick; this keeps the offset so
/// its instants are the universe's.
pub struct CoastingRigidBody {
    block: RigidBody6DofBlock,
    body: BodyEllipsoid,
    start: SimInstant,
}

impl CoastingRigidBody {
    /// `block`, whose first tick is `start` on the universe's clock.
    #[must_use]
    pub fn new(block: RigidBody6DofBlock, body: BodyEllipsoid, start: SimInstant) -> Self {
        Self { block, body, start }
    }
}

impl LocalPlant for CoastingRigidBody {
    fn body(&self) -> BodyEllipsoid {
        self.body
    }

    fn period(&self) -> TickSpan {
        self.block.period()
    }

    fn now(&self) -> SimInstant {
        self.start
            .advanced(self.block.now().duration_since(SimInstant::START))
    }

    fn state(&self) -> VehicleState {
        self.block.output()
    }

    fn mass_and_inertia(&self) -> (f64, DMat3) {
        let mass = self.block.mass_properties();
        (mass.total_mass_kg, mass.inertia_cg_body)
    }

    fn step(&mut self) -> Result<(), UniverseError> {
        Ok(self.block.step(&RigidBodyInput::default())?)
    }
}

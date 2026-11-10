//! Vessels flown by `sim-core` blocks in their central body's rotating frame.

use std::any::Any;

use glam::DMat3;
use sim_core::{Block, RigidBody6DofBlock, RigidBodyInput, SimInstant, TickSpan, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_spacetime::SimEpoch;

use crate::UniverseError;

/// What orbital propagation needs to know of a vessel to carry it on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrbitalBody {
    /// Total mass, kg.
    pub mass_kg: f64,
    /// Inertia about the centre of gravity in body axes, kg·m².
    pub inertia_cg_body: DMat3,
    /// Radius of a sphere about the centre of gravity that encloses the
    /// vessel, m: what the orbital tier's proximity checks treat it as.
    pub bounding_radius_m: f64,
}

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
    /// The vessel as orbital propagation would carry it, or `None` for one
    /// that only flight dynamics can fly, which never leaves the local tier.
    fn orbital_body(&self) -> Option<OrbitalBody>;
    /// Whether something only flight dynamics model is acting on the
    /// vessel now, such as ground contact, air loads or a running engine. A
    /// tier policy leaves such a vessel in the local tier however high it
    /// is.
    fn holds_local_tier(&self) -> bool;
    /// Instantaneous aerodynamic dynamic pressure $q_\infty$ (`Pa`) acting on
    /// the vessel in the local tier (`0.0` for vacuum plants).
    fn dynamic_pressure_pa(&self) -> f64 {
        0.0
    }
    /// Whether the vessel is in contact with or pinned to the ground surface.
    fn is_ground_contact(&self) -> bool {
        false
    }
    /// Optional cannonball solar radiation pressure surface carried into the orbital tier.
    fn srp_surface(&self) -> Option<sim_orbital::SrpSurface> {
        None
    }
    /// Advances one period.
    fn step(&mut self) -> Result<(), UniverseError>;
    /// Advances by whole periods to the last step instant at or before
    /// `until`. A plant that keeps its own account of the time handed to
    /// it, to interpolate a display between steps, overrides this.
    fn advance_to(&mut self, until: SimInstant) -> Result<(), UniverseError> {
        while self.now().advanced(self.period()) <= until {
            self.step()?;
        }
        Ok(())
    }
    /// The plant as its own type, for [`crate::Universe::local`].
    fn as_any(&self) -> &dyn Any;
    /// The plant as its own type, for [`crate::Universe::local_mut`].
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

/// Builds a vessel's local tier from a state handed down from orbit: the
/// body it arrives at, its state in that body's fixed frame, and `start` on
/// the universe's clock. The body is the one the vessel orbits when it
/// comes down, which need not be the one it left.
pub type LocalFactory = Box<
    dyn Fn(
        BodyEllipsoid,
        &VehicleState,
        SimInstant,
        &SimEpoch,
    ) -> Result<Box<dyn LocalPlant>, UniverseError>,
>;

/// A [`RigidBody6DofBlock`] coasting with no thrust, started at `start`.
///
/// The block counts time from its own first tick; this keeps the offset so
/// its instants are the universe's.
pub struct CoastingRigidBody {
    block: RigidBody6DofBlock,
    body: BodyEllipsoid,
    start: SimInstant,
    bounding_radius_m: f64,
}

impl CoastingRigidBody {
    /// `block`, whose first tick is `start` on the universe's clock, inside
    /// a sphere of `bounding_radius_m` about its centre of gravity.
    #[must_use]
    pub fn new(
        block: RigidBody6DofBlock,
        body: BodyEllipsoid,
        start: SimInstant,
        bounding_radius_m: f64,
    ) -> Self {
        Self {
            block,
            body,
            start,
            bounding_radius_m,
        }
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

    fn orbital_body(&self) -> Option<OrbitalBody> {
        let mass = self.block.mass_properties();
        Some(OrbitalBody {
            mass_kg: mass.total_mass_kg,
            inertia_cg_body: mass.inertia_cg_body,
            bounding_radius_m: self.bounding_radius_m,
        })
    }

    fn holds_local_tier(&self) -> bool {
        false
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self) -> Result<(), UniverseError> {
        Ok(self.block.step(&RigidBodyInput::default())?)
    }
}

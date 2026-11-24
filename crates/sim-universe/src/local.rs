//! Vessels flown by `sim-core` blocks in their central body's rotating frame.

use std::any::Any;
use std::sync::Arc;

use glam::DMat3;
use sim_bodies::BodyDescriptor;
use sim_core::{Block, RigidBody6DofBlock, RigidBodyInput, SimInstant, TickSpan, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_spacetime::SimEpoch;

use crate::UniverseError;
use crate::carry::{CarriedLoads, CarryContext, PlantSnapshot};

mod multibody;

pub use multibody::{
    DEFAULT_CARRY_PERIOD, DEFAULT_MAX_CARRIED_THRUST_ACCEL_M_S2, MultibodyLocalPlant,
};

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
    /// vessel now, such as ground contact, air loads, or an engine whose
    /// thrust the plant cannot report through [`Self::carry`]. A tier policy
    /// leaves such a vessel in the local tier however high it is, and brings
    /// a carried one back down to it.
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
    /// What the vessel's surroundings do to it in the orbital tier besides
    /// gravity: by default its radiation-pressure surface alone. A plant that
    /// should decay in thin air, librate in the gravity gradient or feel the
    /// body's magnetic field names those loads here.
    fn environment_loads(&self) -> sim_orbital::EnvironmentLoads {
        self.srp_surface()
            .map(sim_orbital::EnvironmentLoadSpec::SolarRadiationPressure)
            .into_iter()
            .collect()
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
    /// How often the plant exchanges with the orbital tier while that tier
    /// flies it, or `None` for a plant that cannot be carried: one the
    /// universe drops on promotion and rebuilds from its factory on the way
    /// back. A whole number of [`Self::period`]s.
    fn carry_period(&self) -> Option<TickSpan> {
        None
    }
    /// Advances the plant's internals by whole periods to `context.at` with
    /// the base motion prescribed by `context`, and returns what the orbital
    /// tier holds from that instant: the mean loads the internals generated
    /// on the way, or the present ones when `context.at` is [`Self::now`].
    /// Called only at instants the orbital tier has committed.
    fn carry(&mut self, _context: &CarryContext) -> Result<CarriedLoads, UniverseError> {
        Err(UniverseError::NotCarryable)
    }
    /// Takes `state`, fixed to the body `described`, as the plant's motion
    /// at `at` and leaves its internals as they are, so the same plant flies
    /// on in the local tier. The body is the one the vessel orbits now,
    /// which need not be the plant's own: a plant that can fly about another
    /// body makes it its own, with the gravity and air of that catalog
    /// entry. `false` when it cannot, about a body it cannot move to or at
    /// an instant its internals have not reached; the universe then rebuilds
    /// the vessel from its factory.
    fn adopt_base(
        &mut self,
        _described: &Arc<BodyDescriptor>,
        _state: &VehicleState,
        _at: SimInstant,
    ) -> Result<bool, UniverseError> {
        Ok(false)
    }
    /// Everything [`Self::restore_internals`] needs to put the plant back
    /// exactly as it is, or `None` for a plant whose owner checkpoints it.
    fn snapshot_internals(&self) -> Option<PlantSnapshot> {
        None
    }
    /// Puts the plant back to `snapshot`; `false`, changing nothing, when the
    /// snapshot is not one of its own.
    fn restore_internals(&mut self, _snapshot: &PlantSnapshot) -> bool {
        false
    }
    /// The plant as its own type, for [`crate::Universe::local`].
    fn as_any(&self) -> &dyn Any;
    /// The plant as its own type, for [`crate::Universe::local_mut`].
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

/// Builds a vessel's local tier from a state handed down from orbit: the
/// catalog entry of the body it arrives at, its state in that body's fixed
/// frame, and `start` on the universe's clock. The body is the one the
/// vessel orbits when it comes down, which need not be the one it left; its
/// entry carries the gravity, magnetic field and air the new plant and its
/// devices are built on, the instances the orbital tier flew the vessel in.
pub type LocalFactory = Box<
    dyn Fn(
        &Arc<BodyDescriptor>,
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

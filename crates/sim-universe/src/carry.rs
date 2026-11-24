//! What a dormant plant and the orbital tier exchange.
//!
//! A vessel in the orbital tier is one rigid body to its integrator. A plant
//! that can be carried keeps its internals (tanks, bus, thermal state) alive
//! there: at each exchange instant it is told where the orbital tier has
//! flown it, advances its internals to that instant, and hands back the
//! loads and mass the orbital tier holds until the next exchange.

use std::any::Any;
use std::fmt;
use std::sync::Arc;

use glam::DVec3;
use sim_core::{SimInstant, VehicleState};
use sim_geo::BodyEllipsoid;

use crate::local::OrbitalBody;

/// A carried plant's surroundings at an exchange instant: the base motion the
/// orbital tier committed, and the environment the plant cannot work out for
/// itself.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct CarryContext {
    /// The exchange instant, which the plant's internals advance to.
    pub at: SimInstant,
    /// The body whose fixed frame [`Self::state`] is in: the plant's own.
    pub body: BodyEllipsoid,
    /// The vessel's state at [`Self::at`], as the orbital tier flew it.
    pub state: VehicleState,
    /// Unit vector towards the Sun in the body-fixed axes of [`Self::body`].
    pub sun_direction_fixed: DVec3,
    /// Solar irradiance with nothing in the way, W/m².
    pub solar_irradiance_w_m2: f64,
    /// Fraction of the solar disc in view, `0` in an umbra and `1` in full
    /// sunlight.
    pub solar_lit_fraction: f64,
    /// The magnetic field of [`Self::body`] at the vessel, in that body's
    /// fixed axes, nT: the catalog field the orbital tier's magnetic torques
    /// act in, on the date that tier evaluates it for. `None` where the
    /// field has no value, as on a date its model does not cover.
    pub magnetic_field_fixed_nt: Option<DVec3>,
}

/// What the orbital tier holds for a carried plant until the next exchange.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CarriedLoads {
    /// Force the vessel generates on itself (thrust), body axes, N.
    pub force_body_n: DVec3,
    /// Its moment about the centre of gravity, body axes, N·m.
    pub torque_body_nm: DVec3,
    /// The magnetic dipole moment its torquers hold, body axes, A·m². The
    /// orbital tier turns it into a torque in the field where the vessel is
    /// at every integration stage, so the moment is what a plant reports,
    /// never the torque it would make at the exchange instant.
    pub dipole_body_a_m2: DVec3,
    /// Angular impulse the motion of the vessel's joints since the last
    /// exchange puts on its body, about the centre of gravity, body axes,
    /// N·m·s. The orbital tier applies it once, at the exchange.
    pub angular_impulse_body_nms: DVec3,
    /// Angular momentum of the vessel's parts moving relative to its body
    /// axes now, about the centre of gravity, body axes, N·m·s. The orbital
    /// tier holds it in the vessel's gyroscopic coupling.
    pub internal_momentum_body_nms: DVec3,
    /// Mass properties at the exchange instant. The plant owns the mass: the
    /// orbital tier holds this constant and integrates no propellant flow.
    pub body: OrbitalBody,
}

/// A plant's internal state as only that plant understands it, kept by a
/// [`crate::UniverseCheckpoint`].
#[derive(Clone)]
pub struct PlantSnapshot(Arc<dyn Any + Send + Sync>);

impl PlantSnapshot {
    /// Wraps a plant's own snapshot type.
    #[must_use]
    pub fn new<T: Any + Send + Sync>(state: T) -> Self {
        Self(Arc::new(state))
    }

    /// The snapshot as the type its plant made it from, or `None` for
    /// another plant's.
    #[must_use]
    pub fn downcast_ref<T: Any>(&self) -> Option<&T> {
        self.0.downcast_ref()
    }
}

impl fmt::Debug for PlantSnapshot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("PlantSnapshot(..)")
    }
}

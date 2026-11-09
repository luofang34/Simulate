//! Every vessel of a run on one master clock, each in the fidelity tier that
//! suits where it is.

use std::collections::BTreeMap;
use std::sync::Arc;

use sim_core::{SimInstant, TickSpan, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_orbital::{
    BodyOrientationCatalog, CoastControlLaw, ContinuousPlantState, Dop853Tolerances,
    EphemerisCatalog, ForceModel, MassPropertiesModel, NaifId, PropellantMode, ReactionWheelMode,
    VesselId, propagate_plant_interval,
};
use sim_spacetime::{BodyRotationState, SimEpoch};

use crate::UniverseError;
use crate::handoff::{InertialExtras, naif_body, to_body_fixed, to_inertial};
use crate::local::{LocalFactory, LocalPlant};

/// Which dynamics currently fly a vessel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tier {
    /// `sim-core` flight dynamics in the central body's rotating frame.
    Local,
    /// `sim-orbital` propagation in ICRF axes about the central body.
    Orbital,
}

/// Where the orbital tier gets the solar system from, and how it steps.
pub struct OrbitalContext {
    /// Body positions.
    pub ephemeris: Arc<dyn EphemerisCatalog>,
    /// Body orientations; the handoff reads the same catalog, so a vessel
    /// does not jump when it changes tier.
    pub orientation: Arc<dyn BodyOrientationCatalog>,
    /// Gravity the orbital tier integrates.
    pub force_model: ForceModel,
    /// The orbital tier advances in whole quanta from quantum boundaries, so
    /// its states do not depend on how a frame loop slices time.
    pub quantum: TickSpan,
    /// Integrator tolerances.
    pub tolerances: Dop853Tolerances,
}

struct OrbitalVessel {
    state: ContinuousPlantState,
    body: BodyEllipsoid,
    mass: MassPropertiesModel,
    now: SimInstant,
}

enum Flight {
    Local(Box<dyn LocalPlant>),
    Orbital(Box<OrbitalVessel>),
}

struct Vessel {
    flight: Flight,
    rebuild: LocalFactory,
}

/// Every vessel of a run, advanced together on the master clock.
///
/// Vessels step in id order, each on its own period (local) or quantum
/// (orbital), so the result depends only on the instants asked for, never on
/// the order vessels were added or how the caller slices time.
pub struct Universe {
    epoch: SimEpoch,
    orbital: OrbitalContext,
    vessels: BTreeMap<VesselId, Vessel>,
    now: SimInstant,
}

impl Universe {
    /// An empty universe on `epoch`'s clock.
    #[must_use]
    pub fn new(epoch: SimEpoch, orbital: OrbitalContext) -> Self {
        Self {
            epoch,
            orbital,
            vessels: BTreeMap::new(),
            now: SimInstant::START,
        }
    }

    /// The instant every vessel has been advanced to.
    #[must_use]
    pub const fn now(&self) -> SimInstant {
        self.now
    }

    /// The run's clock.
    #[must_use]
    pub const fn epoch(&self) -> &SimEpoch {
        &self.epoch
    }

    /// Adds a vessel flown by `plant`; `rebuild` makes its local tier again
    /// when it comes back from orbit.
    pub fn insert_local(
        &mut self,
        id: VesselId,
        plant: Box<dyn LocalPlant>,
        rebuild: LocalFactory,
    ) -> Result<(), UniverseError> {
        if self.vessels.contains_key(&id) {
            return Err(UniverseError::DuplicateVessel(id));
        }
        let flight = Flight::Local(plant);
        self.vessels.insert(id, Vessel { flight, rebuild });
        Ok(())
    }

    /// The vessels, in stepping order.
    pub fn ids(&self) -> impl Iterator<Item = VesselId> + '_ {
        self.vessels.keys().copied()
    }

    /// Which tier flies `id`.
    pub fn tier(&self, id: VesselId) -> Result<Tier, UniverseError> {
        Ok(match self.vessel(id)?.flight {
            Flight::Local(_) => Tier::Local,
            Flight::Orbital(_) => Tier::Orbital,
        })
    }

    /// `id`'s state fixed to its central body, whichever tier flies it.
    pub fn vehicle_state(&self, id: VesselId) -> Result<VehicleState, UniverseError> {
        match &self.vessel(id)?.flight {
            Flight::Local(plant) => Ok(plant.state()),
            Flight::Orbital(orbit) => {
                let (naif, rotation) = self.rotation(orbit.body, orbit.now)?;
                Ok(to_body_fixed(&orbit.state, naif, &rotation)?.0)
            }
        }
    }

    /// Advances every vessel to `until`: local tiers by whole periods, the
    /// orbital tier by whole quanta.
    pub fn advance_to(&mut self, until: SimInstant) -> Result<(), UniverseError> {
        if until < self.now {
            return Err(UniverseError::Backwards);
        }
        for vessel in self.vessels.values_mut() {
            match &mut vessel.flight {
                Flight::Local(plant) => {
                    while plant.now().advanced(plant.period()) <= until {
                        plant.step()?;
                    }
                }
                Flight::Orbital(orbit) => {
                    advance_orbit(orbit, until, &self.orbital, &self.epoch)?;
                }
            }
        }
        self.now = until;
        Ok(())
    }

    /// Hands `id` from flight dynamics to orbital propagation, at the
    /// instant its local tier has reached. That instant must be on the
    /// orbital quantum grid.
    pub fn promote_to_orbital(&mut self, id: VesselId) -> Result<(), UniverseError> {
        let Flight::Local(plant) = &self.vessel(id)?.flight else {
            return Err(UniverseError::AlreadyInTier(id));
        };
        let at = plant.now();
        if !on_grid(at, self.orbital.quantum) {
            return Err(UniverseError::BetweenSteps { vessel: id });
        }
        let body = plant.body();
        let (naif, rotation) = self.rotation(body, at)?;
        let state = to_inertial(&plant.state(), naif, &rotation, InertialExtras::default());
        let (mass_kg, inertia) = plant.mass_and_inertia();
        let mass = MassPropertiesModel {
            dry_mass_kg: mass_kg,
            dry_inertia_cols: inertia.to_cols_array_2d(),
            propellant_inertia_per_kg_cols: [[0.0; 3]; 3],
            uncompensated_idot_factor: 0.0,
        };
        let orbit = OrbitalVessel {
            state,
            body,
            mass,
            now: at,
        };
        self.vessel_mut(id)?.flight = Flight::Orbital(Box::new(orbit));
        Ok(())
    }

    /// Hands `id` from orbital propagation back to flight dynamics, built by
    /// the vessel's factory at the orbital tier's current instant.
    pub fn demote_to_local(&mut self, id: VesselId) -> Result<(), UniverseError> {
        let vessel = self.vessel(id)?;
        let Flight::Orbital(orbit) = &vessel.flight else {
            return Err(UniverseError::AlreadyInTier(id));
        };
        let (naif, rotation) = self.rotation(orbit.body, orbit.now)?;
        let (state, _extras) = to_body_fixed(&orbit.state, naif, &rotation)?;
        let plant = (vessel.rebuild)(&state, orbit.now, &self.epoch)?;
        self.vessel_mut(id)?.flight = Flight::Local(plant);
        Ok(())
    }

    fn rotation(
        &self,
        body: BodyEllipsoid,
        at: SimInstant,
    ) -> Result<(NaifId, BodyRotationState), UniverseError> {
        let naif = naif_body(body).ok_or(UniverseError::UnmodelledBody)?;
        let rotation = self
            .orbital
            .orientation
            .body_rotation(naif, self.epoch.tdb_at(at))?;
        Ok((naif, rotation))
    }

    fn vessel(&self, id: VesselId) -> Result<&Vessel, UniverseError> {
        self.vessels
            .get(&id)
            .ok_or(UniverseError::UnknownVessel(id))
    }

    fn vessel_mut(&mut self, id: VesselId) -> Result<&mut Vessel, UniverseError> {
        self.vessels
            .get_mut(&id)
            .ok_or(UniverseError::UnknownVessel(id))
    }
}

fn on_grid(at: SimInstant, quantum: TickSpan) -> bool {
    let q = quantum.ticks_u128();
    q != 0 && at.ticks_u128().is_multiple_of(q)
}

fn advance_orbit(
    orbit: &mut OrbitalVessel,
    until: SimInstant,
    context: &OrbitalContext,
    epoch: &SimEpoch,
) -> Result<(), UniverseError> {
    while orbit.now.advanced(context.quantum) <= until {
        let next = orbit.now.advanced(context.quantum);
        let out = propagate_plant_interval(
            &orbit.state,
            epoch.tdb_at(orbit.now),
            epoch.tdb_at(next),
            None,
            &context.tolerances,
            &orbit.mass,
            PropellantMode::InfiniteConstantMass,
            ReactionWheelMode::ConserveMomentum {
                h_max_nms: 0.0,
                tau_max_nm: 0.0,
            },
            &context.force_model,
            context.ephemeris.as_ref(),
            context.orientation.as_ref(),
            &CoastControlLaw,
        )
        .map_err(|e| UniverseError::Propagation(Box::new(e)))?;
        orbit.state = out.final_state;
        orbit.now = next;
    }
    Ok(())
}

#[cfg(test)]
mod tests;

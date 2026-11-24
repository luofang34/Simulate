//! Taking a [`Universe`] back to an earlier instant.

use std::collections::BTreeMap;

use sim_core::SimInstant;
use sim_orbital::{UniverseSnapshot, VesselId};

use super::{Flight, Tier, Universe};
use crate::UniverseError;
use crate::bus::VesselBusSnapshot;
use crate::carry::PlantSnapshot;
use crate::local::LocalPlant;
use crate::planning::PlannerCheckpoint;

/// A [`Universe`] at one instant: its clock, every orbital vessel with its
/// pending events and commands, which tier each vessel was in, and its plans:
/// their schedules and the requests still waiting for their deadlines.
///
/// A plant that offers [`LocalPlant::snapshot_internals`] is in here too,
/// whichever tier it is in, and [`Universe::restore`] puts it back. Any
/// other local plant is not: whoever built it checkpoints it, because only
/// they know its type, and puts it back before calling [`Universe::restore`].
#[derive(Debug, Clone)]
pub struct UniverseCheckpoint {
    now: SimInstant,
    orbital: UniverseSnapshot,
    tiers: BTreeMap<VesselId, Tier>,
    buses: BTreeMap<VesselId, VesselBusSnapshot>,
    plants: BTreeMap<VesselId, PlantSnapshot>,
    planner: PlannerCheckpoint,
}

impl UniverseCheckpoint {
    /// The instant the checkpoint was taken at.
    #[must_use]
    pub const fn at(&self) -> SimInstant {
        self.now
    }
}

impl Universe {
    /// The universe as it is now, for [`Self::restore`].
    #[must_use]
    pub fn checkpoint(&self) -> UniverseCheckpoint {
        let buses = self
            .vessels
            .iter()
            .map(|(&id, vessel)| (id, vessel.bus.snapshot()))
            .collect();
        let plants = self
            .vessels
            .iter()
            .filter_map(|(&id, vessel)| Some((id, vessel.flight.plant()?.snapshot_internals()?)))
            .collect();
        UniverseCheckpoint {
            now: self.now,
            orbital: self.orbital.capture_snapshot(),
            tiers: self.tiers(),
            buses,
            plants,
            planner: self.planner.checkpoint(),
        }
    }

    /// Takes the universe back to `checkpoint`. Every vessel must be in the
    /// tier it was in then; every plant the checkpoint holds must still be
    /// there, every carried plant must be one it holds, and every other local
    /// plant must already have been put back to an instant no later than the
    /// checkpoint's; otherwise nothing changes. A plant that then refuses its
    /// own saved state is reported after the orbital tier has gone back.
    pub fn restore(&mut self, checkpoint: &UniverseCheckpoint) -> Result<(), UniverseError> {
        if self.tiers() != checkpoint.tiers {
            return Err(UniverseError::TiersChangedSinceCheckpoint);
        }
        for (&vessel, entry) in &self.vessels {
            let saved = checkpoint.plants.contains_key(&vessel);
            match &entry.flight {
                Flight::Local(plant) if !saved && plant.now() > checkpoint.now => {
                    return Err(UniverseError::LocalAheadOfCheckpoint { vessel });
                }
                Flight::Orbital { carried } if saved != carried.is_some() => {
                    return Err(UniverseError::TiersChangedSinceCheckpoint);
                }
                Flight::Local(_) | Flight::Orbital { .. } => {}
            }
        }
        self.orbital
            .restore_snapshot(&checkpoint.orbital)
            .map_err(|e| UniverseError::Scheduler(Box::new(e)))?;
        for (&vessel, entry) in &mut self.vessels {
            if let Some(saved) = checkpoint.plants.get(&vessel)
                && let Some(plant) = entry.flight.plant_mut()
                && !plant.restore_internals(saved)
            {
                return Err(UniverseError::PlantSnapshotMismatch { vessel });
            }
        }
        for (&id, entry) in &mut self.vessels {
            if let Some(bus_snap) = checkpoint.buses.get(&id) {
                entry.bus.restore(bus_snap);
            }
        }
        self.now = checkpoint.now;
        self.planner.restore(&checkpoint.planner);
        self.tier_changes
            .retain(|change| change.at <= checkpoint.now);
        self.state_reinitializations = self.state_reinitializations.wrapping_add(1);
        Ok(())
    }

    fn tiers(&self) -> BTreeMap<VesselId, Tier> {
        self.vessels
            .iter()
            .map(|(&id, vessel)| {
                let tier = match vessel.flight {
                    Flight::Local(_) => Tier::Local,
                    Flight::Orbital { .. } => Tier::Orbital,
                };
                (id, tier)
            })
            .collect()
    }
}

impl Flight {
    /// The vessel's plant, flying it or carried.
    fn plant(&self) -> Option<&dyn LocalPlant> {
        match self {
            Self::Local(plant) => Some(plant.as_ref()),
            Self::Orbital { carried } => carried.as_deref(),
        }
    }

    fn plant_mut(&mut self) -> Option<&mut (dyn LocalPlant + 'static)> {
        match self {
            Self::Local(plant) => Some(plant.as_mut()),
            Self::Orbital { carried } => carried.as_deref_mut(),
        }
    }
}

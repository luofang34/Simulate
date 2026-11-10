//! Taking a [`Universe`] back to an earlier instant.

use std::collections::BTreeMap;

use sim_core::SimInstant;
use sim_orbital::{UniverseSnapshot, VesselId};

use super::{Flight, Tier, Universe};
use crate::UniverseError;

/// A [`Universe`] at one instant: its clock, every orbital vessel with its
/// pending events and commands, and which tier each vessel was in.
///
/// A local plant's state is not in here. Whoever built the plant
/// checkpoints it, because only they know its type, and puts it back
/// before calling [`Universe::restore`].
#[derive(Debug, Clone, PartialEq)]
pub struct UniverseCheckpoint {
    now: SimInstant,
    orbital: UniverseSnapshot,
    tiers: BTreeMap<VesselId, Tier>,
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
        UniverseCheckpoint {
            now: self.now,
            orbital: self.orbital.capture_snapshot(),
            tiers: self.tiers(),
        }
    }

    /// Takes the universe back to `checkpoint`. Every vessel must be in the
    /// tier it was in then, and every local plant must already have been
    /// put back to an instant no later than the checkpoint's; otherwise
    /// nothing changes.
    pub fn restore(&mut self, checkpoint: &UniverseCheckpoint) -> Result<(), UniverseError> {
        if self.tiers() != checkpoint.tiers {
            return Err(UniverseError::TiersChangedSinceCheckpoint);
        }
        for (&vessel, entry) in &self.vessels {
            if let Flight::Local(plant) = &entry.flight
                && plant.now() > checkpoint.now
            {
                return Err(UniverseError::LocalAheadOfCheckpoint { vessel });
            }
        }
        self.orbital
            .restore_snapshot(&checkpoint.orbital)
            .map_err(|e| UniverseError::Scheduler(Box::new(e)))?;
        self.now = checkpoint.now;
        self.tier_changes
            .retain(|change| change.at <= checkpoint.now);
        Ok(())
    }

    fn tiers(&self) -> BTreeMap<VesselId, Tier> {
        self.vessels
            .iter()
            .map(|(&id, vessel)| {
                let tier = match vessel.flight {
                    Flight::Local(_) => Tier::Local,
                    Flight::Orbital => Tier::Orbital,
                };
                (id, tier)
            })
            .collect()
    }
}

//! Taking a [`Universe`] back to an earlier instant, including active
//! inter-vessel links and spawned/retired proximity topology.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use sim_core::{EllipsoidSurfaceProvider, MultibodyVehicleBlock, SimInstant, VehicleState};
use sim_orbital::{UniverseSnapshot, VesselId};

use super::proximity_step::ExtractedPlantMeta;
use super::{Flight, RetiredVessel, Tier, Universe, Vessel};
use crate::UniverseError;
use crate::bus::VesselBusSnapshot;
use crate::carry::PlantSnapshot;
use crate::local::{LocalPlant, MultibodyLocalPlant};
use crate::planning::PlannerCheckpoint;
use crate::proximity::{ProximityStepReport, UniverseInterVesselLink};

/// A [`Universe`] at one instant: its clock, every orbital vessel with its
/// pending events and commands, active inter-vessel links and topology counters,
/// which tier each vessel was in, and its plans.
#[derive(Debug, Clone)]
pub struct UniverseCheckpoint {
    now: SimInstant,
    orbital: UniverseSnapshot,
    tiers: BTreeMap<VesselId, Tier>,
    buses: BTreeMap<VesselId, VesselBusSnapshot>,
    plants: BTreeMap<VesselId, PlantSnapshot>,
    carried_samples: BTreeMap<VesselId, Vec<(SimInstant, VehicleState)>>,
    planner: PlannerCheckpoint,
    links: Vec<UniverseInterVesselLink>,
    active_vessel: Option<VesselId>,
    next_vessel_id: u64,
    next_link_id: u64,
    proximity_spawned: BTreeSet<VesselId>,
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
        let carried_samples = self
            .vessels
            .iter()
            .filter_map(|(&id, vessel)| match &vessel.flight {
                Flight::Orbital { samples, .. } if !samples.is_empty() => {
                    Some((id, samples.clone()))
                }
                _ => None,
            })
            .collect();
        UniverseCheckpoint {
            now: self.now,
            orbital: self.orbital.capture_snapshot(),
            tiers: self.tiers(),
            buses,
            plants,
            carried_samples,
            planner: self.planner.checkpoint(),
            links: self.links.clone(),
            active_vessel: self.active_vessel,
            next_vessel_id: self.next_vessel_id,
            next_link_id: self.next_link_id,
            proximity_spawned: self.proximity_spawned.clone(),
        }
    }

    /// Takes the universe back to `checkpoint`, restoring vessel rosters, active
    /// inter-vessel links, and plant states cleanly across spawns, hard-mates,
    /// and tether cuts while rejecting manual tier divergence.
    pub fn restore(&mut self, checkpoint: &UniverseCheckpoint) -> Result<(), UniverseError> {
        self.validate_restore(checkpoint)?;
        self.orbital
            .restore_snapshot(&checkpoint.orbital)
            .map_err(|e| UniverseError::Scheduler(Box::new(e)))?;
        self.reconcile_restore_roster(checkpoint)?;
        self.restore_vessel_payloads(checkpoint)?;
        self.now = checkpoint.now;
        self.planner.restore(&checkpoint.planner);
        self.tier_changes
            .retain(|change| change.at <= checkpoint.now);
        self.links = checkpoint.links.clone();
        self.active_vessel = checkpoint.active_vessel;
        self.next_vessel_id = checkpoint.next_vessel_id;
        self.next_link_id = checkpoint.next_link_id;
        self.proximity_spawned = checkpoint.proximity_spawned.clone();
        self.auto_demoted
            .retain(|id| checkpoint.tiers.get(id) == Some(&Tier::Local));
        self.proximity_report = ProximityStepReport::default();
        self.state_reinitializations = self.state_reinitializations.wrapping_add(1);
        Ok(())
    }

    fn validate_restore(&self, checkpoint: &UniverseCheckpoint) -> Result<(), UniverseError> {
        for (&vessel, entry) in &self.vessels {
            let Some(&saved_tier) = checkpoint.tiers.get(&vessel) else {
                if !self.proximity_spawned.contains(&vessel) && !self.retired_ever.contains(&vessel)
                {
                    return Err(UniverseError::TiersChangedSinceCheckpoint);
                }
                continue;
            };
            let cur_tier = entry.flight.tier();
            if cur_tier != saved_tier
                && !(cur_tier == Tier::Local
                    && saved_tier == Tier::Orbital
                    && self.auto_demoted.contains(&vessel))
            {
                return Err(UniverseError::TiersChangedSinceCheckpoint);
            }
            let saved = checkpoint.plants.contains_key(&vessel);
            match &entry.flight {
                Flight::Local(plant) if !saved && plant.now() > checkpoint.now => {
                    return Err(UniverseError::LocalAheadOfCheckpoint { vessel });
                }
                Flight::Orbital { carried, .. } if saved != carried.is_some() => {
                    return Err(UniverseError::TiersChangedSinceCheckpoint);
                }
                Flight::Local(_) | Flight::Orbital { .. } => {}
            }
        }
        for &vessel in checkpoint.tiers.keys() {
            if !self.vessels.contains_key(&vessel) && !self.retired_vessels.contains_key(&vessel) {
                return Err(UniverseError::TiersChangedSinceCheckpoint);
            }
        }
        Ok(())
    }

    fn reconcile_restore_roster(
        &mut self,
        checkpoint: &UniverseCheckpoint,
    ) -> Result<(), UniverseError> {
        let extra: Vec<VesselId> = self
            .vessels
            .keys()
            .copied()
            .filter(|id| !checkpoint.tiers.contains_key(id))
            .collect();
        for id in extra {
            if let Some(vessel) = self.vessels.remove(&id) {
                let plant = match vessel.flight {
                    Flight::Local(plant) => Some(plant),
                    Flight::Orbital { carried, .. } => carried,
                };
                if let Some(plant) = plant
                    && plant.as_any().is::<MultibodyLocalPlant>()
                    && let Ok(mb) =
                        (plant as Box<dyn std::any::Any>).downcast::<MultibodyLocalPlant>()
                {
                    let (block, input, body, start, radius, carry_period, max_carried) =
                        mb.into_parts();
                    self.retired_vessels.insert(
                        id,
                        RetiredVessel {
                            meta: ExtractedPlantMeta {
                                snapshot: block.structural_snapshot(),
                                input,
                                body,
                                start,
                                bounding_radius_m: radius,
                                carry_period,
                                max_carried,
                                bus: vessel.bus,
                                rebuild: vessel.rebuild,
                            },
                            survivor_world_donor: checkpoint.active_vessel.unwrap_or(id),
                        },
                    );
                }
            }
        }
        let missing: Vec<VesselId> = checkpoint
            .tiers
            .keys()
            .copied()
            .filter(|id| !self.vessels.contains_key(id))
            .collect();
        for id in missing {
            let Some(retired) = self.retired_vessels.remove(&id) else {
                return Err(UniverseError::TiersChangedSinceCheckpoint);
            };
            let vessel = self.rebuild_retired_vessel(id, retired)?;
            self.vessels.insert(id, vessel);
        }
        Ok(())
    }

    fn rebuild_retired_vessel(
        &self,
        id: VesselId,
        retired: RetiredVessel,
    ) -> Result<Vessel, UniverseError> {
        let meta = retired.meta;
        let donor = self
            .local::<MultibodyLocalPlant>(retired.survivor_world_donor)
            .or_else(|| self.carried::<MultibodyLocalPlant>(retired.survivor_world_donor))
            .filter(|p| p.body() == meta.body);
        let block = if let Some(donor) = donor {
            donor
                .block()
                .from_structural_sharing_world(&meta.snapshot)?
        } else if let Some(described) = self
            .bodies
            .naif_of(&meta.body)
            .and_then(|n| self.bodies.get(n))
        {
            MultibodyVehicleBlock::from_structural_snapshot(
                &meta.snapshot,
                described.atmosphere().clone(),
                described.fields().gravity,
                Arc::new(EllipsoidSurfaceProvider::solid(described.ellipsoid)),
            )?
        } else {
            return Err(UniverseError::NotMultibodyPlant(id));
        };
        let plant = MultibodyLocalPlant::from_parts(
            block,
            meta.input,
            meta.body,
            meta.start,
            meta.bounding_radius_m,
            meta.carry_period,
            meta.max_carried,
        );
        Ok(Vessel {
            flight: Flight::Local(Box::new(plant)),
            rebuild: meta.rebuild,
            bus: meta.bus,
        })
    }

    fn restore_vessel_payloads(
        &mut self,
        checkpoint: &UniverseCheckpoint,
    ) -> Result<(), UniverseError> {
        for (&vessel, &saved_tier) in &checkpoint.tiers {
            let entry = self
                .vessels
                .get_mut(&vessel)
                .ok_or(UniverseError::TiersChangedSinceCheckpoint)?;
            if entry.flight.tier() == Tier::Local && saved_tier == Tier::Orbital {
                let Flight::Local(plant) = std::mem::replace(
                    &mut entry.flight,
                    Flight::Orbital {
                        carried: None,
                        samples: Vec::new(),
                    },
                ) else {
                    return Err(UniverseError::TiersChangedSinceCheckpoint);
                };
                if checkpoint.plants.contains_key(&vessel) {
                    entry.flight = Flight::Orbital {
                        carried: Some(plant),
                        samples: Vec::new(),
                    };
                }
            }
            if let Some(saved) = checkpoint.plants.get(&vessel)
                && let Some(plant) = entry.flight.plant_mut()
                && !plant.restore_internals(saved)
            {
                return Err(UniverseError::PlantSnapshotMismatch { vessel });
            }
            if let Flight::Orbital { samples, .. } = &mut entry.flight {
                samples.clear();
                if let Some(saved_samples) = checkpoint.carried_samples.get(&vessel) {
                    samples.extend_from_slice(saved_samples);
                }
            }
            if let Some(bus_snap) = checkpoint.buses.get(&vessel) {
                entry.bus.restore(bus_snap);
            }
        }
        Ok(())
    }

    fn tiers(&self) -> BTreeMap<VesselId, Tier> {
        self.vessels
            .iter()
            .map(|(&id, vessel)| (id, vessel.flight.tier()))
            .collect()
    }
}

impl Flight {
    const fn tier(&self) -> Tier {
        match self {
            Self::Local(_) => Tier::Local,
            Self::Orbital { .. } => Tier::Orbital,
        }
    }

    /// The vessel's plant, flying it or carried.
    fn plant(&self) -> Option<&dyn LocalPlant> {
        match self {
            Self::Local(plant) => Some(plant.as_ref()),
            Self::Orbital { carried, .. } => carried.as_deref(),
        }
    }

    fn plant_mut(&mut self) -> Option<&mut (dyn LocalPlant + 'static)> {
        match self {
            Self::Local(plant) => Some(plant.as_mut()),
            Self::Orbital { carried, .. } => carried.as_deref_mut(),
        }
    }
}

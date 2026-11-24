//! Synchronous connected-component stepping and coordinator state exchange
//! for linked vessels in [`Universe`].

use std::collections::{BTreeMap, BTreeSet};

use mbd_multibody::TetherCutReason;
use sim_core::{
    InterVesselLink, MultiVesselCoordinator, MultibodyStructuralSnapshot, MultibodyVehicleBlock,
    MultibodyVehicleInput, SimInstant, SimVesselId, TickSpan,
};
use sim_geo::BodyEllipsoid;
use sim_orbital::VesselId;

use super::{Flight, Universe, Vessel};
use crate::UniverseError;
use crate::bus::{VesselBusConfig, VesselBusState};
use crate::local::MultibodyLocalPlant;
use crate::proximity::{LinkId, ProximityStepReport, UniverseInterVesselLink, UniverseLinkSpec};

pub(super) struct ExtractedPlantMeta {
    pub(super) snapshot: MultibodyStructuralSnapshot,
    pub(super) input: MultibodyVehicleInput,
    pub(super) body: BodyEllipsoid,
    pub(super) start: SimInstant,
    pub(super) bounding_radius_m: f64,
    pub(super) carry_period: Option<TickSpan>,
    pub(super) max_carried: f64,
    pub(super) bus: VesselBusState,
    pub(super) rebuild: crate::LocalFactory,
}

#[derive(Clone, Copy)]
pub(super) struct SpawnTemplate {
    pub(super) body: BodyEllipsoid,
    pub(super) start: SimInstant,
    pub(super) radius: f64,
    pub(super) carry_period: Option<TickSpan>,
    pub(super) max_carried: f64,
    pub(super) bus: VesselBusConfig,
}

struct ScheduledComponent {
    next_tick: SimInstant,
    vessels: Vec<VesselId>,
}

impl Universe {
    /// Advances all local-tier vessels up to whole periods at or before `until`,
    /// stepping linked components synchronously through [`MultiVesselCoordinator`]
    /// while leaving uncoupled vessels on their native direct path.
    pub(super) fn advance_local_to(&mut self, until: SimInstant) -> Result<(), UniverseError> {
        if self.links.is_empty() {
            for vessel in self.vessels.values_mut() {
                if let Flight::Local(plant) = &mut vessel.flight {
                    plant.advance_to(until)?;
                }
            }
            return Ok(());
        }
        while let Some(next) = self.next_due_linked_component(until)? {
            self.step_linked_component(&next.vessels)?;
        }
        let uncoupled: Vec<VesselId> = self
            .vessels
            .keys()
            .copied()
            .filter(|&id| !self.is_vessel_in_any_link(id))
            .collect();
        for id in uncoupled {
            if let Some(Vessel {
                flight: Flight::Local(plant),
                ..
            }) = self.vessels.get_mut(&id)
            {
                plant.advance_to(until)?;
            }
        }
        Ok(())
    }

    fn next_due_linked_component(
        &self,
        until: SimInstant,
    ) -> Result<Option<ScheduledComponent>, UniverseError> {
        let components = self.active_link_components();
        let mut best: Option<ScheduledComponent> = None;
        for vessels in components {
            let Some(&first) = vessels.first() else {
                continue;
            };
            let Flight::Local(p0) = &self.vessel(first)?.flight else {
                return Err(UniverseError::VesselCoupled(first));
            };
            let common_now = p0.now();
            let period = p0.period();
            for &id in &vessels {
                let Flight::Local(p) = &self.vessel(id)?.flight else {
                    return Err(UniverseError::VesselCoupled(id));
                };
                if p.now() != common_now || p.period() != period {
                    return Err(UniverseError::BetweenSteps { vessel: id });
                }
            }
            if period.ticks_u128() == 0 {
                continue;
            }
            let next_tick = common_now.advanced(period);
            if next_tick <= until
                && best.as_ref().is_none_or(|b| {
                    (next_tick, first) < (b.next_tick, b.vessels.first().copied().unwrap_or(first))
                })
            {
                best = Some(ScheduledComponent { next_tick, vessels });
            }
        }
        Ok(best)
    }

    fn active_link_components(&self) -> Vec<Vec<VesselId>> {
        let mut adj: BTreeMap<VesselId, BTreeSet<VesselId>> = BTreeMap::new();
        for link in &self.links {
            if !link.spec.is_active() {
                continue;
            }
            match link.spec.endpoints() {
                (a, Some(b)) => {
                    adj.entry(a).or_default().insert(b);
                    adj.entry(b).or_default().insert(a);
                }
                (a, None) => {
                    adj.entry(a).or_default();
                }
            }
        }
        let mut seen = BTreeSet::new();
        let mut components = Vec::new();
        let nodes: Vec<VesselId> = adj.keys().copied().collect();
        for start in nodes {
            if !seen.insert(start) {
                continue;
            }
            let mut stack = vec![start];
            let mut comp = Vec::new();
            while let Some(cur) = stack.pop() {
                comp.push(cur);
                if let Some(neighbors) = adj.get(&cur) {
                    for &nbr in neighbors {
                        if seen.insert(nbr) {
                            stack.push(nbr);
                        }
                    }
                }
            }
            comp.sort_unstable();
            components.push(comp);
        }
        components
    }

    fn step_linked_component(&mut self, vessels: &[VesselId]) -> Result<(), UniverseError> {
        let mut inputs = BTreeMap::new();
        for &id in vessels {
            let plant = self
                .local::<MultibodyLocalPlant>(id)
                .ok_or(UniverseError::NotMultibodyPlant(id))?;
            inputs.insert(SimVesselId(id.0), plant.input().clone());
        }
        let (mut meta_map, mut coord, untouched_links) =
            self.extract_component_to_coordinator(vessels, true)?;
        let res = coord.step_all(&inputs);
        let topo_changed = res
            .as_ref()
            .is_ok_and(|r| !r.spawned_vessels.is_empty() || !r.merged_vessels.is_empty());
        let parent_hint = res
            .as_ref()
            .ok()
            .and_then(|r| r.spawned_vessels.first())
            .map(|(p, _)| VesselId(p.0));
        let report = self.reabsorb_from_coordinator(
            coord,
            &mut meta_map,
            untouched_links,
            topo_changed,
            parent_hint,
            res,
        )?;
        for (chaser, target) in &report.merged_vessels {
            let (cid, tid) = (VesselId(chaser.0), VesselId(target.0));
            self.archive_retired_target(tid, meta_map.remove(&tid), cid);
        }
        self.proximity_report.extend(ProximityStepReport {
            spawned_vessels: report
                .spawned_vessels
                .iter()
                .map(|(p, c)| (VesselId(p.0), VesselId(c.0)))
                .collect(),
            merged_vessels: report
                .merged_vessels
                .iter()
                .map(|(c, t)| (VesselId(c.0), VesselId(t.0)))
                .collect(),
            cut_tethers: report
                .cut_tethers
                .iter()
                .map(|(id, r): &(u64, TetherCutReason)| (LinkId(*id), *r))
                .collect(),
        });
        Ok(())
    }

    pub(super) fn extract_to_coordinator(
        &mut self,
        ids: &[VesselId],
    ) -> Result<
        (
            BTreeMap<VesselId, ExtractedPlantMeta>,
            MultiVesselCoordinator,
        ),
        UniverseError,
    > {
        let (meta_map, coord, _) = self.extract_component_to_coordinator(ids, false)?;
        Ok((meta_map, coord))
    }

    #[allow(clippy::type_complexity)]
    fn extract_component_to_coordinator(
        &mut self,
        ids: &[VesselId],
        subset_links_only: bool,
    ) -> Result<
        (
            BTreeMap<VesselId, ExtractedPlantMeta>,
            MultiVesselCoordinator,
            Vec<UniverseInterVesselLink>,
        ),
        UniverseError,
    > {
        let mut meta_map = BTreeMap::new();
        let mut blocks = BTreeMap::new();
        for &id in ids {
            let block = self
                .extract_one_vessel(id, &mut meta_map)
                .inspect_err(|_| {
                    self.restore_extracted_on_err(&mut meta_map, core::mem::take(&mut blocks));
                })?;
            blocks.insert(SimVesselId(id.0), block);
        }
        let id_set: BTreeSet<VesselId> = ids.iter().copied().collect();
        let mut core_links: Vec<InterVesselLink> = Vec::new();
        let mut untouched = Vec::new();
        for link in &self.links {
            let (a, opt_b) = link.spec.endpoints();
            let belongs = id_set.contains(&a) || opt_b.is_some_and(|b| id_set.contains(&b));
            if !subset_links_only || belongs {
                core_links.push(link.spec.to_core_link(link.id.0));
            } else {
                untouched.push(link.clone());
            }
        }
        let active_sim = ids.first().map_or(SimVesselId(1), |v| SimVesselId(v.0));
        let coord =
            MultiVesselCoordinator::from_parts(blocks, core_links, active_sim, self.next_vessel_id)
                .map_err(|err| {
                    // Nothing lost: caller can only hit this if ids were invalid, which is impossible.
                    UniverseError::Block(err)
                })?;
        Ok((meta_map, coord, untouched))
    }

    fn extract_one_vessel(
        &mut self,
        id: VesselId,
        meta_map: &mut BTreeMap<VesselId, ExtractedPlantMeta>,
    ) -> Result<MultibodyVehicleBlock, UniverseError> {
        let vessel = self
            .vessels
            .remove(&id)
            .ok_or(UniverseError::UnknownVessel(id))?;
        let Flight::Local(plant) = vessel.flight else {
            let is_orb = matches!(vessel.flight, Flight::Orbital { .. });
            self.vessels.insert(id, vessel);
            return Err(if is_orb {
                UniverseError::VesselCoupled(id)
            } else {
                UniverseError::NotMultibodyPlant(id)
            });
        };
        if !plant.as_any().is::<MultibodyLocalPlant>() {
            self.vessels.insert(
                id,
                Vessel {
                    flight: Flight::Local(plant),
                    rebuild: vessel.rebuild,
                    bus: vessel.bus,
                },
            );
            return Err(UniverseError::NotMultibodyPlant(id));
        }
        let Ok(mb_plant) = (plant as Box<dyn std::any::Any>).downcast::<MultibodyLocalPlant>()
        else {
            return Err(UniverseError::NotMultibodyPlant(id));
        };
        let (block, input, body, start, bounding_radius_m, carry_period, max_carried) =
            mb_plant.into_parts();
        let snapshot = block.structural_snapshot();
        meta_map.insert(
            id,
            ExtractedPlantMeta {
                snapshot,
                input,
                body,
                start,
                bounding_radius_m,
                carry_period,
                max_carried,
                bus: vessel.bus,
                rebuild: vessel.rebuild,
            },
        );
        Ok(block)
    }

    fn restore_extracted_on_err(
        &mut self,
        meta_map: &mut BTreeMap<VesselId, ExtractedPlantMeta>,
        blocks: BTreeMap<SimVesselId, MultibodyVehicleBlock>,
    ) {
        for (sid, block) in blocks {
            let vid = VesselId(sid.0);
            if let Some(meta) = meta_map.remove(&vid) {
                let plant = MultibodyLocalPlant::from_parts(
                    block,
                    meta.input,
                    meta.body,
                    meta.start,
                    meta.bounding_radius_m,
                    meta.carry_period,
                    meta.max_carried,
                );
                self.vessels.insert(
                    vid,
                    Vessel {
                        flight: Flight::Local(Box::new(plant)),
                        rebuild: meta.rebuild,
                        bus: meta.bus,
                    },
                );
            }
        }
    }

    pub(super) fn reabsorb_from_coordinator<R>(
        &mut self,
        coord: MultiVesselCoordinator,
        meta_map: &mut BTreeMap<VesselId, ExtractedPlantMeta>,
        untouched_links: Vec<UniverseInterVesselLink>,
        topology_changed: bool,
        spawn_parent: Option<VesselId>,
        outcome: Result<R, sim_core::BlockError>,
    ) -> Result<R, UniverseError> {
        let (mut out_blocks, out_links, _active, next_vid) = coord.into_parts();
        let ok = match outcome {
            Ok(v) => v,
            Err(err) => {
                self.restore_extracted_on_err(meta_map, out_blocks);
                return Err(UniverseError::Block(err));
            }
        };
        let parent_tpl = spawn_parent
            .and_then(|pid| meta_map.get(&pid))
            .or_else(|| meta_map.values().next())
            .map(|m| SpawnTemplate {
                body: m.body,
                start: m.start,
                radius: m.bounding_radius_m,
                carry_period: m.carry_period,
                max_carried: m.max_carried,
                bus: m.bus.config(),
            });
        let mut live_ids =
            self.reinsert_existing_vessels(meta_map, &mut out_blocks, topology_changed);
        if let Some(tpl) = parent_tpl {
            self.register_spawned_vessels(out_blocks, tpl, &mut live_ids);
        }
        self.next_vessel_id = self.next_vessel_id.max(next_vid);
        let mut rebuilt_links: Vec<UniverseInterVesselLink> = out_links
            .iter()
            .map(|l| UniverseInterVesselLink {
                id: LinkId(l.id()),
                spec: UniverseLinkSpec::from_core_link(l),
            })
            .collect();
        rebuilt_links.extend(untouched_links);
        rebuilt_links.sort_by_key(|l| l.id);
        self.links = rebuilt_links;
        for vid in live_ids {
            self.refresh_vessel_bus(vid);
        }
        Ok(ok)
    }

    fn reinsert_existing_vessels(
        &mut self,
        meta_map: &mut BTreeMap<VesselId, ExtractedPlantMeta>,
        out_blocks: &mut BTreeMap<SimVesselId, MultibodyVehicleBlock>,
        topology_changed: bool,
    ) -> Vec<VesselId> {
        let mut live_ids = Vec::with_capacity(out_blocks.len());
        let existing_keys: Vec<VesselId> = meta_map.keys().copied().collect();
        for id in existing_keys {
            let Some(block) = out_blocks.remove(&SimVesselId(id.0)) else {
                continue;
            };
            if let Some(meta) = meta_map.remove(&id) {
                let plant = MultibodyLocalPlant::from_parts(
                    block,
                    meta.input,
                    meta.body,
                    meta.start,
                    meta.bounding_radius_m,
                    meta.carry_period,
                    meta.max_carried,
                );
                let rebuild = if topology_changed {
                    plant.rebuild_factory()
                } else {
                    meta.rebuild
                };
                self.vessels.insert(
                    id,
                    Vessel {
                        flight: Flight::Local(Box::new(plant)),
                        rebuild,
                        bus: meta.bus,
                    },
                );
                live_ids.push(id);
            }
        }
        live_ids
    }
}

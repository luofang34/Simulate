//! First-class staging, undocking, soft-capture docking, hard-mate,
//! grapple-lock, and tether command operations on [`Universe`].

use std::collections::BTreeMap;

use glam::DVec3;
use mbd_core::{ComponentId, Placement};
use mbd_multibody::{GrappleLockSpec, TetherCommand, TetherCutReason};
use sim_core::{
    BlockError, MultiVesselCoordinator, MultibodyVehicleBlock, MultibodyVehicleInput, SimInstant,
    SimVesselId,
};
use sim_orbital::VesselId;

use super::proximity_step::{ExtractedPlantMeta, SpawnTemplate};
use super::{Flight, RetiredVessel, Universe, Vessel};
use crate::UniverseError;
use crate::bus::{VesselBusConfig, VesselBusState};
use crate::local::{LocalPlant, MultibodyLocalPlant};
use crate::proximity::{DockingLinkSpec, LinkId, UniverseInterVesselLink, UniverseLinkSpec};

impl Universe {
    /// Registers a new mechanism or inter-vessel link, automatically demoting any
    /// carried orbital endpoint to the local tier at [`Self::now`].
    pub fn add_inter_vessel_link(
        &mut self,
        spec: UniverseLinkSpec,
    ) -> Result<LinkId, UniverseError> {
        let (a, opt_b) = spec.endpoints();
        if let Some(b) = opt_b {
            if a == b {
                return Err(UniverseError::SameVesselLink(a));
            }
            self.precheck_multibody_candidate(a)?;
            self.precheck_multibody_candidate(b)?;
            self.ensure_multibody_local(a)?;
            self.ensure_multibody_local(b)?;
            self.validate_pair_compatibility(a, b)?;
        } else {
            self.precheck_multibody_candidate(a)?;
            self.ensure_multibody_local(a)?;
        }
        let id = LinkId(self.next_link_id);
        self.next_link_id = self.next_link_id.wrapping_add(1);
        self.links.push(UniverseInterVesselLink { id, spec });
        if self.active_vessel.is_none() {
            self.active_vessel = Some(a);
        }
        Ok(id)
    }

    /// Registers a compliant soft-capture docking link between two vessels.
    pub fn dock_soft_capture(&mut self, spec: DockingLinkSpec) -> Result<LinkId, UniverseError> {
        self.add_inter_vessel_link(UniverseLinkSpec::Docking(spec))
    }

    /// Updates the active command held on winch tether `id` for subsequent steps.
    pub fn set_tether_command(
        &mut self,
        id: LinkId,
        command: TetherCommand,
    ) -> Result<(), UniverseError> {
        let link = self
            .links
            .iter_mut()
            .find(|l| l.id == id)
            .ok_or(UniverseError::UnknownLink(id.0))?;
        let UniverseLinkSpec::WinchTether { command: slot, .. } = &mut link.spec else {
            return Err(UniverseError::NotATetherLink(id.0));
        };
        *slot = command;
        Ok(())
    }

    /// Immediately severs winch tether `id`, removing it from active coupling so both
    /// vessels become eligible for orbital promotion again.
    pub fn cut_tether(&mut self, id: LinkId) -> Result<(), UniverseError> {
        let pos = self
            .links
            .iter()
            .position(|l| l.id == id)
            .ok_or(UniverseError::UnknownLink(id.0))?;
        let is_tether = matches!(
            self.links.get(pos).map(|l| &l.spec),
            Some(UniverseLinkSpec::WinchTether { .. })
        );
        if !is_tether {
            return Err(UniverseError::NotATetherLink(id.0));
        }
        let removed = self.links.remove(pos);
        if matches!(removed.spec, UniverseLinkSpec::WinchTether { state, .. } if state.intact) {
            self.proximity_report
                .cut_tethers
                .push((id, TetherCutReason::Commanded));
        }
        Ok(())
    }

    /// Removes and returns link `id` if present.
    pub fn remove_inter_vessel_link(&mut self, id: LinkId) -> Option<UniverseInterVesselLink> {
        self.links
            .iter()
            .position(|l| l.id == id)
            .map(|pos| self.links.remove(pos))
    }

    /// Separates `child_root` from `parent` via a pyrotechnic/spring impulse, conserving
    /// linear and angular momentum across subsystems and registering the child at [`Self::now`].
    pub fn separate_stage_impulse(
        &mut self,
        parent: VesselId,
        child_root: ComponentId,
        impulse_ns: f64,
        axis_in_child: DVec3,
    ) -> Result<VesselId, UniverseError> {
        self.precheck_multibody_candidate(parent)?;
        self.ensure_multibody_local(parent)?;
        self.run_split_action(parent, |coord, sid| {
            coord.separate_stage_impulse(sid, child_root, impulse_ns, axis_in_child)
        })
    }

    /// Detaches an attached/docked `child_root` subtree via `sim-core` undocking release.
    pub fn undock_subtree(
        &mut self,
        parent: VesselId,
        child_root: ComponentId,
        impulse_ns: f64,
        axis_in_child: DVec3,
    ) -> Result<VesselId, UniverseError> {
        self.precheck_multibody_candidate(parent)?;
        self.ensure_multibody_local(parent)?;
        self.run_split_action(parent, |coord, sid| {
            coord.undock_subtree(sid, child_root, impulse_ns, axis_in_child)
        })
    }

    /// Structurally hard-mates `target` (`port_b`) onto `chaser` (`port_a`) conserving momentum,
    /// retiring `target` and remapping active links cleanly.
    pub fn hard_mate(
        &mut self,
        chaser: VesselId,
        port_a: ComponentId,
        target: VesselId,
        port_b: ComponentId,
        mate_placement: Placement,
    ) -> Result<(), UniverseError> {
        self.prepare_mate_pair(chaser, target)?;
        self.run_mate_action(chaser, target, |coord, sc, st| {
            coord.hard_mate(sc, port_a, st, port_b, mate_placement)
        })
    }

    /// Validates robotic grapple envelope on `manipulator_name` and rigidly locks `target`
    /// onto `servicer`'s end-effector component.
    pub fn grapple_lock(
        &mut self,
        spec: &GrappleLockSpec,
        servicer: VesselId,
        manipulator_name: &str,
        target: VesselId,
        fixture_component: ComponentId,
        mate_placement: Placement,
    ) -> Result<(), UniverseError> {
        self.prepare_mate_pair(servicer, target)?;
        self.run_mate_action(servicer, target, |coord, ss, st| {
            coord.grapple_lock_target(
                spec,
                ss,
                manipulator_name,
                st,
                fixture_component,
                mate_placement,
            )
        })
    }

    fn prepare_mate_pair(&mut self, a: VesselId, b: VesselId) -> Result<(), UniverseError> {
        if a == b {
            return Err(UniverseError::SameVesselLink(a));
        }
        self.precheck_multibody_candidate(a)?;
        self.precheck_multibody_candidate(b)?;
        self.ensure_multibody_local(a)?;
        self.ensure_multibody_local(b)?;
        self.validate_pair_compatibility(a, b)
    }

    pub(super) fn precheck_multibody_candidate(&self, id: VesselId) -> Result<(), UniverseError> {
        let vessel = self.vessel(id)?;
        match &vessel.flight {
            Flight::Local(plant) => {
                if !plant.as_any().is::<MultibodyLocalPlant>() {
                    return Err(UniverseError::NotMultibodyPlant(id));
                }
                if plant.now() != self.now {
                    return Err(UniverseError::BetweenSteps { vessel: id });
                }
            }
            Flight::Orbital { carried, .. } => {
                if let Some(plant) = carried.as_deref()
                    && !plant.as_any().is::<MultibodyLocalPlant>()
                {
                    return Err(UniverseError::NotMultibodyPlant(id));
                }
            }
        }
        Ok(())
    }

    pub(super) fn ensure_multibody_local(&mut self, id: VesselId) -> Result<(), UniverseError> {
        if matches!(self.vessel(id)?.flight, Flight::Orbital { .. }) {
            self.demote_to_local(id)?;
            self.auto_demoted.insert(id);
        }
        let Flight::Local(plant) = &self.vessel(id)?.flight else {
            return Err(UniverseError::NotMultibodyPlant(id));
        };
        if !plant.as_any().is::<MultibodyLocalPlant>() {
            return Err(UniverseError::NotMultibodyPlant(id));
        }
        if plant.now() != self.now {
            return Err(UniverseError::BetweenSteps { vessel: id });
        }
        Ok(())
    }

    fn validate_pair_compatibility(&self, a: VesselId, b: VesselId) -> Result<(), UniverseError> {
        let (Flight::Local(pa), Flight::Local(pb)) =
            (&self.vessel(a)?.flight, &self.vessel(b)?.flight)
        else {
            return Err(UniverseError::NotMultibodyPlant(a));
        };
        if pa.body() != pb.body() {
            return Err(UniverseError::IncompatibleCoupling {
                vessel: b,
                detail: "central body does not match partner vessel",
            });
        }
        if pa.period() != pb.period() {
            return Err(UniverseError::IncompatibleCoupling {
                vessel: b,
                detail: "step period does not match partner vessel",
            });
        }
        Ok(())
    }

    fn run_split_action(
        &mut self,
        parent: VesselId,
        op: impl FnOnce(&mut MultiVesselCoordinator, SimVesselId) -> Result<SimVesselId, BlockError>,
    ) -> Result<VesselId, UniverseError> {
        let (mut meta_map, mut coord) = self.extract_to_coordinator(&[parent])?;
        let res = op(&mut coord, SimVesselId(parent.0));
        let child_sim = self.reabsorb_from_coordinator(
            coord,
            &mut meta_map,
            Vec::new(),
            true,
            Some(parent),
            res,
        )?;
        let child_id = VesselId(child_sim.0);
        self.proximity_report
            .spawned_vessels
            .push((parent, child_id));
        Ok(child_id)
    }

    fn run_mate_action(
        &mut self,
        chaser: VesselId,
        target: VesselId,
        op: impl FnOnce(&mut MultiVesselCoordinator, SimVesselId, SimVesselId) -> Result<(), BlockError>,
    ) -> Result<(), UniverseError> {
        let (mut meta_map, mut coord) = self.extract_to_coordinator(&[chaser, target])?;
        let res = op(&mut coord, SimVesselId(chaser.0), SimVesselId(target.0));
        self.reabsorb_from_coordinator(coord, &mut meta_map, Vec::new(), true, Some(chaser), res)?;
        self.archive_retired_target(target, meta_map.remove(&target), chaser);
        self.proximity_report.merged_vessels.push((chaser, target));
        Ok(())
    }

    pub(super) fn register_spawned_vessels(
        &mut self,
        out_blocks: BTreeMap<SimVesselId, MultibodyVehicleBlock>,
        tpl: SpawnTemplate,
        live_ids: &mut Vec<VesselId>,
    ) {
        for (sid, block) in out_blocks {
            let cid = VesselId(sid.0);
            let spawn_now = tpl
                .start
                .advanced(block.now().duration_since(SimInstant::START));
            let plant = MultibodyLocalPlant::from_parts(
                block,
                MultibodyVehicleInput::default(),
                tpl.body,
                tpl.start,
                tpl.radius,
                tpl.carry_period,
                tpl.max_carried,
            );
            let srp_surface = plant.srp_surface();
            let rebuild = plant.rebuild_factory();
            let bus = VesselBusState::new(
                VesselBusConfig {
                    srp_surface,
                    ..tpl.bus
                },
                spawn_now,
            );
            self.vessels.insert(
                cid,
                Vessel {
                    flight: Flight::Local(Box::new(plant)),
                    rebuild,
                    bus,
                },
            );
            self.proximity_spawned.insert(cid);
            live_ids.push(cid);
        }
    }

    pub(super) fn archive_retired_target(
        &mut self,
        target: VesselId,
        meta: Option<ExtractedPlantMeta>,
        survivor: VesselId,
    ) {
        if self.active_vessel == Some(target) {
            self.active_vessel = Some(survivor);
        }
        self.retired_ever.insert(target);
        if let Some(meta) = meta {
            self.retired_vessels.insert(
                target,
                RetiredVessel {
                    meta,
                    survivor_world_donor: survivor,
                },
            );
        }
    }
}

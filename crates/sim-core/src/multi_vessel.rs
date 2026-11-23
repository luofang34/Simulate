use std::collections::BTreeMap;

use glam::DVec3;
use mbd_core::{ComponentId, Placement};
use mbd_multibody::{
    DockingSpec, DockingState, GrappleLockSpec, HingeSwingSpec, HingeSwingState, RailSlideSpec,
    RailSlideState, TetherCommand, TetherCutReason, WinchTetherSpec, WinchTetherState,
    execute_impulse_release, grapple_lock, hard_mate_vessels, undock_vessel,
};
use mbd_systems::payload::transfer_payload_mass;

use crate::BlockError;
use crate::flight_model::planet_frame::fixed_to_instantaneous_inertial;
use crate::multibody_vehicle::{MultibodyVehicleBlock, MultibodyVehicleInput};
use mechanisms::{
    mate_vessels_from_result, prepare_step_inputs, process_post_step_links,
    split_vessel_with_release, take_link_states,
};
use plume::site_world_kinematics;

mod mechanisms;
mod plume;

#[cfg(test)]
mod tests;

pub(super) const COORD_BLOCK: &str = "multi_vessel_coordinator";

/// Unique identifier assigned to each vehicle tracked by [`MultiVesselCoordinator`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SimVesselId(pub u64);

pub(super) fn missing_vessel(id: SimVesselId) -> BlockError {
    BlockError::InputOutOfRange {
        block: COORD_BLOCK,
        input: "vessel_id",
        detail: format!("unknown vessel #{}", id.0),
    }
}

pub(super) fn map_multi_coord(source: mbd_multibody::MultibodyError) -> BlockError {
    BlockError::Multibody {
        block: COORD_BLOCK,
        source,
    }
}

pub(super) fn map_sys_coord(source: mbd_systems::SystemsError) -> BlockError {
    BlockError::Systems {
        block: COORD_BLOCK,
        source,
    }
}

/// Physical mechanism or field link coupled between vessel components across ticks.
#[derive(Debug, Clone, PartialEq)]
pub enum InterVesselLink {
    /// Prismatic seat/rail ejection actuator driving a joint until rail exit (`Release`).
    RailSlide {
        /// Unique link identifier.
        id: u64,
        /// Parent vessel carrying the rail joint.
        vessel: SimVesselId,
        /// Rail length, catapult force, and sliding friction specification.
        spec: RailSlideSpec,
        /// Current rail slide state (`fired`, `ready_to_release`). The link is
        /// dropped once the seat has left the rail.
        state: RailSlideState,
    },
    /// Hinged fairing/petal swing mechanism separating automatically past `release_angle_rad`.
    HingeSwing {
        /// Unique link identifier.
        id: u64,
        /// Parent vessel carrying the hinged component.
        vessel: SimVesselId,
        /// Hinge swing angle and spring/damper specification.
        spec: HingeSwingSpec,
        /// Current hinge state (`unlatched`, `ready_to_release`). The link is
        /// dropped once the panel has separated.
        state: HingeSwingState,
    },
    /// Compliant soft-capture docking interface converging to rigid `hard_mate`.
    Docking {
        /// Unique link identifier.
        id: u64,
        /// Chaser/active vessel.
        chaser: SimVesselId,
        /// Docking port component on `chaser`.
        port_a: ComponentId,
        /// Port ring origin in `port_a` local axes, m.
        offset_a_local_m: DVec3,
        /// Outward unit capture normal in `port_a` local axes.
        axis_a_local: DVec3,
        /// Target/passive vessel.
        target: SimVesselId,
        /// Docking port component on `target`.
        port_b: ComponentId,
        /// Port ring origin in `port_b` local axes, m.
        offset_b_local_m: DVec3,
        /// Relative mated placement of `target` port root in `port_a` axes at hard latch.
        mate_placement: Placement,
        /// Capture envelope, stiffness, damping, and undock spring specification.
        spec: DockingSpec,
        /// Mutable docking phase, standoff, and settle dwell state.
        state: DockingState,
    },
    /// Variable-length Sky Crane bridle or spacecraft umbilical with thermal burn-through & auto-cut.
    WinchTether {
        /// Unique link identifier.
        id: u64,
        /// Carrier/stage vessel holding the winch spool.
        carrier: SimVesselId,
        /// Anchor component on `carrier`.
        anchor_carrier: ComponentId,
        /// Anchor offset in `anchor_carrier` local axes, m.
        offset_carrier_m: DVec3,
        /// Suspended payload/capsule vessel.
        payload: SimVesselId,
        /// Anchor component on `payload`.
        anchor_payload: ComponentId,
        /// Anchor offset in `anchor_payload` local axes, m.
        offset_payload_m: DVec3,
        /// Tensile stiffness, breaking tension, burn-through energy, and touchdown auto-cut spec.
        spec: WinchTetherSpec,
        /// Deployed length, absorbed heat, and intact/cut status.
        state: WinchTetherState,
        /// Winch payout rate, umbilical heating and cutter command held over
        /// each step.
        command: TetherCommand,
    },
    /// Thruster exhaust plume impingement and heating transferred onto a nearby vessel surface.
    PlumeLink {
        /// Unique link identifier.
        id: u64,
        /// Vessel firing the thruster.
        source_vessel: SimVesselId,
        /// Nozzle component on `source_vessel`.
        nozzle_site: ComponentId,
        /// Nozzle exit origin in `nozzle_site` local axes, m.
        nozzle_offset_local_m: DVec3,
        /// Unit exhaust flow direction in `nozzle_site` local axes.
        exhaust_dir_local: DVec3,
        /// Nozzle exit radius, m.
        nozzle_radius_m: f64,
        /// Plume divergence half-angle, rad.
        cone_half_angle_rad: f64,
        /// Exhaust jet thermal power ratio relative to thrust (`thermal_w / thrust_n`).
        exhaust_thermal_w_per_n: f64,
        /// Target vessel struck by the exhaust cone.
        target_vessel: SimVesselId,
        /// Struck target component on `target_vessel`.
        target_site: ComponentId,
        /// Surface patch centroid in `target_site` local axes, m.
        target_point_local_m: DVec3,
        /// Outward unit surface normal in `target_site` local axes.
        target_normal_local: DVec3,
        /// Struck target surface area, m².
        target_area_m2: f64,
        /// Surface thermal absorptivity `[0..=1]`.
        thermal_absorptivity: f64,
    },
}

/// Topology transitions emitted on one coordinated step of [`MultiVesselCoordinator`].
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct MultiVesselStepReport {
    /// New `(parent_id, spawned_child_id)` vessels created by rail, hinge, or stage release.
    pub spawned_vessels: Vec<(SimVesselId, SimVesselId)>,
    /// Merged `(chaser_id, retired_target_id)` pairs hard-mated this step.
    pub merged_vessels: Vec<(SimVesselId, SimVesselId)>,
    /// Severed `(link_id, reason)` umbilical/winch tether events this step.
    pub cut_tethers: Vec<(u64, TetherCutReason)>,
}

/// Deterministic multi-vessel coordinator managing staging, docking, rail/hinge ejection,
/// winch tethers, plume impingement, robotic grapple, and KSP-style vessel switching.
pub struct MultiVesselCoordinator {
    pub(super) vessels: BTreeMap<SimVesselId, MultibodyVehicleBlock>,
    pub(super) links: Vec<InterVesselLink>,
    pub(super) pending_releases: Vec<(SimVesselId, mbd_core::ComponentId, f64, DVec3)>,
    pub(super) pending_hard_mates: Vec<(
        SimVesselId,
        ComponentId,
        SimVesselId,
        ComponentId,
        Placement,
    )>,
    pub(super) active_vessel: SimVesselId,
    pub(super) next_vessel_id: u64,
}

impl MultiVesselCoordinator {
    /// Initializes a coordinator owning `initial_vessel` as [`SimVesselId`] `1` and active focus.
    #[must_use]
    pub fn new(initial_vessel: MultibodyVehicleBlock) -> Self {
        let first = SimVesselId(1);
        let mut vessels = BTreeMap::new();
        vessels.insert(first, initial_vessel);
        Self {
            vessels,
            links: Vec::new(),
            pending_releases: Vec::new(),
            pending_hard_mates: Vec::new(),
            active_vessel: first,
            next_vessel_id: 2,
        }
    }

    /// Registers an independent free-flying vessel and returns its assigned [`SimVesselId`].
    pub fn spawn_independent(&mut self, block: MultibodyVehicleBlock) -> SimVesselId {
        let id = SimVesselId(self.next_vessel_id);
        self.next_vessel_id = self.next_vessel_id.wrapping_add(1);
        self.vessels.insert(id, block);
        id
    }

    /// Immutable reference to vessel `id`.
    #[must_use]
    pub fn vessel(&self, id: SimVesselId) -> Option<&MultibodyVehicleBlock> {
        self.vessels.get(&id)
    }

    /// Mutable reference to vessel `id`.
    pub fn vessel_mut(&mut self, id: SimVesselId) -> Option<&mut MultibodyVehicleBlock> {
        self.vessels.get_mut(&id)
    }

    /// Deterministic sorted list of currently live [`SimVesselId`]s.
    #[must_use]
    pub fn vessel_ids(&self) -> Vec<SimVesselId> {
        self.vessels.keys().copied().collect()
    }

    /// Currently controlled/focused vessel ID (KSP-style vessel switching).
    #[must_use]
    pub const fn active_vessel(&self) -> SimVesselId {
        self.active_vessel
    }

    /// Focuses control on existing vessel `id`.
    pub fn set_active_vessel(&mut self, id: SimVesselId) -> Result<(), BlockError> {
        if !self.vessels.contains_key(&id) {
            return Err(missing_vessel(id));
        }
        self.active_vessel = id;
        Ok(())
    }

    /// Cycles active focus to the next live [`SimVesselId`] in sorted order.
    pub fn cycle_active_vessel(&mut self) -> SimVesselId {
        let ids = self.vessel_ids();
        if let Some(pos) = ids.iter().position(|&id| id == self.active_vessel)
            && let Some(&next) = ids.get((pos.wrapping_add(1)) % ids.len().max(1))
        {
            self.active_vessel = next;
        } else if let Some(&first) = ids.first() {
            self.active_vessel = first;
        }
        self.active_vessel
    }

    /// Registers an [`InterVesselLink`] evaluated on every [`Self::step_all`] call.
    pub fn add_link(&mut self, link: InterVesselLink) {
        self.links.push(link);
    }

    /// Mutable slice of active [`InterVesselLink`] records (to arm/cut tethers or fire rails).
    pub fn links_mut(&mut self) -> &mut [InterVesselLink] {
        &mut self.links
    }

    /// Transfers conserved payload mass between attached components on `vessel_id`.
    pub fn transfer_payload(
        &mut self,
        vessel_id: SimVesselId,
        from_comp: ComponentId,
        to_comp: ComponentId,
        mass_kg: f64,
    ) -> Result<(), BlockError> {
        let v = self
            .vessels
            .get_mut(&vessel_id)
            .ok_or_else(|| missing_vessel(vessel_id))?;
        let mut temp_tree = v.tree().clone();
        transfer_payload_mass(&mut temp_tree, from_comp, to_comp, mass_kg, true)
            .map_err(map_sys_coord)?;
        let states = temp_tree.states().to_vec();
        v.set_component_states(&states)
    }

    /// Separates `child_root` from `parent_id` with an internal pyrotechnic/spring impulse,
    /// partitioning fluids/propulsion/aero/thermal/electrical onto Parent and Child with exact
    /// momentum conservation, returning the spawned child [`SimVesselId`].
    pub fn separate_stage_impulse(
        &mut self,
        parent_id: SimVesselId,
        child_root: ComponentId,
        impulse_ns: f64,
        axis_in_child: DVec3,
    ) -> Result<SimVesselId, BlockError> {
        let parent = self
            .vessels
            .get(&parent_id)
            .ok_or_else(|| missing_vessel(parent_id))?;
        let parent_i =
            fixed_to_instantaneous_inertial(&parent.floating_base_fixed(), &parent.body());
        let masses = parent.dynamic_mass_contributions()?;
        let release = execute_impulse_release(
            parent.tree(),
            &masses,
            &parent_i,
            child_root,
            impulse_ns,
            axis_in_child,
        )
        .map_err(map_multi_coord)?;
        split_vessel_with_release(self, parent_id, release)
    }

    /// Separates a docked or grappled target subtree via `mbd_multibody::undock_vessel`
    /// (preserving high spin rates like Gemini 8 60 rpm centrifugal undocking).
    pub fn undock_subtree(
        &mut self,
        parent_id: SimVesselId,
        child_root: ComponentId,
        impulse_ns: f64,
        axis_in_child: DVec3,
    ) -> Result<SimVesselId, BlockError> {
        let parent = self
            .vessels
            .get(&parent_id)
            .ok_or_else(|| missing_vessel(parent_id))?;
        let parent_i =
            fixed_to_instantaneous_inertial(&parent.floating_base_fixed(), &parent.body());
        let masses = parent.dynamic_mass_contributions()?;
        let release = undock_vessel(
            parent.tree(),
            &masses,
            &parent_i,
            child_root,
            impulse_ns,
            axis_in_child,
        )
        .map_err(map_multi_coord)?;
        split_vessel_with_release(self, parent_id, release)
    }

    /// Structurally hard-mates `target_id` (`port_b`) onto `chaser_id` (`port_a`) conserving linear
    /// and angular momentum, merging fluid/electrical/thermal/propulsion/aero subsystems cleanly.
    pub fn hard_mate(
        &mut self,
        chaser_id: SimVesselId,
        port_a: ComponentId,
        target_id: SimVesselId,
        port_b: ComponentId,
        mate_placement: Placement,
    ) -> Result<(), BlockError> {
        let chaser = self
            .vessels
            .get(&chaser_id)
            .ok_or_else(|| missing_vessel(chaser_id))?;
        let target = self
            .vessels
            .get(&target_id)
            .ok_or_else(|| missing_vessel(target_id))?;
        let chaser_i =
            fixed_to_instantaneous_inertial(&chaser.floating_base_fixed(), &chaser.body());
        let target_i =
            fixed_to_instantaneous_inertial(&target.floating_base_fixed(), &target.body());
        let ma = chaser.dynamic_mass_contributions()?;
        let mb = target.dynamic_mass_contributions()?;
        let res = hard_mate_vessels(
            chaser.tree(),
            &ma,
            &chaser_i,
            port_a,
            target.tree(),
            &mb,
            &target_i,
            port_b,
            mate_placement,
        )
        .map_err(map_multi_coord)?;
        mate_vessels_from_result(self, chaser_id, target_id, res)
    }

    /// Validates robotic grapple capture envelope on `manipulator_name` and rigidly mates
    /// `target_id` onto the end-effector component.
    pub fn grapple_lock_target(
        &mut self,
        spec: &GrappleLockSpec,
        servicer_id: SimVesselId,
        manipulator_name: &str,
        target_id: SimVesselId,
        fixture_component: ComponentId,
        mate_placement: Placement,
    ) -> Result<(), BlockError> {
        let servicer = self
            .vessels
            .get(&servicer_id)
            .ok_or_else(|| missing_vessel(servicer_id))?;
        let target = self
            .vessels
            .get(&target_id)
            .ok_or_else(|| missing_vessel(target_id))?;
        let chain = servicer
            .subsystems()
            .manipulators
            .iter()
            .find(|(n, _)| n == manipulator_name)
            .map(|(_, c)| c)
            .ok_or_else(|| BlockError::InputOutOfRange {
                block: COORD_BLOCK,
                input: "manipulator_name",
                detail: format!("unknown manipulator {manipulator_name:?}"),
            })?;
        let ee_comp = chain.ee_component();
        let servicer_i =
            fixed_to_instantaneous_inertial(&servicer.floating_base_fixed(), &servicer.body());
        let target_i =
            fixed_to_instantaneous_inertial(&target.floating_base_fixed(), &target.body());
        let (ee_pos, ee_rot, ee_vel, _) = site_world_kinematics(
            servicer.pose_table(),
            &servicer_i,
            ee_comp,
            chain.ee_offset_local_m(),
        )?;
        let (fix_pos, fix_rot, fix_vel, _) = site_world_kinematics(
            target.pose_table(),
            &target_i,
            fixture_component,
            DVec3::ZERO,
        )?;
        let ma = servicer.dynamic_mass_contributions()?;
        let mb = target.dynamic_mass_contributions()?;
        let res = grapple_lock(
            spec,
            ee_pos,
            ee_rot,
            ee_vel,
            fix_pos,
            fix_rot,
            fix_vel,
            servicer.tree(),
            &ma,
            &servicer_i,
            ee_comp,
            target.tree(),
            &mb,
            &target_i,
            fixture_component,
            mate_placement,
        )
        .map_err(map_multi_coord)?;
        mate_vessels_from_result(self, servicer_id, target_id, res)
    }

    /// Evaluates active inter-vessel mechanisms, advances every live vessel synchronously on the
    /// shared master clock, and executes triggered separations/hard-mates/tether cuts.
    pub fn step_all(
        &mut self,
        inputs: &BTreeMap<SimVesselId, MultibodyVehicleInput>,
    ) -> Result<MultiVesselStepReport, BlockError> {
        let (mut step_inputs, owners) = prepare_step_inputs(self, inputs)?;
        let ids = self.vessel_ids();
        for id in ids {
            if let Some(vessel) = self.vessels.get_mut(&id) {
                let input = step_inputs.remove(&id).unwrap_or_default();
                crate::Block::step(vessel, &input)?;
            }
        }
        take_link_states(self, &owners);
        process_post_step_links(self)
    }
}

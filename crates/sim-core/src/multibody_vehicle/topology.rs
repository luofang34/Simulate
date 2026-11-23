use mbd_aero::AeroElement;
use mbd_core::{ComponentId, ComponentTree, FaultState};
use mbd_multibody::{HydrodynamicsModel, LoopConstraint, ManipulatorChain};
use mbd_systems::{
    AbepThrusterSpec, AirbreathingEngineSpec, ElectricThrusterSpec, LiquidRocketSpec, TankId,
};

use super::element_state::CanopyState;
use super::subsystems::{MultibodySubsystems, PropulsionAssembly, lookup_mapped_comp};

/// A vehicle's aerodynamic elements, each with the canopy state it holds.
pub(crate) type VehicleAero = Vec<(AeroElement, CanopyState)>;

fn map_opt_comp(map: &[(ComponentId, ComponentId)], old: ComponentId) -> Option<ComponentId> {
    lookup_mapped_comp(map, old)
}

fn remap_tank(map: &[(TankId, TankId)], old: TankId) -> TankId {
    map.iter()
        .find(|&&(k, _)| k == old)
        .map_or(old, |&(_, v)| v)
}

fn remap_opt_tank(map: &[(TankId, TankId)], old: Option<TankId>) -> Option<TankId> {
    old.map(|tid| remap_tank(map, tid))
}

impl PropulsionAssembly {
    /// Extracts engines/thrusters attached to components in `comp_map`, remapping site IDs.
    #[must_use]
    pub fn remap_subset(&self, comp_map: &[(ComponentId, ComponentId)]) -> Self {
        let mut out = Self::default();
        append_propulsion(self, comp_map, &[], &mut out);
        out
    }

    /// Merges two [`PropulsionAssembly`] instances using remapped component and tank IDs.
    #[must_use]
    pub fn merge_remapped(
        &self,
        self_comp_map: &[(ComponentId, ComponentId)],
        other: &Self,
        other_comp_map: &[(ComponentId, ComponentId)],
        other_tank_map: &[(TankId, TankId)],
    ) -> Self {
        let mut out = self.remap_subset(self_comp_map);
        append_propulsion(other, other_comp_map, other_tank_map, &mut out);
        out
    }

    /// Total remaining solid grain propellant mass across all solid motors, kg.
    #[must_use]
    pub fn total_solid_propellant_kg(&self) -> f64 {
        self.solid_motors
            .iter()
            .map(|(_, st)| st.remaining_propellant_kg.max(0.0))
            .sum()
    }
}

fn append_propulsion(
    src: &PropulsionAssembly,
    comp_map: &[(ComponentId, ComponentId)],
    tank_map: &[(TankId, TankId)],
    dst: &mut PropulsionAssembly,
) {
    for &(spec, state) in &src.liquid_rockets {
        if let Some(site) = map_opt_comp(comp_map, spec.site) {
            dst.liquid_rockets.push((
                LiquidRocketSpec {
                    site,
                    fuel_tank: remap_opt_tank(tank_map, spec.fuel_tank),
                    oxidizer_tank: remap_opt_tank(tank_map, spec.oxidizer_tank),
                    ..spec
                },
                state,
            ));
        }
    }
    for &(spec, state) in &src.solid_motors {
        if let Some(site) = map_opt_comp(comp_map, spec.site) {
            dst.solid_motors
                .push((mbd_systems::SolidMotorSpec { site, ..spec }, state));
        }
    }
    for &(spec, state) in &src.electric_thrusters {
        if let Some(site) = map_opt_comp(comp_map, spec.site) {
            dst.electric_thrusters.push((
                ElectricThrusterSpec {
                    site,
                    propellant_tank: remap_tank(tank_map, spec.propellant_tank),
                    ..spec
                },
                state,
            ));
        }
    }
    for &(spec, state) in &src.abep_thrusters {
        if let Some(site) = map_opt_comp(comp_map, spec.site) {
            dst.abep_thrusters.push((
                AbepThrusterSpec {
                    site,
                    buffer_tank: remap_opt_tank(tank_map, spec.buffer_tank),
                    ..spec
                },
                state,
            ));
        }
    }
    for &(spec, state) in &src.airbreathing_engines {
        if let Some(site) = map_opt_comp(comp_map, spec.site) {
            dst.airbreathing_engines.push((
                AirbreathingEngineSpec {
                    site,
                    fuel_tank: remap_tank(tank_map, spec.fuel_tank),
                    ..spec
                },
                state,
            ));
        }
    }
}

/// The elements of `aero` mounted on components that `comp_map` keeps,
/// re-bound to their new ids.
pub(crate) fn remap_aero_subset(
    aero: &[(AeroElement, CanopyState)],
    comp_map: &[(ComponentId, ComponentId)],
) -> VehicleAero {
    aero.iter()
        .filter_map(|(element, canopy)| {
            map_opt_comp(comp_map, element.component)
                .map(|component| (AeroElement::new(component, element.kind.clone()), *canopy))
        })
        .collect()
}

/// The elements of two vehicles joined into one, each re-bound through its
/// own vehicle's map.
pub(crate) fn merge_aero_remapped(
    aero_a: &[(AeroElement, CanopyState)],
    map_a: &[(ComponentId, ComponentId)],
    aero_b: &[(AeroElement, CanopyState)],
    map_b: &[(ComponentId, ComponentId)],
) -> VehicleAero {
    let mut merged = remap_aero_subset(aero_a, map_a);
    merged.extend(remap_aero_subset(aero_b, map_b));
    merged
}

pub(crate) fn remap_subsystems_subset(
    src: &MultibodySubsystems,
    target_tree: &ComponentTree,
    comp_map: &[(ComponentId, ComponentId)],
) -> Result<MultibodySubsystems, mbd_systems::SystemsError> {
    let fluids = src.fluids.remap_subset(comp_map)?;
    let electrical = src.electrical.remap_subset(comp_map)?;
    let thermal = src.thermal.remap_subset(comp_map)?;
    let propulsion = src.propulsion.remap_subset(comp_map);

    let cg_trim = src.cg_trim.filter(|cfg| {
        fluids
            .transfer_lines()
            .iter()
            .any(|(l, _)| l.id == cfg.line_id)
    });
    let mut dst = MultibodySubsystems {
        fluids,
        propulsion,
        electrical,
        thermal,
        cg_trim,
        pinned_breakout_threshold_n: src.pinned_breakout_threshold_n,
        pinned_dwell_s: src.pinned_dwell_s,
        ..MultibodySubsystems::default()
    };
    append_structural_attachments(src, target_tree, comp_map, &mut dst);
    Ok(dst)
}

pub(crate) fn merge_subsystems_remapped(
    a: &MultibodySubsystems,
    map_a: &[(ComponentId, ComponentId)],
    b: &MultibodySubsystems,
    map_b: &[(ComponentId, ComponentId)],
    merged_tree: &ComponentTree,
) -> Result<MultibodySubsystems, mbd_systems::SystemsError> {
    let (fluids, other_tank_map) = a.fluids.merge_remapped(map_a, &b.fluids, map_b)?;
    let electrical = a
        .electrical
        .merge_remapped(map_a, &b.electrical, map_b, &other_tank_map)?;
    let thermal = a.thermal.merge_remapped(map_a, &b.thermal, map_b)?;
    let propulsion = a
        .propulsion
        .merge_remapped(map_a, &b.propulsion, map_b, &other_tank_map);
    let cg_trim = a.cg_trim.filter(|cfg| {
        fluids
            .transfer_lines()
            .iter()
            .any(|(l, _)| l.id == cfg.line_id)
    });

    let mut dst = MultibodySubsystems {
        fluids,
        propulsion,
        electrical,
        thermal,
        cg_trim,
        pinned_breakout_threshold_n: a.pinned_breakout_threshold_n,
        pinned_dwell_s: a.pinned_dwell_s,
        ..MultibodySubsystems::default()
    };
    append_structural_attachments(a, merged_tree, map_a, &mut dst);
    append_structural_attachments(b, merged_tree, map_b, &mut dst);
    Ok(dst)
}

fn append_structural_attachments(
    src: &MultibodySubsystems,
    tree: &ComponentTree,
    comp_map: &[(ComponentId, ComponentId)],
    dst: &mut MultibodySubsystems,
) {
    for &(spec, state) in &src.tps_shields {
        if let Some(component) = map_opt_comp(comp_map, spec.component) {
            dst.tps_shields
                .push((mbd_systems::TpsAblationSpec { component, ..spec }, state));
        }
    }
    for &(spec, state) in &src.ao_coatings {
        if let Some(component) = map_opt_comp(comp_map, spec.component) {
            dst.ao_coatings.push((
                mbd_systems::AtomicOxygenErosionSpec { component, ..spec },
                state,
            ));
        }
    }
    for (name, chain) in &src.manipulators {
        if let Some(ee) = map_opt_comp(comp_map, chain.ee_component()) {
            let mapped_joints: Option<Vec<ComponentId>> = chain
                .joint_ids()
                .iter()
                .map(|&j| map_opt_comp(comp_map, j))
                .collect();
            if let Some(joints) = mapped_joints
                && let Ok(rebuilt) =
                    ManipulatorChain::new(tree, joints, ee, chain.ee_offset_local_m())
            {
                dst.manipulators.push((name.clone(), rebuilt));
            }
        }
    }
    for &(spec, state) in &src.contact_points {
        if let Some(component) = map_opt_comp(comp_map, spec.component) {
            dst.contact_points
                .push((mbd_multibody::ContactPointSpec { component, ..spec }, state));
        }
    }
    for &(spec, state) in &src.wheels {
        if let Some(component) = map_opt_comp(comp_map, spec.component) {
            dst.wheels
                .push((mbd_multibody::WheelSpec { component, ..spec }, state));
        }
    }
    if dst.hydrodynamics.is_none()
        && let Some(hydro) = &src.hydrodynamics
        && let Some(comp) = map_opt_comp(comp_map, hydro.component())
        && let Ok(rebuilt) = HydrodynamicsModel::new(comp, hydro.stations().to_vec())
    {
        dst.hydrodynamics = Some(rebuilt);
    }
    append_loop_constraints(&src.loop_constraints, comp_map, &mut dst.loop_constraints);
}

fn append_loop_constraints(
    constraints: &[LoopConstraint],
    comp_map: &[(ComponentId, ComponentId)],
    dst: &mut Vec<LoopConstraint>,
) {
    for &c in constraints {
        match c {
            LoopConstraint::JointRatio {
                joint_a,
                joint_b,
                ratio,
                offset,
                stiffness,
                damping,
            } => {
                if let (Some(a), Some(b)) = (
                    map_opt_comp(comp_map, joint_a),
                    map_opt_comp(comp_map, joint_b),
                ) {
                    dst.push(LoopConstraint::JointRatio {
                        joint_a: a,
                        joint_b: b,
                        ratio,
                        offset,
                        stiffness,
                        damping,
                    });
                }
            }
            LoopConstraint::PointWeld {
                site_a,
                offset_a_m,
                site_b,
                offset_b_m,
                linear_stiffness_n_m,
                linear_damping_n_s_m,
            } => {
                if let (Some(a), Some(b)) = (
                    map_opt_comp(comp_map, site_a),
                    map_opt_comp(comp_map, site_b),
                ) {
                    dst.push(LoopConstraint::PointWeld {
                        site_a: a,
                        offset_a_m,
                        site_b: b,
                        offset_b_m,
                        linear_stiffness_n_m,
                        linear_damping_n_s_m,
                    });
                }
            }
            LoopConstraint::UnilateralDistance {
                site_a,
                offset_a_m,
                site_b,
                offset_b_m,
                max_length_m,
                stiffness_n_m,
                damping_n_s_m,
            } => {
                if let (Some(a), Some(b)) = (
                    map_opt_comp(comp_map, site_a),
                    map_opt_comp(comp_map, site_b),
                ) {
                    dst.push(LoopConstraint::UnilateralDistance {
                        site_a: a,
                        offset_a_m,
                        site_b: b,
                        offset_b_m,
                        max_length_m,
                        stiffness_n_m,
                        damping_n_s_m,
                    });
                }
            }
        }
    }
}

pub(crate) fn apply_fault_override(slot: &mut FaultState, ov: Option<FaultState>) {
    if let Some(f) = ov {
        *slot = f;
    }
}

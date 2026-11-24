//! The rest latch of a landed vehicle: holding it still while nothing lifts
//! it, and deciding at the end of each free step whether it has settled.

use glam::{DQuat, DVec3};
use mbd_multibody::{PinnedRestInputs, PinnedRestSpec, step_pinned_surface_rest};

use super::subsystems::{MultibodyVehicleInput, MultibodyVehicleTelemetry};
use super::telemetry::build_telemetry;
use super::{MultibodyVehicleBlock, SurfaceContactProvider};
use crate::BlockError;
use crate::flight_model::loads::LoadReadout;
use crate::flight_model::planet_frame::spin;

/// The root pose a vehicle is latched at, in fixed axes.
pub(super) struct PinnedHold {
    pub(super) world_pos_m: DVec3,
    pub(super) world_rot: DQuat,
}

fn surface_up_fixed(surface: &dyn SurfaceContactProvider, point_fixed: DVec3) -> DVec3 {
    surface.sample_surface(point_fixed).map_or_else(
        || point_fixed.normalize_or(DVec3::Z),
        |s| s.outward_normal_world,
    )
}

impl MultibodyVehicleBlock {
    /// Gravity as a body resting on the surface feels it at `point_fixed`:
    /// attraction less the centripetal acceleration of the ground.
    fn apparent_gravity_fixed(&self, point_fixed: DVec3) -> DVec3 {
        let omega_p = spin(&self.config.body);
        self.world.gravity.attraction_fixed(point_fixed) - omega_p.cross(omega_p.cross(point_fixed))
    }

    /// The upward force that frees a resting vehicle: the configured
    /// threshold, or 98 % of its weight at `cg_fixed`.
    fn breakout_threshold_n(&self, cg_fixed: DVec3) -> f64 {
        self.subsystems
            .pinned_breakout_threshold_n
            .unwrap_or_else(|| {
                0.98 * self.mass.total_mass_kg * self.apparent_gravity_fixed(cg_fixed).length()
            })
    }
}

/// Runs a step of a latched vehicle: its subsystems advance while it stays
/// where it is. `None` when thrust, air and outside loads together lift it
/// free, in which case nothing has been stepped.
pub(super) fn hold_pinned_rest(
    block: &mut MultibodyVehicleBlock,
    hold: &PinnedHold,
    input: &MultibodyVehicleInput,
) -> Result<Option<MultibodyVehicleTelemetry>, BlockError> {
    let cg_fixed = hold.world_pos_m + hold.world_rot * block.mass.cg_body_m;
    let up = surface_up_fixed(block.world.surface.as_ref(), cg_fixed);

    // What the engines would deliver is found on copies of the subsystems
    // and the tree, stepped by no time.
    let saved = (block.subsystems.clone(), block.tree.clone());
    let probe = block.step_subsystems(0.0, input);
    (block.subsystems, block.tree) = saved;
    let (_, probe) = probe?;
    block.hold_loads(&probe, input);
    let state = block.state.clone();
    let lift = block.read_loads(&state, input)?;
    block.elements.mechanisms.clear();
    let lifting_n = hold.world_rot * (lift.external_force_body_n - lift.support_force_body_n);
    if lifting_n.dot(up) >= block.breakout_threshold_n(cg_fixed) {
        return Ok(None);
    }

    let dt_s = block.config.period.as_seconds();
    let (env, sub) = block.step_subsystems(dt_s, input)?;
    block.take_up_masses(&sub)?;
    let at_rest = hold.world_rot.conjugate() * (-block.apparent_gravity_fixed(cg_fixed));
    block.last_alpha_root_rad_s2 = DVec3::ZERO;
    block.mechanism_states.clear();
    // A latched vehicle moves with the ground as one body.
    let loads = LoadReadout {
        any_contact: true,
        buoyancy_n: 0.0,
        rigid_fast_path: true,
        ..lift
    };
    Ok(Some(build_telemetry(
        &block.subsystems,
        &env,
        &sub,
        &loads,
        at_rest,
        true,
    )))
}

/// Advances the rest latch at the end of a free step from the loads read
/// there.
pub(super) fn update_pinned_rest(block: &mut MultibodyVehicleBlock, loads: &LoadReadout) {
    let base = block.state.base();
    let cg_fixed = block.cg_fixed_m();
    let up = surface_up_fixed(block.world.surface.as_ref(), cg_fixed);
    let attitude = block.mount.attitude(&base);
    let lifting_n = attitude * loads.external_force_body_n;
    let spec = PinnedRestSpec::lander(
        block.breakout_threshold_n(cg_fixed),
        block.subsystems.pinned_dwell_s,
    );
    step_pinned_surface_rest(
        &mut block.pinned_rest,
        &spec,
        &PinnedRestInputs {
            any_contact: loads.any_contact,
            speed_m_s: base.linear_vel_world_m_s.length(),
            ang_rate_rad_s: base.angular_vel_root_rad_s.length(),
            world_pos_m: base.position_world_m,
            world_rot: base.rotation_world_from_root,
            net_non_grav_upward_n: lifting_n.dot(up),
        },
        block.config.period.as_seconds(),
    );
}

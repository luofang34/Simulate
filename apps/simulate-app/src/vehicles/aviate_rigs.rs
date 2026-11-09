use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint, VehicleController};
use aviate_core::math::Quaternion;
use aviate_core::mixer::Mixer;
use aviate_core::types::{Meters, MetersPerSecond, NormalizedThrust, Radians};
use aviate_hal_io::SystemCommand;
use glam::{DMat3, DVec3, EulerRot, Quat};
use mbd_core::{ComponentSpec, ComponentState, ComponentTree, Placement, VacuumAero};
use sim_atmosphere::FullColumnAtmosphere;
use sim_aviate::{
    AviateDirectConfig, AviateDirectRunner, DirectFixedWingKernel, DirectX500Kernel, RotorLayout,
    build_direct_fixed_wing_kernel, build_direct_x500_kernel, direct_fixed_wing_controls,
};
use sim_core::{
    Block, ControlInputs, JSBSIM_DEFAULT_PERIOD, JsbsimFdmBlock, JsbsimFdmSnapshot,
    RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput, RigidBodySnapshot, SimInstant,
    TICKS_PER_SECOND, TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, UtcTime};
use sim_pack::builtin_c172_package;
use world_core::{GeodeticPosition, normal_gravity};

use super::jsbsim::{build_jsbsim_fdm, step_jsbsim_with_force};
use crate::error::AppSimError;
use crate::world_surface::WorldSurfaceHandle;

#[cfg(test)]
mod tests;

const X500_RATE_HZ: u64 = 400;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(X500_RATE_HZ));
const X500_PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / X500_RATE_HZ);
const X500_XY_SPEED_M_S: f32 = 12.0;
const X500_CLIMB_SPEED_M_S: f32 = 5.0;
const X500_YAW_RATE_RAD_S: f32 = 1.2;
const X500_HOLD_SPEED_M_S: f32 = 0.25;
/// Norm bound on horizontal velocity lead so `vel_p * lead` commands full `20°` tilt authority
/// without saturating Aviate's `1 g` per-axis NED acceleration box clamp at diagonal headings.
const X500_MAX_VEL_LEAD_M_S: f32 = 5.5;

/// Closed-loop Tier-0 Aviate X500 quadrotor plant.
pub struct AviateX500Rig {
    pub(super) body: RigidBody6DofBlock,
    rotors: RotorLayout,
    pub(super) runner: AviateDirectRunner<
        aviate_core::control::multirotor::MultirotorController,
        aviate_core::mixer::QuadXMixerX500,
    >,
    sim_clock_offset: SimInstant,
    origin: Geodetic,
    origin_ecef: DVec3,
    origin_frame: LocalFrame,
    hold_ned_m: [f32; 3],
    holding_position: bool,
    yaw_rad: f32,
    seq: u32,
}

/// Closed-loop Tier-0 Aviate fixed-wing plant.
pub struct AviateFixedWingRig {
    pub(super) fdm: JsbsimFdmBlock,
    pub(super) specific_force_frd_m_s2: DVec3,
    pub(super) runner: AviateDirectRunner<
        aviate_core::control::fixed_wing::FixedWingController,
        sim_aviate::PassthroughFixedWingMixer,
    >,
    sim_clock_offset: SimInstant,
    origin: Geodetic,
    field_elev_m: f64,
    heading_rad: f32,
    seq: u32,
}

pub(super) fn build_aviate_x500(
    origin: Geodetic,
    heading_rad: f64,
) -> Result<AviateX500Rig, AppSimError> {
    let g = normal_gravity(GeodeticPosition::wgs84(
        origin.lat_rad.to_degrees(),
        origin.lon_rad.to_degrees(),
        origin.height_m,
    ))
    .unwrap_or(9.806_65);
    let spec = ComponentSpec {
        name: "X500 Airframe".to_owned(),
        placement: Placement::ROOT,
        dry_mass_kg: 20.25 / g,
        dry_inertia_body: DMat3::from_diagonal(DVec3::new(0.029, 0.029, 0.055)),
        thermal_capacity_j_k: 0.0,
        nominal_power_w: 0.0,
    };
    let tree = ComponentTree::new([(spec, ComponentState::at_temperature(293.15))])
        .map_err(sim_scenario::ScenarioError::from)?;
    let body = RigidBody6DofBlock::new(
        RigidBodyConfig {
            body: BodyEllipsoid::WGS84,
            period: X500_PERIOD,
            epoch: UtcTime::from_unix_seconds(0.0),
            initial: VehicleState::level_flight(&origin, heading_rad, 0.0),
        },
        tree,
        Box::new(VacuumAero),
        Box::new(FullColumnAtmosphere::new()),
    )
    .map_err(sim_scenario::ScenarioError::from)?;
    let kernel: DirectX500Kernel =
        build_direct_x500_kernel().ok_or(AppSimError::AviateKernel { detail: "x500" })?;
    let mut runner = AviateDirectRunner::new(kernel, AviateDirectConfig::industrial(origin))?;
    let mut at = SimInstant::START;
    let rest_force = DVec3::new(0.0, 0.0, -g);
    for _ in 0..400 {
        let _act = runner.step_cg(at, &body.output(), rest_force)?;
        at = at.advanced(X500_PERIOD);
    }
    runner.send_command(SystemCommand::Arm);
    let _act = runner.step_cg(at, &body.output(), rest_force)?;
    at = at.advanced(X500_PERIOD);
    Ok(AviateX500Rig {
        body,
        rotors: RotorLayout::x500(),
        runner,
        sim_clock_offset: at,
        origin,
        origin_ecef: BodyEllipsoid::WGS84.geodetic_to_fixed(&origin),
        origin_frame: LocalFrame::at(&origin),
        hold_ned_m: [0.0, 0.0, 0.0],
        holding_position: true,
        yaw_rad: wrap_pi_f32(heading_rad as f32),
        seq: 0,
    })
}

fn wrap_pi_f32(angle_rad: f32) -> f32 {
    use std::f32::consts::{PI, TAU};
    (angle_rad + PI).rem_euclid(TAU) - PI
}

fn x500_stick_velocity_ned(
    rig: &AviateX500Rig,
    controls: ControlInputs,
    cur_vel_ned: DVec3,
) -> ([f32; 3], bool) {
    let raw_fwd = (-controls.pitch as f32) * X500_XY_SPEED_M_S;
    let raw_right = (controls.roll as f32) * X500_XY_SPEED_M_S;
    let horiz = raw_fwd.hypot(raw_right);
    let scale = if horiz > X500_XY_SPEED_M_S {
        X500_XY_SPEED_M_S / horiz
    } else {
        1.0
    };
    let (fwd_cmd, right_cmd) = (raw_fwd * scale, raw_right * scale);

    let nose_ned = rig
        .origin_frame
        .ecef_to_ned_vector(rig.body.output().attitude * DVec3::X);
    let nose_horiz = nose_ned.x.hypot(nose_ned.y);
    let (cos_h, sin_h) = if nose_horiz > 1e-3 {
        (
            (nose_ned.x / nose_horiz) as f32,
            (nose_ned.y / nose_horiz) as f32,
        )
    } else {
        let (s, c) = rig.yaw_rad.sin_cos();
        (c, s)
    };
    let vn_des = fwd_cmd * cos_h - right_cmd * sin_h;
    let ve_des = fwd_cmd * sin_h + right_cmd * cos_h;

    let (cur_vn, cur_ve) = (cur_vel_ned.x as f32, cur_vel_ned.y as f32);
    let (err_n, err_e) = (vn_des - cur_vn, ve_des - cur_ve);
    let err_h = err_n.hypot(err_e);
    let err_scale = if err_h > X500_MAX_VEL_LEAD_M_S {
        X500_MAX_VEL_LEAD_M_S / err_h
    } else {
        1.0
    };
    let (vn, ve) = (cur_vn + err_n * err_scale, cur_ve + err_e * err_scale);
    let vd = if (controls.throttle - 0.5).abs() < 0.04 {
        0.0
    } else {
        ((0.5 - controls.throttle) * 2.0 * f64::from(X500_CLIMB_SPEED_M_S)) as f32
    };
    let stick_active = fwd_cmd.abs() > 0.05 || right_cmd.abs() > 0.05 || vd.abs() > 0.05;
    ([vn, ve, vd], stick_active)
}

fn update_x500_guidance(rig: &mut AviateX500Rig, controls: ControlInputs) {
    let dt = X500_PERIOD.as_seconds() as f32;
    rig.yaw_rad = wrap_pi_f32(rig.yaw_rad + (controls.yaw as f32) * X500_YAW_RATE_RAD_S * dt);

    let est = rig
        .runner
        .estimated_state()
        .unwrap_or_else(|| rig.body.output());
    let pos_ned = rig
        .origin_frame
        .ecef_to_ned_vector(est.position_ecef - rig.origin_ecef);
    let vel_ned = rig.origin_frame.ecef_to_ned_vector(est.velocity_ecef);
    let current_pos_ned = [pos_ned.x as f32, pos_ned.y as f32, pos_ned.z as f32];
    let speed_m_s = vel_ned.length() as f32;
    let ([vn, ve, vd], stick_active) = x500_stick_velocity_ned(rig, controls, vel_ned);

    let tick = rig.seq;
    rig.seq = rig.seq.wrapping_add(1);
    let command = if stick_active || speed_m_s > X500_HOLD_SPEED_M_S {
        let entered_velocity = rig.holding_position;
        rig.holding_position = false;
        rig.hold_ned_m = current_pos_ned;
        (entered_velocity || tick.is_multiple_of(2)).then(|| {
            (
                ControlMode::VelocityControl,
                Setpoint {
                    velocity: Some([
                        MetersPerSecond(vn),
                        MetersPerSecond(ve),
                        MetersPerSecond(vd),
                    ]),
                    heading: Some(Radians(rig.yaw_rad)),
                    ..Setpoint::default()
                },
            )
        })
    } else {
        let latched = !rig.holding_position;
        if latched {
            rig.hold_ned_m = current_pos_ned;
            rig.holding_position = true;
        }
        (latched || tick.is_multiple_of(8)).then(|| {
            let [n, e, d] = rig.hold_ned_m;
            (
                ControlMode::PositionHold,
                Setpoint {
                    position: Some([Meters(n), Meters(e), Meters(d)]),
                    heading: Some(Radians(rig.yaw_rad)),
                    ..Setpoint::default()
                },
            )
        })
    };
    if let Some((mode, setpoint)) = command {
        rig.runner
            .send_command(SystemCommand::FlightControl(Command {
                mode,
                setpoint,
                config_mode_request: None,
                sensor_overrides: None,
                sequence: rig.seq,
                source: CommandSource::Gcs,
            }));
    }
}

pub(super) fn step_aviate_x500(
    rig: &mut AviateX500Rig,
    controls: ControlInputs,
) -> Result<(), AppSimError> {
    update_x500_guidance(rig, controls);
    let act = rig.runner.step_cg(
        rig.sim_clock_offset,
        &rig.body.output(),
        rig.body.specific_force_cg_body(),
    )?;
    rig.sim_clock_offset = rig.sim_clock_offset.advanced(X500_PERIOD);
    let thrusters = rig.rotors.direct_thrusters(&act)?;
    rig.body
        .step(&RigidBodyInput {
            thrusters,
            ..RigidBodyInput::default()
        })
        .map_err(sim_scenario::ScenarioError::from)?;
    Ok(())
}

pub(super) fn build_aviate_fixed_wing(
    origin: Geodetic,
    heading_rad: f64,
    field_elev_m: f64,
) -> Result<AviateFixedWingRig, AppSimError> {
    let pkg = builtin_c172_package().map_err(sim_scenario::ScenarioError::from)?;
    let (fdm, specific_force_frd_m_s2) =
        build_jsbsim_fdm(&pkg, origin, heading_rad, 52.0, field_elev_m)?;
    let kernel: DirectFixedWingKernel =
        build_direct_fixed_wing_kernel().ok_or(AppSimError::AviateKernel {
            detail: "fixed_wing",
        })?;
    let mut runner = AviateDirectRunner::new(kernel, AviateDirectConfig::industrial(origin))?;
    let mut at = SimInstant::START;
    let state = fdm.output();
    for _ in 0..480 {
        let _act = runner.step_cg(at, &state, specific_force_frd_m_s2)?;
        at = at.advanced(JSBSIM_DEFAULT_PERIOD);
        if runner.is_ready() {
            break;
        }
    }
    runner.send_command(SystemCommand::Arm);
    let _act = runner.step_cg(at, &state, specific_force_frd_m_s2)?;
    at = at.advanced(JSBSIM_DEFAULT_PERIOD);
    Ok(AviateFixedWingRig {
        fdm,
        specific_force_frd_m_s2,
        runner,
        sim_clock_offset: at,
        origin,
        field_elev_m,
        heading_rad: heading_rad as f32,
        seq: 1,
    })
}

pub(super) fn step_aviate_fixed_wing(
    rig: &mut AviateFixedWingRig,
    controls: ControlInputs,
    surface: &WorldSurfaceHandle,
) -> Result<(), AppSimError> {
    let dt = rig.fdm.period().as_seconds() as f32;
    rig.heading_rad += (controls.yaw as f32) * 0.45 * dt;
    let roll_rad = (controls.roll as f32) * 35.0_f32.to_radians();
    let pitch_rad = (controls.pitch as f32) * 18.0_f32.to_radians();
    let q = Quat::from_euler(EulerRot::ZYX, rig.heading_rad, pitch_rad, roll_rad).normalize();
    let thrust = if controls.throttle > 0.01 {
        controls.throttle as f32
    } else {
        0.62
    };
    if rig.seq.is_multiple_of(4) {
        rig.runner
            .send_command(SystemCommand::FlightControl(Command {
                mode: ControlMode::Attitude,
                setpoint: Setpoint {
                    attitude: Some(Quaternion::new(q.w, q.x, q.y, q.z).normalize()),
                    collective_thrust: NormalizedThrust(thrust),
                    ..Setpoint::default()
                },
                config_mode_request: None,
                sensor_overrides: None,
                sequence: rig.seq,
                source: CommandSource::Gcs,
            }));
    }
    rig.seq = rig.seq.wrapping_add(1);
    let act = rig.runner.step_cg(
        rig.sim_clock_offset,
        &rig.fdm.output(),
        rig.specific_force_frd_m_s2,
    )?;
    rig.sim_clock_offset = rig.sim_clock_offset.advanced(rig.fdm.period());
    let fbw_controls = direct_fixed_wing_controls(&act);
    step_jsbsim_with_force(
        &mut rig.fdm,
        &mut rig.specific_force_frd_m_s2,
        fbw_controls,
        surface,
    )
}

/// Steps an Aviate runner on a frozen plant state until its estimator is ready (at least
/// `min_steps`, at most `max_steps`), arms it, and returns the runner's next clock instant.
fn fast_warm<C: VehicleController, M: Mixer>(
    runner: &mut AviateDirectRunner<C, M>,
    state: &VehicleState,
    specific_force_frd_m_s2: DVec3,
    period: TickSpan,
    [min_steps, max_steps]: [u32; 2],
) -> Result<SimInstant, AppSimError> {
    let mut at = SimInstant::START;
    for step in 0..max_steps {
        let _act = runner.step_cg(at, state, specific_force_frd_m_s2)?;
        at = at.advanced(period);
        if step.saturating_add(1) >= min_steps && runner.is_ready() {
            break;
        }
    }
    runner.send_command(SystemCommand::Arm);
    let _act = runner.step_cg(at, state, specific_force_frd_m_s2)?;
    Ok(at.advanced(period))
}

/// Rewindable state of an [`AviateX500Rig`]; the flight controller is rebuilt on restore.
#[derive(Debug, Clone)]
pub struct X500Snapshot {
    body: RigidBodySnapshot,
    origin: Geodetic,
    hold_ned_m: [f32; 3],
    holding_position: bool,
    yaw_rad: f32,
}

/// Rewindable state of an [`AviateFixedWingRig`]; the flight controller is rebuilt on restore.
#[derive(Debug, Clone)]
pub struct FixedWingSnapshot {
    fdm: JsbsimFdmSnapshot,
    origin: Geodetic,
    field_elev_m: f64,
    specific_force_frd_m_s2: DVec3,
    heading_rad: f32,
}

/// Warm steps for a rebuilt X500 controller before it is re-armed.
const X500_REWIND_WARM_STEPS: [u32; 2] = [60, 400];
/// Warm steps for a rebuilt fixed-wing controller before it is re-armed.
const FIXED_WING_REWIND_WARM_STEPS: [u32; 2] = [80, 480];

impl AviateX500Rig {
    pub(super) fn snapshot(&self) -> X500Snapshot {
        X500Snapshot {
            body: self.body.snapshot(),
            origin: self.origin,
            hold_ned_m: self.hold_ned_m,
            holding_position: self.holding_position,
            yaw_rad: self.yaw_rad,
        }
    }

    /// Restores the body and rebuilds, fast-warms, and arms the controller at the restored state.
    pub(super) fn restore(&mut self, snap: &X500Snapshot) -> Result<(), AppSimError> {
        self.body
            .try_restore(&snap.body)
            .map_err(sim_scenario::ScenarioError::from)?;
        let kernel: DirectX500Kernel =
            build_direct_x500_kernel().ok_or(AppSimError::AviateKernel { detail: "x500" })?;
        let mut runner =
            AviateDirectRunner::new(kernel, AviateDirectConfig::industrial(snap.origin))?;
        let state = self.body.output();
        let force = self.body.specific_force_cg_body();
        self.sim_clock_offset = fast_warm(
            &mut runner,
            &state,
            force,
            X500_PERIOD,
            X500_REWIND_WARM_STEPS,
        )?;
        self.runner = runner;
        self.origin = snap.origin;
        self.origin_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&snap.origin);
        self.origin_frame = LocalFrame::at(&snap.origin);
        self.hold_ned_m = snap.hold_ned_m;
        self.holding_position = snap.holding_position;
        self.yaw_rad = snap.yaw_rad;
        self.seq = 0;
        Ok(())
    }
}

impl AviateFixedWingRig {
    pub(super) fn snapshot(&self) -> FixedWingSnapshot {
        FixedWingSnapshot {
            fdm: self.fdm.snapshot(),
            origin: self.origin,
            field_elev_m: self.field_elev_m,
            specific_force_frd_m_s2: self.specific_force_frd_m_s2,
            heading_rad: self.heading_rad,
        }
    }

    /// Restores the aircraft and rebuilds, fast-warms, and arms the controller there.
    pub(super) fn restore(&mut self, snap: &FixedWingSnapshot) -> Result<(), AppSimError> {
        self.fdm
            .try_restore(&snap.fdm)
            .map_err(sim_scenario::ScenarioError::from)?;
        self.fdm
            .set_terrain_elevation_m(snap.field_elev_m)
            .map_err(sim_scenario::ScenarioError::from)?;
        let kernel: DirectFixedWingKernel =
            build_direct_fixed_wing_kernel().ok_or(AppSimError::AviateKernel {
                detail: "fixed_wing",
            })?;
        let mut runner =
            AviateDirectRunner::new(kernel, AviateDirectConfig::industrial(snap.origin))?;
        let state = self.fdm.output();
        self.sim_clock_offset = fast_warm(
            &mut runner,
            &state,
            snap.specific_force_frd_m_s2,
            self.fdm.period(),
            FIXED_WING_REWIND_WARM_STEPS,
        )?;
        self.runner = runner;
        self.origin = snap.origin;
        self.field_elev_m = snap.field_elev_m;
        self.specific_force_frd_m_s2 = snap.specific_force_frd_m_s2;
        self.heading_rad = snap.heading_rad;
        Ok(())
    }
}

//! Commands toward the flight controller, as the MAVLink frames Pilotage's
//! codec encodes.
//!
//! Aviate takes a local-NED velocity with an absolute heading, so a body-frame
//! velocity is rotated by the flight controller's reported yaw and a commanded
//! yaw rate is integrated into the heading setpoint, as Pilotage's own Aviate
//! uplink does.

use pilotage_mavlink::codec::{
    AttitudeTargetKind, encode_arm_command, encode_attitude_target, encode_velocity_setpoint,
};
use pilotage_protocol::{AttitudeThrustIntent, ReferenceFrame, VelocityIntent};
use sim_core::SimInstant;

/// MAVLink identity of the flight controller the frames address.
pub(super) const FC_SYSTEM_ID: u8 = 1;
/// MAVLink component of the flight controller the frames address.
pub(super) const FC_COMPONENT_ID: u8 = 1;
/// Longest gap between two motion frames integrated as one step of yaw rate.
const MAX_YAW_STEP_S: f64 = 0.1;

/// Frame sequencing and the heading setpoint.
#[derive(Debug, Clone, Default)]
pub(super) struct Uplink {
    sequence: u8,
    heading_rad: Option<f64>,
    last_motion: Option<SimInstant>,
}

impl Uplink {
    fn next_sequence(&mut self) -> u8 {
        let sequence = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        sequence
    }

    /// An arm or disarm command. Arming drops the heading setpoint, so the
    /// first motion frame holds the heading the vehicle then has.
    pub(super) fn arm_frame(&mut self, arm: bool) -> [u8; 45] {
        self.heading_rad = None;
        self.last_motion = None;
        encode_arm_command(self.next_sequence(), arm, FC_SYSTEM_ID, FC_COMPONENT_ID)
    }

    /// A velocity setpoint for `intent`, the vehicle heading `yaw_rad` by the
    /// flight controller's own estimate.
    pub(super) fn velocity_frame(
        &mut self,
        intent: &VelocityIntent,
        yaw_rad: f64,
        now: SimInstant,
    ) -> [u8; 65] {
        let (sin, cos) = match intent.frame {
            ReferenceFrame::BodyFrd | ReferenceFrame::Gimbal => yaw_rad.sin_cos(),
            ReferenceFrame::LocalNed => (0.0, 1.0),
        };
        let (vx, vy) = (f64::from(intent.vx), f64::from(intent.vy));
        let velocity_ned = [
            (vx * cos - vy * sin) as f32,
            (vx * sin + vy * cos) as f32,
            intent.vz,
        ];
        let step_s = self.last_motion.map_or(0.0, |last| {
            now.duration_since(last).as_seconds().min(MAX_YAW_STEP_S)
        });
        let heading =
            wrap_pi(self.heading_rad.unwrap_or(yaw_rad) + f64::from(intent.yaw_rate) * step_s);
        self.heading_rad = Some(heading);
        self.last_motion = Some(now);
        encode_velocity_setpoint(
            self.next_sequence(),
            boot_ms(now),
            velocity_ned,
            heading as f32,
            FC_SYSTEM_ID,
            FC_COMPONENT_ID,
        )
    }

    /// An attitude and collective-thrust setpoint.
    pub(super) fn attitude_frame(
        &mut self,
        intent: &AttitudeThrustIntent,
        now: SimInstant,
    ) -> [u8; 51] {
        encode_attitude_target(
            self.next_sequence(),
            boot_ms(now),
            AttitudeTargetKind::Attitude([intent.qw, intent.qx, intent.qy, intent.qz]),
            intent.thrust,
            FC_SYSTEM_ID,
            FC_COMPONENT_ID,
        )
    }
}

/// Milliseconds on the flight controller's boot clock, which starts with the
/// simulation.
pub(super) fn boot_ms(now: SimInstant) -> u32 {
    let millis = now.duration_since(SimInstant::START).as_seconds() * 1_000.0;
    millis as u32
}

fn wrap_pi(angle_rad: f64) -> f64 {
    use std::f64::consts::{PI, TAU};
    (angle_rad + PI).rem_euclid(TAU) - PI
}

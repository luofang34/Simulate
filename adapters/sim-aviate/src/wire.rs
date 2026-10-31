use core::mem::size_of_val;
use core::sync::atomic::AtomicU32;

use aviate_xil_contract::{
    ControlBlock, EXPECTED_SIZE, LAYOUT_VERSION, MAGIC, ModelStateBlock, MotorCommandBlock,
    SharedStateHeader, SharedStateV2, pack_fc_status, seqlock_read, seqlock_write, validate_attach,
};

use crate::{AviateBridgeError, ModelLanes};

#[cfg(test)]
mod tests;

/// The `fc_status` word of a block no flight controller has touched.
const NO_FLIGHT_CONTROLLER: aviate_xil_contract::FcState = aviate_xil_contract::FcState::Init;

/// A block as its writer stamps it at creation, ready for a first publish:
/// world epoch 1, lockstep armed, no model state yet.
///
/// `writer_incarnation` is the identity the contract reserves zero out of, for
/// a writer that never stamped one.
pub fn new_shared_state(writer_incarnation: u64) -> SharedStateV2 {
    SharedStateV2 {
        header: SharedStateHeader {
            magic: MAGIC,
            layout_version: LAYOUT_VERSION,
            declared_size: EXPECTED_SIZE as u32,
            reset_generation: 1,
            plugin_ready: 1,
            writer_incarnation,
            _reserved0: [0; 4],
        },
        state: empty_model_state(),
        command: empty_motor_command(),
        control: ControlBlock {
            lifecycle_request: 0,
            lifecycle_ack_nonce: 0,
            fc_session_nonce: 0,
            fc_status: pack_fc_status(1, NO_FLIGHT_CONTROLLER),
            lockstep_enabled: 1,
            target_rtf_percent: 0,
            _reserved2: [0; 8],
        },
    }
}

fn empty_model_state() -> ModelStateBlock {
    ModelStateBlock {
        seq: 0,
        reset_generation: 1,
        sim_step: 0,
        time_us: 0,
        pos_bits: [0; 3],
        quat_bits: [0; 4],
        vel_bits: [0; 3],
        ang_vel_bits: [0; 3],
        valid: 0,
        _pad1: 0,
        _reserved_state: [0; 7],
    }
}

fn empty_motor_command() -> MotorCommandBlock {
    MotorCommandBlock {
        seq: 0,
        num_motors: 0,
        motor_vel_bits: [0; 8],
        fc_step_ack: 0,
        _reserved1: [0; 6],
    }
}

/// Applies the contract's attach rules, then its readiness rule, so nothing
/// interprets a foreign, stale or half-built block.
pub fn validate_shared_state(shared: &SharedStateV2) -> Result<(), AviateBridgeError> {
    let SharedStateHeader {
        magic,
        layout_version,
        declared_size,
        plugin_ready,
        ..
    } = shared.header;
    validate_attach(magic, layout_version, declared_size, size_of_val(shared))
        .map_err(AviateBridgeError::Attach)?;
    if plugin_ready == 0 {
        return Err(AviateBridgeError::NotReady);
    }
    Ok(())
}

/// Publishes one snapshot under the contract's seqlock.
///
/// Non-finite lanes publish `valid = 0` with their lanes untouched, so a
/// reader withholds rather than believes them, as the Gazebo plugin does for a
/// step it cannot read.
pub(crate) fn publish_model_state(
    shared: &mut SharedStateV2,
    step: u64,
    micros: u64,
    lanes: &ModelLanes,
) -> Result<(), AviateBridgeError> {
    let generation = shared.header.reset_generation;
    let ModelStateBlock {
        seq,
        reset_generation,
        sim_step,
        time_us,
        pos_bits,
        quat_bits,
        vel_bits,
        ang_vel_bits,
        valid,
        ..
    } = &mut shared.state;
    if *seq & 1 != 0 {
        return Err(AviateBridgeError::ModelStateInFlight { seq: *seq });
    }
    let usable = lanes.is_finite();
    seqlock_write(AtomicU32::from_mut(seq), || {
        *reset_generation = generation;
        *sim_step = step;
        *time_us = micros;
        *valid = u32::from(usable);
        if usable {
            *pos_bits = lanes.position_enu_m.map(f64::to_bits);
            *quat_bits = lanes.attitude_wxyz.map(f64::to_bits);
            *vel_bits = lanes.velocity_enu_m_s.map(f64::to_bits);
            *ang_vel_bits = lanes.angular_rate_enu_rad_s.map(f64::to_bits);
        }
    });
    Ok(())
}

/// Rotor speed setpoints in contract order, decoded from the bit-pattern lanes
/// and checked.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorCommand {
    speeds_rad_s: [f64; 8],
    count: usize,
}

impl MotorCommand {
    /// The populated setpoints, rad/s.
    pub fn speeds_rad_s(&self) -> &[f64] {
        self.speeds_rad_s.get(..self.count).unwrap_or_default()
    }
}

/// One consistent command through the contract's seqlock, or an error: on a
/// torn read there is no snapshot to hand on.
pub(crate) fn read_motor_command(
    shared: &mut SharedStateV2,
) -> Result<MotorCommand, AviateBridgeError> {
    let MotorCommandBlock {
        seq,
        num_motors,
        motor_vel_bits,
        ..
    } = &mut shared.command;
    let found = *seq;
    let (bits, count) = seqlock_read(AtomicU32::from_mut(seq), || (*motor_vel_bits, *num_motors))
        .ok_or(AviateBridgeError::MotorCommandTorn { seq: found })?;
    let populated = usize::try_from(count)
        .ok()
        .filter(|populated| *populated <= bits.len())
        .ok_or(AviateBridgeError::MotorCountOutOfRange { count })?;
    let speeds_rad_s = bits.map(f64::from_bits);
    let bad_lane = speeds_rad_s
        .iter()
        .take(populated)
        .position(|speed| !speed.is_finite());
    if let Some(lane) = bad_lane {
        let bits = speeds_rad_s.get(lane).map_or(0, |speed| speed.to_bits());
        return Err(AviateBridgeError::MotorSpeedNotFinite { lane, bits });
    }
    Ok(MotorCommand {
        speeds_rad_s,
        count: populated,
    })
}

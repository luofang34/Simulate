use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan};

use super::MavlinkHilConfig;
use crate::mavlink_hil::messages::{
    HIL_SENSOR_FIELDS_ALL, HilGpsSample, HilSensorSample, HilStateQuaternionSample,
    encode_heartbeat, encode_hil_gps, encode_hil_sensor, encode_hil_state_quaternion,
};
use crate::{XilError, XilStimulus};

const TICKS_PER_MICROSECOND: u64 = TICKS_PER_SECOND / 1_000_000;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(1_000_000));

/// Builds each step's outbound burst: `HEARTBEAT` and `HIL_GPS` at their
/// configured periods, `HIL_SENSOR` every step, ground truth on request.
#[derive(Debug)]
pub(super) struct OutboundSequencer {
    pub(super) config: MavlinkHilConfig,
    seq: u8,
    last_gps_at: Option<SimInstant>,
    last_heartbeat_at: Option<SimInstant>,
    scratch: Vec<u8>,
}

impl OutboundSequencer {
    pub(super) fn new(config: MavlinkHilConfig) -> Self {
        Self {
            config,
            seq: 0,
            last_gps_at: None,
            last_heartbeat_at: None,
            scratch: Vec::with_capacity(256),
        }
    }

    pub(super) fn build_burst(
        &mut self,
        at: SimInstant,
        stimulus: &XilStimulus,
    ) -> Result<&[u8], XilError> {
        self.scratch.clear();
        let time_usec = (at.ticks_u128() / u128::from(TICKS_PER_MICROSECOND)) as u64;
        let mut frame = [0_u8; 280];

        if should_emit(at, self.last_heartbeat_at, self.config.heartbeat_period) {
            let n = encode_heartbeat(
                self.next_seq(),
                self.config.system_id,
                self.config.component_id,
                false,
                &mut frame,
            )?;
            append_prefix(&mut self.scratch, &frame, n);
            self.last_heartbeat_at = Some(at);
        }

        let sensor = HilSensorSample::from_stimulus(time_usec, stimulus, HIL_SENSOR_FIELDS_ALL)?;
        let n = encode_hil_sensor(
            self.next_seq(),
            self.config.system_id,
            self.config.component_id,
            &sensor,
            &mut frame,
        )?;
        append_prefix(&mut self.scratch, &frame, n);

        if should_emit(at, self.last_gps_at, self.config.gps_period) {
            let gps = HilGpsSample::from_stimulus(time_usec, stimulus)?;
            let n = encode_hil_gps(
                self.next_seq(),
                self.config.system_id,
                self.config.component_id,
                &gps,
                &mut frame,
            )?;
            append_prefix(&mut self.scratch, &frame, n);
            self.last_gps_at = Some(at);
        }

        if self.config.send_state_quaternion {
            let truth = HilStateQuaternionSample::from_stimulus(time_usec, stimulus)?;
            let n = encode_hil_state_quaternion(
                self.next_seq(),
                self.config.system_id,
                self.config.component_id,
                &truth,
                &mut frame,
            )?;
            append_prefix(&mut self.scratch, &frame, n);
        }

        Ok(&self.scratch)
    }

    const fn next_seq(&mut self) -> u8 {
        let current = self.seq;
        self.seq = self.seq.wrapping_add(1);
        current
    }
}

fn should_emit(now: SimInstant, last: Option<SimInstant>, period: TickSpan) -> bool {
    match last {
        None => true,
        Some(prev) => now.duration_since(prev) >= period,
    }
}

fn append_prefix(dst: &mut Vec<u8>, src: &[u8], len: usize) {
    if let Some(prefix) = src.get(..len) {
        dst.extend_from_slice(prefix);
    }
}

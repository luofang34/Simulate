//! The simulation oracle's report: the true state as a [`SimTruthSample`],
//! the one form in which truth leaves the vehicle.

use glam::{DMat3, DQuat, DVec3};
use pilotage_adapter_api::{
    MeasurementClock, MeasurementStamp, SimTruthSample, SourceIncarnation, SourceIntegrity,
    SourceRole,
};
use sim_core::{SimInstant, VehicleState};
use sim_geo::{LocalFrame, geodetic_to_ecef};

use super::AviateVehicleConfig;

/// Attitude, position and velocity: a truth sample carries no rates.
const TRUTH_GROUPS: u32 = 0b1101;
const TRUTH_SOURCE: u64 = 2;
const NANOS_PER_SECOND: f64 = 1.0e9;

/// Projects truth into the pad's north-east-down frame and stamps it.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct TruthReporter {
    pad_ecef: DVec3,
    frame: LocalFrame,
    incarnation: SourceIncarnation,
    sequence: u32,
}

impl TruthReporter {
    pub(super) fn new(config: &AviateVehicleConfig) -> Self {
        let mut bytes = [0; 16];
        let (head, tail) = bytes.split_at_mut(8);
        head.copy_from_slice(&config.vehicle.as_u64().to_le_bytes());
        tail.copy_from_slice(&config.noise_seed.to_le_bytes());
        Self {
            pad_ecef: geodetic_to_ecef(&config.pad),
            frame: LocalFrame::at(&config.pad),
            incarnation: SourceIncarnation::new(bytes),
            sequence: 0,
        }
    }

    pub(super) fn sample(&mut self, at: SimInstant, state: &VehicleState) -> SimTruthSample {
        let position = self
            .frame
            .ecef_to_ned_vector(state.position_ecef - self.pad_ecef);
        let velocity = self.frame.ecef_to_ned_vector(state.velocity_ecef);
        let body_to_ned = DQuat::from_mat3(
            &(self.frame.ned_to_ecef().transpose() * DMat3::from_quat(state.attitude)),
        )
        .normalize();
        let sequence = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        let elapsed_s = at.duration_since(SimInstant::START).as_seconds();
        SimTruthSample {
            quat_wxyz: [body_to_ned.w, body_to_ned.x, body_to_ned.y, body_to_ned.z]
                .map(|component| component as f32),
            pos_ned_m: position.to_array().map(|component| component as f32),
            vel_ned_mps: velocity.to_array().map(|component| component as f32),
            valid_flags: TRUTH_GROUPS,
            geodetic: None,
            stamp: MeasurementStamp {
                role: SourceRole::SimulationTruth,
                integrity: SourceIntegrity::Unprotected,
                source_id: TRUTH_SOURCE,
                source_incarnation: self.incarnation,
                source_epoch: 0,
                sequence,
                acquired_at_ns: (elapsed_s * NANOS_PER_SECOND) as u64,
                clock: MeasurementClock::Simulation,
            },
        }
    }
}

use sim_apollo::{CockpitInstrumentPanel, EntryMonitorBlock, ThreeGimbalImuBlock};
use sim_core::{Block, RigidBodySnapshot};

use super::{CapsuleReentryReport, CapsuleReentryRun};
use crate::ScenarioError;

/// Everything a [`CapsuleReentryRun`] changes while it flies.
#[derive(Debug, Clone)]
pub(crate) struct ReentryState {
    vehicle: RigidBodySnapshot,
    imu: ThreeGimbalImuBlock,
    ems: EntryMonitorBlock,
    panel: CockpitInstrumentPanel,
    wanted_bank_rad: f64,
    report: CapsuleReentryReport,
    trim_incidence_sum_rad: f64,
    trim_samples: u64,
    finished: bool,
}

impl CapsuleReentryRun {
    pub(crate) fn save_state(&self) -> ReentryState {
        ReentryState {
            vehicle: self.vehicle.snapshot(),
            imu: self.imu.clone(),
            ems: self.ems.clone(),
            panel: self.panel.clone(),
            wanted_bank_rad: self.wanted_bank_rad,
            report: self.report,
            trim_incidence_sum_rad: self.trim_incidence_sum_rad,
            trim_samples: self.trim_samples,
            finished: self.finished,
        }
    }

    pub(crate) fn restore_state(&mut self, state: &ReentryState) -> Result<(), ScenarioError> {
        self.vehicle.try_restore(&state.vehicle)?;
        self.imu.clone_from(&state.imu);
        self.ems.clone_from(&state.ems);
        self.panel.clone_from(&state.panel);
        self.wanted_bank_rad = state.wanted_bank_rad;
        self.report = state.report;
        self.trim_incidence_sum_rad = state.trim_incidence_sum_rad;
        self.trim_samples = state.trim_samples;
        self.finished = state.finished;
        Ok(())
    }
}

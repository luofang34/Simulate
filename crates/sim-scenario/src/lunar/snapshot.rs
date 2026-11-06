use glam::DVec3;
use sim_apollo::{
    CockpitInstrumentPanel, LandingRadarBlock, LunarContactProbeBlock, RadarSample,
    RateGyroAssemblyBlock, StrapdownAsaBlock, ThreeGimbalImuBlock,
};
use sim_core::{Block, RateAdapter, RigidBodySnapshot};

use super::{LunarLandingReport, LunarLandingRun};
use crate::ScenarioError;

/// Everything a [`LunarLandingRun`] changes while it flies, the redesignated
/// target included. The scenery, window direction, and start bookkeeping are
/// fixed at construction and so are not captured.
#[derive(Debug, Clone)]
pub(crate) struct LunarState {
    vehicle: RigidBodySnapshot,
    imu: ThreeGimbalImuBlock,
    asa: StrapdownAsaBlock,
    rate_gyros: RateGyroAssemblyBlock,
    radar: LandingRadarBlock,
    radar_rate: RateAdapter,
    radar_sample: Option<RadarSample>,
    probes: LunarContactProbeBlock,
    panel: CockpitInstrumentPanel,
    target_east_m: f64,
    target_north_m: f64,
    target_fixed_m: DVec3,
    report: LunarLandingReport,
    finished: bool,
}

impl LunarLandingRun {
    pub(crate) fn save_state(&self) -> LunarState {
        LunarState {
            vehicle: self.vehicle.snapshot(),
            imu: self.imu.clone(),
            asa: self.asa.clone(),
            rate_gyros: self.rate_gyros.clone(),
            radar: self.radar.clone(),
            radar_rate: self.radar_rate,
            radar_sample: self.radar_sample,
            probes: self.probes.clone(),
            panel: self.panel.clone(),
            target_east_m: self.target_east_m,
            target_north_m: self.target_north_m,
            target_fixed_m: self.target_fixed_m,
            report: self.report,
            finished: self.finished,
        }
    }

    pub(crate) fn restore_state(&mut self, state: &LunarState) -> Result<(), ScenarioError> {
        self.vehicle.try_restore(&state.vehicle)?;
        self.imu.clone_from(&state.imu);
        self.asa.clone_from(&state.asa);
        self.rate_gyros.clone_from(&state.rate_gyros);
        self.radar.clone_from(&state.radar);
        self.radar_rate = state.radar_rate;
        self.radar_sample = state.radar_sample;
        self.probes.clone_from(&state.probes);
        self.panel.clone_from(&state.panel);
        self.target_east_m = state.target_east_m;
        self.target_north_m = state.target_north_m;
        self.target_fixed_m = state.target_fixed_m;
        self.report = state.report;
        self.finished = state.finished;
        Ok(())
    }
}

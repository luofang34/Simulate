use sim_apollo::CockpitInstrumentPanel;
use sim_core::{CameraSensorBlock, ControlInputs, RateAdapter, SimInstant, TwoTierVisualHarness};

use super::{Autopilot, CorridorReport, CorridorRun, ScenarioPitot, Source, VisualSource};
use crate::ScenarioError;
use crate::run::snapshot::FdmSnapshot;

/// Everything a [`CorridorRun`] changes while it flies. The profile, origin,
/// and anchor frame are fixed at construction and so are not captured.
#[derive(Debug, Clone)]
pub(crate) struct CorridorState {
    source: FdmSnapshot,
    autopilot: Autopilot,
    pitot: ScenarioPitot,
    last_controls: ControlInputs,
    camera: CameraSensorBlock,
    camera_rate: RateAdapter,
    harness: TwoTierVisualHarness,
    visual: VisualSource,
    panel: CockpitInstrumentPanel,
    now: SimInstant,
    report: CorridorReport,
}

impl CorridorRun {
    pub(crate) fn save_state(&self) -> CorridorState {
        CorridorState {
            source: match &self.source {
                Source::Kinematic(model) => FdmSnapshot::of_kinematic(model),
                Source::Jsbsim(model) => FdmSnapshot::of_jsbsim(model),
            },
            autopilot: self.autopilot,
            pitot: self.pitot,
            last_controls: self.last_controls,
            camera: self.camera.clone(),
            camera_rate: self.camera_rate,
            harness: self.harness.clone(),
            visual: self.visual.clone(),
            panel: self.panel.clone(),
            now: self.now,
            report: self.report.clone(),
        }
    }

    pub(crate) fn restore_state(&mut self, state: &CorridorState) -> Result<(), ScenarioError> {
        match &mut self.source {
            Source::Kinematic(model) => state.source.restore_kinematic(model)?,
            Source::Jsbsim(model) => state.source.restore_jsbsim(model)?,
        }
        self.autopilot = state.autopilot;
        self.pitot = state.pitot;
        self.last_controls = state.last_controls;
        self.camera.clone_from(&state.camera);
        self.camera_rate = state.camera_rate;
        self.harness.clone_from(&state.harness);
        self.visual.clone_from(&state.visual);
        self.panel.clone_from(&state.panel);
        self.now = state.now;
        self.report.clone_from(&state.report);
        Ok(())
    }
}

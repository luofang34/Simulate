use sim_apollo::CockpitInstrumentPanel;
use sim_core::{ControlInputs, RateAdapter, SimInstant, VirtualCameraRigBlock};

use super::{AirportPatternRun, PatternAutopilot, PatternLeg, PatternPlant, PatternReport};
use crate::ScenarioError;
use crate::corridor::ScenarioPitot;
use crate::run::snapshot::FdmSnapshot;

/// Everything an [`AirportPatternRun`] changes while it flies. The aerodrome
/// geometry, traffic direction, and circuit limit are fixed at construction
/// and so are not captured.
#[derive(Debug, Clone)]
pub(crate) struct PatternState {
    plant: FdmSnapshot,
    autopilot: PatternAutopilot,
    pitot: ScenarioPitot,
    last_controls: ControlInputs,
    leg: PatternLeg,
    camera_rig: VirtualCameraRigBlock,
    camera_steps: RateAdapter,
    panel: CockpitInstrumentPanel,
    now: SimInstant,
    report: PatternReport,
}

impl AirportPatternRun {
    pub(crate) fn save_state(&self) -> PatternState {
        PatternState {
            plant: match &self.plant {
                PatternPlant::Kinematic(model) => FdmSnapshot::of_kinematic(model),
                PatternPlant::C172Jsbsim(model) => FdmSnapshot::of_jsbsim(model),
            },
            autopilot: self.autopilot,
            pitot: self.pitot,
            last_controls: self.last_controls,
            leg: self.leg,
            camera_rig: self.camera_rig.clone(),
            camera_steps: self.camera_steps,
            panel: self.panel.clone(),
            now: self.now,
            report: self.report.clone(),
        }
    }

    pub(crate) fn restore_state(&mut self, state: &PatternState) -> Result<(), ScenarioError> {
        match &mut self.plant {
            PatternPlant::Kinematic(model) => state.plant.restore_kinematic(model)?,
            PatternPlant::C172Jsbsim(model) => state.plant.restore_jsbsim(model)?,
        }
        self.autopilot = state.autopilot;
        self.pitot = state.pitot;
        self.last_controls = state.last_controls;
        self.leg = state.leg;
        self.camera_rig.clone_from(&state.camera_rig);
        self.camera_steps = state.camera_steps;
        self.panel.clone_from(&state.panel);
        self.now = state.now;
        self.report.clone_from(&state.report);
        Ok(())
    }
}

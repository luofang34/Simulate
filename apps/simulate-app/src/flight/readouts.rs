//! Plant readouts the cockpit, avionics, and hangar show: specific force, aircraft
//! systems, engine, articulation, and Aviate arming.

use glam::DVec3;
use sim_core::{AirframeArticulation, EngineStatus, SystemControls};

use super::Flight;
use super::mode::Mode;
use super::rewind::seconds;

impl Flight {
    /// Body FRD specific force (`m/s²`), when provided by the plant.
    #[must_use]
    pub fn specific_force_frd_m_s2(&self) -> Option<DVec3> {
        match &self.mode {
            Mode::Free { session, .. } => session.block().plant().specific_force_frd_m_s2(),
            Mode::Scenario(_) => None,
        }
    }

    /// Active [`SystemControls`] on JSBSim free-flight aircraft.
    #[must_use]
    pub fn system_controls(&self) -> Option<SystemControls> {
        match &self.mode {
            Mode::Free { session, .. } => session.block().system_controls(),
            Mode::Scenario(_) => None,
        }
    }

    /// Applies `f` to the [`SystemControls`] on JSBSim free-flight aircraft; the change
    /// takes effect at the start of the next physics step.
    pub fn update_system_controls(&mut self, f: impl FnOnce(&mut SystemControls)) {
        if let Mode::Free { session, .. } = &self.mode
            && let Some(mut sys) = session.block().system_controls()
        {
            f(&mut sys);
            session.block().queue_system_controls(sys);
        }
    }

    /// Latest engine status (`running`, `propeller_rpm`) on JSBSim free-flight aircraft.
    #[must_use]
    pub fn engine_status(&self) -> Option<EngineStatus> {
        match &self.mode {
            Mode::Free { session, .. } => session.block().plant().engine_status(),
            Mode::Scenario(run) => {
                let art = run.articulation();
                (art.engine_running || art.propeller_rpm.abs() > 1.0).then_some(EngineStatus {
                    running: art.engine_running,
                    propeller_rpm: art.propeller_rpm,
                })
            }
        }
    }

    /// Full powerplant, propeller, control-surface, and landing-gear articulation.
    #[must_use]
    pub fn articulation(&self) -> AirframeArticulation {
        match &self.mode {
            Mode::Free { session, .. } => {
                let elapsed_s = seconds(session.physics_time());
                session.block().plant().articulation(elapsed_s)
            }
            Mode::Scenario(run) => run.articulation(),
        }
    }

    /// Whether an Aviate direct flight controller is armed (`None` on non-Aviate flights).
    #[must_use]
    pub fn aviate_armed(&self) -> Option<bool> {
        match &self.mode {
            Mode::Free { session, .. } => session.block().plant().aviate_armed(),
            Mode::Scenario(_) => None,
        }
    }
}

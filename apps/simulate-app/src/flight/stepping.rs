//! One frame's worth of fixed steps, for whichever kind of flight is flying.

use sim_core::{ControlInputs, TickSpan};
use tracing::info;

use super::Flight;
use super::mode::{self, Mode};
use crate::error::AppSimError;

impl Flight {
    /// Advances the flight by `ticks` of master-clock time in whole steps of `step_s`
    /// seconds and returns how many ran.
    pub(super) fn step_mode(
        &mut self,
        ticks: TickSpan,
        controls: ControlInputs,
        step_s: f64,
    ) -> Result<u64, AppSimError> {
        let steps = match &mut self.mode {
            Mode::Free { world, panel, .. } => {
                let steps = world.advance(ticks)?;
                self.exposures.extend(world.take_exposures());
                if let Some(session) = world.session() {
                    mode::observe(panel, session.block().plant(), session.physics_time());
                }
                for _ in 0..steps {
                    self.sensors.step(step_s);
                }
                steps
            }
            Mode::Scenario(run) => {
                let steps = self.steps.steps_for(ticks);
                for _ in 0..steps {
                    self.prev_vehicle = run.vehicle();
                    run.step()?;
                    self.sensors.step(step_s);
                    self.scenario_sensor.on_step(
                        run.period(),
                        run.now(),
                        run.vehicle(),
                        &mut self.exposures,
                    );
                }
                if run.finished() && !self.finish_logged {
                    self.finish_logged = true;
                    info!(scenario = run.profile().title(), "scenario complete");
                }
                steps
            }
            Mode::Space(world) => {
                let steps = self.steps.steps_for(ticks);
                for _ in 0..steps {
                    self.prev_vehicle = world.vehicle();
                    world.step(controls)?;
                    self.sensors.step(step_s);
                    self.scenario_sensor.on_step(
                        mode::space_period(),
                        world.now(),
                        world.vehicle(),
                        &mut self.exposures,
                    );
                }
                steps
            }
        };
        Ok(steps)
    }
}

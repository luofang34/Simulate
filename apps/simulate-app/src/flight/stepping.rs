//! One frame's worth of fixed steps, for whichever kind of flight is flying.

use sim_core::{ControlInputs, TickSpan};
use tracing::info;

use super::Flight;
use super::mode::{self, Mode};
use crate::error::AppSimError;

/// The fastest a flight runs while flight dynamics, not orbital propagation, carry it.
const FLOWN_WARP_LIMIT: f64 = 4.0;

impl Flight {
    /// `factor`, governed by [`Flight::space_warp_status`] when a spaceflight is flying,
    /// and held to [`FLOWN_WARP_LIMIT`] whenever flight dynamics carry the vehicle.
    pub(super) fn governed_warp(&self, factor: f64, controls: ControlInputs) -> f64 {
        if factor <= 0.0 {
            return 0.0;
        }
        match &self.mode {
            Mode::Space(world) if world.coasting(controls) => self
                .space_warp_status(factor, controls)
                .map_or(factor, |status| status.achieved_warp),
            Mode::Space(_) => self
                .space_warp_status(factor, controls)
                .map_or(factor, |status| status.achieved_warp)
                .min(FLOWN_WARP_LIMIT),
            Mode::Free { .. } | Mode::Scenario(_) => factor.min(FLOWN_WARP_LIMIT),
        }
    }

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
                let exposures = world.take_exposures();
                // A page's navigation camera renders its frames apart from the maps, and
                // nothing else in a page reads an optical exposure; handed on, they would
                // only draw the maps' hidden sensor slot.
                if self.page.is_none() {
                    self.exposures.extend(exposures);
                }
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
            Mode::Space(world) if world.coasting(controls) => {
                world.pump_plans();
                world.coast(ticks)?;
                self.prev_vehicle = world.vehicle();
                0
            }
            Mode::Space(world) => {
                world.pump_plans();
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

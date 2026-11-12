//! A link's page flight inside [`Flight`]: lean bare-Earth startup without allocating
//! the offline regional catalog or a free-flight JSBSim plant, plus direct route-time
//! synchronization so high-warp runs (`8×..64×`) never stall behind display catch-up caps.

#[cfg(any(target_arch = "wasm32", test))]
use std::sync::Arc;

use sim_core::{ControlInputs, SimInstant, TICKS_PER_SECOND, TickSpan};
#[cfg(any(target_arch = "wasm32", test))]
use sim_core::{RateAdapter, RealTimeClock};
#[cfg(any(target_arch = "wasm32", test))]
use sim_spacetime::TimeScaleCatalog;

use super::Flight;
#[cfg(any(target_arch = "wasm32", test))]
use super::MAX_CATCH_UP;
#[cfg(any(target_arch = "wasm32", test))]
use super::free_world::SolarSystem;
#[cfg(any(target_arch = "wasm32", test))]
use super::mode::FreeSpawn;
use super::mode::Mode;
#[cfg(any(target_arch = "wasm32", test))]
use super::pacing::PacingMonitor;
#[cfg(any(target_arch = "wasm32", test))]
use super::panel_gauges::PanelGauges;
#[cfg(any(target_arch = "wasm32", test))]
use super::rewind::RewindHistory;
#[cfg(any(target_arch = "wasm32", test))]
use super::sensors::ScenarioSensorClock;
use crate::error::AppSimError;
#[cfg(any(target_arch = "wasm32", test))]
use crate::sensor_bus::{OnboardSensorBus, SensorRealism};
#[cfg(any(target_arch = "wasm32", test))]
use crate::vehicles::{FreeFlightVehicle, PageFeed};
#[cfg(any(target_arch = "wasm32", test))]
use crate::world_surface::WorldSurfaceHandle;

impl Flight {
    /// Starts directly on [`FreeFlightVehicle::C172Mission`] fed by `feed` over a
    /// bare-Earth [`WorldSurfaceHandle`], avoiding throwaway startup allocations of the
    /// offline regional catalog and free-flight JSBSim C172 plant.
    #[cfg(any(target_arch = "wasm32", test))]
    pub fn new_page_mission(feed: PageFeed) -> Result<Self, AppSimError> {
        let surface = WorldSurfaceHandle::new_bare_earth();
        let preferred_vehicle = FreeFlightVehicle::C172Mission;
        let time_scales = Arc::new(TimeScaleCatalog::builtin()?);
        let solar = SolarSystem::load()?;
        let mode = Mode::free(
            preferred_vehicle,
            FreeSpawn::Delaware,
            &surface,
            &time_scales,
            &solar,
            Some(&feed),
        )?;
        let epoch = mode.sim_epoch();
        let prev_vehicle = mode.vehicle();
        let mut flight = Self {
            steps: RateAdapter::new(mode.period()),
            mode,
            preferred_vehicle,
            free_spawn: FreeSpawn::Delaware,
            clock: RealTimeClock::new(MAX_CATCH_UP),
            last_frame: None,
            finish_logged: false,
            prev_vehicle,
            history: RewindHistory::default(),
            pacing: PacingMonitor::default(),
            discontinuity: 0,
            last_submitted: None,
            scenario_sensor: ScenarioSensorClock::new(),
            exposures: Vec::new(),
            exposures_emitted: 0,
            surface,
            sensors: OnboardSensorBus::new(SensorRealism::default()),
            gauges: PanelGauges::default(),
            time_scales,
            solar,
            epoch,
            model_origins: crate::vehicles::ModelOrigins::default(),
            page: Some(feed),
        };
        flight.begin_flight()?;
        Ok(flight)
    }

    /// Operator speed multiplier (`Some(Some(speed))` or `Some(None)` for unthrottled) when
    /// this flight mirrors a link's page, or `None` for standalone flights.
    #[cfg(any(target_arch = "wasm32", test))]
    #[must_use]
    pub fn page_pace(&self) -> Option<Option<f64>> {
        self.page.as_ref().map(PageFeed::pace)
    }

    /// Advances a page-driven flight directly by `due_s` route seconds without clamping to
    /// `MAX_CATCH_UP` (`0.5 s`), skipping intermediate `120 Hz` snapshot interpolations
    /// except for the final two cycles needed by sub-tick display interpolation.
    pub(super) fn advance_page_by_due(
        &mut self,
        due_s: f64,
        controls: ControlInputs,
    ) -> Result<(), AppSimError> {
        let ticks = seconds_to_ticks(due_s);
        if ticks == TickSpan::ZERO {
            return Ok(());
        }
        if let Mode::Free { world, .. } = &self.mode
            && let Some(session) = world.session()
        {
            let until_ticks = world
                .now()
                .advanced(ticks)
                .duration_since(SimInstant::START)
                .ticks_u128();
            let period_ticks = world.period().ticks_u128().max(1);
            let final_cycle = u64::try_from(until_ticks / period_ticks).unwrap_or(u64::MAX);
            session
                .block()
                .set_page_target_cycle(final_cycle.saturating_sub(1));
        }
        if self.last_submitted != Some(controls) {
            self.submit_control_at(self.master_now(), controls);
        }
        let step_s = self.mode.period().as_seconds();
        let steps = self.step_mode(ticks, controls, step_s)?;
        self.after_steps(steps);
        Ok(())
    }
}

fn seconds_to_ticks(seconds: f64) -> TickSpan {
    if !seconds.is_finite() || seconds <= 0.0 {
        return TickSpan::ZERO;
    }
    let clamped_s = seconds.min(120.0);
    let whole_s = clamped_s.trunc();
    let fract_s = clamped_s.fract();
    let subsec_ticks = (fract_s * (TICKS_PER_SECOND as f64)).round();
    let whole_ticks = (whole_s as u128).saturating_mul(u128::from(TICKS_PER_SECOND));
    TickSpan::from_ticks_u128(whole_ticks.saturating_add(subsec_ticks as u128))
}

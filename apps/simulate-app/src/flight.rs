use std::sync::Arc;

use sim_apollo::CockpitInstrumentSnapshot;
use sim_core::{
    AirframeArticulation, CameraPoseEcef, ControlInputs, RateAdapter, RealTimeClock,
    SensorExposureEvent, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState,
};
use sim_render::Camera;
use sim_scenario::{
    AirportPatternRun, OperatorViewportMode, PatternLeg, ScenarioProfile, ScenarioWorld,
    SimTimeScale,
};
use sim_spacetime::{SimEpoch, TimeScaleCatalog};
use tracing::{debug, info};
use web_time::Instant;

use crate::camera_view::{pilot_eye_mount_for, viewport_camera_for};
use crate::error::AppSimError;
use crate::sensor_bus::{OnboardSensorBus, SensorRealism};
use crate::vehicles::FreeFlightVehicle;
use crate::world_surface::WorldSurfaceHandle;

mod airport_spawn;
mod display;
mod estimate;
#[cfg(test)]
mod gauge_hud_tests;
mod manual;
mod mode;
mod pacing;
mod panel_gauges;
mod plant_block;
mod readouts;
mod rewind;
#[cfg(test)]
mod sensor_tests;
mod sensors;
#[cfg(test)]
mod spawn_tests;
#[cfg(test)]
mod tests;
mod views;

pub use manual::SpawnMode;
use mode::{FreeSpawn, Mode};
pub use pacing::{PacingMonitor, PacingSession, PacingStats};
use panel_gauges::{GaugeInputs, PanelGauges};
use rewind::{FlightCheckpoint, RewindHistory, seconds};
use sensors::ScenarioSensorClock;

/// Longest wall-clock gap simulated after a stall (`500 ms`), long enough that a transient
/// tile-compile frame loses no simulated time, short enough that a backgrounded tab does not
/// fast-forward the aircraft into the ground.
const MAX_CATCH_UP_HZ: u64 = 2;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(MAX_CATCH_UP_HZ));
const MAX_CATCH_UP: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / MAX_CATCH_UP_HZ);

/// The simulated vehicle, the clocks that pace it against real time and [`SimTimeScale`],
/// its rewind history, the `30 Hz` optical sensor exposures, and the pacing monitor.
///
/// Free flight runs inside a [`sim_core::SimSession`] (stamped inputs, fixed steps, sensor
/// channel); scripted scenarios step their own run and stamp exposures with a
/// [`RateAdapter`] at each step's exact end.
pub struct Flight {
    mode: Mode,
    preferred_vehicle: FreeFlightVehicle,
    free_spawn: FreeSpawn,
    steps: RateAdapter,
    clock: RealTimeClock,
    last_frame: Option<Instant>,
    finish_logged: bool,
    prev_vehicle: VehicleState,
    history: RewindHistory,
    pacing: PacingMonitor,
    discontinuity: u32,
    last_submitted: Option<ControlInputs>,
    scenario_sensor: ScenarioSensorClock,
    exposures: Vec<SensorExposureEvent>,
    exposures_emitted: u64,
    surface: WorldSurfaceHandle,
    sensors: OnboardSensorBus,
    gauges: PanelGauges,
    time_scales: Arc<TimeScaleCatalog>,
    epoch: SimEpoch,
    model_origins: crate::vehicles::ModelOrigins,
}

impl Flight {
    /// Starts in interactive free flight on [`FreeFlightVehicle::default`] over a world
    /// surface of its own.
    pub fn new() -> Result<Self, AppSimError> {
        Self::with_surface(WorldSurfaceHandle::new())
    }

    /// Starts in interactive free flight on [`FreeFlightVehicle::default`], with ground
    /// contact and spawn heights on `surface` (shared with the app's maps).
    pub fn with_surface(surface: WorldSurfaceHandle) -> Result<Self, AppSimError> {
        let preferred_vehicle = FreeFlightVehicle::default();
        let mode = Mode::free(preferred_vehicle, FreeSpawn::Delaware, &surface)?;
        let time_scales = Arc::new(TimeScaleCatalog::builtin()?);
        let epoch = mode.sim_epoch(&time_scales)?;
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
            epoch,
            model_origins: crate::vehicles::ModelOrigins::default(),
        };
        flight.begin_flight()?;
        Ok(flight)
    }

    /// Starts `profile` from its beginning, or returns to free flight on `None`.
    pub fn select(&mut self, profile: Option<ScenarioProfile>) -> Result<(), AppSimError> {
        self.mode = match profile {
            Some(profile) => Mode::scenario(profile)?,
            None => Mode::free(self.preferred_vehicle, self.free_spawn, &self.surface)?,
        };
        self.begin_flight()
    }

    /// Switches to `vehicle` in interactive free flight.
    pub fn select_vehicle(&mut self, vehicle: FreeFlightVehicle) -> Result<(), AppSimError> {
        self.preferred_vehicle = vehicle;
        self.respawn_free(self.free_spawn)
    }

    /// Cycles to the next [`FreeFlightVehicle`] and enters free flight.
    #[cfg(test)]
    pub fn cycle_vehicle(&mut self) -> Result<FreeFlightVehicle, AppSimError> {
        let next = self.preferred_vehicle.next();
        self.select_vehicle(next)?;
        Ok(next)
    }

    /// Restarts the active scenario, or the free flight at its spawn, from the beginning.
    pub fn restart(&mut self) -> Result<(), AppSimError> {
        self.mode = match self.profile() {
            Some(profile) => Mode::scenario(profile)?,
            None => Mode::free(self.preferred_vehicle, self.free_spawn, &self.surface)?,
        };
        self.begin_flight()
    }

    /// Forgets the last frame's wall-clock instant, so the next frame simulates no time for the
    /// gap (a pause, a menu, a rewind, or a vehicle switch is not a stall to catch up on).
    pub fn note_wall_break(&mut self) {
        self.last_frame = None;
    }

    /// Generation of the flight's timeline, advanced with `wrapping_add(1)` whenever time
    /// jumps: a new flight, a restart, or a rewind.
    #[must_use]
    pub const fn discontinuity(&self) -> u32 {
        self.discontinuity
    }

    /// Rolling real-time pacing over the last two wall-clock seconds of unpaused frames.
    #[must_use]
    pub const fn pacing_stats(&self) -> PacingStats {
        self.pacing.stats()
    }

    /// Session-wide pacing totals.
    #[must_use]
    pub const fn pacing_session(&self) -> PacingSession {
        self.pacing.session()
    }

    /// Free-flight vehicle the next free flight uses (also while a scenario flies).
    #[must_use]
    pub const fn preferred_vehicle(&self) -> FreeFlightVehicle {
        self.preferred_vehicle
    }

    /// A new flight began: fresh clocks and history, one checkpoint at its start.
    fn begin_flight(&mut self) -> Result<(), AppSimError> {
        self.epoch = self.mode.sim_epoch(&self.time_scales)?;
        self.sensors = OnboardSensorBus::new(self.sensors.realism());
        self.gauges = PanelGauges::for_pack(self.instrument_pack());
        self.history.clear();
        self.history
            .push(FlightCheckpoint::capture(&self.mode, &self.sensors));
        self.after_discontinuity();
        Ok(())
    }

    fn after_discontinuity(&mut self) {
        self.steps = RateAdapter::new(self.mode.period());
        self.clock = RealTimeClock::new(MAX_CATCH_UP);
        self.note_wall_break();
        self.finish_logged = false;
        self.prev_vehicle = self.vehicle_state();
        self.discontinuity = self.discontinuity.wrapping_add(1);
        self.last_submitted = None;
        self.scenario_sensor.rephase();
        self.exposures.clear();
        let inputs = self.gauge_inputs();
        self.gauges.settle(&inputs);
    }

    fn gauge_inputs(&self) -> GaugeInputs {
        let air = self.onboard_sensors().air_data.map(|reading| reading.value);
        GaugeInputs::sense(air.as_ref(), &self.articulation()).with_flight(
            self.now(),
            self.vehicle_state(),
            self.specific_force_frd_m_s2(),
            self.world(),
        )
    }

    /// Active scripted scenario profile, or `None` in free flight.
    #[must_use]
    pub fn profile(&self) -> Option<ScenarioProfile> {
        match &self.mode {
            Mode::Free { .. } => None,
            Mode::Scenario(run) => Some(run.profile()),
        }
    }

    /// Active [`PatternLeg`] when flying an airport traffic-pattern scenario.
    #[must_use]
    pub fn pattern_leg(&self) -> Option<PatternLeg> {
        match &self.mode {
            Mode::Free { .. } => None,
            Mode::Scenario(run) => run.pattern_run().map(AirportPatternRun::leg),
        }
    }

    /// Active [`FreeFlightVehicle`] when in free flight.
    #[must_use]
    pub fn free_vehicle(&self) -> Option<FreeFlightVehicle> {
        match &self.mode {
            Mode::Free { vehicle, .. } => Some(*vehicle),
            Mode::Scenario(_) => None,
        }
    }

    /// Central-body terrain world for the active flight.
    #[must_use]
    pub fn world(&self) -> ScenarioWorld {
        self.profile()
            .map_or(ScenarioWorld::EarthGlobe, ScenarioProfile::world)
    }

    /// Current simulation clock instant.
    #[must_use]
    pub fn now(&self) -> SimInstant {
        self.mode.now()
    }

    /// What each tick of this flight means in TDB, UTC and UT1. Dynamics,
    /// sky, air data and displays all read time from here.
    #[must_use]
    pub const fn sim_epoch(&self) -> &SimEpoch {
        &self.epoch
    }

    /// Current true `f64` ECEF [`VehicleState`].
    #[must_use]
    pub fn vehicle_state(&self) -> VehicleState {
        self.mode.vehicle()
    }

    /// Advances by wall-clock elapsed time scaled by `time_scale`, records the frame's pacing
    /// while unpaused, and returns wall-clock seconds.
    pub fn advance_to_now(
        &mut self,
        controls: ControlInputs,
        time_scale: SimTimeScale,
    ) -> Result<f64, AppSimError> {
        let now = Instant::now();
        let elapsed_s = self
            .last_frame
            .map_or(0.0, |last| now.duration_since(last).as_secs_f64());
        self.last_frame = Some(now);
        let before_s = seconds(self.now());
        let epoch = self.discontinuity;
        self.advance_by(elapsed_s, controls, time_scale)?;
        if !matches!(time_scale, SimTimeScale::Paused) && epoch == self.discontinuity {
            self.pacing
                .record(elapsed_s, seconds(self.now()) - before_s);
        }
        Ok(elapsed_s)
    }

    /// Advances deterministically by `elapsed_s` wall-clock seconds under `time_scale`,
    /// latching `controls` through the session's stamped input queue in free flight and
    /// checkpointing for rewind every half simulated second.
    pub fn advance_by(
        &mut self,
        elapsed_s: f64,
        controls: ControlInputs,
        time_scale: SimTimeScale,
    ) -> Result<(), AppSimError> {
        let ticks = self.clock.advance(elapsed_s, time_scale.factor());
        let step_s = self.mode.period().as_seconds();
        if self.last_submitted != Some(controls) {
            self.submit_control_at(self.master_now(), controls);
        }
        let steps = match &mut self.mode {
            Mode::Free { session, panel, .. } => {
                let steps = session
                    .advance(ticks)
                    .map_err(|err| session.block().take_fault().unwrap_or(err.into()))?;
                self.exposures.extend(session.take_exposures());
                mode::observe(panel, session.block().plant(), session.physics_time());
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
        };
        self.after_steps(steps);
        Ok(())
    }

    fn after_steps(&mut self, steps: u64) {
        let dropped = sensors::trim_pending(&mut self.exposures);
        if dropped > 0 {
            debug!(
                dropped,
                "optical sensor exposures were not drained; oldest dropped"
            );
        }
        if steps > 0 {
            let inputs = self.gauge_inputs();
            let step_s = self.mode.period().as_seconds();
            for _ in 0..steps {
                self.gauges.step(step_s, &inputs);
            }
            self.commit_live_branch();
        }
        if steps > 0 && self.history.due(self.now()) {
            self.history
                .push(FlightCheckpoint::capture(&self.mode, &self.sensors));
        }
    }

    /// Master-clock instant handed to the flight so far (at or after [`Self::now`]).
    fn master_now(&self) -> SimInstant {
        match &self.mode {
            Mode::Free { session, .. } => session.now(),
            Mode::Scenario(run) => run.now(),
        }
    }

    /// Queues `controls` to take effect at the first free-flight physics step starting at
    /// or after `at` (clamped forward to the last queued instant), returning the instant
    /// used. Scripted scenarios fly their own controls and ignore it.
    pub fn submit_control_at(&mut self, at: SimInstant, controls: ControlInputs) -> SimInstant {
        self.last_submitted = Some(controls);
        match &mut self.mode {
            Mode::Free { session, .. } => session.submit_at(at, controls),
            Mode::Scenario(_) => at,
        }
    }

    /// Hands over the `30 Hz` optical sensor exposures emitted since the last call, oldest
    /// first, each carrying the exact exposure instant and truth state (never the
    /// display-interpolated pose).
    pub fn drain_sensor_exposures(&mut self) -> Vec<SensorExposureEvent> {
        let drained = std::mem::take(&mut self.exposures);
        self.exposures_emitted = self
            .exposures_emitted
            .wrapping_add(u64::try_from(drained.len()).unwrap_or(u64::MAX));
        drained
    }

    /// Exposures handed over by [`Self::drain_sensor_exposures`] since the app started.
    #[cfg(test)]
    #[must_use]
    pub const fn sensor_exposures_drained(&self) -> u64 {
        self.exposures_emitted
    }

    /// The vehicle as the window draws it: interpolated between the last two physics steps by
    /// the step clock's sub-tick phase. Physics and tests read [`Self::vehicle_state`].
    #[must_use]
    pub fn display_vehicle_state(&self) -> VehicleState {
        match &self.mode {
            Mode::Free { session, .. } => session.display_state(),
            Mode::Scenario(run) => {
                display::interpolate(&self.prev_vehicle, &run.vehicle(), self.steps.phase())
            }
        }
    }

    /// Ellipsoidal height of the ground under the displayed vehicle, where the surface
    /// knows it.
    #[must_use]
    pub fn display_ground_height_m(&self) -> Option<f64> {
        let position = self.display_vehicle_state().position_ecef;
        self.surface
            .ellipsoidal_ground_m(&sim_geo::ecef_to_geodetic(position))
    }

    /// Sub-step display phase in `[0, 1)`: how far the master clock has run past the last
    /// completed physics step, in plant periods.
    fn display_phase(&self) -> f64 {
        match &self.mode {
            Mode::Free { session, .. } => {
                let ahead = session.now().duration_since(session.physics_time());
                let period = self.mode.period().ticks_u128();
                if period == 0 {
                    0.0
                } else {
                    ahead.ticks_u128() as f64 / period as f64
                }
            }
            Mode::Scenario(_) => self.steps.phase(),
        }
    }

    /// Articulation with the propeller advanced smoothly through the current sub-tick phase.
    #[must_use]
    pub fn display_articulation(&self) -> AirframeArticulation {
        let mut art = display::advance_propeller(
            self.articulation(),
            self.display_phase(),
            self.mode.period().as_seconds(),
        );
        art.instrument_needles = self.gauges.needles();
        art
    }

    /// Cockpit instrument snapshot as of the latest simulation step.
    #[must_use]
    pub fn cockpit(&self) -> CockpitInstrumentSnapshot {
        match &self.mode {
            Mode::Free { panel, .. } => panel.snapshot(),
            Mode::Scenario(run) => run.cockpit(),
        }
    }

    /// Aboard pilot eye pose in body-fixed ECEF coordinates, the eye mounted on the drawn
    /// airframe.
    pub fn aboard_eye_pose(&self) -> Result<CameraPoseEcef, AppSimError> {
        match &self.mode {
            Mode::Free {
                vehicle, session, ..
            } => Ok(pilot_eye_mount_for(*vehicle)
                .pose(&session.truth_state(), -self.model_origin_from_cg_frd_m())),
            Mode::Scenario(run) => Ok(run.eye_pose()?),
        }
    }

    /// Camera positioned for `viewport_mode` on [`Self::display_vehicle_state`].
    pub fn display_camera(
        &self,
        viewport_mode: OperatorViewportMode,
    ) -> Result<Camera, AppSimError> {
        let exact = self.vehicle_state();
        let shown = self.display_vehicle_state();
        let aboard = display::reanchor(self.aboard_eye_pose()?, &exact, &shown);
        Ok(viewport_camera_for(
            viewport_mode,
            self.profile(),
            self.chase_back_up_m(),
            &shown,
            aboard,
        ))
    }
}

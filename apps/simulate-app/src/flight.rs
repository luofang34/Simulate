use glam::DVec3;
use sim_apollo::{CockpitInstrumentPanel, CockpitInstrumentSnapshot, EarthVehicleSources};
use sim_core::{
    CameraPoseEcef, ControlInputs, EngineStatus, RateAdapter, RealTimeClock, SimInstant,
    SystemControls, TICKS_PER_SECOND, TickSpan, VehicleState,
};
use sim_geo::ecef_to_geodetic;
use sim_render::Camera;
use sim_scenario::{
    OperatorViewportMode, ScenarioProfile, ScenarioRun, ScenarioWorld, SimTimeScale,
};
use tracing::info;
use web_time::Instant;

use crate::camera_view::{pilot_eye_mount, viewport_camera};
use crate::error::AppSimError;
use crate::vehicles::{FreeFlightPlant, FreeFlightVehicle};

/// Longest wall-clock gap simulated after a stall, so a backgrounded tab does
/// not fast-forward the aircraft into the ground.
const MAX_CATCH_UP_HZ: u64 = 4;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(MAX_CATCH_UP_HZ));
const MAX_CATCH_UP: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / MAX_CATCH_UP_HZ);

/// What the vehicle is doing: flown by hand on a selectable plant, or playing a scripted scenario.
enum Mode {
    Free {
        vehicle: FreeFlightVehicle,
        plant: Box<FreeFlightPlant>,
        panel: Box<CockpitInstrumentPanel>,
        now: SimInstant,
    },
    Scenario(Box<ScenarioRun>),
}

impl Mode {
    fn free(vehicle: FreeFlightVehicle) -> Result<Self, AppSimError> {
        let plant = FreeFlightPlant::new(vehicle)?;
        let mut panel = Box::new(CockpitInstrumentPanel::new(SimInstant::START));
        let state = plant.state();
        panel.observe_earth(&EarthVehicleSources {
            timestamp: SimInstant::START,
            vehicle: state,
            height_m: ecef_to_geodetic(state.position_ecef).height_m,
        });
        Ok(Self::Free {
            vehicle,
            plant: Box::new(plant),
            panel,
            now: SimInstant::START,
        })
    }

    fn period(&self) -> TickSpan {
        match self {
            Self::Free { plant, .. } => plant.period(),
            Self::Scenario(run) => run.period(),
        }
    }
}

/// The simulated vehicle and the clocks that pace it against real time and [`SimTimeScale`].
pub struct Flight {
    mode: Mode,
    preferred_vehicle: FreeFlightVehicle,
    steps: RateAdapter,
    clock: RealTimeClock,
    last_frame: Option<Instant>,
    finish_logged: bool,
}

impl Flight {
    /// Starts in interactive free flight on [`FreeFlightVehicle::default`] (falling back to
    /// [`FreeFlightVehicle::Kinematic`] if an optional asset fails to load).
    pub fn new() -> Result<Self, AppSimError> {
        let preferred_vehicle = FreeFlightVehicle::default();
        let mode = Mode::free(preferred_vehicle)?;
        Ok(Self {
            steps: RateAdapter::new(mode.period()),
            mode,
            preferred_vehicle,
            clock: RealTimeClock::new(MAX_CATCH_UP),
            last_frame: None,
            finish_logged: false,
        })
    }

    /// Starts `profile` from its beginning, or returns to free flight on `None`.
    pub fn select(&mut self, profile: Option<ScenarioProfile>) -> Result<(), AppSimError> {
        self.mode = match profile {
            Some(profile) => Mode::Scenario(Box::new(ScenarioRun::new(profile)?)),
            None => Mode::free(self.preferred_vehicle)?,
        };
        self.reset_clocks();
        Ok(())
    }

    /// Switches to `vehicle` in interactive free flight.
    pub fn select_vehicle(&mut self, vehicle: FreeFlightVehicle) -> Result<(), AppSimError> {
        self.preferred_vehicle = vehicle;
        self.mode = Mode::free(vehicle)?;
        self.reset_clocks();
        Ok(())
    }

    /// Cycles to the next [`FreeFlightVehicle`] and enters free flight.
    pub fn cycle_vehicle(&mut self) -> Result<FreeFlightVehicle, AppSimError> {
        let next = self.preferred_vehicle.next();
        self.select_vehicle(next)?;
        Ok(next)
    }

    fn reset_clocks(&mut self) {
        self.steps = RateAdapter::new(self.mode.period());
        self.clock = RealTimeClock::new(MAX_CATCH_UP);
        self.last_frame = None;
        self.finish_logged = false;
    }

    /// Active scripted scenario profile, or `None` in free flight.
    #[must_use]
    pub fn profile(&self) -> Option<ScenarioProfile> {
        match &self.mode {
            Mode::Free { .. } => None,
            Mode::Scenario(run) => Some(run.profile()),
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
        match &self.mode {
            Mode::Free { now, .. } => *now,
            Mode::Scenario(run) => run.now(),
        }
    }

    /// Current true `f64` ECEF [`VehicleState`].
    #[must_use]
    pub fn vehicle_state(&self) -> VehicleState {
        match &self.mode {
            Mode::Free { plant, .. } => plant.state(),
            Mode::Scenario(run) => run.vehicle(),
        }
    }

    /// Body FRD specific force (`m/s²`), when provided by the plant.
    #[must_use]
    pub fn specific_force_frd_m_s2(&self) -> Option<DVec3> {
        match &self.mode {
            Mode::Free { plant, .. } => plant.specific_force_frd_m_s2(),
            Mode::Scenario(_) => None,
        }
    }

    /// Active [`SystemControls`] on JSBSim free-flight aircraft.
    #[must_use]
    pub fn system_controls(&self) -> Option<SystemControls> {
        match &self.mode {
            Mode::Free { plant, .. } => plant.system_controls(),
            Mode::Scenario(_) => None,
        }
    }

    /// Applies `f` to the latched [`SystemControls`] on JSBSim free-flight aircraft.
    pub fn update_system_controls(&mut self, f: impl FnOnce(&mut SystemControls)) {
        if let Mode::Free { plant, .. } = &mut self.mode
            && let Some(mut sys) = plant.system_controls()
        {
            f(&mut sys);
            plant.set_system_controls(sys);
        }
    }

    /// Latest engine status (`running`, `propeller_rpm`) on JSBSim free-flight aircraft.
    #[must_use]
    pub fn engine_status(&self) -> Option<EngineStatus> {
        match &self.mode {
            Mode::Free { plant, .. } => plant.engine_status(),
            Mode::Scenario(_) => None,
        }
    }

    /// Whether an Aviate direct flight controller is armed (`None` on non-Aviate flights).
    #[must_use]
    pub fn aviate_armed(&self) -> Option<bool> {
        match &self.mode {
            Mode::Free { plant, .. } => plant.aviate_armed(),
            Mode::Scenario(_) => None,
        }
    }

    /// Advances by wall-clock elapsed time scaled by `time_scale` and returns wall-clock seconds.
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
        self.advance_by(elapsed_s, controls, time_scale)?;
        Ok(elapsed_s)
    }

    /// Advances deterministically by `elapsed_s` wall-clock seconds under `time_scale`.
    pub fn advance_by(
        &mut self,
        elapsed_s: f64,
        controls: ControlInputs,
        time_scale: SimTimeScale,
    ) -> Result<(), AppSimError> {
        let ticks = self.clock.advance(elapsed_s, time_scale.factor());
        let steps = self.steps.steps_for(ticks);
        match &mut self.mode {
            Mode::Free {
                plant,
                panel,
                now: sim_now,
                ..
            } => {
                for _ in 0..steps {
                    plant.step(controls)?;
                    *sim_now = sim_now.advanced(plant.period());
                }
                let vehicle = plant.state();
                panel.observe_earth(&EarthVehicleSources {
                    timestamp: *sim_now,
                    vehicle,
                    height_m: ecef_to_geodetic(vehicle.position_ecef).height_m,
                });
            }
            Mode::Scenario(run) => {
                for _ in 0..steps {
                    run.step()?;
                }
                if run.finished() && !self.finish_logged {
                    self.finish_logged = true;
                    info!(scenario = run.profile().title(), "scenario complete");
                }
            }
        }
        Ok(())
    }

    /// Cockpit instrument snapshot as of the latest simulation step.
    #[must_use]
    pub fn cockpit(&self) -> CockpitInstrumentSnapshot {
        match &self.mode {
            Mode::Free { panel, .. } => panel.snapshot(),
            Mode::Scenario(run) => run.cockpit(),
        }
    }

    /// Aboard pilot eye pose in body-fixed ECEF coordinates.
    pub fn aboard_eye_pose(&self) -> Result<CameraPoseEcef, AppSimError> {
        match &self.mode {
            Mode::Free { plant, .. } => Ok(pilot_eye_mount().pose(&plant.state(), DVec3::ZERO)),
            Mode::Scenario(run) => Ok(run.eye_pose()?),
        }
    }

    /// Camera positioned for `viewport_mode`.
    pub fn camera(&self, viewport_mode: OperatorViewportMode) -> Result<Camera, AppSimError> {
        let state = self.vehicle_state();
        let aboard = self.aboard_eye_pose()?;
        Ok(viewport_camera(viewport_mode, &state, aboard))
    }
}

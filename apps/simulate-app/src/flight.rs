use sim_apollo::{CockpitInstrumentPanel, CockpitInstrumentSnapshot, EarthVehicleSources};
use sim_core::{
    Block, ControlInputs, KinematicInput, KinematicModel, RateAdapter, RealTimeClock, SimInstant,
    TICKS_PER_SECOND, TickSpan, VehicleState,
};
use sim_geo::{Geodetic, ecef_to_geodetic};
use sim_render::Camera;
use sim_scenario::{ScenarioError, ScenarioProfile, ScenarioRun, ScenarioWorld};
use tracing::info;
use web_time::Instant;

use crate::camera_view::{cockpit_camera, scenario_camera};

/// Starting point: over San Francisco Bay, heading north at 1,000 m.
const START_LAT_DEG: f64 = 37.62;
const START_LON_DEG: f64 = -122.38;
const START_HEIGHT_M: f64 = 1_000.0;
const START_SPEED_M_S: f64 = 60.0;
/// Longest wall-clock gap simulated after a stall, so a backgrounded tab does
/// not fast-forward the aircraft into the ground.
const MAX_CATCH_UP_HZ: u64 = 4;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(MAX_CATCH_UP_HZ));
const MAX_CATCH_UP: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / MAX_CATCH_UP_HZ);

/// What the vehicle is doing: flown by hand, or playing a scenario.
enum Mode {
    Free {
        model: KinematicModel,
        panel: Box<CockpitInstrumentPanel>,
        now: SimInstant,
    },
    Scenario(Box<ScenarioRun>),
}

impl Mode {
    fn free() -> Self {
        let start = Geodetic::from_degrees(START_LAT_DEG, START_LON_DEG, START_HEIGHT_M);
        Self::Free {
            model: KinematicModel::new(VehicleState::level_flight(&start, 0.0, START_SPEED_M_S)),
            panel: Box::new(CockpitInstrumentPanel::new(SimInstant::START)),
            now: SimInstant::START,
        }
    }

    fn period(&self) -> TickSpan {
        match self {
            Self::Free { model, .. } => model.period(),
            Self::Scenario(run) => run.period(),
        }
    }
}

/// The simulated vehicle and the clocks that pace it against real time.
pub struct Flight {
    mode: Mode,
    steps: RateAdapter,
    clock: RealTimeClock,
    last_frame: Option<Instant>,
    finish_logged: bool,
}

impl Flight {
    pub fn new() -> Self {
        let mode = Mode::free();
        Self {
            steps: RateAdapter::new(mode.period()),
            mode,
            clock: RealTimeClock::new(MAX_CATCH_UP),
            last_frame: None,
            finish_logged: false,
        }
    }

    /// Starts `profile` from its beginning, or free flight for `None`.
    pub fn select(&mut self, profile: Option<ScenarioProfile>) -> Result<(), ScenarioError> {
        self.mode = match profile {
            Some(profile) => Mode::Scenario(Box::new(ScenarioRun::new(profile)?)),
            None => Mode::free(),
        };
        self.steps = RateAdapter::new(self.mode.period());
        self.clock = RealTimeClock::new(MAX_CATCH_UP);
        self.last_frame = None;
        self.finish_logged = false;
        Ok(())
    }

    /// The scenario being flown, if any.
    pub fn profile(&self) -> Option<ScenarioProfile> {
        match &self.mode {
            Mode::Free { .. } => None,
            Mode::Scenario(run) => Some(run.profile()),
        }
    }

    /// The terrain to draw.
    pub fn world(&self) -> ScenarioWorld {
        self.profile()
            .map_or(ScenarioWorld::EarthGlobe, ScenarioProfile::world)
    }

    /// Advances the simulation by the wall-clock time since the last call and
    /// returns that time in seconds.
    pub fn advance_to_now(&mut self, controls: ControlInputs) -> Result<f64, ScenarioError> {
        let now = Instant::now();
        let elapsed_s = self
            .last_frame
            .map_or(0.0, |last| now.duration_since(last).as_secs_f64());
        self.last_frame = Some(now);
        let ticks = self.clock.advance(elapsed_s, 1.0);
        let steps = self.steps.steps_for(ticks);
        match &mut self.mode {
            Mode::Free {
                model,
                panel,
                now: sim_now,
            } => {
                let input = KinematicInput { controls };
                for _ in 0..steps {
                    model.step(&input)?;
                    *sim_now = sim_now.advanced(model.period());
                }
                let vehicle = model.output();
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
        Ok(elapsed_s)
    }

    /// The cockpit as of the last advance.
    pub fn cockpit(&self) -> CockpitInstrumentSnapshot {
        match &self.mode {
            Mode::Free { panel, .. } => panel.snapshot(),
            Mode::Scenario(run) => run.cockpit(),
        }
    }

    /// The view from aboard.
    pub fn camera(&self) -> Result<Camera, ScenarioError> {
        match &self.mode {
            Mode::Free { model, .. } => Ok(cockpit_camera(&model.output())),
            Mode::Scenario(run) => Ok(scenario_camera(run.eye_pose()?)),
        }
    }
}

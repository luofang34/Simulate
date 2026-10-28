use fsg_geo::Geodetic;
use fsg_sim::{
    Block, BlockError, ControlInputs, KinematicInput, KinematicModel, RateAdapter, RealTimeClock,
    TickSpan, VehicleState,
};
use web_time::Instant;

/// Starting point: over San Francisco Bay, heading north at 1,000 m.
const START_LAT_DEG: f64 = 37.62;
const START_LON_DEG: f64 = -122.38;
const START_HEIGHT_M: f64 = 1_000.0;
const START_SPEED_M_S: f64 = 60.0;
/// Longest wall-clock gap simulated after a stall, so a backgrounded tab does
/// not fast-forward the aircraft into the ground.
const MAX_CATCH_UP_HZ: u64 = 4;

/// The simulated vehicle and the clocks that pace it against real time.
pub struct Flight {
    model: KinematicModel,
    steps: RateAdapter,
    clock: RealTimeClock,
    last_frame: Option<Instant>,
}

impl Flight {
    pub fn new() -> Self {
        let start = Geodetic::from_degrees(START_LAT_DEG, START_LON_DEG, START_HEIGHT_M);
        let model = KinematicModel::new(VehicleState::level_flight(&start, 0.0, START_SPEED_M_S));
        let max_catch_up = TickSpan::period_of_hz(MAX_CATCH_UP_HZ).unwrap_or(TickSpan::ZERO);
        Self {
            steps: RateAdapter::new(model.period()),
            model,
            clock: RealTimeClock::new(max_catch_up),
            last_frame: None,
        }
    }

    /// Advances the simulation by the wall-clock time since the last call and
    /// returns that time in seconds.
    pub fn advance_to_now(&mut self, controls: ControlInputs) -> Result<f64, BlockError> {
        let now = Instant::now();
        let elapsed_s = self
            .last_frame
            .map_or(0.0, |last| now.duration_since(last).as_secs_f64());
        self.last_frame = Some(now);
        let ticks = self.clock.advance(elapsed_s, 1.0);
        let input = KinematicInput { controls };
        for _ in 0..self.steps.steps_for(ticks) {
            self.model.step(&input)?;
        }
        Ok(elapsed_s)
    }

    pub fn state(&self) -> VehicleState {
        self.model.output()
    }
}

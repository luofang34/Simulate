//! Stepping a flight in a page without freezing it: the flight advances in
//! slices of a few milliseconds of wall time, the page draws between them,
//! and the pace follows the wall clock at a chosen speed.
//!
//! The steps are the flight's own fixed ticks, so what the flight produces
//! does not depend on how its steps were sliced: only when they run does.

use super::MonitorFeed;

/// Wall time one slice may take before the page gets the thread back,
/// seconds: half a frame at 60 Hz, so the page still draws every frame.
/// A single step longer than this (a navigator frame) still runs whole.
pub const SLICE_BUDGET_S: f64 = 0.008;
/// The most wall time one frame may add to the pace, seconds. A hidden tab
/// gets no frames; when it shows again the flight goes on from where it
/// was instead of racing to catch up, and a flight slower than its pace
/// falls behind instead of building a debt.
pub const MAX_FRAME_S: f64 = 0.1;

/// Whether a flight goes on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepState {
    /// It has more to fly.
    Running,
    /// It has ended; further steps do nothing.
    Finished,
}

/// A flight that advances one fixed tick at a time.
pub trait FlightStepper {
    /// Why a step failed.
    type Error;

    /// Route time reached, seconds.
    fn sim_time_s(&self) -> f64;

    /// Advances one tick, adding what it produced to `out`.
    ///
    /// # Errors
    ///
    /// The flight's own failure; the flight is then over.
    fn step(&mut self, out: &mut Vec<MonitorFeed>) -> Result<StepState, Self::Error>;
}

/// The wall clock, seconds from any fixed origin.
pub trait Clock {
    /// Now.
    fn now_s(&self) -> f64;
}

/// Where the flight should be on the wall clock.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pacer {
    speed: Option<f64>,
    /// The route time due; `None` until the next frame takes it from the
    /// flight, at the start and after a change of speed.
    target_s: Option<f64>,
    last_frame_s: Option<f64>,
}

impl Pacer {
    /// A pace of `speed` route seconds per wall second; `None` for as fast
    /// as the page can step.
    pub const fn new(speed: Option<f64>) -> Self {
        Self {
            speed,
            target_s: None,
            last_frame_s: None,
        }
    }

    /// The speed.
    pub const fn speed(&self) -> Option<f64> {
        self.speed
    }

    /// Changes the speed from now on: the pace starts again from where the
    /// flight is.
    pub const fn set_speed(&mut self, speed: Option<f64>) {
        self.speed = speed;
        self.target_s = None;
    }

    /// The route time to step to in the frame at `wall_s`, with the flight
    /// at `sim_s`: the wall time since the last frame, at most
    /// [`MAX_FRAME_S`], times the speed, added to what was due. The due
    /// time runs on its own, so a flight whose tick is longer than a frame
    /// still keeps the pace, waiting out the frames its last tick ran
    /// ahead; it is never more than [`MAX_FRAME_S`] of pace ahead of the
    /// flight, so a flight slower than the pace builds no debt.
    pub fn frame(&mut self, wall_s: f64, sim_s: f64) -> f64 {
        let elapsed = self
            .last_frame_s
            .map_or(0.0, |last| (wall_s - last).clamp(0.0, MAX_FRAME_S));
        self.last_frame_s = Some(wall_s);
        let Some(speed) = self.speed else {
            return f64::INFINITY;
        };
        let due = self.target_s.unwrap_or(sim_s) + elapsed * speed;
        let target = due.min(sim_s + MAX_FRAME_S * speed);
        self.target_s = Some(target);
        target
    }
}

/// What one slice did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Slice {
    /// Steps taken.
    pub steps: usize,
    /// Whether the flight ended in it.
    pub finished: bool,
}

/// Steps `flight` toward `target_s` until it is reached, the flight ends,
/// or `budget_s` of wall time on `clock` has passed. A flight behind its
/// target always takes one step, however long the step is.
///
/// # Errors
///
/// The first failed step.
pub fn run_slice<F: FlightStepper, C: Clock>(
    flight: &mut F,
    clock: &C,
    budget_s: f64,
    target_s: f64,
    out: &mut Vec<MonitorFeed>,
) -> Result<Slice, F::Error> {
    let start = clock.now_s();
    let mut slice = Slice {
        steps: 0,
        finished: false,
    };
    while flight.sim_time_s() < target_s {
        if slice.steps > 0 && clock.now_s() - start >= budget_s {
            break;
        }
        let state = flight.step(out)?;
        slice.steps = slice.steps.wrapping_add(1);
        if state == StepState::Finished {
            slice.finished = true;
            break;
        }
    }
    Ok(slice)
}

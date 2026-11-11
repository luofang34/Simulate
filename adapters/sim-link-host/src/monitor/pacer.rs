//! Stepping a flight in a page without freezing it: the flight advances in
//! slices of a few milliseconds of wall time, the page draws between them,
//! and the pace follows the wall clock at a chosen speed.
//!
//! The steps are the flight's own fixed ticks, so what the flight produces
//! does not depend on how its steps were sliced: only when they run does.
//! What a route second costs on the wall clock is measured as the flight
//! goes, and the pace is held to what the page can afford.

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

/// The share of the wall clock the flight may take at a held pace; the
/// rest is the page's, to draw and to answer the visitor.
pub const FLIGHT_SHARE: f64 = 0.5;
/// How much of the measured cost each new slice leaves standing: the cost
/// follows a change in the flight within a few dozen slices.
const COST_MEMORY: f64 = 0.95;
/// Route time measured before the pace is held, seconds: the first frames
/// include one-off work.
const COST_WARM_UP_S: f64 = 2.0;

/// Whether a flight goes on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepState {
    /// It has more to fly.
    Running,
    /// It has ended; further steps do nothing.
    Finished,
}

/// A flight that advances in fixed ticks.
pub trait FlightStepper {
    /// Why a step failed.
    type Error;

    /// Route time reached, seconds.
    fn sim_time_s(&self) -> f64;

    /// Advances toward `target_s`, at least one tick and at most to the
    /// first tick at or past it, adding what it produced to `out`. What a
    /// flight produces must not depend on the targets it was stepped to.
    ///
    /// # Errors
    ///
    /// The flight's own failure; the flight is then over.
    fn step(&mut self, target_s: f64, out: &mut Vec<MonitorFeed>)
    -> Result<StepState, Self::Error>;

    /// Wall time spent rendering terrain-camera frames since the last
    /// call, seconds, for a flight whose host renders them; the pace says
    /// when they are what holds it back.
    fn take_camera_wall_s(&mut self) -> f64 {
        0.0
    }
}

/// The wall clock, seconds from any fixed origin.
pub trait Clock {
    /// Now.
    fn now_s(&self) -> f64;
}

/// A pace the page cannot afford, and the one it holds instead.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PaceLimit {
    /// The pace held, route seconds per wall second.
    pub speed: f64,
    /// Whether rendering the terrain camera's frames took most of the
    /// measured cost.
    pub by_camera: bool,
}

/// Where the flight should be on the wall clock.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pacer {
    speed: Option<f64>,
    /// The route time due; `None` until the next frame takes it from the
    /// flight, at the start and after a change of speed.
    target_s: Option<f64>,
    last_frame_s: Option<f64>,
    /// Decayed sums of the route time stepped, the wall time it took, and
    /// the part of that spent rendering camera frames.
    route_s: f64,
    wall_s: f64,
    camera_s: f64,
    observed_route_s: f64,
}

impl Pacer {
    /// A pace of `speed` route seconds per wall second; `None` for as fast
    /// as the page can step.
    pub const fn new(speed: Option<f64>) -> Self {
        Self {
            speed,
            target_s: None,
            last_frame_s: None,
            route_s: 0.0,
            wall_s: 0.0,
            camera_s: 0.0,
            observed_route_s: 0.0,
        }
    }

    /// The speed chosen.
    pub const fn speed(&self) -> Option<f64> {
        self.speed
    }

    /// Changes the speed from now on: the pace starts again from where the
    /// flight is.
    pub const fn set_speed(&mut self, speed: Option<f64>) {
        self.speed = speed;
        self.target_s = None;
    }

    /// Takes what a slice cost: the route time it stepped, and of its wall
    /// time the part spent rendering camera frames.
    pub fn observe(&mut self, slice: &Slice, camera_wall_s: f64) {
        if slice.route_s <= 0.0 || slice.wall_s.is_nan() {
            return;
        }
        self.route_s = self.route_s * COST_MEMORY + slice.route_s;
        self.wall_s = self.wall_s * COST_MEMORY + slice.wall_s.max(0.0);
        self.camera_s = self.camera_s * COST_MEMORY + camera_wall_s.clamp(0.0, slice.wall_s);
        self.observed_route_s += slice.route_s;
    }

    /// The pace the page can afford, when the chosen one is more: the
    /// route seconds a wall second buys at [`FLIGHT_SHARE`] of the clock.
    /// As fast as possible is never held: it is the page's limit already.
    pub fn limit(&self) -> Option<PaceLimit> {
        let chosen = self.speed?;
        if self.observed_route_s < COST_WARM_UP_S || self.wall_s <= 0.0 {
            return None;
        }
        let affordable = FLIGHT_SHARE * self.route_s / self.wall_s;
        (affordable < chosen).then_some(PaceLimit {
            speed: affordable,
            by_camera: self.camera_s >= 0.5 * self.wall_s,
        })
    }

    /// The route time to step to in the frame at `wall_s`, with the flight
    /// at `sim_s`: the wall time since the last frame, at most
    /// [`MAX_FRAME_S`], times the pace held, added to what was due. The
    /// due time runs on its own, so a flight whose tick is longer than a
    /// frame still keeps the pace, waiting out the frames its last tick ran
    /// ahead; it is never more than [`MAX_FRAME_S`] of pace ahead of the
    /// flight, so a flight slower than the pace builds no debt.
    pub fn frame(&mut self, wall_s: f64, sim_s: f64) -> f64 {
        let elapsed = self
            .last_frame_s
            .map_or(0.0, |last| (wall_s - last).clamp(0.0, MAX_FRAME_S));
        self.last_frame_s = Some(wall_s);
        let Some(chosen) = self.speed else {
            return f64::INFINITY;
        };
        let speed = self.limit().map_or(chosen, |limit| limit.speed);
        let due = self.target_s.unwrap_or(sim_s) + elapsed * speed;
        let target = due.min(sim_s + MAX_FRAME_S * speed);
        self.target_s = Some(target);
        target
    }
}

/// What one slice did.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Slice {
    /// Steps taken.
    pub steps: usize,
    /// Whether the flight ended in it.
    pub finished: bool,
    /// Route time stepped, seconds.
    pub route_s: f64,
    /// Wall time taken, seconds.
    pub wall_s: f64,
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
    let (start, from_s) = (clock.now_s(), flight.sim_time_s());
    let mut slice = Slice {
        steps: 0,
        finished: false,
        route_s: 0.0,
        wall_s: 0.0,
    };
    while flight.sim_time_s() < target_s {
        if slice.steps > 0 && clock.now_s() - start >= budget_s {
            break;
        }
        let state = flight.step(target_s, out)?;
        slice.steps = slice.steps.wrapping_add(1);
        if state == StepState::Finished {
            slice.finished = true;
            break;
        }
    }
    slice.route_s = flight.sim_time_s() - from_s;
    slice.wall_s = clock.now_s() - start;
    Ok(slice)
}

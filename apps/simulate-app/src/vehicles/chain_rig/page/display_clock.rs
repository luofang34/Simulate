//! The route time the app draws a page's flight at. The page publishes its flight once a
//! frame, after a slice of uneven length, and sometimes not for hundreds of milliseconds
//! while one long step runs; drawing the newest published instant would move the aircraft
//! in uneven jumps. The display time instead runs smoothly at the flight's measured pace,
//! a little behind the newest instant, and after a gap catches up at a bounded rate.

/// How far behind the newest instant the display runs, in wall seconds at the measured
/// pace: two frames at 60 Hz, enough to absorb one late or short slice.
const LAG_WALL_S: f64 = 2.0 / 60.0;
/// Wall seconds over which the measured pace follows the flight's.
const RATE_MEMORY_S: f64 = 1.0;
/// A frame longer than this, seconds, is a gap: it neither teaches the pace nor moves the
/// display more than this frame's worth.
const MAX_FRAME_S: f64 = 1.0 / 30.0;
/// Share of the remaining lag error made up per wall second.
const CATCH_UP_PER_S: f64 = 3.0;
/// Most the display may run ahead of the pace while catching up, as a share of it.
const MAX_CATCH_UP: f64 = 0.5;

/// A display time following a page's published route time.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DisplayClock {
    display_s: Option<f64>,
    rate: Option<f64>,
    newest_s: Option<f64>,
}

impl DisplayClock {
    /// The display route time after `wall_dt_s` wall seconds, the page's newest and oldest
    /// kept instants being `newest_s` and `oldest_s`. It never runs backwards, past the
    /// newest instant, or behind the oldest.
    pub fn advance(&mut self, wall_dt_s: f64, newest_s: f64, oldest_s: f64) -> f64 {
        let wall_dt_s = if wall_dt_s.is_finite() {
            wall_dt_s.max(0.0)
        } else {
            0.0
        };
        if let Some(previous) = self.newest_s
            && wall_dt_s > 0.0
            && wall_dt_s <= MAX_FRAME_S
        {
            let sample = ((newest_s - previous) / wall_dt_s).max(0.0);
            let keep = (wall_dt_s / RATE_MEMORY_S).min(1.0);
            self.rate = Some(
                self.rate
                    .map_or(sample, |rate| rate + keep * (sample - rate)),
            );
        }
        self.newest_s = Some(newest_s);
        let Some(display) = self.display_s else {
            self.display_s = Some(newest_s);
            return newest_s;
        };
        let rate = self.rate.unwrap_or(0.0);
        let dt = wall_dt_s.min(MAX_FRAME_S);
        let nominal = display + rate * dt;
        let error = newest_s - rate * LAG_WALL_S - nominal;
        let bound = MAX_CATCH_UP * rate * dt;
        let correction = (error * (CATCH_UP_PER_S * dt).min(1.0)).clamp(-bound, bound);
        let next = (nominal + correction)
            .max(display)
            .max(oldest_s)
            .min(newest_s);
        self.display_s = Some(next);
        next
    }

    /// Advances monotonically to `newest_s` for high-warp (`>= 8×` or unthrottled) runs
    /// whose slices span multiple display refreshes and advance seconds per slice.
    pub fn jump_to(&mut self, newest_s: f64) -> f64 {
        self.newest_s = Some(newest_s);
        let next = self
            .display_s
            .map_or(newest_s, |display| newest_s.max(display));
        self.display_s = Some(next);
        next
    }
}

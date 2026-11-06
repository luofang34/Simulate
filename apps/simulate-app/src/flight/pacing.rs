//! Real-time pacing monitor: how much simulated time each wall-clock second delivers.
//!
//! A ratio below the requested time scale means frames took longer than the catch-up
//! limit allows, so simulated seconds were dropped rather than fast-forwarded.

use std::collections::VecDeque;

/// Wall-clock span the rolling statistics cover, seconds.
pub const PACING_WINDOW_S: f64 = 2.0;

/// Rolling pacing statistics over the last [`PACING_WINDOW_S`] of unpaused frames.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PacingStats {
    /// Simulated seconds per wall-clock second (`1.00` at real time, the warp factor under warp).
    pub real_time_ratio: f64,
    /// Frames per wall-clock second.
    pub fps: f64,
    /// Mean wall-clock frame time, milliseconds.
    pub mean_frame_ms: f64,
    /// Longest wall-clock frame time, milliseconds.
    pub max_frame_ms: f64,
}

/// Session-wide pacing totals over every unpaused frame.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PacingSession {
    /// Unpaused frames recorded.
    pub frames: u64,
    /// Unpaused wall-clock seconds.
    pub wall_s: f64,
    /// Simulated seconds delivered during them.
    pub sim_s: f64,
}

impl PacingSession {
    /// Simulated over wall-clock seconds across the session (`0` before any frame).
    #[must_use]
    pub fn real_time_ratio(&self) -> f64 {
        if self.wall_s > 0.0 {
            self.sim_s / self.wall_s
        } else {
            0.0
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct FrameSample {
    wall_s: f64,
    sim_s: f64,
}

/// Records `(wall, sim)` seconds per unpaused frame and keeps rolling and session statistics.
#[derive(Debug, Clone, Default)]
pub struct PacingMonitor {
    window: VecDeque<FrameSample>,
    window_wall_s: f64,
    stats: PacingStats,
    session: PacingSession,
}

impl PacingMonitor {
    /// Records one unpaused frame of `wall_s` wall-clock seconds that advanced `sim_s`.
    /// Non-positive or non-finite wall times (the first frame after a break) are ignored.
    pub fn record(&mut self, wall_s: f64, sim_s: f64) {
        if !(wall_s.is_finite() && wall_s > 0.0 && sim_s.is_finite()) {
            return;
        }
        let sim_s = sim_s.max(0.0);
        self.window.push_back(FrameSample { wall_s, sim_s });
        self.window_wall_s += wall_s;
        while self.window_wall_s > PACING_WINDOW_S && self.window.len() > 1 {
            if let Some(old) = self.window.pop_front() {
                self.window_wall_s -= old.wall_s;
            }
        }
        self.session.frames = self.session.frames.wrapping_add(1);
        self.session.wall_s += wall_s;
        self.session.sim_s += sim_s;
        self.stats = self.window_stats();
    }

    fn window_stats(&self) -> PacingStats {
        let (wall, sim, max) = self
            .window
            .iter()
            .fold((0.0_f64, 0.0_f64, 0.0_f64), |(w, s, m), f| {
                (w + f.wall_s, s + f.sim_s, m.max(f.wall_s))
            });
        if wall <= 0.0 {
            return PacingStats::default();
        }
        let frames = self.window.len() as f64;
        PacingStats {
            real_time_ratio: sim / wall,
            fps: frames / wall,
            mean_frame_ms: wall / frames * 1_000.0,
            max_frame_ms: max * 1_000.0,
        }
    }

    /// Latest rolling statistics.
    #[must_use]
    pub const fn stats(&self) -> PacingStats {
        self.stats
    }

    /// Session-wide totals.
    #[must_use]
    pub const fn session(&self) -> PacingSession {
        self.session
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rolling_window_tracks_real_time_and_drops_old_frames() {
        let mut monitor = PacingMonitor::default();
        monitor.record(0.0, 0.0);
        assert_eq!(monitor.session().frames, 0, "a break frame is not recorded");
        for _ in 0..240 {
            monitor.record(1.0 / 60.0, 1.0 / 60.0);
        }
        let stats = monitor.stats();
        assert!((stats.real_time_ratio - 1.0).abs() < 1e-9);
        assert!((stats.fps - 60.0).abs() < 1e-6);
        assert!((stats.mean_frame_ms - 16.666).abs() < 0.01);
        // One stalled frame that lost half its time to the catch-up limit.
        monitor.record(1.0, 0.5);
        let stats = monitor.stats();
        assert!(stats.real_time_ratio < 0.8, "{stats:?}");
        assert!((stats.max_frame_ms - 1_000.0).abs() < 1e-6);
        assert_eq!(monitor.session().frames, 241);
        assert!(monitor.session().real_time_ratio() < 1.0);
    }
}

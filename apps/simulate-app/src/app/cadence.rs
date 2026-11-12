//! A steady frame cadence in a browser: on a display refreshing faster than 60 Hz, a page
//! whose frame costs more than one refresh would otherwise alternate between one and two
//! refreshes a frame, a judder the eye reads as stutter. Drawing at most every 60 Hz
//! refresh keeps each frame the same length.

/// The shortest time between two drawn frames, seconds: a 60 Hz refresh, less a margin
/// for the browser's timing jitter, so a 60 Hz display still draws every refresh.
pub const MIN_FRAME_S: f64 = 1.0 / 60.0 - 0.0025;
/// Presentation redraw interval during medium-high page warps (`8×..16×`), ~15 Hz.
pub const WARP_15HZ_FRAME_S: f64 = 1.0 / 15.0 - 0.0025;
/// Presentation redraw interval during maximum page warps (`32×`, `64×`, unthrottled), ~10 Hz.
pub const WARP_10HZ_FRAME_S: f64 = 1.0 / 10.0 - 0.0025;

/// Presentation redraw interval for the browser world view given the page's requested pace
/// (`None` when standalone, `Some(Some(speed))` for bounded warp, `Some(None)` unthrottled)
/// and whether operator map gestures are waiting (`interactive`).
#[must_use]
pub fn presentation_interval_s(page_pace: Option<Option<f64>>, interactive: bool) -> f64 {
    if interactive {
        return MIN_FRAME_S;
    }
    match page_pace {
        None => MIN_FRAME_S,
        Some(Some(speed)) if speed < 8.0 => MIN_FRAME_S,
        Some(Some(speed)) if speed < 32.0 => WARP_15HZ_FRAME_S,
        Some(Some(_) | None) => WARP_10HZ_FRAME_S,
    }
}

/// When the last frame was drawn.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FrameCadence {
    last_s: Option<f64>,
}

impl FrameCadence {
    /// Whether a frame offered at `now_s` seconds draws at 60 Hz; when it does, it is the last.
    pub fn due(&mut self, now_s: f64) -> bool {
        self.due_with_interval(now_s, MIN_FRAME_S)
    }

    /// Whether at least `min_interval_s` elapsed since the last drawn frame.
    pub fn due_with_interval(&mut self, now_s: f64, min_interval_s: f64) -> bool {
        let due = self
            .last_s
            .is_none_or(|last| now_s - last >= min_interval_s);
        if due {
            self.last_s = Some(now_s);
        }
        due
    }
}

#[cfg(test)]
mod tests {
    use super::FrameCadence;

    fn drawn(refresh_hz: f64, jitter_s: f64) -> usize {
        let mut cadence = FrameCadence::default();
        (0..240)
            .map(|n| f64::from(n) / refresh_hz + if n % 2 == 0 { 0.0 } else { jitter_s })
            .filter(|t| cadence.due(*t))
            .count()
    }

    #[test]
    fn a_fast_display_draws_every_other_refresh_and_a_60_hz_one_every_refresh() {
        assert_eq!(drawn(120.0, 0.0), 120, "every other refresh at 120 Hz");
        assert_eq!(drawn(60.0, 0.0), 240, "every refresh at 60 Hz");
        assert_eq!(
            drawn(60.0, -0.0015),
            240,
            "a refresh a little early still draws"
        );
        assert_eq!(drawn(144.0, 0.0), 80, "every third refresh at 144 Hz");
    }

    #[test]
    fn high_warp_presentation_interval_throttles_otw_redraws_except_during_gestures() {
        use super::{MIN_FRAME_S, WARP_10HZ_FRAME_S, WARP_15HZ_FRAME_S, presentation_interval_s};

        assert_eq!(presentation_interval_s(None, false), MIN_FRAME_S);
        assert_eq!(presentation_interval_s(Some(Some(4.0)), false), MIN_FRAME_S);
        assert_eq!(
            presentation_interval_s(Some(Some(16.0)), false),
            WARP_15HZ_FRAME_S
        );
        assert_eq!(
            presentation_interval_s(Some(Some(64.0)), false),
            WARP_10HZ_FRAME_S
        );
        assert_eq!(
            presentation_interval_s(Some(None), false),
            WARP_10HZ_FRAME_S
        );
        assert_eq!(
            presentation_interval_s(Some(Some(64.0)), true),
            MIN_FRAME_S,
            "interactive map gestures restore 60 Hz response immediately"
        );
    }
}

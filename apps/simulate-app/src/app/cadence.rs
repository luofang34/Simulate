//! A steady frame cadence in a browser: on a display refreshing faster than 60 Hz, a page
//! whose frame costs more than one refresh would otherwise alternate between one and two
//! refreshes a frame, a judder the eye reads as stutter. Drawing at most every 60 Hz
//! refresh keeps each frame the same length.

/// The shortest time between two drawn frames, seconds: a 60 Hz refresh, less a margin
/// for the browser's timing jitter, so a 60 Hz display still draws every refresh.
pub const MIN_FRAME_S: f64 = 1.0 / 60.0 - 0.0025;

/// When the last frame was drawn.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FrameCadence {
    last_s: Option<f64>,
}

impl FrameCadence {
    /// Whether a frame offered at `now_s` seconds draws; when it does, it is the last.
    pub fn due(&mut self, now_s: f64) -> bool {
        let due = self.last_s.is_none_or(|last| now_s - last >= MIN_FRAME_S);
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
}

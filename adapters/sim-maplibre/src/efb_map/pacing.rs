//! Monotonic wall-clock pacing for an [`EfbMapSurface`](crate::EfbMapSurface) across
//! simulation, live closed-loop telemetry, and standalone display modes.

use std::time::Duration;

use web_time::Instant;

use super::{EfbDisplayMode, EfbRefreshPolicy};

/// Tracks when an EFB surface last rendered on both monotonic wall time and the host's
/// telemetry or simulation clock so VSync loops never spin at `120 Hz` waiting on network I/O.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EfbPacer {
    policy: EfbRefreshPolicy,
    last_wall_frame: Option<Instant>,
    last_host_time: Option<Duration>,
}

impl EfbPacer {
    /// Creates a pacer using `policy`.
    #[must_use]
    pub const fn new(policy: EfbRefreshPolicy) -> Self {
        Self {
            policy,
            last_wall_frame: None,
            last_host_time: None,
        }
    }

    /// Active refresh policy.
    #[must_use]
    pub const fn policy(&self) -> EfbRefreshPolicy {
        self.policy
    }

    /// Switches between [`EfbRefreshPolicy::CockpitAdaptive`] and
    /// [`EfbRefreshPolicy::StandaloneInteractive`] pacing.
    pub fn set_policy(&mut self, policy: EfbRefreshPolicy) {
        self.policy = policy;
    }

    /// Decides whether the EFB should execute a GPU frame at monotonic `now` and host
    /// telemetry or simulation `host_time`.
    ///
    /// Operator actions (`dirty`), missing initial views (`!has_view`), and host clock rewinds
    /// always draw immediately (`0 ms` lag). Arrived worker tiles (`has_ready_work`) draw
    /// immediately in [`EfbRefreshPolicy::StandaloneInteractive`] and expanded EFB views, and
    /// coalesce to the `15 Hz` wall-clock cadence in the compact cockpit corner inset.
    #[must_use]
    pub fn should_render(
        &self,
        now: Instant,
        host_time: Duration,
        mode: EfbDisplayMode,
        has_view: bool,
        dirty: bool,
        has_ready_work: bool,
    ) -> bool {
        if !has_view || dirty {
            return true;
        }
        if self.last_host_time.is_some_and(|prev| host_time < prev) {
            return true;
        }
        if has_ready_work && self.policy.immediate_on_ready_work(mode) {
            return true;
        }
        let clock_advanced = self.last_host_time != Some(host_time);
        if !clock_advanced && !has_ready_work {
            return false;
        }
        let interval = self.policy.motion_interval(mode);
        self.last_wall_frame
            .is_none_or(|last| now.duration_since(last) >= interval)
    }

    /// Records a completed EFB render at monotonic `now` and `host_time`.
    pub fn record_rendered(&mut self, now: Instant, host_time: Duration) {
        self.last_wall_frame = Some(now);
        self.last_host_time = Some(host_time);
    }
}

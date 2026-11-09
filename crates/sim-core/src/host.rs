use std::sync::Arc;
use std::time::Duration;

use crate::{
    Block, SensorExposureEvent, SessionError, SimInstant, SimSession, TickSpan, VehicleState,
};

mod inline;
#[cfg(test)]
mod tests;
#[cfg(not(target_arch = "wasm32"))]
mod threaded;

pub use inline::InlineHost;
#[cfg(not(target_arch = "wasm32"))]
pub use threaded::ThreadedHost;

/// Identifies one [`SimHost::request_until`] call; the frame published for
/// it carries the same number.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FrameSeq(pub u64);

/// What a host publishes for the render thread after each request: the two
/// exact plant outputs bracketing the requested instant and how far between
/// them it lies, so the renderer interpolates without touching the session.
#[derive(Debug, Clone, PartialEq)]
pub struct DisplayFrame {
    /// The request this frame answers; `FrameSeq(0)` is the state before
    /// any request.
    pub seq: FrameSeq,
    /// Master-clock time the session has been handed.
    pub now: SimInstant,
    /// End of the last completed physics step.
    pub physics_time: SimInstant,
    /// Exact plant output before the last completed step.
    pub previous: VehicleState,
    /// Exact plant output after the last completed step.
    pub current: VehicleState,
    /// Fraction of the next step `now` has reached past `physics_time`.
    pub phase: f64,
    /// Timeline discontinuity counter of the session.
    pub source_epoch: u32,
    /// Episode counter of the session.
    pub episode_id: u64,
}

impl DisplayFrame {
    /// The plant output interpolated to [`Self::now`], for display only.
    #[must_use]
    pub fn display_state(&self) -> VehicleState {
        crate::interpolate_vehicle_state(&self.previous, &self.current, self.phase)
    }
}

/// One request's results as they cross from the simulation to the frame
/// thread: the frame, plus every exposure the request emitted, which is
/// kept even when a newer frame supersedes this one.
#[derive(Debug)]
struct Published {
    frame: Arc<DisplayFrame>,
    exposures: Vec<SensorExposureEvent>,
}

impl Published {
    fn capture<B>(session: &mut SimSession<B>, seq: FrameSeq) -> Self
    where
        B: Block<Output = VehicleState>,
        B::Input: Clone,
    {
        let frame = DisplayFrame {
            seq,
            now: session.now(),
            physics_time: session.physics_time(),
            previous: session.previous_truth_state(),
            current: session.truth_state(),
            phase: session.display_phase(),
            source_epoch: session.source_epoch(),
            episode_id: session.episode_id(),
        };
        Self {
            frame: Arc::new(frame),
            exposures: session.take_exposures(),
        }
    }
}

/// Why a host refused a command.
#[derive(Debug, thiserror::Error)]
pub enum HostError {
    /// The session failed to step; the host steps no further. Boxed so
    /// every host result stays small.
    #[error("simulation session failed")]
    Session(#[source] Box<SessionError>),
    /// The host was shut down.
    #[error("simulation host is shut down")]
    ShutDown,
    /// The simulation thread could not be started.
    #[error("could not spawn simulation thread {thread:?}")]
    Spawn {
        /// Name of the thread.
        thread: String,
        /// The operating system's refusal.
        #[source]
        source: std::io::Error,
    },
    /// The simulation thread ended without being asked to.
    #[error("simulation thread {thread:?} ended unexpectedly")]
    ThreadLost {
        /// Name of the thread.
        thread: String,
    },
    /// A waited-for frame was not published in time.
    #[error("frame {seq:?} not published within {waited:?}")]
    Timeout {
        /// The awaited request.
        seq: FrameSeq,
        /// How long the caller waited.
        waited: Duration,
    },
    /// A wait named a request that was never issued.
    #[error("frame {seq:?} was never requested (last request {last:?})")]
    NotRequested {
        /// The awaited request.
        seq: FrameSeq,
        /// The newest request issued.
        last: FrameSeq,
    },
}

impl From<SessionError> for HostError {
    fn from(source: SessionError) -> Self {
        Self::Session(Box::new(source))
    }
}

/// Which simulation threads ended during [`SimHost::shutdown`].
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ShutdownReport {
    /// Threads that exited and were joined.
    pub joined: Vec<String>,
    /// Threads still running when the timeout elapsed; they were detached,
    /// since a native thread cannot be aborted, and exit at their next
    /// command boundary.
    pub stuck: Vec<String>,
}

/// Runs a deterministic [`SimSession`] for a frame loop, on the caller's
/// thread or another. The simulation advances only to instants the caller
/// requests, so where it runs and how wall-clock time jitters never change
/// its truth. Requests are never refused for running ahead: a frame loop
/// that must stay real-time bounds its own lag, skipping a request while
/// [`Self::requested`] leads [`Self::latest`]'s `now` by too much.
pub trait SimHost {
    /// The plant block's input.
    type Input;

    /// Queues `input` to take effect at `at`, clamped forward to the last
    /// queued instant as [`SimSession::submit_at`] does.
    fn submit_at(&mut self, at: SimInstant, input: Self::Input) -> Result<(), HostError>;

    /// Asks the simulation to advance to `target` (a no-op advance if it is
    /// not after the last request) and publish a frame for it.
    fn request_until(&mut self, target: SimInstant) -> Result<FrameSeq, HostError>;

    /// The newest instant requested so far.
    fn requested(&self) -> SimInstant;

    /// Asks the simulation to advance `elapsed` past [`Self::requested`].
    fn request_by(&mut self, elapsed: TickSpan) -> Result<FrameSeq, HostError> {
        let target = self.requested().advanced(elapsed);
        self.request_until(target)
    }

    /// The newest frame published, without waiting for the simulation.
    fn latest(&mut self) -> Result<Arc<DisplayFrame>, HostError>;

    /// Waits up to `timeout` for the frame answering `seq` (or a newer one)
    /// and returns it.
    fn frame_blocking(
        &mut self,
        seq: FrameSeq,
        timeout: Duration,
    ) -> Result<Arc<DisplayFrame>, HostError>;

    /// Hands over the sensor exposures published so far, oldest first,
    /// including those of frames [`Self::latest`] skipped.
    fn take_exposures(&mut self) -> Vec<SensorExposureEvent>;

    /// Stops the simulation, waiting up to `timeout` for its thread; every
    /// later command fails with [`HostError::ShutDown`].
    fn shutdown(&mut self, timeout: Duration) -> Result<ShutdownReport, HostError>;
}

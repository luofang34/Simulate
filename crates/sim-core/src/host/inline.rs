use std::sync::Arc;
use std::time::Duration;

use crate::{Block, SensorExposureEvent, SessionError, SimInstant, SimSession, VehicleState};

use super::{DisplayFrame, FrameSeq, HostError, Published, ShutdownReport, SimHost};

/// Steps its session on the caller's thread inside
/// [`SimHost::request_until`]: the host for browsers, where the page has a
/// single thread, and for tests.
pub struct InlineHost<B>
where
    B: Block<Output = VehicleState>,
    B::Input: Clone,
{
    session: SimSession<B>,
    latest: Arc<DisplayFrame>,
    exposures: Vec<SensorExposureEvent>,
    last_seq: FrameSeq,
    requested: SimInstant,
    fault: Option<SessionError>,
    closed: bool,
}

impl<B> std::fmt::Debug for InlineHost<B>
where
    B: Block<Output = VehicleState>,
    B::Input: Clone,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("InlineHost")
            .field("last_seq", &self.last_seq)
            .field("requested", &self.requested)
            .field("closed", &self.closed)
            .finish_non_exhaustive()
    }
}

impl<B> InlineHost<B>
where
    B: Block<Output = VehicleState>,
    B::Input: Clone,
{
    /// Hosts `session`, publishing its current state as frame 0.
    pub fn new(mut session: SimSession<B>) -> Self {
        let first = Published::capture(&mut session, FrameSeq(0));
        Self {
            requested: session.now(),
            session,
            latest: first.frame,
            exposures: first.exposures,
            last_seq: FrameSeq(0),
            fault: None,
            closed: false,
        }
    }

    /// The hosted session, for reading block state beyond the frame.
    #[must_use]
    pub fn session(&self) -> &SimSession<B> {
        &self.session
    }

    fn ensure_open(&self) -> Result<(), HostError> {
        if self.closed {
            return Err(HostError::ShutDown);
        }
        match &self.fault {
            Some(fault) => Err(HostError::from(fault.clone())),
            None => Ok(()),
        }
    }
}

impl<B> SimHost for InlineHost<B>
where
    B: Block<Output = VehicleState>,
    B::Input: Clone,
{
    type Input = B::Input;

    fn submit_at(&mut self, at: SimInstant, input: B::Input) -> Result<(), HostError> {
        self.ensure_open()?;
        self.session.submit_at(at, input);
        Ok(())
    }

    fn request_until(&mut self, target: SimInstant) -> Result<FrameSeq, HostError> {
        self.ensure_open()?;
        let seq = FrameSeq(self.last_seq.0.wrapping_add(1));
        self.last_seq = seq;
        self.requested = self.requested.max(target);
        if let Err(fault) = self.session.advance_to(target) {
            self.fault = Some(fault.clone());
            return Err(HostError::from(fault));
        }
        let published = Published::capture(&mut self.session, seq);
        self.latest = published.frame;
        self.exposures.extend(published.exposures);
        Ok(seq)
    }

    fn requested(&self) -> SimInstant {
        self.requested
    }

    fn latest(&mut self) -> Result<Arc<DisplayFrame>, HostError> {
        self.ensure_open()?;
        Ok(Arc::clone(&self.latest))
    }

    fn frame_blocking(
        &mut self,
        seq: FrameSeq,
        _timeout: Duration,
    ) -> Result<Arc<DisplayFrame>, HostError> {
        self.ensure_open()?;
        if seq > self.last_seq {
            return Err(HostError::NotRequested {
                seq,
                last: self.last_seq,
            });
        }
        Ok(Arc::clone(&self.latest))
    }

    fn take_exposures(&mut self) -> Vec<SensorExposureEvent> {
        std::mem::take(&mut self.exposures)
    }

    fn shutdown(&mut self, _timeout: Duration) -> Result<ShutdownReport, HostError> {
        if self.closed {
            return Err(HostError::ShutDown);
        }
        self.closed = true;
        Ok(ShutdownReport::default())
    }
}

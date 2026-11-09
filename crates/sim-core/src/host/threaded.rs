use std::sync::Arc;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender, TryRecvError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use tracing::{debug, error};

use crate::{Block, SensorExposureEvent, SessionError, SimInstant, SimSession, VehicleState};

use super::{DisplayFrame, FrameSeq, HostError, Published, ShutdownReport, SimHost};

enum Command<I> {
    Submit(SimInstant, I),
    RequestUntil(SimInstant, FrameSeq),
    Shutdown,
}

type Outcome = Result<Published, SessionError>;

/// Steps its session on a named thread of its own. Commands reach it in
/// order over one channel and every request's frame comes back over
/// another, so the frame thread never waits on physics unless it asks to
/// ([`SimHost::frame_blocking`]), and the session never sees wall time.
///
/// Generic over the plant input alone: the frame thread never touches the
/// block, which lives only on the simulation thread.
pub struct ThreadedHost<I> {
    thread: String,
    commands: Sender<Command<I>>,
    published: Receiver<Outcome>,
    exited: Receiver<()>,
    handle: Option<JoinHandle<()>>,
    latest: Arc<DisplayFrame>,
    exposures: Vec<SensorExposureEvent>,
    last_seq: FrameSeq,
    requested: SimInstant,
    fault: Option<SessionError>,
    lost: bool,
    closed: bool,
}

impl<I> std::fmt::Debug for ThreadedHost<I> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ThreadedHost")
            .field("thread", &self.thread)
            .field("last_seq", &self.last_seq)
            .field("requested", &self.requested)
            .field("closed", &self.closed)
            .finish_non_exhaustive()
    }
}

impl<I: Send + 'static> ThreadedHost<I> {
    /// Moves `session` onto a new thread called `thread`, publishing its
    /// current state as frame 0.
    pub fn spawn<B>(mut session: SimSession<B>, thread: &str) -> Result<Self, HostError>
    where
        B: Block<Output = VehicleState, Input = I> + Send + 'static,
        I: Clone,
    {
        let first = Published::capture(&mut session, FrameSeq(0));
        let requested = session.now();
        let (commands, command_rx) = mpsc::channel();
        let (published_tx, published) = mpsc::channel();
        let (exit_guard, exited) = mpsc::channel::<()>();
        let handle = std::thread::Builder::new()
            .name(thread.to_owned())
            .spawn(move || {
                serve(session, &command_rx, &published_tx);
                drop(exit_guard);
            })
            .map_err(|source| HostError::Spawn {
                thread: thread.to_owned(),
                source,
            })?;
        Ok(Self {
            thread: thread.to_owned(),
            commands,
            published,
            exited,
            handle: Some(handle),
            latest: first.frame,
            exposures: first.exposures,
            last_seq: FrameSeq(0),
            requested,
            fault: None,
            lost: false,
            closed: false,
        })
    }

    /// Name of the simulation thread.
    #[must_use]
    pub fn thread_name(&self) -> &str {
        &self.thread
    }

    fn absorb(&mut self, outcome: Outcome) {
        match outcome {
            Ok(published) => {
                self.latest = published.frame;
                self.exposures.extend(published.exposures);
            }
            Err(fault) => self.fault = Some(fault),
        }
    }

    /// Takes every outcome already published without waiting.
    fn drain(&mut self) {
        loop {
            match self.published.try_recv() {
                Ok(outcome) => self.absorb(outcome),
                Err(TryRecvError::Empty) => return,
                Err(TryRecvError::Disconnected) => {
                    self.lost = true;
                    return;
                }
            }
        }
    }

    fn health(&self) -> Result<(), HostError> {
        if self.closed {
            return Err(HostError::ShutDown);
        }
        if let Some(fault) = &self.fault {
            return Err(HostError::from(fault.clone()));
        }
        if self.lost {
            return Err(HostError::ThreadLost {
                thread: self.thread.clone(),
            });
        }
        Ok(())
    }

    fn send(&mut self, command: Command<I>) -> Result<(), HostError> {
        self.drain();
        self.health()?;
        if self.commands.send(command).is_err() {
            self.lost = true;
            self.drain();
            return self.health();
        }
        Ok(())
    }

    fn join(&mut self, report: &mut ShutdownReport) -> Result<(), HostError> {
        let Some(handle) = self.handle.take() else {
            return Ok(());
        };
        if handle.join().is_err() {
            error!(thread = %self.thread, "simulation thread panicked");
            return Err(HostError::ThreadLost {
                thread: self.thread.clone(),
            });
        }
        debug!(thread = %self.thread, "simulation thread joined");
        report.joined.push(self.thread.clone());
        Ok(())
    }
}

impl<I: Clone + Send + 'static> SimHost for ThreadedHost<I> {
    type Input = I;

    fn submit_at(&mut self, at: SimInstant, input: I) -> Result<(), HostError> {
        self.send(Command::Submit(at, input))
    }

    fn request_until(&mut self, target: SimInstant) -> Result<FrameSeq, HostError> {
        let seq = FrameSeq(self.last_seq.0.wrapping_add(1));
        self.send(Command::RequestUntil(target, seq))?;
        self.last_seq = seq;
        self.requested = self.requested.max(target);
        Ok(seq)
    }

    fn requested(&self) -> SimInstant {
        self.requested
    }

    fn latest(&mut self) -> Result<Arc<DisplayFrame>, HostError> {
        self.drain();
        self.health()?;
        Ok(Arc::clone(&self.latest))
    }

    fn frame_blocking(
        &mut self,
        seq: FrameSeq,
        timeout: Duration,
    ) -> Result<Arc<DisplayFrame>, HostError> {
        if seq > self.last_seq {
            return Err(HostError::NotRequested {
                seq,
                last: self.last_seq,
            });
        }
        // A timeout too long to represent as an instant waits indefinitely.
        let deadline = Instant::now().checked_add(timeout);
        while self.latest.seq < seq {
            self.health()?;
            let received = match deadline {
                Some(deadline) => self
                    .published
                    .recv_timeout(deadline.saturating_duration_since(Instant::now())),
                None => self
                    .published
                    .recv()
                    .map_err(|_| RecvTimeoutError::Disconnected),
            };
            match received {
                Ok(outcome) => self.absorb(outcome),
                Err(RecvTimeoutError::Timeout) => {
                    return Err(HostError::Timeout {
                        seq,
                        waited: timeout,
                    });
                }
                Err(RecvTimeoutError::Disconnected) => self.lost = true,
            }
        }
        Ok(Arc::clone(&self.latest))
    }

    fn take_exposures(&mut self) -> Vec<SensorExposureEvent> {
        self.drain();
        std::mem::take(&mut self.exposures)
    }

    fn shutdown(&mut self, timeout: Duration) -> Result<ShutdownReport, HostError> {
        if self.closed {
            return Err(HostError::ShutDown);
        }
        self.closed = true;
        // The thread may already have stopped after a fault; that is not an
        // error here.
        self.commands.send(Command::Shutdown).ok();
        let mut report = ShutdownReport::default();
        match self.exited.recv_timeout(timeout) {
            Ok(()) | Err(RecvTimeoutError::Disconnected) => self.join(&mut report)?,
            Err(RecvTimeoutError::Timeout) => {
                error!(
                    thread = %self.thread,
                    ?timeout,
                    "simulation thread did not exit; detaching it"
                );
                self.handle = None;
                report.stuck.push(self.thread.clone());
            }
        }
        Ok(report)
    }
}

impl<I> Drop for ThreadedHost<I> {
    fn drop(&mut self) {
        if !self.closed {
            // Detached: the thread exits at its next command boundary.
            self.commands.send(Command::Shutdown).ok();
        }
    }
}

fn serve<B>(
    mut session: SimSession<B>,
    commands: &Receiver<Command<B::Input>>,
    published: &Sender<Outcome>,
) where
    B: Block<Output = VehicleState>,
    B::Input: Clone,
{
    while let Ok(command) = commands.recv() {
        match command {
            Command::Submit(at, input) => {
                session.submit_at(at, input);
            }
            Command::RequestUntil(target, seq) => {
                let outcome = session
                    .advance_to(target)
                    .map(|_| Published::capture(&mut session, seq));
                let failed = outcome.is_err();
                if published.send(outcome).is_err() || failed {
                    return;
                }
            }
            Command::Shutdown => return,
        }
    }
}

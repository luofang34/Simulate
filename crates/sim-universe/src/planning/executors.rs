//! Where plan jobs run. Each executor runs the same job to the same output; they
//! differ only in which thread does the work and when.

use std::collections::BTreeMap;

use super::{PlanId, PlanJob, PlanOutput};
use crate::UniverseError;

/// Runs plan jobs for a universe.
pub trait PlanExecutor: Send {
    /// Starts `job` as plan `id`.
    fn submit(&mut self, id: PlanId, job: Box<dyn PlanJob>);
    /// Whether plan `id`'s output is already computed.
    fn is_ready(&self, id: PlanId) -> bool;
    /// Plan `id`'s output, computing what remains of it on the calling thread or
    /// waiting for it; the plan is forgotten afterwards.
    ///
    /// # Errors
    /// The job's own error, or [`UniverseError::UnknownPlan`] when no job `id` was
    /// submitted.
    fn finish(&mut self, id: PlanId) -> Result<PlanOutput, UniverseError>;
    /// Gives queued jobs up to `slices` slices of work on the calling thread; a
    /// no-op for executors that run jobs elsewhere.
    fn pump(&mut self, slices: u32);
    /// Abandons every job.
    fn cancel_all(&mut self);
}

fn run_to_end(job: &mut dyn PlanJob) -> Result<PlanOutput, UniverseError> {
    loop {
        if let Some(output) = job.run_slice()? {
            return Ok(output);
        }
    }
}

/// Runs each job to completion the moment it is submitted.
#[derive(Default)]
pub struct SynchronousPlanExecutor {
    done: BTreeMap<PlanId, Result<PlanOutput, UniverseError>>,
}

impl PlanExecutor for SynchronousPlanExecutor {
    fn submit(&mut self, id: PlanId, mut job: Box<dyn PlanJob>) {
        self.done.insert(id, run_to_end(job.as_mut()));
    }

    fn is_ready(&self, id: PlanId) -> bool {
        self.done.contains_key(&id)
    }

    fn finish(&mut self, id: PlanId) -> Result<PlanOutput, UniverseError> {
        self.done
            .remove(&id)
            .unwrap_or(Err(UniverseError::UnknownPlan(id)))
    }

    fn pump(&mut self, _slices: u32) {}

    fn cancel_all(&mut self) {
        self.done.clear();
    }
}

enum SlicedJob {
    Running(Box<dyn PlanJob>),
    Done(Result<PlanOutput, UniverseError>),
}

/// Runs jobs a slice at a time when pumped, in plan order, on the calling thread;
/// for hosts without threads.
#[derive(Default)]
pub struct SlicedPlanExecutor {
    jobs: BTreeMap<PlanId, SlicedJob>,
}

impl PlanExecutor for SlicedPlanExecutor {
    fn submit(&mut self, id: PlanId, job: Box<dyn PlanJob>) {
        self.jobs.insert(id, SlicedJob::Running(job));
    }

    fn is_ready(&self, id: PlanId) -> bool {
        matches!(self.jobs.get(&id), Some(SlicedJob::Done(_)))
    }

    fn finish(&mut self, id: PlanId) -> Result<PlanOutput, UniverseError> {
        match self.jobs.remove(&id) {
            Some(SlicedJob::Running(mut job)) => run_to_end(job.as_mut()),
            Some(SlicedJob::Done(result)) => result,
            None => Err(UniverseError::UnknownPlan(id)),
        }
    }

    fn pump(&mut self, slices: u32) {
        let mut left = slices;
        for entry in self.jobs.values_mut() {
            while left > 0 {
                let SlicedJob::Running(job) = entry else {
                    break;
                };
                left = left.saturating_sub(1);
                match job.run_slice() {
                    Ok(None) => {}
                    Ok(Some(output)) => *entry = SlicedJob::Done(Ok(output)),
                    Err(error) => *entry = SlicedJob::Done(Err(error)),
                }
            }
            if left == 0 {
                break;
            }
        }
    }

    fn cancel_all(&mut self) {
        self.jobs.clear();
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub use threaded::ThreadPlanExecutor;

#[cfg(not(target_arch = "wasm32"))]
mod threaded {
    use std::collections::BTreeMap;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::thread::JoinHandle;

    use super::super::{PlanId, PlanJob, PlanOutput};
    use super::PlanExecutor;
    use crate::UniverseError;

    enum Threaded {
        Running {
            handle: JoinHandle<Result<PlanOutput, UniverseError>>,
            cancel: Arc<AtomicBool>,
        },
        NotStarted(UniverseError),
    }

    /// Runs each job on a thread of its own.
    #[derive(Default)]
    pub struct ThreadPlanExecutor {
        jobs: BTreeMap<PlanId, Threaded>,
        stack_size: Option<usize>,
    }

    impl ThreadPlanExecutor {
        /// This executor starting each job's thread with `bytes` of stack; a job
        /// whose thread cannot be started finishes with
        /// [`UniverseError::PlanThreadSpawn`].
        #[must_use]
        pub const fn with_stack_size(mut self, bytes: usize) -> Self {
            self.stack_size = Some(bytes);
            self
        }
    }

    impl PlanExecutor for ThreadPlanExecutor {
        fn submit(&mut self, id: PlanId, mut job: Box<dyn PlanJob>) {
            let cancel = Arc::new(AtomicBool::new(false));
            let stop = Arc::clone(&cancel);
            let thread = format!("plan-{}", id.0);
            let mut builder = std::thread::Builder::new().name(thread.clone());
            if let Some(bytes) = self.stack_size {
                builder = builder.stack_size(bytes);
            }
            let spawned = builder.spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    if let Some(output) = job.run_slice()? {
                        return Ok(output);
                    }
                }
                Err(UniverseError::UnknownPlan(id))
            });
            let entry = match spawned {
                Ok(handle) => Threaded::Running { handle, cancel },
                Err(source) => {
                    Threaded::NotStarted(UniverseError::PlanThreadSpawn { id, thread, source })
                }
            };
            self.jobs.insert(id, entry);
        }

        fn is_ready(&self, id: PlanId) -> bool {
            match self.jobs.get(&id) {
                Some(Threaded::Running { handle, .. }) => handle.is_finished(),
                Some(Threaded::NotStarted(_)) => true,
                None => false,
            }
        }

        fn finish(&mut self, id: PlanId) -> Result<PlanOutput, UniverseError> {
            match self.jobs.remove(&id) {
                Some(Threaded::Running { handle, .. }) => handle
                    .join()
                    .unwrap_or(Err(UniverseError::PlanThreadFailed(id))),
                Some(Threaded::NotStarted(error)) => Err(error),
                None => Err(UniverseError::UnknownPlan(id)),
            }
        }

        fn pump(&mut self, _slices: u32) {}

        fn cancel_all(&mut self) {
            for job in std::mem::take(&mut self.jobs).into_values() {
                if let Threaded::Running { cancel, .. } = job {
                    cancel.store(true, Ordering::Relaxed);
                }
            }
        }
    }
}

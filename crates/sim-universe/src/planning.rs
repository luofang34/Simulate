//! Planned commands: work that decides what a vessel will be commanded to do,
//! computed away from the instant it is asked for and applied at a fixed later one.
//!
//! A [`PlanSchedule`] is asked at its due instants for a [`PlanRequest`]: a
//! self-contained snapshot of whatever the plan needs, with a deadline. A
//! [`PlanJob`] started from the request runs in bounded slices on whatever
//! [`PlanExecutor`] the host supplies (in place, a slice per frame, or on another
//! thread) and yields commands with the instants they act at. The universe stamps
//! those commands when its clock reaches the deadline, never earlier and never
//! later, so what is commanded and when is a function of the snapshot alone,
//! however fast the host ran and wherever the job ran.

mod executors;

use std::collections::BTreeMap;
use std::fmt;
use std::sync::Arc;

use sim_core::SimInstant;
use sim_orbital::{PlayerCommandPayload, VesselId};

use crate::{L2KeepingBurn, Universe, UniverseError};

#[cfg(not(target_arch = "wasm32"))]
pub use executors::ThreadPlanExecutor;
pub use executors::{PlanExecutor, SlicedPlanExecutor, SynchronousPlanExecutor};

/// Identifies one plan for the life of a universe.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PlanId(pub u64);

/// One command a plan decided on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlannedCommand {
    /// The vessel commanded.
    pub vessel: VesselId,
    /// When the command acts; no earlier than the plan's deadline.
    pub at: SimInstant,
    /// What it commands.
    pub payload: PlayerCommandPayload,
}

/// What a plan says, for whoever reports it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlanReport {
    /// A station-keeping burn about $L_2$.
    L2Holding(L2KeepingBurn),
}

/// A finished plan.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PlanOutput {
    /// Commands to stamp at the deadline, in order.
    pub commands: Vec<PlannedCommand>,
    /// What the commands amount to, if the plan says.
    pub report: Option<PlanReport>,
}

/// Work started from a [`PlanRequest`], run a bounded slice at a time.
pub trait PlanJob: Send {
    /// Runs one slice; the output once the plan is finished. Every slice must be
    /// short (a few milliseconds in an optimised build) so a host can run one per
    /// frame, and the output must not depend on how slices are spread over time.
    ///
    /// # Errors
    /// Whatever stopped the plan; the universe records it and stamps nothing.
    fn run_slice(&mut self) -> Result<Option<PlanOutput>, UniverseError>;
}

/// Everything a plan needs, taken at the instant it was asked for.
pub trait PlanRequest: Send + Sync + fmt::Debug {
    /// The instant the plan's commands are stamped at.
    fn deadline(&self) -> SimInstant;
    /// A fresh job computing the plan; every job started from one request
    /// computes the same output.
    fn start(&self) -> Box<dyn PlanJob>;
    /// The vessel the plan commands, if it is about one, for diagnostics.
    fn vessel(&self) -> Option<VesselId> {
        None
    }
}

/// Something that asks for plans at instants of its own.
pub trait PlanSchedule: Send + fmt::Debug {
    /// The next instant it wants to plan at, if any.
    fn next_due(&self) -> Option<SimInstant>;
    /// Plans at the universe's current instant, which is its due instant, and moves
    /// its schedule on past that instant; a schedule that does not is detached with
    /// [`UniverseError::PlanScheduleStalled`]. `None` when there is nothing to plan
    /// this time.
    ///
    /// # Errors
    /// The plan could not be set up; the schedule has still moved on.
    fn plan(&mut self, universe: &Universe) -> Result<Option<Arc<dyn PlanRequest>>, UniverseError>;
    /// A copy, for checkpoints.
    fn clone_box(&self) -> Box<dyn PlanSchedule>;
}

/// What became of one plan.
#[derive(Debug)]
pub struct PlanRecord {
    /// The plan, if it got as far as a request.
    pub id: Option<PlanId>,
    /// When it was asked for.
    pub requested_at: SimInstant,
    /// When its commands were stamped (or would have been).
    pub deadline: SimInstant,
    /// Its output, or why there is none.
    pub outcome: Result<PlanOutput, UniverseError>,
}

/// The earliest plan the universe is waiting on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlanHold {
    /// The plan.
    pub id: PlanId,
    /// The instant its commands are stamped at.
    pub deadline: SimInstant,
    /// Whether its output is already computed, so reaching the deadline costs
    /// nothing.
    pub ready: bool,
}

#[derive(Debug, Clone)]
pub(crate) struct PendingPlan {
    pub(crate) requested_at: SimInstant,
    pub(crate) request: Arc<dyn PlanRequest>,
}

/// The plans of a universe: who asks for them, which are pending, and where they run.
pub(crate) struct Planner {
    pub(crate) schedules: Vec<Box<dyn PlanSchedule>>,
    pub(crate) pending: BTreeMap<PlanId, PendingPlan>,
    pub(crate) next_id: u64,
    pub(crate) executor: Box<dyn PlanExecutor>,
    pub(crate) records: Vec<PlanRecord>,
    pub(crate) finished_inline: u64,
}

impl Planner {
    pub(crate) fn new() -> Self {
        Self {
            schedules: Vec::new(),
            pending: BTreeMap::new(),
            next_id: 0,
            executor: Box::new(SynchronousPlanExecutor::default()),
            records: Vec::new(),
            finished_inline: 0,
        }
    }

    /// The pending plan with the earliest deadline (ties by id).
    pub(crate) fn earliest_pending(&self) -> Option<(PlanId, SimInstant)> {
        self.pending
            .iter()
            .map(|(&id, plan)| (id, plan.request.deadline()))
            .min_by_key(|&(id, deadline)| (deadline, id))
    }

    /// The earliest instant after `now` a schedule is due at.
    pub(crate) fn next_due_after(&self, now: SimInstant) -> Option<SimInstant> {
        self.schedules
            .iter()
            .filter_map(|schedule| schedule.next_due())
            .filter(|&due| due > now)
            .min()
    }

    /// The schedules due at or before `now`, in attachment order.
    pub(crate) fn due_by(&self, now: SimInstant) -> Vec<usize> {
        self.schedules
            .iter()
            .enumerate()
            .filter(|(_, schedule)| schedule.next_due().is_some_and(|due| due <= now))
            .map(|(index, _)| index)
            .collect()
    }

    pub(crate) fn checkpoint(&self) -> PlannerCheckpoint {
        PlannerCheckpoint {
            schedules: self.schedules.iter().map(|s| s.clone_box()).collect(),
            pending: self.pending.clone(),
            next_id: self.next_id,
        }
    }

    /// Puts the plans back as `saved` had them and restarts every pending job from
    /// its request, abandoning whatever was running.
    pub(crate) fn restore(&mut self, saved: &PlannerCheckpoint) {
        self.executor.cancel_all();
        self.schedules = saved.schedules.iter().map(|s| s.clone_box()).collect();
        self.pending = saved.pending.clone();
        self.next_id = saved.next_id;
        for (&id, plan) in &self.pending {
            self.executor.submit(id, plan.request.start());
        }
    }
}

/// The plans of a universe as a checkpoint holds them.
pub(crate) struct PlannerCheckpoint {
    schedules: Vec<Box<dyn PlanSchedule>>,
    pending: BTreeMap<PlanId, PendingPlan>,
    next_id: u64,
}

impl Clone for PlannerCheckpoint {
    fn clone(&self) -> Self {
        Self {
            schedules: self.schedules.iter().map(|s| s.clone_box()).collect(),
            pending: self.pending.clone(),
            next_id: self.next_id,
        }
    }
}

impl fmt::Debug for PlannerCheckpoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PlannerCheckpoint")
            .field("schedules", &self.schedules)
            .field("pending", &self.pending)
            .field("next_id", &self.next_id)
            .finish()
    }
}

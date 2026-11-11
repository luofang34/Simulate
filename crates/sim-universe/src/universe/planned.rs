//! Advancing a [`Universe`] through its plans' due instants and deadlines.

use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan};
use tracing::warn;

use super::Universe;
use crate::UniverseError;
use crate::planning::{
    PendingPlan, PlanExecutor, PlanHold, PlanId, PlanOutput, PlanRecord, PlanSchedule,
};

impl Universe {
    /// Runs plan jobs on `executor` instead of to completion the moment they are
    /// asked for. Plans already pending stay where they are; set the executor
    /// before attaching schedules.
    #[must_use]
    pub fn with_plan_executor(mut self, executor: Box<dyn PlanExecutor>) -> Self {
        self.planner.executor = executor;
        self
    }

    /// Lets `schedule` ask for plans as the universe advances; one due now plans at
    /// the next advance.
    pub fn attach_plan_schedule(&mut self, schedule: Box<dyn PlanSchedule>) {
        self.planner.schedules.push(schedule);
    }

    /// Gives pending plan jobs up to `slices` slices of work on this thread, for
    /// executors that run jobs where they are pumped.
    pub fn pump_plans(&mut self, slices: u32) {
        self.planner.executor.pump(slices);
    }

    /// The pending plan with the earliest deadline, if any.
    #[must_use]
    pub fn plan_hold(&self) -> Option<PlanHold> {
        let (id, deadline) = self.planner.earliest_pending()?;
        Some(PlanHold {
            id,
            deadline,
            ready: self.planner.executor.is_ready(id),
        })
    }

    /// The latest instant the universe can be advanced to without computing a
    /// plan on this thread: one tick before the earliest deadline of a plan whose
    /// output is not ready yet, or `None` when nothing is waited on.
    #[must_use]
    pub fn plan_free_until(&self) -> Option<SimInstant> {
        self.planner
            .pending
            .iter()
            .filter(|&(&id, _)| !self.planner.executor.is_ready(id))
            .map(|(_, plan)| plan.request.deadline())
            .min()
            .map(|deadline| {
                let before = deadline.ticks_u128().saturating_sub(1);
                SimInstant::START.advanced(TickSpan::from_ticks_u128(before))
            })
    }

    /// What became of the plans finished since this was last called, in the order
    /// their commands were stamped.
    pub fn take_plan_records(&mut self) -> Vec<PlanRecord> {
        std::mem::take(&mut self.planner.records)
    }

    /// Advances every vessel to `until`: local tiers by whole periods, the
    /// orbital tier to exactly `until`. A tier policy acts at each of its
    /// check instants on the way; plans are asked for at their due instants and
    /// their commands stamped at their deadlines, finishing a plan on this thread
    /// if its executor has not.
    pub fn advance_to(&mut self, until: SimInstant) -> Result<(), UniverseError> {
        if until < self.now {
            return Err(UniverseError::Backwards);
        }
        loop {
            self.serve_plans()?;
            let stop = [
                self.planner.earliest_pending().map(|(_, at)| at),
                self.planner.next_due_after(self.now),
            ]
            .into_iter()
            .flatten()
            .filter(|&at| at > self.now)
            .min()
            .filter(|&at| at <= until);
            let Some(stop) = stop else {
                break;
            };
            self.advance_unplanned_to(stop)?;
        }
        self.advance_unplanned_to(until)
    }

    /// Stamps every plan whose deadline has come and asks every schedule that is
    /// due, each in attachment order, deadlines first, until nothing more is due at
    /// this instant.
    fn serve_plans(&mut self) -> Result<(), UniverseError> {
        loop {
            if let Some((id, deadline)) = self.planner.earliest_pending()
                && deadline <= self.now
            {
                self.stamp_plan(id)?;
                continue;
            }
            let due = self.planner.due_by(self.now);
            if due.is_empty() {
                return Ok(());
            }
            let mut stalled = Vec::new();
            for index in due {
                if self.ask_schedule(index) {
                    stalled.push(index);
                }
            }
            for &index in stalled.iter().rev() {
                if index < self.planner.schedules.len() {
                    let schedule = self.planner.schedules.remove(index);
                    self.planner.records.push(PlanRecord {
                        id: None,
                        requested_at: self.now,
                        deadline: self.now,
                        outcome: Err(UniverseError::PlanScheduleStalled {
                            schedule: format!("{schedule:?}"),
                            at: self.now,
                        }),
                    });
                }
            }
        }
    }

    fn stamp_plan(&mut self, id: PlanId) -> Result<(), UniverseError> {
        let Some(plan) = self.planner.pending.remove(&id) else {
            return Ok(());
        };
        let deadline = plan.request.deadline();
        if !self.planner.executor.is_ready(id) {
            self.planner.finished_inline = self.planner.finished_inline.wrapping_add(1);
            warn!(
                plan = id.0,
                vessel = ?plan.request.vessel(),
                requested_at_s = plan.requested_at.ticks_u128() as f64 / TICKS_PER_SECOND as f64,
                deadline_s = deadline.ticks_u128() as f64 / TICKS_PER_SECOND as f64,
                "a plan reached its deadline unfinished; finishing it on the stepping thread"
            );
        }
        let outcome = self
            .planner
            .executor
            .finish(id)
            .and_then(|output| self.stamp_output(id, deadline, output));
        self.planner.records.push(PlanRecord {
            id: Some(id),
            requested_at: plan.requested_at,
            deadline,
            outcome,
        });
        Ok(())
    }

    fn stamp_output(
        &mut self,
        id: PlanId,
        deadline: SimInstant,
        output: PlanOutput,
    ) -> Result<PlanOutput, UniverseError> {
        if output.commands.iter().any(|command| command.at < deadline) {
            return Err(UniverseError::PlanCommandTooEarly(id));
        }
        for command in &output.commands {
            self.command(command.vessel, command.at, command.payload)?;
        }
        Ok(output)
    }

    /// Asks schedule `index` for a plan; `true` when it did not move past now.
    fn ask_schedule(&mut self, index: usize) -> bool {
        let mut schedules = std::mem::take(&mut self.planner.schedules);
        let asked = schedules.get_mut(index).map(|schedule| schedule.plan(self));
        let stalled = schedules
            .get(index)
            .and_then(|schedule| schedule.next_due())
            .is_some_and(|due| due <= self.now);
        self.planner.schedules = schedules;
        let now = self.now;
        match asked {
            Some(Ok(Some(request))) => {
                let id = PlanId(self.planner.next_id);
                self.planner.next_id = self.planner.next_id.wrapping_add(1);
                self.planner.executor.submit(id, request.start());
                self.planner.pending.insert(
                    id,
                    PendingPlan {
                        requested_at: now,
                        request,
                    },
                );
            }
            Some(Ok(None)) | None => {}
            Some(Err(error)) => self.planner.records.push(PlanRecord {
                id: None,
                requested_at: now,
                deadline: now,
                outcome: Err(error),
            }),
        }
        stalled
    }

    /// How many plans reached their deadline unfinished and were finished on the
    /// thread advancing the universe, stalling it; zero when the host keeps to
    /// [`Self::plan_free_until`].
    #[must_use]
    pub const fn plans_finished_inline(&self) -> u64 {
        self.planner.finished_inline
    }
}

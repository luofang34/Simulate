//! Planned commands: asked for at due instants, stamped at deadlines, the same
//! whichever executor computes them.

use std::sync::Arc;

use sim_core::{SimInstant, TickSpan};
use sim_orbital::{LatchedBurnLaw, PlayerCommandPayload, VesselId};

use super::{seconds, universe_with};
use crate::UniverseError;
use crate::planning::{
    PlanExecutor, PlanJob, PlanOutput, PlanRecord, PlanRequest, PlanSchedule, PlannedCommand,
    SlicedPlanExecutor, SynchronousPlanExecutor, ThreadPlanExecutor,
};
use crate::universe::Universe;

/// A job that takes `slices` slices and then commands the vessel to coast at its
/// deadline.
struct Countdown {
    slices: u32,
    deadline: SimInstant,
}

impl PlanJob for Countdown {
    fn run_slice(&mut self) -> Result<Option<PlanOutput>, UniverseError> {
        if self.slices > 0 {
            self.slices = self.slices.saturating_sub(1);
            return Ok(None);
        }
        Ok(Some(PlanOutput {
            commands: vec![PlannedCommand {
                vessel: VesselId(1),
                at: self.deadline,
                payload: PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::Coast),
            }],
            report: None,
        }))
    }
}

#[derive(Debug)]
struct CountdownRequest {
    deadline: SimInstant,
}

impl PlanRequest for CountdownRequest {
    fn deadline(&self) -> SimInstant {
        self.deadline
    }

    fn start(&self) -> Box<dyn PlanJob> {
        Box::new(Countdown {
            slices: 5,
            deadline: self.deadline,
        })
    }
}

/// Plans every 20 s, each due 7 s later.
#[derive(Debug, Clone)]
struct Every20s {
    next: SimInstant,
}

impl PlanSchedule for Every20s {
    fn next_due(&self) -> Option<SimInstant> {
        Some(self.next)
    }

    fn plan(&mut self, universe: &Universe) -> Result<Option<Arc<dyn PlanRequest>>, UniverseError> {
        self.next = universe
            .now()
            .advanced(TickSpan::from_seconds_and_subsec_ticks(20, 0));
        Ok(Some(Arc::new(CountdownRequest {
            deadline: universe
                .now()
                .advanced(TickSpan::from_seconds_and_subsec_ticks(7, 0)),
        })))
    }

    fn clone_box(&self) -> Box<dyn PlanSchedule> {
        Box::new(self.clone())
    }
}

/// A schedule that never moves its due instant on.
#[derive(Debug, Clone)]
struct Stuck;

impl PlanSchedule for Stuck {
    fn next_due(&self) -> Option<SimInstant> {
        Some(SimInstant::START)
    }

    fn plan(
        &mut self,
        _universe: &Universe,
    ) -> Result<Option<Arc<dyn PlanRequest>>, UniverseError> {
        Ok(None)
    }

    fn clone_box(&self) -> Box<dyn PlanSchedule> {
        Box::new(self.clone())
    }
}

/// Flies vessel 1 in orbit for `end_s` with `schedules`, pumping `slices` before each
/// 3 s advance, and returns every plan record with the universe.
fn fly(
    executor: Box<dyn PlanExecutor>,
    schedules: Vec<Box<dyn PlanSchedule>>,
    slices: u32,
    end_s: u64,
) -> (Vec<PlanRecord>, Universe) {
    let mut universe = universe_with(&[1]).with_plan_executor(executor);
    universe.promote_to_orbital(VesselId(1)).unwrap();
    for schedule in schedules {
        universe.attach_plan_schedule(schedule);
    }
    let mut records = Vec::new();
    let mut t = 0;
    while t < end_s {
        t = (t + 3).min(end_s);
        universe.pump_plans(slices);
        if let Some(free) = universe.plan_free_until() {
            assert!(free < universe.plan_hold().unwrap().deadline);
        }
        universe.advance_to(seconds(t)).unwrap();
        records.extend(universe.take_plan_records());
    }
    (records, universe)
}

fn every_20s() -> Vec<Box<dyn PlanSchedule>> {
    vec![Box::new(Every20s {
        next: SimInstant::START,
    })]
}

/// When each plan was asked for and stamped, and what it commanded.
fn stamped(records: Vec<PlanRecord>) -> Vec<(SimInstant, SimInstant, PlanOutput)> {
    records
        .into_iter()
        .map(|record| {
            let output = record.outcome.unwrap();
            assert!(output.commands.iter().all(|c| c.at == record.deadline));
            (record.requested_at, record.deadline, output)
        })
        .collect()
}

#[test]
fn plans_are_stamped_at_their_deadlines_whichever_executor_runs_them() {
    let run = |executor: Box<dyn PlanExecutor>| stamped(fly(executor, every_20s(), 2, 100).0);
    let synchronous = run(Box::<SynchronousPlanExecutor>::default());
    assert_eq!(
        synchronous.first().map(|(asked, due, _)| (*asked, *due)),
        Some((seconds(0), seconds(7)))
    );
    assert_eq!(synchronous.len(), 5);
    assert_eq!(run(Box::<SlicedPlanExecutor>::default()), synchronous);
    assert_eq!(run(Box::<ThreadPlanExecutor>::default()), synchronous);
}

#[test]
fn a_plan_unfinished_at_its_deadline_is_finished_inline_and_counted() {
    // Never pumped, the sliced executor still has the plan's slices to run when its
    // deadline comes.
    let (records, universe) = fly(Box::<SlicedPlanExecutor>::default(), every_20s(), 0, 10);
    assert_eq!(universe.plans_finished_inline(), 1);
    let (threaded, threaded_universe) =
        fly(Box::<ThreadPlanExecutor>::default(), every_20s(), 0, 10);
    let inline = stamped(records);
    assert_eq!(inline.len(), 1);
    assert_eq!(inline, stamped(threaded));
    assert_eq!(
        universe.vehicle_state(VesselId(1)).unwrap(),
        threaded_universe.vehicle_state(VesselId(1)).unwrap()
    );
}

#[test]
fn a_schedule_that_does_not_move_on_is_detached_without_holding_up_others() {
    let schedules: Vec<Box<dyn PlanSchedule>> = vec![
        Box::new(Stuck),
        Box::new(Every20s {
            next: SimInstant::START,
        }),
    ];
    let (records, _) = fly(Box::<SynchronousPlanExecutor>::default(), schedules, 0, 100);
    let (stalled, planned): (Vec<_>, Vec<_>) = records
        .into_iter()
        .partition(|record| record.outcome.is_err());
    assert_eq!(stalled.len(), 1);
    assert!(matches!(
        stalled[0].outcome,
        Err(UniverseError::PlanScheduleStalled { .. })
    ));
    assert_eq!(planned.len(), 5);
}

#[test]
fn a_plan_whose_thread_cannot_start_says_so() {
    let executor = ThreadPlanExecutor::default().with_stack_size(usize::MAX);
    let (records, universe) = fly(Box::new(executor), every_20s(), 0, 10);
    assert_eq!(records.len(), 1);
    match &records[0].outcome {
        Err(UniverseError::PlanThreadSpawn { id, thread, .. }) => {
            assert_eq!(Some(*id), records[0].id);
            assert_eq!(thread, "plan-0");
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(universe.plans_finished_inline(), 0);
}

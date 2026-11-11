//! The JWST-like observatory about Sun–Earth L2 and its keeper: held in its box for a
//! year, lost without it, and its burns the same however and wherever its solves run.

use std::sync::Arc;

use glam::DVec3;
use sim_core::{SimInstant, TickSpan};
use sim_orbital::{VesselWarpConstraintInput, WarpBottleneckReason, evaluate_global_warp_status};
use sim_universe::{
    L2KeepingBurn, PlanExecutor, PlanOutput, PlanReport, SlicedPlanExecutor,
    SynchronousPlanExecutor, ThreadPlanExecutor,
};

use super::free_world::SolarSystem;
use super::space_world::{JwstAlone, SpaceWorld};

const JWST_YEAR_DAYS: u64 = 365;
const DAY_S: u64 = 86_400;
/// Box the observatory must stay in about L2, in the Sun–Earth rotating frame (`m`).
const JWST_BOX_M: DVec3 = DVec3::new(4.0e8, 1.0e9, 3.0e8);

fn span(seconds: u64) -> TickSpan {
    TickSpan::from_seconds_and_subsec_ticks(seconds, 0)
}

fn at_day(days: u64) -> SimInstant {
    SimInstant::START.advanced(span(days * DAY_S))
}

/// The observatory alone, from the date a spaceflight starts on when the host clock
/// is outside the kernels: they cover the year flown from it and the keeper's
/// lookahead beyond.
fn jwst_alone(solar: &SolarSystem, executor: Box<dyn PlanExecutor>, kept: bool) -> JwstAlone {
    let (start_utc_s, _) = super::mode::certified_space_utc_s(f64::NEG_INFINITY);
    let epoch =
        sim_spacetime::SimEpoch::builtin_from_utc(sim_geo::UtcTime::from_unix_seconds(start_utc_s))
            .expect("epoch");
    JwstAlone::new(&epoch, solar, executor, kept).expect("JWST_L2 universe")
}

fn in_jwst_box(offset: DVec3) -> bool {
    offset.abs().cmplt(JWST_BOX_M).all()
}

/// Every plan a run stamped, with when it was asked for and stamped, and the
/// observatory's state where the run ended, bit for bit.
#[derive(Debug, PartialEq)]
struct Flown {
    plans: Vec<(SimInstant, SimInstant, PlanOutput)>,
    end_state_bits: [u64; 6],
}

fn take_plans(jwst: &mut JwstAlone, plans: &mut Vec<(SimInstant, SimInstant, PlanOutput)>) {
    for record in jwst.universe.take_plan_records() {
        let output = record.outcome.expect("a holding plan");
        plans.push((record.requested_at, record.deadline, output));
    }
}

fn end_state_bits(jwst: &JwstAlone) -> [u64; 6] {
    let state = jwst
        .universe
        .inertial_state(SpaceWorld::jwst_l2_id())
        .expect("observatory");
    let p = state.pos_rel_icrf_m.to_array();
    let v = state.vel_rel_icrf_m_s.to_array();
    [p[0], p[1], p[2], v[0], v[1], v[2]].map(f64::to_bits)
}

fn burns(plans: &[(SimInstant, SimInstant, PlanOutput)]) -> Vec<L2KeepingBurn> {
    plans
        .iter()
        .filter_map(|(_, _, output)| output.report.map(|PlanReport::L2Holding(burn)| burn))
        .collect()
}

/// How a host slices time: each entry is one advance, repeated until the end.
#[derive(Clone, Copy)]
enum Pacing {
    /// Every six hours.
    Looks,
    /// Five hours seventeen minutes, then forty-one minutes, over and over.
    Ragged,
}

impl Pacing {
    fn advance(self, index: u64) -> TickSpan {
        match (self, index % 2) {
            (Self::Looks, _) => span(6 * 3_600),
            (Self::Ragged, 0) => span(5 * 3_600 + 17 * 60),
            (Self::Ragged, _) => span(41 * 60),
        }
    }
}

/// Flies the kept observatory `days` with plans run on `executor`, pumping one slice
/// before each advance when `pumped`.
fn fly(
    solar: &SolarSystem,
    executor: Box<dyn PlanExecutor>,
    pumped: bool,
    pacing: Pacing,
    days: u64,
) -> Flown {
    let mut jwst = jwst_alone(solar, executor, true);
    let end = at_day(days);
    let mut plans = Vec::new();
    let mut index = 0_u64;
    while jwst.universe.now() < end {
        if pumped {
            jwst.universe.pump_plans(1);
        }
        let next = jwst.universe.now().advanced(pacing.advance(index)).min(end);
        jwst.universe.advance_to(next).expect("advance");
        take_plans(&mut jwst, &mut plans);
        index = index.wrapping_add(1);
    }
    Flown {
        plans,
        end_state_bits: end_state_bits(&jwst),
    }
}

#[test]
fn jwst_l2_burns_do_not_depend_on_where_or_how_fast_its_solves_run() {
    let solar = SolarSystem::load().expect("solar system");
    let days = 50;
    type Run = (fn() -> Box<dyn PlanExecutor>, bool, Pacing);
    let runs: [Run; 6] = [
        (
            || Box::new(SynchronousPlanExecutor::default()),
            false,
            Pacing::Looks,
        ),
        (
            || Box::new(ThreadPlanExecutor::default()),
            false,
            Pacing::Looks,
        ),
        (
            || Box::new(SlicedPlanExecutor::default()),
            true,
            Pacing::Looks,
        ),
        (
            || Box::new(SynchronousPlanExecutor::default()),
            false,
            Pacing::Ragged,
        ),
        (
            || Box::new(ThreadPlanExecutor::default()),
            false,
            Pacing::Ragged,
        ),
        (
            || Box::new(SlicedPlanExecutor::default()),
            true,
            Pacing::Ragged,
        ),
    ];
    let flown: Vec<Flown> = std::thread::scope(|scope| {
        let handles: Vec<_> = runs
            .iter()
            .map(|&(executor, pumped, pacing)| {
                let solar = Arc::clone(&solar);
                scope.spawn(move || fly(&solar, executor(), pumped, pacing, days))
            })
            .collect();
        handles
            .into_iter()
            .map(|handle| handle.join().expect("run"))
            .collect()
    });
    let first = &flown[0];
    let made = burns(&first.plans);
    assert!(first.plans.len() >= 3, "{:?}", first.plans);
    assert!(made.first().is_some_and(|burn| burn.insertion), "{made:?}");
    for (index, other) in flown.iter().enumerate().skip(1) {
        assert_eq!(other, first, "run {index} differs");
    }
}

/// The observatory flown at the highest warp a host asks for, a frame of a quarter
/// second at a time, with plans computed a slice per frame and warp governed as
/// the spaceflight governs it; returns what was flown and how many frames the
/// planner held warp.
fn fly_at_high_warp(solar: &SolarSystem, days: u64) -> (Flown, u32) {
    let mut jwst = jwst_alone(solar, Box::new(SlicedPlanExecutor::default()), true);
    let end = at_day(days);
    let (mut plans, mut held) = (Vec::new(), 0_u32);
    while jwst.universe.now() < end {
        jwst.universe.pump_plans(1);
        let now = jwst.universe.now();
        let plan_wait_s = match jwst.universe.plan_hold() {
            Some(hold) if !hold.ready => hold.deadline.duration_since(now).as_seconds(),
            Some(_) | None => f64::INFINITY,
        };
        let observatory = VesselWarpConstraintInput {
            vessel_id: SpaceWorld::jwst_l2_id(),
            is_pinned_surface_rest: false,
            is_active_ground_contact: false,
            q_inf_pa: 0.0,
            in_proximity_group: false,
            hazard_tgo_s: f64::INFINITY,
            on_reduced_manifold: true,
            plan_wait_s,
        };
        let status = evaluate_global_warp_status(100_000.0, 0.25, &[observatory]);
        if status.bottleneck_reason == WarpBottleneckReason::PendingPlan {
            held = held.wrapping_add(1);
        }
        let frame = TickSpan::from_ticks_u128(
            (status.achieved_warp * 0.25 * sim_core::TICKS_PER_SECOND as f64) as u128,
        );
        let mut target = now.advanced(frame).min(end);
        if let Some(free) = jwst.universe.plan_free_until() {
            target = target.min(free.max(now));
        }
        jwst.universe.advance_to(target).expect("advance");
        take_plans(&mut jwst, &mut plans);
    }
    let flown = Flown {
        plans,
        end_state_bits: end_state_bits(&jwst),
    };
    (flown, held)
}

#[test]
fn at_high_warp_time_waits_for_the_keeper_and_its_burns_are_unchanged() {
    let solar = SolarSystem::load().expect("solar system");
    let days = 30;
    let (warped, held) = fly_at_high_warp(&solar, days);
    let paced = fly(
        &solar,
        Box::new(SynchronousPlanExecutor::default()),
        false,
        Pacing::Looks,
        days,
    );
    assert!(held > 0, "the planner never held warp");
    assert_eq!(warped, paced);
}

#[test]
fn restoring_a_checkpoint_with_a_solve_in_flight_replays_the_same_burns() {
    let solar = SolarSystem::load().expect("solar system");
    let mut jwst = jwst_alone(&solar, Box::new(ThreadPlanExecutor::default()), true);
    let mut before = Vec::new();
    // The maintenance burn asked for on day 21 is due on day 22.
    let saved_at = at_day(21).advanced(span(6 * 3_600));
    jwst.universe.advance_to(saved_at).expect("advance");
    take_plans(&mut jwst, &mut before);
    let hold = jwst.universe.plan_hold().expect("a plan in flight");
    assert!(hold.deadline > saved_at);
    let saved = jwst.universe.checkpoint();
    let continue_to_day_50 = |jwst: &mut JwstAlone| {
        let mut plans = Vec::new();
        let mut index = 0_u64;
        while jwst.universe.now() < at_day(50) {
            let next = jwst.universe.now().advanced(Pacing::Ragged.advance(index));
            jwst.universe
                .advance_to(next.min(at_day(50)))
                .expect("advance");
            take_plans(jwst, &mut plans);
            index = index.wrapping_add(1);
        }
        Flown {
            plans,
            end_state_bits: end_state_bits(jwst),
        }
    };
    let first = continue_to_day_50(&mut jwst);
    jwst.universe.restore(&saved).expect("restore");
    let replayed = continue_to_day_50(&mut jwst);
    assert!(first.plans.len() >= 2, "{:?}", first.plans);
    assert_eq!(replayed, first);
}

#[test]
fn jwst_l2_is_held_in_its_box_for_a_year_by_its_keeper() {
    let solar = SolarSystem::load().expect("solar system");
    let mut jwst = jwst_alone(&solar, Box::new(ThreadPlanExecutor::default()), true);
    let look_every = span(6 * 3_600);
    let mut plans = Vec::new();
    let mut largest = DVec3::ZERO;
    let mut last_y = 0.0_f64;
    let mut sign_changes = 0_u32;
    for look in 1..=JWST_YEAR_DAYS * 4 {
        let next = jwst.universe.now().advanced(look_every);
        jwst.universe.advance_to(next).expect("advance");
        take_plans(&mut jwst, &mut plans);
        let offset = jwst
            .keeper
            .offset_from_l2_m(&jwst.universe)
            .expect("offset from L2");
        largest = largest.max(offset.abs());
        assert!(
            in_jwst_box(offset),
            "left the box on day {:.2} at {offset:?} after {} burns",
            look as f64 / 4.0,
            burns(&plans).len()
        );
        if last_y * offset.y < 0.0 {
            sign_changes = sign_changes.wrapping_add(1);
        }
        last_y = offset.y;
    }
    let made = burns(&plans);
    let (insertion, upkeep) = made.split_first().expect("an insertion burn");
    let upkeep_m_s: f64 = upkeep.iter().map(|burn| burn.delta_v_m_s.abs()).sum();
    let summary = format!(
        "insertion {:.4} m/s; upkeep {upkeep_m_s:.4} m/s in {} burns; largest |offset| {largest:?} m; {sign_changes} y sign changes",
        insertion.delta_v_m_s.abs(),
        upkeep.len()
    );
    assert!(insertion.insertion, "{summary}");
    assert!(insertion.delta_v_m_s.abs() <= 10.0, "{summary}");
    assert!(upkeep_m_s <= 10.0, "{summary}");
    assert!(upkeep.iter().all(|burn| !burn.insertion), "{summary}");
    assert!(sign_changes >= 3, "{summary}");
    assert_eq!(jwst.universe.state_reinitializations(), 0, "{summary}");
}

#[test]
fn jwst_l2_left_without_its_keeper_leaves_its_box_within_a_year() {
    let solar = SolarSystem::load().expect("solar system");
    let mut jwst = jwst_alone(&solar, Box::new(SynchronousPlanExecutor::default()), false);
    let left_on_day = (1..=JWST_YEAR_DAYS).find(|&day| {
        jwst.universe.advance_to(at_day(day)).expect("advance");
        let offset = jwst
            .keeper
            .offset_from_l2_m(&jwst.universe)
            .expect("offset from L2");
        !in_jwst_box(offset)
    });
    assert!(left_on_day.is_some(), "unheld, it stayed a whole year");
}

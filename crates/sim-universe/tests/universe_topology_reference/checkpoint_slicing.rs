//! Mid-sequence checkpoint and `advance_to` slicing invariance tests across
//! post-staging, mid-tether reel-in, mid-docking dwell, and post-hard-mate.

use glam::DVec3;
use mbd_core::{ComponentId, Placement};
use sim_core::SimInstant;
use sim_orbital::VesselId;
use sim_universe::{Tier, Universe, UniverseCheckpoint};

use super::models::{
    STAGE1_CORE_IN_CG_M, at_ms, build_dock_spec, build_leo_mission_universe, build_leo_pair_at,
    build_tether_link, report_line,
};

fn advance_with_schedule(
    u: &mut Universe,
    start: SimInstant,
    target: SimInstant,
    chunks_ms: &[u64],
) {
    let mut cur = start;
    let mut idx = 0_usize;
    while cur < target {
        let step_ms = chunks_ms[idx % chunks_ms.len()];
        idx += 1;
        let next = cur
            .advanced(sim_core::TickSpan::from_ticks(
                step_ms * (sim_core::TICKS_PER_SECOND / 1_000),
            ))
            .min(target);
        u.advance_to(next).unwrap();
        cur = next;
    }
}

fn assert_universe_bit_equal(a: &Universe, b: &Universe, ids: &[VesselId]) {
    assert_eq!(a.now(), b.now());
    assert_eq!(a.active_vessel(), b.active_vessel());
    assert_eq!(a.inter_vessel_links(), b.inter_vessel_links());
    for &id in ids {
        assert_eq!(a.tier(id).unwrap(), b.tier(id).unwrap());
        assert_eq!(a.vehicle_state(id).unwrap(), b.vehicle_state(id).unwrap());
        if a.tier(id).unwrap() == Tier::Orbital {
            assert_eq!(a.inertial_state(id).unwrap(), b.inertial_state(id).unwrap());
        }
    }
}

fn replay_and_compare(
    mut mk_base_a: impl FnMut() -> Universe,
    mut mk_base_b: impl FnMut() -> Universe,
    cp: &UniverseCheckpoint,
    target: SimInstant,
    ids: &[VesselId],
    schedule_a: &[u64],
    schedule_b: &[u64],
) {
    let mut base_a = mk_base_a();
    base_a.restore(cp).unwrap();
    advance_with_schedule(&mut base_a, cp.at(), target, schedule_a);
    let mut states_a = Vec::new();
    for &id in ids {
        states_a.push((
            id,
            base_a.tier(id).unwrap(),
            base_a.vehicle_state(id).unwrap(),
        ));
    }
    let links_a = base_a.inter_vessel_links().to_vec();
    let active_a = base_a.active_vessel();

    let mut base_b = mk_base_b();
    base_b.restore(cp).unwrap();
    advance_with_schedule(&mut base_b, cp.at(), target, schedule_b);
    assert_eq!(base_b.inter_vessel_links(), links_a.as_slice());
    assert_eq!(base_b.active_vessel(), active_a);
    for (id, tier_a, st_a) in states_a {
        assert_eq!(base_b.tier(id).unwrap(), tier_a);
        assert_eq!(base_b.vehicle_state(id).unwrap(), st_a);
    }
}

#[test]
fn staging_and_tether_checkpoints_replay_bit_identically_across_slicing() {
    let mk_staged = || {
        let mut u = build_leo_mission_universe(true);
        let b = u
            .separate_stage_impulse(VesselId(1), ComponentId(2), 180.0, DVec3::NEG_X)
            .unwrap();
        (u, b)
    };
    let (mut u, booster) = mk_staged();
    u.advance_to(at_ms(200)).unwrap();
    let cp_post_stage = u.checkpoint();

    u.add_inter_vessel_link(build_tether_link(VesselId(1), VesselId(2), 3.6, -0.5))
        .unwrap();
    u.advance_to(at_ms(500)).unwrap();
    let cp_mid_tether = u.checkpoint();

    let (mut u_mono, b_mono) = mk_staged();
    assert_eq!(b_mono, booster);
    u_mono.advance_to(at_ms(2_200)).unwrap();

    let (mut u_stage, b_stage) = mk_staged();
    assert_eq!(b_stage, booster);
    u_stage.restore(&cp_post_stage).unwrap();
    advance_with_schedule(
        &mut u_stage,
        cp_post_stage.at(),
        at_ms(2_200),
        &[170, 390, 500, 940],
    );
    assert_universe_bit_equal(&u_stage, &u_mono, &[VesselId(1), VesselId(2), booster]);

    replay_and_compare(
        || mk_staged().0,
        || mk_staged().0,
        &cp_mid_tether,
        at_ms(2_500),
        &[VesselId(1), VesselId(2), booster],
        &[2_000],
        &[20, 140, 340, 500, 1_000],
    );
    assert!(matches!(
        u_stage.restore(&cp_mid_tether),
        Err(sim_universe::UniverseError::TiersChangedSinceCheckpoint)
    ));
    report_line("staging & tether checkpoint slicing: bit-identical");
}

#[test]
fn mid_docking_and_post_mate_orbital_checkpoints_replay_bit_identically() {
    let mk_pair = || build_leo_pair_at(true, STAGE1_CORE_IN_CG_M + DVec3::new(1.25, 0.0, 0.0));
    let mut u_dock = mk_pair();
    u_dock
        .dock_soft_capture(build_dock_spec(VesselId(1), VesselId(2), 0.25))
        .unwrap();
    u_dock.advance_to(at_ms(160)).unwrap();
    let cp_mid_docking = u_dock.checkpoint();

    u_dock.advance_to(at_ms(3_000)).unwrap();
    assert_eq!(u_dock.tier(VesselId(1)).unwrap(), Tier::Orbital);
    let cp_post_mate_orbital = u_dock.checkpoint();

    let mk_post_mate = || {
        let mut u = mk_pair();
        u.dock_soft_capture(build_dock_spec(VesselId(1), VesselId(2), 0.25))
            .unwrap();
        u.advance_to(at_ms(3_000)).unwrap();
        u
    };

    replay_and_compare(
        mk_pair,
        mk_pair,
        &cp_mid_docking,
        at_ms(4_000),
        &[VesselId(1)],
        &[3_840],
        &[40, 120, 680, 1_000],
    );
    replay_and_compare(
        mk_post_mate,
        mk_post_mate,
        &cp_post_mate_orbital,
        at_ms(8_000),
        &[VesselId(1)],
        &[5_000],
        &[300, 700, 1_300, 2_700],
    );

    let mut u_check = mk_post_mate();
    u_check.restore(&cp_post_mate_orbital).unwrap();
    assert_eq!(u_check.tier(VesselId(1)).unwrap(), Tier::Orbital);
    assert!(matches!(
        u_check.restore(&cp_mid_docking),
        Err(sim_universe::UniverseError::TiersChangedSinceCheckpoint)
    ));
    report_line("docking & post-mate orbital checkpoint slicing: bit-identical");
}

#[test]
fn restore_retired_vessel_while_survivor_is_orbital_preserves_world_providers() {
    let mut u = build_leo_pair_at(false, STAGE1_CORE_IN_CG_M + DVec3::new(2.0, 0.0, 0.0));
    u.promote_to_orbital(VesselId(1)).unwrap();
    u.promote_to_orbital(VesselId(2)).unwrap();
    u.advance_to(at_ms(1_000)).unwrap();
    let cp_pre_mate = u.checkpoint();

    u.hard_mate(
        VesselId(1),
        ComponentId(0),
        VesselId(2),
        ComponentId(0),
        Placement::child_of(ComponentId(0), DVec3::new(2.0, 0.0, 0.0)),
    )
    .unwrap();
    u.promote_to_orbital(VesselId(1)).unwrap();
    u.advance_to(at_ms(2_000)).unwrap();

    u.restore(&cp_pre_mate).unwrap();
    assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Orbital);
    assert_eq!(u.tier(VesselId(2)).unwrap(), Tier::Orbital);
    u.demote_to_local(VesselId(2)).unwrap();
    let p2 = u
        .local::<sim_universe::MultibodyLocalPlant>(VesselId(2))
        .unwrap();
    assert_eq!(p2.block().telemetry().dynamic_pressure_pa, 0.0);
}

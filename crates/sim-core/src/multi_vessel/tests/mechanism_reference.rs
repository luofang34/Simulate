//! Recorded outcomes of the coordinator's mechanisms: when each one acts
//! and how the vessels move afterwards.

use mbd_multibody::{TetherCommand, WinchTetherSpec, WinchTetherState};

use super::*;
use crate::Block;
use crate::recorded::{assert_rows, vec_rows};

const ORBIT_X_M: f64 = 7_000_000.0;

fn single(name: &str, mass: f64, inertia: f64, x_m: f64) -> MultibodyVehicleBlock {
    single_stepping(name, mass, inertia, x_m, 50)
}

fn single_stepping(
    name: &str,
    mass: f64,
    inertia: f64,
    x_m: f64,
    steps_per_second: u64,
) -> MultibodyVehicleBlock {
    tank_block_stepping(
        ComponentTree::new([part(name, Placement::ROOT, JointSpec::FIXED, mass, inertia)]).unwrap(),
        ComponentId(0),
        0,
        50.0,
        DVec3::new(x_m, 0.0, 0.0),
        steps_per_second,
    )
}

#[test]
fn a_stage_pushed_off_by_an_impulse_leaves_at_the_recorded_speeds() {
    let tree = ComponentTree::new([
        part("core", Placement::ROOT, JointSpec::FIXED, 800.0, 300.0),
        part(
            "booster",
            Placement::child_of(ComponentId(0), DVec3::new(-4.0, 0.0, 0.0)),
            JointSpec::FIXED,
            400.0,
            150.0,
        ),
    ])
    .unwrap();
    let parent = tank_block(
        tree,
        ComponentId(1),
        7,
        120.0,
        DVec3::new(ORBIT_X_M, 0.0, 0.0),
    );
    let mut coord = MultiVesselCoordinator::new(parent);
    let child_id = coord
        .separate_stage_impulse(SimVesselId(1), ComponentId(1), 500.0, DVec3::NEG_X)
        .unwrap();
    for _ in 0..50 {
        coord.step_all(&BTreeMap::new()).unwrap();
    }
    let parent = coord.vessel(SimVesselId(1)).unwrap().output();
    let child = coord.vessel(child_id).unwrap().output();
    // An impulse split between two free bodies with no field: both coast,
    // so the recorded speeds are the momentum split to round-off.
    let mut rows = Vec::new();
    rows.extend(vec_rows(
        ["parent_vx", "parent_vy", "parent_vz"],
        parent.velocity_ecef,
        [0.625, 0.0, 0.0],
        1e-12,
    ));
    rows.extend(vec_rows(
        ["child_vx", "child_vy", "child_vz"],
        child.velocity_ecef,
        [-0.961_538_461_538_461_6, 0.0, 0.0],
        1e-12,
    ));
    let gap = (parent.position_ecef - child.position_ecef).length();
    rows.push(("gap_m", gap, 5.586_538_460_105_658, 1e-9));
    assert_rows(&rows);
}

#[test]
fn a_soft_capture_mates_at_the_recorded_step() {
    let mut coord = MultiVesselCoordinator::new(single("chaser", 1_000.0, 400.0, ORBIT_X_M));
    let target_id = coord.spawn_independent(single("target", 800.0, 300.0, ORBIT_X_M + 0.2));
    coord.add_link(docking_link(target_id));
    let mut mated_at = None;
    for step in 0..80_u32 {
        let report = coord.step_all(&BTreeMap::new()).unwrap();
        if !report.merged_vessels.is_empty() {
            mated_at = Some(step);
            break;
        }
    }
    let merged = coord.vessel(SimVesselId(1)).unwrap();
    let state = merged.output();
    // The ring's spring and damper act inside each step and its phase
    // changes land on step boundaries, so the latch may move by a step or
    // two and the chaser a few millimetres with it. Each vessel integrates
    // against the other's port carried on from the start of the step, so
    // the pair's momentum, zero here, closes with the step and not to
    // round-off: 0.03 kg·m/s on 1,900 kg at 50 Hz.
    assert_rows(&[
        ("mated_at_step", f64::from(mated_at.unwrap()), 44.0, 2.0),
        (
            "mated_cg_x",
            state.position_ecef.x - ORBIT_X_M,
            0.069_786_892_272_531_99,
            5e-3,
        ),
        ("mated_speed", state.velocity_ecef.length(), 0.0, 1e-4),
        (
            "mated_mass",
            merged.mass_properties().total_mass_kg,
            1_900.0,
            1e-9,
        ),
    ]);
}

#[test]
fn a_rail_seat_leaves_at_the_recorded_step_and_speed() {
    let mut coord = MultiVesselCoordinator::new(rail_carrier());
    coord.add_link(rail_link());
    let mut ejected = None;
    for step in 0..40_u32 {
        let report = coord.step_all(&BTreeMap::new()).unwrap();
        if let Some(&(_, child)) = report.spawned_vessels.first() {
            ejected = Some((step, child));
            break;
        }
    }
    let (step, child) = ejected.unwrap();
    let seat = coord.vessel(child).unwrap().output();
    let airframe = coord.vessel(SimVesselId(1)).unwrap().output();
    let momentum_z = 100.0 * seat.velocity_ecef.z + 2_010.0 * airframe.velocity_ecef.z;
    // The catapult and guide friction act along the rail at every stage and
    // the release latches on a step boundary: the seat's exit speed follows
    // the work done over the metre of rail, within the last step's share.
    assert_rows(&[
        ("released_at_step", f64::from(step), 11.0, 1.0),
        ("seat_vz", seat.velocity_ecef.z, 8.700_000_000_000_001, 0.2),
        (
            "airframe_vz",
            airframe.velocity_ecef.z,
            -0.432_835_820_895_522_36,
            0.01,
        ),
        ("momentum_z", momentum_z, 0.0, 1e-6),
    ]);
}

fn reel_in_tether(payload: SimVesselId) -> InterVesselLink {
    InterVesselLink::WinchTether {
        id: 3,
        carrier: SimVesselId(1),
        anchor_carrier: ComponentId(0),
        offset_carrier_m: DVec3::new(0.5, 0.0, 0.0),
        payload,
        anchor_payload: ComponentId(0),
        offset_payload_m: DVec3::new(-0.5, 0.1, 0.0),
        spec: WinchTetherSpec {
            stiffness_n_m: 2_000.0,
            damping_n_s_m: 400.0,
            max_length_m: 20.0,
            breaking_tension_n: 50_000.0,
            burn_through_energy_j: None,
            auto_cut_low_tension_n: None,
            auto_cut_dwell_s: 0.0,
        },
        state: WinchTetherState::intact_at(9.5),
        command: TetherCommand {
            payout_rate_m_s: -0.5,
            heat_rate_w: 0.0,
            cut: false,
        },
    }
}

#[test]
fn a_reeled_in_tether_draws_two_vessels_together_as_recorded() {
    let mut coord = MultiVesselCoordinator::new(single("carrier", 1_000.0, 400.0, ORBIT_X_M));
    let payload = coord.spawn_independent(single("payload", 800.0, 300.0, ORBIT_X_M + 10.0));
    coord.add_link(reel_in_tether(payload));
    for _ in 0..200 {
        let report = coord.step_all(&BTreeMap::new()).unwrap();
        assert!(report.cut_tethers.is_empty());
    }
    let a = coord.vessel(SimVesselId(1)).unwrap().output();
    let b = coord.vessel(payload).unwrap().output();
    let InterVesselLink::WinchTether { state, .. } = &coord.links_mut()[0] else {
        panic!("the tether is the only link");
    };
    let momentum = a.velocity_ecef * 1_050.0 + b.velocity_ecef * 850.0;
    let gap = (b.position_ecef - a.position_ecef).length();
    let closing = (b.velocity_ecef - a.velocity_ecef).x;
    // The line's tension follows both vessels inside a step and its length
    // pays out continuously. Each vessel integrates against the other's
    // anchor carried on from the start of the step, so the pair's momentum
    // closes with the step and not to round-off;
    // `halving_the_step_keeps_the_reeled_in_gap_and_closes_the_pair_momentum`
    // holds both to numbers.
    assert_rows(&[
        ("gap_m", gap, REELED_IN_GAP_M, 0.02),
        ("closing_m_s", closing, -0.777_834_309_755_426_6, 0.02),
        (
            "line_m",
            state.deployed_length_m,
            7.500_000_000_000_043,
            0.011,
        ),
        (
            "payload_spin_z",
            b.angular_rate_body.z,
            0.114_208_178_058_870_24,
            2e-3,
        ),
        ("momentum", momentum.length(), 0.0, 0.25),
    ]);
}

/// The gap between the two vessels and the momentum of the pair after four
/// seconds of reeling in, stepping `steps_per_second` times a second.
fn reeled_in(steps_per_second: u64) -> (f64, f64) {
    let mut coord = MultiVesselCoordinator::new(single_stepping(
        "carrier",
        1_000.0,
        400.0,
        ORBIT_X_M,
        steps_per_second,
    ));
    let payload = coord.spawn_independent(single_stepping(
        "payload",
        800.0,
        300.0,
        ORBIT_X_M + 10.0,
        steps_per_second,
    ));
    coord.add_link(reel_in_tether(payload));
    for _ in 0..4 * steps_per_second {
        coord.step_all(&BTreeMap::new()).unwrap();
    }
    let a = coord.vessel(SimVesselId(1)).unwrap().output();
    let b = coord.vessel(payload).unwrap().output();
    let momentum = a.velocity_ecef * 1_050.0 + b.velocity_ecef * 850.0;
    (
        (b.position_ecef - a.position_ecef).length(),
        momentum.length(),
    )
}

/// The gap recorded at 50 Hz by
/// [`a_reeled_in_tether_draws_two_vessels_together_as_recorded`].
const REELED_IN_GAP_M: f64 = 8.113_764_781_763_89;

#[test]
fn halving_the_step_keeps_the_reeled_in_gap_and_closes_the_pair_momentum() {
    let flown = [50, 100, 200, 400, 800, 1_600].map(|rate| (rate, reeled_in(rate)));
    // The gap does not close on a limit as the step halves: it moves by
    // tenths of a millimetre either way from one step size to the next.
    // What holds is that the recorded 50 Hz gap is within half a millimetre
    // of the gap at every one of these steps, a fortieth of the tolerance
    // it is held to.
    for (rate, (gap, _)) in flown {
        assert!(
            (gap - REELED_IN_GAP_M).abs() < 5e-4,
            "at {rate} Hz the gap is {gap} m"
        );
    }
    // Each vessel integrates against the other's anchor carried on from the
    // start of the step, so the momentum of the pair is not exact; it is
    // first order in the step, halving with it towards zero.
    assert!(
        flown[0].1.1 < 0.025,
        "pair momentum at 50 Hz: {:?}",
        flown[0]
    );
    for pair in flown.windows(2) {
        let ((_, (_, coarse)), (rate, (_, fine))) = (pair[0], pair[1]);
        let ratio = coarse / fine;
        assert!(
            (1.7..2.2).contains(&ratio),
            "halving the step to {rate} Hz took the pair momentum from {coarse} to {fine} kg·m/s"
        );
    }
}

//! Every ironbird rig steps its plant through a `SimSession`: the 30 Hz
//! optical channel exposes on the master clock, identically across runs and
//! regardless of wall-clock pacing.

use aviate_xil_contract::{FcState, pack_fc_status, seqlock_write};
use core::sync::atomic::AtomicU32;
use ironbird::{
    FixedWingVehicle, OPTICAL_CHANNEL, OPTICAL_PERIOD, run_tier0_fixed_wing_cruise,
    run_tier1_shm_x500,
};
use sim_aviate::{DirectNoiseTier, SharedStateV2};
use sim_core::{HitlSyncMode, SimInstant, TICKS_PER_SECOND, TickSpan};
use sim_geo::Geodetic;

fn hover_peer(shared: &mut SharedStateV2) {
    let step = shared.state.sim_step;
    let generation = shared.header.reset_generation;
    shared.control.fc_status = pack_fc_status(generation, FcState::Ready);
    let cmd = &mut shared.command;
    seqlock_write(AtomicU32::from_mut(&mut cmd.seq), || {
        cmd.fc_step_ack = step;
        cmd.num_motors = 4;
        for slot in cmd.motor_vel_bits.iter_mut().take(4) {
            *slot = 790.0_f64.to_bits();
        }
    });
}

fn exposure_instant(k: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_ticks(OPTICAL_PERIOD.ticks() * k))
}

#[test]
fn tier1_session_exposures_replay_bit_for_bit_and_ignore_pacing() {
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 20.0);
    let lockstep = HitlSyncMode::LockstepBarrier {
        processing_delay: TickSpan::from_ticks(TICKS_PER_SECOND / 400),
    };
    let run_a = run_tier1_shm_x500(origin, 1.0, lockstep, hover_peer).unwrap();
    let run_b = run_tier1_shm_x500(origin, 1.0, lockstep, hover_peer).unwrap();

    assert_eq!(run_a.steps_executed, 400);
    assert_eq!(run_a.optical_exposures, 30);
    let last = run_a.last_optical_exposure.unwrap();
    assert_eq!(last.channel, OPTICAL_CHANNEL);
    assert_eq!(last.sequence, 29);
    assert_eq!(last.exposure_at, exposure_instant(30));
    assert_eq!(last.state, run_a.final_state);
    assert_eq!(run_b.optical_exposures, run_a.optical_exposures);
    assert_eq!(run_b.last_optical_exposure, run_a.last_optical_exposure);
    assert_eq!(run_b.final_state, run_a.final_state);

    // Wall-clock pacing must not move a single exposure.
    let fast = run_tier1_shm_x500(origin, 0.10, lockstep, hover_peer).unwrap();
    let paced =
        run_tier1_shm_x500(origin, 0.10, HitlSyncMode::RealTimeStamped, hover_peer).unwrap();
    assert_eq!(fast.optical_exposures, 3);
    assert_eq!(paced.optical_exposures, fast.optical_exposures);
    assert_eq!(
        paced.last_optical_exposure.map(|e| e.exposure_at),
        Some(exposure_instant(3))
    );
}

#[test]
fn tier0_fixed_wing_session_exposures_replay_exactly() {
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 450.0);
    let run = || {
        run_tier0_fixed_wing_cruise(
            FixedWingVehicle::C172,
            origin,
            1.0,
            DirectNoiseTier::Consumer,
            7,
        )
        .unwrap()
    };
    let run_a = run();
    let run_b = run();
    assert_eq!(run_a.steps_executed, 120);
    assert_eq!(run_a.optical_exposures, 30);
    assert_eq!(run_a.last_optical_exposure, run_b.last_optical_exposure);
    assert_eq!(
        run_a.last_optical_exposure.map(|e| e.exposure_at),
        Some(exposure_instant(30))
    );
}

//! Multi-tier closed-loop integration tests across Tier 0 (`direct-aviate`),
//! Tier 1 (`SharedStateV2` seqlock), and Tier 2 (`MAVLink v2 HIL` over TCP).

use aviate_xil_contract::{FcState, pack_fc_status, seqlock_write};
use ironbird::{
    FixedWingVehicle, run_tier0_fixed_wing_cruise, run_tier0_x500_hover_climb, run_tier1_shm_x500,
};
use sim_aviate::DirectNoiseTier;
use sim_core::{HitlSyncMode, TICKS_PER_SECOND, TickSpan};
use sim_geo::Geodetic;

#[test]
fn tier0_x500_climbs_and_replays_bit_identically_with_industrial_noise() {
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 15.0);
    let run_a =
        run_tier0_x500_hover_climb(origin, 4.0, 5.0, DirectNoiseTier::Industrial, 42).unwrap();
    let run_b =
        run_tier0_x500_hover_climb(origin, 4.0, 5.0, DirectNoiseTier::Industrial, 42).unwrap();

    assert_eq!(run_a.steps_executed, 2_000);
    assert_eq!(run_a.final_state, run_b.final_state);
    let climbed = run_a.final_geodetic.height_m - origin.height_m;
    assert!(climbed > 2.5 && climbed < 5.5, "climbed {climbed:.3} m");
    assert!(run_a.realtime_speedup > 10.0);
}

#[test]
fn tier0_c172_and_light_trainer_fly_closed_loop_with_exact_determinism() {
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 450.0);
    let c172_a = run_tier0_fixed_wing_cruise(
        FixedWingVehicle::C172,
        origin,
        5.0,
        DirectNoiseTier::Consumer,
        7,
    )
    .unwrap();
    let c172_b = run_tier0_fixed_wing_cruise(
        FixedWingVehicle::C172,
        origin,
        5.0,
        DirectNoiseTier::Consumer,
        7,
    )
    .unwrap();
    assert_eq!(c172_a.final_state, c172_b.final_state);
    assert!(c172_a.final_geodetic.lat_rad > origin.lat_rad);

    let trainer = run_tier0_fixed_wing_cruise(
        FixedWingVehicle::LightTrainer,
        origin,
        5.0,
        DirectNoiseTier::Off,
        1,
    )
    .unwrap();
    assert!(trainer.final_geodetic.lat_rad > origin.lat_rad);
}

#[test]
fn tier1_shm_seqlock_flies_x500_plant_deterministically() {
    use core::sync::atomic::AtomicU32;

    let origin = Geodetic::from_degrees(39.1582, -75.5244, 20.0);
    let mode = HitlSyncMode::LockstepBarrier {
        processing_delay: TickSpan::from_ticks(TICKS_PER_SECOND / 400),
    };
    let summary = run_tier1_shm_x500(origin, 1.0, mode, |shared| {
        let step = shared.state.sim_step;
        let generation = shared.header.reset_generation;
        shared.control.fc_status = pack_fc_status(generation, FcState::Ready);
        let cmd = &mut shared.command;
        seqlock_write(AtomicU32::from_mut(&mut cmd.seq), || {
            cmd.fc_step_ack = step;
            cmd.num_motors = 4;
            let hover_rad_s = 790.0_f64.to_bits();
            for slot in cmd.motor_vel_bits.iter_mut().take(4) {
                *slot = hover_rad_s;
            }
        });
    })
    .unwrap();

    assert_eq!(summary.steps_executed, 400);
    assert!(summary.bridge_stats.delivered >= 399);
    assert!(summary.final_geodetic.height_m > origin.height_m);
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn tier2_mavlink_hil_tcp_flies_jsbsim_c172_closed_loop() {
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::thread;

    use ironbird::run_tier2_mavlink_fixed_wing;
    use sim_xil::mavlink_hil::{
        HilActuatorControls, HilControlMapping, MAV_MODE_FLAG_SAFETY_ARMED, MavlinkHilConfig,
        MavlinkHilReader, MavlinkHilTcpEndpoint, encode_hil_actuator_controls,
    };

    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();

    let fc_peer = thread::spawn(move || {
        let (mut sock, _) = listener.accept().unwrap();
        let mut rx = [0_u8; 512];
        let mut seq = 0_u8;
        let mut reader = MavlinkHilReader::new();
        for step in 0..120_u64 {
            let n = sock.read(&mut rx).unwrap();
            assert!(n > 0);
            reader.push_bytes(rx.get(..n).unwrap_or_default());
            let mut controls = [0.0_f32; 16];
            if let Some(thr) = controls.get_mut(3) {
                *thr = 0.7;
            }
            let reply = HilActuatorControls {
                time_usec: step * 8_333,
                controls,
                mode: MAV_MODE_FLAG_SAFETY_ARMED,
                flags: 0,
            };
            let mut tx = [0_u8; 128];
            let out_n = encode_hil_actuator_controls(seq, 1, 1, &reply, &mut tx).unwrap();
            seq = seq.wrapping_add(1);
            sock.write_all(tx.get(..out_n).unwrap_or_default()).unwrap();
        }
    });

    let ep = MavlinkHilTcpEndpoint::connect(addr, MavlinkHilConfig::default()).unwrap();
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 500.0);
    let summary = run_tier2_mavlink_fixed_wing(
        FixedWingVehicle::C172,
        origin,
        1.0,
        HitlSyncMode::LockstepBarrier {
            processing_delay: TickSpan::from_ticks(TICKS_PER_SECOND / 120),
        },
        HilControlMapping::default(),
        ep,
    )
    .unwrap();

    fc_peer.join().unwrap();
    assert_eq!(summary.steps_executed, 120);
    assert!(summary.final_geodetic.lat_rad > origin.lat_rad);
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn tier2_betaflight_udp_flies_x500_plant_closed_loop() {
    use std::net::UdpSocket;
    use std::thread;

    use ironbird::run_tier2_betaflight_x500;
    use sim_xil::betaflight::{
        BetaflightFdmPacket, BetaflightServoPacket, BetaflightUdpConfig, BetaflightUdpEndpoint,
    };

    let bf_rx = UdpSocket::bind("127.0.0.1:0").unwrap();
    let fdm_target = bf_rx.local_addr().unwrap();
    let ep = BetaflightUdpEndpoint::bind(
        "127.0.0.1:0".parse().unwrap(),
        fdm_target,
        BetaflightUdpConfig::default(),
    )
    .unwrap();
    let servo_target = ep.servo_local_addr().unwrap();

    let bf_peer = thread::spawn(move || {
        let mut buf = [0_u8; 256];
        let servo = BetaflightServoPacket {
            motors: [0.65, 0.65, 0.65, 0.65],
            motor_count: 4,
            all_motors: [0.0; 16],
        };
        let reply = servo.to_quad_bytes();
        for _ in 0..200_u32 {
            let (n, _) = bf_rx.recv_from(&mut buf).unwrap();
            let fdm = BetaflightFdmPacket::from_le_bytes(buf.get(..n).unwrap_or_default()).unwrap();
            assert!(fdm.pressure_pa > 80_000.0);
            let _ = bf_rx.send_to(&reply, servo_target).unwrap();
        }
    });

    let origin = Geodetic::from_degrees(39.1582, -75.5244, 15.0);
    let summary = run_tier2_betaflight_x500(
        origin,
        0.5,
        HitlSyncMode::LockstepBarrier {
            processing_delay: TickSpan::from_ticks(TICKS_PER_SECOND / 400),
        },
        ep,
    )
    .unwrap();

    bf_peer.join().unwrap();
    assert_eq!(summary.steps_executed, 200);
    assert!(summary.final_geodetic.height_m > origin.height_m);
}

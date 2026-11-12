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
    // Tier 0 runs unpaced: no pacer step is ever taken, so speed is bounded by compute alone.
    assert_eq!(run_a.pacer_stats, sim_xil::PacerStats::default());
    assert!((run_a.simulated_seconds - 5.0).abs() < 1e-9);
}

#[test]
fn tier0_c172_flies_closed_loop_with_exact_determinism() {
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

    let noiseless =
        run_tier0_fixed_wing_cruise(FixedWingVehicle::C172, origin, 5.0, DirectNoiseTier::Off, 1)
            .unwrap();
    assert!(noiseless.final_geodetic.lat_rad > origin.lat_rad);
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

#[cfg(test)]
fn hover_peer(shared: &mut sim_aviate::SharedStateV2) {
    use aviate_xil_contract::{FcState, pack_fc_status, seqlock_write};
    use core::sync::atomic::AtomicU32;

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

#[test]
fn tier1_realtime_paces_to_wall_clock_while_lockstep_runs_faster() {
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 20.0);
    let realtime =
        run_tier1_shm_x500(origin, 0.10, HitlSyncMode::RealTimeStamped, hover_peer).unwrap();
    assert_eq!(realtime.steps_executed, 40);
    assert!(realtime.wall_seconds >= 0.085, "{}", realtime.wall_seconds);
    assert_eq!(realtime.pacer_stats.steps, 40);
    assert!(realtime.pacer_stats.total_sleep_us > 0);

    let lockstep = HitlSyncMode::LockstepBarrier {
        processing_delay: TickSpan::from_ticks(TICKS_PER_SECOND / 400),
    };
    let fast = run_tier1_shm_x500(origin, 0.10, lockstep, hover_peer).unwrap();
    assert_eq!(fast.steps_executed, 40);
    assert_eq!(fast.pacer_stats, sim_xil::PacerStats::default());
}

#[test]
fn tier1_realtime_slow_peer_records_overruns() {
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 20.0);
    let slow_peer = |shared: &mut sim_aviate::SharedStateV2| {
        std::thread::sleep(std::time::Duration::from_millis(6));
        hover_peer(shared);
    };
    let summary =
        run_tier1_shm_x500(origin, 0.05, HitlSyncMode::RealTimeStamped, slow_peer).unwrap();
    assert_eq!(summary.steps_executed, 20);
    assert!(
        summary.pacer_stats.overrun_count > 0,
        "{:?}",
        summary.pacer_stats
    );
    assert!(
        summary.pacer_stats.max_overrun_us > 2_000,
        "{:?}",
        summary.pacer_stats
    );
}

#[cfg(test)]
#[cfg(not(target_arch = "wasm32"))]
fn sample_stimulus() -> sim_xil::XilStimulus {
    use sim_atmosphere::{AtmosphereField, StandardAtmosphere};
    use sim_geo::UtcTime;

    let geo = Geodetic::from_degrees(39.158, -75.524, 300.0);
    let vehicle = sim_core::VehicleState::level_flight(&geo, 0.0, 55.0);
    let atm = StandardAtmosphere
        .sample(&geo, UtcTime::from_unix_seconds(0.0))
        .unwrap();
    sim_xil::XilStimulus::from_state_and_atmosphere(
        vehicle,
        glam::DVec3::new(0.0, 0.0, -sim_xil::STANDARD_GRAVITY_M_S2),
        &atm,
    )
    .unwrap()
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn mavlink_tcp_post_never_blocks_reports_full_buffer_and_drains_intact() {
    use std::io::Read;
    use std::net::TcpListener;
    use std::sync::mpsc;
    use std::thread;
    use std::time::{Duration, Instant};

    use sim_core::{ExternalEndpoint, SimInstant};
    use sim_xil::mavlink_hil::{MavlinkHilConfig, MavlinkHilReader, MavlinkHilTcpEndpoint};

    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();
    let (start_reading, reading_gate) = mpsc::channel::<()>();
    let stalled_peer = thread::spawn(move || {
        let (mut sock, _) = listener.accept().unwrap();
        reading_gate.recv().unwrap();
        let mut reader = MavlinkHilReader::new();
        let mut buf = [0_u8; 4096];
        loop {
            let n = sock.read(&mut buf).unwrap();
            if n == 0 {
                break;
            }
            reader.push_bytes(&buf[..n]);
        }
        (
            reader.decoded_frames() + reader.unknown_frames(),
            reader.crc_errors(),
        )
    });

    let mut ep = MavlinkHilTcpEndpoint::connect(addr, MavlinkHilConfig::default()).unwrap();
    let stimulus = sample_stimulus();
    let mut at = SimInstant::START;
    let mut accepted = 0_u64;
    let full = loop {
        assert!(accepted < 5_000_000, "socket never filled");
        match ep.post(at, &stimulus) {
            Ok(()) => accepted += 1,
            Err(err) => break err,
        }
        at = at.advanced(TickSpan::from_ticks(1));
    };
    assert!(
        full.detail
            .contains("outbound buffer full (peer not draining)"),
        "{full}"
    );
    assert!(ep.pending_tx_bytes() > 0 && ep.pending_tx_bytes() <= 65_536);

    start_reading.send(()).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while ep.pending_tx_bytes() > 0 {
        assert!(
            Instant::now() < deadline,
            "{} bytes never drained",
            ep.pending_tx_bytes()
        );
        assert!(ep.try_receive().unwrap().is_none());
        thread::yield_now();
    }
    drop(ep);

    let (frames, crc_errors) = stalled_peer.join().unwrap();
    assert_eq!(crc_errors, 0);
    // The first burst also carries one HEARTBEAT and one HIL_GPS.
    assert_eq!(frames, accepted + 2);
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn tier2_mavlink_tcp_realtime_paces_without_blocking_on_peer() {
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::thread;

    use ironbird::run_tier2_mavlink_fixed_wing;
    use sim_xil::mavlink_hil::{
        HilActuatorControls, HilControlMapping, MAV_MODE_FLAG_SAFETY_ARMED, MavlinkHilConfig,
        MavlinkHilTcpEndpoint, encode_hil_actuator_controls,
    };

    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();
    let fc_peer = thread::spawn(move || {
        let (mut sock, _) = listener.accept().unwrap();
        let mut rx = [0_u8; 1024];
        let mut seq = 0_u8;
        let mut controls = [0.0_f32; 16];
        controls[3] = 0.7;
        let reply = HilActuatorControls {
            time_usec: 0,
            controls,
            mode: MAV_MODE_FLAG_SAFETY_ARMED,
            flags: 0,
        };
        while sock.read(&mut rx).unwrap_or(0) > 0 {
            let mut tx = [0_u8; 128];
            let n = encode_hil_actuator_controls(seq, 1, 1, &reply, &mut tx).unwrap();
            seq = seq.wrapping_add(1);
            if sock.write_all(&tx[..n]).is_err() {
                break;
            }
        }
    });

    let ep = MavlinkHilTcpEndpoint::connect(addr, MavlinkHilConfig::default()).unwrap();
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 500.0);
    let summary = run_tier2_mavlink_fixed_wing(
        FixedWingVehicle::C172,
        origin,
        0.10,
        HitlSyncMode::RealTimeStamped,
        HilControlMapping::default(),
        ep,
    )
    .unwrap();
    fc_peer.join().unwrap();

    assert_eq!(summary.steps_executed, 12);
    assert!(summary.wall_seconds >= 0.085, "{}", summary.wall_seconds);
    assert_eq!(summary.pacer_stats.steps, 12);
    assert!(
        summary.bridge_stats.delivered >= 1,
        "{:?}",
        summary.bridge_stats
    );
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn cli_supports_directory_tier0_and_design_estimation() {
    use ironbird::cli::{CliMode, CliOutcome, execute_cli, parse_args};
    use std::path::PathBuf;

    let c172_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../vehicles/c172s")
        .display()
        .to_string();

    let tier0_args = parse_args([
        "--mode",
        "tier0-dir",
        "--package-dir",
        &c172_dir,
        "--duration-s",
        "0.25",
    ])
    .unwrap();
    assert_eq!(tier0_args.mode, CliMode::Tier0Dir);
    match execute_cli(&tier0_args).unwrap() {
        CliOutcome::Rig(summary) => assert_eq!(summary.steps_executed, 30),
        other => panic!("expected Rig outcome, got {other:?}"),
    }

    let est_args = parse_args(["--mode", "estimate-dir", "--package-dir", &c172_dir]).unwrap();
    assert_eq!(est_args.mode, CliMode::EstimateDir);
    match execute_cli(&est_args).unwrap() {
        CliOutcome::DesignEstimate(rep) => {
            assert_eq!(rep.package_id, "c172s");
            assert!(rep.service_ceiling_ft > 10_000.0);
            assert_eq!(rep.payload_range.len(), 3);
            assert_eq!(rep.mission_profiles.len(), 4);
        }
        other => panic!("expected DesignEstimate outcome, got {other:?}"),
    }

    let trade_args = parse_args(["--mode", "trade-c172"]).unwrap();
    assert_eq!(trade_args.mode, CliMode::TradeC172);
    match execute_cli(&trade_args).unwrap() {
        CliOutcome::TradeStudy(rep) => {
            assert_eq!(rep.candidates.len(), 4);
        }
        other => panic!("expected TradeStudy outcome, got {other:?}"),
    }
}

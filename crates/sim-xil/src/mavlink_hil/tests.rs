use glam::DVec3;
use sim_atmosphere::{AtmosphereField, StandardAtmosphere};
use sim_core::{
    ExternalNodeBridge, HitlSyncMode, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState,
};
use sim_geo::{Geodetic, UtcTime};

use super::*;
use crate::{STANDARD_GRAVITY_M_S2, XilStimulus};

fn sample_stimulus() -> XilStimulus {
    let geo = Geodetic {
        lat_rad: 39.158_f64.to_radians(),
        lon_rad: (-75.524_f64).to_radians(),
        height_m: 300.0,
    };
    let vehicle = VehicleState::level_flight(&geo, 0.0, 55.0);
    let atm = StandardAtmosphere
        .sample(&geo, UtcTime::from_unix_seconds(0.0))
        .unwrap();
    XilStimulus::from_state_and_atmosphere(
        vehicle,
        DVec3::new(0.0, 0.0, -STANDARD_GRAVITY_M_S2),
        &atm,
    )
    .unwrap()
}

#[test]
fn x25_crc_matches_ieee_123456789_vector_before_extra() {
    let mut crc: u16 = 0xFFFF;
    for &b in b"123456789" {
        let mut tmp = b ^ ((crc & 0xFF) as u8);
        tmp ^= tmp << 4;
        let w = u16::from(tmp);
        crc = (crc >> 8) ^ (w << 8) ^ (w << 3) ^ (w >> 4);
    }
    assert_eq!(crc, 0x6F91);
    assert_eq!(x25_crc(b"123456789", 0), x25_crc(b"123456789", 0));
}

#[test]
fn hil_actuator_controls_round_trips_with_trailing_zero_trimming() {
    let mut controls = [0.0_f32; 16];
    controls[0] = -0.25;
    controls[1] = 0.5;
    controls[2] = 0.125;
    controls[3] = 0.75;
    let cmd = HilActuatorControls {
        time_usec: 125_000,
        controls,
        mode: 0, // zero trailing mode exercises MAVLink v2 zero-trimming!
        flags: 1,
    };

    let mut wire = [0_u8; 128];
    let n = encode_hil_actuator_controls(7, 1, 1, &cmd, &mut wire).unwrap();
    // Trailing zeros in controls[4..16] and mode=0 trim the 81-byte payload down to 32 bytes.
    assert_eq!(n, 10 + 32 + 2);

    let mut reader = MavlinkHilReader::new();
    reader.push_bytes(&wire[..n]);
    assert_eq!(reader.crc_errors(), 0);
    let decoded = match reader.pop_frame().unwrap() {
        MavlinkHilFrame::ActuatorControls { controls, .. } => controls,
        other => panic!("unexpected frame: {other:?}"),
    };
    assert_eq!(decoded, cmd);
    assert!(!decoded.is_armed());
    // Disarmed clamps throttle to 0 by default.
    let mapped = decoded.to_control_inputs(&HilControlMapping::default());
    assert_eq!(mapped.throttle, 0.0);
    assert!((mapped.roll + 0.25).abs() < 1.0e-6);
}

#[test]
fn reader_resynchronizes_after_corrupt_bytes_and_bad_crc() {
    let mut wire_hb = [0_u8; 64];
    let hb_len = encode_heartbeat(1, 1, 1, true, &mut wire_hb).unwrap();

    let mut bad_frame = wire_hb[..hb_len].to_vec();
    bad_frame[hb_len - 1] ^= 0xFF;

    let cmd = HilActuatorControls {
        time_usec: 500_000,
        controls: [0.4; 16],
        mode: MAV_MODE_FLAG_SAFETY_ARMED,
        flags: 0,
    };
    let mut wire_act = [0_u8; 128];
    let act_len = encode_hil_actuator_controls(2, 1, 1, &cmd, &mut wire_act).unwrap();

    let mut stream = vec![0x00, 0xFD, 0x05, 0x99];
    stream.extend_from_slice(&bad_frame);
    stream.extend_from_slice(&wire_hb[..hb_len]);
    stream.extend_from_slice(&wire_act[..act_len]);

    let mut reader = MavlinkHilReader::new();
    // Push byte-by-byte to prove arbitrary TCP chunk boundaries never drop a frame.
    for b in &stream {
        reader.push_bytes(std::slice::from_ref(b));
    }
    assert_eq!(reader.crc_errors(), 1);
    assert_eq!(
        reader.pop_frame(),
        Some(MavlinkHilFrame::Heartbeat {
            system_id: 1,
            component_id: 1,
            armed: true,
        })
    );
    match reader.pop_frame() {
        Some(MavlinkHilFrame::ActuatorControls { controls, .. }) => {
            assert!(controls.is_armed());
            let inputs = controls.to_control_inputs(&HilControlMapping::default());
            assert!((inputs.throttle - 0.4).abs() < 1.0e-6);
        }
        other => panic!("expected actuator controls, got {other:?}"),
    }
}

#[test]
fn sensor_gps_and_state_quaternion_samples_encode_valid_frames() {
    let stim = sample_stimulus();
    let sensor = HilSensorSample::from_stimulus(4_000, &stim, HIL_SENSOR_FIELDS_ALL).unwrap();
    assert!((sensor.accel_frd_m_s2[2] + 9.806_65).abs() < 1.0e-3);
    assert!(sensor.abs_pressure_hpa > 950.0 && sensor.abs_pressure_hpa < 1_015.0);

    let gps = HilGpsSample::from_stimulus(4_000, &stim).unwrap();
    assert_eq!(gps.fix_type, 3);
    assert!((gps.vel_ned_cm_s[0] - 5_500).abs() <= 2);

    let truth = HilStateQuaternionSample::from_stimulus(4_000, &stim).unwrap();
    assert!((truth.attitude_quat_wxyz[0] - 1.0).abs() < 1.0e-4);
    assert!(truth.true_airspeed_cm_s > 5_400);

    let mut buf = [0_u8; 128];
    let n_sensor = encode_hil_sensor(0, 1, 195, &sensor, &mut buf).unwrap();
    let crc = u16::from_le_bytes([buf[n_sensor - 2], buf[n_sensor - 1]]);
    assert_eq!(crc, x25_crc(&buf[1..n_sensor - 2], CRC_EXTRA_HIL_SENSOR));

    let n_gps = encode_hil_gps(1, 1, 195, &gps, &mut buf).unwrap();
    let crc_gps = u16::from_le_bytes([buf[n_gps - 2], buf[n_gps - 1]]);
    assert_eq!(crc_gps, x25_crc(&buf[1..n_gps - 2], CRC_EXTRA_HIL_GPS));

    let n_q = encode_hil_state_quaternion(2, 1, 195, &truth, &mut buf).unwrap();
    let crc_q = u16::from_le_bytes([buf[n_q - 2], buf[n_q - 1]]);
    assert_eq!(
        crc_q,
        x25_crc(&buf[1..n_q - 2], CRC_EXTRA_HIL_STATE_QUATERNION)
    );
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn tcp_endpoint_completes_lockstep_exchange() {
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::thread;

    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();

    let fc_thread = thread::spawn(move || {
        let (mut sock, _) = listener.accept().unwrap();
        let mut rx = [0_u8; 512];
        let n = sock.read(&mut rx).unwrap();
        assert!(n >= 21);
        assert_eq!(rx[0], MAVLINK_V2_MAGIC);

        let mut controls = [0.0_f32; 16];
        controls[0] = 0.2;
        controls[3] = 0.8;
        let reply = HilActuatorControls {
            time_usec: 10_000,
            controls,
            mode: MAV_MODE_FLAG_SAFETY_ARMED,
            flags: 0,
        };
        let mut tx = [0_u8; 128];
        let out_n = encode_hil_actuator_controls(0, 1, 1, &reply, &mut tx).unwrap();
        sock.write_all(&tx[..out_n]).unwrap();
    });

    let cfg = MavlinkHilConfig {
        send_state_quaternion: true,
        ..MavlinkHilConfig::default()
    };
    let tcp_ep = MavlinkHilTcpEndpoint::connect(addr, cfg).unwrap();
    let delay = TickSpan::from_ticks(TICKS_PER_SECOND / 250);
    let mut bridge = ExternalNodeBridge::new(
        tcp_ep,
        HitlSyncMode::LockstepBarrier {
            processing_delay: delay,
        },
    );
    let stim = sample_stimulus();
    bridge.stimulate(SimInstant::START, &stim).unwrap();
    assert!(bridge.poll(SimInstant::START).unwrap().is_empty());
    let delivered = bridge.poll(SimInstant::START.advanced(delay)).unwrap();
    assert_eq!(delivered.len(), 1);
    assert!(delivered[0].response.is_armed());
    assert!((delivered[0].response.controls[3] - 0.8).abs() < 1.0e-6);
    fc_thread.join().unwrap();
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn udp_endpoint_completes_exchange() {
    use sim_core::ExternalEndpoint;
    use std::net::UdpSocket;
    use std::thread;

    let cfg = MavlinkHilConfig::default();
    let fc_udp = UdpSocket::bind("127.0.0.1:0").unwrap();
    let fc_udp_addr = fc_udp.local_addr().unwrap();
    let mut udp_ep =
        MavlinkHilUdpEndpoint::bind("127.0.0.1:0".parse().unwrap(), fc_udp_addr, cfg).unwrap();
    let sim_udp_addr = udp_ep.local_addr().unwrap();

    let udp_peer = thread::spawn(move || {
        let mut rx = [0_u8; 512];
        let (n, _) = fc_udp.recv_from(&mut rx).unwrap();
        assert!(n > 12);
        let reply = HilActuatorControls {
            time_usec: 25_000,
            controls: [0.5; 16],
            mode: MAV_MODE_FLAG_SAFETY_ARMED,
            flags: 0,
        };
        let mut tx = [0_u8; 128];
        let out_n = encode_hil_actuator_controls(1, 1, 1, &reply, &mut tx).unwrap();
        fc_udp.send_to(&tx[..out_n], sim_udp_addr).unwrap();
    });

    let stim = sample_stimulus();
    let got = udp_ep.exchange_blocking(SimInstant::START, &stim).unwrap();
    assert_eq!(got.time_usec, 25_000);
    udp_peer.join().unwrap();
}

#[cfg(not(target_arch = "wasm32"))]
fn udp_pair(cfg: MavlinkHilConfig) -> (MavlinkHilUdpEndpoint, std::net::UdpSocket) {
    let fc = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    let ep = MavlinkHilUdpEndpoint::bind(
        "127.0.0.1:0".parse().unwrap(),
        fc.local_addr().unwrap(),
        cfg,
    )
    .unwrap();
    fc.connect(ep.local_addr().unwrap()).unwrap();
    (ep, fc)
}

#[cfg(not(target_arch = "wasm32"))]
fn send_controls(fc: &std::net::UdpSocket, time_usec: u64, throttle: f32) {
    let mut controls = [0.0_f32; 16];
    controls[3] = throttle;
    let reply = HilActuatorControls {
        time_usec,
        controls,
        mode: MAV_MODE_FLAG_SAFETY_ARMED,
        flags: 0,
    };
    let mut tx = [0_u8; 128];
    let n = encode_hil_actuator_controls(0, 1, 1, &reply, &mut tx).unwrap();
    fc.send(&tx[..n]).unwrap();
}

#[cfg(not(target_arch = "wasm32"))]
fn receive_blocking(
    ep: &mut MavlinkHilUdpEndpoint,
) -> Option<sim_core::StampedResponse<HilActuatorControls>> {
    use sim_core::ExternalEndpoint;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    let before = ep.link_stats();
    while std::time::Instant::now() < deadline {
        if let Some(got) = ep.try_receive().unwrap() {
            return Some(got);
        }
        let stats = ep.link_stats();
        if stats.stale_actuator_drops != before.stale_actuator_drops {
            return None;
        }
        std::thread::yield_now();
    }
    panic!("no actuator frame arrived");
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn udp_endpoint_maps_a_day_long_px4_uptime_onto_the_send_instant() {
    use sim_core::ExternalEndpoint;
    let (mut ep, fc) = udp_pair(MavlinkHilConfig::default());
    let stim = sample_stimulus();
    let step = TickSpan::from_ticks(TICKS_PER_SECOND / 250);
    let t0 = SimInstant::START.advanced(step);
    ep.post(t0, &stim).unwrap();
    send_controls(&fc, 86_400_000_000, 0.6);
    let got = receive_blocking(&mut ep).unwrap();
    assert_eq!(got.deliver_at, t0);

    ep.post(t0.advanced(step), &stim).unwrap();
    send_controls(&fc, 86_400_000_000 + 4_000, 0.6);
    let got = receive_blocking(&mut ep).unwrap();
    assert_eq!(got.deliver_at, t0.advanced(step));
    let stats = ep.link_stats();
    assert_eq!(stats.sensor_frames_sent, 2);
    assert_eq!(stats.correlated_actuator_frames, 2);
    assert_eq!(stats.clock_resyncs, 0);
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn udp_endpoint_drops_a_stale_reply_and_keeps_the_fresh_one() {
    use sim_core::ExternalEndpoint;
    let (mut ep, fc) = udp_pair(MavlinkHilConfig {
        max_reply_age: Some(DEFAULT_MAX_REPLY_AGE),
        ..MavlinkHilConfig::default()
    });
    let stim = sample_stimulus();
    let ms =
        |n: u64| SimInstant::START.advanced(TickSpan::from_ticks(n * TICKS_PER_SECOND / 1_000));
    ep.post(ms(0), &stim).unwrap();
    send_controls(&fc, 5_000_000, 0.5);
    assert_eq!(receive_blocking(&mut ep).unwrap().deliver_at, ms(0));

    ep.post(ms(300), &stim).unwrap();
    send_controls(&fc, 5_000_000 + 100_000, 0.9);
    assert!(receive_blocking(&mut ep).is_none());
    send_controls(&fc, 5_000_000 + 296_000, 0.7);
    let fresh = receive_blocking(&mut ep).unwrap();
    assert_eq!(fresh.deliver_at, ms(296));
    assert!((fresh.response.controls[3] - 0.7).abs() < 1.0e-6);
    let stats = ep.link_stats();
    assert_eq!(stats.stale_actuator_drops, 1);
    assert_eq!(stats.correlated_actuator_frames, 2);
    assert_eq!(
        stats.last_round_trip_ticks,
        ms(300).duration_since(ms(0)).ticks()
    );
}

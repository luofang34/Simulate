use std::f64::consts::FRAC_PI_2;

use glam::DVec3;
use sim_atmosphere::{AtmosphereField, StandardAtmosphere};
use sim_core::{SimInstant, VehicleState};
use sim_geo::{Geodetic, UtcTime};

use super::*;
use crate::{STANDARD_GRAVITY_M_S2, XilStimulus};

fn stimulus_heading(heading_rad: f64, speed_m_s: f64) -> XilStimulus {
    let geo = Geodetic {
        lat_rad: 39.158_f64.to_radians(),
        lon_rad: (-75.524_f64).to_radians(),
        height_m: 120.0,
    };
    let vehicle = VehicleState::level_flight(&geo, heading_rad, speed_m_s);
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
fn fdm_packet_conventions_and_144_byte_round_trip() {
    // Heading due East (+90 deg yaw in NED => -90 deg yaw about +Z Up in NWU!).
    let stim = stimulus_heading(FRAC_PI_2, 12.0);
    let mut clock = BetaflightPacketClock::new();
    let t0 = clock.next_timestamp_s();
    let t1 = clock.next_timestamp_s();
    assert_eq!(t0, 0.0);
    assert!((t1 - 0.025).abs() < 1.0e-12);

    let pkt = BetaflightFdmPacket::from_stimulus(t1, &stim).unwrap();
    // Negated FLU specific force at rest/level is [0, 0, -g].
    assert!(pkt.imu_linear_acceleration_xyz[0].abs() < 1.0e-6);
    assert!(pkt.imu_linear_acceleration_xyz[1].abs() < 1.0e-6);
    assert!((pkt.imu_linear_acceleration_xyz[2] + STANDARD_GRAVITY_M_S2).abs() < 1.0e-6);

    // ENU velocity when flying East at 12 m/s: [12, 0, 0].
    assert!((pkt.velocity_xyz[0] - 12.0).abs() < 1.0e-6);
    assert!(pkt.velocity_xyz[1].abs() < 1.0e-6);
    assert!(pkt.velocity_xyz[2].abs() < 1.0e-6);

    // LLH ordering is [lon_deg, lat_deg, alt_m].
    assert!((pkt.position_llh[0] + 75.524).abs() < 1.0e-4);
    assert!((pkt.position_llh[1] - 39.158).abs() < 1.0e-4);
    assert!((pkt.position_llh[2] - 120.0).abs() < 1.0e-3);

    // FLU->NWU quaternion for 90 deg East heading: cos(45 deg), 0, 0, -sin(45 deg).
    let half = std::f64::consts::FRAC_PI_4;
    assert!((pkt.imu_orientation_quat[0] - half.cos()).abs() < 1.0e-6);
    assert!((pkt.imu_orientation_quat[3] + half.sin()).abs() < 1.0e-6);

    let wire = pkt.to_le_bytes();
    assert_eq!(wire.len(), BETAFLIGHT_FDM_PACKET_BYTES);
    let decoded = BetaflightFdmPacket::from_le_bytes(&wire).unwrap();
    assert_eq!(decoded, pkt);
}

#[test]
fn servo_packet_validates_bounds_and_channels() {
    let servo = BetaflightServoPacket {
        motors: [0.1, 0.4, 0.7, 1.0],
        motor_count: 4,
        all_motors: [
            0.1, 0.4, 0.7, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ],
    };
    let bytes = servo.to_quad_bytes();
    let parsed = BetaflightServoPacket::from_le_bytes(&bytes).unwrap();
    assert_eq!(parsed, servo);
    assert!(BetaflightServoPacket::from_le_bytes(&[0_u8; 12]).is_err());
}

#[test]
fn msp_v1_and_v2_stream_decoder_handles_typed_payloads_and_errors() {
    let rc = MspRawRc {
        channels_us: [
            1500, 1500, 1250, 1500, 1800, 1000, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
            1500, 1500,
        ],
    };
    let f1 = encode_msp_v1(MspDirection::ToFc, MSP_SET_RAW_RC as u8, &rc.to_bytes()).unwrap();

    let att = MspAttitude {
        roll_decideg: -125,
        pitch_decideg: 45,
        yaw_deg: 270,
    };
    let f2 = encode_msp_v2(MspDirection::FromFc, 0, MSP_ATTITUDE, &att.to_bytes()).unwrap();

    let imu = MspRawImu {
        acc: [0, 0, 512],
        gyro: [1, -2, 3],
        mag: [120, -40, 300],
    };
    let f3 = encode_msp_v2(MspDirection::FromFc, 0, MSP_RAW_IMU, &imu.to_bytes()).unwrap();

    let status = MspStatus {
        cycle_time_us: 125,
        i2c_errors: 0,
        active_sensors: 0x07,
        flight_mode_flags: 0x01,
        current_pid_profile: 0,
    };
    let alt = MspAltitude {
        estimated_alt_cm: 12_000,
        vario_cm_s: 150,
    };
    assert_eq!(MspStatus::from_bytes(&status.to_bytes()).unwrap(), status);
    assert_eq!(MspAltitude::from_bytes(&alt.to_bytes()).unwrap(), alt);

    let mut corrupt = f1.clone();
    let last = corrupt.len() - 1;
    corrupt[last] ^= 0x55;

    let mut stream = vec![0x99, b'$', b'Z'];
    stream.extend_from_slice(&corrupt);
    stream.extend_from_slice(&f1);
    stream.extend_from_slice(&f2);
    stream.extend_from_slice(&f3);

    let mut decoder = MspStreamDecoder::new();
    for chunk in stream.chunks(5) {
        decoder.push_bytes(chunk);
    }
    assert_eq!(decoder.checksum_errors(), 1);

    let got1 = decoder.pop_frame().unwrap();
    assert_eq!(got1.version, MspVersion::V1);
    assert_eq!(got1.command, MSP_SET_RAW_RC);
    assert_eq!(MspRawRc::from_bytes(&got1.payload).unwrap(), rc);

    let got2 = decoder.pop_frame().unwrap();
    assert_eq!(got2.version, MspVersion::V2);
    assert_eq!(got2.command, MSP_ATTITUDE);
    assert_eq!(MspAttitude::from_bytes(&got2.payload).unwrap(), att);

    let got3 = decoder.pop_frame().unwrap();
    assert_eq!(MspRawImu::from_bytes(&got3.payload).unwrap(), imu);
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn betaflight_udp_endpoint_exchanges_fdm_and_servo_packets() {
    use sim_core::ExternalEndpoint;
    use std::net::UdpSocket;
    use std::thread;

    let bf_fdm_rx = UdpSocket::bind("127.0.0.1:0").unwrap();
    let bf_fdm_addr = bf_fdm_rx.local_addr().unwrap();

    let mut ep = BetaflightUdpEndpoint::bind(
        "127.0.0.1:0".parse().unwrap(),
        bf_fdm_addr,
        BetaflightUdpConfig {
            use_fixed_packet_clock: true,
            ..BetaflightUdpConfig::default()
        },
    )
    .unwrap();
    let sim_servo_addr = ep.servo_local_addr().unwrap();

    let peer = thread::spawn(move || {
        let mut buf = [0_u8; 256];
        let (n, _) = bf_fdm_rx.recv_from(&mut buf).unwrap();
        let pkt = BetaflightFdmPacket::from_le_bytes(&buf[..n]).unwrap();
        assert_eq!(pkt.timestamp_s, 0.0);

        let reply = BetaflightServoPacket {
            motors: [0.35, 0.36, 0.37, 0.38],
            motor_count: 4,
            all_motors: [0.0; 16],
        };
        let tx = UdpSocket::bind("127.0.0.1:0").unwrap();
        tx.send_to(&reply.to_quad_bytes(), sim_servo_addr).unwrap();
    });

    let stim = stimulus_heading(0.0, 0.0);
    let servo = ep.exchange_blocking(SimInstant::START, &stim).unwrap();
    assert!((servo.motors[0] - 0.35).abs() < 1.0e-6);
    assert!((servo.motors[3] - 0.38).abs() < 1.0e-6);
    peer.join().unwrap();
}

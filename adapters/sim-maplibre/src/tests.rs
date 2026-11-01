mod gpu;

use flightbag::{RunwayDesignator, RunwayEndSpec};
use glam::{DQuat, DVec3};
use indicate_frames::Quat;
use indicate_instrument_state::{
    AirData, AircraftState, Attitude, EstimateQuality, FreshnessPolicy, Kinematics, PanelData,
    Stamped, ValidFlags, resolve,
};
use indicate_instrument_volumes::{CANONICAL_EXTENT, TerrainPatchAux};
use indicate_interaction::{
    GesturePhase, InteractionFrame, InteractionStamp, SpatialEvent, TargetId,
};
use maplibre::headless::ViewSlotId;
use mbd_core::{ComponentState, ComponentTree, Placement};
use sim_core::{Block, CameraMount, CameraPoseEcef, SimInstant, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, SensorIntrinsics};
use sim_indicate::{VOLUME_SCENE_BYTES, VolumeInstrument};
use world::{
    GeodeticPosition, Ned,
    surveillance::{
        AddressNamespace, DeliveryPath, EmitterCategory, IntegrityQuality, ObservationSource,
        SurveillanceStore, TrackKey, TrackObservation,
    },
};

use crate::{
    CompanionStreamSession, DEFAULT_FAR_CLIP_M, DEFAULT_NEAR_CLIP_M, DynamicEntity3D,
    XVS_DEFAULT_PERIOD, XvsSvsConfig, XvsSvsInput, XvsSvsMbdBlock, build_conformal_overlay,
    conformal_stamp_from_vehicle, decode_outbound_packet, encode_inbound_interaction_bytes,
    entities_from_surveillance_belief, external_view_from_pose, frustum_from_intrinsics,
    orientation_from_ned_body_quat, project_geodetic_to_screen, symmetric_frustum,
    xvs_svs_component_spec,
};

pub(super) fn sample_panel_data() -> PanelData {
    let state = AircraftState {
        attitude: Stamped {
            data: Some(Attitude {
                quat: Quat::IDENTITY,
                rates_rps: [0.0; 3],
            }),
            age_ms: Some(10.0),
        },
        kinematics: Stamped {
            data: Some(Kinematics {
                pos_ned_m: [0.0, 0.0, -600.0],
                vel_ned_mps: [55.0, 0.0, 0.0],
            }),
            age_ms: Some(10.0),
        },
        air: Stamped {
            data: Some(AirData {
                ias_mps: Some(55.0),
                baro_setting_hpa: Some(1013.25),
                tas_mps: Some(58.0),
            }),
            age_ms: Some(10.0),
        },
        quality: EstimateQuality::Good,
        valid: ValidFlags {
            attitude: true,
            rates: true,
            position: true,
            velocity_horizontal: true,
            velocity_vertical: true,
            ..ValidFlags::default()
        },
        ..AircraftState::default()
    };
    resolve(&state, &FreshnessPolicy::default())
}

pub(super) fn sample_vehicle_at(
    lat_deg: f64,
    lon_deg: f64,
    height_m: f64,
    ned_body: DQuat,
) -> VehicleState {
    let geo = Geodetic {
        lat_rad: lat_deg.to_radians(),
        lon_rad: lon_deg.to_radians(),
        height_m,
    };
    let pos_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&geo);
    let ned_to_ecef = LocalFrame::at(&geo).ned_to_ecef();
    let ecef_from_ned = DQuat::from_mat3(&ned_to_ecef);
    VehicleState {
        position_ecef: pos_ecef,
        velocity_ecef: ned_to_ecef * DVec3::new(60.0, 0.0, 0.0),
        attitude: (ecef_from_ned * ned_body).normalize(),
        angular_rate_body: DVec3::ZERO,
    }
}

#[test]
fn below_msl_and_zenith_attitudes_produce_valid_external_views() {
    let frustum = symmetric_frustum(
        55.0_f64.to_radians(),
        1280,
        720,
        DEFAULT_NEAR_CLIP_M,
        DEFAULT_FAR_CLIP_M,
    )
    .expect("frustum");

    for (lat, lon, alt_msl) in [(36.24, -116.82, -86.0), (52.3086, 4.7639, -3.4)] {
        let pitch_90 = DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2);
        let veh = sample_vehicle_at(lat, lon, alt_msl, pitch_90);
        let mount = CameraMount::forward_nose(DVec3::new(1.2, 0.0, -0.3));
        let stamp = conformal_stamp_from_vehicle(
            SimInstant::START,
            &veh,
            &mount,
            DVec3::ZERO,
            frustum,
            0.0,
        )
        .expect("below-MSL zenith pose valid");
        // At +90 deg vertical pitch, +1.2 m forward nose lever arm points straight Up (+1.2 m MSL).
        assert!((stamp.anchor.altitude_msl_m - (alt_msl + 1.2)).abs() < 1e-3);
    }

    let too_low = sample_vehicle_at(31.5, 35.5, -500.0, DQuat::IDENTITY);
    let pose = CameraPoseEcef {
        position_ecef: too_low.position_ecef,
        orientation: too_low.attitude,
    };
    assert!(external_view_from_pose(&pose, frustum, 0.0).is_err());
}

#[test]
fn ned_body_quaternion_and_surveillance_belief_mapping_preserve_6dof_and_masks() {
    let intrinsics = SensorIntrinsics {
        width: 800,
        height: 600,
        fx: 700.0,
        fy: 700.0,
        cx: 402.0,
        cy: 298.0,
    };
    let asym = frustum_from_intrinsics(&intrinsics, 0.5, 200_000.0).expect("asymmetric frustum");
    assert!((asym.left - 402.0 / 700.0).abs() < 1e-9);

    let q_ned = DQuat::from_rotation_z(90.0_f64.to_radians())
        * DQuat::from_rotation_y(30.0_f64.to_radians())
        * DQuat::from_rotation_x(45.0_f64.to_radians());
    let converted = orientation_from_ned_body_quat(q_ned).to_rotation_matrix3();
    let expected =
        maplibre::headless::Orientation3d::from_heading_pitch_roll_degrees(90.0, 30.0, 45.0)
            .to_rotation_matrix3();
    let c_cols: &[[f64; 3]; 3] = converted.as_ref();
    let e_cols: &[[f64; 3]; 3] = expected.as_ref();
    for (ca, ea) in c_cols.iter().zip(e_cols.iter()) {
        for (cv, ev) in ca.iter().zip(ea.iter()) {
            assert!((cv - ev).abs() < 1e-9, "matrix mismatch: {cv} vs {ev}");
        }
    }

    let otw_slot = ViewSlotId::from_raw(0);
    let chase_slot = ViewSlotId::from_raw(1);
    let own_geo = Geodetic {
        lat_rad: 39.0_f64.to_radians(),
        lon_rad: (-75.5_f64).to_radians(),
        height_m: 450.0,
    };
    let own = DynamicEntity3D::ownship_exterior(1, &own_geo, q_ned, 0.0, otw_slot, 11.0);
    assert!(!own.placement.view_mask.includes(otw_slot));
    assert!(own.placement.view_mask.includes(chase_slot));

    let mut store = SurveillanceStore::default();
    let own_key = TrackKey {
        namespace: AddressNamespace::Icao24,
        address: 0xABCDEF,
    };
    let traffic_key = TrackKey {
        namespace: AddressNamespace::Icao24,
        address: 0x123456,
    };
    store.set_ownship_keys(&[own_key]);
    for (key, lon_deg) in [(own_key, -75.50), (traffic_key, -75.48)] {
        let obs = TrackObservation {
            key,
            source: ObservationSource::Adsb1090,
            delivery: DeliveryPath::LocalReceiver,
            observed_at_us: 1_000_000,
            position: GeodeticPosition::wgs84(39.02, lon_deg, 500.0),
            velocity_ned_m_s: Ned {
                north_m: 55.0,
                east_m: 5.0,
                down_m: 0.0,
            },
            heading_rad: Some(0.1),
            callsign: None,
            category: EmitterCategory::Light,
            on_ground: false,
            quality: IntegrityQuality::default(),
        };
        store.ingest(obs, 1_000_000).expect("ingest");
    }
    assert_eq!(entities_from_surveillance_belief(&store, 0.0).len(), 1);
}

#[test]
fn conformal_projection_and_xvs_mbd_block_snapshot_restore() {
    let spec = xvs_svs_component_spec("avionics.xvs_lru", Placement::ROOT);
    let tree = ComponentTree::new([(spec, ComponentState::at_temperature(293.15))])
        .expect("valid mbd component spec");
    assert_eq!(tree.len(), 1);

    let frustum = symmetric_frustum(1.0, 800, 600, 0.5, 250_000.0).expect("frustum");
    let mut block = XvsSvsMbdBlock::new(XvsSvsConfig {
        period: XVS_DEFAULT_PERIOD,
        mount: CameraMount::forward_nose(DVec3::ZERO),
        frustum,
    });
    let input = XvsSvsInput {
        instant: SimInstant::START.advanced(XVS_DEFAULT_PERIOD),
        belief_vehicle: sample_vehicle_at(39.0, -75.5, 250.0, DQuat::IDENTITY),
        cg_body_m: DVec3::ZERO,
        geoid_undulation_m: -32.0,
        belief_terrain_ready: true,
        evs_video_available: true,
        evs_calibrated: true,
        surveillance_track_count: 2,
        active_notice_count: 1,
    };
    block.step(&input).expect("step 1");
    let out1 = block.output();
    let snap = block.snapshot();
    assert_eq!(
        out1.active_contract,
        indicate_instrument_descriptor::UnderlayContract::CombinedXvs
    );
    assert!((out1.taws_reference_altitude_m - 282.0).abs() < 0.05);

    let mut input2 = input;
    input2.evs_video_available = false;
    block.step(&input2).expect("step 2");
    assert_eq!(
        block.output().active_contract,
        indicate_instrument_descriptor::UnderlayContract::ConformalSvs3D
    );
    block.restore(&snap);
    assert_eq!(block.output().step_count, 1);

    let stamp = out1.stamp.expect("stamp");
    let rwy = RunwayEndSpec {
        designator: RunwayDesignator::parse("36").expect("rwy"),
        threshold: GeodeticPosition::wgs84(39.015, -75.5, 10.0),
        true_bearing_rad: 0.0,
        length_m: 1_800.0,
        width_m: 30.0,
    };
    let thr_geo = Geodetic {
        lat_rad: rwy.threshold.latitude_deg.to_radians(),
        lon_rad: rwy.threshold.longitude_deg.to_radians(),
        height_m: 10.0,
    };
    let (px, rev_z) =
        project_geodetic_to_screen(&stamp, &thr_geo, -32.0, [800, 600]).expect("visible ahead");
    assert!((px[0] - 400.0).abs() < 2.0);
    assert!(rev_z > 0.0 && rev_z < 1.0);
    let (mesh, probes) = build_conformal_overlay(&stamp, [800, 600], Some(&rwy), &[], None);
    assert!(!mesh.is_empty());
    assert_eq!(probes.len(), 1);
}

#[test]
fn companion_stream_session_roundtrips_outbound_and_inbound_gestures() {
    let mut session = CompanionStreamSession::new(VolumeInstrument::Terrain);
    let aux = TerrainPatchAux::encode_4x4(1_000.0, &[0.02_f32; 16]);
    let mut scene_buf = vec![0_u8; VOLUME_SCENE_BYTES];
    let panel = sample_panel_data();
    let rendered = sim_indicate::emit_volume_scene(
        VolumeInstrument::Terrain,
        &panel,
        session.view_pose(),
        None,
        &[indicate_instrument_descriptor::AuxEntry {
            id: indicate_instrument_descriptor::AuxChannelId::TERRAIN_PATCH_3D,
            payload: &aux,
        }],
        Some(CANONICAL_EXTENT),
        &mut scene_buf,
    )
    .expect("emit terrain volume");

    let pkt = session
        .encode_outbound_packet(&scene_buf[..rendered.scene_bytes], Some(&aux))
        .expect("encode packet");
    let decoded = decode_outbound_packet(&pkt, None)
        .expect("decode")
        .expect("fresh");
    assert_eq!(decoded.sequence, 1);
    assert_eq!(decoded.scene_bytes.len(), rendered.scene_bytes);
    assert!(
        decode_outbound_packet(&pkt, Some(1))
            .expect("stale")
            .is_none()
    );

    let gesture = InteractionFrame {
        stamp: InteractionStamp {
            device_id: 7,
            sequence: 10,
            timestamp_us: 50_000,
        },
        event: SpatialEvent::Transform3D {
            phase: GesturePhase::Changed,
            pivot_local: [0.0, 0.0, 0.0],
            translation_local: [0.03, -0.02, 0.0],
            rotation: Quat::IDENTITY,
            scale: 1.25,
        },
    };
    let wire = encode_inbound_interaction_bytes(&gesture).expect("encode gesture");
    let res = session
        .apply_inbound_interaction_bytes(&wire)
        .expect("apply")
        .expect("accepted");
    assert!((res.updated_pose.zoom_scale - 1.25).abs() < 1e-4);
    assert!(
        session
            .apply_inbound_interaction_bytes(&wire)
            .expect("dup")
            .is_none()
    );

    let tap = InteractionFrame {
        stamp: InteractionStamp {
            device_id: 7,
            sequence: 11,
            timestamp_us: 60_000,
        },
        event: SpatialEvent::Activate {
            target: TargetId(42),
            point_local: [0.1, 0.0, 0.05],
        },
    };
    let tap_wire = encode_inbound_interaction_bytes(&tap).expect("encode tap");
    let tap_res = session
        .apply_inbound_interaction_bytes(&tap_wire)
        .expect("apply tap")
        .expect("accepted tap");
    assert_eq!(tap_res.activated_target, Some(TargetId(42)));
}

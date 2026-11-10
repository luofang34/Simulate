use indicate_instrument_scene::volume::VolumeBandId;
use indicate_instrument_state::{
    AirData, AircraftState, Attitude, EstimateQuality, FreshnessPolicy, Kinematics, Quat, Stamped,
    ValidFlags, resolve,
};
use indicate_interaction::{
    GesturePhase, InteractionFrame, InteractionOutcome, InteractionStamp, SpatialEvent, TargetId,
    VIEW_POSE_BYTES, ViewPose3D, decode_interaction, decode_view_pose, encode_interaction,
    encode_view_pose,
};

use super::{VOLUME_SCENE_BYTES, VolumeInstrument, apply_volume_gesture, emit_volume_scene};

fn sample_panel_data() -> indicate_instrument_state::PanelData {
    let state = AircraftState {
        attitude: Stamped {
            data: Some(Attitude {
                quat: Quat::from_euler(0.05, 0.03, 1.2),
                rates_rps: [0.0, 0.0, 0.01],
            }),
            age_ms: Some(15.0),
        },
        kinematics: Stamped {
            data: Some(Kinematics {
                pos_ned_m: [120.0, -40.0, -1500.0],
                vel_ned_mps: [55.0, 5.0, -1.5],
            }),
            age_ms: Some(15.0),
        },
        air: Stamped {
            data: Some(AirData {
                ias_mps: Some(52.0),
                baro_setting_hpa: Some(1013.25),
                tas_mps: Some(58.0),
                vsi_mps: None,
            }),
            age_ms: Some(15.0),
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

#[test]
fn terrain_globe_and_orrery_volumes_emit_valid_band_isolated_scenes() {
    let data = sample_panel_data();
    let mut buf = vec![0_u8; VOLUME_SCENE_BYTES];

    for kind in [
        VolumeInstrument::Terrain,
        VolumeInstrument::Globe,
        VolumeInstrument::Orrery,
    ] {
        let rendered =
            emit_volume_scene(kind, &data, ViewPose3D::IDENTITY, None, &[], None, &mut buf);
        assert!(rendered.is_ok(), "{kind:?} failed: {rendered:?}");
        if let Ok(summary) = rendered {
            assert!(summary.scene_bytes > 16);
            let ann_bit = 1_u16 << (VolumeBandId::Annunciation as u8);
            assert_ne!(summary.report.present & ann_bit, 0);
        }
    }
}

#[test]
fn spatial_gestures_evolve_view_pose_and_round_trip_wire_bytes() {
    let data = sample_panel_data();
    let mut pose = ViewPose3D::IDENTITY;

    let pinch = SpatialEvent::Transform3D {
        phase: GesturePhase::Changed,
        pivot_local: [0.0, 0.0, 0.0],
        translation_local: [0.04, 0.0, -0.02],
        rotation: Quat::from_euler(0.0, 0.1, 0.2),
        scale: 1.5,
    };
    let out = apply_volume_gesture(VolumeInstrument::Orrery, &mut pose, &pinch);
    assert_eq!(out, Ok(InteractionOutcome::ViewUpdated));
    assert!((pose.zoom_scale - 1.5).abs() < 1.0e-5);

    let scrub = SpatialEvent::ScrubTime { offset_s: 600.0 };
    let scrub_out = apply_volume_gesture(VolumeInstrument::Orrery, &mut pose, &scrub);
    assert_eq!(scrub_out, Ok(InteractionOutcome::ViewUpdated));
    assert!((pose.time_offset_s - 600.0).abs() < 1.0e-6);

    let mut pose_wire = [0_u8; VIEW_POSE_BYTES];
    assert!(encode_view_pose(&pose, &mut pose_wire).is_ok());
    assert_eq!(decode_view_pose(&pose_wire), Ok(pose));

    let frame = InteractionFrame {
        stamp: InteractionStamp {
            device_id: 42,
            sequence: 9,
            timestamp_us: 250_000,
        },
        event: SpatialEvent::Activate {
            target: TargetId(1),
            point_local: [0.0, 0.05, 0.0],
        },
    };
    let mut event_wire = [0_u8; 72];
    let len = encode_interaction(&frame, &mut event_wire).unwrap_or(0);
    let slice = event_wire.get(..len).unwrap_or_default();
    assert_eq!(decode_interaction(slice), Ok(frame));

    let mut scene_a = vec![0_u8; VOLUME_SCENE_BYTES];
    let mut scene_b = vec![0_u8; VOLUME_SCENE_BYTES];
    let res_a = emit_volume_scene(
        VolumeInstrument::Orrery,
        &data,
        ViewPose3D::IDENTITY,
        None,
        &[],
        None,
        &mut scene_a,
    );
    let res_b = emit_volume_scene(
        VolumeInstrument::Orrery,
        &data,
        pose,
        None,
        &[],
        None,
        &mut scene_b,
    );
    assert!(res_a.is_ok() && res_b.is_ok());
    if let (Ok(a), Ok(b)) = (res_a, res_b) {
        assert_ne!(
            scene_a.get(..a.scene_bytes),
            scene_b.get(..b.scene_bytes),
            "orrery scene must reflect updated ViewPose3D zoom/orbit/time_offset_s"
        );
    }
}

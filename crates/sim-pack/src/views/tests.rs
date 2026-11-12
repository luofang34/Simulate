use glam::DVec3;
use sim_core::{AirframeVisibility, VehicleState, VirtualCameraPlacement};
use sim_geo::Geodetic;

use super::*;
use crate::{builtin_c172_package, builtin_p38g_package};

const P38_EYE_FRD_M: DVec3 = DVec3::new(0.42, 0.0, -0.68);

fn parked() -> VehicleState {
    VehicleState::level_flight(&Geodetic::from_degrees(39.12, -75.47, 30.0), 0.7, 0.0)
}

#[test]
fn p38g_declares_every_requested_view() {
    let views = builtin_p38g_package().unwrap().camera_view_set();
    let ids: Vec<&str> = views.views.iter().map(|v| v.id.as_str()).collect();
    for id in [
        "chase",
        "wingman",
        "flyby_front",
        "side_profile",
        "top_down",
        "ground_spotter",
        "wingtip_left",
        "wingtip_right",
        "cockpit_forward",
        "cockpit_gunsight",
        "cockpit_left_engine",
        "cockpit_right_engine",
        "cockpit_over_shoulder",
        "cockpit_panel",
    ] {
        assert!(ids.contains(&id), "P-38G lacks {id}: {ids:?}");
    }
    // A 52 ft span gets a chase camera further back than a light single's.
    let (back, up) = views.chase_offsets_m().unwrap();
    let span = builtin_p38g_package().unwrap().jsbsim_wingspan_m().unwrap();
    assert!((span - 15.85).abs() < 0.01, "{span}");
    assert!(back > 22.0 && back / span > 1.5 && up > 7.0, "{back} {up}");
}

#[test]
fn c172_declares_cockpit_looks_including_the_strut_window() {
    let views = builtin_c172_package().unwrap().camera_view_set();
    let strut = views
        .views
        .iter()
        .find(|v| v.id == "cockpit_left_strut")
        .unwrap();
    assert_eq!(strut.airframe, ViewAirframe::CabinWindow);
    assert_eq!(views.chase_offsets_m(), Some((22.0, 6.5)));
}

#[test]
fn every_builtin_view_resolves_to_a_usable_camera() {
    for package in [
        builtin_c172_package().unwrap(),
        crate::builtin_c172p_package().unwrap(),
        builtin_p38g_package().unwrap(),
    ] {
        for spec in package.camera_view_set().virtual_cameras(P38_EYE_FRD_M) {
            let (pose, intrinsics) = spec.pose_and_intrinsics(&parked(), DVec3::ZERO).unwrap();
            assert!(
                pose.position_ecef.is_finite() && intrinsics.is_usable(),
                "{}",
                spec.id
            );
        }
    }
}

#[test]
fn look_at_views_aim_through_their_target_and_cockpit_looks_sit_at_the_eye() {
    let set = builtin_p38g_package().unwrap().camera_view_set();
    let state = parked();
    for (view, spec) in set.views.iter().zip(set.virtual_cameras(P38_EYE_FRD_M)) {
        let (pose, _) = spec.pose_and_intrinsics(&state, DVec3::ZERO).unwrap();
        let axis = pose.orientation * DVec3::NEG_Z;
        let to_body = |frd: [f64; 3]| state.position_ecef + state.attitude * DVec3::from(frd);
        match view.placement {
            ViewPlacement::LookAt { target_frd_m, .. } => {
                let to_target = (to_body(target_frd_m) - pose.position_ecef).normalize();
                assert!(axis.dot(to_target) > 0.9999, "{}", view.id);
            }
            ViewPlacement::CockpitLook {
                eye_shift_frd_m, ..
            } => {
                let eye = to_body((P38_EYE_FRD_M + DVec3::from(eye_shift_frd_m)).to_array());
                assert!(pose.position_ecef.distance(eye) < 1e-6, "{}", view.id);
                assert_eq!(spec.airframe, AirframeVisibility::PilotEye, "{}", view.id);
            }
            ViewPlacement::Chase { .. } | ViewPlacement::Spotter { .. } => {}
        }
    }
}

#[test]
fn derived_views_scale_with_span_and_keep_a_light_singles_chase() {
    let single = CameraViewSet::derived(11.0);
    let (back, up) = single.chase_offsets_m().unwrap();
    assert!((back - 22.0).abs() < 1e-9 && (up - 6.49).abs() < 1e-9);
    let (big_back, _) = CameraViewSet::derived(40.0).chase_offsets_m().unwrap();
    assert!((big_back - 80.0).abs() < 1e-9);
    let (drone_back, drone_up) = CameraViewSet::derived(0.7).chase_offsets_m().unwrap();
    assert!(drone_back >= 4.0 && drone_up > 1.0);
    assert_eq!(
        CameraViewSet::derived(f64::NAN),
        CameraViewSet::derived(11.0)
    );
    for spec in single.virtual_cameras(DVec3::ZERO) {
        assert!(spec.pose_and_intrinsics(&parked(), DVec3::ZERO).is_ok());
    }
    assert!(
        single
            .virtual_cameras(DVec3::ZERO)
            .iter()
            .any(|s| matches!(s.placement, VirtualCameraPlacement::BodyMounted(_)))
    );
}

#[test]
fn wingspan_reads_feet_metres_and_refuses_other_units() {
    assert_eq!(
        jsbsim_wingspan_m(r#"<wingspan unit="FT"> 10.0 </wingspan>"#),
        Some(3.048)
    );
    assert_eq!(
        jsbsim_wingspan_m(r#"<wingspan unit="M">12.5</wingspan>"#),
        Some(12.5)
    );
    assert_eq!(
        jsbsim_wingspan_m(r#"<wingspan unit="IN">12</wingspan>"#),
        None
    );
    assert_eq!(jsbsim_wingspan_m("<metrics/>"), None);
    let c172 = builtin_c172_package().unwrap().jsbsim_wingspan_m().unwrap();
    assert!((c172 - 11.0).abs() < 0.01, "{c172}");
}

#[test]
fn malformed_view_sets_are_refused() {
    let view = r#"{"id":"a","title":"A","fov_y_deg":40.0,"airframe":"external",
        "placement":{"kind":"chase","back_m":10.0,"up_m":3.0}}"#;
    let doc = |views: &str| format!(r#"{{"schema_version":1,"views":[{views}]}}"#);
    assert!(CameraViewSet::from_json_bytes(doc(view).as_bytes()).is_ok());
    for bad in [
        doc(&format!("{view},{view}")),
        doc(""),
        doc(&view.replace("40.0", "0.0")),
        doc(&view.replace("\"chase\"", "\"orbit\"")),
        doc(&view.replace("\"up_m\":3.0", "\"up_m\":3.0,\"pitch_deg\":1.0")),
        doc(
            r#"{"id":"b","title":"B","fov_y_deg":40.0,"airframe":"external",
            "placement":{"kind":"look_at","eye_frd_m":[0,0,-5],"target_frd_m":[0,0,0]}}"#,
        ),
        doc(
            r#"{"id":"c","title":"C","fov_y_deg":40.0,"airframe":"external",
            "placement":{"kind":"spotter","grid_step_m":100.0,"lateral_m":0.0,
            "height_m":1.0,"lead_time_s":0.1,"frame_span_m":10.0}}"#,
        ),
        doc(view).replace("\"schema_version\":1", "\"schema_version\":2"),
    ] {
        assert!(
            CameraViewSet::from_json_bytes(bad.as_bytes()).is_err(),
            "{bad}"
        );
    }
}

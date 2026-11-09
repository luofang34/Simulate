use std::time::Duration;

use glam::DQuat;
use maplibre::{
    coords::LatLon,
    headless::{AltitudeReference, Placement3d, ViewSlotId, ViewSlotMask},
};
use sim_core::AirframeArticulation;

use super::{c172, sample_kewr_otw_and_tower_stamps, sample_scenery_and_runtime_at, try_host_gpu};
use crate::{
    AircraftInstance, OwnshipVisualKind, SceneryEndpoints, SlotCameraRole, TruthSceneConfig,
    TruthSceneFrame, TruthSceneMap, orientation_from_ned_body_quat, truth_real_world_style,
};

fn differing_pixels(a: &[u8], b: &[u8]) -> usize {
    a.as_chunks::<4>()
        .0
        .iter()
        .zip(b.as_chunks::<4>().0)
        .filter(|(p, q)| p.iter().zip(q.iter()).any(|(x, y)| x.abs_diff(*y) > 24))
        .count()
}

/// Renders aircraft-free frames until the tower slot stops changing; returns the next
/// timestamp. Drapes bake over several frames, so comparing earlier would measure terrain
/// churn rather than the traffic.
fn settle_empty_frames(
    truth: &mut TruthSceneMap,
    tower: ViewSlotId,
    render: &impl Fn(&mut TruthSceneMap, &[AircraftInstance], u64),
) -> u64 {
    let mut ms = 0;
    let mut previous: Option<Vec<u8>> = None;
    for _ in 0..60 {
        render(truth, &[], ms);
        ms += 33;
        let frame = truth.read_slot_rgba(tower).expect("rgba");
        let settled = previous
            .as_ref()
            .is_some_and(|p| differing_pixels(p, &frame) == 0);
        previous = Some(frame);
        if settled && !truth.needs_redraw() {
            break;
        }
    }
    ms
}

/// A KEWR truth scene with a tower camera, and the OTW and tower stamps that view runway 22L.
struct KewrScene {
    truth: TruthSceneMap,
    tower: ViewSlotId,
    otw_stamp: crate::ConformalFrameStamp,
    tower_stamp: crate::ConformalFrameStamp,
    threshold: LatLon,
}

fn kewr_scene() -> Option<KewrScene> {
    let gpu = try_host_gpu()?;
    // Sea-level ground keeps the KEWR tower eye and the traffic above the terrain.
    let (scenery, _, runtime) = sample_scenery_and_runtime_at(gpu, 0.0);
    let mut truth = TruthSceneMap::new(
        &runtime,
        truth_real_world_style(&SceneryEndpoints::catalog(0, 0)).expect("style"),
        TruthSceneConfig {
            otw_size: [128, 72],
            stereo_otw: false,
            otw_depth_readout: false,
            sensor_size: [96, 72],
            request_overscan: 1.1,
        },
        c172(),
    )
    .expect("truth map");
    let _supplied = truth.supply_dem_synchronously(&scenery).expect("dem");
    let tower = truth.register_virtual_camera(96, 72, true).expect("tower");
    let kewr = sim_scenery::OsmAerodromeDiagram::builtin(sim_scenery::AerodromeId::NewarkKewr)
        .expect("kewr");
    let rwy = kewr.primary_runway_end().expect("22L");
    let (otw_stamp, tower_stamp) = sample_kewr_otw_and_tower_stamps(&kewr, rwy);
    Some(KewrScene {
        truth,
        tower,
        otw_stamp,
        tower_stamp,
        threshold: LatLon::new(rwy.threshold.latitude_deg, rwy.threshold.longitude_deg),
    })
}

fn aircraft_over(threshold: LatLon, view_mask: ViewSlotMask, ownship: bool) -> AircraftInstance {
    AircraftInstance {
        placement: Placement3d {
            position: threshold,
            altitude_meters: 120.0,
            altitude_reference: AltitudeReference::AboveMsl,
            orientation: orientation_from_ned_body_quat(DQuat::IDENTITY),
            view_mask,
        },
        articulation: AirframeArticulation::default(),
        ownship,
        ground_height_m: None,
    }
}

fn render_kewr(
    scene: &KewrScene,
    role: SlotCameraRole,
) -> impl Fn(&mut TruthSceneMap, &[AircraftInstance], u64) + use<> {
    let (otw_stamp, tower, tower_stamp) = (scene.otw_stamp, scene.tower, scene.tower_stamp);
    move |truth: &mut TruthSceneMap, aircraft: &[AircraftInstance], ms: u64| {
        truth
            .render_frame(TruthSceneFrame {
                timestamp: Duration::from_millis(ms),
                otw_stamp: Some(otw_stamp),
                stereo_right_stamp: None,
                sensor_stamp: None,
                extra_camera_slots: &[(tower, tower_stamp)],
                entities: &[],
                aircraft,
                ownship_camera_roles: &[(tower, role)],
                polar_runway: None,
            })
            .expect("frame");
    }
}

#[test]
fn a_tracked_aircraft_appears_in_its_own_slot_only() {
    let Some(mut scene) = kewr_scene() else {
        return;
    };
    let tower = scene.tower;
    let traffic = aircraft_over(
        scene.threshold,
        ViewSlotMask::ALL.without(scene.truth.otw_slot()),
        false,
    );
    let render = render_kewr(&scene, SlotCameraRole::BellyNadir);
    let truth = &mut scene.truth;
    let read = |truth: &TruthSceneMap, slot| truth.read_slot_rgba(slot).expect("rgba");
    let ms = settle_empty_frames(truth, tower, &render);
    let (otw_before, tower_before) = (read(truth, truth.otw_slot()), read(truth, tower));
    render(truth, &[], ms);
    let tower_control = read(truth, tower);
    assert!(
        differing_pixels(&tower_before, &tower_control) < 96 * 72 / 200,
        "an aircraft-absent control frame matches the settled frame"
    );
    render(truth, &[traffic], ms + 33);
    let (otw_after, tower_after) = (read(truth, truth.otw_slot()), read(truth, tower));
    // A side-on C172 (8 m long) in a 12 m-tall frame covers roughly 400 of 6912 pixels.
    assert!(
        differing_pixels(&tower_control, &tower_after) > 96 * 72 / 40,
        "the tower camera sees the C172"
    );
    assert!(
        differing_pixels(&otw_before, &otw_after) < 128 * 72 / 200,
        "the masked-out slot does not"
    );
}

/// The tower camera sees the ownship airframe the scene selects, and nothing once hidden;
/// a traffic C172 is unaffected by the selection.
#[test]
fn ownship_draws_the_selected_airframe_and_nothing_when_hidden() {
    let Some(mut scene) = kewr_scene() else {
        return;
    };
    let tower = scene.tower;
    let own = aircraft_over(scene.threshold, ViewSlotMask::ALL, true);
    let render = render_kewr(&scene, SlotCameraRole::External);
    let truth = &mut scene.truth;
    let ms = settle_empty_frames(truth, tower, &render);
    let control = truth.read_slot_rgba(tower).expect("control");
    let frame_with = |truth: &mut TruthSceneMap, kind, ms| {
        truth.set_ownship_visual(kind).expect("select airframe");
        assert_eq!(truth.ownship_visual(), kind);
        render(truth, &[own], ms);
        truth.read_slot_rgba(tower).expect("rgba")
    };
    let c172 = frame_with(truth, OwnshipVisualKind::C172, ms);
    let quad = frame_with(truth, OwnshipVisualKind::QuadrotorX500, ms + 33);
    let hidden = frame_with(truth, OwnshipVisualKind::Hidden, ms + 66);
    let c172_again = frame_with(truth, OwnshipVisualKind::C172, ms + 99);
    assert!(
        differing_pixels(&control, &c172) > 96 * 72 / 40,
        "C172 drawn"
    );
    assert!(differing_pixels(&control, &quad) > 0, "quadrotor drawn");
    assert!(
        differing_pixels(&c172, &quad) > 96 * 72 / 80,
        "the quadrotor is not a C172"
    );
    assert!(
        differing_pixels(&control, &hidden) < 96 * 72 / 200,
        "a hidden ownship leaves the scene empty"
    );
    assert!(
        differing_pixels(&c172, &c172_again) < 96 * 72 / 200,
        "switching back draws the same C172"
    );
}

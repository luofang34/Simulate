//! Camera cycling, global airports, vertical datum, and traffic heights.

use glam::{DMat3, DQuat, DVec3};
use sim_core::{SimInstant, VehicleState, VirtualCameraPlacement};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, UtcTime, egm96_undulation_m};
use sim_maplibre::ViewSlotMask;
use sim_scenario::{OperatorViewportMode, ScenarioProfile};
use sim_scenery::{AerodromeId, AirportDatabase, CatalogVerticalDatum};

use super::SupervisorRig;
use crate::camera_view::camera_from_virtual_spec_on;

fn idx(code: &str) -> sim_scenery::AirportIdx {
    AirportDatabase::builtin().by_code(code).unwrap().0
}

#[test]
fn six_presses_visit_every_camera_then_return_to_the_cockpit() {
    let mut rig = SupervisorRig::new();
    rig.set_viewport_mode(OperatorViewportMode::Cockpit);
    let mut ids = Vec::new();
    for press in 1..=5 {
        let spec = rig.cycle_virtual_camera(None);
        assert_eq!(
            rig.viewport_mode(),
            OperatorViewportMode::SensorAperture,
            "press {press}"
        );
        assert_eq!(rig.active_virtual_camera(None).id, spec.id);
        ids.push(spec.id);
    }
    ids.sort_unstable();
    ids.dedup();
    assert_eq!(ids.len(), 5, "five distinct cameras: {ids:?}");
    rig.cycle_virtual_camera(None);
    assert_eq!(rig.viewport_mode(), OperatorViewportMode::Cockpit);
    rig.cycle_virtual_camera(None);
    assert_eq!(rig.viewport_mode(), OperatorViewportMode::SensorAperture);
    assert_eq!(rig.active_virtual_camera(None).id, "nose_nav");
}

#[test]
fn the_cycle_walks_the_p38s_own_views_after_the_suite_with_their_airframe_roles() {
    use crate::camera_view::vehicle_camera_suite;
    use crate::vehicles::FreeFlightVehicle;
    use sim_core::AirframeVisibility;

    let own = &vehicle_camera_suite(FreeFlightVehicle::JsbsimP38g).cameras;
    let mut rig = SupervisorRig::new();
    rig.set_vehicle_cameras(own);
    rig.set_viewport_mode(OperatorViewportMode::Cockpit);
    let mut seen = Vec::new();
    for _ in 0..5_usize.saturating_add(own.len()) {
        seen.push(rig.cycle_virtual_camera(None));
    }
    assert_eq!(rig.viewport_mode(), OperatorViewportMode::SensorAperture);
    let tail: Vec<&str> = seen.iter().skip(5).map(|s| s.id.as_ref()).collect();
    let declared: Vec<&str> = own.iter().map(|s| s.id.as_ref()).collect();
    assert_eq!(tail, declared);
    let role = |id: &str| seen.iter().find(|s| s.id == id).map(|s| s.airframe);
    assert_eq!(role("wingtip_left"), Some(AirframeVisibility::External));
    assert_eq!(
        role("cockpit_over_shoulder"),
        Some(AirframeVisibility::PilotEye)
    );
    rig.cycle_virtual_camera(None);
    assert_eq!(rig.viewport_mode(), OperatorViewportMode::Cockpit);

    // A smaller suite pulls a selection past its end back to the first camera.
    rig.set_virtual_camera_index(5_usize.saturating_add(own.len()).saturating_sub(1));
    rig.set_vehicle_cameras(&[]);
    assert_eq!(rig.active_virtual_camera(None).id, "nose_nav");
}

#[test]
fn a_new_profile_starts_its_camera_suite_from_the_first_camera() {
    let mut rig = SupervisorRig::new();
    let kewr = Some(ScenarioProfile::AirportKewrToweredPattern);
    for _ in 0..5 {
        rig.cycle_virtual_camera(Some(ScenarioProfile::Airport47NUncontrolledPattern));
    }
    let first = rig.cycle_virtual_camera(kewr);
    assert_eq!(rig.viewport_mode(), OperatorViewportMode::SensorAperture);
    assert_eq!(first.id, "nose_nav");
}

#[test]
fn lunar_landing_cameras_stand_on_the_moon() {
    let rig = SupervisorRig::new();
    let profile = Some(ScenarioProfile::Apollo11LunarLanding);
    let (body, _) = rig.active_virtual_camera_on_body(profile);
    assert_eq!(body, BodyEllipsoid::MOON_IAU);

    let site = Geodetic::from_degrees(0.674, 23.473, 600.0);
    let frame = LocalFrame::at(&site);
    let lm = VehicleState {
        position_ecef: body.geodetic_to_fixed(&site),
        velocity_ecef: -frame.up() * 2.0,
        attitude: DQuat::from_mat3(&DMat3::from_cols(frame.up(), frame.east(), frame.north())),
        angular_rate_body: glam::DVec3::ZERO,
    };
    let mut rig = SupervisorRig::new();
    for _ in 0..5 {
        let spec = rig.cycle_virtual_camera(profile);
        let (body, active) = rig.active_virtual_camera_on_body(profile);
        assert_eq!(active.id, spec.id);
        let camera =
            camera_from_virtual_spec_on(body, &spec, &lm, DVec3::ZERO).expect("lunar camera pose");
        let position = camera.pose.position_ecef;
        assert!(
            position.is_finite() && camera.fov_y_rad.is_finite(),
            "{}",
            spec.id
        );
        let height = body.fixed_to_geodetic(position).height_m;
        assert!(height > 0.0, "{} at {height} m", spec.id);
    }
}

#[test]
fn global_airport_trackers_stand_at_that_airport() {
    let mut rig = SupervisorRig::new();
    rig.set_free_airport(idx("EGLL"), 0);
    assert_eq!(rig.effective_airport_code(None), "EGLL");
    let record = rig.active_airport_record(None).unwrap();
    let datum = rig.vertical_datum();
    let tower = record.tower_viewpoint(datum);
    rig.set_virtual_camera_index(3);
    let (body, spec) = rig.active_virtual_camera_on_body(None);
    assert_eq!(body, BodyEllipsoid::WGS84);
    let VirtualCameraPlacement::RangeLimitedAutoTrack { position, .. } = spec.placement else {
        panic!("tower camera is a tracker: {:?}", spec.placement);
    };
    assert_eq!(position, tower);

    rig.set_virtual_camera_index(4);
    let runway = rig.active_runway(None).copied().unwrap();
    let VirtualCameraPlacement::RangeLimitedAutoTrack { position, .. } =
        rig.active_virtual_camera(None).placement
    else {
        panic!("approach camera is a tracker");
    };
    let threshold = Geodetic::from_degrees(
        runway.threshold.latitude_deg,
        runway.threshold.longitude_deg,
        runway.threshold.ellipsoidal_height_m,
    );
    let apart = (BodyEllipsoid::WGS84.geodetic_to_fixed(&position)
        - BodyEllipsoid::WGS84.geodetic_to_fixed(&threshold))
    .length();
    assert!(
        apart < 150.0,
        "approach tripod {apart} m from the threshold"
    );
}

#[test]
fn runway_heights_follow_the_vertical_datum() {
    let mut rig = SupervisorRig::new();
    let kewr = Some(ScenarioProfile::AirportKewrToweredPattern);
    assert_eq!(rig.effective_airport_code(kewr), "KEWR");
    let msl = rig
        .active_runway_in_datum(kewr, CatalogVerticalDatum::EllipsoidWgs84)
        .unwrap();
    let orthometric = *rig.active_runway(kewr).unwrap();
    assert_eq!(msl.designator, orthometric.designator);
    let threshold = orthometric.threshold;
    let undulation = egm96_undulation_m(
        threshold.latitude_deg.to_radians(),
        threshold.longitude_deg.to_radians(),
    );
    assert!(undulation < -30.0, "KEWR undulation {undulation}");
    let shift = threshold.ellipsoidal_height_m - msl.threshold.ellipsoidal_height_m;
    assert!((shift - undulation).abs() < 1e-6, "{shift} vs {undulation}");

    rig.set_vertical_datum(CatalogVerticalDatum::EllipsoidWgs84);
    assert_eq!(rig.active_runway(kewr).copied(), Some(msl));
    assert!(
        rig.active_runway(Some(ScenarioProfile::Apollo11LunarLanding))
            .is_none()
    );
}

#[test]
fn picking_a_featured_aerodrome_selects_its_database_airport() {
    let mut rig = SupervisorRig::new();
    assert_eq!(rig.effective_airport_code(None), "KDOV");
    rig.set_free_airport(idx("RJTT"), 1);
    assert_eq!(rig.selected_runway_end(), 1);
    rig.set_free_aerodrome(AerodromeId::NewarkKewr);
    assert_eq!(rig.free_airport_idx(), Some(idx("KEWR")));
    assert_eq!(rig.selected_runway_end(), 0);
    rig.set_free_airport(idx("47N"), 0);
    assert_eq!(rig.effective_aerodrome(None), AerodromeId::CentralJersey47N);
    assert_eq!(rig.step_aerodrome(true), AerodromeId::NewarkKewr);
    assert_eq!(rig.effective_airport_code(None), "KEWR");
}

/// Scene entities carry orthometric MSL heights in `OrthometricMslEgm96` (so adding
/// `geoid_undulation_m` recovers exact WGS84 ellipsoidal height) and ellipsoidal heights in
/// `EllipsoidWgs84`, circulating around the airport at its true ellipsoidal height.
#[test]
fn traffic_entities_follow_the_vertical_datum_around_the_airport() {
    let mut rig = SupervisorRig::new();
    let kewr = Some(ScenarioProfile::AirportKewrToweredPattern);
    let record = rig.active_airport_record(kewr).unwrap();
    let arp = record.arp_geodetic(rig.vertical_datum());
    let ownship = BodyEllipsoid::WGS84.geodetic_to_fixed(&arp);
    rig.sync(
        ownship,
        SimInstant::START,
        UtcTime::from_unix_seconds(1_782_864_000.0),
        kewr,
    );
    let truth = rig.supervisor().truth_world();
    let tracks = truth.surveillance.tracks();
    let entities = rig.truth_entities(ViewSlotMask::ALL);
    assert_eq!(entities.len(), tracks.len());
    for (entity, track) in entities.iter().zip(tracks) {
        let undulation = egm96_undulation_m(
            track.position.latitude_deg.to_radians(),
            track.position.longitude_deg.to_radians(),
        );
        assert!(
            ((entity.placement.altitude_meters + undulation) - track.position.ellipsoidal_height_m)
                .abs()
                < 1e-9
        );
    }
    rig.set_vertical_datum(CatalogVerticalDatum::EllipsoidWgs84);
    for (entity, track) in rig
        .truth_entities(ViewSlotMask::ALL)
        .iter()
        .zip(rig.supervisor().truth_world().surveillance.tracks())
    {
        assert_eq!(
            entity.placement.altitude_meters,
            track.position.ellipsoidal_height_m
        );
    }
    // The pattern traffic flies 420 m over the field a few kilometres out; the distant
    // drone is the lower track.
    let pattern = rig
        .supervisor()
        .truth_world()
        .surveillance
        .tracks()
        .iter()
        .map(|track| track.position.ellipsoidal_height_m - arp.height_m)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        (pattern - 420.0).abs() < 5.0,
        "pattern traffic {pattern} m over the ARP"
    );
}

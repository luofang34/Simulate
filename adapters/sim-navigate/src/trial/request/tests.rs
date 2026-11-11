use super::super::epoch::parse_utc;

#[test]
fn an_iso_epoch_reads_as_unix_seconds() {
    let utc = parse_utc("2026-03-20T02:00:00Z").unwrap();
    assert_eq!(utc.unix_seconds(), 1_773_972_000.0);
    let fractional = parse_utc("1970-01-01T00:00:01.5Z").unwrap();
    assert_eq!(fractional.unix_seconds(), 1.5);
    assert!(parse_utc("2026-03-20 02:00:00").is_err());
    assert!(parse_utc("2026-13-20T02:00:00Z").is_err());
}

use crate::trial::{RoutePoint, SensorKind, TrialError, TrialRequest, request_from_route};

fn point(lat_deg: f64, lon_deg: f64, altitude_m: f64) -> RoutePoint {
    RoutePoint {
        ident: None,
        lat_deg,
        lon_deg,
        altitude_m,
        speed_m_s: None,
    }
}

fn route_request(defaults: Option<&str>) -> TrialRequest {
    let route = [
        point(39.0, -75.6, 1_000.0),
        point(39.0, -75.55, 1_000.0),
        point(39.03, -75.5, 1_100.0),
    ];
    request_from_route(&route, defaults).unwrap()
}

fn refusal(request: &TrialRequest) -> TrialError {
    request.validate().unwrap_err()
}

#[test]
fn a_route_completes_with_documented_defaults() {
    let request = route_request(None);
    request.validate().unwrap();
    assert_eq!(request.waypoints[1].ident, "WP1");
    assert_eq!(request.sensor_sets.len(), 7);
    let merged = route_request(Some(r#"{ "vehicle": { "max_climb_m_s": 8.0 } }"#));
    assert_eq!(merged.vehicle.max_climb_m_s, 8.0);
    assert_eq!(merged.vehicle.cruise_speed_m_s, 60.0);
    let waypoints = r#"{ "waypoints": [] }"#;
    assert!(request_from_route(&[point(0.0, 0.0, 100.0)], Some(waypoints)).is_err());
    assert!(request_from_route(&[], Some(r#"{ "vehicle": { "wings": 2 } }"#)).is_err());
}

#[test]
fn a_bad_waypoint_is_refused_by_row_and_field() {
    let field_of = |request: &TrialRequest| match refusal(request) {
        TrialError::InvalidWaypoint { row, field, .. } => (row, field),
        other => panic!("{other:?}"),
    };
    let mut request = route_request(None);
    request.waypoints[2].lat_deg = 95.0;
    assert_eq!(field_of(&request), (2, "lat_deg"));
    let mut request = route_request(None);
    request.waypoints[1].lon_deg = -190.0;
    assert_eq!(field_of(&request), (1, "lon_deg"));
    let mut request = route_request(None);
    request.waypoints[0].altitude_m = 30_000.0;
    assert_eq!(field_of(&request), (0, "altitude_m"));
    let mut request = route_request(None);
    request.waypoints[1].speed_m_s = Some(10.0);
    assert_eq!(field_of(&request), (1, "speed_m_s"));
    let mut request = route_request(None);
    request.waypoints[1].lat_deg = 39.0;
    request.waypoints[1].lon_deg = -75.5995;
    assert_eq!(field_of(&request), (1, "lat_deg"));
    let mut request = route_request(None);
    request.waypoints.truncate(1);
    assert!(matches!(
        refusal(&request),
        TrialError::InvalidRequest {
            field: "waypoints",
            ..
        }
    ));
}

#[test]
fn schedules_must_exist_match_their_sensor_and_name_one_set_means() {
    let declared = r#"{ "availability": [
        { "name": "jam", "sensor": "gnss", "unavailable": [{ "start_s": 10.0, "end_s": 20.0 }] },
        { "name": "fog", "sensor": "terrain_camera", "unavailable": [] }
    ] }"#;
    let mut request = route_request(Some(declared));
    request.validate().unwrap();
    let invalid_set =
        |request: &TrialRequest| matches!(refusal(request), TrialError::InvalidSet { .. });
    request.sensor_sets[2]
        .availability
        .insert(SensorKind::Gnss, "jam".into());
    request.validate().unwrap();
    request.sensor_sets[2]
        .availability
        .insert(SensorKind::Gnss, "fog".into());
    assert!(invalid_set(&request));
    request.sensor_sets[2]
        .availability
        .insert(SensorKind::Gnss, "rain".into());
    assert!(invalid_set(&request));
    let mut request = route_request(Some(declared));
    request.sensor_sets[1]
        .availability
        .insert(SensorKind::Gnss, "jam".into());
    assert!(invalid_set(&request));
    let mut request = route_request(Some(declared));
    request.availability[1].name = "jam".into();
    assert!(matches!(
        refusal(&request),
        TrialError::InvalidSchedule { .. }
    ));
    let mut request = route_request(Some(declared));
    request.availability[0].unavailable[0].end_s = 5.0;
    assert!(matches!(
        refusal(&request),
        TrialError::InvalidSchedule { .. }
    ));
}

#[test]
fn an_epoch_outside_the_ephemeris_is_refused_when_stars_fly() {
    let mut request = route_request(None);
    request.epoch_utc = "3500-01-01T00:00:00Z".into();
    assert!(matches!(
        refusal(&request),
        TrialError::InvalidRequest {
            field: "epoch_utc",
            ..
        }
    ));
    request
        .sensor_sets
        .retain(|set| !set.name.contains("stars"));
    request.validate().unwrap();
}

#[test]
fn the_terrain_camera_and_maps_are_checked_as_stated() {
    use crate::trial::{
        AssumedAccuracy, CameraBand, MapsSpec, OnboardMapSpec, PackageRef, TerrainCameraSpec,
    };
    let camera: TerrainCameraSpec = serde_json::from_str(
        r#"{ "band": "visible", "width": 800, "height": 600,
             "fx_px": 600.0, "fy_px": 600.0, "cx_px": 399.5, "cy_px": 299.5,
             "mount": { "offset_body_m": [1.2, 0.0, 0.4], "roll_deg": 0.0, "pitch_deg": -90.0, "yaw_deg": 0.0 },
             "period_s": 1.0, "exposure_s": 0.004,
             "radiometry": { "gain": 1.0, "offset_dn": 0.0, "gamma": 1.0, "noise_dn": 2.0 },
             "min_sun_elevation_deg": 5.0 }"#,
    )
    .unwrap();
    let mut request = route_request(None);
    request.terrain_camera = Some(camera);
    request.validate().unwrap();
    let mut lwir = request.clone();
    if let Some(camera) = lwir.terrain_camera.as_mut() {
        camera.band = CameraBand::Lwir;
    }
    assert!(lwir.validate().is_err());
    let package = PackageRef {
        store: "store".into(),
        package: "0".repeat(64),
    };
    request.maps = Some(MapsSpec {
        onboard: OnboardMapSpec {
            store: package.store.clone(),
            package: package.package.clone(),
            assumed_accuracy: Some(AssumedAccuracy {
                horizontal_m: -1.0,
                vertical_m: 2.0,
            }),
            max_match_pixels: None,
        },
        world: package,
        geoid: None,
    });
    assert!(matches!(
        refusal(&request),
        TrialError::InvalidRequest {
            field: "maps.onboard.assumed_accuracy",
            ..
        }
    ));
    let text = request.to_json().unwrap();
    assert_eq!(TrialRequest::from_json(&text).unwrap(), request);
}

#[test]
fn a_visual_set_needs_a_camera_maps_and_a_seed() {
    let mut request = route_request(None);
    request.sensor_sets[0].means.push(crate::NavMeans::Visual);
    assert!(matches!(
        refusal(&request),
        TrialError::InvalidRequest {
            field: "terrain_camera",
            ..
        }
    ));
}

use super::*;

#[test]
fn real_world_imagery_stops_at_esri_served_zoom() {
    let endpoints = crate::SceneryEndpoints::real_world();
    assert_eq!(endpoints.ortho_maxzoom, 18);
    assert!(
        endpoints
            .ortho_tiles
            .starts_with("https://services.arcgisonline.com/")
    );
    assert!(!endpoints.ortho_tiles.contains("blankTile"));
}

#[test]
fn non_earth_stamps_and_globe_ned_scale_with_moon_and_mars_bodies() {
    use crate::{
        LUNAR_RENDER_BODY, MARS_RENDER_BODY, conformal_stamp_from_pose_on, default_render_body,
        globe_ned_between_geodetic, project_geodetic_to_screen,
    };
    use sim_core::CameraPoseEcef;
    use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};

    assert_eq!(
        default_render_body(BodyEllipsoid::MOON_IAU),
        LUNAR_RENDER_BODY
    );
    assert_eq!(
        default_render_body(BodyEllipsoid::MARS_IAU),
        MARS_RENDER_BODY
    );

    let frustum = symmetric_frustum(50.0_f64.to_radians(), 800, 600, 0.5, 250_000.0).expect("f");
    // Hellas Planitia sits ~7.1 km below the Martian datum; Earth's -450 m floor would reject it,
    // whereas `MARS_RENDER_BODY` (-8,200 m floor) admits it cleanly.
    let hellas_eye = Geodetic::from_degrees(-42.4, 70.5, -7_100.0);
    let local = LocalFrame::at(&hellas_eye);
    let cam_pose = CameraPoseEcef {
        position_ecef: BodyEllipsoid::MARS_IAU.geodetic_to_fixed(&hellas_eye),
        orientation: DQuat::from_mat3(&DMat3::from_cols(local.east(), local.up(), -local.north())),
    };
    assert!(external_view_from_pose(&cam_pose, frustum, 0.0).is_err());
    let mars_stamp = conformal_stamp_from_pose_on(
        SimInstant::START,
        BodyEllipsoid::MARS_IAU,
        MARS_RENDER_BODY,
        &cam_pose,
        frustum,
        0.0,
    )
    .expect("Mars Hellas stamp");
    assert!((mars_stamp.anchor.altitude_msl_m + 7_100.0).abs() < 1e-6);

    let ahead = Geodetic::from_degrees(-42.39, 70.5, -7_100.0);
    assert!(project_geodetic_to_screen(&mars_stamp, &ahead, 0.0, [800, 600]).is_some());

    // 0.01 deg north on the Moon (~303.2 m) vs Earth (~1,111.9 m): ratio equals R_moon / R_earth.
    let moon_a = Geodetic::from_degrees(0.0, 23.47, 0.0);
    let moon_b = Geodetic::from_degrees(0.01, 23.47, 0.0);
    let ned_moon = globe_ned_between_geodetic(LUNAR_RENDER_BODY, &moon_a, 0.0, &moon_b, 0.0)
        .expect("moon ned");
    let ned_earth =
        globe_ned_between_geodetic(crate::RenderBody::EARTH, &moon_a, 0.0, &moon_b, 0.0)
            .expect("earth ned");
    let ratio = ned_moon.x / ned_earth.x;
    assert!(
        (ratio - LUNAR_RENDER_BODY.radius_meters / crate::RenderBody::EARTH.radius_meters).abs()
            < 1e-9
    );
}

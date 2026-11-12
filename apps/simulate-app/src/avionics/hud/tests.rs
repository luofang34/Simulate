use super::*;
mod conformal_alignment;
use crate::camera_view::{pilot_eye_mount, scenario_camera};
use flightbag::AuthoritySource;
use sim_core::{CameraPoseEcef, SimInstant};
use sim_indicate::{
    DisplayAxes, FeedConfig, FlightSample, SourceMonitor, StateFeed, VectorHudPanel,
};
use sim_maplibre::{
    DEFAULT_FAR_CLIP_M, DEFAULT_NEAR_CLIP_M, LUNAR_RENDER_BODY, MARS_RENDER_BODY,
    conformal_stamp_from_pose_on, project_geodetic_to_screen, symmetric_frustum,
};
use sim_scenery::{
    AerodromeId, AirportDatabase, CompactCode, OsmAerodromeDiagram, RunwaySurfaceKind,
};

#[test]
fn vector_hud_tessellates_at_window_and_retina_resolution() {
    let origin = Geodetic::from_degrees(39.13, -75.47, 450.0);
    let vehicle = VehicleState::level_flight(&origin, 0.05, 55.0);
    let cam = scenario_camera(pilot_eye_mount().pose(&vehicle, DVec3::ZERO));
    let diag = OsmAerodromeDiagram::builtin(AerodromeId::DoverKdov).expect("KDOV diagram");
    let rwy = diag
        .primary_runway_end()
        .map(LandingRunwayGeometry::from_runway_end_spec);
    let view = HudView {
        surface_size: [1280, 720],
        camera: cam,
        traffic: &[],
        airframe_attitude: vehicle.attitude,
        eye_body_lever_m: vehicle.attitude.inverse()
            * (cam.pose.position_ecef - vehicle.position_ecef),
        conformal_surface: ConformalSurface::earth_globe(),
        vertical_datum: Some(CatalogVerticalDatum::EllipsoidWgs84),
    };
    let c172_env = sim_pack::builtin_c172_package()
        .expect("C172 package")
        .resolved_airspeed_envelope();
    let ctx = hud_context(&vehicle, &view, rwy.as_ref(), c172_env.as_ref());

    let mut feed = StateFeed::new(FeedConfig {
        body: BodyEllipsoid::WGS84,
        axes: DisplayAxes::AEROSPACE,
        origin,
        origin_id: 1,
        altitude: sim_indicate::AltitudeSource::OriginRelative,
        heading_mode: sim_indicate::HeadingMode::EarthMagnetic,
        deviation_scale: sim_indicate::DeviationScaleProfile::GNSS_NAVIGATOR,
    });
    let sample = FlightSample::truth(SimInstant::START, vehicle);
    let monitored = SourceMonitor::new()
        .step(&mut feed, &sample)
        .expect("monitored frame");
    let mut panel = VectorHudPanel::new();
    for [w, h] in [[1280, 720], [2560, 1440], [3840, 2160], [3440, 1440]] {
        let verts = panel
            .tessellate(&monitored.panel, &ctx, Some(&monitored.alerts), w, h)
            .expect("vector HUD");
        assert!(!verts.is_empty(), "{w}x{h}");
        assert!(verts.len().is_multiple_of(3));
    }
    assert!(VectorHudPanel::design_frame(4000, 1000).is_err());
}

/// A 10 m east navigation error moves the conformal runway 10 m west on the glass, while
/// the glass itself stays where the physical airframe and window camera put it.
#[test]
fn navigation_error_shifts_the_runway_not_the_glass() {
    let origin = Geodetic::from_degrees(39.10, -75.50, 450.0);
    let truth = VehicleState::level_flight(&origin, 0.05, 55.0);
    let cam = scenario_camera(pilot_eye_mount().pose(&truth, DVec3::ZERO));
    let diag = OsmAerodromeDiagram::builtin(AerodromeId::DoverKdov).expect("KDOV diagram");
    let rwy = diag
        .primary_runway_end()
        .map(LandingRunwayGeometry::from_runway_end_spec);
    let east = LocalFrame::at(&origin).east();
    let estimated = VehicleState {
        position_ecef: truth.position_ecef + east * 10.0,
        attitude: (truth.attitude * DQuat::from_rotation_z(0.02)).normalize(),
        ..truth
    };
    let view = HudView {
        surface_size: [1280, 720],
        camera: cam,
        traffic: &[],
        airframe_attitude: truth.attitude,
        eye_body_lever_m: truth.attitude.inverse() * (cam.pose.position_ecef - truth.position_ecef),
        conformal_surface: ConformalSurface::earth_globe(),
        vertical_datum: Some(CatalogVerticalDatum::EllipsoidWgs84),
    };
    let ideal = hud_context(&truth, &view, rwy.as_ref(), None);
    let offset = hud_context(&estimated, &view, rwy.as_ref(), None);
    let (Some(a), Some(b)) = (ideal.approach, offset.approach) else {
        panic!("runway missing from the HUD context");
    };
    let shift = [0, 1, 2].map(|i| b.threshold_m[i] - a.threshold_m[i]);
    assert!((shift[1] + 10.0).abs() < 0.05, "east shift {shift:?}");
    assert!(shift[0].abs() < 0.05 && shift[2].abs() < 0.05, "{shift:?}");
    assert_eq!(ideal.projection, offset.projection);
    let level_glass = ideal.projection.value.body_from_view;
    assert!((level_glass.w.abs() - 1.0).abs() < 1e-3, "{level_glass:?}");
}

#[test]
fn polar_hud_runway_course_aligns_across_south_pole() {
    use flightbag::{RunwayDesignator, RunwayEndSpec};
    use world::GeodeticPosition;

    let thr_geo = Geodetic::from_degrees(-89.9975, 139.2729, 2_835.0);
    let thr_local = LocalFrame::at(&thr_geo);
    let course = 20.0_f64.to_radians();
    let rwy_dir_ecef = thr_local.north() * course.cos() + thr_local.east() * course.sin();
    // Eye 3 NM behind NZSP RWY 02T on the opposite side of 90S (lon ~ -40.7 deg).
    let eye_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&thr_geo) - rwy_dir_ecef * 5_556.0
        + thr_local.up() * 305.0;
    let spec = RunwayEndSpec {
        designator: RunwayDesignator::parse("02T").expect("02T"),
        threshold: GeodeticPosition::wgs84(-89.9975, 139.2729, 2_835.0),
        true_bearing_rad: course,
        length_m: 3_658.0,
        width_m: 46.0,
    };
    let rwy = LandingRunwayGeometry::from_runway_end_spec(&spec);
    let hud = hud_runway(
        eye_ecef,
        ConformalSurface::earth_globe(),
        Some(CatalogVerticalDatum::EllipsoidWgs84),
        &rwy,
    );
    let to_thr_course = hud.threshold_m[1]
        .atan2(hud.threshold_m[0])
        .rem_euclid(std::f32::consts::TAU);
    let diff = (hud.course_rad - to_thr_course + std::f32::consts::PI)
        .rem_euclid(std::f32::consts::TAU)
        - std::f32::consts::PI;
    assert!(
        diff.abs() < 0.005,
        "HUD runway course ({:.1} deg) must match approach vector ({:.1} deg) across 90S",
        hud.course_rad.to_degrees(),
        to_thr_course.to_degrees()
    );
}

#[test]
fn faa_sloped_and_displaced_runway_carries_exact_far_end_drop() {
    let (_, a47n) = AirportDatabase::builtin().by_code("47N").expect("47N");
    let datum = CatalogVerticalDatum::OrthometricMslEgm96;
    let rwy = a47n.landing_runway("07", datum).expect("RWY 07");
    let eye = Geodetic {
        height_m: rwy.threshold.height_m + 1.4,
        ..rwy.threshold
    };
    let hud = hud_runway(
        BodyEllipsoid::WGS84.geodetic_to_fixed(&eye),
        ConformalSurface::earth_globe(),
        Some(datum),
        &rwy,
    );
    assert!((hud.threshold_m[2] - 1.4).abs() < 0.02);
    assert!(
        hud.far_down_offset_m > 5.0,
        "expected downhill far_down_offset_m > 5 m, got {}",
        hud.far_down_offset_m
    );
    assert!((hud.glide_path_rad.to_degrees() - 3.4).abs() < 0.05);
}

#[test]
fn hud_runway_aligns_subpixel_with_maplibre_globe_under_banked_approach() {
    let datum = CatalogVerticalDatum::OrthometricMslEgm96;
    let surface = ConformalSurface::earth_globe();
    for (code, desig) in [("47N", "07"), ("N51", "04"), ("KEWR", "22L")] {
        let (_, rec) = AirportDatabase::builtin().by_code(code).expect(code);
        let rwy = rec.landing_runway(desig, datum).expect(desig);
        for range_m in [280.0_f64, 950.0, 5_556.0] {
            assert_banked_sphere_alignment(code, desig, surface, Some(datum), &rwy, range_m);
        }
    }
}

#[test]
fn lunar_hud_runway_aligns_on_moon_without_wgs84_or_egm96_leakage() {
    let moon = BodyEllipsoid::MOON_IAU;
    // Deep lunar basin (-1,900 m, well below Earth's -450 m floor) near Sea of Tranquility.
    let rwy = planetary_strip(
        moon,
        0.674_16,
        23.473_15,
        -1_900.0,
        55.0_f64.to_radians(),
        1_200.0,
    );
    let sphere_surface = ConformalSurface::GlobeSphere {
        physics: moon,
        render_body: LUNAR_RENDER_BODY,
    };
    let ellip_surface = ConformalSurface::ellipsoid(moon);

    for range_m in [280.0_f64, 1_500.0] {
        assert_banked_sphere_alignment("MOON", "06", sphere_surface, None, &rwy, range_m);
        assert_banked_ellipsoid_alignment("MOON", "06", ellip_surface, &rwy, range_m);
    }

    // Passing an Earth EGM96 datum on MOON_IAU must be ignored (0.0 m undulation), and
    // since MOON_IAU has flattening == 0.0, sphere and ellipsoid projections match exactly.
    let thr_local = LocalFrame::at(&rwy.threshold);
    let eye_fixed =
        moon.geodetic_to_fixed(&rwy.threshold) - thr_local.north() * 600.0 + thr_local.up() * 45.0;
    let clean = hud_runway(eye_fixed, sphere_surface, None, &rwy);
    let with_earth_datum = hud_runway(
        eye_fixed,
        sphere_surface,
        Some(CatalogVerticalDatum::OrthometricMslEgm96),
        &rwy,
    );
    let ellip = hud_runway(eye_fixed, ellip_surface, None, &rwy);
    assert_eq!(clean, with_earth_datum);
    for i in 0..3 {
        assert!((clean.threshold_m[i] - ellip.threshold_m[i]).abs() < 1e-3);
    }
    assert!(
        (clean.length_m - 1_200.0).abs() < 0.05,
        "{}",
        clean.length_m
    );
}

#[test]
fn martian_hud_runway_aligns_on_both_mars_sphere_and_oblate_mars_ellipsoid() {
    let mars = BodyEllipsoid::MARS_IAU;
    // Jezero Crater sits at ~-2,550 m below the Martian datum, with 1.19% equatorial N/M
    // oblateness (nearly 2x Earth's). Both rendering surfaces must align sub-pixel, while
    // differing by ~10+ m at 3 NM as required by their distinct geometries.
    let rwy = planetary_strip(
        mars,
        18.4446,
        77.4509,
        -2_550.0,
        55.0_f64.to_radians(),
        1_800.0,
    );
    let sphere_surface = ConformalSurface::GlobeSphere {
        physics: mars,
        render_body: MARS_RENDER_BODY,
    };
    let ellip_surface = ConformalSurface::ellipsoid(mars);

    for range_m in [280.0_f64, 950.0, 5_556.0] {
        assert_banked_sphere_alignment("MARS", "06", sphere_surface, None, &rwy, range_m);
        assert_banked_ellipsoid_alignment("MARS", "06", ellip_surface, &rwy, range_m);
    }

    let thr_local = LocalFrame::at(&rwy.threshold);
    let (sb, cb) = rwy.true_bearing_rad.sin_cos();
    let eye_fixed = mars.geodetic_to_fixed(&rwy.threshold)
        - (thr_local.north() * cb + thr_local.east() * sb) * 5_556.0
        + thr_local.up() * 300.0;
    let on_sphere = hud_runway(eye_fixed, sphere_surface, None, &rwy);
    let on_ellip = hud_runway(eye_fixed, ellip_surface, None, &rwy);
    let lateral_diff_m = (on_sphere.threshold_m[1] - on_ellip.threshold_m[1]).abs();
    assert!(
        lateral_diff_m > 10.0,
        "Mars 1.19% oblateness must separate sphere vs ellipsoid by >10 m at 3 NM (got {lateral_diff_m:.2} m)"
    );
}

fn planetary_strip(
    body: BodyEllipsoid,
    lat_deg: f64,
    lon_deg: f64,
    elev_m: f64,
    bearing_rad: f64,
    length_m: f64,
) -> LandingRunwayGeometry {
    let threshold = Geodetic::from_degrees(lat_deg, lon_deg, elev_m);
    let local = LocalFrame::at(&threshold);
    let (sb, cb) = bearing_rad.sin_cos();
    let fwd = local.north() * cb + local.east() * sb;
    let thr_fixed = body.geodetic_to_fixed(&threshold);
    let start_elev_m = elev_m + 3.5 * (120.0 / length_m);
    let mut start = body.fixed_to_geodetic(thr_fixed - fwd * 120.0);
    start.height_m = start_elev_m;
    let mut far_end = body.fixed_to_geodetic(thr_fixed + fwd * length_m);
    far_end.height_m = elev_m - 3.5;
    LandingRunwayGeometry {
        designator: CompactCode::parse("06").expect("designator"),
        pavement_start: start,
        threshold,
        far_end,
        full_length_m: length_m + 120.0,
        displaced_threshold_m: 120.0,
        reciprocal_displaced_m: 90.0,
        landing_length_m: length_m,
        width_m: 30.0,
        true_bearing_rad: bearing_rad,
        glidepath_rad: 3.0_f64.to_radians(),
        start_elevation_msl_m: start_elev_m,
        threshold_elevation_msl_m: elev_m,
        far_elevation_msl_m: elev_m - 3.5,
        field_elevation_msl_m: start_elev_m,
        surface: RunwaySurfaceKind::Paved,
        authority: AuthoritySource::OpenStreetMap,
        effective_ymd: [2026, 11, 5],
    }
}

fn build_banked_camera_on_body(
    body: BodyEllipsoid,
    rwy: &LandingRunwayGeometry,
    range_m: f64,
) -> sim_render::Camera {
    let thr_local = LocalFrame::at(&rwy.threshold);
    let (sb, cb) = rwy.true_bearing_rad.sin_cos();
    let along_fixed = thr_local.north() * cb + thr_local.east() * sb;
    let pos_fixed = body.geodetic_to_fixed(&rwy.threshold) - along_fixed * range_m
        + thr_local.up() * (range_m * rwy.glidepath_rad.tan());
    let pos_geo = body.fixed_to_geodetic(pos_fixed);
    let pos_local = LocalFrame::at(&pos_geo);
    let hdg = rwy.true_bearing_rad + 0.035;
    let base_att = DQuat::from_mat3(&pos_local.ned_to_ecef()) * DQuat::from_rotation_z(hdg);
    let bank_pitch = DQuat::from_euler(
        glam::EulerRot::ZYX,
        0.0,
        2.2_f64.to_radians(),
        (-4.5_f64).to_radians(),
    );
    let truth = VehicleState {
        position_ecef: pos_fixed,
        velocity_ecef: pos_local.north() * (52.0 * hdg.cos())
            + pos_local.east() * (52.0 * hdg.sin()),
        attitude: (base_att * bank_pitch).normalize(),
        angular_rate_body: DVec3::ZERO,
    };
    scenario_camera(pilot_eye_mount().pose(&truth, DVec3::ZERO))
}

fn sphere_stamp(
    surface: ConformalSurface,
    datum: Option<CatalogVerticalDatum>,
    cam: &sim_render::Camera,
) -> sim_maplibre::ConformalFrameStamp {
    let ConformalSurface::GlobeSphere {
        physics,
        render_body,
    } = surface
    else {
        panic!("expected GlobeSphere");
    };
    let eye_geo = physics.fixed_to_geodetic(cam.pose.position_ecef);
    let frustum = symmetric_frustum(
        cam.fov_y_rad,
        2560,
        1440,
        DEFAULT_NEAR_CLIP_M,
        DEFAULT_FAR_CLIP_M,
    )
    .expect("frustum");
    conformal_stamp_from_pose_on(
        SimInstant::START,
        physics,
        render_body,
        &CameraPoseEcef {
            position_ecef: cam.pose.position_ecef,
            orientation: cam.pose.orientation,
        },
        frustum,
        undulation_for_surface(surface, datum, &eye_geo),
    )
    .expect("stamp")
}

fn assert_banked_sphere_alignment(
    code: &str,
    desig: &str,
    surface: ConformalSurface,
    datum: Option<CatalogVerticalDatum>,
    rwy: &LandingRunwayGeometry,
    range_m: f64,
) {
    let physics = surface.physics_ellipsoid();
    let cam = build_banked_camera_on_body(physics, rwy, range_m);
    let stamp = sphere_stamp(surface, datum, &cam);
    let eye_geo = physics.fixed_to_geodetic(cam.pose.position_ecef);
    let hud = hud_runway(cam.pose.position_ecef, surface, datum, rwy);
    let (sc, cc) = hud.course_rad.sin_cos();
    let slope = hud.far_down_offset_m / hud.length_m;
    let cam_to_ned = DMat3::from_quat(
        DQuat::from_mat3(&LocalFrame::at(&eye_geo).ned_to_ecef()).inverse() * cam.pose.orientation,
    );
    let fy = 720.0 / (cam.fov_y_rad * 0.5).tan();
    for (name, geo, a_m) in [
        ("pavement_start", &rwy.pavement_start, -hud.pre_threshold_m),
        ("threshold", &rwy.threshold, 0.0_f32),
        ("far_end", &rwy.far_end, hud.length_m),
    ] {
        let hud_ned = DVec3::new(
            f64::from(hud.threshold_m[0] + cc * a_m),
            f64::from(hud.threshold_m[1] + sc * a_m),
            f64::from(hud.threshold_m[2] + slope * a_m),
        );
        let v_gl = cam_to_ned.transpose() * hud_ned;
        let maybe_otw = project_geodetic_to_screen(
            &stamp,
            geo,
            undulation_for_surface(surface, datum, geo),
            [2560, 1440],
        );
        if a_m >= 0.0 {
            assert!(maybe_otw.is_some(), "{code} {desig} {name} must be in view");
        }
        let Some((otw_px, _)) = maybe_otw else {
            assert!(-v_gl.z <= DEFAULT_NEAR_CLIP_M, "must also be behind camera");
            continue;
        };
        let hud_px = [
            (1280.0 + fy * (v_gl.x / -v_gl.z)) as f32,
            (720.0 - fy * (v_gl.y / -v_gl.z)) as f32,
        ];
        let dx = (hud_px[0] - otw_px[0]).abs();
        let dy = (hud_px[1] - otw_px[1]).abs();
        let max_dy = if a_m >= 0.0 { 0.12 } else { 0.75 };
        assert!(
            dx < 0.12 && dy < max_dy,
            "{code} {desig} {name} @ {range_m}m: HUD {hud_px:?} vs sphere OTW {otw_px:?} (dx={dx:.4}, dy={dy:.4})"
        );
    }
}

fn assert_banked_ellipsoid_alignment(
    code: &str,
    desig: &str,
    surface: ConformalSurface,
    rwy: &LandingRunwayGeometry,
    range_m: f64,
) {
    let physics = surface.physics_ellipsoid();
    let cam = build_banked_camera_on_body(physics, rwy, range_m);
    let eye_geo = physics.fixed_to_geodetic(cam.pose.position_ecef);
    let hud = hud_runway(cam.pose.position_ecef, surface, None, rwy);
    let (sc, cc) = hud.course_rad.sin_cos();
    let slope = hud.far_down_offset_m / hud.length_m;
    let cam_to_ned = DMat3::from_quat(
        DQuat::from_mat3(&LocalFrame::at(&eye_geo).ned_to_ecef()).inverse() * cam.pose.orientation,
    );
    let fixed_to_cam = DMat3::from_quat(cam.pose.orientation.inverse());
    let fy = 720.0 / (cam.fov_y_rad * 0.5).tan();
    for (name, geo, a_m) in [
        ("pavement_start", &rwy.pavement_start, -hud.pre_threshold_m),
        ("threshold", &rwy.threshold, 0.0_f32),
        ("far_end", &rwy.far_end, hud.length_m),
    ] {
        let hud_ned = DVec3::new(
            f64::from(hud.threshold_m[0] + cc * a_m),
            f64::from(hud.threshold_m[1] + sc * a_m),
            f64::from(hud.threshold_m[2] + slope * a_m),
        );
        let v_hud = cam_to_ned.transpose() * hud_ned;
        let v_mesh = fixed_to_cam * (physics.geodetic_to_fixed(geo) - cam.pose.position_ecef);
        if -v_mesh.z <= DEFAULT_NEAR_CLIP_M {
            assert!(-v_hud.z <= DEFAULT_NEAR_CLIP_M);
            continue;
        }
        let hud_px = [
            (1280.0 + fy * (v_hud.x / -v_hud.z)) as f32,
            (720.0 - fy * (v_hud.y / -v_hud.z)) as f32,
        ];
        let mesh_px = [
            (1280.0 + fy * (v_mesh.x / -v_mesh.z)) as f32,
            (720.0 - fy * (v_mesh.y / -v_mesh.z)) as f32,
        ];
        let dx = (hud_px[0] - mesh_px[0]).abs();
        let dy = (hud_px[1] - mesh_px[1]).abs();
        let max_dy = if a_m >= 0.0 { 0.05 } else { 0.50 };
        assert!(
            dx < 0.05 && dy < max_dy,
            "{code} {desig} {name} @ {range_m}m: HUD {hud_px:?} vs GlobeMesh {mesh_px:?} (dx={dx:.4}, dy={dy:.4})"
        );
    }
}

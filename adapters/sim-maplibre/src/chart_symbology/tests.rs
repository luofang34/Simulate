use flightbag::{RunwayDesignator, RunwayEndSpec};
use maplibre::headless::{AltitudeReference, Orientation3d, Placement3d, ViewSlotMask};
use world::GeodeticPosition;

use super::*;
use crate::geodesy::METRES_PER_NM;

fn camera(lat: f64, lon: f64, bearing_deg: f64) -> ChartCameraSpec {
    camera_at_zoom(lat, lon, 14.0, bearing_deg)
}

fn camera_at_zoom(lat: f64, lon: f64, zoom: f64, bearing_deg: f64) -> ChartCameraSpec {
    ChartCameraSpec {
        center_lat_deg: lat,
        center_lon_deg: lon,
        zoom,
        bearing_deg,
        pitch_deg: 0.0,
        surveillance_revision: 0,
        notices_revision: 0,
        symbology_revision: 0,
    }
}

#[test]
fn the_chart_centre_projects_to_the_viewport_centre_and_north_is_up() {
    let projection = ChartProjection::new(&camera(40.6, -74.2, 0.0), [800, 600], 1.0).unwrap();
    let [x, y] = projection.project(40.6, -74.2).unwrap();
    assert!((x - 400.0).abs() < 1e-3 && (y - 300.0).abs() < 1e-3);
    let [nx, ny] = projection.project(40.61, -74.2).unwrap();
    assert!((nx - 400.0).abs() < 1e-2 && ny < 300.0);
    // Antipodal point is hidden behind the planet.
    assert!(projection.project(-40.6, 105.8).is_none());
}

#[test]
fn retina_pixel_ratio_matches_logical_globe_camera_in_physical_pixels() {
    let cam = camera_at_zoom(39.13, -75.47, 10.6, 3.0);
    let logical = ChartProjection::new(&cam, [512, 384], 1.0).unwrap();
    let retina = ChartProjection::new(&cam, [1024, 768], 2.0).unwrap();
    for (lat, lon) in [(39.13, -75.47), (39.18, -75.45), (39.05, -75.52)] {
        let [lx, ly] = logical.project(lat, lon).unwrap();
        let [rx, ry] = retina.project(lat, lon).unwrap();
        assert!(
            (rx - lx * 2.0).abs() < 1e-3 && (ry - ly * 2.0).abs() < 1e-3,
            "Retina overlay must match 2x logical camera: ({rx}, {ry}) vs 2*({lx}, {ly})"
        );
    }
}

#[test]
fn a_track_up_chart_puts_the_heading_straight_up() {
    let projection = ChartProjection::new(&camera(40.6, -74.2, 90.0), [800, 600], 1.0).unwrap();
    // Due east of centre appears above it when the chart is rotated to a 090 track.
    let [x, y] = projection.project(40.6, -74.19).unwrap();
    assert!((x - 400.0).abs() < 0.5, "{x}");
    assert!(y < 300.0, "{y}");
    assert!(projection.screen_angle_at(40.6, -74.2, 90.0).abs() < 1e-6);
}

#[test]
fn ownship_is_a_haloed_silhouette_on_its_projected_position() {
    let own = ChartOwnship {
        lat_deg: 40.6,
        lon_deg: -74.2,
        true_heading_deg: 45.0,
        ground_velocity_en_mps: [40.0, 40.0],
    };
    let mesh = build_chart_symbology(&ChartSymbologyInput {
        camera: camera(40.6, -74.2, 0.0),
        viewport_px: [480, 360],
        ownship: Some(own),
        runway: None,
        traffic: &[],
    });
    assert_eq!(mesh.indices.len() % 3, 0);
    assert!(
        mesh.indices
            .iter()
            .all(|&i| (i as usize) < mesh.vertices.len())
    );
    let white = premultiplied(OWNSHIP_WHITE);
    let halo = premultiplied(HALO);
    let first_halo = mesh.vertices.iter().position(|v| v.color == halo);
    let first_white = mesh.vertices.iter().position(|v| v.color == white);
    assert!(
        first_halo < first_white,
        "halo draws beneath the silhouette"
    );
    let near_centre = mesh
        .vertices
        .iter()
        .filter(|v| v.color == white)
        .all(|v| (v.position[0] - 240.0).hypot(v.position[1] - 180.0) < 20.0);
    assert!(near_centre);
    assert!(mesh.vertices.iter().all(|v| v.depth == DEPTH));
    // The ground-track predictor is drawn, and nothing of ownship's is the active leg's
    // magenta.
    let track = premultiplied(TRACK_VECTOR);
    assert!(mesh.vertices.iter().any(|v| v.color == track));
    assert!(
        mesh.vertices
            .iter()
            .all(|v| !(v.color[0] > 0.7 && v.color[1] < 0.5 && v.color[2] > 0.6)),
        "no magenta beside the plan's"
    );
}

#[test]
fn south_pole_nzsp_rwy_02t_feather_crosses_90s_cleanly_and_culls_far_side_traffic() {
    let cam = camera_at_zoom(-89.9975, 139.2729, 10.2, 20.0);
    let projection = ChartProjection::new(&cam, [800, 600], 1.0).unwrap();
    let [cx, cy] = projection.project(-89.9975, 139.2729).unwrap();
    assert!((cx - 400.0).abs() < 1e-3 && (cy - 300.0).abs() < 1e-3);

    // Outbound 200° true from NZSP crosses into the Western Hemisphere (approaching
    // 139.2729° + 200° - 360° = -20.7271°); all 1..=5 NM points stay collinear straight
    // down the screen in a 020° track-up chart.
    for nm in 1..=5 {
        let [lon, lat] = destination([139.2729, -89.9975], 200.0, f64::from(nm) * METRES_PER_NM);
        assert!(
            (-21.0..-16.5).contains(&lon),
            "crossed polar meridian at {nm} NM: {lon}"
        );
        let [px, py] = projection
            .project(lat, lon)
            .expect("unoccluded polar point");
        assert!(
            (px - 400.0).abs() < 0.05 && py > 300.0,
            "collinear at {nm} NM: ({px}, {py})"
        );
    }

    let rwy_02t = RunwayEndSpec {
        designator: RunwayDesignator::parse("02").expect("02"),
        threshold: GeodeticPosition::wgs84(-89.9975, 139.2729, 2_835.0),
        true_bearing_rad: 20.0_f64.to_radians(),
        length_m: 3_658.0,
        width_m: 61.0,
    };
    let antipodal_traffic = DynamicEntity3D {
        id: 99,
        kind: DynamicEntityKind::FixedWing,
        placement: Placement3d {
            position: LatLon::new(89.9, -40.7),
            altitude_meters: 1_000.0,
            altitude_reference: AltitudeReference::AboveMsl,
            orientation: Orientation3d::IDENTITY,
            view_mask: ViewSlotMask::ALL,
        },
        velocity_enu_mps: DVec3::new(0.0, 60.0, 0.0),
        characteristic_span_m: 12.0,
    };
    let own = ChartOwnship {
        lat_deg: -89.9975,
        lon_deg: 139.2729,
        true_heading_deg: 20.0,
        ground_velocity_en_mps: [
            20.0_f64.to_radians().sin() * 55.0,
            20.0_f64.to_radians().cos() * 55.0,
        ],
    };
    let rwy_02t_geom = sim_scenery::LandingRunwayGeometry::from_runway_end_spec(&rwy_02t);
    let mesh = build_chart_symbology(&ChartSymbologyInput {
        camera: cam,
        viewport_px: [800, 600],
        ownship: Some(own),
        runway: Some(&rwy_02t_geom),
        traffic: &[antipodal_traffic],
    });
    assert!(
        mesh.vertices
            .iter()
            .all(|v| v.position[0].is_finite() && v.position[1].is_finite())
    );
    let traffic_color = premultiplied(TRAFFIC_CYAN);
    assert!(
        mesh.vertices.iter().all(|v| v.color != traffic_color),
        "far-side-of-Earth traffic must be occluded"
    );
    let feather_cyan = premultiplied(FEATHER_CYAN);
    assert!(mesh.vertices.iter().any(|v| v.color == feather_cyan));
}

#[test]
fn the_true_position_is_a_cased_crossed_circle_about_its_projected_point() {
    let chart = (&camera(40.6, -74.2, 0.0), [480, 360], 1.0);
    let mut mesh = OverlayMesh::default();
    marks::append_truth_mark(&mut mesh, chart, None, [40.6, -74.2]);
    assert!(!mesh.vertices.is_empty() && mesh.indices.len() % 3 == 0);
    let [r, g, b] = crate::TRUTH_MARK.map(|v| f32::from(v) / 255.0);
    let (mark, casing) = (
        premultiplied([r, g, b, 1.0]),
        premultiplied([0.0, 0.0, 0.0, 0.85]),
    );
    let first = |colour| mesh.vertices.iter().position(|v| v.color == colour);
    assert!(
        first(casing) < first(mark),
        "the casing draws beneath the mark"
    );
    assert!(
        mesh.vertices
            .iter()
            .all(|v| (v.position[0] - 240.0).hypot(v.position[1] - 180.0) < 16.0)
    );
    // A true position off the view, or behind the globe, draws nothing.
    let mut away = OverlayMesh::default();
    marks::append_truth_mark(&mut away, chart, None, [41.6, -74.2]);
    marks::append_truth_mark(&mut away, chart, None, [-40.6, 105.8]);
    assert!(away.vertices.is_empty());
}

#[test]
fn unproject_screen_position_round_trips_project_across_retina_and_rotation() {
    let cam = camera_at_zoom(55.7510, 37.6240, 14.2, 33.0);
    let chart = (&cam, [1024, 768], 2.0);
    let projection = ChartProjection::new(&cam, [1024, 768], 2.0).unwrap();
    for (lat, lon) in [(55.7510, 37.6240), (55.7532, 37.6227), (55.7485, 37.6253)] {
        let px = projection.project(lat, lon).unwrap();
        let [un_lat, un_lon] = marks::unproject_screen_position(chart, px).unwrap();
        assert!(
            (un_lat - lat).abs() < 1e-4 && (un_lon - lon).abs() < 1e-4,
            "expected ({lat}, {lon}), got ({un_lat}, {un_lon})"
        );
    }
    assert!(marks::unproject_screen_position(chart, [f32::NAN, 100.0]).is_none());
}

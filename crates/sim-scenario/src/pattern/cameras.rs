//! Canonical virtual camera preset suite for airport pattern & terminal-area operations.
//!
//! Pairs three aircraft-mounted sensors (`nose_nav`, `belly_optical`, `pattern_window`)
//! with two ground/tower cameras (`tower_ptz` auto-tracking with telephoto zoom from the
//! OSM control tower / CTAF viewpoint, and `threshold_final` fixed along the approach path).

use std::f64::consts::PI;

use glam::DVec3;
use sim_core::{CAMERA_DEFAULT_PERIOD, CameraMount, VirtualCameraPlacement, VirtualCameraSpec};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_scenery::OsmAerodromeDiagram;

/// Builds the 5-channel `30 Hz` [`VirtualCameraSpec`] suite for `diagram`.
#[must_use]
pub fn pattern_virtual_cameras(diagram: &OsmAerodromeDiagram) -> Vec<VirtualCameraSpec> {
    let base_53 = VirtualCameraSpec::pinhole_800x600(53.0_f64.to_radians());
    let base_58 = VirtualCameraSpec::pinhole_800x600(58.0_f64.to_radians());
    let base_42 = VirtualCameraSpec::pinhole_800x600(42.0_f64.to_radians());

    let (threshold_pos, approach_azimuth_rad) = threshold_camera_placement(diagram);

    vec![
        VirtualCameraSpec {
            id: "nose_nav",
            title: "Nose Nav Camera",
            placement: VirtualCameraPlacement::BodyMounted(CameraMount::forward_nose(DVec3::new(
                1.25, 0.0, -0.15,
            ))),
            base_intrinsics: base_53,
            period: CAMERA_DEFAULT_PERIOD,
            depth_readout: true,
        },
        VirtualCameraSpec {
            id: "belly_optical",
            title: "Belly Nadir Camera",
            placement: VirtualCameraPlacement::BodyMounted(CameraMount::nadir_belly(DVec3::new(
                0.10, 0.0, 0.55,
            ))),
            base_intrinsics: base_53,
            period: CAMERA_DEFAULT_PERIOD,
            depth_readout: true,
        },
        VirtualCameraSpec {
            id: "pattern_window",
            title: "Left Pattern Window",
            placement: VirtualCameraPlacement::BodyMounted(CameraMount::oblique(
                DVec3::new(0.35, -0.45, -0.25),
                -86.0_f64.to_radians(),
                -18.5_f64.to_radians(),
                0.0,
            )),
            base_intrinsics: base_58,
            period: CAMERA_DEFAULT_PERIOD,
            depth_readout: true,
        },
        VirtualCameraSpec {
            id: "tower_ptz",
            title: "Tower PTZ Auto-Track",
            placement: VirtualCameraPlacement::WorldAutoTrack {
                position: diagram.tower_viewpoint,
                lead_time_s: 0.20,
                auto_zoom_span_m: Some(220.0),
                min_fov_y_rad: 10.0_f64.to_radians(),
                max_fov_y_rad: 48.0_f64.to_radians(),
            },
            base_intrinsics: base_53,
            period: CAMERA_DEFAULT_PERIOD,
            depth_readout: true,
        },
        VirtualCameraSpec {
            id: "threshold_final",
            title: "Threshold Final Cam",
            placement: VirtualCameraPlacement::WorldFixed {
                position: threshold_pos,
                azimuth_rad: approach_azimuth_rad,
                elevation_rad: 3.4_f64.to_radians(),
                roll_rad: 0.0,
            },
            base_intrinsics: base_42,
            period: CAMERA_DEFAULT_PERIOD,
            depth_readout: true,
        },
    ]
}

fn threshold_camera_placement(diagram: &OsmAerodromeDiagram) -> (Geodetic, f64) {
    let Some(rwy) = diagram.primary_runway_end() else {
        return (diagram.tower_viewpoint, 0.0);
    };
    let thresh = Geodetic::from_degrees(
        rwy.threshold.latitude_deg,
        rwy.threshold.longitude_deg,
        diagram.field_elevation_m + 4.5,
    );
    let frame = LocalFrame::at(&thresh);
    let (sin_b, cos_b) = rwy.true_bearing_rad.sin_cos();
    let fwd = frame.north() * cos_b + frame.east() * sin_b;
    let right = frame.north() * (-sin_b) + frame.east() * cos_b;
    // Stand 65 m ahead of the threshold and 26 m to the side of the pavement edge,
    // looking back out along the 3-degree final approach path (`bearing + PI`).
    let side_m = 0.5 * rwy.width_m + 18.0;
    let cam_ecef = geodetic_to_ecef(&thresh) + fwd * 65.0 - right * side_m;
    let mut cam_geo = ecef_to_geodetic(cam_ecef);
    cam_geo.height_m = diagram.field_elevation_m + 4.5;
    (cam_geo, (rwy.true_bearing_rad + PI).rem_euclid(2.0 * PI))
}

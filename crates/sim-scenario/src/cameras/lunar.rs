//! Cameras for Eagle's descent. The Lunar Module thrusts along body `+x`, up
//! during the descent, so `-x` faces the regolith and the footpads; the
//! Commander's window faces `+z`.

use glam::DVec3;
use sim_core::{
    CAMERA_DEFAULT_PERIOD, CameraMount, FlybyTracker, VirtualCameraPlacement, VirtualCameraSpec,
};
use sim_geo::Geodetic;
use sim_scenery::{ElevationSource, LunarTranquilityScenery};

/// Depression of the pilot's view below the window axis, where the landing
/// point designator's lines fall.
const LPD_DEPRESSION_DEG: f64 = 35.0;
/// Tilt of the side camera below the `+y` axis.
const QUAD_DEPRESSION_DEG: f64 = 20.0;
/// The landing tracker: south of the approach so it looks across the low
/// eastern Sun rather than into it, metres east and north of the site.
const TRACKER_EAST_M: f64 = 400.0;
const TRACKER_NORTH_M: f64 = -300.0;
const TRACKER_HEIGHT_M: f64 = 2.0;
/// Range (`m`) beyond which the landing tracker hands over to a flyby.
const TRACKER_MAX_RANGE_M: f64 = 8_000.0;

const DESCENT_FLYBY: FlybyTracker = FlybyTracker {
    grid_step_m: 800.0,
    lateral_offset_m: 150.0,
    vertical_offset_m: 3.0,
    lead_time_s: 0.2,
    reference_distance_m: 200.0,
    reference_fov_y_deg: 18.0,
    fov_clamp_deg: (3.0, 55.0),
};

pub(super) fn cameras() -> [VirtualCameraSpec; 5] {
    let base = VirtualCameraSpec::pinhole_800x600(55.0_f64.to_radians());
    let camera = |id, title, placement| VirtualCameraSpec {
        id,
        title,
        placement,
        base_intrinsics: base,
        period: CAMERA_DEFAULT_PERIOD,
        depth_readout: true,
    };
    let (sin_lpd, cos_lpd) = LPD_DEPRESSION_DEG.to_radians().sin_cos();
    let (sin_quad, cos_quad) = QUAD_DEPRESSION_DEG.to_radians().sin_cos();
    [
        camera(
            "lpd_window",
            "LPD Pilot Window",
            VirtualCameraPlacement::BodyMounted(CameraMount::looking_along(
                DVec3::new(1.2, -0.4, 1.0),
                DVec3::Z * cos_lpd - DVec3::X * sin_lpd,
                DVec3::X * cos_lpd + DVec3::Z * sin_lpd,
            )),
        ),
        camera(
            "dps_nadir",
            "Descent Engine / Footpad Nadir",
            VirtualCameraPlacement::BodyMounted(CameraMount::looking_along(
                DVec3::new(-1.5, 0.0, 0.0),
                DVec3::NEG_X,
                DVec3::Z,
            )),
        ),
        camera(
            "starboard_quad",
            "Starboard Thruster Quad Cam",
            VirtualCameraPlacement::BodyMounted(CameraMount::looking_along(
                DVec3::new(0.5, 2.2, 0.0),
                DVec3::Y * cos_quad - DVec3::X * sin_quad,
                DVec3::X,
            )),
        ),
        camera(
            "tranquility_tracker",
            "Tranquility Base Landing Tracker",
            VirtualCameraPlacement::RangeLimitedAutoTrack {
                position: tracker_position(),
                lead_time_s: 0.2,
                auto_zoom_span_m: Some(30.0),
                min_fov_y_rad: 1.5_f64.to_radians(),
                max_fov_y_rad: 50.0_f64.to_radians(),
                max_range_m: TRACKER_MAX_RANGE_M,
                fallback: DESCENT_FLYBY,
            },
        ),
        camera(
            "lunar_flyby",
            "Lunar Descent Flyby Cam",
            VirtualCameraPlacement::FlybyAutoTrack(DESCENT_FLYBY),
        ),
    ]
}

/// The landing tracker's tripod on the Tranquility scenery, on the IAU Moon.
fn tracker_position() -> Geodetic {
    let scenery = LunarTranquilityScenery::apollo_11_landing();
    let ground = scenery.at_local(TRACKER_EAST_M, TRACKER_NORTH_M);
    // The scenery covers this point; were it ever not to, the mean sphere is
    // the best remaining guess for the ground.
    let ground_m = scenery
        .ground_height_m(&ground)
        .ok()
        .flatten()
        .unwrap_or(0.0);
    Geodetic {
        height_m: ground_m + TRACKER_HEIGHT_M,
        ..ground
    }
}

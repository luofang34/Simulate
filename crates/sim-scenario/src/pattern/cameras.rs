//! Canonical virtual camera preset suite for airport pattern & terminal-area operations.
//!
//! Pairs three aircraft-mounted sensors (`nose_nav`, `belly_optical`, `pattern_window`)
//! with two ground trackers: `tower_ptz` auto-tracking with telephoto zoom from the
//! OSM control tower / CTAF viewpoint, and `approach_flyby` tracking short final from
//! beside the runway. Both hand over to a flyby observer near ownship when it is far
//! from the airport, so cross-country flights still see the aircraft.

use glam::DVec3;
use std::borrow::Cow;

use sim_core::{
    AirframeVisibility, CAMERA_DEFAULT_PERIOD, CameraMount, FlybyTracker, VirtualCameraPlacement,
    VirtualCameraSpec,
};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, SensorIntrinsics};
use sim_scenery::{CatalogVerticalDatum, OsmAerodromeDiagram};

/// Range (`m`) beyond which the tower hands over to a nearby range camera.
const TOWER_MAX_RANGE_M: f64 = 18_000.0;
/// Range (`m`) beyond which the approach camera hands over to a flyby.
const APPROACH_MAX_RANGE_M: f64 = 12_000.0;
/// Approach camera tripod height above the field (`m`).
const APPROACH_CAMERA_AGL_M: f64 = 2.8;

/// A range camera standing well abeam ownship when the tower is too far.
const TOWER_FALLBACK: FlybyTracker = FlybyTracker {
    grid_step_m: 1_200.0,
    lateral_offset_m: 350.0,
    vertical_offset_m: 3.0,
    lead_time_s: 0.20,
    reference_distance_m: 350.0,
    reference_fov_y_deg: 12.0,
    fov_clamp_deg: (3.5, 48.0),
};

/// A cinematic trackside camera close abeam ownship.
const APPROACH_FALLBACK: FlybyTracker = FlybyTracker {
    grid_step_m: 500.0,
    lateral_offset_m: -220.0,
    vertical_offset_m: 3.0,
    lead_time_s: 0.15,
    reference_distance_m: 220.0,
    reference_fov_y_deg: 14.0,
    fov_clamp_deg: (4.0, 52.0),
};

/// Builds the 5-channel `30 Hz` [`VirtualCameraSpec`] suite for `diagram`,
/// taking its heights as already on the WGS84 ellipsoid.
#[must_use]
pub fn pattern_virtual_cameras(diagram: &OsmAerodromeDiagram) -> Vec<VirtualCameraSpec> {
    pattern_cameras_on_datum(diagram, CatalogVerticalDatum::EllipsoidWgs84).to_vec()
}

/// The pattern suite with `diagram`'s heights read in `datum` and raised onto
/// the WGS84 ellipsoid, where the trackers are placed.
pub(crate) fn pattern_cameras_on_datum(
    diagram: &OsmAerodromeDiagram,
    datum: CatalogVerticalDatum,
) -> [VirtualCameraSpec; 5] {
    let base_53 = VirtualCameraSpec::pinhole_800x600(53.0_f64.to_radians());
    let base_58 = VirtualCameraSpec::pinhole_800x600(58.0_f64.to_radians());
    let base_42 = VirtualCameraSpec::pinhole_800x600(42.0_f64.to_radians());
    let tower = on_ellipsoid(datum, diagram.tower_viewpoint);
    let approach = approach_camera_position(diagram, datum);
    [
        camera(
            "nose_nav",
            "Nose Nav Camera",
            AirframeVisibility::Hidden,
            VirtualCameraPlacement::BodyMounted(CameraMount::forward_nose(DVec3::new(
                1.25, 0.0, -0.15,
            ))),
            base_53,
        ),
        // High enough under the fuselage to stay in focus while parked.
        camera(
            "belly_optical",
            "Belly Nadir Camera",
            AirframeVisibility::Hidden,
            VirtualCameraPlacement::BodyMounted(CameraMount::nadir_belly(DVec3::new(
                0.15, 0.0, 0.18,
            ))),
            base_53,
        ),
        camera(
            "pattern_window",
            "Left Wing / Pattern Cam",
            AirframeVisibility::CabinWindow,
            VirtualCameraPlacement::BodyMounted(CameraMount::oblique(
                DVec3::new(0.35, -0.45, -0.25),
                -86.0_f64.to_radians(),
                -16.0_f64.to_radians(),
                0.0,
            )),
            base_58,
        ),
        camera(
            "tower_ptz",
            "Tower PTZ Auto-Track",
            AirframeVisibility::External,
            VirtualCameraPlacement::RangeLimitedAutoTrack {
                position: tower,
                lead_time_s: 0.20,
                auto_zoom_span_m: Some(50.0),
                min_fov_y_rad: 3.5_f64.to_radians(),
                max_fov_y_rad: 48.0_f64.to_radians(),
                max_range_m: TOWER_MAX_RANGE_M,
                fallback: TOWER_FALLBACK,
            },
            base_53,
        ),
        camera(
            "approach_flyby",
            "Approach / Flyby Track",
            AirframeVisibility::External,
            VirtualCameraPlacement::RangeLimitedAutoTrack {
                position: approach,
                lead_time_s: 0.15,
                auto_zoom_span_m: Some(40.0),
                min_fov_y_rad: 10.0_f64.to_radians(),
                max_fov_y_rad: 52.0_f64.to_radians(),
                max_range_m: APPROACH_MAX_RANGE_M,
                fallback: APPROACH_FALLBACK,
            },
            base_42,
        ),
    ]
}

/// A `30 Hz` camera with a depth readout.
fn camera(
    id: &'static str,
    title: &'static str,
    airframe: AirframeVisibility,
    placement: VirtualCameraPlacement,
    base_intrinsics: SensorIntrinsics,
) -> VirtualCameraSpec {
    VirtualCameraSpec {
        id: Cow::Borrowed(id),
        title: Cow::Borrowed(title),
        placement,
        airframe,
        base_intrinsics,
        period: CAMERA_DEFAULT_PERIOD,
        depth_readout: true,
    }
}

/// `position` with its height read in `datum`, on the WGS84 ellipsoid.
fn on_ellipsoid(datum: CatalogVerticalDatum, position: Geodetic) -> Geodetic {
    let (ellipsoidal, _) = datum.heights(position.lat_rad, position.lon_rad, position.height_m);
    Geodetic {
        height_m: ellipsoidal.0,
        ..position
    }
}

/// A tripod 65 m down the primary runway from its threshold and clear of the
/// pavement edge, from where short final and the touchdown are both in view.
fn approach_camera_position(
    diagram: &OsmAerodromeDiagram,
    datum: CatalogVerticalDatum,
) -> Geodetic {
    let Some(rwy) = diagram.primary_runway_end() else {
        return on_ellipsoid(datum, diagram.tower_viewpoint);
    };
    let field = on_ellipsoid(
        datum,
        Geodetic::from_degrees(
            rwy.threshold.latitude_deg,
            rwy.threshold.longitude_deg,
            diagram.field_elevation_m,
        ),
    );
    let body = BodyEllipsoid::WGS84;
    let frame = LocalFrame::at(&field);
    let (sin_b, cos_b) = rwy.true_bearing_rad.sin_cos();
    let fwd = frame.north() * cos_b + frame.east() * sin_b;
    let right = frame.north() * (-sin_b) + frame.east() * cos_b;
    let side_m = 0.5 * rwy.width_m + 18.0;
    let tripod = body.geodetic_to_fixed(&field) + fwd * 65.0 - right * side_m;
    Geodetic {
        height_m: field.height_m + APPROACH_CAMERA_AGL_M,
        ..body.fixed_to_geodetic(tripod)
    }
}

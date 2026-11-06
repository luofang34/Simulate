use glam::{DMat3, DQuat, DVec3};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, SensorIntrinsics};

use super::BLOCK;
use crate::{BlockError, CameraPoseEcef, VehicleState};

/// Shortest range used for aiming and zoom, so a camera at the vehicle still
/// has a direction and a finite focal length.
const MIN_RANGE_M: f64 = 0.1;
/// Horizontal speed below which the track is taken from the nose, because a
/// hovering or parked vehicle's velocity points nowhere in particular.
const MIN_TRACK_SPEED_M_S: f64 = 0.5;
/// Size of the latitude-longitude patches the flyby grid is measured in.
/// Distances are taken from the patch centre, because a coordinate far from
/// its origin swings with every turn of the track's heading; small patches
/// keep a straight pass within about a percent of stationary, and crossing
/// one moves the observer once.
const FLYBY_PATCH_RAD: f64 = std::f64::consts::PI / 180.0;
/// Field-of-view limits any zoom is held to (`rad`).
const MIN_FOV_Y_RAD: f64 = 0.01;
const MAX_FOV_Y_RAD: f64 = 2.5;

/// A stationary observer near ownship that watches it fly past, usable
/// anywhere on any body because it needs no surveyed site.
///
/// The track is cut into cells `grid_step_m` long; the observer stands at the
/// middle of the cell the vehicle is in, so in straight flight it holds still
/// while the vehicle approaches, passes abeam, and recedes, then steps one
/// cell ahead. Turning re-orients the cells, which moves the observer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlybyTracker {
    /// Along-track length of one observer station's cell (`m`).
    pub grid_step_m: f64,
    /// Observer offset to the right of the track (`m`, negative is left).
    pub lateral_offset_m: f64,
    /// Observer height above the vehicle (`m`). The terrain under the
    /// observer is unknown, so near the ground this should not be negative.
    pub vertical_offset_m: f64,
    /// Velocity lead time (`s`) added to the vehicle position when aiming.
    pub lead_time_s: f64,
    /// Range (`m`) at which the view is `reference_fov_y_deg` tall; the view
    /// narrows with range so the vehicle keeps its apparent size.
    pub reference_distance_m: f64,
    /// Vertical field of view (`deg`) at `reference_distance_m`.
    pub reference_fov_y_deg: f64,
    /// Narrowest and widest vertical field of view (`deg`).
    pub fov_clamp_deg: (f64, f64),
}

impl FlybyTracker {
    pub(super) fn validate(&self) -> Result<(), BlockError> {
        let usable = self.grid_step_m.is_finite()
            && self.grid_step_m > 0.0
            && self.reference_distance_m.is_finite()
            && self.reference_distance_m > 0.0
            && [
                self.lateral_offset_m,
                self.vertical_offset_m,
                self.lead_time_s,
                self.reference_fov_y_deg,
                self.fov_clamp_deg.0,
                self.fov_clamp_deg.1,
            ]
            .iter()
            .all(|v| v.is_finite());
        if usable {
            Ok(())
        } else {
            Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "flyby_tracker",
                detail: format!("{self:?}"),
            })
        }
    }

    fn fov_y_rad(&self, range_m: f64) -> f64 {
        let half = 0.5 * self.reference_fov_y_deg.to_radians();
        let fov = 2.0 * (half.tan() * self.reference_distance_m / range_m).atan();
        let (a, b) = self.fov_clamp_deg;
        let lo = a.min(b).to_radians().clamp(MIN_FOV_Y_RAD, MAX_FOV_Y_RAD);
        let hi = a.max(b).to_radians().clamp(lo, MAX_FOV_Y_RAD);
        fov.clamp(lo, hi)
    }
}

/// How a fixed tracker zooms with range.
#[derive(Debug, Clone, Copy)]
pub(super) struct AutoZoom {
    pub(super) span_m: Option<f64>,
    pub(super) min_fov_y_rad: f64,
    pub(super) max_fov_y_rad: f64,
}

fn frame_at(body: &BodyEllipsoid, fixed: DVec3) -> LocalFrame {
    LocalFrame::at(&body.fixed_to_geodetic(fixed))
}

/// Looks along `forward` with the image's top towards `zenith`.
fn look_orientation(forward: DVec3, frame: &LocalFrame) -> DQuat {
    let right = forward
        .cross(frame.up())
        .try_normalize()
        .unwrap_or_else(|| frame.east());
    let cam_up = right.cross(forward).normalize();
    DQuat::from_mat3(&DMat3::from_cols(right, cam_up, -forward)).normalize()
}

fn with_fov_y(base: &SensorIntrinsics, fov_y_rad: f64) -> SensorIntrinsics {
    let fy = 0.5 * f64::from(base.height) / (0.5 * fov_y_rad).tan();
    let aspect_scale = base.fx / base.fy.max(1e-6);
    SensorIntrinsics {
        fx: fy * aspect_scale,
        fy,
        ..*base
    }
}

pub(super) fn chase_pose(
    body: &BodyEllipsoid,
    vehicle: &VehicleState,
    offsets_m: DVec3,
    pitch_rad: f64,
) -> CameraPoseEcef {
    let frame = frame_at(body, vehicle.position_ecef);
    let zenith = frame.up();
    let fwd_h = horizontal_track_of_nose(vehicle, &frame);
    let right = fwd_h
        .cross(zenith)
        .try_normalize()
        .unwrap_or_else(|| frame.east());
    let [back_m, right_m, up_m] = offsets_m.to_array();
    let position_ecef = vehicle.position_ecef - fwd_h * back_m + right * right_m + zenith * up_m;
    let (sin_p, cos_p) = pitch_rad.sin_cos();
    let forward = (fwd_h * cos_p + zenith * sin_p).normalize();
    let cam_up = right.cross(forward).normalize();
    let orientation = DQuat::from_mat3(&DMat3::from_cols(right, cam_up, -forward)).normalize();
    CameraPoseEcef {
        position_ecef,
        orientation,
    }
}

pub(super) fn world_fixed_pose(
    body: &BodyEllipsoid,
    position: &Geodetic,
    azimuth_rad: f64,
    elevation_rad: f64,
    roll_rad: f64,
) -> CameraPoseEcef {
    let position_ecef = body.geodetic_to_fixed(position);
    let frame = LocalFrame::at(position);
    let (sin_az, cos_az) = azimuth_rad.sin_cos();
    let (sin_el, cos_el) = elevation_rad.sin_cos();
    let horiz = frame.north() * cos_az + frame.east() * sin_az;
    let zenith = frame.up();
    let forward = (horiz * cos_el + zenith * sin_el).normalize();
    let right0 = (frame.east() * cos_az - frame.north() * sin_az).normalize();
    let up0 = (-horiz * sin_el + zenith * cos_el).normalize();
    let (sin_r, cos_r) = roll_rad.sin_cos();
    let right = (right0 * cos_r + up0 * sin_r).normalize();
    let cam_up = (-right0 * sin_r + up0 * cos_r).normalize();
    let orientation = DQuat::from_mat3(&DMat3::from_cols(right, cam_up, -forward)).normalize();
    CameraPoseEcef {
        position_ecef,
        orientation,
    }
}

/// Aims from a fixed observer at the led vehicle and zooms with range.
pub(super) fn world_auto_track_pose(
    body: &BodyEllipsoid,
    position: &Geodetic,
    vehicle: &VehicleState,
    lead_time_s: f64,
    base: &SensorIntrinsics,
    zoom: AutoZoom,
) -> (CameraPoseEcef, SensorIntrinsics) {
    let position_ecef = body.geodetic_to_fixed(position);
    let frame = LocalFrame::at(position);
    let (orientation, range_m) = aim_from(position_ecef, &frame, vehicle, lead_time_s);
    let mut intrinsics = *base;
    if let Some(span_m) = zoom.span_m
        && span_m > 0.0
    {
        let lo = zoom.min_fov_y_rad.clamp(MIN_FOV_Y_RAD, 2.4);
        let hi = zoom.max_fov_y_rad.clamp(lo, MAX_FOV_Y_RAD);
        let desired_fov = (2.0 * (0.5 * span_m / range_m).atan()).clamp(lo, hi);
        intrinsics = with_fov_y(base, desired_fov);
    }
    (
        CameraPoseEcef {
            position_ecef,
            orientation,
        },
        intrinsics,
    )
}

/// The slant range from a fixed observer to the vehicle (`m`).
pub(super) fn range_from(body: &BodyEllipsoid, position: &Geodetic, vehicle: &VehicleState) -> f64 {
    (vehicle.position_ecef - body.geodetic_to_fixed(position)).length()
}

pub(super) fn flyby_pose(
    body: &BodyEllipsoid,
    tracker: &FlybyTracker,
    vehicle: &VehicleState,
    base: &SensorIntrinsics,
) -> (CameraPoseEcef, SensorIntrinsics) {
    let position_ecef = flyby_observer_ecef(body, tracker, vehicle);
    let frame = frame_at(body, position_ecef);
    let (orientation, range_m) = aim_from(position_ecef, &frame, vehicle, tracker.lead_time_s);
    (
        CameraPoseEcef {
            position_ecef,
            orientation,
        },
        with_fov_y(base, tracker.fov_y_rad(range_m)),
    )
}

fn aim_from(
    observer_ecef: DVec3,
    frame: &LocalFrame,
    vehicle: &VehicleState,
    lead_time_s: f64,
) -> (DQuat, f64) {
    let aim_ecef = vehicle.position_ecef + vehicle.velocity_ecef * lead_time_s;
    let delta = aim_ecef - observer_ecef;
    let range_m = delta.length().max(MIN_RANGE_M);
    let forward = (delta / range_m)
        .try_normalize()
        .unwrap_or_else(|| frame.north());
    (look_orientation(forward, frame), range_m)
}

/// The vehicle's horizontal nose direction, or north when the nose is
/// vertical.
fn horizontal_track_of_nose(vehicle: &VehicleState, frame: &LocalFrame) -> DVec3 {
    let zenith = frame.up();
    let nose = vehicle.forward_ecef();
    (nose - zenith * nose.dot(zenith))
        .try_normalize()
        .unwrap_or_else(|| frame.north())
}

/// The vehicle's horizontal direction of travel, from the nose when it is
/// nearly still.
fn horizontal_track(vehicle: &VehicleState, frame: &LocalFrame) -> DVec3 {
    let zenith = frame.up();
    let v = vehicle.velocity_ecef;
    let horizontal = v - zenith * v.dot(zenith);
    if horizontal.length() >= MIN_TRACK_SPEED_M_S {
        horizontal.normalize()
    } else {
        horizontal_track_of_nose(vehicle, frame)
    }
}

fn flyby_observer_ecef(
    body: &BodyEllipsoid,
    tracker: &FlybyTracker,
    vehicle: &VehicleState,
) -> DVec3 {
    let geo = body.fixed_to_geodetic(vehicle.position_ecef);
    let frame = LocalFrame::at(&geo);
    let track = horizontal_track(vehicle, &frame);
    let right = track
        .cross(frame.up())
        .try_normalize()
        .unwrap_or_else(|| frame.east());
    // A distance along the track that grows with the distance flown. The
    // body-fixed position cannot give one, since on a sphere it stays square
    // to a horizontal track; latitude and longitude scaled to metres can.
    let patch_centre = |angle: f64| ((angle / FLYBY_PATCH_RAD).floor() + 0.5) * FLYBY_PATCH_RAD;
    let (lat0, lon0) = (patch_centre(geo.lat_rad), patch_centre(geo.lon_rad));
    let east_m = (geo.lon_rad - lon0) * body.semi_major_m * lat0.cos();
    let north_m = (geo.lat_rad - lat0) * body.semi_major_m;
    let along_m = east_m * track.dot(frame.east()) + north_m * track.dot(frame.north());
    let step = tracker.grid_step_m;
    let station_m = ((along_m / step).floor() + 0.5) * step;
    vehicle.position_ecef
        + track * (station_m - along_m)
        + right * tracker.lateral_offset_m
        + frame.up() * tracker.vertical_offset_m
}

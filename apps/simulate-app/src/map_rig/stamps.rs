//! Conformal frame stamps of the window, sensor, and SVS cameras, and the geoid
//! undulation they are stamped with.

use glam::DVec3;
use sim_core::{SensorExposureEvent, SimInstant, VehicleState, VirtualCameraSpec};
use sim_geo::BodyEllipsoid;
use sim_maplibre::{
    ConformalFrameStamp, DEFAULT_FAR_CLIP_M, DEFAULT_NEAR_CLIP_M, conformal_stamp_from_pose,
    conformal_stamp_from_pose_on, conformal_stamp_from_vehicle_on, default_render_body,
    frustum_from_intrinsics, symmetric_frustum,
};
use sim_render::Camera;
use sim_scenery::CatalogVerticalDatum;

use crate::camera_view::{COCKPIT_FOV_Y_DEG, SVS_FOV_Y_DEG, svs_mount_for};
use crate::error::AppSimError;
use crate::vehicles::FreeFlightVehicle;

/// Geoid height above the WGS84 ellipsoid (`N = h − H`) the maps' altitudes are read
/// against: zero over a catalog whose terrain is ellipsoidal, the EGM96 undulation
/// otherwise, since real-world DEMs and the `AboveMsl` placements are orthometric.
#[must_use]
pub fn geoid_undulation_m(datum: Option<CatalogVerticalDatum>, lat_rad: f64, lon_rad: f64) -> f64 {
    match datum {
        Some(CatalogVerticalDatum::EllipsoidWgs84) => 0.0,
        Some(CatalogVerticalDatum::OrthometricMslEgm96) | None => {
            sim_geo::egm96_undulation_m(lat_rad, lon_rad)
        }
    }
}

/// The centerline cockpit-height SVS camera's stamp on the navigation solution `vehicle`
/// over `body`.
pub fn svs_stamp(
    timestamp: SimInstant,
    body: BodyEllipsoid,
    active_vehicle: Option<FreeFlightVehicle>,
    vehicle: &VehicleState,
    [w, h]: [u32; 2],
    undulation_m: f64,
) -> Result<ConformalFrameStamp, AppSimError> {
    let frustum = symmetric_frustum(
        SVS_FOV_Y_DEG.to_radians(),
        w,
        h,
        DEFAULT_NEAR_CLIP_M,
        DEFAULT_FAR_CLIP_M,
    )?;
    Ok(conformal_stamp_from_vehicle_on(
        timestamp,
        body,
        default_render_body(body),
        vehicle,
        &svs_mount_for(active_vehicle),
        DVec3::ZERO,
        frustum,
        undulation_m,
    )?)
}

/// The sensor camera's stamp at `exposure`'s exact instant, posed on its truth state.
pub fn sensor_exposure_stamp(
    body: BodyEllipsoid,
    cam_spec: &VirtualCameraSpec,
    exposure: &SensorExposureEvent,
    undulation_m: f64,
) -> Result<ConformalFrameStamp, AppSimError> {
    let (pose, intrinsics) = cam_spec
        .pose_and_intrinsics_on(&body, &exposure.state, DVec3::ZERO)
        .map_err(sim_scenario::ScenarioError::from)?;
    let frustum = frustum_from_intrinsics(&intrinsics, DEFAULT_NEAR_CLIP_M, DEFAULT_FAR_CLIP_M)?;
    Ok(conformal_stamp_from_pose_on(
        exposure.exposure_at,
        body,
        default_render_body(body),
        &pose,
        frustum,
        undulation_m,
    )?)
}

/// The window camera's stamp at `size_px`.
pub fn build_camera_stamp(
    timestamp: SimInstant,
    camera: &Camera,
    size_px: [u32; 2],
    undulation_m: f64,
) -> Result<ConformalFrameStamp, AppSimError> {
    let [w_px, h_px] = size_px;
    let fov_y = if camera.fov_y_rad.is_finite() && camera.fov_y_rad > 0.01 {
        camera.fov_y_rad
    } else {
        COCKPIT_FOV_Y_DEG.to_radians()
    };
    let pose_ecef = sim_core::CameraPoseEcef {
        position_ecef: camera.pose.position_ecef,
        orientation: camera.pose.orientation,
    };
    let frustum = symmetric_frustum(
        fov_y,
        w_px.max(1),
        h_px.max(1),
        DEFAULT_NEAR_CLIP_M,
        DEFAULT_FAR_CLIP_M,
    )?;
    Ok(conformal_stamp_from_pose(
        timestamp,
        &pose_ecef,
        frustum,
        undulation_m,
    )?)
}

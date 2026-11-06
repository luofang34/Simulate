//! Live OpenUSD stage snapshot of one gallery shot.

use std::path::Path;

use glam::{DMat3, DVec3};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, ecef_to_geodetic};
use sim_maplibre::{
    UsdCameraSample, UsdRunwaySample, UsdSceneExportInput, ViewSlotMask, export_scene_usda,
};
use sim_render::Camera;

use crate::cockpit_rig::CockpitRig;
use crate::error::AppSimError;
use crate::flight::Flight;

/// Writes the shot's ownship, traffic, runway, sun, and camera as a `.usda` stage.
pub(super) fn write_scene_usda(
    path: &Path,
    scenario_id: &str,
    rig: &CockpitRig,
    flight: &Flight,
    camera: &Camera,
    aspect: f64,
) -> Result<(), AppSimError> {
    let vehicle = flight.vehicle_state();
    let profile = flight.profile();
    let runway = rig.supervisor().active_runway(profile);
    let origin = runway.map_or_else(
        || ecef_to_geodetic(vehicle.position_ecef),
        |rwy| {
            Geodetic::from_degrees(
                rwy.threshold.latitude_deg,
                rwy.threshold.longitude_deg,
                rwy.threshold.ellipsoidal_height_m,
            )
        },
    );
    let enu = LocalFrame::at(&origin);
    let to_enu = |v: DVec3| DVec3::new(v.dot(enu.east()), v.dot(enu.north()), v.dot(enu.up()));
    let sun = to_enu(flight.world().sun_direction_ecef());
    let cam_rotation = DMat3::from_quat(camera.pose.orientation);
    let camera_to_enu = DMat3::from_cols(
        to_enu(cam_rotation.x_axis),
        to_enu(cam_rotation.y_axis),
        to_enu(cam_rotation.z_axis),
    );
    let runways = runway
        .map(|rwy| {
            let threshold_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&origin);
            let (sin_b, cos_b) = rwy.true_bearing_rad.sin_cos();
            let along = (enu.north() * cos_b + enu.east() * sin_b) * rwy.length_m;
            UsdRunwaySample {
                ident: rwy.designator.to_string(),
                threshold_geodetic: origin,
                opposite_geodetic: BodyEllipsoid::WGS84.fixed_to_geodetic(threshold_ecef + along),
                width_m: rwy.width_m,
            }
        })
        .into_iter()
        .collect();
    let input = UsdSceneExportInput {
        scenario_id: scenario_id.to_owned(),
        sim_time_s: flight
            .now()
            .duration_since(sim_core::SimInstant::START)
            .as_seconds(),
        enu_origin: origin,
        c172_asset_path: "c172.usdc".to_owned(),
        ownship_state: vehicle,
        ownship_articulation: flight.articulation(),
        traffic: rig.supervisor().truth_entities(ViewSlotMask::ALL),
        runways,
        sun_direction_enu: sun.to_array(),
        cameras: vec![UsdCameraSample {
            name: "ChaseCamera".to_owned(),
            eye_geodetic: ecef_to_geodetic(camera.pose.position_ecef),
            camera_to_enu_rotation: camera_to_enu,
            vfov_deg: camera.fov_y_rad.to_degrees(),
            aspect,
            clipping_range_m: [0.25, 350_000.0],
        }],
    };
    let usda = export_scene_usda(&input)?;
    std::fs::write(path, usda).map_err(|err| AppSimError::Screenshot {
        detail: format!("writing {}: {err}", path.display()),
    })
}

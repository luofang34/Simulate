//! Structured OpenUSD (`.usda`) scene stage exporter for AI inspection and cinematic DCC pipelines.
//!
//! Exports exact local-ENU metric coordinates (`+X` east, `+Y` north, `+Z` up) anchored to a
//! geodetic aerodrome origin, referencing `c172.usdc` (`</C172>`) with live engine/propeller
//! telemetry, articulated joint overrides, traffic aircraft, aerodrome runways, directional
//! sunlight (`UsdLuxDistantLight`), and calibrated virtual cameras (`UsdGeomCamera`).

use std::collections::HashMap;

use glam::{DMat3, DMat4, DQuat, DVec3};
use openusd::{
    gf, sdf,
    tf::Token,
    usd::{SchemaBase, Stage},
};
use openusd_schemas::{
    geom::{self, CameraSchema, XformableExt},
    lux::{self, DistantLightSchema, LightAPI},
};
use sim_core::{AirframeArticulation, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};

use crate::{DynamicEntity3D, MapAdapterError};

/// Calibrated camera entry exported as a `UsdGeomCamera` prim in the `.usda` scene.
#[derive(Clone, Debug)]
pub struct UsdCameraSample {
    /// Sanitized USD prim name under `/World/Cameras/<name>`.
    pub name: String,
    /// Camera eye position in WGS84 geodetic coordinates.
    pub eye_geodetic: Geodetic,
    /// Camera orientation mapping camera-local (`+X` right, `+Y` up, `-Z` forward) to local ENU.
    pub camera_to_enu_rotation: DMat3,
    /// Vertical field of view in degrees.
    pub vfov_deg: f64,
    /// Viewport width over height aspect ratio.
    pub aspect: f64,
    /// Near and far clip distances in metres.
    pub clipping_range_m: [f32; 2],
}

/// Runway threshold pair exported under `/World/Aerodrome/<ident>`.
#[derive(Clone, Debug)]
pub struct UsdRunwaySample {
    /// Runway identifier (e.g. `"22L"` or `"05_23"`).
    pub ident: String,
    /// Approach threshold geodetic coordinate.
    pub threshold_geodetic: Geodetic,
    /// Departure/opposite end geodetic coordinate.
    pub opposite_geodetic: Geodetic,
    /// Runway width in metres.
    pub width_m: f64,
}

/// Complete simulation snapshot authored into a structured `.usda` OpenUSD stage.
#[derive(Clone, Debug)]
pub struct UsdSceneExportInput {
    /// Aerodrome / corridor identifier (e.g. `"KEWR"`, `"47N"`).
    pub scenario_id: String,
    /// Simulation elapsed time in seconds.
    pub sim_time_s: f64,
    /// Local ENU origin (`+X` east, `+Y` north, `+Z` up) in WGS84 coordinates.
    pub enu_origin: Geodetic,
    /// Relative or absolute path to `c172.usdc` authored in USD `references` arcs.
    pub c172_asset_path: String,
    /// Ownship truth kinematic state.
    pub ownship_state: VehicleState,
    /// Ownship engine, propeller, control-surface, and landing-gear articulation.
    pub ownship_articulation: AirframeArticulation,
    /// Traffic aircraft and ground vehicles in the scene.
    pub traffic: Vec<DynamicEntity3D>,
    /// Aerodrome runway geometry samples.
    pub runways: Vec<UsdRunwaySample>,
    /// Sun unit vector in local ENU pointing toward the sun (`[east, north, up]`).
    pub sun_direction_enu: [f64; 3],
    /// Calibrated virtual and cockpit cameras active at this instant.
    pub cameras: Vec<UsdCameraSample>,
}

/// Exports `input` as a standards-compliant ASCII OpenUSD (`.usda`) stage string.
pub fn export_scene_usda(input: &UsdSceneExportInput) -> Result<String, MapAdapterError> {
    let stage = Stage::builder()
        .schema_registry(openusd_schemas::schema_registry())
        .in_memory("simulate_scene.usda")
        .map_err(usd_err)?;

    author_stage_header(&stage, input)?;
    let origin_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&input.enu_origin);
    let enu_frame = LocalFrame::at(&input.enu_origin);

    let world = geom::Xform::define(&stage, "/World").map_err(usd_err)?;
    let wp = world.prim();
    set_custom_double(wp, "sim:simTimeSeconds", input.sim_time_s)?;
    set_custom_double(
        wp,
        "sim:originLatDeg",
        input.enu_origin.lat_rad.to_degrees(),
    )?;
    set_custom_double(
        wp,
        "sim:originLonDeg",
        input.enu_origin.lon_rad.to_degrees(),
    )?;
    set_custom_double(wp, "sim:originAltEllipsoidM", input.enu_origin.height_m)?;
    set_custom_string(wp, "sim:scenarioId", &input.scenario_id)?;

    author_sun_light(&stage, input.sun_direction_enu)?;
    author_ownship(&stage, input, origin_ecef, &enu_frame)?;
    author_traffic(&stage, input, origin_ecef, &enu_frame)?;
    author_aerodrome(&stage, &input.runways, origin_ecef, &enu_frame)?;
    author_cameras(&stage, &input.cameras, origin_ecef, &enu_frame)?;

    stage
        .root_layer()
        .export_to_string()
        .map_err(|err| MapAdapterError::OpenUsd {
            detail: err.to_string(),
        })
}

fn author_stage_header(stage: &Stage, input: &UsdSceneExportInput) -> Result<(), MapAdapterError> {
    stage.set_default_prim("World").map_err(author_err)?;
    let doc = format!(
        "Simulate OpenUSD scene snapshot ({}) at t={:.3}s",
        input.scenario_id, input.sim_time_s
    );
    let root_id = stage.root_layer().identifier().to_owned();
    if let Some(mut layer) = stage.layer_mut(&root_id) {
        let _ = layer
            .edit(|ed| {
                ed.set_time_codes_per_second(60.0)?;
                let mut pr = ed.pseudo_root_mut()?;
                pr.set("upAxis", Token::new("Z"));
                pr.set("metersPerUnit", 1.0_f64);
                pr.set(sdf::FieldKey::Documentation, doc);
                Ok(())
            })
            .map_err(|err| MapAdapterError::OpenUsd {
                detail: err.to_string(),
            })?;
    }
    Ok(())
}

fn author_sun_light(stage: &Stage, sun_enu: [f64; 3]) -> Result<(), MapAdapterError> {
    let sun = lux::DistantLight::define(stage, "/World/Sun").map_err(usd_err)?;
    let api = LightAPI::from_prim_unchecked(sun.prim().clone());
    api.create_intensity_attr()
        .map_err(usd_err)?
        .set(1.0_f32)
        .map(|_| ())
        .map_err(author_err)?;
    sun.create_angle_attr()
        .map_err(usd_err)?
        .set(0.53_f32)
        .map(|_| ())
        .map_err(author_err)?;
    set_custom_vec3d(sun.prim(), "sim:sunDirectionEnu", sun_enu)
}

fn author_ownship(
    stage: &Stage,
    input: &UsdSceneExportInput,
    origin_ecef: DVec3,
    enu_frame: &LocalFrame,
) -> Result<(), MapAdapterError> {
    let ownship = geom::Xform::define(stage, "/World/Ownship").map_err(usd_err)?;
    let enu = enu_frame.ecef_to_enu_vector(input.ownship_state.position_ecef - origin_ecef);
    let q_aircraft_to_enu = aircraft_usd_to_enu_quat(enu_frame, input.ownship_state.attitude);
    let col_mat = DMat4::from_rotation_translation(q_aircraft_to_enu, enu);
    let ownship = ownship
        .set_transform(gf::Matrix4d::from(col_mat.to_cols_array()))
        .map_err(schema_err)?;

    if !input.c172_asset_path.is_empty() {
        let c172_path = sdf::try_into_path("/C172").map_err(path_err)?;
        let reference = sdf::Reference {
            asset_path: input.c172_asset_path.clone(),
            prim_path: c172_path,
            layer_offset: sdf::LayerOffset::default(),
            custom_data: HashMap::new(),
        };
        let _ = ownship
            .prim()
            .clone()
            .set_metadata(
                "references",
                sdf::Value::ReferenceListOp(sdf::ListOp::prepended([reference])),
            )
            .map_err(author_err)?;
    }

    author_ownship_telemetry(ownship.prim(), &input.ownship_articulation)?;
    author_ownship_joint_overrides(stage, &input.ownship_articulation)
}

fn author_ownship_telemetry(
    prim: &openusd::usd::Prim,
    art: &AirframeArticulation,
) -> Result<(), MapAdapterError> {
    let vis = art.propeller_visual(1.0 / 60.0);
    set_custom_double(prim, "sim:engineRpm", art.engine_rpm)?;
    set_custom_double(prim, "sim:propellerRpm", art.propeller_rpm)?;
    set_custom_double(prim, "sim:bladeAngleDeg", art.propeller_blade_angle_deg)?;
    set_custom_double(prim, "sim:thrustN", art.propeller_thrust_n)?;
    let [engine, _] = art.gauges.engines;
    set_custom_double(prim, "sim:manifoldPressurePa", engine.manifold_pressure_pa)?;
    set_custom_double(prim, "sim:fuelFlowGph", art.gauges.total_fuel_flow_gph())?;
    set_custom_double(prim, "sim:egtDegF", engine.egt_deg_f)?;
    set_custom_double(prim, "sim:chtDegF", engine.cht_deg_f)?;
    set_custom_double(prim, "sim:oilTempDegF", engine.oil_temp_deg_f)?;
    set_custom_double(prim, "sim:oilPressurePsi", engine.oil_pressure_psi)?;
    set_custom_double(prim, "sim:propellerPhaseRad", art.propeller_phase_rad)?;
    set_custom_double(
        prim,
        "sim:propellerDiscOpacity",
        f64::from(vis.disc_opacity),
    )?;
    set_custom_double(prim, "sim:flapDeg", art.flap_deg)?;
    set_custom_double(prim, "sim:elevatorDeg", art.elevator_rad.to_degrees())?;
    set_custom_double(prim, "sim:rudderDeg", art.rudder_rad.to_degrees())?;
    Ok(())
}

fn author_ownship_joint_overrides(
    stage: &Stage,
    art: &AirframeArticulation,
) -> Result<(), MapAdapterError> {
    let hub = stage
        .override_prim("/World/Ownship/Powerplant/PropellerHub")
        .map_err(author_err)?;
    let spin_deg = (-art.propeller_sense * art.propeller_phase_rad).to_degrees() as f32;
    set_float_attr(&hub, "xformOp:rotateY", spin_deg)?;

    for (subpath, deg) in [
        ("FlapLeft", art.flap_deg as f32),
        ("FlapRight", art.flap_deg as f32),
        ("AileronLeft", art.left_aileron_rad.to_degrees() as f32),
        ("AileronRight", art.right_aileron_rad.to_degrees() as f32),
        ("Elevator", art.elevator_rad.to_degrees() as f32),
    ] {
        let p = stage
            .override_prim(format!("/World/Ownship/ControlSurfaces/{subpath}"))
            .map_err(author_err)?;
        set_float_attr(&p, "xformOp:rotateX", deg)?;
    }
    Ok(())
}

fn author_traffic(
    stage: &Stage,
    input: &UsdSceneExportInput,
    origin_ecef: DVec3,
    enu_frame: &LocalFrame,
) -> Result<(), MapAdapterError> {
    let _ = geom::Xform::define(stage, "/World/Traffic").map_err(usd_err)?;
    for (idx, entity) in input.traffic.iter().enumerate() {
        let name = format!("/World/Traffic/Contact_{idx:02}_{}", entity.id);
        let xf = geom::Xform::define(stage, &name).map_err(usd_err)?;
        let g = Geodetic {
            lat_rad: entity.placement.position.latitude.to_radians(),
            lon_rad: entity.placement.position.longitude.to_radians(),
            height_m: entity.placement.altitude_meters,
        };
        let ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&g);
        let enu = enu_frame.ecef_to_enu_vector(ecef - origin_ecef);
        let [qx, qy, qz, qw] = entity.placement.orientation.enu_from_body;
        let q_enu = DQuat::from_xyzw(qx, qy, qz, qw).normalize();
        let col_mat = DMat4::from_rotation_translation(q_enu, enu);
        let xf = xf
            .set_transform(gf::Matrix4d::from(col_mat.to_cols_array()))
            .map_err(schema_err)?;
        let pr = xf.prim();
        set_custom_double(pr, "sim:characteristicSpanM", entity.characteristic_span_m)?;
        set_custom_vec3d(
            pr,
            "sim:velocityEnuMps",
            [
                entity.velocity_enu_mps.x,
                entity.velocity_enu_mps.y,
                entity.velocity_enu_mps.z,
            ],
        )?;
    }
    Ok(())
}

fn author_aerodrome(
    stage: &Stage,
    runways: &[UsdRunwaySample],
    origin_ecef: DVec3,
    enu_frame: &LocalFrame,
) -> Result<(), MapAdapterError> {
    let _ = geom::Xform::define(stage, "/World/Aerodrome").map_err(usd_err)?;
    for rwy in runways {
        let path = format!("/World/Aerodrome/Runway_{}", sanitize_ident(&rwy.ident));
        let xf = geom::Xform::define(stage, &path).map_err(usd_err)?;
        let e0 = BodyEllipsoid::WGS84.geodetic_to_fixed(&rwy.threshold_geodetic);
        let e1 = BodyEllipsoid::WGS84.geodetic_to_fixed(&rwy.opposite_geodetic);
        let p0 = enu_frame.ecef_to_enu_vector(e0 - origin_ecef);
        let p1 = enu_frame.ecef_to_enu_vector(e1 - origin_ecef);
        let pr = xf.prim();
        set_custom_vec3d(pr, "sim:thresholdEnuM", [p0.x, p0.y, p0.z])?;
        set_custom_vec3d(pr, "sim:oppositeEnuM", [p1.x, p1.y, p1.z])?;
        set_custom_double(pr, "sim:widthM", rwy.width_m)?;
    }
    Ok(())
}

fn author_cameras(
    stage: &Stage,
    cameras: &[UsdCameraSample],
    origin_ecef: DVec3,
    enu_frame: &LocalFrame,
) -> Result<(), MapAdapterError> {
    let _ = geom::Xform::define(stage, "/World/Cameras").map_err(usd_err)?;
    for cam in cameras {
        let path = format!("/World/Cameras/{}", sanitize_ident(&cam.name));
        let usd_cam = geom::Camera::define(stage, &path).map_err(usd_err)?;
        let v_ap_mm = 24.0_f64;
        let h_ap_mm = v_ap_mm * cam.aspect.max(0.1);
        let half_vfov = (cam.vfov_deg.clamp(5.0, 150.0) * 0.5).to_radians();
        let focal_mm = (0.5 * v_ap_mm) / half_vfov.tan().max(1e-4);

        usd_cam
            .create_focal_length_attr()
            .map_err(usd_err)?
            .set(focal_mm as f32)
            .map(|_| ())
            .map_err(author_err)?;
        usd_cam
            .create_horizontal_aperture_attr()
            .map_err(usd_err)?
            .set(h_ap_mm as f32)
            .map(|_| ())
            .map_err(author_err)?;
        usd_cam
            .create_vertical_aperture_attr()
            .map_err(usd_err)?
            .set(v_ap_mm as f32)
            .map(|_| ())
            .map_err(author_err)?;
        let [near_m, far_m] = cam.clipping_range_m;
        usd_cam
            .create_clipping_range_attr()
            .map_err(usd_err)?
            .set(gf::Vec2f {
                x: near_m,
                y: far_m,
            })
            .map(|_| ())
            .map_err(author_err)?;

        let eye_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&cam.eye_geodetic);
        let eye_enu = enu_frame.ecef_to_enu_vector(eye_ecef - origin_ecef);
        let col_mat = DMat4::from_cols_array_2d(&[
            [
                cam.camera_to_enu_rotation.x_axis.x,
                cam.camera_to_enu_rotation.x_axis.y,
                cam.camera_to_enu_rotation.x_axis.z,
                0.0,
            ],
            [
                cam.camera_to_enu_rotation.y_axis.x,
                cam.camera_to_enu_rotation.y_axis.y,
                cam.camera_to_enu_rotation.y_axis.z,
                0.0,
            ],
            [
                cam.camera_to_enu_rotation.z_axis.x,
                cam.camera_to_enu_rotation.z_axis.y,
                cam.camera_to_enu_rotation.z_axis.z,
                0.0,
            ],
            [eye_enu.x, eye_enu.y, eye_enu.z, 1.0],
        ]);
        let _ = usd_cam
            .set_transform(gf::Matrix4d::from(col_mat.to_cols_array()))
            .map_err(schema_err)?;
    }
    Ok(())
}

/// Maps JSBSim/Simulate NED-from-FRD quaternion to ENU-from-USD-aircraft (`+X` right, `+Y` fwd, `+Z` up).
/// Rotation of the model's `+X` right, `+Y` forward, `+Z` up axes into the stage's ENU
/// frame, from a body FRD-to-ECEF attitude.
pub(crate) fn aircraft_usd_to_enu_quat(enu_frame: &LocalFrame, ecef_from_frd: DQuat) -> DQuat {
    let enu_from_ecef =
        DMat3::from_cols(enu_frame.east(), enu_frame.north(), enu_frame.up()).transpose();
    let frd_from_usd = DMat3::from_cols(DVec3::Y, DVec3::X, DVec3::NEG_Z);
    let rotation = enu_from_ecef * DMat3::from_quat(ecef_from_frd.normalize()) * frd_from_usd;
    DQuat::from_mat3(&rotation).normalize()
}

fn sanitize_ident(raw: &str) -> String {
    let mut out = String::with_capacity(raw.len().max(1));
    for ch in raw.chars() {
        if ch.is_ascii_alphanumeric() || ch == '_' {
            out.push(ch);
        } else {
            out.push('_');
        }
    }
    if out.is_empty() || out.starts_with(|c: char| c.is_ascii_digit()) {
        out.insert(0, 'P');
    }
    out
}

fn set_custom_double(
    prim: &openusd::usd::Prim,
    name: &str,
    val: f64,
) -> Result<(), MapAdapterError> {
    prim.create_attribute(name, "double")
        .map_err(author_err)?
        .set(val)
        .map(|_| ())
        .map_err(author_err)
}

fn set_custom_string(
    prim: &openusd::usd::Prim,
    name: &str,
    val: &str,
) -> Result<(), MapAdapterError> {
    prim.create_attribute(name, "string")
        .map_err(author_err)?
        .set(val.to_owned())
        .map(|_| ())
        .map_err(author_err)
}

fn set_custom_vec3d(
    prim: &openusd::usd::Prim,
    name: &str,
    val: [f64; 3],
) -> Result<(), MapAdapterError> {
    let [x, y, z] = val;
    prim.create_attribute(name, "double3")
        .map_err(author_err)?
        .set(gf::Vec3d { x, y, z })
        .map(|_| ())
        .map_err(author_err)
}

fn set_float_attr(prim: &openusd::usd::Prim, name: &str, val: f32) -> Result<(), MapAdapterError> {
    prim.create_attribute(name, "float")
        .map_err(author_err)?
        .set(val)
        .map(|_| ())
        .map_err(author_err)
}

fn usd_err(err: openusd::Error) -> MapAdapterError {
    MapAdapterError::OpenUsd {
        detail: err.to_string(),
    }
}

fn author_err(err: openusd::usd::StageAuthoringError) -> MapAdapterError {
    MapAdapterError::OpenUsd {
        detail: err.to_string(),
    }
}

fn schema_err(err: openusd_schemas::SchemaError) -> MapAdapterError {
    MapAdapterError::OpenUsd {
        detail: err.to_string(),
    }
}

fn path_err(err: sdf::PathParseError) -> MapAdapterError {
    MapAdapterError::OpenUsd {
        detail: err.to_string(),
    }
}

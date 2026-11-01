//! Geodetic-local 6-DoF [`ExternalView`] bridge and conformal frame stamps.
//!
//! Bridges WGS-84 (`sim-geo` / `sim-core`) into `maplibre`'s spherical globe
//! strictly through geodetic `(lat, lon, h_msl)` and quaternion-derived local
//! ENU rotation matrices — never by linearly mapping WGS-84 Cartesian ECEF into
//! unit-sphere coordinates, and never via Euler `forward.cross(up)` decomposition.

use cgmath::{Matrix3, Matrix4, Rad};
use glam::{DMat3, DVec3};
use maplibre::{
    coords::LatLon,
    projection::{body::Body, globe::lat_lon_to_unit_sphere},
    render::{
        camera::EyeFrustum,
        view_state::{ExternalAnchor, ExternalView},
    },
};
use sim_core::{CameraMount, CameraPoseEcef, SimInstant, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, SensorIntrinsics};

use crate::MapAdapterError;

/// Default far-plane clip distance in local metres (`350 km`, safely past Earth's horizon).
pub const DEFAULT_FAR_CLIP_M: f64 = 350_000.0;

/// Default near-plane clip distance in local metres (`0.25 m`, safe at `0.5 m` runway eye height).
pub const DEFAULT_NEAR_CLIP_M: f64 = 0.25;

/// Geodetic camera anchor on Earth with explicit orthometric MSL height.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeodeticAnchor {
    /// WGS-84 geodetic latitude in degrees (`[-90, 90]`).
    pub lat_deg: f64,
    /// WGS-84 geodetic longitude in degrees (`[-180, 180]`).
    pub lon_deg: f64,
    /// Orthometric altitude in metres above mean sea level (may be negative, e.g. `-86 m` in Death Valley).
    pub altitude_msl_m: f64,
}

/// Atomic pairing of a rendered underlay frame with the exact camera state that drew it.
#[derive(Clone, Copy, Debug)]
pub struct ConformalFrameStamp {
    /// Simulation timestamp when the pose was sampled.
    pub instant: SimInstant,
    /// Geodetic camera origin used for the render pass.
    pub anchor: GeodeticAnchor,
    /// Exact `ExternalView` submitted to `maplibre` for this texture frame.
    pub external_view: ExternalView,
    /// Geoid undulation (`h_ellipsoid - h_msl`, metres) applied to this frame.
    pub geoid_undulation_m: f64,
    /// Deterministic 64-bit hash of the view and frustum matrices for dirty-gating.
    pub projection_fingerprint: u64,
}

/// Converts [`SensorIntrinsics`] (`width, height, fx, fy, cx, cy`) into an exact
/// asymmetric [`EyeFrustum`].
pub fn frustum_from_intrinsics(
    intrinsics: &SensorIntrinsics,
    near_m: f64,
    far_m: f64,
) -> Result<EyeFrustum, MapAdapterError> {
    if intrinsics.width == 0
        || intrinsics.height == 0
        || !intrinsics.fx.is_finite()
        || !intrinsics.fy.is_finite()
        || intrinsics.fx <= 0.0
        || intrinsics.fy <= 0.0
        || !near_m.is_finite()
        || !far_m.is_finite()
        || near_m <= 0.0
        || far_m <= near_m
    {
        return Err(MapAdapterError::InvalidPose {
            detail: "sensor intrinsics or clip planes are non-positive or non-finite",
        });
    }
    let w = f64::from(intrinsics.width);
    let h = f64::from(intrinsics.height);
    Ok(EyeFrustum {
        left: intrinsics.cx / intrinsics.fx,
        right: (w - intrinsics.cx) / intrinsics.fx,
        top: intrinsics.cy / intrinsics.fy,
        bottom: (h - intrinsics.cy) / intrinsics.fy,
        near: near_m,
        far: far_m,
    })
}

/// Builds a symmetric [`EyeFrustum`] from vertical field of view (radians) and viewport size.
pub fn symmetric_frustum(
    fov_y_rad: f64,
    width: u32,
    height: u32,
    near_m: f64,
    far_m: f64,
) -> Result<EyeFrustum, MapAdapterError> {
    if width == 0 || height == 0 {
        return Err(MapAdapterError::InvalidViewportSize { width, height });
    }
    if !fov_y_rad.is_finite()
        || fov_y_rad <= 0.0
        || fov_y_rad >= std::f64::consts::PI
        || !near_m.is_finite()
        || !far_m.is_finite()
        || near_m <= 0.0
        || far_m <= near_m
    {
        return Err(MapAdapterError::InvalidPose {
            detail: "symmetric field of view or clip distances out of range",
        });
    }
    let aspect = f64::from(width) / f64::from(height);
    Ok(EyeFrustum::symmetric(Rad(fov_y_rad), aspect, near_m, far_m))
}

/// Converts a WGS-84 [`CameraPoseEcef`] into a quaternion-direct [`ExternalView`].
///
/// `geoid_undulation_m` is `h_wgs84 - h_msl` (`0.0` when heights are already MSL).
/// Valid across all 6-DoF attitudes (`±90°` zenith/nadir, steep roll, inverted)
/// and below-MSL elevations down to `Body::EARTH.lowest_ground_meters` (`-450 m`).
pub fn external_view_from_pose(
    pose: &CameraPoseEcef,
    frustum: EyeFrustum,
    geoid_undulation_m: f64,
) -> Result<(GeodeticAnchor, ExternalView), MapAdapterError> {
    if !pose.position_ecef.is_finite()
        || !pose.orientation.is_finite()
        || !geoid_undulation_m.is_finite()
    {
        return Err(MapAdapterError::InvalidPose {
            detail: "camera ECEF position, quaternion, or geoid offset is non-finite",
        });
    }
    let geodetic = BodyEllipsoid::WGS84.fixed_to_geodetic(pose.position_ecef);
    let altitude_msl_m = geodetic.height_m - geoid_undulation_m;
    if altitude_msl_m <= Body::EARTH.lowest_ground_meters {
        return Err(MapAdapterError::InvalidPose {
            detail: "camera altitude is below Earth's lowest ground floor (-450 m MSL)",
        });
    }
    let local = LocalFrame::at(&geodetic);
    let ned_to_ecef = local.ned_to_ecef();
    // ENU basis vectors in ECEF: East = NED +Y, North = NED +X, Up = -NED +Z.
    let enu_to_ecef = DMat3::from_cols(ned_to_ecef.y_axis, ned_to_ecef.x_axis, -ned_to_ecef.z_axis);
    let ecef_to_eye = DMat3::from_quat(pose.orientation.normalize().inverse());
    let enu_to_eye = ecef_to_eye * enu_to_ecef;

    let c0 = enu_to_eye.x_axis;
    let c1 = enu_to_eye.y_axis;
    let c2 = enu_to_eye.z_axis;
    let rot = Matrix3::new(c0.x, c0.y, c0.z, c1.x, c1.y, c1.z, c2.x, c2.y, c2.z);

    let anchor = GeodeticAnchor {
        lat_deg: geodetic.lat_rad.to_degrees(),
        lon_deg: geodetic.lon_rad.to_degrees(),
        altitude_msl_m,
    };
    let view = ExternalView {
        anchor: ExternalAnchor {
            position: LatLon::new(anchor.lat_deg, anchor.lon_deg),
            altitude_meters: anchor.altitude_msl_m,
        },
        view: Matrix4::from(rot),
        frustum,
    };
    Ok((anchor, view))
}

/// Builds a [`ConformalFrameStamp`] directly from a world [`CameraPoseEcef`].
pub fn conformal_stamp_from_pose(
    instant: SimInstant,
    pose: &CameraPoseEcef,
    frustum: EyeFrustum,
    geoid_undulation_m: f64,
) -> Result<ConformalFrameStamp, MapAdapterError> {
    let (anchor, external_view) = external_view_from_pose(pose, frustum, geoid_undulation_m)?;
    let projection_fingerprint = fingerprint_external_view(&external_view);
    Ok(ConformalFrameStamp {
        instant,
        anchor,
        external_view,
        geoid_undulation_m,
        projection_fingerprint,
    })
}

/// Builds a [`ConformalFrameStamp`] from [`VehicleState`] and a [`CameraMount`].
pub fn conformal_stamp_from_vehicle(
    instant: SimInstant,
    vehicle: &VehicleState,
    mount: &CameraMount,
    cg_body_m: DVec3,
    frustum: EyeFrustum,
    geoid_undulation_m: f64,
) -> Result<ConformalFrameStamp, MapAdapterError> {
    let pose = mount.pose(vehicle, cg_body_m);
    conformal_stamp_from_pose(instant, &pose, frustum, geoid_undulation_m)
}

/// Projects a geodetic world point `(lat, lon, h_msl)` onto target pixel coordinates
/// `([x_px, y_px], reversed_z_depth)` through `stamp`'s exact spherical camera frame,
/// guaranteeing zero pixel split between `maplibre` terrain and conformal overlays.
#[must_use]
pub fn project_geodetic_to_screen(
    stamp: &ConformalFrameStamp,
    point: &Geodetic,
    point_geoid_undulation_m: f64,
    viewport_px: [u32; 2],
) -> Option<([f32; 2], f32)> {
    let [w_px, h_px] = viewport_px;
    if w_px == 0 || h_px == 0 {
        return None;
    }
    let body = Body::EARTH;
    let anchor_ll = stamp.external_view.anchor.position;
    let point_ll = LatLon::new(point.lat_rad.to_degrees(), point.lon_rad.to_degrees());
    let point_msl = point.height_m - point_geoid_undulation_m;

    // Position on `maplibre`'s exact mean sphere in globe world metres, then into
    // `stamp.anchor`'s local ENU tangent frame.
    let anchor_u = lat_lon_to_unit_sphere(anchor_ll);
    let point_u = lat_lon_to_unit_sphere(point_ll);
    // Horizon / far-hemisphere guard (`dot <= 0` is behind the globe center).
    if anchor_u.x * point_u.x + anchor_u.y * point_u.y + anchor_u.z * point_u.z <= 0.0 {
        return None;
    }
    let r_anchor = body.radius_meters + stamp.anchor.altitude_msl_m;
    let r_point = body.radius_meters + point_msl;
    let delta_globe = DVec3::new(
        point_u.x * r_point - anchor_u.x * r_anchor,
        point_u.y * r_point - anchor_u.y * r_anchor,
        point_u.z * r_point - anchor_u.z * r_anchor,
    );

    let (lat, lon) = (
        stamp.anchor.lat_deg.to_radians(),
        stamp.anchor.lon_deg.to_radians(),
    );
    let east = DVec3::new(lon.cos(), 0.0, -lon.sin());
    let up = DVec3::new(lat.cos() * lon.sin(), lat.sin(), lat.cos() * lon.cos());
    let north = up.cross(east);
    let local_enu = cgmath::Vector4::new(
        delta_globe.dot(east),
        delta_globe.dot(north),
        delta_globe.dot(up),
        1.0,
    );

    let eye = stamp.external_view.view * local_enu;
    // OpenGL camera looks along -Z; points at or behind `near` are clipped.
    let depth_forward = -eye.z;
    if !depth_forward.is_finite() || depth_forward <= stamp.external_view.frustum.near {
        return None;
    }
    let clip = stamp.external_view.frustum.projection() * eye;
    if clip.w.abs() <= f64::EPSILON {
        return None;
    }
    let ndc_x = clip.x / clip.w;
    let ndc_y = clip.y / clip.w;
    let x_px = ((ndc_x + 1.0) * 0.5 * f64::from(w_px)) as f32;
    let y_px = ((1.0 - ndc_y) * 0.5 * f64::from(h_px)) as f32;
    let rev_z = (stamp.external_view.frustum.near / depth_forward).clamp(0.0, 1.0) as f32;
    Some(([x_px, y_px], rev_z))
}

fn fingerprint_external_view(view: &ExternalView) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    view.anchor.position.latitude.to_bits().hash(&mut hasher);
    view.anchor.position.longitude.to_bits().hash(&mut hasher);
    view.anchor.altitude_meters.to_bits().hash(&mut hasher);
    let cols: &[[f64; 4]; 4] = view.view.as_ref();
    for col in cols {
        for v in col {
            v.to_bits().hash(&mut hasher);
        }
    }
    for v in [
        view.frustum.left,
        view.frustum.right,
        view.frustum.top,
        view.frustum.bottom,
        view.frustum.near,
        view.frustum.far,
    ] {
        v.to_bits().hash(&mut hasher);
    }
    hasher.finish()
}

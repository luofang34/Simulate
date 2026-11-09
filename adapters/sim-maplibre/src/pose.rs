//! Geodetic-local 6-DoF [`ExternalView`] bridge and conformal frame stamps.
//!
//! Bridges any body reference ellipsoid ([`BodyEllipsoid`]: `WGS84`, `MOON_IAU`,
//! `MARS_IAU`, or custom) into `maplibre`'s spherical globe ([`Body`]) strictly
//! through planetographic `(lat, lon, h_datum)` and quaternion-derived local ENU
//! rotation matrices — never by linearly mapping oblate body-fixed Cartesian coordinates
//! into unit-sphere coordinates, and never via Euler `forward.cross(up)` decomposition.

use cgmath::{Matrix3, Matrix4, Rad};
use glam::{DMat3, DMat4, DVec3};
use maplibre::{
    coords::LatLon,
    projection::body::Body,
    render::{
        camera::{EyeFrustum, OPENGL_TO_WGPU_MATRIX, REVERSED_Z},
        view_state::{ExternalAnchor, ExternalView},
    },
};
use sim_core::{CameraMount, CameraPoseEcef, SimInstant, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, SensorIntrinsics};

use crate::MapAdapterError;

mod globe;
pub use globe::{anchor_enu_from_local_enu, globe_ned_between_geodetic, local_enu_from_geodetic};
pub(crate) use globe::{geodetic_zero_from_globe_vec, sphere_enu_basis, unit_sphere_from_geodetic};

/// Default far-plane clip distance in local metres (`350 km`, safely past planetary horizons).
pub const DEFAULT_FAR_CLIP_M: f64 = 350_000.0;

/// Default near-plane clip distance in local metres (`0.25 m`, safe at `0.5 m` runway eye height).
pub const DEFAULT_NEAR_CLIP_M: f64 = 0.25;

/// Spherical `maplibre` globe figure for the IAU Moon (`R = 1,737,400 m`, spanning the
/// Selenean summit `+10,786 m` down to the South Pole-Aitken basin `-9,120 m`).
pub const LUNAR_RENDER_BODY: Body = Body {
    radius_meters: 1_737_400.0,
    highest_ground_meters: 10_786.0,
    lowest_ground_meters: -9_120.0,
};

/// Spherical `maplibre` globe figure for IAU Mars (`R₁ = (2a + b) / 3 = 3,389,526.67 m`,
/// spanning Olympus Mons `+21,287 m` down to Hellas Planitia `-8,200 m`).
pub const MARS_RENDER_BODY: Body = Body {
    radius_meters: BodyEllipsoid::MARS_IAU.mean_radius_m(),
    highest_ground_meters: 21_287.0,
    lowest_ground_meters: -8_200.0,
};

/// Default spherical [`Body`] matching `physics_ellipsoid` when rendered on a `maplibre` globe.
#[must_use]
pub fn default_render_body(physics_ellipsoid: BodyEllipsoid) -> Body {
    if physics_ellipsoid == BodyEllipsoid::WGS84 {
        Body::EARTH
    } else if physics_ellipsoid == BodyEllipsoid::MOON_IAU {
        LUNAR_RENDER_BODY
    } else if physics_ellipsoid == BodyEllipsoid::MARS_IAU {
        MARS_RENDER_BODY
    } else {
        let r = physics_ellipsoid.mean_radius_m();
        let relief = (r * 0.01).max(1_000.0);
        Body {
            radius_meters: r,
            highest_ground_meters: relief,
            lowest_ground_meters: -relief,
        }
    }
}

/// Planetographic camera anchor on a celestial body with height above its vertical datum.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeodeticAnchor {
    /// Planetographic (geodetic) latitude in degrees (`[-90, 90]`).
    pub lat_deg: f64,
    /// East-positive longitude in degrees (`[-180, 180]`).
    pub lon_deg: f64,
    /// Altitude in metres above the body's vertical datum (MSL geoid on Earth, reference
    /// sphere/ellipsoid or areoid/selenoid elsewhere; may be negative in deep basins).
    pub altitude_msl_m: f64,
}

/// Atomic pairing of a rendered underlay frame with the exact camera and body state that drew it.
#[derive(Clone, Copy, Debug)]
pub struct ConformalFrameStamp {
    /// Simulation timestamp when the pose was sampled.
    pub instant: SimInstant,
    /// Reference ellipsoid of the vehicle/camera body-fixed Cartesian coordinates.
    pub physics_ellipsoid: BodyEllipsoid,
    /// Spherical globe figure on which `maplibre` renders this frame's terrain and overlays.
    pub render_body: Body,
    /// Geodetic camera origin used for the render pass.
    pub anchor: GeodeticAnchor,
    /// Exact `ExternalView` submitted to `maplibre` for this texture frame.
    pub external_view: ExternalView,
    /// Equipotential undulation (`h_ellipsoid - h_datum`, metres) applied to this frame.
    pub geoid_undulation_m: f64,
    /// Deterministic 64-bit hash of the view, frustum, and body radius for dirty-gating.
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

/// Converts a body-fixed [`CameraPoseEcef`] on `physics_ellipsoid` into a quaternion-direct
/// [`ExternalView`] on `render_body`.
///
/// `geoid_undulation_m` is `h_ellipsoid - h_datum` (`0.0` when heights are already above the
/// reference ellipsoid/sphere). Valid across all 6-DoF attitudes (`±90°` zenith/nadir, steep
/// roll, inverted) and deep basins down to `render_body.lowest_ground_meters`.
pub fn external_view_from_pose_on(
    physics_ellipsoid: BodyEllipsoid,
    render_body: Body,
    pose: &CameraPoseEcef,
    frustum: EyeFrustum,
    geoid_undulation_m: f64,
) -> Result<(GeodeticAnchor, ExternalView), MapAdapterError> {
    if !pose.position_ecef.is_finite()
        || !pose.orientation.is_finite()
        || !geoid_undulation_m.is_finite()
        || !render_body.radius_meters.is_finite()
        || render_body.radius_meters <= 0.0
    {
        return Err(MapAdapterError::InvalidPose {
            detail: "camera body-fixed position, quaternion, undulation, or body radius is invalid",
        });
    }
    let geodetic = physics_ellipsoid.fixed_to_geodetic(pose.position_ecef);
    let altitude_msl_m = geodetic.height_m - geoid_undulation_m;
    if altitude_msl_m <= render_body.lowest_ground_meters {
        return Err(MapAdapterError::InvalidPose {
            detail: "camera altitude is below the target body's lowest ground floor",
        });
    }
    let local = LocalFrame::at(&geodetic);
    let ned_to_ecef = local.ned_to_ecef();
    // ENU basis vectors in body-fixed axes: East = NED +Y, North = NED +X, Up = -NED +Z.
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

/// Earth shorthand for [`external_view_from_pose_on`] (`BodyEllipsoid::WGS84` + `Body::EARTH`).
pub fn external_view_from_pose(
    pose: &CameraPoseEcef,
    frustum: EyeFrustum,
    geoid_undulation_m: f64,
) -> Result<(GeodeticAnchor, ExternalView), MapAdapterError> {
    external_view_from_pose_on(
        BodyEllipsoid::WGS84,
        Body::EARTH,
        pose,
        frustum,
        geoid_undulation_m,
    )
}

/// Builds a [`ConformalFrameStamp`] on an explicit `(physics_ellipsoid, render_body)` pair.
pub fn conformal_stamp_from_pose_on(
    instant: SimInstant,
    physics_ellipsoid: BodyEllipsoid,
    render_body: Body,
    pose: &CameraPoseEcef,
    frustum: EyeFrustum,
    geoid_undulation_m: f64,
) -> Result<ConformalFrameStamp, MapAdapterError> {
    let (anchor, external_view) = external_view_from_pose_on(
        physics_ellipsoid,
        render_body,
        pose,
        frustum,
        geoid_undulation_m,
    )?;
    let projection_fingerprint = fingerprint_external_view(&external_view, render_body);
    Ok(ConformalFrameStamp {
        instant,
        physics_ellipsoid,
        render_body,
        anchor,
        external_view,
        geoid_undulation_m,
        projection_fingerprint,
    })
}

/// Earth shorthand for [`conformal_stamp_from_pose_on`] (`BodyEllipsoid::WGS84` + `Body::EARTH`).
pub fn conformal_stamp_from_pose(
    instant: SimInstant,
    pose: &CameraPoseEcef,
    frustum: EyeFrustum,
    geoid_undulation_m: f64,
) -> Result<ConformalFrameStamp, MapAdapterError> {
    conformal_stamp_from_pose_on(
        instant,
        BodyEllipsoid::WGS84,
        Body::EARTH,
        pose,
        frustum,
        geoid_undulation_m,
    )
}

/// Builds a [`ConformalFrameStamp`] from [`VehicleState`] and a [`CameraMount`] on an
/// explicit `(physics_ellipsoid, render_body)` pair.
#[allow(clippy::too_many_arguments)]
pub fn conformal_stamp_from_vehicle_on(
    instant: SimInstant,
    physics_ellipsoid: BodyEllipsoid,
    render_body: Body,
    vehicle: &VehicleState,
    mount: &CameraMount,
    cg_body_m: DVec3,
    frustum: EyeFrustum,
    geoid_undulation_m: f64,
) -> Result<ConformalFrameStamp, MapAdapterError> {
    let pose = mount.pose(vehicle, cg_body_m);
    conformal_stamp_from_pose_on(
        instant,
        physics_ellipsoid,
        render_body,
        &pose,
        frustum,
        geoid_undulation_m,
    )
}

/// Earth shorthand for [`conformal_stamp_from_vehicle_on`] (`BodyEllipsoid::WGS84` + `Body::EARTH`).
pub fn conformal_stamp_from_vehicle(
    instant: SimInstant,
    vehicle: &VehicleState,
    mount: &CameraMount,
    cg_body_m: DVec3,
    frustum: EyeFrustum,
    geoid_undulation_m: f64,
) -> Result<ConformalFrameStamp, MapAdapterError> {
    conformal_stamp_from_vehicle_on(
        instant,
        BodyEllipsoid::WGS84,
        Body::EARTH,
        vehicle,
        mount,
        cg_body_m,
        frustum,
        geoid_undulation_m,
    )
}

/// Projects a geodetic world point `(lat, lon, h_ellipsoid)` onto target pixel coordinates
/// `([x_px, y_px], reversed_z_depth)` through `stamp`'s exact spherical camera frame,
/// guaranteeing zero pixel split between `maplibre` terrain and conformal overlays on any body.
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
    let local = local_enu_from_geodetic(stamp, point, point_geoid_undulation_m)?;
    let local_enu = cgmath::Vector4::new(local.x, local.y, local.z, 1.0);

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

impl ConformalFrameStamp {
    /// Returns `self` with its camera anchor raised to at least `min_altitude_msl_m` on
    /// `self.render_body`, keeping its planetographic `(lat, lon)` and 6-DoF view rotation
    /// unchanged so pitch, roll, heading, and horizon projection remain strictly conformal.
    #[must_use]
    pub fn with_min_altitude_msl(mut self, min_altitude_msl_m: f64) -> Self {
        if !min_altitude_msl_m.is_finite() || min_altitude_msl_m <= self.anchor.altitude_msl_m {
            return self;
        }
        self.anchor.altitude_msl_m = min_altitude_msl_m;
        self.external_view.anchor.altitude_meters = min_altitude_msl_m;
        self.projection_fingerprint =
            fingerprint_external_view(&self.external_view, self.render_body);
        self
    }
}

/// Exact GPU clip transform of `stamp`'s anchor ENU frame: WebGPU depth range with
/// reversed Z, matching `maplibre`'s terrain and overlay depth test (`GreaterEqual`).
#[must_use]
pub fn clip_from_anchor_enu(stamp: &ConformalFrameStamp) -> DMat4 {
    let view = &stamp.external_view;
    let m = REVERSED_Z * OPENGL_TO_WGPU_MATRIX * view.frustum.projection() * view.view;
    let cols: &[[f64; 4]; 4] = m.as_ref();
    DMat4::from_cols_array_2d(cols)
}

fn fingerprint_external_view(view: &ExternalView, body: Body) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    body.radius_meters.to_bits().hash(&mut hasher);
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

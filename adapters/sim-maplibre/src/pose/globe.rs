//! Spherical `maplibre` globe tangent-frame geometry for [`super::ConformalFrameStamp`].

use glam::{DMat3, DVec3};
use maplibre::{
    coords::LatLon,
    projection::{body::Body, globe::lat_lon_to_unit_sphere},
};
use sim_geo::Geodetic;

use super::ConformalFrameStamp;

/// Position of `point` in `stamp`'s local East-North-Up frame, measured on `stamp.render_body`
/// exactly as `maplibre` draws its terrain; `None` past the horizon hemisphere.
#[must_use]
pub fn local_enu_from_geodetic(
    stamp: &ConformalFrameStamp,
    point: &Geodetic,
    point_geoid_undulation_m: f64,
) -> Option<DVec3> {
    globe_enu_between_geodetic(
        stamp.render_body,
        stamp.anchor.lat_deg,
        stamp.anchor.lon_deg,
        stamp.anchor.altitude_msl_m,
        point,
        point_geoid_undulation_m,
    )
}

/// Position of `point` in `eye`'s local North-East-Down frame on `render_body`'s sphere,
/// matching [`TruthSceneMap`](crate::TruthSceneMap) and [`super::project_geodetic_to_screen`]
/// on any celestial body with zero ellipsoid-vs-sphere anisotropy skew; `None` past the
/// horizon hemisphere.
#[must_use]
pub fn globe_ned_between_geodetic(
    render_body: Body,
    eye: &Geodetic,
    eye_geoid_undulation_m: f64,
    point: &Geodetic,
    point_geoid_undulation_m: f64,
) -> Option<DVec3> {
    let enu = globe_enu_between_geodetic(
        render_body,
        eye.lat_rad.to_degrees(),
        eye.lon_rad.to_degrees(),
        eye.height_m - eye_geoid_undulation_m,
        point,
        point_geoid_undulation_m,
    )?;
    Some(DVec3::new(enu.y, enu.x, -enu.z))
}

/// Rotation taking vectors in the sphere's ENU frame at `(lat_deg, lon_deg)` into
/// `stamp`'s anchor ENU frame, so a model oriented locally keeps its attitude anywhere.
#[must_use]
pub fn anchor_enu_from_local_enu(stamp: &ConformalFrameStamp, lat_deg: f64, lon_deg: f64) -> DMat3 {
    let [ae, an, au] = sphere_enu_basis(stamp.anchor.lat_deg, stamp.anchor.lon_deg);
    let [pe, pn, pu] = sphere_enu_basis(lat_deg, lon_deg);
    let anchor_from_globe = DMat3::from_cols(ae, an, au).transpose();
    anchor_from_globe * DMat3::from_cols(pe, pn, pu)
}

fn globe_enu_between_geodetic(
    body: Body,
    anchor_lat_deg: f64,
    anchor_lon_deg: f64,
    anchor_altitude_msl_m: f64,
    point: &Geodetic,
    point_geoid_undulation_m: f64,
) -> Option<DVec3> {
    if !body.radius_meters.is_finite() || body.radius_meters <= 0.0 {
        return None;
    }
    let anchor_ll = LatLon::new(anchor_lat_deg, anchor_lon_deg);
    let point_ll = LatLon::new(point.lat_rad.to_degrees(), point.lon_rad.to_degrees());
    let point_msl = point.height_m - point_geoid_undulation_m;
    let anchor_u = lat_lon_to_unit_sphere(anchor_ll);
    let point_u = lat_lon_to_unit_sphere(point_ll);
    // `dot <= 0` lies behind the globe center.
    if anchor_u.x * point_u.x + anchor_u.y * point_u.y + anchor_u.z * point_u.z <= 0.0 {
        return None;
    }
    let r_anchor = body.radius_meters + anchor_altitude_msl_m;
    let r_point = body.radius_meters + point_msl;
    let delta_globe = DVec3::new(
        point_u.x * r_point - anchor_u.x * r_anchor,
        point_u.y * r_point - anchor_u.y * r_anchor,
        point_u.z * r_point - anchor_u.z * r_anchor,
    );
    let [east, north, up] = sphere_enu_basis(anchor_lat_deg, anchor_lon_deg);
    Some(DVec3::new(
        delta_globe.dot(east),
        delta_globe.dot(north),
        delta_globe.dot(up),
    ))
}

/// Unit-sphere coordinates (`+Y` polar) of `geo`'s `(lat, lon)` in `maplibre`'s globe frame.
pub(crate) fn unit_sphere_from_geodetic(geo: &Geodetic) -> DVec3 {
    let p = lat_lon_to_unit_sphere(LatLon::new(geo.lat_deg(), geo.lon_deg()));
    DVec3::new(p.x, p.y, p.z)
}

/// Zero-height [`Geodetic`] at the `(lat, lon)` direction of globe vector `v` (`+Y` polar).
pub(crate) fn geodetic_zero_from_globe_vec(v: DVec3) -> Geodetic {
    let u = v.try_normalize().unwrap_or(DVec3::Y);
    let lat_rad = u.y.clamp(-1.0, 1.0).asin();
    let lon_rad = if u.x.hypot(u.z) > 1e-15 {
        u.x.atan2(u.z)
    } else {
        0.0
    };
    Geodetic {
        lat_rad,
        lon_rad,
        height_m: 0.0,
    }
}

/// Sphere tangent basis `[east, north, up]` in `maplibre` globe coordinates (`+Y` polar).
pub(crate) fn sphere_enu_basis(lat_deg: f64, lon_deg: f64) -> [DVec3; 3] {
    let (lat, lon) = (lat_deg.to_radians(), lon_deg.to_radians());
    let east = DVec3::new(lon.cos(), 0.0, -lon.sin());
    let up = DVec3::new(lat.cos() * lon.sin(), lat.sin(), lat.cos() * lon.cos());
    [east, up.cross(east), up]
}

//! Host symbology drawn over 2D instrument charts as one [`OverlayMesh`]: compass range
//! ring, approach centreline feather, traffic chevrons, ownship ground-track predictor,
//! heading lubber line, and a filled top-down ownship silhouette.
//!
//! Positions and local screen azimuths use `maplibre`'s vertical-perspective
//! [`GlobeCameraState`] with horizon occlusion culling and spherical geodesy, so marks
//! register accurately on the globe all the way across `±90°`.

use cgmath::{InnerSpace, Point2, Vector3};
use glam::{DQuat, DVec3};
use maplibre::{
    coords::LatLon,
    overlay::{OverlayMesh, OverlayVertex},
    projection::{
        body::Body,
        globe::{
            camera::{GlobeCameraOptions, GlobeCameraState},
            scale,
        },
    },
    window::PhysicalSize,
};
use sim_scenery::LandingRunwayGeometry;

use crate::{
    ChartCameraSpec, DynamicEntity3D, DynamicEntityKind,
    efb_map::effective_globe_zoom,
    geodesy::{destination, wrap_lon_deg},
};

mod marks;
mod runway;

pub(crate) use marks::{
    append_sigma_rings, append_truth_mark, screen_positions, unproject_screen_position,
};

const TILE_PX: f64 = 512.0;
/// `HeadlessMap`'s default vertical field of view (`2 * atan(1/3)` radians).
const GLOBE_FOV_RAD: f64 = 0.643_501_108_793_284_4;
/// Chart symbology depth: in front of every 2D chart layer under reversed-Z.
const DEPTH: f32 = 0.5;
const RING: [f32; 4] = [148.0 / 255.0, 163.0 / 255.0, 184.0 / 255.0, 0.42];
/// The ground-track predictor: a pale cyan, since magenta marks the active leg and white
/// the legs to come, and paler than the cyan heading lubber line.
pub(super) const TRACK_VECTOR: [f32; 4] = [0.62, 0.93, 1.0, 0.95];
const LUBBER_CYAN: [f32; 4] = [0.0, 0.88, 1.0, 0.85];
const TRAFFIC_CYAN: [f32; 4] = [0.0, 0.92, 0.92, 0.95];
pub(super) const FEATHER_CYAN: [f32; 4] = [0.22, 0.85, 1.0, 0.9];
pub(super) const FEATHER_WHITE: [f32; 4] = [0.95, 0.97, 1.0, 0.9];
pub(super) const HALO: [f32; 4] = [0.02, 0.04, 0.07, 0.96];
const OWNSHIP_WHITE: [f32; 4] = [0.98, 0.99, 1.0, 1.0];
const WINGTIP_CYAN: [f32; 4] = [0.0, 0.88, 1.0, 1.0];

/// Ownship as the chart shows it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChartOwnship {
    /// Latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees.
    pub lon_deg: f64,
    /// True heading, degrees clockwise from north.
    pub true_heading_deg: f64,
    /// Ground velocity `[east, north]`, metres per second.
    pub ground_velocity_en_mps: [f64; 2],
}

/// Everything one chart frame's symbology depends on.
#[derive(Clone, Copy, Debug)]
pub struct ChartSymbologyInput<'a> {
    /// The chart camera the symbology must register with.
    pub camera: ChartCameraSpec,
    /// Chart viewport, physical pixels.
    pub viewport_px: [u32; 2],
    /// Ownship, when its position is known.
    pub ownship: Option<ChartOwnship>,
    /// Selected landing runway, feathered with its extended approach centreline.
    pub runway: Option<&'a LandingRunwayGeometry>,
    /// Belief traffic.
    pub traffic: &'a [DynamicEntity3D],
}

/// Vertical-perspective globe projection matching `HeadlessMap`'s chart camera.
pub(super) struct ChartProjection {
    globe: GlobeCameraState,
    half: [f64; 2],
    phys_width: f64,
    phys_height: f64,
    logical_to_phys: [f64; 2],
    pub(super) scale: f32,
}

impl ChartProjection {
    fn new(camera: &ChartCameraSpec, viewport_px: [u32; 2], pixel_ratio: f64) -> Option<Self> {
        let [w, h] = viewport_px;
        let phys = PhysicalSize::new(w, h)?;
        let ratio = if pixel_ratio.is_finite() {
            pixel_ratio.clamp(0.5, 4.0)
        } else {
            1.0
        };
        let logical = phys.to_logical(ratio);
        let (phys_width, phys_height) = (f64::from(w), f64::from(h));
        let (logical_w, logical_h) = (f64::from(logical.width()), f64::from(logical.height()));
        let finite = [
            camera.center_lat_deg,
            camera.center_lon_deg,
            camera.zoom,
            camera.bearing_deg,
            camera.pitch_deg,
        ]
        .iter()
        .all(|v| v.is_finite());
        if !finite {
            return None;
        }
        let center_lat = camera.center_lat_deg.clamp(-90.0, 90.0);
        let center_lon = wrap_lon_deg(camera.center_lon_deg);
        let eff_zoom = effective_globe_zoom(camera.zoom, center_lat);
        let globe = GlobeCameraState::new(GlobeCameraOptions {
            width: logical_w,
            height: logical_h,
            field_of_view_degrees: GLOBE_FOV_RAD.to_degrees(),
            center: LatLon::new(center_lat, center_lon),
            world_size: TILE_PX * 2_f64.powf(eff_zoom),
            bearing_degrees: camera.bearing_deg,
            pitch_degrees: camera.pitch_deg,
            roll_degrees: 0.0,
            center_offset: Point2::new(0.0, 0.0),
            body: Body::EARTH,
            target_elevation_meters: 0.0,
            radius_pixels: Some(scale::radius_pixels(eff_zoom, center_lat)),
        })
        .ok()?;
        Some(Self {
            globe,
            half: [phys_width * 0.5, phys_height * 0.5],
            phys_width,
            phys_height,
            logical_to_phys: [phys_width / logical_w, phys_height / logical_h],
            scale: ratio as f32,
        })
    }

    pub(super) fn project(&self, lat_deg: f64, lon_deg: f64) -> Option<[f32; 2]> {
        if !lat_deg.is_finite() || !lon_deg.is_finite() {
            return None;
        }
        let loc = LatLon::new(lat_deg.clamp(-90.0, 90.0), wrap_lon_deg(lon_deg));
        if self.globe.is_location_occluded(loc) {
            return None;
        }
        let p = self.globe.location_to_screen(loc, 0.0);
        let [sx, sy] = self.logical_to_phys;
        let (x, y) = ((p.x * sx) as f32, (p.y * sy) as f32);
        (x.is_finite() && y.is_finite()).then_some([x, y])
    }

    pub(super) fn unproject(&self, at_px: [f32; 2]) -> Option<[f64; 2]> {
        let [px, py] = at_px;
        let [sx, sy] = self.logical_to_phys;
        if !px.is_finite() || !py.is_finite() || sx <= 0.0 || sy <= 0.0 {
            return None;
        }
        let logical = Point2::new(f64::from(px) / sx, f64::from(py) / sy);
        if !self.globe.is_point_on_map_surface(logical) {
            return None;
        }
        let loc = self.globe.screen_point_to_location(logical)?;
        (loc.latitude.is_finite() && loc.longitude.is_finite())
            .then(|| [loc.latitude.clamp(-90.0, 90.0), wrap_lon_deg(loc.longitude)])
    }

    /// Screen angle (radians, clockwise from screen-up) of `true_azimuth_deg` at `(lat_deg, lon_deg)`.
    fn screen_angle_at(&self, lat_deg: f64, lon_deg: f64, true_azimuth_deg: f64) -> f32 {
        if !lat_deg.is_finite() || !lon_deg.is_finite() || !true_azimuth_deg.is_finite() {
            return 0.0;
        }
        let lat = lat_deg.clamp(-90.0, 90.0).to_radians();
        let lon = wrap_lon_deg(lon_deg).to_radians();
        let (sin_lat, cos_lat) = lat.sin_cos();
        let (sin_lon, cos_lon) = lon.sin_cos();
        let surface = Vector3::new(sin_lon * cos_lat, sin_lat, cos_lon * cos_lat);
        let east = Vector3::new(cos_lon, 0.0, -sin_lon);
        let north = Vector3::new(-sin_lon * sin_lat, cos_lat, -cos_lon * sin_lat);
        let (sin_az, cos_az) = true_azimuth_deg.to_radians().sin_cos();
        let dir = north * cos_az + east * sin_az;
        let vp = self.globe.view_projection();
        let c0 = vp * surface.extend(1.0);
        let dc = vp * dir.extend(0.0);
        let dx = self.phys_width * (dc.x * c0.w - c0.x * dc.w);
        let dy_up = self.phys_height * (dc.y * c0.w - c0.y * dc.w);
        if c0.w > 0.0 && dx.hypot(dy_up) > 1e-18 {
            return dx.atan2(dy_up) as f32;
        }
        self.globe
            .world_direction_to_view(dir)
            .filter(|v| v.magnitude2() > 0.0)
            .map_or(0.0, |v| v.x.atan2(v.y) as f32)
    }

    fn contains(&self, p: [f32; 2], margin: f32) -> bool {
        let [x, y] = p;
        let [hx, hy] = self.half;
        let m = margin * self.scale;
        x >= -m && y >= -m && x <= (hx * 2.0) as f32 + m && y <= (hy * 2.0) as f32 + m
    }
}

/// Builds the chart's symbology mesh at `1.0` physical pixel per logical pixel.
#[must_use]
pub fn build_chart_symbology(input: &ChartSymbologyInput<'_>) -> OverlayMesh {
    build_chart_symbology_scaled(input, 1.0)
}

/// Builds the chart's symbology mesh registered to a `HeadlessMap` drawing at `pixel_ratio`
/// physical pixels per logical pixel (`2.0` on Retina).
#[must_use]
pub fn build_chart_symbology_scaled(
    input: &ChartSymbologyInput<'_>,
    pixel_ratio: f64,
) -> OverlayMesh {
    let Some(projection) = ChartProjection::new(&input.camera, input.viewport_px, pixel_ratio)
    else {
        return OverlayMesh::default();
    };
    let mut mesh = OverlayMesh::default();
    let ownship_px = input
        .ownship
        .and_then(|own| projection.project(own.lat_deg, own.lon_deg))
        .filter(|p| projection.contains(*p, 0.0));
    let [hx, hy] = projection.half;
    let ring_centre = ownship_px.unwrap_or([hx as f32, hy as f32]);
    let ring_ref = input.ownship.filter(|_| ownship_px.is_some()).map_or(
        (input.camera.center_lat_deg, input.camera.center_lon_deg),
        |own| (own.lat_deg, own.lon_deg),
    );
    append_range_ring(
        &mut mesh,
        &projection,
        ring_centre,
        ring_ref,
        (hx.min(hy) * 0.72) as f32,
    );
    if let Some(runway) = input.runway {
        runway::append_approach_feather(&mut mesh, &projection, runway);
    }
    for entity in input.traffic {
        append_traffic(&mut mesh, &projection, entity);
    }
    if let (Some(own), Some(px)) = (input.ownship, ownship_px) {
        append_ownship(&mut mesh, &projection, &own, px);
    }
    mesh
}

fn append_range_ring(
    mesh: &mut OverlayMesh,
    projection: &ChartProjection,
    c: [f32; 2],
    (ref_lat, ref_lon): (f64, f64),
    r: f32,
) {
    const STEPS: u32 = 96;
    let s = projection.scale;
    let [cx, cy] = c;
    let point = |angle: f32, radius: f32| [cx + radius * angle.sin(), cy - radius * angle.cos()];
    for i in 0..STEPS {
        let a0 = i as f32 / STEPS as f32 * std::f32::consts::TAU;
        let a1 = i.wrapping_add(1) as f32 / STEPS as f32 * std::f32::consts::TAU;
        push_segment(mesh, point(a0, r), point(a1, r), 1.4 * s, RING);
    }
    for azimuth in (0..360).step_by(30) {
        let angle = projection.screen_angle_at(ref_lat, ref_lon, f64::from(azimuth));
        let length = (if azimuth % 90 == 0 { 14.0 } else { 7.0 }) * s;
        let width = (if azimuth % 90 == 0 { 2.2 } else { 1.4 }) * s;
        push_segment(mesh, point(angle, r - length), point(angle, r), width, RING);
    }
    let north = projection.screen_angle_at(ref_lat, ref_lon, 0.0);
    let [r0, r1, r2, _] = RING;
    push_triangle(
        mesh,
        [
            point(north, r + 12.0 * s),
            point(north - 0.035, r + 2.0 * s),
            point(north + 0.035, r + 2.0 * s),
        ],
        [r0, r1, r2, 0.8],
    );
}

fn append_traffic(mesh: &mut OverlayMesh, projection: &ChartProjection, entity: &DynamicEntity3D) {
    if entity.kind == DynamicEntityKind::OwnshipExterior {
        return;
    }
    let lat_deg = entity.placement.position.latitude;
    let lon_deg = entity.placement.position.longitude;
    let Some(p) = projection.project(lat_deg, lon_deg) else {
        return;
    };
    if !projection.contains(p, 20.0) {
        return;
    }
    let s = projection.scale;
    let v = entity.velocity_enu_mps;
    let speed = v.x.hypot(v.y);
    let track_deg = if speed > 1.0 {
        v.x.atan2(v.y).to_degrees()
    } else {
        let [x, y, z, w] = entity.placement.orientation.enu_from_body;
        let fwd = DQuat::from_xyzw(x, y, z, w).normalize() * DVec3::Y;
        fwd.x.atan2(fwd.y).to_degrees()
    };
    let angle = projection.screen_angle_at(lat_deg, lon_deg, track_deg);
    let shape = [[0.0, -9.0], [6.5, 6.0], [0.0, 2.5], [-6.5, 6.0]];
    let [a, b, c, d] = shape.map(|[qx, qy]| rotate([qx * s, qy * s], angle, p));
    push_triangle(mesh, [a, b, c], TRAFFIC_CYAN);
    push_triangle(mesh, [a, c, d], TRAFFIC_CYAN);
    if speed > 1.0 {
        let [lon2, lat2] = destination([lon_deg, lat_deg], track_deg, speed * 30.0);
        if let Some(tip) = projection.project(lat2, lon2) {
            push_segment(mesh, p, tip, 1.6 * s, TRAFFIC_CYAN);
        }
    }
}

fn append_ownship(
    mesh: &mut OverlayMesh,
    projection: &ChartProjection,
    own: &ChartOwnship,
    px: [f32; 2],
) {
    let s = projection.scale;
    let [east, north] = own.ground_velocity_en_mps;
    let speed = east.hypot(north);
    if speed > 2.0 {
        let track = east.atan2(north).to_degrees();
        let [lon2, lat2] = destination([own.lon_deg, own.lat_deg], track, speed * 60.0);
        if let Some(tip) = projection.project(lat2, lon2) {
            push_segment(mesh, px, tip, 3.0 * s, HALO);
            push_segment(mesh, px, tip, 2.0 * s, TRACK_VECTOR);
        }
    }
    let angle = projection.screen_angle_at(own.lat_deg, own.lon_deg, own.true_heading_deg);
    let nose = rotate([0.0, -18.0 * s], angle, px);
    let lubber_end = rotate([0.0, -62.0 * s], angle, px);
    push_segment(mesh, nose, lubber_end, 1.8 * s, LUBBER_CYAN);
    for (polygon, _) in silhouette(s) {
        push_convex(mesh, &inflate(&polygon, 2.5 * s), angle, px, HALO);
    }
    for (polygon, color) in silhouette(s) {
        push_convex(mesh, &polygon, angle, px, color);
    }
}

/// Top-down high-wing trainer, nose up, pixels about its centre of gravity.
fn silhouette(s: f32) -> [(Vec<[f32; 2]>, [f32; 4]); 6] {
    let sp = |pts: &[[f32; 2]]| pts.iter().map(|&[x, y]| [x * s, y * s]).collect();
    [
        (
            sp(&[
                [0.0, -15.0],
                [2.3, -11.5],
                [2.2, 7.0],
                [1.0, 13.5],
                [-1.0, 13.5],
                [-2.2, 7.0],
                [-2.3, -11.5],
            ]),
            OWNSHIP_WHITE,
        ),
        (
            sp(&[[-15.0, -4.6], [15.0, -4.6], [15.0, -1.0], [-15.0, -1.0]]),
            OWNSHIP_WHITE,
        ),
        (
            sp(&[[2.0, -5.4], [12.0, -4.6], [12.0, -0.8], [2.0, 0.2]]),
            OWNSHIP_WHITE,
        ),
        (
            sp(&[[-6.5, 9.5], [6.5, 9.5], [6.0, 12.4], [-6.0, 12.4]]),
            OWNSHIP_WHITE,
        ),
        (
            sp(&[[12.8, -4.6], [15.0, -4.6], [15.0, -1.0], [12.8, -1.0]]),
            WINGTIP_CYAN,
        ),
        (
            sp(&[[-15.0, -4.6], [-12.8, -4.6], [-12.8, -1.0], [-15.0, -1.0]]),
            WINGTIP_CYAN,
        ),
    ]
}

/// Grows a convex polygon outward by about `by` pixels.
fn inflate(polygon: &[[f32; 2]], by: f32) -> Vec<[f32; 2]> {
    let n = polygon.len().max(1) as f32;
    let (sx, sy) = polygon
        .iter()
        .fold((0.0, 0.0), |(x, y), [px, py]| (x + px, y + py));
    let (cx, cy) = (sx / n, sy / n);
    polygon
        .iter()
        .map(|&[x, y]| {
            let (dx, dy) = (x - cx, y - cy);
            let len = dx.hypot(dy).max(1e-3);
            [x + dx / len * by, y + dy / len * by]
        })
        .collect()
}

fn rotate([x, y]: [f32; 2], angle: f32, [ox, oy]: [f32; 2]) -> [f32; 2] {
    let (s, c) = angle.sin_cos();
    [ox + x * c - y * s, oy + x * s + y * c]
}

fn push_convex(
    mesh: &mut OverlayMesh,
    polygon: &[[f32; 2]],
    angle: f32,
    origin: [f32; 2],
    color: [f32; 4],
) {
    let Some((&first, rest)) = polygon.split_first() else {
        return;
    };
    let a = rotate(first, angle, origin);
    for pair in rest.windows(2) {
        if let [b, c] = pair {
            push_triangle(
                mesh,
                [a, rotate(*b, angle, origin), rotate(*c, angle, origin)],
                color,
            );
        }
    }
}

fn premultiplied([r, g, b, a]: [f32; 4]) -> [f32; 4] {
    [r * a, g * a, b * a, a]
}

pub(crate) fn push_triangle(mesh: &mut OverlayMesh, points: [[f32; 2]; 3], color: [f32; 4]) {
    let Ok(base) = u32::try_from(mesh.vertices.len()) else {
        return;
    };
    let color = premultiplied(color);
    mesh.vertices.extend(points.map(|position| OverlayVertex {
        position,
        depth: DEPTH,
        color,
    }));
    mesh.indices
        .extend_from_slice(&[base, base.wrapping_add(1), base.wrapping_add(2)]);
}

pub(crate) fn push_segment(
    mesh: &mut OverlayMesh,
    a: [f32; 2],
    b: [f32; 2],
    width_px: f32,
    color: [f32; 4],
) {
    let [ax, ay] = a;
    let [bx, by] = b;
    let len = (bx - ax).hypot(by - ay);
    if len <= 1e-3 {
        return;
    }
    let (nx, ny) = (
        -(by - ay) / len * width_px * 0.5,
        (bx - ax) / len * width_px * 0.5,
    );
    let [c0, c1, c2, c3] = [
        [ax + nx, ay + ny],
        [ax - nx, ay - ny],
        [bx - nx, by - ny],
        [bx + nx, by + ny],
    ];
    push_triangle(mesh, [c0, c1, c2], color);
    push_triangle(mesh, [c0, c2, c3], color);
}

#[cfg(test)]
mod tests;

//! Turntable orbit state and the studio camera / light rig framed on an asset's bounds.

use std::f64::consts::TAU;

use glam::{DMat4, DVec3, Mat4};

/// Turntable auto-orbit rate.
pub const HANGAR_AUTO_ROTATE_RAD_PER_S: f64 = 0.35;
/// Lowest orbit elevation (just above the floor plane).
const MIN_ELEVATION_RAD: f64 = -0.05;
/// Highest orbit elevation (short of straight down, where the up vector degenerates).
const MAX_ELEVATION_RAD: f64 = 1.45;
const MIN_ZOOM: f64 = 0.35;
const MAX_ZOOM: f64 = 4.0;
/// Vertical field of view of the studio camera.
const FOV_Y_RAD: f64 = 0.56;

/// Orbit camera state of the hangar preview.
///
/// The camera looks at the centre of the asset's bounding box from azimuth
/// [`HangarOrbit::azimuth_rad`] (`0` in front of the nose on `+Y`, increasing toward the
/// right wing on `+X`) and elevation [`HangarOrbit::elevation_rad`] above the horizon.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HangarOrbit {
    /// Azimuth about the vertical axis in radians, kept in `[0, 2π)`.
    pub azimuth_rad: f64,
    /// Elevation above the horizon in radians.
    pub elevation_rad: f64,
    /// Magnification relative to the auto-framed distance (`> 1` moves closer).
    pub zoom: f64,
    /// Whether [`HangarOrbit::advance`] turns the camera like a turntable.
    pub auto_rotate: bool,
}

impl Default for HangarOrbit {
    /// Front-left three-quarter view, slightly above, turning.
    fn default() -> Self {
        Self {
            azimuth_rad: TAU - 0.8,
            elevation_rad: 0.3,
            zoom: 1.0,
            auto_rotate: true,
        }
    }
}

impl HangarOrbit {
    /// Advances the turntable by `dt_s` seconds when [`HangarOrbit::auto_rotate`] is set.
    ///
    /// Steps are capped at a quarter second so a stalled frame does not jump the view.
    pub fn advance(&mut self, dt_s: f64) {
        if self.auto_rotate && dt_s.is_finite() && dt_s > 0.0 {
            let step = HANGAR_AUTO_ROTATE_RAD_PER_S * dt_s.min(0.25);
            self.azimuth_rad = (self.azimuth_rad + step).rem_euclid(TAU);
        }
    }

    /// Orbits by drag / key deltas; elevation is clamped between the floor and overhead.
    ///
    /// Leaves [`HangarOrbit::auto_rotate`] unchanged, so the caller decides whether a
    /// drag stops the turntable.
    pub fn nudge(&mut self, d_azimuth_rad: f64, d_elevation_rad: f64) {
        if d_azimuth_rad.is_finite() {
            self.azimuth_rad = (self.azimuth_rad + d_azimuth_rad).rem_euclid(TAU);
        }
        if d_elevation_rad.is_finite() {
            self.elevation_rad =
                (self.elevation_rad + d_elevation_rad).clamp(MIN_ELEVATION_RAD, MAX_ELEVATION_RAD);
        }
    }

    /// Multiplies the zoom by `factor` (`> 1` closer), clamped to a usable range.
    pub fn zoom_by(&mut self, factor: f64) {
        if factor.is_finite() && factor > 0.0 {
            self.zoom = (self.zoom * factor).clamp(MIN_ZOOM, MAX_ZOOM);
        }
    }
}

/// Axis-aligned rest-pose bounds of an asset in aircraft axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Bounds {
    pub(super) min: DVec3,
    pub(super) max: DVec3,
}

impl Default for Bounds {
    /// A light-aircraft-sized volume, used before any asset is set.
    fn default() -> Self {
        Self {
            min: DVec3::new(-5.5, -4.5, -1.2),
            max: DVec3::new(5.5, 4.0, 1.6),
        }
    }
}

/// Per-frame camera, floor, and studio light rig, all in camera-relative aircraft axes
/// (eye at the origin) so the shaders' view vector is simply `-position`.
pub(super) struct StudioRig {
    /// Camera-relative world translation (`-eye`).
    pub(super) world_from_aircraft: DMat4,
    pub(super) clip_from_world: DMat4,
    pub(super) eye: DVec3,
    pub(super) key: DVec3,
    pub(super) rim: DVec3,
    pub(super) fill: DVec3,
    pub(super) bounds: Bounds,
}

impl StudioRig {
    pub(super) fn frame(orbit: &HangarOrbit, bounds: Bounds, aspect: f64) -> Self {
        let centre = (bounds.min + bounds.max) * 0.5;
        let half = (bounds.max - bounds.min) * 0.5;
        let radius = half.length().max(0.05);
        let footprint = half.truncate().length().max(0.05);
        let zoom = if orbit.zoom.is_finite() {
            orbit.zoom.clamp(MIN_ZOOM, MAX_ZOOM)
        } else {
            1.0
        };
        let elevation = orbit
            .elevation_rad
            .clamp(MIN_ELEVATION_RAD, MAX_ELEVATION_RAD);
        // Fit the turntable footprint across the width and its projected height up the
        // frame separately: aircraft are wide and flat, so one bounding sphere framed on
        // the narrower field of view leaves them small in a landscape preview.
        let tan_v = (FOV_Y_RAD * 0.5).tan();
        let tan_h = tan_v * aspect.max(1e-3);
        let projected_height = footprint * elevation.sin().abs() + half.z * elevation.cos();
        let fit = (footprint / tan_h).max(projected_height / tan_v);
        let distance = (fit * 1.08 + footprint * 0.3) / zoom;
        let (sin_az, cos_az) = orbit.azimuth_rad.sin_cos();
        let (sin_el, cos_el) = elevation.sin_cos();
        let direction = DVec3::new(sin_az * cos_el, cos_az * cos_el, sin_el);
        let mut eye = centre + direction * distance;
        eye.z = eye.z.max(bounds.min.z + radius * 0.04);
        let forward = (centre - eye).normalize_or(DVec3::NEG_Y);
        let right = forward.cross(DVec3::Z).normalize_or(DVec3::X);
        let up = right.cross(forward);
        let view = glam::dcamera::rh::view::look_at_mat4(DVec3::ZERO, forward, DVec3::Z);
        let near = ((centre - eye).length() - radius * 1.3).max(radius * 0.01);
        let projection = glam::dcamera::rh::proj::directx::perspective_infinite_reverse(
            FOV_Y_RAD,
            aspect.max(1e-3),
            near,
        );
        Self {
            world_from_aircraft: DMat4::from_translation(-eye),
            clip_from_world: projection * view,
            eye,
            key: (-0.6 * right + 0.75 * up - 0.4 * forward).normalize(),
            rim: (0.55 * right + 0.5 * up + 0.85 * forward).normalize(),
            fill: (0.85 * right + 0.05 * up - 0.55 * forward).normalize(),
            bounds,
        }
    }

    /// The shader `Frame` block, packed little-endian.
    pub(super) fn frame_bytes(&self) -> [u8; FRAME_BYTES] {
        let world_from_clip = self.clip_from_world.inverse().as_mat4();
        let half = (self.bounds.max - self.bounds.min) * 0.5;
        let centre = (self.bounds.min + self.bounds.max) * 0.5;
        let footprint = half.truncate().length();
        let floor_z = self.bounds.min.z - footprint * 0.002;
        let spacing = 2.0_f64.powf((footprint / 3.0).max(1e-3).log2().round());
        let vec4 = |v: DVec3, w: f64| [v.x as f32, v.y as f32, v.z as f32, w as f32];
        let floats = Mat4::to_cols_array(&world_from_clip)
            .into_iter()
            .chain(vec4(self.eye, floor_z))
            .chain([
                centre.x as f32,
                centre.y as f32,
                (footprint * 1.12) as f32,
                spacing as f32,
            ])
            .chain(vec4(self.key, 3.1))
            .chain(vec4(self.rim, 2.4))
            .chain(vec4(self.fill, 0.55))
            .chain([(half.x * 1.05) as f32, (half.y * 1.05) as f32, 0.0, 0.0]);
        let mut out = [0_u8; FRAME_BYTES];
        for (chunk, value) in out.as_chunks_mut::<4>().0.iter_mut().zip(floats) {
            *chunk = value.to_le_bytes();
        }
        out
    }
}

/// Bytes of the shader `Frame` uniform block.
pub(super) const FRAME_BYTES: usize = 160;

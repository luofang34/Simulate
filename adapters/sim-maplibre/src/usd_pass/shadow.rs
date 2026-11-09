//! Ownship sun shadow: an orthographic shadow map fitted to the airframe's bounding sphere,
//! sampled by the airframe itself (wings shade the gondola and booms) and by a receiver
//! quad laid on the local ground plane under it, so a parked or taxiing aircraft sits on
//! the runway instead of floating over it.
//!
//! Terrain is drawn by `maplibre` before this pass and its depth is not readable here, so
//! the ground receiver is a plane at the ground height the host reports under ownship.

use glam::{DVec3, Mat4, Vec3};
use sim_geo::Geodetic;

use super::{AircraftInstance, AircraftLibrary, AircraftSlotView, enu_from_aircraft};
use crate::pose::{anchor_enu_from_local_enu, clip_from_anchor_enu, local_enu_from_geodetic};

pub(crate) mod gpu;

/// Bytes of the shader's `Shadow` uniform block.
pub(crate) const SHADOW_BYTES: usize = 192;
/// Shadow-map edge in texels; at the P-38G's 9 m radius one texel is under 1 cm.
pub(crate) const SHADOW_MAP_SIZE: u32 = 2048;

/// Lift of the ground receiver above the reported ground, against depth fighting.
const GROUND_LIFT_M: f64 = 0.03;
/// Above this height the shadow is too far and faint to be worth a receiver.
const MAX_GROUND_HEIGHT_M: f64 = 600.0;
/// Below this Sun elevation sine the shadow stretches toward the horizon; skip it.
const MIN_SUN_SINE: f64 = 0.035;
/// Longest ground receiver half-extent.
const MAX_RECEIVER_HALF_M: f64 = 160.0;
/// Penumbra width per metre between occluder and receiver: the Sun's 0.53° disc widened
/// by forward scatter, the softness seen in photographs.
const PENUMBRA_PER_M: f32 = 0.018;
/// Darkening of sunlit ground at full daylight; sky light still reaches the shadow.
const GROUND_STRENGTH: f32 = 0.5;

/// The ground receiver: a quad on the local ground plane, in the slot's anchor ENU.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct GroundReceiver {
    pub(crate) centre: Vec3,
    pub(crate) half_u: Vec3,
    pub(crate) half_v: Vec3,
    pub(crate) strength: f32,
}

/// One slot's ownship shadow: the light volume and the optional ground receiver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ShadowFrame {
    pub(crate) clip_from_enu: Mat4,
    pub(crate) light_from_enu: Mat4,
    pub(crate) ground: Option<GroundReceiver>,
    pub(crate) depth_range_m: f32,
    pub(crate) width_m: f32,
}

impl ShadowFrame {
    /// The shader's `Shadow` block; a `None` frame disables every lookup.
    pub(crate) fn block(frame: Option<&Self>) -> [u8; SHADOW_BYTES] {
        let zero = Mat4::ZERO.to_cols_array();
        let (clip, light) = frame.map_or((zero, zero), |f| {
            (
                f.clip_from_enu.to_cols_array(),
                f.light_from_enu.to_cols_array(),
            )
        });
        let ground = frame.and_then(|f| f.ground);
        let [cx, cy, cz] = ground.map_or([0.0; 3], |g| g.centre.to_array());
        let [ux, uy, uz] = ground.map_or([0.0; 3], |g| g.half_u.to_array());
        let [vx, vy, vz] = ground.map_or([0.0; 3], |g| g.half_v.to_array());
        let strength = ground.map_or(0.0, |g| g.strength);
        let (range, width, enabled) = frame.map_or((1.0, 1.0, 0.0), |f| {
            (f.depth_range_m.max(1e-3), f.width_m.max(1e-3), 1.0)
        });
        let texel_m = width / SHADOW_MAP_SIZE as f32;
        let floats = clip.into_iter().chain(light).chain([
            cx,
            cy,
            cz,
            strength,
            ux,
            uy,
            uz,
            range,
            vx,
            vy,
            vz,
            enabled,
            1.0 / SHADOW_MAP_SIZE as f32,
            texel_m * 2.5,
            PENUMBRA_PER_M,
            1.0 / width,
        ]);
        let mut out = [0_u8; SHADOW_BYTES];
        for (chunk, value) in out.as_chunks_mut::<4>().0.iter_mut().zip(floats) {
            *chunk = value.to_le_bytes();
        }
        out
    }
}

/// The ownship shadow of one slot, when the Sun is up and ownship has an airframe.
pub(crate) fn ownship_shadow(
    library: &AircraftLibrary,
    aircraft: &[AircraftInstance],
    view: &AircraftSlotView<'_>,
) -> Option<ShadowFrame> {
    let instance = aircraft.iter().find(|a| a.ownship)?;
    let (_, asset) = library.asset_for(true)?;
    if view.daylight <= 0.01 {
        return None;
    }
    let radius = f64::from(asset.bounding_radius_m()).max(0.5);
    let centre = enu_from_aircraft(view.stamp, &instance.placement)?
        .w_axis
        .truncate();
    let to_sun = view.sun_enu.try_normalize()?;
    let ground = ground_receiver(instance, view, centre, to_sun, radius);
    let reach = ground.map_or(0.0, |(_, along_light)| along_light);
    // The volume spans the airframe and, past it, the ground the shadow falls on; a
    // receiver beyond the far plane clamps to it and still reads as shadowed.
    let depth_range = 3.0 * radius + reach;
    let up_hint = if to_sun.z.abs() < 0.99 {
        DVec3::Z
    } else {
        DVec3::Y
    };
    let eye = centre + to_sun * (radius * 1.5);
    let view_from_enu = glam::dcamera::rh::view::look_to_mat4(eye, -to_sun, up_hint);
    let projection = glam::dcamera::rh::proj::directx::orthographic(
        -radius,
        radius,
        -radius,
        radius,
        0.05,
        depth_range,
    );
    let light_from_enu = projection * view_from_enu;
    light_from_enu.is_finite().then(|| ShadowFrame {
        clip_from_enu: clip_from_anchor_enu(view.stamp).as_mat4(),
        light_from_enu: light_from_enu.as_mat4(),
        ground: ground.map(|(g, _)| g),
        depth_range_m: depth_range as f32,
        width_m: (2.0 * radius) as f32,
    })
}

/// The receiver quad under ownship and the distance along the light from the airframe's
/// origin to the plane, when the host reports the ground and the Sun clears the horizon.
fn ground_receiver(
    instance: &AircraftInstance,
    view: &AircraftSlotView<'_>,
    centre: DVec3,
    to_sun: DVec3,
    radius: f64,
) -> Option<(GroundReceiver, f64)> {
    let ground_msl = instance.ground_height_m?;
    let height = instance.placement.altitude_meters - ground_msl;
    if !(-1.0..=MAX_GROUND_HEIGHT_M).contains(&height) {
        return None;
    }
    let position = instance.placement.position;
    let undulation = view.stamp.geoid_undulation_m;
    let below = Geodetic {
        lat_rad: position.latitude.to_radians(),
        lon_rad: position.longitude.to_radians(),
        height_m: ground_msl + undulation,
    };
    let up =
        anchor_enu_from_local_enu(view.stamp, position.latitude, position.longitude) * DVec3::Z;
    let ground = local_enu_from_geodetic(view.stamp, &below, undulation)? + up * GROUND_LIFT_M;
    let sun_sine = to_sun.dot(up);
    if sun_sine < MIN_SUN_SINE {
        return None;
    }
    let above = (centre - ground).dot(up);
    let along_light = above.max(0.0) / sun_sine;
    let shadow_centre = centre - to_sun * along_light;
    let across = up.cross(to_sun).try_normalize()?;
    let along = across.cross(up);
    let half_along = (radius * 1.15 / sun_sine).min(MAX_RECEIVER_HALF_M);
    let half_across = (radius * 1.15).min(MAX_RECEIVER_HALF_M);
    let fade = (1.0 - (height / MAX_GROUND_HEIGHT_M).clamp(0.0, 1.0)) as f32;
    let receiver = GroundReceiver {
        centre: shadow_centre.as_vec3(),
        half_u: (along * half_along).as_vec3(),
        half_v: (across * half_across).as_vec3(),
        strength: GROUND_STRENGTH * view.daylight * fade,
    };
    Some((receiver, along_light))
}

#[cfg(test)]
mod tests;

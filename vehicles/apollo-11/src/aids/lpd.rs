use glam::{DQuat, DVec3};

use crate::SurfacePlane;

/// Highest graduation scribed on the window, degrees.
const LPD_MAX_GRADUATION_DEG: f64 = 70.0;

/// A target as the Commander sees it against the window scale.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LpdReading {
    /// Graduation the target lies on: angle below LM `+z` in the `x-z`
    /// plane, rad.
    pub elevation_rad: f64,
    /// Angle of the target out of the `x-z` plane, towards `+y`, rad. The
    /// scale only reads true when this is zero, so the LM yaws to bring it
    /// there.
    pub cross_track_rad: f64,
    /// Whether the elevation falls on the `0°..=70°` scale and ahead of the
    /// window.
    pub on_scale: bool,
}

/// The Landing Point Designator: the double scale on the Commander's window
/// that maps a graduation to the point on the Moon the guidance is flying to.
///
/// Works from the eye's pose, so the eye's offset from the centre of gravity
/// is taken as the crew would see it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LpdProjector;

impl LpdProjector {
    /// The reading for `target_fixed_m` from an eye at `eye_fixed_m`, with
    /// `lm_to_fixed` the LM's attitude.
    pub fn project(
        &self,
        eye_fixed_m: DVec3,
        lm_to_fixed: DQuat,
        target_fixed_m: DVec3,
    ) -> LpdReading {
        let look = lm_to_fixed.inverse() * (target_fixed_m - eye_fixed_m);
        let elevation_rad = (-look.x).atan2(look.z);
        let cross_track_rad = look.y.atan2(look.x.hypot(look.z));
        let scale = 0.0..=LPD_MAX_GRADUATION_DEG.to_radians();
        LpdReading {
            elevation_rad,
            cross_track_rad,
            on_scale: scale.contains(&elevation_rad),
        }
    }

    /// The surface point at a reading, the inverse of [`Self::project`]: what
    /// a crew redesignation means to the guidance. `None` if that line of
    /// sight never reaches `surface`.
    pub fn redesignate(
        &self,
        eye_fixed_m: DVec3,
        lm_to_fixed: DQuat,
        reading: &LpdReading,
        surface: &SurfacePlane,
    ) -> Option<DVec3> {
        let (sin_elevation, cos_elevation) = reading.elevation_rad.sin_cos();
        let (sin_cross, cos_cross) = reading.cross_track_rad.sin_cos();
        let look = lm_to_fixed
            * DVec3::new(
                -sin_elevation * cos_cross,
                sin_cross,
                cos_elevation * cos_cross,
            );
        let descent = -look.dot(surface.normal_fixed);
        let clearance = surface.clearance_m(eye_fixed_m);
        (descent > 0.0 && clearance > 0.0).then(|| eye_fixed_m + look * (clearance / descent))
    }
}

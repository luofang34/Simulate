use glam::{DVec3, Vec3};

#[cfg(test)]
mod tests;

/// An `f64` ECEF anchor that world positions are expressed against before they
/// are narrowed to `f32` for the GPU.
///
/// An absolute ECEF coordinate is about 6.4e6 m, where an `f32` resolves only
/// about half a metre; relative to an origin within a few kilometres it
/// resolves well under a millimetre. Subtraction always happens in `f64`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FloatingOrigin {
    anchor_ecef: DVec3,
}

impl FloatingOrigin {
    /// An origin anchored at `anchor_ecef`.
    pub fn new(anchor_ecef: DVec3) -> Self {
        Self { anchor_ecef }
    }

    /// The anchor in ECEF metres.
    pub fn anchor(&self) -> DVec3 {
        self.anchor_ecef
    }

    /// `position_ecef` relative to the anchor, narrowed to `f32` after the
    /// `f64` subtraction.
    pub fn to_local(&self, position_ecef: DVec3) -> Vec3 {
        (position_ecef - self.anchor_ecef).as_vec3()
    }

    /// Moves the anchor to `focus_ecef` when the focus has drifted more than
    /// `max_offset_m` away; returns whether it moved, so cached local-space
    /// data can be rebuilt.
    pub fn rebase_if_far(&mut self, focus_ecef: DVec3, max_offset_m: f64) -> bool {
        if focus_ecef.distance_squared(self.anchor_ecef) > max_offset_m * max_offset_m {
            self.anchor_ecef = focus_ecef;
            true
        } else {
            false
        }
    }
}

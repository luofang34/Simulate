#[cfg(test)]
mod tests;

/// Pinhole camera intrinsics, in pixels.
///
/// The principal point follows `Navigate`'s convention: pixel centres sit at
/// integer coordinates, so the centre of an `800 × 600` image is
/// `(399.5, 299.5)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorIntrinsics {
    /// Image width.
    pub width: u32,
    /// Image height.
    pub height: u32,
    /// Horizontal focal length.
    pub fx: f64,
    /// Vertical focal length.
    pub fy: f64,
    /// Principal point, horizontal.
    pub cx: f64,
    /// Principal point, vertical.
    pub cy: f64,
}

impl SensorIntrinsics {
    /// Whether these describe a real image: nonzero size, positive focal
    /// lengths, and a finite principal point.
    pub fn is_usable(&self) -> bool {
        self.width > 0
            && self.height > 0
            && self.fx.is_finite()
            && self.fy.is_finite()
            && self.fx > 0.0
            && self.fy > 0.0
            && self.cx.is_finite()
            && self.cy.is_finite()
    }
}

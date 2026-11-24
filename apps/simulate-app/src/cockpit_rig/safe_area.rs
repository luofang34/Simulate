//! The window edges the system keeps for itself (a phone's sensor housing, rounded
//! corners and home indicator), and the one rule every docked control places itself by.

/// The parts of the window the system covers, each as a fraction of the window's width
/// (left, right) or height (top, bottom).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SafeAreaInsets {
    /// Covered at the left edge, as a fraction of the width.
    pub left: f32,
    /// Covered at the top edge, as a fraction of the height.
    pub top: f32,
    /// Covered at the right edge, as a fraction of the width.
    pub right: f32,
    /// Covered at the bottom edge, as a fraction of the height.
    pub bottom: f32,
}

/// The most of an axis one edge's inset may claim, so two opposite insets always leave
/// room between them.
const MAX_EDGE_FRACTION: f32 = 0.4;

impl SafeAreaInsets {
    /// A window the system covers no part of.
    pub const NONE: Self = Self {
        left: 0.0,
        top: 0.0,
        right: 0.0,
        bottom: 0.0,
    };

    /// The insets a host reports as `[left, top, right, bottom]` lengths, in the same
    /// unit as the window's `[width, height]`.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "only a browser reports insets")
    )]
    #[must_use]
    pub fn from_lengths([left, top, right, bottom]: [f64; 4], [width, height]: [f64; 2]) -> Self {
        let fraction = |length: f64, extent: f64| {
            if length.is_finite() && extent.is_finite() && extent > 0.0 {
                ((length / extent) as f32).clamp(0.0, MAX_EDGE_FRACTION)
            } else {
                0.0
            }
        };
        Self {
            left: fraction(left, width),
            top: fraction(top, height),
            right: fraction(right, width),
            bottom: fraction(bottom, height),
        }
    }

    /// The margins, in pixels of a `[width, height]` surface, a control keeps from the
    /// `[left, top, right, bottom]` edges when it would otherwise keep `standard_px`: the
    /// larger of the two at each edge, never their sum.
    #[must_use]
    pub fn margins_px(&self, [width, height]: [f32; 2], standard_px: [f32; 4]) -> [f32; 4] {
        let [left, top, right, bottom] = standard_px;
        [
            left.max(self.left * width),
            top.max(self.top * height),
            right.max(self.right * width),
            bottom.max(self.bottom * height),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_margin_is_the_larger_of_the_standard_and_the_inset_never_their_sum() {
        let insets = SafeAreaInsets::from_lengths([59.0, 0.0, 4.0, 21.0], [874.0, 402.0]);
        let margins = insets.margins_px([1748.0, 804.0], [24.0, 10.0, 24.0, 6.0]);
        assert!((margins[0] - 118.0).abs() < 0.5, "{margins:?}");
        assert!((margins[1] - 10.0).abs() < 1e-3, "{margins:?}");
        assert!((margins[2] - 24.0).abs() < 1e-3, "{margins:?}");
        assert!((margins[3] - 42.0).abs() < 0.5, "{margins:?}");
    }

    #[test]
    fn unusable_reports_leave_the_window_uncovered() {
        let insets =
            SafeAreaInsets::from_lengths([f64::NAN, -5.0, 1.0e9, f64::INFINITY], [0.0, 402.0]);
        assert!(insets.left.abs() < f32::EPSILON);
        assert!(insets.top.abs() < f32::EPSILON);
        assert!(insets.right.abs() < f32::EPSILON);
        assert!(insets.bottom.abs() < f32::EPSILON);
        let wide = SafeAreaInsets::from_lengths([800.0, 0.0, 800.0, 0.0], [874.0, 402.0]);
        assert!(wide.left + wide.right < 1.0);
    }
}

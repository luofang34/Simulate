//! Digital Crown virtual-to-reality blend law with guaranteed real-world passthrough floor.
//!
//! On Apple Vision Pro (`visionOS`), turning the Digital Crown adjusts the mix between
//! synthetic 3D terrain/airspace and real-world camera passthrough. To prevent total
//! visual occlusion of the cockpit and real outside window, real-world passthrough is
//! strictly floored at [`MIN_PASSTHROUGH_FRACTION`] (`15%`), capping synthetic terrain
//! opacity at [`MAX_VIRTUAL_OPACITY`] (`85%`).

/// Minimum fraction of real-world passthrough preserved at full Digital Crown immersion (`15%`).
pub const MIN_PASSTHROUGH_FRACTION: f32 = 0.15;

/// Maximum opacity of the synthetic 3D terrain shell (`85%`), preserving [`MIN_PASSTHROUGH_FRACTION`].
pub const MAX_VIRTUAL_OPACITY: f32 = 1.0 - MIN_PASSTHROUGH_FRACTION;

/// Default initial Digital Crown normalized turn (`0.65` -> ~`55%` synthetic terrain, ~`45%` passthrough).
pub const DEFAULT_CROWN_TURN: f32 = 0.65;

/// Resolved opacity weights driven by the Apple Vision Pro Digital Crown.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BlendWeights {
    /// Clamped normalized Digital Crown position in `0.0..=1.0`.
    pub crown_turn: f32,
    /// Opacity of synthetic 3D terrain and runway surfaces in `0.0..=MAX_VIRTUAL_OPACITY` (`0.85`).
    pub virtual_terrain_opacity: f32,
    /// Remaining real-world passthrough visibility in `MIN_PASSTHROUGH_FRACTION..=1.0` (`0.15..=1.0`).
    pub passthrough_fraction: f32,
    /// Translucent 3D airspace curtain fill opacity (`0.14..=0.42`).
    pub airspace_fill_opacity: f32,
    /// 3D airspace floor/ceiling rim line opacity (`0.45..=0.90`).
    pub airspace_rim_opacity: f32,
    /// Conformal HWD/HUD flight symbology and traffic callsign opacity (`0.90..=1.0`).
    pub symbology_opacity: f32,
}

/// Stateful controller for the Digital Crown virtual/reality proportion dial.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CrownBlend {
    turn: f32,
}

impl Default for CrownBlend {
    fn default() -> Self {
        Self::new(DEFAULT_CROWN_TURN)
    }
}

impl CrownBlend {
    /// Creates a [`CrownBlend`] clamping `crown_turn` to `0.0..=1.0` (treating NaN as `0.0`).
    #[must_use]
    pub fn new(crown_turn: f32) -> Self {
        let sanitized = if crown_turn.is_finite() {
            crown_turn.clamp(0.0, 1.0)
        } else {
            0.0
        };
        Self { turn: sanitized }
    }

    /// Sets the normalized Digital Crown turn (`0.0` = pure AR passthrough + symbology,
    /// `1.0` = `85%` synthetic vision + `15%` real passthrough).
    pub fn set_turn(&mut self, crown_turn: f32) {
        *self = Self::new(crown_turn);
    }

    /// Applies an incremental Crown rotation delta (`delta_turn` in `-1.0..=1.0`).
    pub fn rotate_by(&mut self, delta_turn: f32) {
        if delta_turn.is_finite() {
            self.set_turn(self.turn + delta_turn);
        }
    }

    /// Current normalized Digital Crown value (`0.0..=1.0`).
    #[must_use]
    pub const fn turn(self) -> f32 {
        self.turn
    }

    /// Computes all layer opacities while guaranteeing `passthrough_fraction >= 0.15`.
    #[must_use]
    pub fn weights(self) -> BlendWeights {
        let t = self.turn.clamp(0.0, 1.0);
        let virtual_terrain_opacity = (t * MAX_VIRTUAL_OPACITY).clamp(0.0, MAX_VIRTUAL_OPACITY);
        let passthrough_fraction =
            (1.0 - virtual_terrain_opacity).clamp(MIN_PASSTHROUGH_FRACTION, 1.0);
        let airspace_fill_opacity = (0.14 + 0.28 * t).clamp(0.14, 0.42);
        let airspace_rim_opacity = (0.45 + 0.45 * t).clamp(0.45, 0.90);
        let symbology_opacity = (0.90 + 0.10 * t).clamp(0.90, 1.0);
        BlendWeights {
            crown_turn: t,
            virtual_terrain_opacity,
            passthrough_fraction,
            airspace_fill_opacity,
            airspace_rim_opacity,
            symbology_opacity,
        }
    }
}

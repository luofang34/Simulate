//! Procedural paint finish of a part: station-aligned panel seams and rivet rows drawn in
//! the shader from part-local position, uneven fading and chalking of upper surfaces, and
//! exhaust soot streaks. Parts that keep the default draw exactly as their material says.

/// Exhaust soot laid down aft of a source, mirrored across the aircraft's `X = 0` plane.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SootStreak {
    /// Lateral distance of the source from the centreline, in metres (`|x|`).
    pub lateral_m: f32,
    /// Fuselage station (`y`) where the streak starts; it runs toward `-y`.
    pub station_m: f32,
    /// Length over which the deposit fades to a third, in metres.
    pub length_m: f32,
    /// Darkening at the source in `[0, 1]`.
    pub strength: f32,
}

/// Paint finish parameters for one part, in the part's local metres.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SurfaceDetail {
    /// Panel seam pitch along local `x`, `y`, `z` (ribs, frames, waterlines); `0.0` draws no
    /// seams across that axis. All zero leaves the part without seams or rivets.
    pub panel_pitch_m: [f32; 3],
    /// Fading, chalking and grime in `[0, 1]`; upper surfaces fade most.
    pub weathering: f32,
    /// Exhaust soot over the part, if any.
    pub soot: Option<SootStreak>,
}

impl SurfaceDetail {
    /// The two `vec4` rows of the shaders' `Draw.detail` and `Draw.soot`.
    pub(crate) fn uniform_rows(&self) -> [[f32; 4]; 2] {
        let [px, py, pz] = self.panel_pitch_m;
        let soot = self.soot.unwrap_or_default();
        [
            [px, py, pz, self.weathering],
            [soot.lateral_m, soot.station_m, soot.length_m, soot.strength],
        ]
    }
}

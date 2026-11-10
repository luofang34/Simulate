//! Central gondola mould line and canopy, lofted through cross-sections read off the
//! AN 01-75F-2 general arrangement (`design/an01-75f-2_general_arrangement.json`).
//!
//! What each view fixes:
//! - side view: the nose deck and keel lines, the windscreen (a straight line from the
//!   sill at `FS 100` to the top bow at `FS 126.2`), the canopy top to `FS 165` and the
//!   sill line `25.6 in` above the fuselage reference line;
//! - plan view: the half-breadth ahead of the wing (`18.995 in` callout), the gondola
//!   tail cone above the wing (`FS 156..236`) and the canopy's footprint, which closes
//!   at `FS 100` and `FS 184.5`;
//! - front view: the belly section and the canopy section at its peak.
//!
//! The port wing hides the canopy top aft of `FS 165` and the deck behind it on the
//! side view, so those heights continue the visible slope to the canopy tail and on to
//! the tail tip at the wing trailing edge; the height of maximum breadth is not drawn
//! and is assumed.

use std::f32::consts::FRAC_PI_2;

use glam::Vec3;

use crate::drawing::{
    CANOPY_SILL_ABOVE_FRL_IN, CANOPY_TAIL_FS_IN, FRL_Z_M, GONDOLA_HALF_BREADTH_IN,
    HATCH_AFT_FRAME_FS_IN, INCH_M, NOSE_Y_M, WINDSCREEN_BASE_FS_IN, WINDSCREEN_TOP_FRAME_FS_IN,
};

/// One gondola cross-section in drawing inches: stations aft of the nose, heights above
/// the fuselage reference line, half-widths off the ship centreline.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GondolaSection {
    /// Fuselage station.
    pub fs_in: f32,
    /// Height of the maximum breadth.
    pub breadth_h_in: f32,
    /// Maximum half-breadth.
    pub half_breadth_in: f32,
    /// Crown of the closed skin, or the canopy sill line where the cockpit opens it.
    pub deck_h_in: f32,
    /// Keel.
    pub keel_h_in: f32,
    /// Canopy top; the deck height where there is no canopy.
    pub canopy_top_h_in: f32,
    /// Canopy half-width at the sill; zero where there is no canopy.
    pub canopy_half_in: f32,
}

const fn row(
    fs_in: f32,
    breadth_h_in: f32,
    half_breadth_in: f32,
    deck_h_in: f32,
    keel_h_in: f32,
    canopy_top_h_in: f32,
    canopy_half_in: f32,
) -> GondolaSection {
    GondolaSection {
        fs_in,
        breadth_h_in,
        half_breadth_in,
        deck_h_in,
        keel_h_in,
        canopy_top_h_in,
        canopy_half_in,
    }
}

const HB: f32 = GONDOLA_HALF_BREADTH_IN;
const SILL: f32 = CANOPY_SILL_ABOVE_FRL_IN;

/// Gondola sections, nose to tail.
pub const GONDOLA_SECTIONS: &[GondolaSection] = &[
    row(0.4, -3.75, 0.3, -3.6, -3.9, -3.6, 0.0), // nose tip
    row(2.0, -3.4, 5.15, 0.8, -7.5, 0.8, 0.0),
    row(4.0, -3.4, 6.25, 3.9, -11.1, 3.9, 0.0),
    row(8.0, -3.3, 8.1, 7.6, -14.5, 7.6, 0.0),
    row(12.0, -3.2, 9.35, 9.9, -16.4, 9.9, 0.0),
    row(16.0, -3.1, 10.65, 11.6, -18.2, 11.6, 0.0),
    row(20.0, -3.0, 11.6, 13.1, -19.8, 13.1, 0.0),
    row(30.0, -2.5, 13.75, 16.3, -22.5, 16.3, 0.0),
    row(40.0, -1.5, 15.3, 18.6, -24.8, 18.6, 0.0),
    row(50.0, 0.0, 16.6, 20.2, -26.3, 20.2, 0.0),
    row(60.0, 2.0, 17.0, 22.0, -27.8, 22.0, 0.0),
    row(70.0, 4.0, 17.8, 23.2, -28.7, 23.2, 0.0),
    row(80.0, 6.0, 18.65, 23.8, -29.5, 23.8, 0.0),
    row(90.0, 7.5, HB, 24.3, -29.7, 24.3, 0.0),
    row(WINDSCREEN_BASE_FS_IN, 8.0, HB, SILL, -30.4, SILL, 0.0),
    row(102.0, 8.0, HB, SILL, -30.5, 26.72, 5.3),
    row(105.0, 8.0, HB, SILL, -30.7, 28.4, 10.1),
    row(108.6, 8.0, HB, SILL, -30.9, 30.42, 11.7),
    row(112.0, 8.0, HB, SILL, -31.01, 32.33, 13.1), // deepest keel
    row(119.0, 8.0, HB, SILL, -30.6, 36.26, 15.2),  // wing root leading edge
    row(WINDSCREEN_TOP_FRAME_FS_IN, 8.0, HB, SILL, -29.9, 40.3, 15.2),
    row(130.0, 8.0, HB, SILL, -29.2, 41.2, 15.2),
    row(135.0, 8.0, HB, SILL, -29.1, 41.5, 15.2), // canopy peak
    row(140.0, 8.0, HB, SILL, -28.4, 41.5, 14.9),
    row(145.0, 8.0, HB, SILL, -28.2, 41.2, 14.6),
    row(150.0, 8.0, 18.6, SILL, -27.3, 40.5, 14.1),
    row(HATCH_AFT_FRAME_FS_IN, 8.0, 17.6, SILL, -26.6, 39.4, 13.9),
    row(160.0, 8.0, 17.05, SILL, -25.6, 37.8, 13.2), // main beam
    row(165.0, 8.0, 16.75, 26.3, -24.9, 36.3, 12.4),
    row(170.0, 7.5, 16.25, 27.0, -23.5, 34.6, 10.8),
    row(175.0, 7.0, 15.6, 27.7, -22.6, 32.7, 8.9),
    row(180.0, 6.5, 15.0, 28.4, -21.0, 30.8, 5.6),
    row(CANOPY_TAIL_FS_IN, 6.0, 14.35, 29.0, -20.0, 29.0, 0.0),
    row(198.0, 5.0, 11.9, 22.0, -18.1, 22.0, 0.0),
    row(210.0, 4.0, 9.4, 15.5, -16.0, 15.5, 0.0),
    row(223.0, 3.0, 5.25, 8.5, -11.0, 8.5, 0.0),
    row(232.0, 1.5, 1.9, 3.5, -3.5, 3.5, 0.0),
    row(236.2, 0.0, 0.3, 0.6, -0.6, 0.6, 0.0), // tail tip at the wing trailing edge
];

/// Superellipse exponents of the section above the maximum breadth (`x`, `z`).
const UPPER_EXP: (f32, f32) = (0.84, 0.88);
/// Exponents of the belly, fitted to the front view's outline below the wing.
const LOWER_EXP: (f32, f32) = (0.92, 1.18);
/// Exponents of the canopy arc, fitted to the front view's outline above the sill.
const CANOPY_EXP: (f32, f32) = (1.32, 1.38);

/// Model `y` of a section.
#[must_use]
pub fn section_y(section: &GondolaSection) -> f32 {
    NOSE_Y_M - section.fs_in * INCH_M
}

fn z_of(h_in: f32) -> f32 {
    FRL_Z_M + h_in * INCH_M
}

/// The gondola section at model `y`, interpolated linearly between the drawn sections.
#[must_use]
pub fn gondola_section(y: f32) -> GondolaSection {
    let fs = (NOSE_Y_M - y) / INCH_M;
    let Some(&first) = GONDOLA_SECTIONS.first() else {
        return row(fs, 0.0, 1.0, 1.0, -1.0, 1.0, 0.0);
    };
    if fs <= first.fs_in {
        return first;
    }
    for pair in GONDOLA_SECTIONS.windows(2) {
        if let [a, b] = pair
            && fs <= b.fs_in
        {
            let t = ((fs - a.fs_in) / (b.fs_in - a.fs_in).max(1e-4)).clamp(0.0, 1.0);
            let lerp = |u: f32, v: f32| u + (v - u) * t;
            return row(
                fs,
                lerp(a.breadth_h_in, b.breadth_h_in),
                lerp(a.half_breadth_in, b.half_breadth_in),
                lerp(a.deck_h_in, b.deck_h_in),
                lerp(a.keel_h_in, b.keel_h_in),
                lerp(a.canopy_top_h_in, b.canopy_top_h_in),
                lerp(a.canopy_half_in, b.canopy_half_in),
            );
        }
    }
    GONDOLA_SECTIONS.last().copied().unwrap_or(first)
}

/// Arc angle from the crown at which the upper skin is as wide as the canopy: the edge
/// of the cockpit opening. Zero where there is no canopy.
#[must_use]
pub fn opening_edge_theta(y: f32) -> f32 {
    let s = gondola_section(y);
    let ratio = (s.canopy_half_in / s.half_breadth_in.max(1e-3)).clamp(0.0, 0.98);
    ratio.powf(1.0 / UPPER_EXP.0).asin()
}

/// Apex of the upper section curve, inches above the reference line: the deck where the
/// skin is closed. Under the canopy the curve is the one that passes through the sill at
/// the canopy's half-width, and its apex lies inside the cockpit opening.
fn upper_apex_h_in(s: &GondolaSection) -> f32 {
    let ratio = (s.canopy_half_in / s.half_breadth_in.max(1e-3)).clamp(0.0, 0.98);
    let cos_edge = (1.0 - ratio.powf(2.0 / UPPER_EXP.0)).max(1e-4).sqrt();
    s.breadth_h_in + (s.deck_h_in - s.breadth_h_in) / cos_edge.powf(UPPER_EXP.1)
}

/// The gondola cross-section at `y` in model metres: `(z_mid, half_w, z_top, z_bot,
/// z_canopy_top, w_canopy)`. `z_top` is the apex of the upper section curve, which is
/// the deck only where the skin is closed; [`canopy_sill_z`] is the canopy's base.
#[must_use]
pub fn interpolate_gondola(y: f32) -> (f32, f32, f32, f32, f32, f32) {
    let s = gondola_section(y);
    (
        z_of(s.breadth_h_in),
        s.half_breadth_in * INCH_M,
        z_of(upper_apex_h_in(&s)),
        z_of(s.keel_h_in),
        z_of(s.canopy_top_h_in),
        s.canopy_half_in * INCH_M,
    )
}

/// Model `z` of the canopy sill (the deck line) at `y`.
#[must_use]
pub fn canopy_sill_z(y: f32) -> f32 {
    z_of(gondola_section(y).deck_h_in)
}

/// A point on the gondola skin at `y` and arc angle `theta` from the crown (`0` up,
/// `±π/2` the maximum breadth, `π` the keel).
#[must_use]
pub fn gondola_point(y: f32, theta: f32) -> Vec3 {
    let (z_mid, half_w, z_top, z_bot, _, _) = interpolate_gondola(y);
    let (sin_t, cos_t) = theta.sin_cos();
    if cos_t >= 0.0 {
        Vec3::new(
            sin_t.signum() * sin_t.abs().powf(UPPER_EXP.0) * half_w,
            y,
            z_mid + cos_t.powf(UPPER_EXP.1) * (z_top - z_mid),
        )
    } else {
        Vec3::new(
            sin_t.signum() * sin_t.abs().powf(LOWER_EXP.0) * half_w,
            y,
            z_mid - (-cos_t).powf(LOWER_EXP.1) * (z_mid - z_bot),
        )
    }
}

/// A point on the canopy glazing at `y` and arc angle `a` (`-π/2` port sill, `0` crown,
/// `+π/2` starboard sill).
#[must_use]
pub fn canopy_point(y: f32, a: f32) -> Vec3 {
    let s = gondola_section(y);
    let (sin_a, cos_a) = a.clamp(-FRAC_PI_2, FRAC_PI_2).sin_cos();
    let sill = z_of(s.deck_h_in);
    let rise = ((s.canopy_top_h_in - s.deck_h_in) * INCH_M).max(0.0);
    Vec3::new(
        sin_a.signum() * sin_a.abs().powf(CANOPY_EXP.0) * s.canopy_half_in * INCH_M,
        y,
        sill + cos_a.max(0.0).powf(CANOPY_EXP.1) * rise,
    )
}

/// Half-width of the gondola's mould line (skin, or canopy glazing above the sill) at
/// station `y` and height `z`; zero above the canopy or below the keel.
#[must_use]
pub fn gondola_half_width_at(y: f32, z: f32) -> f32 {
    let s = gondola_section(y);
    let (z_mid, half_w, z_top, z_bot, z_canopy, w_canopy) = interpolate_gondola(y);
    let sill = z_of(s.deck_h_in);
    // Inverts `x = w sin^a(t)`, `z = z0 + dz cos^b(t)` for `x` at height fraction `f`.
    let across = |f: f32, (a, b): (f32, f32), w: f32| {
        let cos_t = f.clamp(0.0, 1.0).powf(1.0 / b);
        w * (1.0 - cos_t * cos_t).max(0.0).sqrt().powf(a)
    };
    if z > sill && w_canopy > 0.0 {
        across(
            (z - sill) / (z_canopy - sill).max(1e-4),
            CANOPY_EXP,
            w_canopy,
        )
    } else if z >= z_mid {
        across((z - z_mid) / (z_top - z_mid).max(1e-4), UPPER_EXP, half_w)
    } else {
        across((z_mid - z) / (z_mid - z_bot).max(1e-4), LOWER_EXP, half_w)
    }
}

/// Top-view reference half-width of the central gondola at longitudinal station `y`.
#[must_use]
pub fn three_view_gondola_outline_x(y: f32) -> f32 {
    interpolate_gondola(y).1
}

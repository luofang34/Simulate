//! Panel seams and the flush rivet rows beside them: their relief, the dirt
//! that settles in them and the shadow they hold, from the traced breaks of
//! `panel_lines` placed on the skin.
//!
//! Rivets: Lockheed flush-riveted the P-38's outer skin. One row each side
//! of a seam at about 1 in pitch and 0.4 in edge distance, judged by eye on
//! NARA 196367 (an unpainted nose, no scale in frame); not a sourced
//! drawing value.

use glam::{Vec2, Vec3};

use super::anchors::{Anchors, M_PER_IN};
use super::noise;
use super::panel_lines;
use super::panels::{Hit, LineSet};
use super::weathering::smooth;

/// Width of a panel seam's gap, inches.
const SEAM_WIDTH_IN: f32 = 0.03;
/// Depth of a seam's groove, inches.
const SEAM_DEPTH_IN: f32 = 0.02;
/// Rivet row offset from the seam, pitch and head radius, inches.
const RIVET_OFFSET_IN: f32 = 0.4;
const RIVET_PITCH_IN: f32 = 1.0;
const RIVET_RADIUS_IN: f32 = 0.08;
/// The turbosupercharger wheel's centre in the traced plan view (the
/// circle at FS 203–216): the nacelle's breaks forward of the radiators are
/// carried with the airframe's own turbo, so they stay on its nacelle
/// wherever the geometry puts it.
const TRACED_TURBO_FS: f32 = 209.5;
/// Stations over which that carry fades out, ahead of the radiators.
const NACELLE_CARRY_FS: (f32, f32) = (235.0, 265.0);

/// How seams and rivets mark a point of the skin.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Relief {
    /// Share of the texel a seam's dark gap covers.
    pub(crate) seam: f32,
    /// Share of the texel a rivet head covers.
    pub(crate) rivet: f32,
    /// Slope of the surface across the seam (inches per inch), and the unit
    /// direction in the aircraft frame it rises toward.
    pub(crate) slope: f32,
    pub(crate) uphill: Vec3,
    /// Distance to the nearest seam, inches; wear concentrates near seams.
    pub(crate) distance_in: f32,
    /// Fresh paint over the whole panel, `0..1`: a panel taken off and
    /// repainted, darker than the faded skin round it, as on 43-2239's nose
    /// (USMC 127-GR-8-72001).
    pub(crate) touch_up: f32,
}

impl Relief {
    const NONE: Self = Self {
        seam: 0.0,
        rivet: 0.0,
        slope: 0.0,
        uphill: Vec3::ZERO,
        distance_in: f32::INFINITY,
        touch_up: 0.0,
    };
}

/// The traced breaks, ready to look up.
#[derive(Clone, Debug)]
pub(crate) struct Seams {
    wing: LineSet,
    boom_top: LineSet,
    gondola_top: LineSet,
    boom_side: LineSet,
    gondola_side: LineSet,
    fin_side: LineSet,
}

impl Seams {
    pub(crate) fn new() -> Self {
        let inches = Vec2::splat(6.0);
        Self {
            wing: LineSet::new(panel_lines::WING, Vec2::new(6.0, 0.08)),
            boom_top: LineSet::new(panel_lines::BOOM_TOP, inches),
            gondola_top: LineSet::new(panel_lines::GONDOLA_TOP, inches),
            boom_side: LineSet::new(panel_lines::BOOM_SIDE, inches),
            gondola_side: LineSet::new(panel_lines::GONDOLA_SIDE, inches),
            fin_side: LineSet::new(panel_lines::FIN_SIDE, inches),
        }
    }

    /// The table a point of part `path` consults, and where it is in it.
    fn locate(&self, anchors: &Anchors, path: &str, p: Vec3, n: Vec3) -> Option<Located<'_>> {
        let top = n.z > n.x.abs();
        let set = if path.contains("/Wing/") {
            &self.wing
        } else if path.contains("VerticalFins") || path.contains("Rudder") {
            &self.fin_side
        } else if path.contains("/Booms/") {
            if top { &self.boom_top } else { &self.boom_side }
        } else if path.contains("/Gondola/") {
            if top {
                &self.gondola_top
            } else {
                &self.gondola_side
            }
        } else {
            return None;
        };
        let mut fs = anchors.fs_in(p.y);
        if path.contains("/Booms/") {
            let turbo = anchors.turbo.iter().find(|t| t.x * p.x > 0.0);
            if let Some(turbo) = turbo {
                let shift = anchors.fs_in(turbo.y) - TRACED_TURBO_FS;
                fs -= shift * smooth(NACELLE_CARRY_FS.1, NACELLE_CARRY_FS.0, fs);
            }
        }
        let side = p.x.signum() as i32;
        if path.contains("/Wing/") {
            let (lead, trail) = anchors.wing_chord(p.x)?;
            return Some(Located {
                set,
                at: Vec2::new(p.x / M_PER_IN, (lead - p.y) / (lead - trail).max(1e-3)),
                v_scale: (lead - trail) / M_PER_IN,
                u_axis: Vec3::X,
                v_axis: Vec3::NEG_Y,
                salt: [1, side],
            });
        }
        // Side lines were traced on the port side; the starboard side mirrors them.
        let (v, v_axis, table) = if top {
            (p.x / M_PER_IN, Vec3::X, 2)
        } else {
            (anchors.wl_in(p.z), Vec3::Z, 3)
        };
        Some(Located {
            set,
            at: Vec2::new(fs, v),
            v_scale: 1.0,
            u_axis: Vec3::NEG_Y,
            v_axis,
            salt: [table, side],
        })
    }

    /// The relief at `p` (normal `n`) of part `path`, a texel `footprint_in`
    /// wide.
    pub(crate) fn relief(
        &self,
        anchors: &Anchors,
        path: &str,
        p: Vec3,
        n: Vec3,
        footprint_in: f32,
    ) -> Relief {
        let Some(at) = self.locate(anchors, path, p, n) else {
            return Relief::NONE;
        };
        at.set
            .nearest(at.at, at.v_scale)
            .map_or(Relief::NONE, |hit| {
                let mut relief = shape(hit, at.u_axis, at.v_axis, n, footprint_in);
                relief.touch_up = repainted(&at);
                relief
            })
    }
}

/// Where a point lies in the table it consults.
struct Located<'a> {
    set: &'a LineSet,
    at: Vec2,
    /// Inches per unit of the table's second axis.
    v_scale: f32,
    u_axis: Vec3,
    v_axis: Vec3,
    /// Which table and which side, so mirrored panels repaint independently.
    salt: [i32; 2],
}

/// Share of closed panels repainted after they were off.
const REPAINTED: f32 = 0.3;
/// Grid cells searched for a panel's bounding breaks.
const PANEL_REACH_CELLS: i32 = 10;

/// Whether the panel holding `at` was repainted: a panel is the region the
/// nearest breaks on its four sides enclose, so fresh paint fills it to its
/// seams and no further. Regions the traced lines leave open are not panels
/// and are never repainted.
fn repainted(at: &Located<'_>) -> f32 {
    let bounds = [Vec2::X, Vec2::NEG_X, Vec2::Y, Vec2::NEG_Y]
        .map(|dir| at.set.bound(at.at, dir, PANEL_REACH_CELLS));
    let [Some(a), Some(b), Some(c), Some(d)] = bounds else {
        return 0.0;
    };
    let key = Vec3::new(
        a.wrapping_mul(31).wrapping_add(b) as f32 * 0.731,
        c.wrapping_mul(17).wrapping_add(d) as f32 * 0.593,
        (at.salt[0] * 7 + at.salt[1]) as f32 * 1.37,
    );
    if noise::value(key + Vec3::splat(0.5)) < REPAINTED {
        1.0
    } else {
        0.0
    }
}

/// The seam and rivets of `hit`, the table's axes lying along `u_axis` and
/// `v_axis`.
fn shape(hit: Hit, u_axis: Vec3, v_axis: Vec3, n: Vec3, footprint_in: f32) -> Relief {
    let d = hit.distance_in;
    let width = SEAM_WIDTH_IN.max(footprint_in);
    // A gap narrower than the texel is widened and dimmed in proportion, so
    // the skin keeps its average tone at every mip.
    let seam = (1.0 - smooth(0.5 * width, 0.5 * width + footprint_in, d)) * (SEAM_WIDTH_IN / width);
    let half = (1.5 * footprint_in).max(SEAM_WIDTH_IN);
    let slope = if d < half {
        SEAM_DEPTH_IN * (SEAM_WIDTH_IN / half) * 6.0 * (d / half) * (1.0 - d / half) / half
    } else {
        0.0
    };
    let away = u_axis * hit.away.x + v_axis * hit.away.y;
    let uphill = (away - n * n.dot(away)).normalize_or_zero();
    let row = (d - RIVET_OFFSET_IN).abs();
    let phase = (hit.along_in / RIVET_PITCH_IN).fract() - 0.5;
    let dot = Vec2::new(row, phase * RIVET_PITCH_IN).length();
    let rivet = 1.0
        - smooth(
            RIVET_RADIUS_IN - 0.5 * footprint_in,
            RIVET_RADIUS_IN + 0.5 * footprint_in,
            dot,
        );
    Relief {
        seam,
        rivet: rivet * (RIVET_RADIUS_IN / RIVET_RADIUS_IN.max(footprint_in * 0.5)),
        slope,
        uphill,
        distance_in: d,
        touch_up: 0.0,
    }
}

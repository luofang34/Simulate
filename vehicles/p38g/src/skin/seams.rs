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
    /// Fresh paint brushed along the seam after its panel was off, `0..1`:
    /// the darker outlines round the nose panels of 43-2239 (USMC
    /// 127-GR-8-72001).
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

/// Which table a point consults, and how its axes lie in the aircraft
/// frame.
enum View<'a> {
    Wing,
    Top(&'a LineSet),
    Side(&'a LineSet),
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
        let view = if path.contains("/Wing/") {
            Some(View::Wing)
        } else if path.contains("VerticalFins") || path.contains("Rudder") {
            Some(View::Side(&self.fin_side))
        } else if path.contains("/Booms/") {
            Some(if n.z > n.x.abs() {
                View::Top(&self.boom_top)
            } else {
                View::Side(&self.boom_side)
            })
        } else if path.contains("/Gondola/") {
            Some(if n.z > n.x.abs() {
                View::Top(&self.gondola_top)
            } else {
                View::Side(&self.gondola_side)
            })
        } else {
            None
        };
        let Some(view) = view else {
            return Relief::NONE;
        };
        let mut fs = anchors.fs_in(p.y);
        if path.contains("/Booms/") {
            let turbo = anchors.turbo.iter().find(|t| t.x * p.x > 0.0);
            if let Some(turbo) = turbo {
                let shift = anchors.fs_in(turbo.y) - TRACED_TURBO_FS;
                fs -= shift * smooth(NACELLE_CARRY_FS.1, NACELLE_CARRY_FS.0, fs);
            }
        }
        let (hit, u_axis, v_axis) = match view {
            View::Wing => {
                let Some((lead, trail)) = anchors.wing_chord(p.x) else {
                    return Relief::NONE;
                };
                let chord_in = (lead - trail) / M_PER_IN;
                let at = Vec2::new(p.x / M_PER_IN, (lead - p.y) / (lead - trail).max(1e-3));
                (self.wing.nearest(at, chord_in), Vec3::X, Vec3::NEG_Y)
            }
            View::Top(set) => (
                set.nearest(Vec2::new(fs, p.x / M_PER_IN), 1.0),
                Vec3::NEG_Y,
                Vec3::X,
            ),
            // Traced on the port side; the starboard side mirrors it.
            View::Side(set) => (
                set.nearest(Vec2::new(fs, anchors.wl_in(p.z)), 1.0),
                Vec3::NEG_Y,
                Vec3::Z,
            ),
        };
        let salt = [u_axis.x, v_axis.z, p.x.signum()].map(|c| c as i32);
        hit.map_or(Relief::NONE, |hit| {
            let mut relief = shape(hit, u_axis, v_axis, n, footprint_in);
            relief.touch_up = touch_up(hit, salt, p);
            relief
        })
    }
}

/// Share of seams whose panels were off and touched up again.
const TOUCHED_UP: f32 = 0.4;

/// How much fresh paint lies at `p` beside the break of `hit`: a brushed
/// band an inch or three wide along a third or so of the seams.
fn touch_up(hit: Hit, salt: [i32; 3], p: Vec3) -> f32 {
    let line = i32::try_from(hit.line).unwrap_or(0);
    let pick = noise::value(
        Vec3::new(
            line as f32 * 7.31,
            salt[0] as f32 * 3.1 + salt[1] as f32 * 1.7,
            salt[2] as f32 * 5.3,
        ) + Vec3::splat(0.5),
    );
    if pick > TOUCHED_UP {
        return 0.0;
    }
    let brush = noise::fractal(p, 9.0);
    let width = 1.0 + 2.0 * pick / TOUCHED_UP + 1.2 * brush;
    smooth(width, width - 0.6, hit.distance_in)
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

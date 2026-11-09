//! National insignia, nose number and radio call number, painted as vector
//! art in the skin. Each marking is laid out flat in the plane a viewer
//! reads it in, inches, and placed against drawing stations or features
//! measured from the airframe, so it reads correctly from its own side:
//! numerals on the starboard side run nose-ward, as a viewer there reads
//! them.

use glam::{Vec2, Vec3};

use super::anchors::{Anchors, M_PER_IN};
use super::glyphs;
use super::weathering::smooth;
use crate::stations::{INSIGNIA_BLUE, INSIGNIA_WHITE, STENCIL_YELLOW};

/// What is painted.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Art {
    /// White five-pointed star in a blue disc, its points on the disc's
    /// rim, one point up (sides) or forward (wings).
    Insignia { diameter_in: f32 },
    /// Numerals `text`, `height_in` tall, stroked at `stroke` of the height.
    Numerals {
        text: &'static str,
        height_in: f32,
        stroke: f32,
        colour: Colour,
    },
}

/// Marking paint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Colour {
    White,
    Yellow,
}

/// Where a marking is painted. `side` is the sign of `x` of the half of
/// the airframe it is on.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Place {
    /// Upper (`upper`) or lower wing surface, centred at wing station
    /// `ws_in` and `chord` of the local chord aft of the leading edge.
    Wing {
        side: f32,
        upper: bool,
        ws_in: f32,
        chord: f32,
    },
    /// Boom side, outboard or inboard, centred `aft_of_te_in` aft of the
    /// wing trailing edge beside the boom, at waterline `wl_in`.
    Boom {
        side: f32,
        outboard: bool,
        aft_of_te_in: f32,
        wl_in: f32,
    },
    /// Gondola side, centred at fuselage station `fs_in`, waterline `wl_in`.
    Nose { side: f32, fs_in: f32, wl_in: f32 },
    /// Fin side, outboard or inboard, centred at `fs_in`, `wl_in`.
    Fin {
        side: f32,
        outboard: bool,
        fs_in: f32,
        wl_in: f32,
    },
}

/// One marking.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Marking {
    pub(crate) art: Art,
    pub(crate) place: Place,
}

/// A marking's paint at a point: linear colour and the share it covers.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Paint {
    pub(crate) colour: Vec3,
    pub(crate) cover: f32,
}

impl Marking {
    /// The marking's paint at `p` (normal `n`) of part `path`, a texel
    /// `footprint_in` wide, if it reaches there.
    pub(crate) fn paint(
        &self,
        anchors: &Anchors,
        path: &str,
        p: Vec3,
        n: Vec3,
        footprint_in: f32,
    ) -> Option<Paint> {
        let (centre, right, up, facing) = self.frame(anchors, path, p)?;
        if n.dot(facing) < 0.25 {
            return None;
        }
        let offset = (p - centre) / M_PER_IN;
        let at = Vec2::new(offset.dot(right), offset.dot(up));
        self.art.paint(at, footprint_in)
    }

    /// The marking's centre, its reading axes and the way its face looks,
    /// if `path` at `p` is the surface it is painted on.
    fn frame(&self, anchors: &Anchors, path: &str, p: Vec3) -> Option<(Vec3, Vec3, Vec3, Vec3)> {
        match self.place {
            Place::Wing {
                side,
                upper,
                ws_in,
                chord,
            } => {
                if !path.contains("/Wing/") || p.x * side <= 0.0 {
                    return None;
                }
                let x = side * ws_in * M_PER_IN;
                let (lead, trail) = anchors.wing_chord(x)?;
                let centre = Vec3::new(x, lead - chord * (lead - trail), p.z);
                // From below, starboard lies on the viewer's left.
                let (right, facing) = if upper {
                    (Vec3::X, Vec3::Z)
                } else {
                    (Vec3::NEG_X, Vec3::NEG_Z)
                };
                Some((centre, right, Vec3::Y, facing))
            }
            Place::Boom {
                side,
                outboard,
                aft_of_te_in,
                wl_in,
            } => {
                if !path.contains("/Booms/") || p.x * side <= 0.0 {
                    return None;
                }
                let face = if outboard { side } else { -side };
                let beside = side * (anchors.boom_x + 0.6);
                let (_, trail) = anchors.wing_chord(beside)?;
                let y = trail - aft_of_te_in * M_PER_IN;
                Some(side_frame(
                    Vec3::new(p.x, y, anchors.frl_z + wl_in * M_PER_IN),
                    face,
                ))
            }
            Place::Nose { side, fs_in, wl_in } => {
                if !path.contains("/Gondola/") || p.x * side <= 0.0 {
                    return None;
                }
                let at = Vec3::new(
                    p.x,
                    anchors.nose_y - fs_in * M_PER_IN,
                    anchors.frl_z + wl_in * M_PER_IN,
                );
                Some(side_frame(at, side))
            }
            Place::Fin {
                side,
                outboard,
                fs_in,
                wl_in,
            } => {
                let fin = path.contains("VerticalFins") || path.contains("Rudder");
                if !fin || p.x * side <= 0.0 {
                    return None;
                }
                let face = if outboard { side } else { -side };
                let at = Vec3::new(
                    p.x,
                    anchors.nose_y - fs_in * M_PER_IN,
                    anchors.frl_z + wl_in * M_PER_IN,
                );
                Some(side_frame(at, face))
            }
        }
    }
}

/// The frame of a marking on a side facing `face` (sign of `x`): the viewer
/// there has the nose on the right when looking at the starboard side.
fn side_frame(centre: Vec3, face: f32) -> (Vec3, Vec3, Vec3, Vec3) {
    let facing = Vec3::new(face, 0.0, 0.0);
    let right = if face > 0.0 { Vec3::Y } else { Vec3::NEG_Y };
    (centre, right, Vec3::Z, facing)
}

impl Art {
    fn paint(&self, at: Vec2, footprint_in: f32) -> Option<Paint> {
        let aa = footprint_in.max(0.02);
        match *self {
            Art::Insignia { diameter_in } => {
                let r = diameter_in * 0.5;
                let disc = 1.0 - smooth(r - 0.5 * aa, r + 0.5 * aa, at.length());
                if disc <= 0.0 {
                    return None;
                }
                let star = 1.0 - smooth(-0.5 * aa, 0.5 * aa, star_distance(at / r) * r);
                let colour = Vec3::from(INSIGNIA_BLUE.diffuse_rgb)
                    .lerp(Vec3::from(INSIGNIA_WHITE.diffuse_rgb), star);
                Some(Paint {
                    colour,
                    cover: disc,
                })
            }
            Art::Numerals {
                text,
                height_in,
                stroke,
                colour,
            } => {
                let h = height_in.max(0.1);
                let local = at / h + Vec2::new(glyphs::width(text) * 0.5, 0.5);
                let d = glyphs::distance(text, local) * h;
                let half = 0.5 * stroke * h;
                let cover = 1.0 - smooth(half - 0.5 * aa, half + 0.5 * aa, d);
                (cover > 0.0).then(|| Paint {
                    colour: match colour {
                        Colour::White => Vec3::from(INSIGNIA_WHITE.diffuse_rgb),
                        Colour::Yellow => Vec3::from(STENCIL_YELLOW.diffuse_rgb),
                    },
                    cover,
                })
            }
        }
    }
}

/// Signed distance from `p` to a regular five-pointed star whose points
/// lie on the unit circle, one point along `+y`; negative inside.
fn star_distance(p: Vec2) -> f32 {
    // A regular star's inner vertices lie at sin 18° / sin 54° of its points.
    let inner = 18.0_f32.to_radians().sin() / 54.0_f32.to_radians().sin();
    let corners: Vec<Vec2> = (0..10)
        .map(|i| {
            let a = std::f32::consts::FRAC_PI_2 + i as f32 * std::f32::consts::PI / 5.0;
            let r = if i % 2 == 0 { 1.0 } else { inner };
            Vec2::new(a.cos(), a.sin()) * r
        })
        .collect();
    let mut distance = f32::INFINITY;
    let mut inside = false;
    for i in 0..corners.len() {
        let (Some(&a), Some(&b)) = (corners.get(i), corners.get((i + 1) % corners.len())) else {
            continue;
        };
        let ab = b - a;
        let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
        distance = distance.min(p.distance(a + ab * t));
        if (a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) / (b.y - a.y) * ab.x {
            inside = !inside;
        }
    }
    if inside { -distance } else { distance }
}

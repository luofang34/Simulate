//! Where the skin's features sit, measured from the airframe's own meshes so
//! markings and wear follow the geometry wherever it is drawn.

use glam::{Vec2, Vec3};

use super::P38SkinError;
use crate::mesh::P38MeshPrim;

/// Metres per inch.
pub(crate) const M_PER_IN: f32 = 0.0254;
/// The fuselage reference line lies this far below the thrust line
/// (AN 01-75F-2 p.2, side view: 5.078 in).
const FRL_BELOW_THRUST_IN: f32 = 5.078;
/// The prims the skin measures its anchors from: the gondola's upper skin
/// (nose tip), the booms' upper skin (boom line), the turbosupercharger
/// wheels and the wing's upper skin (edges per span).
const GONDOLA: &str = "/P38g/Gondola/UpperOlive";
const BOOMS: &str = "/P38g/Booms/UpperOlive";
const TURBO_WHEELS: &str = "/P38g/Powerplant/TurboBucketWheels";
const WING: &str = "/P38g/Wing/UpperOlive";

/// Span bins of the wing's measured leading and trailing edges.
const WING_BIN_M: f32 = 0.10;

/// Measured anchors of the airframe in its rest pose (aircraft frame,
/// metres).
#[derive(Clone, Debug)]
pub(crate) struct Anchors {
    /// The gondola's nose tip, `FS 0`.
    pub(crate) nose_y: f32,
    /// The fuselage reference line, `WL 0` of the drawing.
    pub(crate) frl_z: f32,
    /// Boom centrelines, port then starboard (`|x|`).
    pub(crate) boom_x: f32,
    /// Turbosupercharger wheel centres, port then starboard.
    pub(crate) turbo: [Vec3; 2],
    /// Wing leading and trailing edge `y` by span bin of `|x|`.
    wing_edges: Vec<Vec2>,
}

fn points<'a>(prims: &'a [P38MeshPrim], path: &'a str) -> impl Iterator<Item = Vec3> + 'a {
    prims
        .iter()
        .filter(move |p| p.prim_path == path)
        .flat_map(|p| {
            p.vertices.iter().map(|v| {
                p.rest_to_aircraft
                    .as_mat4()
                    .transform_point3(Vec3::from(v.position))
            })
        })
}

fn centroid(points: impl Iterator<Item = Vec3>) -> Option<Vec3> {
    let (sum, n) = points.fold((Vec3::ZERO, 0_u32), |(s, n), p| (s + p, n.wrapping_add(1)));
    (n > 0).then(|| sum / n as f32)
}

impl Anchors {
    /// Measures the anchors of `prims`.
    ///
    /// # Errors
    /// [`P38SkinError::MissingAnchor`] when a prim the skin measures is
    /// absent or empty, as after a rename: the skin is not placed by guess.
    pub(crate) fn measure(prims: &[P38MeshPrim]) -> Result<Self, P38SkinError> {
        let missing = |prim: &'static str| P38SkinError::MissingAnchor { prim };
        let nose_y = points(prims, GONDOLA)
            .map(|p| p.y)
            .fold(f32::NEG_INFINITY, f32::max);
        if !nose_y.is_finite() {
            return Err(missing(GONDOLA));
        }
        let side = |path: &'static str, sign: f32| {
            centroid(points(prims, path).filter(|p| p.x * sign > 0.0)).ok_or(missing(path))
        };
        let booms = side(BOOMS, 1.0)?;
        let wing_edges = wing_edges(prims);
        if wing_edges.is_empty() {
            return Err(missing(WING));
        }
        Ok(Self {
            nose_y,
            frl_z: crate::PROP_HUB_Z - FRL_BELOW_THRUST_IN * M_PER_IN,
            boom_x: booms.x.abs(),
            turbo: [side(TURBO_WHEELS, -1.0)?, side(TURBO_WHEELS, 1.0)?],
            wing_edges,
        })
    }

    /// Fuselage station of `y`, inches aft of the nose.
    pub(crate) fn fs_in(&self, y: f32) -> f32 {
        (self.nose_y - y) / M_PER_IN
    }

    /// Waterline of `z`, inches above the fuselage reference line.
    pub(crate) fn wl_in(&self, z: f32) -> f32 {
        (z - self.frl_z) / M_PER_IN
    }

    /// The wing's leading and trailing edge `y` at span `|x|`, if the wing
    /// reaches it.
    pub(crate) fn wing_chord(&self, x: f32) -> Option<(f32, f32)> {
        let bin = (x.abs() / WING_BIN_M).floor() as usize;
        let edges = self.wing_edges.get(bin).copied()?;
        (edges.x > edges.y).then_some((edges.x, edges.y))
    }
}

/// Leading (`x`) and trailing (`y`) edge of the upper wing skin per span bin.
fn wing_edges(prims: &[P38MeshPrim]) -> Vec<Vec2> {
    let mut edges: Vec<Vec2> = Vec::new();
    for p in points(prims, WING) {
        let bin = (p.x.abs() / WING_BIN_M).floor() as usize;
        if edges.len() <= bin {
            edges.resize(bin + 1, Vec2::new(f32::NEG_INFINITY, f32::INFINITY));
        }
        if let Some(e) = edges.get_mut(bin) {
            *e = Vec2::new(e.x.max(p.y), e.y.min(p.y));
        }
    }
    // Bins between loft rows interpolate their neighbours' edges.
    let known: Vec<(usize, Vec2)> = edges
        .iter()
        .copied()
        .enumerate()
        .filter(|(_, e)| e.x.is_finite())
        .collect();
    for pair in known.windows(2) {
        let (Some(&(i0, a)), Some(&(i1, b))) = (pair.first(), pair.get(1)) else {
            continue;
        };
        for i in i0 + 1..i1 {
            let t = (i - i0) as f32 / (i1 - i0) as f32;
            if let Some(e) = edges.get_mut(i) {
                *e = a.lerp(b, t);
            }
        }
    }
    edges
}

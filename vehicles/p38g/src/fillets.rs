//! Wing-root fillets: a concave blend lofted along the chord where the wing's upper skin
//! meets the gondola and each side of the booms. Each section is a quadratic curve from a
//! point on the body skin to a point on the wing skin, so both ends lie on the surfaces
//! they join instead of a solid poking through them.

use std::f32::consts::PI;

use glam::Vec3;

use crate::mesh::{Inside, MeshBuilder, P38MeshPrim, P38PartRole, fixed};
use crate::stations::{BOOM_X, OLIVE_DRAB, boom_point, gondola_point, wing_surface_point};

/// Chordwise sections per fillet (the wing loft's cosine spacing, so the fillet's wing
/// edge runs along the wing mesh's own chord lines) and points per section.
const CHORD_STEPS: usize = 16;
const CURVE_STEPS: usize = 6;
/// Fillet radius at the leading and trailing edges: the Karman fairing swells aft.
const RADIUS_LE_M: f32 = 0.05;
const RADIUS_TE_M: f32 = 0.16;

/// A body the wing passes through.
enum Body {
    Gondola,
    /// The boom on side `-1` (port) or `+1` (starboard).
    Boom(f32),
}

impl Body {
    fn centre_x(&self) -> f32 {
        match self {
            Self::Gondola => 0.0,
            Self::Boom(side) => side * BOOM_X,
        }
    }

    /// Skin point at angle `theta` from the top on the side facing `dir`.
    fn point(&self, dir: f32, y: f32, theta: f32) -> Vec3 {
        match self {
            Self::Gondola => gondola_point(y, dir * theta),
            Self::Boom(side) => boom_point(*side, y, dir * theta),
        }
    }

    /// Half-width off the centre at station `y` and height `z`, on the side facing `dir`.
    fn half_width(&self, dir: f32, y: f32, z: f32) -> f32 {
        (0..=96)
            .map(|i| self.point(dir, y, PI * i as f32 / 96.0))
            .min_by(|a, b| (a.z - z).abs().total_cmp(&(b.z - z).abs()))
            .map_or(0.0, |p| (p.x - self.centre_x()).abs())
    }
}

pub(crate) fn fillet_prims() -> Vec<P38MeshPrim> {
    let mut fillets = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        build_fillet(&mut fillets, &Body::Gondola, side, 0.55);
        // Inboard and outboard of the boom.
        for face in [-side, side] {
            build_fillet(
                &mut fillets,
                &Body::Boom(side),
                face,
                BOOM_X + face * side * 0.45,
            );
        }
    }
    vec![fixed(
        "/P38g/Wing/KarmanFillets",
        P38PartRole::WingStructure,
        OLIVE_DRAB,
        fillets,
    )]
}

/// Fillet on the `dir` side (`-1` towards `-x`) of `body`, starting the wing sample at
/// `|x| = guess_x` and settling on the body's skin.
fn build_fillet(mesh: &mut MeshBuilder, body: &Body, dir: f32, guess_x: f32) {
    let centre = body.centre_x();
    let x_sign = (centre + dir * 0.01).signum();
    let mut rows = Vec::with_capacity(CHORD_STEPS + 1);
    let mut inside = Vec3::ZERO;
    for i in 0..=CHORD_STEPS {
        let u = (0.5 * (1.0 - (PI * i as f32 / CHORD_STEPS as f32).cos())).clamp(0.01, 0.99);
        let guess = wing_surface_point(x_sign * guess_x, u, true);
        let half = body.half_width(dir, guess.y, guess.z);
        let corner_x = centre + dir * half;
        let wing_at = |x: f32| wing_surface_point(x, u, true);
        let corner = wing_at(corner_x);
        let r = RADIUS_LE_M + (RADIUS_TE_M - RADIUS_LE_M) * u;
        // Lifted a few millimetres so the edge stays clear of the wing's flat facets.
        let on_wing = wing_at(corner_x + dir * r) + Vec3::new(0.0, 0.0, 0.003);
        let up = corner.z + r;
        let on_body = Vec3::new(
            centre + dir * body.half_width(dir, corner.y, up),
            corner.y,
            up,
        );
        let control = Vec3::new(corner_x, corner.y, corner.z);
        rows.push(
            (0..=CURVE_STEPS)
                .map(|k| {
                    let t = k as f32 / CURVE_STEPS as f32;
                    on_body * (1.0 - t) * (1.0 - t)
                        + control * 2.0 * t * (1.0 - t)
                        + on_wing * t * t
                })
                .collect::<Vec<_>>(),
        );
        inside += control - Vec3::new(dir * 0.05, 0.0, 0.05);
    }
    let n = rows.len().max(1) as f32;
    mesh.loft_open(&rows, Inside::Point(inside / n));
}

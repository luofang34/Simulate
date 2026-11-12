//! Which attributes each path terminator must and may carry.

use super::{Key, PathTerminator};

/// What a leg of one path terminator carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Shape {
    /// Whether a fix follows the code.
    pub fix: bool,
    /// The keys the leg must carry.
    pub required: &'static [Key],
    /// The keys the leg may carry besides [`Shape::required`] and the ones
    /// every leg may carry, [`EVERY_LEG`].
    pub optional: &'static [Key],
}

impl Shape {
    /// Whether the leg may carry `key`.
    pub fn allows(&self, key: Key) -> bool {
        EVERY_LEG.contains(&key) || self.required.contains(&key) || self.optional.contains(&key)
    }
}

/// The keys any leg may carry: the missed-approach mark, the turn
/// direction, the altitude and speed constraints and the required
/// navigation performance.
pub const EVERY_LEG: &[Key] = &[Key::M, Key::T, Key::A, Key::S, Key::P];

/// What a fix-terminated leg may say about its fix: fly-over, roles, the
/// approach's vertical values, and where the fix lies from a navaid.
const AT_FIX: &[Key] = &[
    Key::O,
    Key::R,
    Key::G,
    Key::V,
    Key::X,
    Key::N,
    Key::B,
    Key::D,
];
/// [`AT_FIX`] without the navaid reference, for legs whose own geometry
/// needs no navaid.
const AT_FIX_ONLY: &[Key] = &[Key::O, Key::R, Key::G, Key::V, Key::X];
/// What a hold may say about its fix.
const AT_HOLD_FIX: &[Key] = &[Key::O, Key::R];

const fn with_fix(required: &'static [Key], optional: &'static [Key]) -> Shape {
    Shape {
        fix: true,
        required,
        optional,
    }
}

const fn without_fix(required: &'static [Key], optional: &'static [Key]) -> Shape {
    Shape {
        fix: false,
        required,
        optional,
    }
}

/// The shape of a leg with path terminator `path`.
pub const fn shape(path: PathTerminator) -> Shape {
    use Key::{A, B, C, D, H, L, N, T, Z};
    use PathTerminator as P;
    match path {
        P::If | P::Tf | P::Df => with_fix(&[], AT_FIX),
        P::Cf | P::Fm => with_fix(&[C], AT_FIX),
        P::Fa => with_fix(&[C, A], AT_FIX),
        P::Fc => with_fix(&[C, D], AT_FIX),
        P::Fd => with_fix(&[C, N, D], AT_FIX),
        P::Pi => with_fix(&[C, T, D], AT_FIX),
        P::Rf => with_fix(&[Z, T], AT_FIX_ONLY),
        P::Af => with_fix(&[N, D, B, T], AT_FIX_ONLY),
        P::Ha => with_fix(&[C, T, L, A], AT_HOLD_FIX),
        P::Hf | P::Hm => with_fix(&[C, T, L], AT_HOLD_FIX),
        P::Ca => without_fix(&[C, A], &[]),
        P::Cd => without_fix(&[C, N, D], &[B]),
        P::Ci => without_fix(&[C], &[]),
        P::Cr => without_fix(&[C, N, B], &[D]),
        P::Va => without_fix(&[H, A], &[]),
        P::Vd => without_fix(&[H, N, D], &[B]),
        P::Vi | P::Vm => without_fix(&[H], &[]),
        P::Vr => without_fix(&[H, N, B], &[D]),
    }
}

/// Whether `d` on a leg of `path` is a distance from the navaid, so the
/// leg needs `n` when it carries `d`. On `FC` it runs along the course
/// from the fix, and on `PI` it limits the excursion from the fix.
pub const fn distance_from_navaid(path: PathTerminator) -> bool {
    !matches!(path, PathTerminator::Fc | PathTerminator::Pi)
}

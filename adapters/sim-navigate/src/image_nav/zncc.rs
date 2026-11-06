//! Zero-mean normalised cross-correlation search with parabolic sub-cell
//! refinement, and the coarse-to-fine registration built on it.

use glam::DVec2;

use super::patch::GroundPatch;

/// A window must share at least this fraction of seen cells to be scored.
const MIN_OVERLAP: f64 = 0.6;

/// A square window of a patch: top-left cell and side.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Window {
    pub(crate) i: usize,
    pub(crate) j: usize,
    pub(crate) size: usize,
}

/// Where a template window best matched in a search patch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Match {
    /// The search-patch cell the template's top-left landed on, refined
    /// below a cell.
    pub(crate) top_left: DVec2,
    /// The correlation there, `-1..=1`.
    pub(crate) score: f64,
}

/// ZNCC of `window` of `template` placed with its top-left at `(i, j)` of
/// `search`, over the cells both have seen.
pub(crate) fn zncc(
    template: &GroundPatch,
    window: Window,
    search: &GroundPatch,
    i: i64,
    j: i64,
) -> Option<f64> {
    let (i, j) = (usize::try_from(i).ok()?, usize::try_from(j).ok()?);
    let (mut n, mut st, mut ss, mut stt, mut sss, mut sts) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for y in 0..window.size {
        for x in 0..window.size {
            let t = f64::from(template.get(window.i + x, window.j + y)?);
            let s = f64::from(search.get(i + x, j + y)?);
            if t.is_finite() && s.is_finite() {
                n += 1.0;
                st += t;
                ss += s;
                stt += t * t;
                sss += s * s;
                sts += t * s;
            }
        }
    }
    let cells = (window.size * window.size) as f64;
    if n < MIN_OVERLAP * cells {
        return None;
    }
    let variance = (n * stt - st * st) * (n * sss - ss * ss);
    if variance <= f64::EPSILON {
        return None;
    }
    Some((n * sts - st * ss) / variance.sqrt())
}

/// The best placement of `window` of `template` within `radius` cells of
/// `centre` in `search`, refined by a parabola through each axis's
/// neighbours.
pub(crate) fn best_match(
    template: &GroundPatch,
    window: Window,
    search: &GroundPatch,
    centre: (i64, i64),
    radius: i64,
) -> Option<Match> {
    let mut best: Option<(i64, i64, f64)> = None;
    for j in centre.1 - radius..=centre.1 + radius {
        for i in centre.0 - radius..=centre.0 + radius {
            if let Some(score) = zncc(template, window, search, i, j)
                && best.is_none_or(|(_, _, top)| score > top)
            {
                best = Some((i, j, score));
            }
        }
    }
    let (i, j, score) = best?;
    let score_at = |di: i64, dj: i64| zncc(template, window, search, i + di, j + dj);
    let refine = |minus: Option<f64>, plus: Option<f64>| match (minus, plus) {
        (Some(m), Some(p)) => {
            let curvature = m - 2.0 * score + p;
            if curvature < 0.0 {
                (0.5 * (m - p) / curvature).clamp(-0.5, 0.5)
            } else {
                0.0
            }
        }
        _ => 0.0,
    };
    let di = refine(score_at(-1, 0), score_at(1, 0));
    let dj = refine(score_at(0, -1), score_at(0, 1));
    Some(Match {
        top_left: DVec2::new(i as f64 + di, j as f64 + dj),
        score,
    })
}

/// Registers the whole of `template` inside `search`, which is wider by
/// `margin` cells on every side: exhaustively at half resolution, then
/// within two cells at full resolution. Returns the template centre's
/// offset from the search centre in cells (`x` east, `y` south) and the
/// score.
pub(crate) fn register(
    template: &GroundPatch,
    search: &GroundPatch,
    margin: usize,
) -> Option<(DVec2, f64)> {
    let (coarse_template, coarse_search) = (template.halved(), search.halved());
    let coarse_margin = (margin / 2) as i64;
    let coarse = best_match(
        &coarse_template,
        Window {
            i: 0,
            j: 0,
            size: coarse_template.size(),
        },
        &coarse_search,
        (coarse_margin, coarse_margin),
        coarse_margin,
    )?;
    let guess = (
        (coarse.top_left.x * 2.0).round() as i64,
        (coarse.top_left.y * 2.0).round() as i64,
    );
    let fine = best_match(
        template,
        Window {
            i: 0,
            j: 0,
            size: template.size(),
        },
        search,
        guess,
        2,
    )?;
    Some((fine.top_left - DVec2::splat(margin as f64), fine.score))
}

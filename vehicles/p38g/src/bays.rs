//! Open landing-gear bays: a lower skin lofted with a rectangular opening, the bay's
//! interior walls and ceiling behind it, and two doors that close the opening flush. The
//! doors carry a hinge axis along the bay; the renderer swings them open with the gear.
//!
//! AN 01-75FF-2 (P-38L-1 erection and maintenance instructions) describes the main
//! doors: "two doors, hinged to the lower channels of the forward boom, operate
//! automatically with the landing gear movement". AN 01-75F-2 does not draw the bays.
//! Their stations are secondary readings of LeRoy Weber's P-38L-5-LO drawing
//! (Superscale, 1958): the nose-wheel door sections run from station `59 1/4` to the
//! hinge section at `143 13/16`, and the boom keel is stepped at station `155 1/4`,
//! where the main doors begin. Where the main doors end is assumed: the drawing's next
//! keel station, `238.7677`, clear of the stowed 36 in wheel. The openings' widths are
//! assumed from the tyres they pass (27 in nose, 36 in main).

use glam::{DMat4, DVec3, Vec3};

use crate::drawing::fs_to_y;
use crate::mesh::{Inside, MeshBuilder, P38MeshPrim, P38PartRole, part};
use crate::stations::{NEUTRAL_GRAY, WHEEL_WELL_ZINC};

/// A gear bay opening in a lower skin.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Bay {
    /// Forward and aft edges of the opening, fuselage stations.
    pub(crate) fs: (f32, f32),
    /// Lateral centre of the opening, metres.
    pub(crate) centre_x: f32,
    /// Half-width of the opening, metres.
    pub(crate) half_width: f32,
    /// Height of the bay ceiling above the opening's lowest point, metres.
    pub(crate) depth: f32,
}

/// Rows of the skin around a bay: the given stations plus the opening's edges and a few
/// rows across it, ordered forward to aft.
fn rows_y(stations_y: &[f32], bay: Bay) -> Vec<f32> {
    let (y_fwd, y_aft) = (fs_to_y(bay.fs.0), fs_to_y(bay.fs.1));
    let mut ys: Vec<f32> = stations_y
        .iter()
        .copied()
        .filter(|y| (*y - y_fwd).abs() > 0.02 && (*y - y_aft).abs() > 0.02)
        .chain((0..=4).map(|i| y_fwd + (y_aft - y_fwd) * (i as f32 / 4.0)))
        .collect();
    ys.sort_by(|a, b| b.total_cmp(a));
    ys
}

/// Lofts a lower skin into `skin` through `stations_y` (`point(y, column)` gives column
/// `0..=columns` of the arc at `y`) with `bay` left open, and returns the bay's interior
/// and door prims under `path`.
pub(crate) fn skin_with_bay(
    skin: &mut MeshBuilder,
    path: &str,
    stations_y: &[f32],
    columns: usize,
    point: impl Fn(f32, usize) -> Vec3,
    bay: Bay,
    inside: Inside,
) -> Vec<P38MeshPrim> {
    let ys = rows_y(stations_y, bay);
    let grid: Vec<Vec<Vec3>> = ys
        .iter()
        .map(|&y| (0..=columns).map(|c| point(y, c)).collect())
        .collect();
    let (y_fwd, y_aft) = (fs_to_y(bay.fs.0), fs_to_y(bay.fs.1));
    let r_fwd = ys
        .iter()
        .position(|y| (*y - y_fwd).abs() < 1e-6)
        .unwrap_or(0);
    let r_aft = ys
        .iter()
        .position(|y| (*y - y_aft).abs() < 1e-6)
        .unwrap_or(0);
    let mid_row = grid.get((r_fwd + r_aft) / 2).cloned().unwrap_or_default();
    let open: Vec<usize> = (0..=columns)
        .filter(|&c| {
            mid_row
                .get(c)
                .is_some_and(|p| (p.x - bay.centre_x).abs() < bay.half_width)
        })
        .collect();
    let (Some(&c_a), Some(&c_b)) = (open.first(), open.last()) else {
        skin.loft_open(&grid, inside);
        return Vec::new();
    };
    let rows = |from: usize, to: usize, c0: usize, c1: usize| -> Vec<Vec<Vec3>> {
        grid.iter()
            .skip(from)
            .take(to + 1 - from)
            .map(|row| row.iter().skip(c0).take(c1 + 1 - c0).copied().collect())
            .collect()
    };
    // A strip beside the opening does not surround the body's axis, so it is oriented
    // from the middle of the full rows it was cut from.
    let beside = rows(r_fwd, r_aft, 0, columns);
    let count = beside.iter().map(Vec::len).sum::<usize>().max(1);
    let middle = beside.iter().flatten().copied().sum::<Vec3>() / count as f32;
    skin.loft_open(&rows(0, r_fwd, 0, columns), inside);
    skin.loft_open(&rows(r_fwd, r_aft, 0, c_a), Inside::Point(middle));
    skin.loft_open(&rows(r_fwd, r_aft, c_b, columns), Inside::Point(middle));
    skin.loft_open(&rows(r_aft, grid.len() - 1, 0, columns), inside);
    let opening = rows(r_fwd, r_aft, c_a, c_b);
    bay_prims(path, &opening, bay)
}

/// The bay's interior (walls and ceiling) and its two doors, from the opening's grid
/// (rows forward to aft, columns across).
fn bay_prims(path: &str, opening: &[Vec<Vec3>], bay: Bay) -> Vec<P38MeshPrim> {
    let lowest = opening
        .iter()
        .flatten()
        .map(|p| p.z)
        .fold(f32::INFINITY, f32::min);
    let ceiling = lowest + bay.depth;
    let up = |p: Vec3| Vec3::new(p.x, p.y, ceiling);
    let edge = |pick: fn(&Vec<Vec3>) -> Option<&Vec3>| -> Vec<Vec3> {
        opening.iter().filter_map(|r| pick(r).copied()).collect()
    };
    let (left, right) = (edge(|r| r.first()), edge(|r| r.last()));
    let raise = |pts: &[Vec3]| pts.iter().map(|&p| up(p)).collect::<Vec<_>>();
    let mut interior = MeshBuilder::new();
    let centre = Vec3::new(
        bay.centre_x,
        0.5 * (fs_to_y(bay.fs.0) + fs_to_y(bay.fs.1)),
        lowest,
    );
    for wall in [&left, &right] {
        interior.loft_open(&[wall.clone(), raise(wall)], Inside::Point(centre));
    }
    interior.loft_open(&[raise(&left), raise(&right)], Inside::Point(centre));
    for row in [opening.first(), opening.last()].into_iter().flatten() {
        let mut ring = row.clone();
        ring.extend(row.iter().rev().map(|&p| up(p)));
        interior.polygon(&ring, Vec3::Z);
    }
    let mut interior = part(
        &format!("{path}/BayInterior"),
        P38PartRole::CabinExterior,
        WHEEL_WELL_ZINC,
        DMat4::IDENTITY,
        interior,
    );
    interior.double_sided = true;
    let mut prims = vec![interior];
    let columns = opening.first().map_or(0, Vec::len);
    let mid = columns / 2;
    for (name, from, to) in [("A", 0, mid), ("B", mid, columns.saturating_sub(1))] {
        prims.push(door(
            &format!("{path}/GearDoor{name}"),
            opening,
            from,
            to,
            from == 0,
        ));
    }
    prims
}

/// One door: the skin patch between opening columns `from..=to`, hinged on its outer
/// edge (`from` when `hinge_first`, else `to`).
fn door(
    path: &str,
    opening: &[Vec<Vec3>],
    from: usize,
    to: usize,
    hinge_first: bool,
) -> P38MeshPrim {
    let hinge_col = if hinge_first { from } else { to };
    let free_col = if hinge_first { to } else { from };
    let mid_row = opening.get(opening.len() / 2);
    let pivot = mid_row
        .and_then(|r| r.get(hinge_col))
        .copied()
        .unwrap_or(Vec3::ZERO);
    let free = mid_row
        .and_then(|r| r.get(free_col))
        .copied()
        .unwrap_or(pivot);
    let local: Vec<Vec<Vec3>> = opening
        .iter()
        .map(|r| {
            r.iter()
                .skip(from)
                .take(to + 1 - from)
                .map(|&p| p - pivot)
                .collect()
        })
        .collect();
    let mut mesh = MeshBuilder::new();
    mesh.loft_open(&local, Inside::Point(Vec3::new(0.0, 0.0, 1.0)));
    let rest = DMat4::from_translation(pivot.as_dvec3());
    let mut prim = part(path, P38PartRole::CabinExterior, NEUTRAL_GRAY, rest, mesh);
    prim.double_sided = true;
    // Opening swings the free edge down: about +y when the door reaches towards +x.
    let sign = if free.x > pivot.x { 1.0 } else { -1.0 };
    prim.custom_hinge_axis = Some(DVec3::new(0.0, sign, 0.0));
    prim
}

/// The nose-gear bay in the gondola keel, between the first and last nose-wheel door
/// sections; the ceiling stays under the cockpit floor.
pub(crate) const NOSE_BAY: Bay = Bay {
    fs: (59.25, 143.8125),
    centre_x: 0.0,
    half_width: 7.5 * 0.0254,
    depth: 0.50,
};

/// The main-gear bay under the boom on `side`, from the step in the boom keel aft; the
/// wheel hangs `3 in` outboard of the boom centreline (`198 in` tread on `96 in` booms).
pub(crate) fn main_bay(side: f32) -> Bay {
    Bay {
        fs: (155.25, 238.7677),
        centre_x: side * (crate::stations::BOOM_X + 1.5 * 0.0254),
        half_width: 8.5 * 0.0254,
        depth: 0.50,
    }
}

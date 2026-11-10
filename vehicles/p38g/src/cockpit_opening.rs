//! Upper gondola skin with the cockpit cut out under the canopy, and the sill rail
//! along the edge of the opening.
//!
//! Without the opening the dorsal skin runs straight across the cockpit at sill
//! height and the seated pilot looks down onto olive drab instead of the panel.

use std::f32::consts::PI;

use glam::Vec3;

use crate::drawing::{CANOPY_TAIL_FS_IN, WINDSCREEN_BASE_FS_IN, fs_to_y};
use crate::gondola::{
    GONDOLA_SECTIONS, GondolaSection, gondola_point, opening_edge_theta, section_y,
};
use crate::mesh::{Inside, MeshBuilder, P38MeshPrim, P38PartRole, fixed};
use crate::stations::{CANOPY_FRAME_INTERIOR, OLIVE_DRAB};

const ARC_STEPS: usize = 20;
const UPPER_HALF_ARC: f32 = 0.54 * PI;
/// Width of the sill rail inboard of the opening's edge.
const SILL_RAIL_M: f32 = 0.025;

fn arc(y: f32, from: f32, to: f32) -> Vec<Vec3> {
    (0..=ARC_STEPS)
        .map(|i| gondola_point(y, from + (to - from) * i as f32 / ARC_STEPS as f32))
        .collect()
}

fn sections(keep: impl Fn(&GondolaSection) -> bool) -> Vec<f32> {
    GONDOLA_SECTIONS
        .iter()
        .filter(|s| keep(s))
        .map(section_y)
        .collect()
}

/// Upper olive skin: closed ahead of the windscreen and behind the canopy, a strip
/// either side of the cockpit up to the canopy sill, and the sill rails.
pub(crate) fn upper_gondola_skin(prims: &mut Vec<P38MeshPrim>) {
    let full = |y: f32| arc(y, -UPPER_HALF_ARC, UPPER_HALF_ARC);
    let fwd: Vec<Vec<Vec3>> = sections(|s| s.fs_in <= WINDSCREEN_BASE_FS_IN)
        .into_iter()
        .map(full)
        .collect();
    let aft: Vec<Vec<Vec3>> = sections(|s| s.fs_in >= CANOPY_TAIL_FS_IN)
        .into_iter()
        .map(full)
        .collect();
    let cockpit = sections(|s| (WINDSCREEN_BASE_FS_IN..=CANOPY_TAIL_FS_IN).contains(&s.fs_in));
    let mid_y = fs_to_y(0.5 * (WINDSCREEN_BASE_FS_IN + CANOPY_TAIL_FS_IN));
    let mut skin = MeshBuilder::new();
    skin.loft_open(&fwd, Inside::Axis);
    skin.loft_open(&aft, Inside::Axis);
    let mut sills = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        let mut strip = Vec::with_capacity(cockpit.len());
        let mut rail = Vec::with_capacity(cockpit.len());
        for &y in &cockpit {
            let edge = opening_edge_theta(y);
            strip.push(arc(y, side * edge, side * UPPER_HALF_ARC));
            let lip = gondola_point(y, side * edge);
            let inboard = (lip.x.abs() - SILL_RAIL_M).max(0.0);
            rail.push(vec![lip, Vec3::new(side * inboard, y, lip.z)]);
        }
        skin.loft_open(&strip, Inside::Point(Vec3::new(0.0, mid_y, 0.0)));
        sills.loft_open(&rail, Inside::Point(Vec3::new(0.0, mid_y, 0.2)));
    }
    prims.push(fixed(
        "/P38g/Gondola/UpperOlive",
        P38PartRole::CabinExterior,
        OLIVE_DRAB,
        skin,
    ));
    let mut rails = fixed(
        "/P38g/Gondola/CockpitSillRails",
        P38PartRole::CabinExterior,
        CANOPY_FRAME_INTERIOR,
        sills,
    );
    rails.double_sided = true;
    prims.push(rails);
}

#[cfg(test)]
mod tests;

//! Canopy glazing lofted over the gondola's canopy sections, the flush frame straps
//! over the drawn bows, and the flat armoured glass behind the windscreen.

use std::f32::consts::PI;

use glam::Vec3;

use crate::drawing::{
    CANOPY_TAIL_FS_IN, FRL_Z_M, HATCH_AFT_FRAME_FS_IN, INCH_M, WINDSCREEN_BASE_FS_IN,
    WINDSCREEN_SLOPE_DEG, WINDSCREEN_TOP_FRAME_FS_IN, fs_to_y,
};
use crate::gondola::{GONDOLA_SECTIONS, canopy_point, canopy_sill_z, section_y};
use crate::mesh::{Inside, MeshBuilder, P38MeshPrim, P38PartRole, fixed};
use crate::stations::{ARMOR_GLASS, CANOPY_FRAME, CANOPY_GLASS};

/// Model `y` of the middle of the top hatch.
pub(crate) fn hatch_mid_y() -> f32 {
    fs_to_y(0.5 * (WINDSCREEN_TOP_FRAME_FS_IN + HATCH_AFT_FRAME_FS_IN))
}

/// Stations the armoured glass spans, and its gap inside the windscreen line.
const ARMOR_GLASS_FS_IN: (f32, f32) = (108.0, 123.0);
const ARMOR_GLASS_INSET_M: f32 = 0.045;
/// Half-widths of the armoured glass at its lower and upper edges.
const ARMOR_GLASS_HALF_M: (f32, f32) = (0.14, 0.09);

/// Corners of the armoured glass (lower port, lower starboard, upper starboard, upper
/// port): a flat pane parallel to the windscreen's centre line, just inside it.
pub(crate) fn armor_glass_corners() -> [Vec3; 4] {
    let slope = WINDSCREEN_SLOPE_DEG.to_radians();
    let on_line = |fs: f32| {
        let y = fs_to_y(fs);
        let rise = (fs - WINDSCREEN_BASE_FS_IN) * INCH_M * slope.tan();
        let z = canopy_sill_z(fs_to_y(WINDSCREEN_BASE_FS_IN)) + rise;
        // Inward normal of the windscreen line: aft and down.
        Vec3::new(0.0, y, z) + Vec3::new(0.0, -slope.sin(), -slope.cos()) * ARMOR_GLASS_INSET_M
    };
    let (lo, hi) = (on_line(ARMOR_GLASS_FS_IN.0), on_line(ARMOR_GLASS_FS_IN.1));
    let x = |half: f32, side: f32| Vec3::new(side * half, 0.0, 0.0);
    [
        lo + x(ARMOR_GLASS_HALF_M.0, -1.0),
        lo + x(ARMOR_GLASS_HALF_M.0, 1.0),
        hi + x(ARMOR_GLASS_HALF_M.1, 1.0),
        hi + x(ARMOR_GLASS_HALF_M.1, -1.0),
    ]
}

pub(crate) fn canopy_prims() -> Vec<P38MeshPrim> {
    let mut prims = Vec::new();
    const STEPS: usize = 16;
    let glass_rows: Vec<Vec<Vec3>> = GONDOLA_SECTIONS
        .iter()
        .filter(|s| (WINDSCREEN_BASE_FS_IN..=CANOPY_TAIL_FS_IN).contains(&s.fs_in))
        .map(section_y)
        .map(|y| {
            (0..=STEPS)
                .map(|i| canopy_point(y, -0.5 * PI + PI * i as f32 / STEPS as f32))
                .collect()
        })
        .collect();
    let mut glass = MeshBuilder::new();
    let inside = Vec3::new(0.0, hatch_mid_y(), FRL_Z_M + 0.6);
    glass.loft_open(&glass_rows, Inside::Point(inside));
    // The pilot looks out through the back faces.
    let mut glazing = fixed(
        "/P38g/Canopy/Glazing",
        P38PartRole::CabinGlass,
        CANOPY_GLASS,
        glass,
    );
    glazing.double_sided = true;
    prims.push(glazing);

    // Internal flat bulletproof windshield pane behind the windscreen
    let mut armor_pane = MeshBuilder::new();
    armor_pane.polygon(&armor_glass_corners(), Vec3::new(0.0, -0.5, 0.86));
    prims.push(fixed(
        "/P38g/Canopy/BulletproofWindshield",
        P38PartRole::CabinInterior,
        ARMOR_GLASS,
        armor_pane,
    ));

    // Flush canopy framing straps over the bows
    let mut frames = MeshBuilder::new();
    for bow_fs in [WINDSCREEN_TOP_FRAME_FS_IN, HATCH_AFT_FRAME_FS_IN] {
        let mut strip = Vec::with_capacity(2);
        for dy in [-0.015_f32, 0.015] {
            strip.push(
                (0..=STEPS)
                    .map(|i| {
                        let y = fs_to_y(bow_fs) + dy;
                        let p = canopy_point(y, -0.5 * PI + PI * i as f32 / STEPS as f32);
                        let out = (p - Vec3::new(0.0, y, canopy_sill_z(y))).normalize_or(Vec3::Z);
                        p + out * 0.0025
                    })
                    .collect(),
            );
        }
        frames.loft_open(
            &strip,
            Inside::Point(Vec3::new(0.0, fs_to_y(bow_fs), FRL_Z_M + 0.6)),
        );
    }
    prims.push(fixed(
        "/P38g/Canopy/Frames",
        P38PartRole::CabinExterior,
        CANOPY_FRAME,
        frames,
    ));
    prims
}

//! Canopy glazing lofted over the gondola's canopy sections, the flush frame straps
//! over the drawn bows, and the flat armoured glass behind the windscreen.

use std::f32::consts::PI;

use glam::{DMat4, DVec3, Vec3};

use crate::cockpit_reservation::{
    AUX_SIGHT_LINE_ABOVE_FRL_IN, AUX_SIGHT_PORT_IN, BEAD_SIGHT_FS_IN, RING_SIGHT_FS_IN,
};
use crate::drawing::{
    CANOPY_TAIL_FS_IN, FRL_Z_M, HATCH_AFT_FRAME_FS_IN, INCH_M, MIRROR_FAIRING_APEX_ABOVE_FRL_IN,
    MIRROR_FAIRING_FS_IN, MIRROR_FAIRING_WIDTH_IN, WINDSCREEN_BASE_FS_IN, WINDSCREEN_SLOPE_DEG,
    WINDSCREEN_TOP_FRAME_FS_IN, frl_to_z, fs_to_y,
};
use crate::gondola::{GONDOLA_SECTIONS, canopy_point, canopy_sill_z, section_y};
use crate::mesh::{Inside, MeshBuilder, P38ControlJoint, P38MeshPrim, P38PartRole, fixed, part};
use crate::stations::{ARMOR_GLASS, CANOPY_FRAME, CANOPY_GLASS, GUNMETAL};

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
    prims.extend(glazing_prims());

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
    prims.push(mirror_fairing());
    prims.extend(auxiliary_sight());
    prims
}

/// The auxiliary ring-and-bead sight of T.O. 01-75FG-2 (Lightning I; taken for the G,
/// whose 339th FS photographs show the bead post at the left of the windscreen base):
/// the bead on a post on the nose deck and the ring on the windscreen's top frame, on
/// a line three inches to port.
fn auxiliary_sight() -> [P38MeshPrim; 2] {
    let x = -AUX_SIGHT_PORT_IN * INCH_M;
    let line_z = frl_to_z(AUX_SIGHT_LINE_ABOVE_FRL_IN);
    let bead_y = fs_to_y(BEAD_SIGHT_FS_IN);
    let mut bead = MeshBuilder::new();
    // The post starts just under the deck skin so no gap shows at its foot.
    let foot = Vec3::new(x, bead_y, canopy_sill_z(bead_y) - 0.04);
    let tip = Vec3::new(x, bead_y, line_z);
    bead.tube(foot, tip, 0.004, 0.003, 8);
    bead.ellipsoid(tip, Vec3::splat(0.006), 8, 4);
    let mut ring = MeshBuilder::new();
    let ring_y = fs_to_y(RING_SIGHT_FS_IN) + 0.02;
    let rim: Vec<Vec3> = (0..=16)
        .map(|i| {
            let (s, c) = (std::f32::consts::TAU * i as f32 / 16.0).sin_cos();
            Vec3::new(x + 0.028 * c, ring_y, line_z + 0.028 * s)
        })
        .collect();
    for pair in rim.windows(2) {
        if let [a, b] = pair {
            ring.tube(*a, *b, 0.003, 0.003, 6);
        }
    }
    [
        fixed(
            "/P38g/Gondola/BeadSight",
            P38PartRole::CabinExterior,
            GUNMETAL,
            bead,
        ),
        fixed(
            "/P38g/Canopy/RingSight",
            P38PartRole::CabinInterior,
            GUNMETAL,
            ring,
        ),
    ]
}

/// The rear-view mirror's fairing on the hatch, as the general arrangement draws it: a
/// wedge rising aft from the canopy top to a square back, where the mirror faces the pilot.
fn mirror_fairing() -> P38MeshPrim {
    let (fwd, aft) = (
        fs_to_y(MIRROR_FAIRING_FS_IN.0),
        fs_to_y(MIRROR_FAIRING_FS_IN.1),
    );
    let half = 0.5 * MIRROR_FAIRING_WIDTH_IN * INCH_M;
    let apex = frl_to_z(MIRROR_FAIRING_APEX_ABOVE_FRL_IN);
    // Sunk a little into the glazing so no gap shows under the wedge.
    let on_canopy = |y: f32, x: f32| canopy_point(y, 0.0) + Vec3::new(x, 0.0, -0.01);
    let section = |y: f32, top: f32, half: f32| {
        vec![
            on_canopy(y, -half),
            Vec3::new(-0.7 * half, y, top),
            Vec3::new(0.7 * half, y, top),
            on_canopy(y, half),
        ]
    };
    let nose = canopy_point(fwd, 0.0).z + 0.004;
    let mut mesh = MeshBuilder::new();
    mesh.loft(
        &[section(fwd, nose, 0.4 * half), section(aft, apex, half)],
        true,
        Inside::Axis,
    );
    fixed(
        "/P38g/Canopy/MirrorFairing",
        P38PartRole::CabinExterior,
        GUNMETAL,
        mesh,
    )
}

/// Arc angle of the hatch's side rails, where the top hatch meets the wind-down side
/// windows. The drawing does not show the rail; assumed.
pub(crate) const HATCH_RAIL_A: f32 = 1.0;

/// How far the hatch swings up on its aft hinge when fully open, rad. Assumed:
/// no source found gives the angle, and the cockpit photographs that show it
/// raised are not yet measured.
const HATCH_OPEN_RAD: f32 = 1.05;

/// One pane of the glazing: the stations and arc angles it spans, the arc steps it is
/// lofted with, and the point its prim is authored about.
struct Pane {
    name: &'static str,
    stations: (f32, f32),
    arc: (f32, f32),
    steps: usize,
    origin: Vec3,
}

/// The glazing in the panes that open or stay: the fixed windscreen and rear glazing,
/// the top hatch between its drawn frames (authored about its aft frame's crown, on
/// which it swings up, with the hinge axis across the ship), and a wind-down window
/// either side under the hatch's rails.
fn glazing_prims() -> Vec<P38MeshPrim> {
    use std::f32::consts::FRAC_PI_2;
    let (base, bow) = (WINDSCREEN_BASE_FS_IN, WINDSCREEN_TOP_FRAME_FS_IN);
    let (aft, tail) = (HATCH_AFT_FRAME_FS_IN, CANOPY_TAIL_FS_IN);
    let hinge = canopy_point(fs_to_y(aft), 0.0);
    let pane = |name, stations, arc, steps, origin| Pane {
        name,
        stations,
        arc,
        steps,
        origin,
    };
    let panes = [
        pane(
            "Windscreen",
            (base, bow),
            (-FRAC_PI_2, FRAC_PI_2),
            16,
            Vec3::ZERO,
        ),
        pane(
            "Hatch",
            (bow, aft),
            (-HATCH_RAIL_A, HATCH_RAIL_A),
            12,
            hinge,
        ),
        pane(
            "SideWindowLeft",
            (bow, aft),
            (-FRAC_PI_2, -HATCH_RAIL_A),
            4,
            Vec3::ZERO,
        ),
        pane(
            "SideWindowRight",
            (bow, aft),
            (HATCH_RAIL_A, FRAC_PI_2),
            4,
            Vec3::ZERO,
        ),
        pane("Rear", (aft, tail), (-FRAC_PI_2, FRAC_PI_2), 16, Vec3::ZERO),
    ];
    let inside = Vec3::new(0.0, hatch_mid_y(), FRL_Z_M + 0.6);
    panes
        .into_iter()
        .map(|pane| {
            let mut prim = pane.prim(inside);
            prim.control_joints = pane_joint(pane.name).into_iter().collect();
            prim
        })
        .collect()
}

/// The joint the pane named `name` opens on, if it opens: the hatch turning
/// about its hinge (the origin its prim is authored about), a side window
/// sliding down along its own chord from the rail to the sill, so its top
/// edge ends on the sill and the pane inside the wall.
pub(crate) fn pane_joint(name: &str) -> Option<P38ControlJoint> {
    let mid = hatch_mid_y();
    let chord = |side: f32| {
        canopy_point(mid, side * std::f32::consts::FRAC_PI_2)
            - canopy_point(mid, side * HATCH_RAIL_A)
    };
    let (channel, axis, per_unit, slides) = match name {
        "Hatch" => ("canopy_hatch", Vec3::X, HATCH_OPEN_RAD, false),
        "SideWindowLeft" => ("window_left", chord(-1.0), chord(-1.0).length(), true),
        "SideWindowRight" => ("window_right", chord(1.0), chord(1.0).length(), true),
        _ => return None,
    };
    Some(P38ControlJoint {
        channel,
        origin_m: [0.0; 3],
        axis: axis.normalize_or_zero().to_array(),
        per_unit,
        rest: 0.0,
        slides,
    })
}

impl Pane {
    /// The pane's prim, lofted over the canopy sections it spans; `inside` is a point
    /// in the cockpit.
    fn prim(&self, inside: Vec3) -> P38MeshPrim {
        let (fs_from, fs_to) = self.stations;
        let (a_from, a_to) = self.arc;
        let rows: Vec<Vec<Vec3>> = GONDOLA_SECTIONS
            .iter()
            .filter(|s| (fs_from..=fs_to).contains(&s.fs_in))
            .map(section_y)
            .map(|y| {
                (0..=self.steps)
                    .map(|i| {
                        let a = a_from + (a_to - a_from) * i as f32 / self.steps as f32;
                        canopy_point(y, a) - self.origin
                    })
                    .collect()
            })
            .collect();
        let mut glass = MeshBuilder::new();
        glass.loft_open(&rows, Inside::Point(inside - self.origin));
        // The pilot looks out through the back faces.
        let mut pane = part(
            &format!("/P38g/Canopy/Glazing/{}", self.name),
            P38PartRole::CabinGlass,
            CANOPY_GLASS,
            DMat4::from_translation(self.origin.as_dvec3()),
            glass,
        );
        pane.double_sided = true;
        if self.origin != Vec3::ZERO {
            pane.custom_hinge_axis = Some(DVec3::X);
        }
        pane
    }
}

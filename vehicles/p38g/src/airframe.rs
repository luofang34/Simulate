//! Calibrated P-38G-13-LO airframe geometry (`FS 0..248` gondola, nose armament,
//! multi-pane canopy, `WS 0..312` `NACA 23016 -> 4412` wing with Karman fillets,
//! `BS 26..420` twin booms, and empennage).

use std::f32::consts::PI;

use glam::{DMat4, Vec3};

use crate::articulated::push_two_tone;
use crate::canopy::canopy_prims;
use crate::cockpit_opening::upper_gondola_skin;
use crate::cockpit_reservation::SEAT_ARMOUR_FS_IN;
use crate::drawing::{FRL_Z_M, INCH_M, WINDSCREEN_BASE_FS_IN, fs_to_y, y_to_fs};
use crate::fillets::fillet_prims;
use crate::gondola::{GONDOLA_SECTIONS, gondola_point, section_y};
use crate::mesh::{Inside, MeshBuilder, P38MeshPrim, P38PartRole, aerofoil, fixed};
use crate::stations::{
    BOOM_STATIONS, BOOM_X, BRASS_SHELL, CHROME_OLEO, ELEVATOR_HINGE_Y, FIN_PROFILE, GUNMETAL,
    HTAIL_Z, NEUTRAL_GRAY, OLIVE_DRAB, PANEL_BLACK, RUDDER_HINGE_Y, SEMI_SPAN,
    TAILPLANE_SECTIONS_BL, TIP_START_X, boom_point, tailplane_planform_fs, wing_station,
    wing_surface_point,
};

pub(crate) fn airframe_prims() -> Vec<P38MeshPrim> {
    let mut prims = Vec::new();
    build_gondola_and_armament(&mut prims);
    prims.extend(canopy_prims());
    build_cambered_wings_and_fillets(&mut prims);
    build_twin_booms(&mut prims);
    build_empennage(&mut prims);
    prims
}

fn build_gondola_and_armament(prims: &mut Vec<P38MeshPrim>) {
    const ARC_STEPS: usize = 32;
    let mut lower_rows = Vec::with_capacity(GONDOLA_SECTIONS.len());
    for y in GONDOLA_SECTIONS.iter().map(section_y) {
        let lo = (0..=ARC_STEPS)
            .map(|i| gondola_point(y, 0.54 * PI + (i as f32 / ARC_STEPS as f32) * (0.92 * PI)))
            .collect();
        lower_rows.push(lo);
    }
    upper_gondola_skin(prims);

    let mut lower = MeshBuilder::new();
    lower.loft_open(&lower_rows, Inside::Axis);
    prims.push(fixed(
        "/P38g/Gondola/LowerGray",
        P38PartRole::CabinExterior,
        NEUTRAL_GRAY,
        lower,
    ));

    // Upper forward nose deck visible ahead of windscreen from cockpit camera
    let mut nose_deck = MeshBuilder::new();
    let deck_rows: Vec<Vec<Vec3>> = GONDOLA_SECTIONS
        .iter()
        .filter(|s| (2.0..=WINDSCREEN_BASE_FS_IN).contains(&s.fs_in))
        .map(section_y)
        .map(|y| {
            (0..=12)
                .map(|i| {
                    let theta = -0.36 * PI + (i as f32 / 12.0) * (0.72 * PI);
                    gondola_point(y, theta) + Vec3::new(0.0, 0.0, 0.002)
                })
                .collect()
        })
        .collect();
    nose_deck.loft_open(&deck_rows, Inside::Point(Vec3::new(0.0, 2.5, 0.0)));
    prims.push(fixed(
        "/P38g/Gondola/NoseUpperDeck",
        P38PartRole::NoseCowling,
        OLIVE_DRAB,
        nose_deck,
    ));

    build_nose_armament(prims);
}

/// Nose axis the armament is grouped about: the centre of the nose sections, `3.4 in`
/// below the fuselage reference line (AN 01-75F-2 side view).
const NOSE_AXIS_Z: f32 = FRL_Z_M - 3.4 * INCH_M;

fn build_nose_armament(prims: &mut Vec<P38MeshPrim>) {
    let mut guns = MeshBuilder::new();
    let mut troughs = MeshBuilder::new();
    // 1x 20mm Hispano AN-M2 cannon just below the nose axis (protruding to y = 3.74 m)
    let cannon_z = NOSE_AXIS_Z - 0.05;
    guns.tube(
        Vec3::new(0.0, 2.85, cannon_z),
        Vec3::new(0.0, 3.74, cannon_z),
        0.025,
        0.021,
        14,
    );
    guns.tube(
        Vec3::new(0.0, 3.70, cannon_z),
        Vec3::new(0.0, 3.77, cannon_z),
        0.028,
        0.024,
        14,
    );
    // 4x .50 cal AN/M2 Browning barrels in staggered nose ports + perforated jackets
    for (gx, gy, above_axis) in [
        (-0.105_f32, 3.68_f32, 0.125_f32),
        (0.105, 3.64, 0.125),
        (-0.195, 3.54, 0.045),
        (0.195, 3.50, 0.045),
    ] {
        let gz = NOSE_AXIS_Z + above_axis;
        guns.tube(
            Vec3::new(gx, gy - 0.55, gz),
            Vec3::new(gx, gy - 0.12, gz),
            0.021,
            0.021,
            12,
        );
        guns.tube(
            Vec3::new(gx, gy - 0.12, gz),
            Vec3::new(gx, gy, gz),
            0.013,
            0.011,
            10,
        );
        troughs.tube(
            Vec3::new(gx, gy - 0.36, gz),
            Vec3::new(gx, (gy - 0.16).min(3.52), gz),
            0.027,
            0.024,
            10,
        );
    }
    troughs.tube(
        Vec3::new(-0.08, 3.24, NOSE_AXIS_Z - 0.16),
        Vec3::new(-0.08, 3.38, NOSE_AXIS_Z - 0.16),
        0.024,
        0.020,
        10,
    );
    guns.tube(
        Vec3::new(0.0, 2.72, -0.36),
        Vec3::new(0.0, 2.58, -0.68),
        0.010,
        0.005,
        8,
    );
    prims.push(fixed(
        "/P38g/Gondola/NoseArmament",
        P38PartRole::CabinExterior,
        GUNMETAL,
        guns,
    ));
    prims.push(fixed(
        "/P38g/Gondola/GunBlastTroughs",
        P38PartRole::CabinExterior,
        PANEL_BLACK,
        troughs,
    ));
    build_shell_chutes(prims);
}

/// Height of the gondola's lower skin at lateral offset `x` and station `y`.
pub(crate) fn keel_z_at(x: f32, y: f32) -> f32 {
    (0..=64)
        .map(|i| gondola_point(y, PI * (0.5 + 0.5 * i as f32 / 64.0)))
        .min_by(|a, b| (a.x - x).abs().total_cmp(&(b.x - x).abs()))
        .map_or(0.0, |p| p.z)
}

fn build_shell_chutes(prims: &mut Vec<P38MeshPrim>) {
    let mut chutes = MeshBuilder::new();
    for (cx, cy) in [
        (-0.18_f32, 2.48_f32),
        (-0.09, 2.42),
        (0.0, 2.36),
        (0.09, 2.42),
        (0.18, 2.48),
    ] {
        let z = keel_z_at(cx, cy);
        chutes.cuboid(
            Vec3::new(cx - 0.016, cy - 0.06, z - 0.03),
            Vec3::new(cx + 0.016, cy + 0.06, z + 0.03),
        );
    }
    prims.push(fixed(
        "/P38g/Gondola/ShellEjectionChutes",
        P38PartRole::CabinExterior,
        BRASS_SHELL,
        chutes,
    ));
}

/// Chordwise rows of the wing loft (cosine spaced, dense at the nose and tail).
pub(crate) const WING_CHORD_STEPS: usize = 28;
/// Span station of the wing loft's row at the gondola wall, metres: just inside the
/// `18.995 in` half-breadth, so the wing's skin ends in the gondola's.
pub(crate) const WING_GONDOLA_X: f32 = 0.48;

fn build_cambered_wings_and_fillets(prims: &mut Vec<P38MeshPrim>) {
    // The taper is straight to the tip arc, so the rows only need to follow the arc
    // and the junctions with the gondola and booms.
    const OUTER: [f32; 14] = [
        WING_GONDOLA_X,
        1.35,
        BOOM_X,
        3.55,
        4.80,
        6.20,
        TIP_START_X,
        7.50,
        7.62,
        7.72,
        7.80,
        7.86,
        7.90,
        SEMI_SPAN,
    ];
    let us: Vec<f32> = (0..=WING_CHORD_STEPS)
        .map(|i| 0.5 * (1.0 - (PI * i as f32 / WING_CHORD_STEPS as f32).cos()))
        .collect();
    let rows = |xs: &[f32], upper: bool, us: &[f32]| -> Vec<Vec<Vec3>> {
        xs.iter()
            .map(|&x| {
                us.iter()
                    .map(|&u| wing_surface_point(x, u, upper))
                    .collect()
            })
            .collect()
    };
    // Inside the gondola the wing carries through only behind the seat: the cockpit
    // sits between the gondola walls ahead of the main beam, where there is no skin.
    let (root_le, root_te, ..) = wing_station(0.0);
    let behind_seat =
        (y_to_fs(root_le) - SEAT_ARMOUR_FS_IN) / (y_to_fs(root_le) - y_to_fs(root_te));
    let aft_us: Vec<f32> = us.iter().copied().filter(|&u| u >= behind_seat).collect();
    let centre = [-WING_GONDOLA_X, 0.0, WING_GONDOLA_X];
    let port: Vec<f32> = OUTER.iter().rev().map(|x| -x).collect();

    let mut upper = MeshBuilder::new();
    let mut lower = MeshBuilder::new();
    for (mesh, is_upper, inside_z) in [(&mut upper, true, -4.0), (&mut lower, false, 4.0)] {
        let inside = Inside::Point(Vec3::new(0.0, -0.4, inside_z));
        mesh.loft_open(&rows(&port, is_upper, &us), inside);
        mesh.loft_open(&rows(&centre, is_upper, &aft_us), inside);
        mesh.loft_open(&rows(&OUTER, is_upper, &us), inside);
    }
    prims.push(fixed(
        "/P38g/Wing/UpperOlive",
        P38PartRole::WingStructure,
        OLIVE_DRAB,
        upper,
    ));
    prims.push(fixed(
        "/P38g/Wing/LowerGray",
        P38PartRole::WingStructure,
        NEUTRAL_GRAY,
        lower,
    ));
    build_wing_fillets_and_pylons(prims);
}

fn build_wing_fillets_and_pylons(prims: &mut Vec<P38MeshPrim>) {
    prims.extend(fillet_prims());

    // Under-wing 150-gal drop-tank shackles & L-shaped pitot static mast
    let mut pylons = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        let px = side * 1.46;
        let p_lo = wing_surface_point(px, 0.36, false);
        pylons.cuboid(
            Vec3::new(px - 0.032, p_lo.y - 0.32, p_lo.z - 0.15),
            Vec3::new(px + 0.032, p_lo.y + 0.24, p_lo.z + 0.02),
        );
    }
    prims.push(fixed(
        "/P38g/Wing/InnerPylons",
        P38PartRole::WingStrut,
        NEUTRAL_GRAY,
        pylons,
    ));

    let mut pitot = MeshBuilder::new();
    let p_tip = wing_surface_point(-6.35, 0.20, false);
    pitot.tube(
        p_tip + Vec3::new(0.0, 0.0, 0.01),
        p_tip + Vec3::new(0.0, 0.0, -0.22),
        0.012,
        0.009,
        8,
    );
    pitot.tube(
        p_tip + Vec3::new(0.0, -0.02, -0.21),
        p_tip + Vec3::new(0.0, 0.36, -0.21),
        0.008,
        0.004,
        8,
    );
    prims.push(fixed(
        "/P38g/Wing/PitotMast",
        P38PartRole::WingStrut,
        CHROME_OLEO,
        pitot,
    ));
}

fn build_twin_booms(prims: &mut Vec<P38MeshPrim>) {
    const ARC_STEPS: usize = 32;
    let mut upper = MeshBuilder::new();
    let mut lower = MeshBuilder::new();

    for side in [-1.0_f32, 1.0] {
        let mut up_rows = Vec::with_capacity(BOOM_STATIONS.len());
        let mut lo_rows = Vec::with_capacity(BOOM_STATIONS.len());
        for &(y, ..) in BOOM_STATIONS {
            let mut up = Vec::with_capacity(ARC_STEPS + 1);
            let mut lo = Vec::with_capacity(ARC_STEPS + 1);
            for i in 0..=ARC_STEPS {
                let frac = i as f32 / ARC_STEPS as f32;
                let theta_up = -0.54 * PI + frac * (1.08 * PI);
                let theta_lo = 0.54 * PI + frac * (0.92 * PI);
                up.push(boom_point(side, y, theta_up));
                lo.push(boom_point(side, y, theta_lo));
            }
            up_rows.push(up);
            lo_rows.push(lo);
        }
        upper.loft_open(&up_rows, Inside::Axis);
        lower.loft_open(&lo_rows, Inside::Axis);
    }

    prims.push(fixed(
        "/P38g/Booms/UpperOlive",
        P38PartRole::CabinExterior,
        OLIVE_DRAB,
        upper,
    ));
    prims.push(fixed(
        "/P38g/Booms/LowerGray",
        P38PartRole::CabinExterior,
        NEUTRAL_GRAY,
        lower,
    ));
}

fn build_empennage(prims: &mut Vec<P38MeshPrim>) {
    let mut fins = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        let bx = side * BOOM_X;
        let sections: Vec<Vec<Vec3>> = FIN_PROFILE
            .iter()
            .map(|&(z, ly, ty, half_t)| {
                // The fin ends at the rudder hinge, or at the trailing edge where the
                // outline closes ahead of it near the tips.
                let fwd_chord = (ty.max(RUDDER_HINGE_Y) - ly).min(-0.02);
                aerofoil(
                    Vec3::new(bx, ly, z),
                    Vec3::new(0.0, fwd_chord, 0.0),
                    Vec3::new(half_t, 0.0, 0.0),
                )
            })
            .collect();
        fins.loft(&sections, true, Inside::Axis);
    }
    prims.push(fixed(
        "/P38g/Empennage/VerticalFins",
        P38PartRole::CabinExterior,
        OLIVE_DRAB,
        fins,
    ));

    // Stabiliser: the fixed `25 in` ahead of the elevator hinge, across both booms to the
    // semicircular tips.
    let hinge_fs = y_to_fs(ELEVATOR_HINGE_Y);
    let h_sections: Vec<Vec<Vec3>> = TAILPLANE_SECTIONS_BL
        .iter()
        .filter_map(|&bl| {
            let (le, te) = tailplane_planform_fs(bl)?;
            let chord_in = (te.min(hinge_fs) - le).max(0.3);
            let scale = (te - le) / 45.0;
            Some(aerofoil(
                Vec3::new(bl * INCH_M, fs_to_y(le), HTAIL_Z),
                Vec3::new(0.0, -chord_in * INCH_M, 0.0),
                Vec3::new(0.0, 0.0, (0.066 * scale).max(0.004)),
            ))
        })
        .collect();
    push_two_tone(
        prims,
        "/P38g/Empennage/HorizontalTail",
        P38PartRole::CabinExterior,
        DMat4::IDENTITY,
        &h_sections,
        None,
    );
}

//! Articulated P-38G-13-LO flight controls, Curtiss Electric full-feathering
//! propellers, flaps, ailerons, elevator and rudders (`T.O. 01-75F-1`); the landing gear
//! is built in [`crate::gear`].

use std::f32::consts::TAU;

use glam::{DMat4, DVec3, Vec3};

use crate::drawing::{ELEVATOR_CHORD_IN, INCH_M, y_to_fs};
use crate::gear::{build_main_gear, build_nose_gear};
use crate::mesh::{
    Inside, MeshBuilder, P38Material, P38MeshPrim, P38PartRole, aerofoil, circle_segments, part,
};
use crate::stations::{
    AILERON_END_X, BOOM_X, ELEVATOR_HINGE_Y, FIN_PROFILE, HTAIL_Z, NEUTRAL_GRAY, OLIVE_DRAB,
    PROP_BLACK, PROP_BLADE_OFFSET_M, PROP_HUB_Y, PROP_HUB_Z, PROP_RADIUS_M, PROP_YELLOW,
    RUDDER_HINGE_Y, TAILPLANE_SECTIONS_BL, tailplane_planform_fs, wing_station,
};

pub(crate) fn articulated_prims() -> Vec<P38MeshPrim> {
    let mut prims = Vec::new();
    build_curtiss_propellers(&mut prims);
    build_propeller_discs(&mut prims);
    build_fowler_flaps(&mut prims);
    build_ailerons_and_trims(&mut prims);
    build_elevator_and_tabs(&mut prims);
    build_rudders_and_tabs(&mut prims);
    build_nose_gear(&mut prims);
    build_main_gear(&mut prims);
    prims
}

fn build_curtiss_propellers(prims: &mut Vec<P38MeshPrim>) {
    for (label, side) in [("PropLeft", -1.0_f32), ("PropRight", 1.0_f32)] {
        let hub_rest = DMat4::from_translation(DVec3::new(
            f64::from(side * BOOM_X),
            f64::from(PROP_HUB_Y),
            f64::from(PROP_HUB_Z),
        ));
        // Spinners painted with the airframe, as on the 339th FS's P-38G-13s (photo of
        // 43-2239 at Fighter 2, 1943); the nose is the blunt ogive of the AN 01-75F-2 plan
        // view (26 in base, half-width 11.5 in ten inches aft of the tip).
        for (sub, mat, rings_spec) in [
            (
                "SpinnerBase",
                OLIVE_DRAB,
                &[(0.0_f32, 0.33_f32), (0.14, 0.31), (0.26, 0.27)][..],
            ),
            (
                "SpinnerNose",
                OLIVE_DRAB,
                &[
                    (0.26, 0.285),
                    (0.40, 0.245),
                    (0.53, 0.18),
                    (0.64, 0.095),
                    (0.69, 0.045),
                    (0.71, 0.012),
                ][..],
            ),
        ] {
            let mut m = MeshBuilder::new();
            let rings: Vec<Vec<Vec3>> = rings_spec
                .iter()
                .map(|&(y, r)| circle_xz(y, r, circle_segments(0.33)))
                .collect();
            m.loft(&rings, true, Inside::Axis);
            prims.push(part(
                &format!("/P38g/{label}/{sub}"),
                P38PartRole::PropellerSpinner,
                mat,
                hub_rest,
                m,
            ));
        }
        for b in 0..3_u32 {
            let blade_rest =
                hub_rest * DMat4::from_rotation_y(f64::from(b) * (std::f64::consts::TAU / 3.0));
            let (body, tip) = build_curtiss_paddle_blade(-side);
            for (sub, mat, mesh) in [("Blade", PROP_BLACK, body), ("BladeTip", PROP_YELLOW, tip)] {
                let mut p = part(
                    &format!("/P38g/{label}/{sub}{b}"),
                    P38PartRole::PropellerBlades,
                    mat,
                    blade_rest,
                    mesh,
                );
                p.custom_hinge_axis = Some(DVec3::X);
                prims.push(p);
            }
        }
    }
}

fn build_propeller_discs(prims: &mut Vec<P38MeshPrim>) {
    let disc_mat = P38Material {
        diffuse_rgb: [0.16, 0.17, 0.18],
        emissive_rgb: [0.02, 0.02, 0.02],
        metallic: 0.15,
        roughness: 0.35,
        opacity: 0.22,
    };
    for (name, side) in [("PropDiscLeft", -1.0_f32), ("PropDiscRight", 1.0_f32)] {
        let mut disc = MeshBuilder::new();
        disc.disc_xz(PROP_RADIUS_M, 40);
        let rest = DMat4::from_translation(DVec3::new(
            f64::from(side * BOOM_X),
            f64::from(PROP_HUB_Y + PROP_BLADE_OFFSET_M),
            f64::from(PROP_HUB_Z),
        ));
        let mut p = part(
            &format!("/P38g/{name}"),
            P38PartRole::PropellerDisc,
            disc_mat,
            rest,
            disc,
        );
        p.double_sided = true;
        prims.push(p);
    }
}

fn circle_xz(y: f32, r: f32, segs: u32) -> Vec<Vec3> {
    (0..segs)
        .map(|i| {
            let (s, c) = (TAU * i as f32 / segs as f32).sin_cos();
            Vec3::new(c * r, y, s * r)
        })
        .collect()
}

/// Blade planform of `design/curtiss_electric_3b.json` (the propeller performance model's
/// own blade): chord over diameter `base + bulge * sin(pi * (x - 0.2) / 0.85)` less a
/// linear `tip_taper` over the outer tenth, and thickness over chord falling linearly
/// from `root` to `tip`, at radius fraction `x`.
pub(crate) const BLADE_CHORD_OVER_D: (f32, f32, f32) = (0.055, 0.030, 0.02);
pub(crate) const BLADE_THICKNESS_OVER_CHORD: (f32, f32) = (0.30, 0.06);

/// Chord and half-thickness of the blade at radius `r_m`, metres.
fn blade_section(r_m: f32) -> (f32, f32) {
    let (base, bulge, taper) = BLADE_CHORD_OVER_D;
    let (root_t, tip_t) = BLADE_THICKNESS_OVER_CHORD;
    let x = (r_m / PROP_RADIUS_M).clamp(0.0, 1.0);
    let planform = base + bulge * (std::f32::consts::PI * ((x - 0.2) / 0.85).min(1.0)).sin();
    let chord = (planform - taper * ((x - 0.9).max(0.0) / 0.1)) * 2.0 * PROP_RADIUS_M;
    let t_over_c = root_t + (tip_t - root_t) * ((x - 0.2) / 0.8).clamp(0.0, 1.0);
    (chord, 0.5 * t_over_c * chord)
}

fn build_curtiss_paddle_blade(pitch_hand: f32) -> (MeshBuilder, MeshBuilder) {
    let mut body = MeshBuilder::new();
    body.tube(
        Vec3::new(0.18, PROP_BLADE_OFFSET_M, 0.0),
        Vec3::new(0.38, PROP_BLADE_OFFSET_M, 0.0),
        0.048,
        0.056,
        12,
    );
    // Blade angle eases from 34 deg at the root to 9.5 deg at the tip.
    let ring = |r: f32, squeeze: f32| {
        let (chord, half_t) = blade_section(r);
        let deg = 34.0 - 24.5 * ((r - 0.36) / (PROP_RADIUS_M - 0.36)).clamp(0.0, 1.0);
        blade_ring(
            r,
            chord * squeeze,
            half_t * squeeze,
            deg.to_radians() * pitch_hand,
        )
    };
    let body_rings: Vec<Vec<Vec3>> = [0.36_f32, 0.55, 0.80, 1.05, 1.30, 1.56]
        .into_iter()
        .map(|r| ring(r, 1.0))
        .collect();
    body.loft(&body_rings, true, Inside::Axis);

    // The paddle tip: full chord to the last few inches, then rounded off.
    let mut tip = MeshBuilder::new();
    let tip_rings: Vec<Vec<Vec3>> = [
        (1.56_f32, 1.0_f32),
        (1.68, 1.0),
        (PROP_RADIUS_M - 0.03, 0.85),
        (PROP_RADIUS_M, 0.45),
    ]
    .into_iter()
    .map(|(r, squeeze)| ring(r, squeeze))
    .collect();
    tip.loft(&tip_rings, true, Inside::Axis);
    (body, tip)
}

fn blade_ring(x_r: f32, chord: f32, half_t: f32, beta: f32) -> Vec<Vec3> {
    let (sb, cb) = beta.sin_cos();
    let chord_dir = Vec3::new(0.0, sb, cb);
    let thick_dir = Vec3::new(0.0, cb, -sb);
    let lead = Vec3::new(x_r, PROP_BLADE_OFFSET_M, 0.0) - chord_dir * (0.42 * chord);
    aerofoil(lead, chord_dir * chord, thick_dir * half_t)
}

fn build_fowler_flaps(prims: &mut Vec<P38MeshPrim>) {
    // AN 01-75F-2 plan view: flaps from the gondola to `WS 69.25` and from `WS 118.75` to
    // `WS 157.25`, `23.5 in` chord at the flap leading edge.
    const FLAP_CHORD_M: f32 = 23.5 * 0.0254;
    let (fl, fr) = (P38PartRole::FlapLeft, P38PartRole::FlapRight);
    let (inner, outer) = (
        (0.50_f32, 69.25 * 0.0254),
        (118.75 * 0.0254, 157.25 * 0.0254),
    );
    for (name, role, x0, x1) in [
        ("FlapInboardLeft", fl, -inner.1, -inner.0),
        ("FlapOutboardLeft", fl, -outer.1, -outer.0),
        ("FlapInboardRight", fr, inner.0, inner.1),
        ("FlapOutboardRight", fr, outer.0, outer.1),
    ] {
        let xm = 0.5 * (x0 + x1);
        let (_, ty_m, zm_m, ..) = wing_station(xm);
        let (hy, hz) = (ty_m + FLAP_CHORD_M, zm_m - 0.025);
        let sections: Vec<Vec<Vec3>> = [x0, xm, x1]
            .into_iter()
            .map(|x| {
                let (_, ty, zm, ..) = wing_station(x);
                aerofoil(
                    Vec3::new(x - xm, 0.0, zm - hz),
                    Vec3::new(0.0, ty - hy, -0.015),
                    Vec3::new(0.0, 0.0, 0.038),
                )
            })
            .collect();
        let rest = DMat4::from_translation(DVec3::new(f64::from(xm), f64::from(hy), f64::from(hz)));
        push_two_tone(
            prims,
            &format!("/P38g/Wing/{name}"),
            role,
            rest,
            &sections,
            None,
        );
    }
}

/// A control surface lofted through closed `aerofoil` sections (ordered by increasing
/// `x`), painted like the wing: olive drab on top and on the end ribs, neutral gray
/// underneath. Both halves share the role and rest frame so they deflect together.
pub(crate) fn push_two_tone(
    prims: &mut Vec<P38MeshPrim>,
    path: &str,
    role: P38PartRole,
    rest: DMat4,
    sections: &[Vec<Vec3>],
    hinge_axis: Option<DVec3>,
) {
    // `aerofoil` rings run leading edge (index 0) over the top to the trailing edge
    // (index half) and back underneath.
    let half = sections.first().map_or(0, |r| r.len() / 2);
    let upper_rows: Vec<Vec<Vec3>> = sections
        .iter()
        .map(|r| r.iter().take(half + 1).copied().collect())
        .collect();
    let lower_rows: Vec<Vec<Vec3>> = sections
        .iter()
        .map(|r| {
            r.iter()
                .skip(half)
                .chain(r.iter().take(1))
                .copied()
                .collect()
        })
        .collect();
    let mut upper = MeshBuilder::new();
    upper.loft_open(&upper_rows, Inside::Point(Vec3::new(0.0, 0.0, -10.0)));
    for (ring, outward) in [(sections.first(), Vec3::NEG_X), (sections.last(), Vec3::X)] {
        if let Some(ring) = ring {
            upper.polygon(ring, outward);
        }
    }
    let mut lower = MeshBuilder::new();
    lower.loft_open(&lower_rows, Inside::Point(Vec3::new(0.0, 0.0, 10.0)));
    for (sub, mat, mesh) in [("", OLIVE_DRAB, upper), ("Underside", NEUTRAL_GRAY, lower)] {
        let mut prim = part(&format!("{path}{sub}"), role, mat, rest, mesh);
        prim.custom_hinge_axis = hinge_axis;
        prims.push(prim);
    }
}

/// Aileron hinge line: `70 %` chord (AN 01-75F-2 `HINGE ℄ AT 70% C.`).
fn aileron_hinge(x: f32) -> (f32, f32, f32) {
    let (lead, trail, zm, ..) = wing_station(x);
    let aft_chord = 0.30 * (lead - trail);
    (trail + aft_chord, zm, aft_chord)
}

fn build_ailerons_and_trims(prims: &mut Vec<P38MeshPrim>) {
    // AN 01-75F-2 plan view: `107 in` of aileron outboard of `WS 182`.
    const AILERON_START_X: f32 = 182.2 * 0.0254;
    for (name, trim_name, role, x0, x1) in [
        (
            "AileronLeft",
            "AileronTrimTabLeft",
            P38PartRole::AileronLeft,
            -AILERON_END_X,
            -AILERON_START_X,
        ),
        (
            "AileronRight",
            "AileronTrimTabRight",
            P38PartRole::AileronRight,
            AILERON_START_X,
            AILERON_END_X,
        ),
    ] {
        let xm = 0.5 * (x0 + x1);
        let (hy, hz, mid_chord) = aileron_hinge(xm);
        let (hy0, hz0, _) = aileron_hinge(x0);
        let (hy1, hz1, _) = aileron_hinge(x1);
        let axis = DVec3::new(
            f64::from(x1 - x0),
            f64::from(hy1 - hy0),
            f64::from(hz1 - hz0),
        )
        .normalize_or(DVec3::X);

        let sections: Vec<Vec<Vec3>> = [x0, xm, x1]
            .into_iter()
            .map(|x| {
                let (y, z, chord) = aileron_hinge(x);
                aerofoil(
                    Vec3::new(x - xm, y - hy, z - hz),
                    Vec3::new(0.0, -chord, 0.0),
                    Vec3::new(0.0, 0.0, 0.032),
                )
            })
            .collect();
        let rest = DMat4::from_translation(DVec3::new(f64::from(xm), f64::from(hy), f64::from(hz)));
        push_two_tone(
            prims,
            &format!("/P38g/Wing/{name}"),
            role,
            rest,
            &sections,
            Some(axis),
        );

        let mut tab = MeshBuilder::new();
        tab.cuboid(
            Vec3::new(-0.24, -mid_chord, -0.012),
            Vec3::new(0.24, 0.10 - mid_chord, 0.012),
        );
        let mut tab_prim = part(
            &format!("/P38g/Wing/{trim_name}"),
            P38PartRole::CabinExterior,
            OLIVE_DRAB,
            rest,
            tab,
        );
        tab_prim.custom_hinge_axis = Some(axis);
        prims.push(tab_prim);
    }
}

fn build_elevator_and_tabs(prims: &mut Vec<P38MeshPrim>) {
    let elev_rest = DMat4::from_translation(DVec3::new(
        0.0,
        f64::from(ELEVATOR_HINGE_Y),
        f64::from(HTAIL_Z),
    ));
    // The `20 in` elevator aft of the hinge, tip to tip; it narrows to nothing where the
    // semicircular tips close ahead of the hinge.
    let hinge_fs = y_to_fs(ELEVATOR_HINGE_Y);
    let e_sections: Vec<Vec<Vec3>> = TAILPLANE_SECTIONS_BL
        .iter()
        .filter_map(|&bl| {
            let (le, te) = tailplane_planform_fs(bl)?;
            let start = le.max(hinge_fs);
            let chord_in = (te - start).max(0.3);
            let scale = (te - le) / 45.0;
            Some(aerofoil(
                Vec3::new(bl * INCH_M, -(start - hinge_fs) * INCH_M, 0.0),
                Vec3::new(0.0, -chord_in * INCH_M, 0.0),
                Vec3::new(0.0, 0.0, (0.045 * scale).max(0.004)),
            ))
        })
        .collect();
    push_two_tone(
        prims,
        "/P38g/Empennage/Elevator",
        P38PartRole::Elevator,
        elev_rest,
        &e_sections,
        None,
    );

    // One trim tab across the centre: `47.686 in` span, `5.25 in` chord (AN 01-75F-2).
    let mut elev_tabs = MeshBuilder::new();
    let (half_span, chord, elev_chord) =
        (23.843 * INCH_M, 5.25 * INCH_M, ELEVATOR_CHORD_IN * INCH_M);
    elev_tabs.cuboid(
        Vec3::new(-half_span, -elev_chord, -0.010),
        Vec3::new(half_span, chord - elev_chord, 0.010),
    );
    prims.push(part(
        "/P38g/Empennage/ElevatorTrimTab",
        P38PartRole::CabinExterior,
        OLIVE_DRAB,
        elev_rest,
        elev_tabs,
    ));
}

fn build_rudders_and_tabs(prims: &mut Vec<P38MeshPrim>) {
    for (name, side) in [("RudderLeft", -1.0_f32), ("RudderRight", 1.0_f32)] {
        let r_rest = DMat4::from_translation(DVec3::new(
            f64::from(side * BOOM_X),
            f64::from(RUDDER_HINGE_Y),
            0.40,
        ));
        let mut r_mesh = MeshBuilder::new();
        let r_sections: Vec<Vec<Vec3>> = FIN_PROFILE
            .iter()
            .map(|&(z, _ly, ty, half_t)| {
                let aft_c = (ty - RUDDER_HINGE_Y).min(-0.01);
                aerofoil(
                    Vec3::new(0.0, 0.0, z - 0.40),
                    Vec3::new(0.0, aft_c, 0.0),
                    Vec3::new(half_t * 0.85, 0.0, 0.0),
                )
            })
            .collect();
        r_mesh.loft(&r_sections, true, Inside::Axis);
        let mut rudder = part(
            &format!("/P38g/Empennage/{name}"),
            P38PartRole::Rudder,
            OLIVE_DRAB,
            r_rest,
            r_mesh,
        );
        // The mesh hinges on a vertical line (the drawn hinge rakes only 1 deg 15 min);
        // without an axis the renderer swings a rudder about a raked default axis.
        rudder.custom_hinge_axis = Some(DVec3::Z);
        prims.push(rudder);

        let mut r_tab = MeshBuilder::new();
        r_tab.cuboid(
            Vec3::new(-0.010, -0.60, -0.18),
            Vec3::new(0.010, -0.48, 0.22),
        );
        prims.push(part(
            &format!("/P38g/Empennage/{name}TrimTab"),
            P38PartRole::CabinExterior,
            OLIVE_DRAB,
            r_rest,
            r_tab,
        ));
    }
}

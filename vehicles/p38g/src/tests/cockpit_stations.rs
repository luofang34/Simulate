//! The gondola, canopy and cockpit contract measured on the built mesh against the
//! stations of the AN 01-75F-2 general arrangement in [`crate::drawing`].

use glam::Vec3;

use crate::cockpit_reservation as c;
use crate::drawing::{self as d, INCH_M, y_to_fs, z_to_frl};
use crate::{P38MeshPrim, P38PartRole, gondola_half_width_at, p38g_cockpit_contract};

fn prims() -> Vec<P38MeshPrim> {
    crate::p38g_mesh_prims()
}

/// Vertices of the prims whose path contains `path`, as `(BL, FS, height above FRL)` in
/// inches.
fn stations(prims: &[P38MeshPrim], path: &str) -> Vec<Vec3> {
    let points: Vec<Vec3> = prims
        .iter()
        .filter(|p| p.prim_path.contains(path))
        .flat_map(|p| {
            p.vertices.iter().map(|v| {
                let p = p
                    .rest_to_aircraft
                    .transform_point3(Vec3::from(v.position).as_dvec3())
                    .as_vec3();
                Vec3::new(p.x / INCH_M, y_to_fs(p.y), z_to_frl(p.z))
            })
        })
        .collect();
    assert!(!points.is_empty(), "no prim matches {path}");
    points
}

fn near(what: &str, measured: f32, drawn: f32, tolerance: f32) {
    assert!(
        (measured - drawn).abs() <= tolerance,
        "{what}: mesh {measured:.2} in, drawing {drawn:.2} in"
    );
}

fn max_by(points: &[Vec3], key: impl Fn(&Vec3) -> f32) -> f32 {
    points.iter().map(key).fold(f32::NEG_INFINITY, f32::max)
}

fn min_by(points: &[Vec3], key: impl Fn(&Vec3) -> f32) -> f32 {
    points.iter().map(key).fold(f32::INFINITY, f32::min)
}

#[test]
fn the_canopy_stands_on_the_drawn_stations() {
    let prims = prims();
    let glass = stations(&prims, "/P38g/Canopy/Glazing");
    near(
        "windscreen base",
        min_by(&glass, |p| p.y),
        d::WINDSCREEN_BASE_FS_IN,
        0.3,
    );
    near(
        "canopy tail",
        max_by(&glass, |p| p.y),
        d::CANOPY_TAIL_FS_IN,
        0.3,
    );
    near(
        "canopy top",
        max_by(&glass, |p| p.z),
        d::CANOPY_TOP_ABOVE_FRL_IN,
        0.5,
    );
    near(
        "sill",
        min_by(&glass, |p| p.z),
        d::CANOPY_SILL_ABOVE_FRL_IN,
        0.3,
    );
    // The windscreen is the straight drawn line up to the top bow.
    let top_at = |fs: f32| {
        let row: Vec<Vec3> = glass
            .iter()
            .copied()
            .filter(|p| (p.y - fs).abs() < 0.05)
            .collect();
        assert!(!row.is_empty(), "no glazing row at FS {fs}");
        max_by(&row, |p| p.z)
    };
    let bow = top_at(d::WINDSCREEN_TOP_FRAME_FS_IN);
    let run = d::WINDSCREEN_TOP_FRAME_FS_IN - d::WINDSCREEN_BASE_FS_IN;
    let slope = ((bow - d::CANOPY_SILL_ABOVE_FRL_IN) / run)
        .atan()
        .to_degrees();
    assert!(
        (slope - d::WINDSCREEN_SLOPE_DEG).abs() < 0.5,
        "windscreen slope {slope:.1} deg"
    );
    // The peak lies under the top hatch, between its drawn frames.
    let peak = glass
        .iter()
        .max_by(|a, b| a.z.total_cmp(&b.z))
        .map(|p| p.y)
        .expect("glazing");
    assert!(
        (d::WINDSCREEN_TOP_FRAME_FS_IN..=d::HATCH_AFT_FRAME_FS_IN).contains(&peak),
        "canopy peak at FS {peak:.1}"
    );
    // Front view: 15.2 in half-width at the sill.
    near("canopy half-width", max_by(&glass, |p| p.x), 15.2, 0.3);
}

#[test]
fn the_gondola_is_as_wide_deep_and_long_as_drawn() {
    let prims = prims();
    let mut skin = stations(&prims, "/P38g/Gondola/LowerGray");
    skin.extend(stations(&prims, "/P38g/Gondola/UpperOlive"));
    // The closed nose-gear doors are part of the keel.
    skin.extend(stations(&prims, "/P38g/Gondola/NoseBay/GearDoor"));
    near(
        "half-breadth",
        max_by(&skin, |p| p.x),
        d::GONDOLA_HALF_BREADTH_IN,
        0.1,
    );
    near(
        "keel",
        -min_by(&skin, |p| p.z),
        d::GONDOLA_BOTTOM_BELOW_FRL_IN,
        0.3,
    );
    // The nose tip sits 3.75 in below the reference line on the side view.
    let nose = skin
        .iter()
        .min_by(|a, b| a.y.total_cmp(&b.y))
        .expect("skin");
    near("nose station", nose.y, 0.4, 0.3);
    near("nose height", nose.z, -3.75, 0.5);
    // The deck ahead of the windscreen never rises above the sill line.
    let deck = max_by(
        &skin
            .iter()
            .copied()
            .filter(|p| p.y < d::WINDSCREEN_BASE_FS_IN)
            .collect::<Vec<_>>(),
        |p| p.z,
    );
    assert!(deck <= d::CANOPY_SILL_ABOVE_FRL_IN + 0.05, "deck {deck}");
}

#[test]
fn the_contract_sits_on_its_stations() {
    let contract = p38g_cockpit_contract();
    let fs = |y: f32| y_to_fs(y);
    let eye = contract.design_eyepoint_m;
    let panel = contract.main_instrument_panel;
    near(
        "panel",
        fs(panel.center_m[1]),
        c::INSTRUMENT_PANEL_FS_IN,
        0.01,
    );
    assert!(panel.tilt_deg.abs() < 1e-6);
    // The board stands behind the windscreen base and ahead of the top bow.
    let panel_fs = fs(panel.center_m[1]);
    assert!(panel_fs > d::WINDSCREEN_BASE_FS_IN && panel_fs < d::WINDSCREEN_TOP_FRAME_FS_IN);
    // The eye is under the top hatch, ahead of the armour, above the sill.
    let eye_fs = fs(eye[1]);
    assert!((d::WINDSCREEN_TOP_FRAME_FS_IN..d::HATCH_AFT_FRAME_FS_IN).contains(&eye_fs));
    let armour_fs = fs(contract.seat_armour_y_m);
    assert!(eye_fs < armour_fs && armour_fs < c::MAIN_BEAM_FS_IN);
    near(
        "eye height",
        z_to_frl(eye[2]),
        c::DESIGN_EYE_ABOVE_FRL_IN,
        0.01,
    );
    // The seat reference point is on the floor side of the eye, above the floor.
    let seat = contract.seat_reference_point_m;
    assert!(seat[2] > contract.floor_z_m[1] && seat[2] < eye[2] - 0.7);
    for frame in contract.bulkheads {
        near(frame.station_name, fs(frame.y_m), frame.station_in, 0.01);
    }
    assert!(
        contract
            .bulkheads
            .windows(2)
            .all(|w| w[0].station_in < w[1].station_in)
    );
}

/// Nothing inside the cockpit pokes through the skin or the glazing.
#[test]
fn the_cockpit_interior_stays_inside_the_mould_line() {
    const SLACK_M: f32 = 0.004;
    let mut outside = Vec::new();
    for prim in prims() {
        if !matches!(
            prim.role,
            P38PartRole::CabinInterior
                | P38PartRole::PilotFigure
                | P38PartRole::DisplaySurface
                | P38PartRole::CockpitControl
        ) {
            continue;
        }
        let worst = prim
            .vertices
            .iter()
            .map(|v| {
                let p = prim
                    .rest_to_aircraft
                    .transform_point3(Vec3::from(v.position).as_dvec3())
                    .as_vec3();
                (p.x.abs() - gondola_half_width_at(p.y, p.z), p)
            })
            .max_by(|a, b| a.0.total_cmp(&b.0));
        if let Some((excess, p)) = worst
            && excess > SLACK_M
        {
            outside.push(format!(
                "{} by {:.0} mm at FS {:.1}, BL {:.1}, h {:.1}",
                prim.prim_path,
                excess * 1000.0,
                y_to_fs(p.y),
                p.x / INCH_M,
                z_to_frl(p.z)
            ));
        }
    }
    assert!(outside.is_empty(), "{outside:#?}");
}

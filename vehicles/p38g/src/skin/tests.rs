use glam::{Vec2, Vec3};

use super::anchors::{Anchors, M_PER_IN};
use super::markings::{Art, Colour, Marking, Place};
use super::panel_lines;
use crate::mesh::{Inside, MeshBuilder};

#[test]
fn a_closed_loft_repeats_its_seam_column_with_metric_coordinates() {
    let ring = |y: f32| -> Vec<Vec3> {
        (0..12)
            .map(|i| {
                let a = std::f32::consts::TAU * i as f32 / 12.0;
                Vec3::new(a.cos(), y, a.sin())
            })
            .collect()
    };
    let mut mesh = MeshBuilder::new();
    mesh.loft(&[ring(0.0), ring(-3.0)], false, Inside::Axis);
    let (vertices, indices) = mesh.into_buffers();
    assert_eq!(vertices.len(), 2 * 13);
    let first = vertices[0];
    let seam = vertices[12];
    assert_eq!(first.position, seam.position);
    assert_eq!(first.normal, seam.normal);
    // Girth of the twelve-sided ring and the loft's length, in metres.
    let girth = 12.0 * 2.0 * (std::f32::consts::PI / 12.0).sin();
    assert!((seam.uv[1] - girth).abs() < 1e-4, "{:?}", seam.uv);
    assert!((vertices[13].uv[0] - 3.0).abs() < 1e-4);
    // No triangle wraps from the last column back to the first.
    for t in indices.as_chunks::<3>().0 {
        let v: Vec<f32> = t.iter().map(|&i| vertices[i as usize].uv[1]).collect();
        let span =
            v.iter().cloned().fold(f32::MIN, f32::max) - v.iter().cloned().fold(f32::MAX, f32::min);
        assert!(span < girth * 0.5, "{v:?}");
    }
}

#[test]
fn traced_wing_joints_land_on_the_drawings_wing_stations() {
    // AN 01-75F-2 p.2: the outer-wing joint at WS 115 and the 36 in chord
    // callout at WS 289, where the rounded tip begins.
    let spanwise: Vec<f32> = panel_lines::WING
        .iter()
        .filter(|l| l.iter().all(|p| (p[0] - l[0][0]).abs() < 0.5) && l.len() == 2)
        .filter(|l| (l[0][1] - l[1][1]).abs() > 0.9)
        .map(|l| l[0][0].abs())
        .collect();
    for ws in [115.0, 289.0] {
        assert!(
            spanwise.iter().any(|w| (w - ws).abs() < 5.0),
            "{ws} in {spanwise:?}"
        );
    }
}

#[test]
fn the_nose_number_reads_forward_from_starboard_and_aft_from_port() {
    let anchors = Anchors::measure(&crate::p38g_mesh_prims()).unwrap();
    let number = |side: f32| Marking {
        art: Art::Numerals {
            text: "147",
            height_in: 11.0,
            stroke: 0.15,
            colour: Colour::White,
        },
        place: Place::Nose {
            side,
            fs_in: 48.0,
            wl_in: 9.5,
        },
    };
    let centre_y = anchors.nose_y - 48.0 * M_PER_IN;
    let z = anchors.frl_z + 12.5 * M_PER_IN;
    let cover = |side: f32, dy_in: f32| {
        let p = Vec3::new(side * 0.5, centre_y + dy_in * M_PER_IN, z);
        number(side)
            .paint(
                &anchors,
                "/P38g/Gondola/UpperOlive",
                p,
                Vec3::X * side,
                0.05,
            )
            .map_or(0.0, |paint| paint.cover)
    };
    let offsets: Vec<f32> = (-120..=120).map(|i| i as f32 * 0.1).collect();
    let port: Vec<f32> = offsets.iter().map(|&d| cover(-1.0, d)).collect();
    let starboard: Vec<f32> = offsets.iter().map(|&d| cover(1.0, d)).collect();
    assert!(port.iter().sum::<f32>() > 5.0, "the number is painted");
    // Each side is the other seen in a mirror: the same glyph order read
    // from the other side runs the other way along the nose.
    let mirrored: Vec<f32> = starboard.iter().rev().copied().collect();
    let differ = |a: &[f32], b: &[f32]| a.iter().zip(b).map(|(x, y)| (x - y).abs()).sum::<f32>();
    assert!(
        differ(&port, &mirrored) < 0.5,
        "{}",
        differ(&port, &mirrored)
    );
    assert!(
        differ(&port, &starboard) > 5.0,
        "starboard must not be port's copy"
    );
}

#[test]
fn victory_row_reads_two_rising_suns_then_the_ship() {
    use super::victories::{Ink, paint};
    // Row of two 1.5-wide flags and a 2.8-long ship, 0.4 apart, 3 in tall:
    // 6.6 heights wide, so it starts 3.3 heights left of centre.
    let h = 3.0;
    let at =
        |x_heights: f32, y_heights: f32| Vec2::new((x_heights - 3.3) * h, (y_heights - 0.5) * h);
    let ink = |x, y| paint(2, 1, h, at(x, y), 0.01).map(|(ink, _)| ink);
    for flag_centre in [0.75, 2.65] {
        assert_eq!(
            ink(flag_centre, 0.5),
            Some(Ink::Red),
            "disc at {flag_centre}"
        );
        assert_eq!(
            ink(flag_centre - 0.6, 0.15),
            Some(Ink::White),
            "field at {flag_centre}"
        );
    }
    assert_eq!(ink(5.0, 0.15), Some(Ink::White), "hull");
    assert_eq!(ink(5.0, 0.9), None, "sky above the hull");
}

#[test]
fn a_panel_is_bounded_by_the_breaks_round_it() {
    use super::panels::LineSet;
    // A 10 in square panel with a break dividing its right half.
    let square: &[&[[f32; 2]]] = &[
        &[[0.0, 0.0], [10.0, 0.0]],
        &[[10.0, 0.0], [10.0, 10.0]],
        &[[10.0, 10.0], [0.0, 10.0]],
        &[[0.0, 10.0], [0.0, 0.0]],
        &[[5.0, 0.0], [5.0, 10.0]],
    ];
    let set = LineSet::new(square, Vec2::splat(6.0));
    let bounds = |p: Vec2| [Vec2::X, Vec2::NEG_X, Vec2::Y, Vec2::NEG_Y].map(|d| set.bound(p, d, 4));
    assert_eq!(
        bounds(Vec2::new(2.0, 3.0)),
        [Some(4), Some(3), Some(2), Some(0)]
    );
    assert_eq!(
        bounds(Vec2::new(7.0, 3.0)),
        [Some(1), Some(4), Some(2), Some(0)]
    );
    // Outside the panel the region is open on one side.
    assert_eq!(bounds(Vec2::new(12.0, 3.0))[0], None);
}

#[test]
fn a_missing_anchor_prim_is_an_error_not_a_guess() {
    let prims: Vec<_> = crate::p38g_mesh_prims()
        .into_iter()
        .filter(|p| !p.prim_path.contains("TurboBucketWheels"))
        .collect();
    let err = super::P38Skin::new(&prims).unwrap_err();
    assert!(err.to_string().contains("TurboBucketWheels"), "{err}");
    assert!(super::P38Skin::new(&crate::p38g_mesh_prims()).is_ok());
}

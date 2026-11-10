use glam::Vec3;

use super::fitting_prims;
use crate::cockpit_reservation::p38g_cockpit_contract;
use crate::mesh::P38MeshPrim;

fn bounds(prims: &[P38MeshPrim], path: &str) -> (Vec3, Vec3) {
    let prim = prims
        .iter()
        .find(|p| p.prim_path == path)
        .unwrap_or_else(|| panic!("no prim {path}"));
    prim.vertices.iter().fold(
        (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)),
        |(lo, hi), v| {
            let p = prim
                .rest_to_aircraft
                .transform_point3(Vec3::from(v.position).as_dvec3())
                .as_vec3();
            (lo.min(p), hi.max(p))
        },
    )
}

/// Z-4986 shows the switch boxes hanging from the board's lower edge, wholly
/// below the instruments, reaching from left of the centreline to the board's
/// right edge, their faces toward the pilot.
#[test]
fn the_switch_boxes_hang_under_the_right_of_the_board() {
    let contract = p38g_cockpit_contract();
    let board = contract.main_instrument_panel;
    let [_, board_y, board_z] = board.center_m;
    let (lo, hi) = bounds(&fitting_prims(), "/P38g/CockpitInterior/SwitchBoxes");
    let lowest_instrument = board_z - 0.186 - 0.03;
    assert!(hi.z < lowest_instrument + 0.03, "top {} m", hi.z);
    assert!(lo.z > contract.floor_z_m[0] + 0.2, "bottom {} m", lo.z);
    assert!(lo.x < -0.1 && hi.x > 0.35 && hi.x <= 0.5 * board.width_m);
    assert!(hi.y <= board_y + 1e-6 && lo.y > board_y - 0.12);
}

/// The sight's frame stands at the gunsight's station, across the centreline,
/// above the sill and the board's crown, with the reflector over its bar.
#[test]
fn the_gunsight_frame_stands_at_the_sights_station_over_the_coaming() {
    let contract = p38g_cockpit_contract();
    let mount = contract.reflector_gunsight_mount;
    let prims = fitting_prims();
    let (lo, hi) = bounds(&prims, "/P38g/CockpitInterior/GunsightFrame");
    assert!((0.5 * (lo.y + hi.y) - mount.center_m[1]).abs() < 0.02);
    assert!((lo.x + hi.x).abs() < 1e-3 && hi.x - lo.x > 0.3);
    let board = contract.main_instrument_panel;
    let crown = board.center_m[2] + 0.5 * board.height_m;
    assert!(
        lo.z > crown,
        "frame foot {} m under the crown {crown} m",
        lo.z
    );
    let (glass_lo, glass_hi) = bounds(&prims, mount.prim_path);
    assert!(glass_lo.z > lo.z && glass_hi.z < hi.z + 0.08);
}

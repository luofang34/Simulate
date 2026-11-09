//! Camera placements of the fidelity audit, in the airframe model frame (metres).

use super::{Shot, shot};

/// P-38G ground shots: period-photo matches (named after the reference photo) and
/// close-ups of the areas the audit grades. The tyres touch `z ≈ -1.9 m`.
pub(super) const P38G_GROUND: &[Shot] = &[
    shot(
        "p38g_match_43-2239_front34_left",
        [0.0, 0.5, 0.0],
        [-9.0, 11.0, -0.3],
        50.0,
    ),
    shot(
        "p38g_match_339FS_rear",
        [0.0, -3.0, 0.3],
        [0.0, -16.0, 0.4],
        45.0,
    ),
    shot(
        "p38g_match_side_left",
        [0.0, -2.2, 0.0],
        [-24.0, 0.0, 0.0],
        32.0,
    ),
    shot(
        "p38g_match_JBER_front34_left_low",
        [0.0, 1.0, 0.0],
        [-8.0, 9.0, -0.9],
        55.0,
    ),
    shot(
        "p38g_match_Salard_above_front_left",
        [0.0, 0.0, 0.0],
        [-8.0, 8.0, 7.0],
        50.0,
    ),
    shot("p38g_match_headon", [0.0, 0.0, 0.2], [0.0, 20.0, 0.0], 45.0),
    shot(
        "p38g_close_nacelle_turbo_left",
        [-2.36, -1.15, 0.5],
        [-3.0, -2.5, 2.5],
        40.0,
    ),
    shot(
        "p38g_close_fillet_gondola_wing_right",
        [0.8, -0.5, 0.2],
        [3.5, 2.5, 2.5],
        45.0,
    ),
    shot(
        "p38g_close_canopy_left",
        [0.0, 0.6, 0.7],
        [-3.0, 3.0, 1.5],
        40.0,
    ),
    shot(
        "p38g_close_main_gear_left",
        [-2.36, -0.4, -1.1],
        [-1.5, 3.5, -0.5],
        50.0,
    ),
    shot(
        "p38g_close_nose_gear",
        [0.0, 2.0, -1.0],
        [-2.5, 3.0, 0.0],
        50.0,
    ),
    shot(
        "p38g_close_tail_left",
        [0.0, -6.8, 0.6],
        [-5.0, -5.0, 2.5],
        45.0,
    ),
    shot(
        "p38g_close_prop_spinner_left",
        [-2.36, 2.6, 0.04],
        [-1.5, 3.5, 0.4],
        45.0,
    ),
    shot(
        "p38g_close_coolant_scoop_left",
        [-2.36, -3.25, 0.1],
        [-4.0, 0.0, 0.6],
        35.0,
    ),
    shot(
        "p38g_close_cockpit_from_above",
        [0.0, 0.4, 0.4],
        [-1.2, 0.6, 2.2],
        50.0,
    ),
    shot(
        "p38g_close_nose_guns",
        [0.0, 3.0, 0.2],
        [-1.6, 2.4, 0.5],
        45.0,
    ),
    shot(
        "p38g_close_nose_markings_starboard",
        [0.0, 2.3, 0.3],
        [3.5, 1.5, 0.3],
        40.0,
    ),
    shot(
        "p38g_close_nose_markings_port",
        [0.0, 2.3, 0.3],
        [-3.5, 1.5, 0.3],
        40.0,
    ),
];

/// C172 ground shots, matched to the N642SP photos in the audit's reference set. The
/// tyres touch `z ≈ -1.23 m`.
pub(super) const C172_GROUND: &[Shot] = &[
    shot(
        "c172_match_FDK2006_front34_left",
        [0.0, 0.0, 0.0],
        [-6.0, 7.0, 0.3],
        50.0,
    ),
    shot(
        "c172_match_N51_side_left",
        [0.0, -1.5, 0.0],
        [-15.0, 0.0, 0.2],
        35.0,
    ),
    shot(
        "c172_match_IMG6031_rear_over_roof",
        [0.0, -0.5, 0.9],
        [0.3, -4.5, 1.4],
        50.0,
    ),
    shot(
        "c172_match_IMG6030_fin_tip",
        [0.0, -5.4, 1.3],
        [-1.5, -1.0, 0.4],
        40.0,
    ),
    shot(
        "c172_match_IMG6058_belly_left_low",
        [0.0, -1.5, -0.6],
        [-5.0, 0.5, -0.4],
        55.0,
    ),
    shot(
        "c172_match_IMG7157_nose_gear",
        [0.0, 1.4, -0.8],
        [-1.2, 1.0, -0.1],
        50.0,
    ),
    shot(
        "c172_close_main_gear_left",
        [-1.29, -0.43, -0.9],
        [-1.2, 1.5, 0.0],
        45.0,
    ),
    shot(
        "c172_close_cowl_front_left",
        [0.0, 1.8, 0.0],
        [-1.5, 3.5, 0.2],
        45.0,
    ),
    shot(
        "c172_close_wingtip_left",
        [-5.4, -0.2, 0.7],
        [-1.0, 2.5, 0.4],
        40.0,
    ),
    shot(
        "c172_close_cabin_through_door",
        [0.0, 0.3, 0.1],
        [-2.0, -0.3, 0.4],
        55.0,
    ),
    shot("c172_match_headon", [0.0, 0.0, 0.0], [0.0, 15.0, 0.0], 40.0),
    shot(
        "c172_rear34_right_high",
        [0.0, -1.5, 0.3],
        [7.0, -8.0, 3.0],
        45.0,
    ),
];

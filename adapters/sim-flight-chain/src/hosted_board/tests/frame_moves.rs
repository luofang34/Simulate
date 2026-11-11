//! The flight controller's account stays in WGS84 terms however often it
//! moves its navigation frame's origin.

use super::*;

/// About 2.4 km east: past two moves of the frame's origin. The height the
/// account reports as the target is the commanded one above the origin's
/// datum, and the account's estimate stays on the aeroplane.
#[test]
fn the_account_keeps_the_commanded_height_across_frame_moves() {
    let mut board = c172(5);
    boot_and_arm(&mut board);
    let mut sequence = 0;
    let commanded_m = ground().height_m + START_HEIGHT_M;
    let east = core::f64::consts::FRAC_PI_2;
    let (mut worst_target_m, mut worst_estimate_m) = (0.0_f64, 0.0_f64);
    for _ in 0..40 {
        fly(
            &mut board,
            &mut sequence,
            1.0,
            east,
            CRUISE_M_S,
            START_HEIGHT_M,
        );
        let avionics = board.avionics();
        let target_m = avionics.targets.altitude_m.unwrap();
        worst_target_m = worst_target_m.max((target_m - commanded_m).abs());
        let estimate = avionics.estimate.unwrap();
        let off_m = (estimate.position_ecef - board.true_state().position_ecef).length();
        worst_estimate_m = worst_estimate_m.max(off_m);
    }
    let rebases = board.controller().board().navigation_frame().rebases;
    tracing::info!(
        rebases,
        worst_target_m,
        worst_estimate_m,
        "across frame moves"
    );
    assert!(rebases >= 2, "{rebases} moves");
    assert!(worst_target_m < 1.0, "the target strays {worst_target_m} m");
    assert!(
        worst_estimate_m < 15.0,
        "the estimate strays {worst_estimate_m} m"
    );
}

//! The instrument board's hole pattern checked against a period photograph taken
//! square on to it: Lockheed negative Z-4986, captioned "P-38 #3474 INSTRUMENT
//! PANEL 12-23-42", a view forward over the control wheel in the series of the
//! Z-4972 cockpit photographs. A flat board seen by any pinhole camera maps onto
//! the picture by one homography, so the pack's layout (measured on the surviving
//! board of 43-2285) and the instrument centres read off the photograph agree
//! exactly when one homography carries every slot onto its instrument, whatever
//! the camera was. The photograph is not in the repository; the centres are in
//! pixels of a 1728 x 1356 copy of it, read to about 4 px.

use super::resection::solve_linear;

/// Layout slot and the centre of its instrument in the photograph, px.
const SEEN: [(&str, [f64; 2]); 15] = [
    ("engine_mp_left", [606.0, 455.0]),
    ("engine_mp_right", [727.0, 455.0]),
    ("engine_rpm_left", [615.0, 555.0]),
    ("engine_rpm_right", [735.0, 553.0]),
    ("engine_gage_left", [628.0, 657.0]),
    ("engine_gage_right", [743.0, 657.0]),
    ("engine_coolant_left", [516.0, 593.0]),
    ("engine_coolant_right", [524.0, 668.0]),
    ("flight_altimeter", [855.0, 497.0]),
    ("magnetic_compass", [1001.0, 545.0]),
    ("flight_rate_of_climb", [1148.0, 497.0]),
    ("turn_and_bank", [1155.0, 597.0]),
    ("aux_suction", [1345.0, 607.0]),
    ("aux_hydraulic_pressure", [1193.0, 685.0]),
    ("aux_hydraulic_system_pressure", [1275.0, 685.0]),
];

/// The homography carrying board points onto pixels, least squares over
/// `pairs` (`board [right, up]`, `pixel`), as its first eight entries with the
/// ninth one.
fn homography(pairs: &[([f64; 2], [f64; 2])]) -> Option<Vec<f64>> {
    let mut normal = vec![vec![0.0; 8]; 8];
    let mut rhs = vec![0.0; 8];
    for ([x, y], [u, v]) in pairs {
        for (row, target) in [
            ([*x, *y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y], *u),
            ([0.0, 0.0, 0.0, *x, *y, 1.0, -v * x, -v * y], *v),
        ] {
            for i in 0..8 {
                rhs[i] += row[i] * target;
                for j in 0..8 {
                    normal[i][j] += row[i] * row[j];
                }
            }
        }
    }
    solve_linear(normal, rhs)
}

fn project(h: &[f64], [x, y]: [f64; 2]) -> [f64; 2] {
    let w = h[6] * x + h[7] * y + 1.0;
    [
        (h[0] * x + h[1] * y + h[2]) / w,
        (h[3] * x + h[4] * y + h[5]) / w,
    ]
}

/// Every slot seen clearly in Z-4986, with its place on the pack's board. The
/// instruments along the crown are behind the gunsight's frame there.
fn seen_slots() -> Vec<(&'static str, [f64; 2], [f64; 2])> {
    let pack = sim_p38g::p38g_instrument_pack().expect("pack");
    SEEN.iter()
        .map(|(slot, px)| {
            let place = pack
                .layout
                .iter()
                .find(|place| place.slot == *slot)
                .unwrap_or_else(|| panic!("the pack has no slot {slot}"));
            (*slot, place.offset_m, *px)
        })
        .collect()
}

#[test]
fn the_pack_layout_lands_on_the_instruments_of_lockheeds_frontal_photograph() {
    let slots = seen_slots();
    let pairs: Vec<_> = slots.iter().map(|(_, board, px)| (*board, *px)).collect();
    let h = homography(&pairs).expect("homography");
    let errors: Vec<(&str, f64)> = slots
        .iter()
        .map(|(slot, board, [u, v])| {
            let [pu, pv] = project(&h, *board);
            (*slot, (pu - u).hypot(pv - v))
        })
        .collect();
    let rms = (errors.iter().map(|(_, e)| e * e).sum::<f64>() / errors.len() as f64).sqrt();
    // About 1.3 px to the millimetre at the board: 5 px rms is 4 mm.
    assert!(rms < 5.0, "rms {rms:.1} px: {errors:?}");
    for (slot, error) in &errors {
        assert!(*error < 10.0, "{slot} is {error:.1} px off: {errors:?}");
    }
}

/// Each slot must land on its own instrument when placed by the others alone,
/// so one misplaced slot cannot hide in a fit it helped make.
#[test]
fn each_slot_is_placed_by_the_rest_of_the_board() {
    let slots = seen_slots();
    for (i, (slot, board, [u, v])) in slots.iter().enumerate() {
        let others: Vec<_> = slots
            .iter()
            .enumerate()
            .filter(|(j, _)| *j != i)
            .map(|(_, (_, board, px))| (*board, *px))
            .collect();
        let h = homography(&others).expect("homography");
        let [pu, pv] = project(&h, *board);
        let error = (pu - u).hypot(pv - v);
        // 16 px is 12 mm, a seventh of a 3-1/8 in case.
        assert!(
            error < 16.0,
            "{slot} lands {error:.1} px from its instrument"
        );
    }
}

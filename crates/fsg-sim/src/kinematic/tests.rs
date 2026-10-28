use fsg_geo::{Geodetic, LocalFrame};

use super::*;

fn start() -> KinematicModel {
    KinematicModel::new(VehicleState::level_flight(
        &Geodetic::from_degrees(37.62, -122.38, 1_000.0),
        0.0,
        60.0,
    ))
}

fn run(model: &mut KinematicModel, steps: usize, controls: ControlInputs) {
    for _ in 0..steps {
        model.step(&KinematicInput { controls }).unwrap();
    }
}

#[test]
fn neutral_controls_fly_north_and_hold_height() {
    let mut model = start();
    let before = model.output();
    run(
        &mut model,
        1_200,
        ControlInputs {
            throttle: 0.333,
            ..Default::default()
        },
    );
    let after = model.output();
    let travel = LocalFrame::at(&fsg_geo::ecef_to_geodetic(before.position_ecef))
        .ecef_to_enu_vector(after.position_ecef - before.position_ecef);
    assert!(travel.y > 500.0 && travel.x.abs() < 1.0, "{travel:?}");
    let height = fsg_geo::ecef_to_geodetic(after.position_ecef).height_m;
    assert!((height - 1_000.0).abs() < 1.0, "{height}");
}

#[test]
fn right_stick_rolls_right() {
    let mut model = start();
    run(
        &mut model,
        30,
        ControlInputs {
            roll: 1.0,
            ..Default::default()
        },
    );
    let state = model.output();
    let up = LocalFrame::at(&fsg_geo::ecef_to_geodetic(state.position_ecef)).up();
    let right_wing = state.attitude * DVec3::Y;
    assert!(
        right_wing.dot(up) < -0.2,
        "right wing should drop, dot {}",
        right_wing.dot(up)
    );
}

#[test]
fn runs_are_bit_identical_and_restore_resumes_exactly() {
    let controls = ControlInputs {
        roll: 0.3,
        pitch: 0.2,
        yaw: -0.1,
        throttle: 0.8,
    };
    let mut a = start();
    let mut b = start();
    run(&mut a, 500, controls);
    run(&mut b, 500, controls);
    assert_eq!(a.output(), b.output());

    let saved = a.snapshot();
    run(&mut a, 250, controls);
    b.restore(&saved);
    run(&mut b, 250, controls);
    assert_eq!(a.output(), b.output());
}

#[test]
fn never_descends_below_the_floor() {
    let mut model = start();
    run(
        &mut model,
        6_000,
        ControlInputs {
            pitch: -1.0,
            ..Default::default()
        },
    );
    let height = fsg_geo::ecef_to_geodetic(model.output().position_ecef).height_m;
    assert!(height >= FLOOR_HEIGHT_M - 1e-6, "{height}");
}

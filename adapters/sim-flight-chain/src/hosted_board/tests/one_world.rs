//! The board's devices and its airframe's sensed motion read one world.

use std::sync::Arc;

use sim_devices::MagneticEnvironment;
use sim_geo::BodyEllipsoid;

use super::*;

/// The composed C172 board hands its sensor suite the very magnetic-field
/// instance its airframe holds, and the airframe's sensed specific force is
/// worked out in the gravity of the same body entry. A board that built a
/// field of its own for either side fails here.
#[test]
fn the_boards_devices_and_dynamics_hold_one_bodys_fields() {
    let mut board = c172(4);
    let fields = board.airframe().fields().clone();
    assert!(
        std::ptr::addr_eq(
            Arc::as_ptr(board.suite().magnetic_field()),
            Arc::as_ptr(&fields.magnetic)
        ),
        "the magnetometer reads another instance than the airframe's body"
    );
    assert!(
        !std::ptr::addr_eq(
            Arc::as_ptr(board.suite().magnetic_field()),
            Arc::as_ptr(
                &sim_bodies::BodyDescriptor::earth()
                    .unwrap()
                    .fields()
                    .magnetic
            )
        ),
        "an equal field built elsewhere is another instance"
    );

    // Content: what each side senses is that body's field, to the bit. In
    // trimmed flight the specific force is the reaction to the attraction
    // and the frame's own terms.
    let state = board.true_state();
    let earth_rate = DVec3::Z * BodyEllipsoid::WGS84.omega_rad_s;
    let reaction = |gravity: &dyn sim_geo::GravityField| {
        state.attitude.inverse()
            * (2.0 * earth_rate.cross(state.velocity_ecef)
                + earth_rate.cross(earth_rate.cross(state.position_ecef))
                - gravity.attraction_fixed(state.position_ecef))
    };
    let sensed_force = board.airframe().specific_force_body_m_s2();
    assert_eq!(
        sensed_force.to_array().map(f64::to_bits),
        reaction(fields.gravity.as_ref())
            .to_array()
            .map(f64::to_bits),
        "the specific force is worked out in another gravity than the body's"
    );
    let detailed = sim_bodies::HarmonicGravity::egm96_degree_36().unwrap();
    assert!(
        (sensed_force - reaction(&detailed)).length() > 1e-6,
        "and a different field would have shown"
    );

    for _ in 0..40 {
        board.step().unwrap();
    }
    let truth = board.airframe().sensed().unwrap();
    let state = truth.vehicle;
    let world_ut = MagneticEnvironment::main_field(2026.5)
        .field_fixed_ut(fields.magnetic.as_ref(), state.position_ecef)
        .unwrap();
    let ideal = SensorSuiteSpec {
        magnetometer: sim_devices::MagnetometerSpec::IDEAL,
        magnetometer_every: 1,
        ..c172_sensor_suite(2026.5, 4)
    };
    let mut suite = SensorSuite::new(&ideal, fields.magnetic.clone()).unwrap();
    let sensed = suite
        .sense(&truth, 1.0 / RATE_HZ as f64)
        .unwrap()
        .magnetic_field_body_ut
        .unwrap();
    assert_eq!(
        sensed.to_array().map(f64::to_bits),
        (state.attitude.inverse() * world_ut)
            .to_array()
            .map(f64::to_bits),
        "an ideal magnetometer on the body's field reads that field"
    );
}

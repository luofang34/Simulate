//! Sub-tick display interpolation: the window draws the vehicle between its last two
//! physics steps, so a `120 Hz` plant looks continuous at any frame rate.

use std::f64::consts::TAU;

use glam::DVec3;
use sim_core::{AirframeArticulation, CameraPoseEcef, VehicleState, interpolate_vehicle_state};

/// The vehicle `phase` (`0..=1`) of the way from `prev` to `curr`.
pub(super) fn interpolate(prev: &VehicleState, curr: &VehicleState, phase: f64) -> VehicleState {
    let t = if phase.is_finite() { phase } else { 1.0 };
    interpolate_vehicle_state(prev, curr, t)
}

/// `articulation` with the propellers advanced `phase` of one `period_s` step at their RPMs.
pub(super) fn advance_propeller(
    mut articulation: AirframeArticulation,
    phase: f64,
    period_s: f64,
) -> AirframeArticulation {
    let dt = phase.clamp(0.0, 1.0) * period_s;
    let adv_l = dt * articulation.propeller_rpm * TAU / 60.0;
    if adv_l.is_finite() && !articulation.propeller_feathered[0] {
        articulation.propeller_phase_rad = (articulation.propeller_phase_rad
            + articulation.propeller_sense * adv_l)
            .rem_euclid(TAU);
    }
    let adv_r = dt * articulation.propeller_rpm_right * TAU / 60.0;
    if adv_r.is_finite() && !articulation.propeller_feathered[1] {
        articulation.propeller_phase_right_rad = (articulation.propeller_phase_right_rad
            + articulation.propeller_sense_right * adv_r)
            .rem_euclid(TAU);
    }
    articulation
}

/// `pose`, rigidly attached to `exact`, carried onto `display`: the same body-frame lever arm
/// and relative rotation, so a scenario's eye follows the interpolated airframe.
pub(super) fn reanchor(
    pose: CameraPoseEcef,
    exact: &VehicleState,
    display: &VehicleState,
) -> CameraPoseEcef {
    let to_body = exact.attitude.inverse();
    let lever_body: DVec3 = to_body * (pose.position_ecef - exact.position_ecef);
    let relative = to_body * pose.orientation;
    CameraPoseEcef {
        position_ecef: display.position_ecef + display.attitude * lever_body,
        orientation: (display.attitude * relative).normalize(),
    }
}

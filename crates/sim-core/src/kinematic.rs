use glam::{DQuat, DVec3};
use sim_geo::{ecef_to_geodetic, geodetic_to_ecef};

use crate::{Block, BlockError, ControlInputs, TICKS_PER_SECOND, TickSpan, VehicleState};

#[cfg(test)]
mod tests;

/// Full-stick body rates, rad/s: roll, pitch, yaw.
const MAX_RATES: DVec3 = DVec3::new(1.0, 0.35, 0.2);
/// Speed range from idle to full throttle, m/s.
const IDLE_SPEED_M_S: f64 = 30.0;
const FULL_SPEED_M_S: f64 = 120.0;
/// Time constant of the speed response, seconds.
const SPEED_TIME_CONSTANT_S: f64 = 4.0;
/// Lowest height above the ellipsoid the placeholder will descend to.
const FLOOR_HEIGHT_M: f64 = 2.0;
const STEP_HZ: u64 = 120;
// A rate that does not divide the master clock would drift, so it is a build
// error rather than a runtime fallback.
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(STEP_HZ));
const STEP_PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / STEP_HZ);

/// Inputs to [`KinematicModel`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct KinematicInput {
    /// Pilot controls.
    pub controls: ControlInputs,
}

/// A stand-in vehicle that turns stick deflection directly into body rates
/// and throttle into airspeed, with no aerodynamics. It keeps input,
/// scheduling, camera, and rendering runnable with no aircraft model, and
/// shares its input and output with [`crate::JsbsimFdmBlock`].
#[derive(Debug, Clone, PartialEq)]
pub struct KinematicModel {
    state: VehicleState,
    speed_m_s: f64,
}

impl KinematicModel {
    /// A model starting from `state`, moving at the speed of its velocity.
    pub fn new(state: VehicleState) -> Self {
        Self {
            speed_m_s: state.velocity_ecef.length(),
            state,
        }
    }
}

impl Block for KinematicModel {
    type Input = KinematicInput;
    type Output = VehicleState;
    type Snapshot = Self;

    fn period(&self) -> TickSpan {
        STEP_PERIOD
    }

    fn step(&mut self, input: &KinematicInput) -> Result<(), BlockError> {
        let dt = self.period().as_seconds();
        let controls = input.controls.clamped();
        let rates = MAX_RATES * DVec3::new(controls.roll, controls.pitch, controls.yaw);
        let target_speed = IDLE_SPEED_M_S + (FULL_SPEED_M_S - IDLE_SPEED_M_S) * controls.throttle;
        self.speed_m_s += (target_speed - self.speed_m_s) * (dt / SPEED_TIME_CONSTANT_S);

        let state = &mut self.state;
        state.attitude = (state.attitude * DQuat::from_scaled_axis(rates * dt)).normalize();
        state.angular_rate_body = rates;
        let previous_velocity = state.velocity_ecef;
        state.velocity_ecef = state.forward_ecef() * self.speed_m_s;
        // The trapezoid makes the step a constant-acceleration arc, so the
        // samples are the path a strapdown IMU would sense: an Euler step on
        // the new velocity alone puts the path half a step's Δv·dt off it.
        state.position_ecef += (previous_velocity + state.velocity_ecef) * (0.5 * dt);

        let mut position = ecef_to_geodetic(state.position_ecef);
        if position.height_m < FLOOR_HEIGHT_M {
            position.height_m = FLOOR_HEIGHT_M;
            state.position_ecef = geodetic_to_ecef(&position);
        }
        Ok(())
    }

    fn output(&self) -> VehicleState {
        self.state
    }

    fn snapshot(&self) -> Self {
        self.clone()
    }

    fn restore(&mut self, snapshot: &Self) {
        self.clone_from(snapshot);
    }
}

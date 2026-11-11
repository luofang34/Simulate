//! What the flight controller reports, the avionics' view of the flight.
//!
//! Every field comes from the Aviate kernel's own state or from the device
//! samples it consumed on its last cycle. The airframe is never read, so a
//! display drawn from it shows what the aeroplane's own systems know and
//! keeps the sensor boundary a real installation has.

#[cfg(test)]
mod tests;

use aviate_core::control::ControlMode;
use aviate_core::control::fixed_wing::FixedWingController;
use aviate_core::mixer::Mixer;
use glam::DVec3;
use sim_core::{SimInstant, VehicleState};

use crate::{Airframe, HostedBoard};

/// The flight controller's account of one cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FcAvionics {
    /// The end of the cycle.
    pub at: SimInstant,
    /// The flight controller's estimate in WGS84 body-fixed axes: position,
    /// velocity, attitude body to ECEF, and the body rates it flies on.
    /// `None` while the estimator grades it unusable.
    pub estimate: Option<VehicleState>,
    /// The accelerometers' reading of the cycle, body forward-right-down,
    /// m/s²: the sample the flight controller consumed.
    pub specific_force_body_m_s2: Option<DVec3>,
    /// The flight controller's air data, from the first cycle it flies on them: the
    /// fixed-wing law takes its first pitot-static sample once armed.
    pub air: Option<FcAirData>,
    /// The control mode the flight controller flew.
    pub mode: ControlMode,
    /// Whether the actuator outputs are armed.
    pub armed: bool,
    /// The setpoint the flight controller executed.
    pub targets: FcTargets,
}

/// The flight controller's filtered pitot-static reading.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FcAirData {
    /// Indicated airspeed, m/s: the dynamic pressure at sea-level density.
    pub indicated_m_s: f64,
    /// True airspeed, m/s.
    pub true_m_s: f64,
    /// Static pressure, pascals, when the static port has delivered one.
    pub static_pressure_pa: Option<f64>,
    /// Whether the last sample is recent enough for the controller to fly
    /// on; a stale reading is the last one held.
    pub fresh: bool,
}

/// The targets of the setpoint the flight controller executed.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FcTargets {
    /// Heading, radians clockwise from north.
    pub heading_rad: Option<f64>,
    /// Height above the WGS84 ellipsoid, metres. The controller holds a
    /// height above its frame's datum, the configured origin's ellipsoidal
    /// height, which this adds back.
    pub altitude_m: Option<f64>,
    /// Indicated airspeed, m/s.
    pub airspeed_m_s: Option<f64>,
    /// Climb rate, m/s, up positive.
    pub vertical_speed_m_s: Option<f64>,
}

impl<A, M> HostedBoard<A, FixedWingController, M>
where
    A: Airframe,
    M: Mixer,
{
    /// What the flight controller reports for its last cycle.
    pub fn avionics(&self) -> FcAvionics {
        let runner = self.controller();
        let board = runner.board();
        let latch = &board.kernel().state.controller.air;
        let air = latch.primed.then(|| FcAirData {
            indicated_m_s: f64::from(latch.indicated_mps),
            true_m_s: f64::from(latch.true_mps),
            static_pressure_pa: latch.static_primed.then(|| f64::from(latch.static_pa)),
            fresh: latch.is_fresh(),
        });
        let specific_force_body_m_s2 = self
            .last_samples()
            .imu
            .filter(|increment| increment.dt_s > 0.0)
            .map(|increment| increment.delta_velocity_m_s / increment.dt_s);
        let command = board.last_effective_command();
        let setpoint = &command.setpoint;
        let datum_m = self.origin().height_m;
        FcAvionics {
            at: self.now(),
            estimate: runner.estimated_state(),
            specific_force_body_m_s2,
            air,
            mode: command.mode,
            armed: runner.is_armed(),
            targets: FcTargets {
                heading_rad: setpoint.heading.map(|h| f64::from(h.0)),
                altitude_m: setpoint.altitude.map(|h| datum_m + f64::from(h.0)),
                airspeed_m_s: setpoint.airspeed.map(|v| f64::from(v.0)),
                vertical_speed_m_s: setpoint.vertical_speed.map(|v| -f64::from(v.0)),
            },
        }
    }
}

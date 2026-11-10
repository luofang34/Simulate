//! The flight state the gyro, turn and compass instruments read, and the time
//! the cockpit clock shows.

use std::f64::consts::TAU;

use glam::DVec3;
use sim_core::{SimInstant, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_indicate::{FlightSample, StateFeed};
use sim_scenario::ScenarioWorld;
use tracing::debug;

use super::GaugeInputs;
use crate::avionics::earth_aircraft_feed_config;

const SECONDS_PER_DAY: f64 = 86_400.0;

impl GaugeInputs {
    /// These readings with the flight state of `vehicle` at `now`: attitude,
    /// rates, heading with the World Magnetic Model's variation, and the
    /// lateral `specific_force_frd_m_s2` the turn indicator's ball feels; and
    /// the clock set to the local solar time `world`'s Sun shows at the
    /// aircraft, running on with simulation time. Only an aircraft over the
    /// Earth has these.
    pub fn with_flight(
        mut self,
        now: SimInstant,
        vehicle: VehicleState,
        specific_force_frd_m_s2: Option<DVec3>,
        world: ScenarioWorld,
    ) -> Self {
        if world.body() != BodyEllipsoid::WGS84 {
            return self;
        }
        let sun_ecef = world.sun_direction_ecef();
        let sample = FlightSample {
            specific_force_body_m_s2: specific_force_frd_m_s2,
            ..FlightSample::truth(now, vehicle)
        };
        self.flight = StateFeed::new(earth_aircraft_feed_config())
            .state(&sample)
            .inspect_err(|err| debug!(error = %err, "no flight state for the panel"))
            .ok();
        self.clock_time_of_day_s = Some(solar_time_of_day_s(
            vehicle.position_ecef,
            sun_ecef,
            now.duration_since(SimInstant::START).as_seconds(),
        ));
        self
    }
}

/// Local apparent solar time at `position_ecef` under a Sun in direction
/// `sun_ecef`, advanced `elapsed_s`, seconds after midnight: noon where the
/// Sun stands on the meridian.
pub(super) fn solar_time_of_day_s(position_ecef: DVec3, sun_ecef: DVec3, elapsed_s: f64) -> f64 {
    let longitude = position_ecef.y.atan2(position_ecef.x);
    let subsolar = sun_ecef.y.atan2(sun_ecef.x);
    let hour_angle = longitude - subsolar;
    (SECONDS_PER_DAY * (0.5 + hour_angle / TAU) + elapsed_s).rem_euclid(SECONDS_PER_DAY)
}

//! Requests the trial tests share.

#![allow(
    dead_code,
    reason = "each test crate that includes this support uses part of it"
)]

use sim_navigate::NavMeans;
use sim_navigate::trial::{
    AlignmentSpec, AvailabilitySchedule, ImuGradeName, OutputSpec, SeedSpec, SensorKind,
    SensorSetSpec, TrialRequest, UnavailableInterval, VehicleSpec, WaypointSpec,
};

/// The name of the GNSS jamming schedule the outage sets honour.
pub const GNSS_JAMMED: &str = "gnss-jammed";

/// A night over the Delaware corridor.
pub const NIGHT_UTC: &str = "2026-03-20T02:00:00Z";
/// Local solar noon over the corridor on the same day.
pub const NOON_UTC: &str = "2026-03-20T17:05:00Z";

pub fn waypoint(ident: &str, lat_deg: f64, lon_deg: f64, altitude_m: f64) -> WaypointSpec {
    WaypointSpec {
        ident: ident.into(),
        lat_deg,
        lon_deg,
        altitude_m,
        speed_m_s: None,
        fly_over: false,
    }
}

/// Four waypoints about 8 and 7 km apart: east, a 70° left turn with a
/// 300 m climb, then an 80° right turn.
pub fn four_waypoints() -> Vec<WaypointSpec> {
    vec![
        waypoint("W0", 39.0, -75.6, 1_000.0),
        waypoint("W1", 39.0, -75.5075, 1_000.0),
        waypoint("W2", 39.0592, -75.4798, 1_300.0),
        waypoint("W3", 39.0483, -75.4001, 1_300.0),
    ]
}

/// A straight eastbound route of at least `minutes` at 60 m/s: a few
/// seconds over, since the last waypoint captures 100 m short.
pub fn straight(minutes: f64) -> Vec<WaypointSpec> {
    let length_deg = 60.0 * (60.0 * minutes + 10.0) / 86_500.0;
    vec![
        waypoint("S0", 39.0, -75.6, 1_000.0),
        waypoint("S1", 39.0, -75.6 + length_deg, 1_000.0),
    ]
}

/// A sensor set over `means`, every sensor available.
pub fn set(name: &str, means: &[NavMeans]) -> SensorSetSpec {
    SensorSetSpec {
        name: name.into(),
        means: means.to_vec(),
        availability: Default::default(),
    }
}

/// `set` with `sensor` withheld by the schedule `schedule`.
pub fn withheld(mut set: SensorSetSpec, sensor: SensorKind, schedule: &str) -> SensorSetSpec {
    set.availability.insert(sensor, schedule.into());
    set
}

/// A set over `means` honouring [`GNSS_JAMMED`].
pub fn jammed_set(name: &str, means: &[NavMeans]) -> SensorSetSpec {
    withheld(set(name, means), SensorKind::Gnss, GNSS_JAMMED)
}

/// A schedule withholding `sensor` from `start_s` to `end_s`.
pub fn schedule(name: &str, sensor: SensorKind, start_s: f64, end_s: f64) -> AvailabilitySchedule {
    AvailabilitySchedule {
        name: name.into(),
        sensor,
        unavailable: vec![UnavailableInterval {
            start_s,
            end_s: Some(end_s),
            reason: None,
        }],
    }
}

/// A schedule withholding `sensor` from `start_s` to the end of the flight.
pub fn schedule_from(name: &str, sensor: SensorKind, start_s: f64) -> AvailabilitySchedule {
    let mut schedule = schedule(name, sensor, start_s, start_s);
    schedule.unavailable[0].end_s = None;
    schedule
}

/// The [`GNSS_JAMMED`] schedule over `start_s` to `end_s`.
pub fn gnss_jammed(start_s: f64, end_s: f64) -> AvailabilitySchedule {
    schedule(GNSS_JAMMED, SensorKind::Gnss, start_s, end_s)
}

/// The reference sets of the trial; the outage sets honour
/// [`GNSS_JAMMED`], which a request running them must declare.
pub fn reference_sets() -> Vec<SensorSetSpec> {
    use NavMeans::{Barometric, Celestial, DeadReckoning, Gnss};
    vec![
        set("gnss+baro+ins", &[DeadReckoning, Barometric, Gnss]),
        set("ins", &[DeadReckoning]),
        set("ins+baro", &[DeadReckoning, Barometric]),
        set("ins+baro+stars", &[DeadReckoning, Barometric, Celestial]),
        jammed_set("gnss-outage+baro+ins", &[DeadReckoning, Barometric, Gnss]),
        jammed_set(
            "gnss-outage+baro+ins+stars",
            &[DeadReckoning, Barometric, Gnss, Celestial],
        ),
    ]
}

/// A request over `waypoints` with the default sensor models.
pub fn request(
    waypoints: Vec<WaypointSpec>,
    grade: ImuGradeName,
    sets: Vec<SensorSetSpec>,
) -> TrialRequest {
    TrialRequest {
        epoch_utc: NIGHT_UTC.into(),
        waypoints,
        vehicle: VehicleSpec {
            cruise_speed_m_s: 60.0,
            bank_limit_deg: 18.0,
            max_climb_m_s: 5.0,
            capture_radius_m: 100.0,
            guidance: Default::default(),
            guidance_min_quality: None,
        },
        imu_grade: grade,
        alignment: AlignmentSpec::default(),
        gnss: Default::default(),
        baro: Default::default(),
        stars: Default::default(),
        availability: Vec::new(),
        sensor_sets: sets,
        seeds: SeedSpec {
            imu: 0x5_7A2,
            alignment: 31,
            gnss: 0x6E55,
            baro: 0xBA20,
            star_camera: 7,
            terrain_camera: None,
        },
        output: OutputSpec::default(),
        termination: None,
        terrain_camera: None,
        maps: None,
    }
}

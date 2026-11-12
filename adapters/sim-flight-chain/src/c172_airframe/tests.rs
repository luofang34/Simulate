use sim_atmosphere::GeoidStandardAtmosphere;
use sim_aviate::DirectActuatorCmd;
use sim_core::TICKS_PER_SECOND;
use sim_devices::AirFlow;
use sim_geo::LocalFrame;

use super::*;
use crate::C172_RATE_HZ;

/// Flies the trimmed C172 east for `seconds` in a wind of `wind_ned_m_s` and
/// returns its track and heading, degrees, and its air-relative speed as
/// its sensors see it, m/s.
fn east_in(wind_ned_m_s: DVec3, seconds: f64) -> (f64, f64, f64) {
    let start = C172Start {
        position: Geodetic::from_degrees(41.70, -1.07, 2050.0),
        heading_rad: core::f64::consts::FRAC_PI_2,
        speed_m_s: 60.0,
        period: TickSpan::from_ticks(TICKS_PER_SECOND / C172_RATE_HZ),
        utc: UtcTime::from_unix_seconds(1_781_525_040.0),
    };
    let air = SharedAir::new(GeoidStandardAtmosphere::with_wind(wind_ned_m_s));
    let mut airframe = C172Airframe::trimmed(&start, air).unwrap();
    for _ in 0..(seconds * C172_RATE_HZ as f64) as u64 {
        airframe.fly(&DirectActuatorCmd::default()).unwrap();
    }
    let sensed = airframe.sensed().unwrap();
    let state = sensed.vehicle;
    let frame = LocalFrame::at(&ecef_to_geodetic(state.position_ecef));
    let velocity = frame.ecef_to_ned_vector(state.velocity_ecef);
    let nose = frame.ecef_to_ned_vector(state.attitude * DVec3::X);
    let flow = AirFlow::of(&sensed.air, &state);
    (
        velocity.y.atan2(velocity.x).to_degrees(),
        nose.y.atan2(nose.x).to_degrees(),
        flow.air_velocity_body_m_s.length(),
    )
}

/// The flight model and the sensors share one wind: in a steady crosswind
/// the aeroplane holds its heading through the air and drifts with the
/// wind by `atan(wind / airspeed)`, and the air-relative speed its sensors
/// read is the speed it was trimmed to.
#[test]
fn a_crosswind_drifts_the_track_off_the_heading() {
    let (calm_track, calm_heading, calm_speed) = east_in(DVec3::ZERO, 20.0);
    assert!(
        (calm_track - calm_heading).abs() < 0.5,
        "{calm_track} {calm_heading}"
    );
    let (track, heading, speed) = east_in(DVec3::new(10.0, 0.0, 0.0), 20.0);
    let drift = heading - track;
    let expected = (10.0 / speed).atan().to_degrees();
    assert!(
        (drift - expected).abs() < 1.0,
        "drift {drift} against {expected}"
    );
    assert!(
        (speed - calm_speed).abs() < 1.0,
        "{speed} against {calm_speed}"
    );
}

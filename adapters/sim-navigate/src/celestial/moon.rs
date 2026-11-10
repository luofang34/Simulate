use glam::DVec3;
use navigate_celestial::{
    AnalyticEphemeris, Correction, Ephemeris, Epoch, FrameId, State, civil_time, observe_body,
};
use sim_geo::{InertialTransform, UtcTime};

use super::frame::CelestialFrame;

/// IAU 2015 nominal solar radius, metres.
const SUN_RADIUS_M: f64 = 695_700_000.0;
const SUN: i32 = 10;
/// The Moon's centre with J2000 axes, a frame [`AnalyticEphemeris`] accepts
/// for an observer.
const MOON_J2000: FrameId = FrameId {
    center_naif_id: 301,
    orientation_naif_id: 1,
};

/// The rotation from J2000 to the Moon's mean-Earth/polar-axis frame at the
/// civil reading `epoch`.
pub(super) fn moon_orientation(epoch: Epoch) -> InertialTransform {
    InertialTransform::moon_at(UtcTime::from_unix_seconds(epoch.to_unix_seconds()))
}

/// The sky for an observer at `observer_fixed_m` in Moon-fixed axes, evaluated
/// with `provider` first and falling back to [`AnalyticEphemeris`] outside
/// `provider`'s coverage.
pub(super) fn frame_with<P: Ephemeris>(
    provider: &P,
    epoch: Epoch,
    observer_fixed_m: DVec3,
) -> Option<CelestialFrame> {
    let orientation = moon_orientation(epoch);
    let observer_j2000 = orientation.fixed_to_j2000(observer_fixed_m);
    let observer = State {
        position_m: observer_j2000.to_array(),
        velocity_m_s: [0.0; 3],
        epoch: civil_time(epoch).terrestrial,
        frame: MOON_J2000,
    };
    let sight = observe_body(provider, observer, SUN, Correction::VacuumReception)
        .ok()
        .or_else(|| {
            observe_body(
                &AnalyticEphemeris,
                observer,
                SUN,
                Correction::VacuumReception,
            )
            .ok()
        })?;
    let direction = orientation
        .j2000_to_fixed(DVec3::from_array(sight.direction))
        .normalize();
    let radius_deg = (SUN_RADIUS_M / sight.coordinate_range_m)
        .clamp(-1.0, 1.0)
        .asin()
        .to_degrees();
    Some(CelestialFrame::with_sun(direction, radius_deg))
}

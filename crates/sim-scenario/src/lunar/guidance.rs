use glam::{DMat3, DQuat, DVec3};
use sim_scenery::{ElevationSource, LunarTranquilityScenery};

/// Below this time to go, the velocity-matching law asks for unbounded
/// acceleration, so it stops shrinking, seconds.
const MIN_TIME_TO_GO_S: f64 = 3.0;
/// Touchdown phase gains: descent rate error to acceleration (1/s), and
/// position and velocity across the surface to acceleration (1/s², 1/s).
const SINK_GAIN: f64 = 0.8;
const LATERAL_POSITION_GAIN: f64 = 0.08;
const LATERAL_VELOCITY_GAIN: f64 = 0.7;
/// Natural frequency and damping of the attitude loop.
const ATTITUDE_FREQUENCY_RAD_S: f64 = 0.8;
const ATTITUDE_DAMPING: f64 = 1.0;
/// Largest moment the reaction control jets can make, N·m.
const RCS_MAX_MOMENT_NM: f64 = 3_000.0;
/// Ring of terrain samples around a candidate landing point.
const HAZARD_RING_M: f64 = 7.0;
const HAZARD_SAMPLES: u32 = 12;

/// A position and velocity a vehicle should reach together.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct ApproachTarget {
    pub(super) position_m: DVec3,
    pub(super) velocity_m_s: DVec3,
}

/// The acceleration that carries a vehicle to `target` in `time_to_go_s`,
/// matching its velocity, with no thought for gravity: the quadratic explicit
/// guidance law of a powered descent's final approach.
pub(super) fn approach_acceleration(
    position_m: DVec3,
    velocity_m_s: DVec3,
    target: &ApproachTarget,
    time_to_go_s: f64,
) -> DVec3 {
    let t = time_to_go_s.max(MIN_TIME_TO_GO_S);
    (target.position_m - position_m) * 6.0 / (t * t)
        - (velocity_m_s * 4.0 + target.velocity_m_s * 2.0) / t
}

/// The acceleration that holds `sink_m_s` straight down onto a target beneath
/// the vehicle.
pub(super) fn touchdown_acceleration(
    position_m: DVec3,
    velocity_m_s: DVec3,
    target_m: DVec3,
    up: DVec3,
    sink_m_s: f64,
) -> DVec3 {
    let across = |v: DVec3| v - up * v.dot(up);
    up * ((-sink_m_s - velocity_m_s.dot(up)) * SINK_GAIN)
        + across(target_m - position_m) * LATERAL_POSITION_GAIN
        - across(velocity_m_s) * LATERAL_VELOCITY_GAIN
}

/// The attitude with the thrust axis (`+x`) along `thrust_direction` and the
/// Commander's window (`+z`) as near `window_towards` as that leaves it.
pub(super) fn thrust_attitude(thrust_direction: DVec3, window_towards: DVec3) -> DQuat {
    let x = thrust_direction.normalize();
    let z = (window_towards - x * window_towards.dot(x)).normalize();
    DQuat::from_mat3(&DMat3::from_cols(x, z.cross(x), z)).normalize()
}

/// The jets' moment, body axes, turning `attitude` to `wanted`. Both are body
/// to the same frame.
pub(super) fn attitude_moment_nm(
    attitude: DQuat,
    wanted: DQuat,
    rate_body_rad_s: DVec3,
    inertia_body: DMat3,
) -> DVec3 {
    let error = attitude.inverse() * wanted;
    let error = if error.w < 0.0 { -error } else { error };
    let omega = ATTITUDE_FREQUENCY_RAD_S;
    let demand =
        error.to_scaled_axis() * omega * omega - rate_body_rad_s * 2.0 * ATTITUDE_DAMPING * omega;
    (inertia_body * demand).clamp_length_max(RCS_MAX_MOMENT_NM)
}

/// The spread of terrain height over a footprint at `east_m`, `north_m`: what
/// a landing pad's boulders and crater walls look like to the crew. Ground
/// with no data counts as infinitely rough.
pub(super) fn roughness_m(scenery: &LunarTranquilityScenery, east_m: f64, north_m: f64) -> f64 {
    let height =
        |east: f64, north: f64| match scenery.ground_height_m(&scenery.at_local(east, north)) {
            Ok(Some(height)) => height,
            _ => f64::INFINITY,
        };
    let centre = height(east_m, north_m);
    (0..HAZARD_SAMPLES)
        .map(|index| {
            let (sin, cos) =
                (f64::from(index) * std::f64::consts::TAU / f64::from(HAZARD_SAMPLES)).sin_cos();
            (height(east_m + HAZARD_RING_M * cos, north_m + HAZARD_RING_M * sin) - centre).abs()
        })
        .fold(0.0, f64::max)
}

use glam::{DMat3, DQuat, DVec3};

/// Natural frequency and damping of the roll loop about the velocity.
const BANK_FREQUENCY_RAD_S: f64 = 1.0;
const BANK_DAMPING: f64 = 1.0;
/// Damping of the rates the aerodynamics otherwise leave ringing, 1/s.
const RATE_DAMPING_PER_S: f64 = 0.4;
/// Largest moment the CM's reaction control jets can make, N·m.
const RCS_MAX_MOMENT_NM: f64 = 3_000.0;
/// Atmospheric scale height at the altitudes that carry the load, metres.
const SCALE_HEIGHT_M: f64 = 7_000.0;
/// The capsule's trimmed lift-to-drag ratio.
const LIFT_TO_DRAG: f64 = 0.3;
/// Sink rate error and drag error to vertical acceleration, 1/s and unitless.
const SINK_GAIN: f64 = 0.1;
const DRAG_GAIN: f64 = 0.15;

/// An instant of the entry, as the drag-holding law sees it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct EntryCondition {
    /// Sensed deceleration, m/s².
    pub(super) drag_m_s2: f64,
    /// Speed, m/s.
    pub(super) speed_m_s: f64,
    /// Rate of climb, m/s.
    pub(super) climb_m_s: f64,
    /// Distance from the Earth's centre, metres.
    pub(super) radius_m: f64,
    /// Gravity, m/s².
    pub(super) gravity_m_s2: f64,
}

/// The cosine of the bank that holds drag at `target_drag_m_s2`, on `-1..=1`.
///
/// Holding drag constant while the capsule slows means sinking at a rate set
/// by the scale height, so the law flies the vertical channel to that rate
/// with the part of the lift it points upwards. Lift up is the natural result
/// early on, while the steep arrival is still being pulled out of.
pub(super) fn lift_up_fraction(condition: &EntryCondition, target_drag_m_s2: f64) -> f64 {
    let &EntryCondition {
        drag_m_s2,
        speed_m_s,
        climb_m_s,
        radius_m,
        gravity_m_s2,
    } = condition;
    let speed = speed_m_s.max(1.0);
    let lift = (LIFT_TO_DRAG * drag_m_s2).max(1e-3);
    let climb_for_constant_drag = 2.0 * drag_m_s2 * SCALE_HEIGHT_M / speed;
    let wanted = SINK_GAIN * (climb_for_constant_drag - climb_m_s)
        + DRAG_GAIN * (drag_m_s2 - target_drag_m_s2);
    let weight_less_centrifugal = gravity_m_s2 - speed * speed / radius_m;
    ((wanted + weight_less_centrifugal) / lift).clamp(-1.0, 1.0)
}

/// The bank angle of `attitude` flying along `velocity` at `position`: the
/// rotation of the lift vector about the velocity, zero with lift straight up
/// and positive to the right of the velocity as seen from behind.
///
/// Lift lies on the side of the velocity the centre of gravity sits on, body
/// `-z`.
pub(super) fn bank_rad(attitude: DQuat, position: DVec3, velocity: DVec3) -> f64 {
    let along = velocity.normalize();
    let lift_up = (position - along * position.dot(along)).normalize();
    let body_minus_z = attitude * DVec3::NEG_Z;
    let lift = body_minus_z - along * body_minus_z.dot(along);
    lift.dot(along.cross(lift_up)).atan2(lift.dot(lift_up))
}

/// The jets' moment, body axes: rolls about the velocity to `wanted_bank_rad`
/// and damps the other axes.
pub(super) fn bank_moment_nm(
    attitude: DQuat,
    position: DVec3,
    velocity: DVec3,
    rate_body_rad_s: DVec3,
    inertia_body: DMat3,
    wanted_bank_rad: f64,
) -> DVec3 {
    let along_body = (attitude.inverse() * velocity).normalize();
    let error =
        crate::corridor::angle_difference(wanted_bank_rad, bank_rad(attitude, position, velocity));
    let roll_rate = rate_body_rad_s.dot(along_body);
    let omega = BANK_FREQUENCY_RAD_S;
    let roll = omega * omega * error - 2.0 * BANK_DAMPING * omega * roll_rate;
    let other_rates = rate_body_rad_s - along_body * roll_rate;
    let demand = along_body * roll - other_rates * RATE_DAMPING_PER_S;
    (inertia_body * demand).clamp_length_max(RCS_MAX_MOMENT_NM)
}

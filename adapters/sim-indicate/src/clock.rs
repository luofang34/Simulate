use sim_core::{SimInstant, TICKS_PER_SECOND};

const TICKS_PER_MILLISECOND: u64 = TICKS_PER_SECOND / 1_000;

#[cfg(test)]
mod tests;

/// `instant` as whole milliseconds from simulation start, the scale Indicate's
/// comparators run on.
///
/// Rounds down, so a source stamp never lands after the `now` it is read at.
pub fn instant_ms(instant: SimInstant) -> u64 {
    let (seconds, subsecond_ticks) = instant.seconds_and_subsec_ticks();
    seconds
        .saturating_mul(1_000)
        .saturating_add(subsecond_ticks / TICKS_PER_MILLISECOND)
}

/// How long before `now` a reading stamped `timestamp` was taken, milliseconds.
pub fn age_ms(now: SimInstant, timestamp: SimInstant) -> f32 {
    (now.duration_since(timestamp).as_seconds() * 1_000.0) as f32
}

//! Master simulation clock types re-exported from [`sim_spacetime`].

pub use sim_spacetime::{SimInstant, TICKS_PER_SECOND, TickSpan};

#[cfg(test)]
mod tests;

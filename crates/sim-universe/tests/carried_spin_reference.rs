//! Independent physical reference verification suite for carried articulated
//! vehicles across spin regimes, asymmetric inertia coupling, off-CG thrust
//! spin-up, joint stop-limit impacts, and mid-arc exchange/checkpoint control.

#[cfg(test)]
#[path = "carried_spin_reference/arc_continuity.rs"]
mod arc_continuity;
#[cfg(test)]
#[path = "carried_spin_reference/models.rs"]
mod models;
#[cfg(test)]
#[path = "carried_spin_reference/spin_slew.rs"]
mod spin_slew;
#[cfg(test)]
#[path = "carried_spin_reference/thrust_and_stops.rs"]
mod thrust_and_stops;

use std::fmt::Write as _;
use std::io::Write as _;
use std::sync::Once;

use tracing::field::{Field, Visit};
use tracing::span::{Attributes, Record};
use tracing::{Event, Id, Metadata, Subscriber};

#[cfg(test)]
static INIT_LOG: Once = Once::new();

#[cfg(test)]
struct LineVisitor(String);

#[cfg(test)]
impl Visit for LineVisitor {
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        if !self.0.is_empty() {
            self.0.push(' ');
        }
        if write!(self.0, "{}={:?}", field.name(), value).is_err() {}
    }
}

#[cfg(test)]
struct TestLogger;

#[cfg(test)]
impl Subscriber for TestLogger {
    fn enabled(&self, _metadata: &Metadata<'_>) -> bool {
        std::env::var_os("CARRIED_SPIN_TRACE").is_some()
    }

    fn new_span(&self, _span: &Attributes<'_>) -> Id {
        Id::from_u64(1)
    }

    fn record(&self, _span: &Id, _values: &Record<'_>) {}

    fn record_follows_from(&self, _span: &Id, _follows: &Id) {}

    fn event(&self, event: &Event<'_>) {
        if !self.enabled(event.metadata()) {
            return;
        }
        let mut vis = LineVisitor(String::new());
        event.record(&mut vis);
        if writeln!(
            std::io::stdout(),
            "[{}] {}",
            event.metadata().level(),
            vis.0
        )
        .is_err()
        {}
    }

    fn enter(&self, _span: &Id) {}

    fn exit(&self, _span: &Id) {}
}

/// Installs the trace logger once per test binary run.
#[cfg(test)]
pub fn ensure_logger() {
    INIT_LOG.call_once(
        || {
            if tracing::subscriber::set_global_default(TestLogger).is_err() {}
        },
    );
}

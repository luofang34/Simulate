//! The spoofer of the GNSS receiver: when it holds the receiver, the fault
//! it is for the device, and the events of its capture and release.
//!
//! The fault lives in the receiver, so the flight controller and every
//! navigation set take the same dragged fixes; nothing here touches the
//! truth or any consumer.

use sim_devices::GnssSpoof;
use sim_navigate::trial::{EventKind, EventRecord, ProgressRecord, SensorKind};

use crate::{ChainTrialError, GnssSpoofSpec, SpoofSpan};

#[cfg(test)]
mod tests;

/// The spoofer and whether it held the receiver in the last cycle.
pub(super) struct Spoofer {
    spec: GnssSpoofSpec,
    holding: bool,
}

impl Spoofer {
    /// The spoofer of `spec`, or one that holds nothing until switched.
    pub(super) fn new(spec: Option<&GnssSpoofSpec>) -> Result<Self, ChainTrialError> {
        let spec = spec.cloned().unwrap_or_else(GnssSpoofSpec::by_hand);
        let usable =
            spec.rate_m_s.is_finite() && spec.rate_m_s > 0.0 && spec.bearing_deg.is_finite();
        if !usable {
            return Err(ChainTrialError::Invalid {
                detail: format!(
                    "a GNSS spoofer dragging {} m/s toward {} degrees",
                    spec.rate_m_s, spec.bearing_deg
                ),
            });
        }
        Ok(Self {
            spec,
            holding: false,
        })
    }

    /// The spoofer as flown so far: its drag and every span, stated and
    /// switched.
    pub(super) const fn spec(&self) -> &GnssSpoofSpec {
        &self.spec
    }

    /// Captures the receiver from route time `at_s` on, or lets it go.
    pub(super) fn switch(&mut self, on: bool, at_s: f64) {
        let open = self.spec.spans.iter_mut().find(|s| s.end_s.is_none());
        match (open, on) {
            (Some(span), false) => span.end_s = Some(at_s),
            (None, true) => self.spec.spans.push(SpoofSpan {
                start_s: at_s,
                end_s: None,
            }),
            _ => {}
        }
    }

    /// The receiver's fault for the cycle starting at route time `t_s`, and
    /// the event of a capture or release beginning with it.
    pub(super) fn cycle(&mut self, t_s: f64) -> (Option<GnssSpoof>, Option<ProgressRecord>) {
        let holds = self.spec.spans.iter().any(|span| span.contains(t_s));
        let kind = match (self.holding, holds) {
            (false, true) => Some(EventKind::SpoofingStart {
                sensor: SensorKind::Gnss,
                rate_m_s: self.spec.rate_m_s,
                bearing_deg: self.spec.bearing_deg,
            }),
            (true, false) => Some(EventKind::SpoofingEnd {
                sensor: SensorKind::Gnss,
            }),
            _ => None,
        };
        self.holding = holds;
        let fault = holds
            .then(|| GnssSpoof::toward(self.spec.rate_m_s, self.spec.bearing_deg.to_radians()));
        let event = kind.map(|kind| {
            ProgressRecord::Event(EventRecord {
                t_s,
                set: None,
                kind,
            })
        });
        (fault, event)
    }
}

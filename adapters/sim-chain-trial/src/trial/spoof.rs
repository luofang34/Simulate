//! The spoofer of the GNSS receiver: when it holds the receiver, the fault
//! it is for the device, and the events of its capture and release.
//!
//! The fault lives in the receiver, so the flight controller and every
//! navigation set take the same dragged fixes; nothing here touches the
//! truth or any consumer.

use glam::DVec3;
use sim_devices::{GnssReceiver, GnssSpoof};
use sim_navigate::trial::{EventKind, EventRecord, ProgressRecord, SensorKind};

use crate::{ChainTrialError, GnssSpoofSpec, SpoofKind, SpoofSpan, SpoofWindowSpec};

#[cfg(test)]
mod tests;

/// Ground-truth adversarial spoofing state active at one instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ActiveSpoof {
    /// Whether the GNSS receiver is currently held by a spoofer.
    pub(crate) active: bool,
    /// Active spoofing profile kind (`none` when inactive).
    pub(crate) kind: SpoofKind,
    /// Accumulated injected position offset `[east, north, up]`, metres.
    pub(crate) offset_enu_m: [f64; 3],
    /// Active spoofing pull rate `[east, north, up]`, m/s.
    pub(crate) rate_enu_mps: [f64; 3],
    /// Horizontal magnitude of `offset_enu_m`, metres.
    pub(crate) horizontal_mag_m: f64,
}

/// The spoofer and whether it held the receiver in the last cycle.
pub(super) struct Spoofer {
    spec: GnssSpoofSpec,
    windows: Vec<SpoofWindowSpec>,
    holding: bool,
    last_kind: SpoofKind,
}

impl Spoofer {
    /// The spoofer of `spec` and `windows`, or one that holds nothing until
    /// switched.
    pub(super) fn new(
        spec: Option<&GnssSpoofSpec>,
        windows: &[SpoofWindowSpec],
    ) -> Result<Self, ChainTrialError> {
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
        for window in windows {
            validate_window(window)?;
        }
        Ok(Self {
            spec,
            windows: windows.to_vec(),
            holding: false,
            last_kind: SpoofKind::None,
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

    /// Ground-truth adversarial spoofing state read directly from `receiver`.
    pub(super) fn active_state(&self, receiver: &GnssReceiver) -> ActiveSpoof {
        match receiver.spoof() {
            Some((spoof, total_ned_m)) => {
                let offset_enu_m = ned_to_enu(total_ned_m);
                let [east_m, north_m, _] = offset_enu_m;
                let rate_ned = if spoof.drag_ned_m_s.length_squared() > 0.0 {
                    spoof.drag_ned_m_s
                } else {
                    spoof.velocity_ned_m_s
                };
                let rate_enu_mps = ned_to_enu(rate_ned);
                ActiveSpoof {
                    active: true,
                    kind: if self.last_kind == SpoofKind::None {
                        SpoofKind::Ramp
                    } else {
                        self.last_kind
                    },
                    offset_enu_m,
                    rate_enu_mps,
                    horizontal_mag_m: east_m.hypot(north_m),
                }
            }
            None => ActiveSpoof {
                active: false,
                kind: SpoofKind::None,
                offset_enu_m: [0.0; 3],
                rate_enu_mps: [0.0; 3],
                horizontal_mag_m: 0.0,
            },
        }
    }

    /// The receiver's fault for the cycle starting at route time `t_s`, and
    /// the event of a capture or release beginning with it.
    pub(super) fn cycle(&mut self, t_s: f64) -> (Option<GnssSpoof>, Option<ProgressRecord>) {
        let resolved = self.resolve_at(t_s);
        let holds = resolved.is_some();
        let event_kind = match (self.holding, &resolved) {
            (false, Some((_, _, rate_m_s, bearing_deg))) => Some(EventKind::SpoofingStart {
                sensor: SensorKind::Gnss,
                rate_m_s: *rate_m_s,
                bearing_deg: *bearing_deg,
            }),
            (true, None) => Some(EventKind::SpoofingEnd {
                sensor: SensorKind::Gnss,
            }),
            _ => None,
        };
        self.holding = holds;
        self.last_kind = resolved
            .as_ref()
            .map_or(SpoofKind::None, |(_, kind, _, _)| *kind);
        let fault = resolved.map(|(spoof, _, _, _)| spoof);
        let event = event_kind.map(|kind| {
            ProgressRecord::Event(EventRecord {
                t_s,
                set: None,
                kind,
            })
        });
        (fault, event)
    }

    fn resolve_at(&self, t_s: f64) -> Option<(GnssSpoof, SpoofKind, f64, f64)> {
        if !self.windows.is_empty() {
            let window = self.windows.iter().rfind(|w| w.contains(t_s))?;
            return window_fault(window);
        }
        self.spec
            .spans
            .iter()
            .any(|span| span.contains(t_s))
            .then(|| {
                (
                    GnssSpoof::toward(self.spec.rate_m_s, self.spec.bearing_deg.to_radians()),
                    SpoofKind::Ramp,
                    self.spec.rate_m_s,
                    self.spec.bearing_deg,
                )
            })
    }
}

fn validate_window(window: &SpoofWindowSpec) -> Result<(), ChainTrialError> {
    let finite_vec = |v: [f64; 3]| v.iter().all(|c| c.is_finite());
    let valid_times = window.start_s.is_finite()
        && window
            .end_s
            .is_none_or(|end| end.is_finite() && end > window.start_s);
    let valid_vectors = finite_vec(window.offset_enu_m)
        && finite_vec(window.rate_enu_mps)
        && window.velocity_enu_mps.is_none_or(finite_vec);
    if !valid_times || !valid_vectors || window.kind == SpoofKind::None {
        return Err(ChainTrialError::Invalid {
            detail: format!("invalid spoofing window: {window:?}"),
        });
    }
    Ok(())
}

fn window_fault(window: &SpoofWindowSpec) -> Option<(GnssSpoof, SpoofKind, f64, f64)> {
    let offset_ned = enu_to_ned(window.offset_enu_m);
    let rate_ned = enu_to_ned(window.rate_enu_mps);
    let vel_ned = window.velocity_enu_mps.map_or(rate_ned, enu_to_ned);
    let spoof = match window.kind {
        SpoofKind::None => return None,
        SpoofKind::Offset => GnssSpoof::step_offset(offset_ned),
        SpoofKind::Ramp => GnssSpoof::ramp(offset_ned, rate_ned),
        SpoofKind::VelocityPull => GnssSpoof::velocity_pull(offset_ned, rate_ned, vel_ned),
    };
    let active_vec = if rate_ned.length_squared() > 0.0 {
        rate_ned
    } else if vel_ned.length_squared() > 0.0 {
        vel_ned
    } else {
        offset_ned
    };
    let rate_m_s = rate_ned.x.hypot(rate_ned.y);
    let bearing_deg = active_vec
        .y
        .atan2(active_vec.x)
        .to_degrees()
        .rem_euclid(360.0);
    Some((spoof, window.kind, rate_m_s, bearing_deg))
}

const fn enu_to_ned([east, north, up]: [f64; 3]) -> DVec3 {
    DVec3::new(north, east, -up)
}

const fn ned_to_enu(ned: DVec3) -> [f64; 3] {
    [ned.y, ned.x, -ned.z]
}

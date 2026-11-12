//! A navigator's check of its GNSS receiver against a solution kept free of
//! it, as a host reports it: the navigator's own verdict and numbers, in
//! route time.
//!
//! The check is Navigate's. Nothing here judges a fix; this is the status
//! the navigator states, carried to a host's displays.

use navigate_contract::MonotonicNanos;
use navigate_onboard::{GnssIntegrityState, GnssIntegrityStatus};

/// What the check says of the receiver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GnssCheckState {
    /// The fixes agree with the reference.
    Consistent,
    /// The fixes disagree with the reference: still used, with caution.
    Disagreeing,
    /// The receiver is excluded: the navigator no longer uses its fixes.
    Excluded,
}

/// What the receiver is checked against.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GnssReference {
    /// A GNSS-free solution holding a position fix of another sensor.
    PositionFix,
    /// Inertial and barometric navigation only, which finds a drift no
    /// slower than `detectable_rate_m_s`, when the navigator states one.
    InertialOnly {
        /// The slowest drift the reference can find, m/s.
        detectable_rate_m_s: Option<f64>,
    },
}

/// The latest fix against the reference.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GnssCheckSeparation {
    /// Horizontal distance of the fix from the reference, metres.
    pub separation_m: f64,
    /// The separation at which the check cautions, metres.
    pub caution_bound_m: f64,
    /// The separation at which it excludes, metres.
    pub exclusion_bound_m: f64,
}

/// One set's GNSS check at an instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GnssCheck {
    /// The verdict.
    pub state: GnssCheckState,
    /// When the verdict began, on the navigator's clock, when it states it.
    pub since: Option<MonotonicNanos>,
    /// The latest fix against the reference, once one was compared.
    pub last: Option<GnssCheckSeparation>,
    /// What the receiver is checked against.
    pub reference: GnssReference,
    /// Whether the reference is tight enough to check against; without it
    /// the verdict says nothing of the receiver.
    pub checkable: bool,
}

impl GnssCheck {
    /// Route time the verdict began, seconds: the navigator's clock reads
    /// zero at the first route instant.
    pub fn since_s(&self) -> Option<f64> {
        self.since.map(|at| at.as_nanos() as f64 * 1e-9)
    }

    /// The check `status` reports.
    pub fn of(status: &GnssIntegrityStatus) -> Self {
        Self {
            state: match status.state {
                GnssIntegrityState::Consistent => GnssCheckState::Consistent,
                GnssIntegrityState::Disagreeing => GnssCheckState::Disagreeing,
                GnssIntegrityState::Excluded => GnssCheckState::Excluded,
            },
            since: status.since,
            last: status.last.map(|last| GnssCheckSeparation {
                separation_m: last.separation_m,
                caution_bound_m: last.caution_bound_m,
                exclusion_bound_m: last.exclusion_bound_m,
            }),
            reference: match status.reference_position_fix_at {
                Some(_) => GnssReference::PositionFix,
                None => GnssReference::InertialOnly {
                    detectable_rate_m_s: status.inertial_detectable_rate_m_s,
                },
            },
            checkable: status.within_alert_limit,
        }
    }
}

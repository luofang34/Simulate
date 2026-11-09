//! The error budgets of each onboard sensor realism mode.

use super::noise::GaussMarkov;

/// How far the onboard sensors and navigation solution stray from truth.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum SensorRealism {
    /// WAAS GNSS/INS + baro-inertial hybrid navigation, AHRS, BMP390-grade static pressure,
    /// and pitot noise.
    #[default]
    Realistic,
    /// Degraded GNSS with baro-aided vertical damping: tens of metres of slowly wandering
    /// horizontal position error and realistic baro/QNH vertical residual, for checking that
    /// HUD and SVS runway symbology visibly misregisters against the real runway without
    /// unstabilized free-space VDOP plunging `30 m` underground.
    DegradedNavigation,
    /// Zero error: every reading equals truth, for exact calibration.
    Ideal,
}

impl SensorRealism {
    /// The next mode, wrapping.
    pub const fn next(self) -> Self {
        match self {
            Self::Realistic => Self::DegradedNavigation,
            Self::DegradedNavigation => Self::Ideal,
            Self::Ideal => Self::Realistic,
        }
    }

    /// Human-readable name.
    pub const fn title(self) -> &'static str {
        match self {
            Self::Realistic => "realistic",
            Self::DegradedNavigation => "degraded navigation",
            Self::Ideal => "ideal",
        }
    }

    pub(super) fn budget(self) -> Budget {
        let gm = |tau_s, sigma| GaussMarkov { tau_s, sigma };
        let (horizontal, vertical, velocity, attitude, nav_tau_s) = match self {
            // FAA WAAS LPV + RTCA DO-315B conformal HUD/SVS IRS error budget (~0.35 m 1σ
            // horizontal, ~0.018° 1σ attitude), keeping conformal HUD cues within sub-metre
            // tolerances while staying strictly non-zero and deterministic.
            Self::Realistic => (0.35, 0.45, 0.03, 0.000_32, 30.0),
            Self::DegradedNavigation => (18.0, 2.2, 0.6, 0.001_5, 120.0),
            Self::Ideal => (0.0, 0.0, 0.0, 0.0, 1.0),
        };
        let scale = if self == Self::Ideal { 0.0 } else { 1.0 };
        Budget {
            horizontal: gm(nav_tau_s, horizontal),
            vertical: gm(nav_tau_s, vertical),
            velocity: gm(nav_tau_s / 3.0, velocity),
            attitude: gm(60.0, attitude),
            rate: gm(120.0, 0.000_8 * scale),
            accel: gm(300.0, 0.03 * scale),
            static_pa: gm(5.0, 8.0 * scale),
            pitot_m_s: gm(2.0, 0.4 * scale),
        }
    }
}

pub(super) struct Budget {
    pub(super) horizontal: GaussMarkov,
    pub(super) vertical: GaussMarkov,
    pub(super) velocity: GaussMarkov,
    pub(super) attitude: GaussMarkov,
    pub(super) rate: GaussMarkov,
    pub(super) accel: GaussMarkov,
    pub(super) static_pa: GaussMarkov,
    pub(super) pitot_m_s: GaussMarkov,
}

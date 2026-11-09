use navigate_contract::DurationNanos;
use navigate_geodesy::GeodesyError;
use navigate_visual::VisualError;
use navigate_visual::fusion_bridge::VisualFusionError;
use navigate_visual::session::SessionError;
use sim_core::SimInstant;

/// Why the bridge could not carry a value into Navigate.
#[derive(Debug, thiserror::Error)]
pub enum NavigateBridgeError {
    /// A master-clock instant does not fit Navigate's `u64` nanoseconds.
    #[error("instant {instant:?} is past the range of monotonic nanoseconds")]
    InstantOutOfRange {
        /// The offending instant.
        instant: SimInstant,
    },
    /// A position has no coordinates in the corridor's local frame.
    #[error("position {position_ecef_m:?} has no corridor coordinates")]
    OutsideFrame {
        /// The position, ECEF metres.
        position_ecef_m: [f64; 3],
        /// The underlying refusal.
        #[source]
        source: VisualError,
    },
    /// The corridor anchor is not a usable map frame.
    #[error("corridor anchor at {lat_deg}°, {lon_deg}° is not a usable frame")]
    InvalidAnchor {
        /// Anchor latitude, degrees.
        lat_deg: f64,
        /// Anchor longitude, degrees.
        lon_deg: f64,
        /// The underlying refusal.
        #[source]
        source: VisualError,
    },
    /// The attitude allowance is not a finite, non-negative angle.
    #[error("attitude RMS {attitude_rms_rad} rad is not usable")]
    InvalidAttitudeAllowance {
        /// The refused allowance.
        attitude_rms_rad: f64,
    },
    /// The declared calibration error does not cover the lever arm's error
    /// under the declared attitude allowance.
    #[error(
        "calibration allowance {calibration_m} m is below the {lever_arm_error_m} m the lever arm can contribute"
    )]
    LeverArmUncovered {
        /// Declared calibration error, metres.
        calibration_m: f64,
        /// Lever-arm length times the attitude allowance, metres.
        lever_arm_error_m: f64,
    },
    /// Fixes are held longer than the filter accepts them.
    #[error(
        "hold of {} ns reaches the filter's staleness bound of {} ns",
        hold.as_nanos(),
        bound.as_nanos()
    )]
    HoldReachesStaleness {
        /// The configured hold.
        hold: DurationNanos,
        /// The filter's staleness bound.
        bound: DurationNanos,
    },
    /// The visual session refused an input.
    #[error("visual session refused the frame captured at {captured_at:?}")]
    Session {
        /// The frame's capture instant.
        captured_at: SimInstant,
        /// The underlying refusal.
        #[source]
        source: SessionError,
    },
    /// The visual fix source refused its setup or a fix.
    #[error("visual fix source refused the input")]
    VisualFusion(#[source] VisualFusionError),
    /// A solution's position has no local tangent plane.
    #[error("solution position has no local tangent plane")]
    SolutionPlane(#[source] GeodesyError),
    /// A solution's covariance has no Cholesky factor.
    #[error("solution position covariance is not positive definite")]
    SolutionCovariance,
}

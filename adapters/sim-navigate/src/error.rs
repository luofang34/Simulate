use navigate_camera::CameraError;
use navigate_contract::DurationNanos;
use navigate_fusion::RejectionReason;
use navigate_geodesy::GeodesyError;
use navigate_visual::VisualError;
use navigate_visual::fusion_bridge::VisualFusionError;
use navigate_visual::session::SessionError;
use sim_core::SimInstant;
use sim_devices::DeviceError;

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
    /// The ephemeris refused the epoch of a star-camera frame.
    #[error("no star catalogue {elapsed_s} s after the scenario start at UTC {start_unix_s} s")]
    StarEphemeris {
        /// The scenario's start, UTC Unix seconds.
        start_unix_s: f64,
        /// Simulated time since the scenario start, seconds.
        elapsed_s: f64,
        /// The underlying refusal.
        #[source]
        source: navigate_celestial::AnalyticError,
    },
    /// A solution's covariance has no Cholesky factor.
    #[error("solution position covariance is not positive definite")]
    SolutionCovariance,
    /// The camera model refused a sensor description or a frame.
    #[error("camera model refused {what}")]
    Camera {
        /// What was being converted.
        what: &'static str,
        /// The underlying refusal.
        #[source]
        source: CameraError,
    },
    /// A rendered buffer does not hold the pixels its size needs.
    #[error("{what} holds {actual} values; a {width} x {height} frame needs {needed}")]
    FrameSize {
        /// Which buffer.
        what: &'static str,
        /// Frame width, pixels.
        width: u32,
        /// Frame height, pixels.
        height: u32,
        /// Values the frame needs.
        needed: usize,
        /// Values the buffer holds.
        actual: usize,
    },
    /// A score paired an output with truth from another instant.
    #[error("truth at {truth_at:?} scored against an output valid at {valid_at:?}")]
    TruthInstantMismatch {
        /// The truth's instant.
        truth_at: SimInstant,
        /// The instant the output describes.
        valid_at: SimInstant,
    },
    /// The simulated IMU's error model refused its statistics or an
    /// increment.
    #[error("simulated IMU refused its input")]
    ImuModel(#[source] DeviceError),
    /// Two IMU readings did not advance the clock.
    #[error("IMU interval from {start:?} to {end:?} does not advance")]
    ImuIntervalNotAdvancing {
        /// Start of the interval.
        start: SimInstant,
        /// End of the interval.
        end: SimInstant,
    },
    /// The inertial filter refused an alignment or an IMU increment.
    #[error("inertial filter refused the {stage}")]
    InertialRefused {
        /// What was offered.
        stage: &'static str,
        /// The filter's reason.
        #[source]
        reason: RejectionReason,
    },
}

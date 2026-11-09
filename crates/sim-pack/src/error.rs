use sim_core::BlockError;
use syspack::SyspackError;

/// Why a system package could not become a simulated one.
#[derive(Debug, thiserror::Error)]
pub enum PackError {
    /// An `ace-rs` design could not be read or compared with a package.
    #[error(transparent)]
    AceBridge(#[from] crate::calibration::AceBridgeError),
    /// The package itself did not verify, or does not hold what was asked of
    /// it.
    #[error(transparent)]
    Syspack(#[from] SyspackError),
    /// A device block refused its configuration.
    #[error("device {device:?} could not be built")]
    DeviceBuild {
        /// The device.
        device: String,
        /// The block's refusal.
        #[source]
        source: Box<BlockError>,
    },
    /// The package's airspeed envelope JSON could not be parsed.
    #[error("airspeed envelope {path:?} could not be parsed")]
    AirspeedEnvelopeParse {
        /// Asset path inside the package.
        path: String,
        /// JSON decoder error.
        #[source]
        source: serde_json::Error,
    },
    /// The package's airspeed envelope failed physical sanity checks.
    #[error("airspeed envelope {path:?} is invalid: {detail}")]
    InvalidAirspeedEnvelope {
        /// Asset path inside the package.
        path: String,
        /// Why the envelope was refused.
        detail: String,
    },
    /// A package calibration or pitot-static JSON table could not be parsed.
    #[error("calibration asset {path:?} could not be parsed")]
    CalibrationParse {
        /// Asset path inside the package.
        path: String,
        /// JSON decoder error.
        #[source]
        source: serde_json::Error,
    },
    /// A package calibration or pitot-static table failed physical validation.
    #[error("calibration asset {path:?} is invalid: {detail}")]
    InvalidCalibration {
        /// Asset path inside the package.
        path: String,
        /// Why the calibration payload was refused.
        detail: String,
    },
    /// The package's structural landmarks JSON could not be parsed.
    #[error("structural landmarks {path:?} could not be parsed")]
    LandmarksParse {
        /// Asset path inside the package.
        path: String,
        /// JSON decoder error.
        #[source]
        source: serde_json::Error,
    },
    /// The package's structural landmarks are incomplete.
    #[error("structural landmarks {path:?} are invalid: {detail}")]
    InvalidLandmarks {
        /// Asset path inside the package.
        path: String,
        /// Why the landmarks were refused.
        detail: String,
    },
    /// A package's camera views document could not be parsed.
    #[error("camera views {path:?} could not be parsed")]
    CameraViewsParse {
        /// Asset path inside the package.
        path: String,
        /// JSON decoder error.
        #[source]
        source: serde_json::Error,
    },
    /// A package's camera views document declared an unusable view.
    #[error("camera views {path:?} are invalid: {detail}")]
    InvalidCameraViews {
        /// Asset path inside the package.
        path: String,
        /// Which view was refused and why.
        detail: String,
    },
    /// The instrument pack a vehicle pins could not be resolved, or one of its
    /// instruments reads a source the vehicle does not have.
    #[error("instrument pack {pack:?} refused: {detail}")]
    InstrumentPack {
        /// The pinned pack's id.
        pack: String,
        /// Why it was refused.
        detail: String,
    },
    /// Closed-loop FDM execution failed during calibration or sensitivity evaluation.
    #[error("closed-loop calibration run failed: {detail}")]
    CalibrationRun {
        /// Failure diagnostic from JSBSim or the trim/hold loop.
        detail: String,
    },
}

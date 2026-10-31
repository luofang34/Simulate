use mbd_core::MbdError;
use pilotage_aircraft::AircraftError;
use sim_core::BlockError;
use sim_pack::{PackError, SystemKind};

/// Why a vehicle, camera, or package could not cross into Pilotage.
#[derive(Debug, thiserror::Error)]
pub enum PilotageBridgeError {
    /// The dynamics block could not be built.
    #[error("vehicle dynamics could not be built")]
    Block(#[source] Box<BlockError>),
    /// The package could not supply what was asked of it.
    #[error("vehicle package failed")]
    Pack(#[source] Box<PackError>),
    /// The component tree could not be evaluated.
    #[error("component tree failed")]
    Tree(#[source] MbdError),
    /// Pilotage refused the profile or loading.
    #[error("aircraft domain refused the data")]
    Aircraft(#[source] AircraftError),
    /// The package is not something Pilotage's aircraft domain describes.
    #[error("{kind:?} systems have no aircraft profile")]
    UnsupportedKind {
        /// The package's kind.
        kind: SystemKind,
    },
    /// A station or tank names a component the package does not have.
    #[error("{site} is on component {component}, which the package does not have")]
    UnknownComponent {
        /// The station or tank.
        site: String,
        /// The missing component key.
        component: String,
    },
    /// Two stations or two tanks share a component, so its load could not be
    /// split again on the way back.
    #[error("{first} and {second} both load component {component}")]
    SharedComponent {
        /// One station or tank.
        first: String,
        /// The other.
        second: String,
        /// The shared component key.
        component: String,
    },
    /// A loading names a station or tank the specification does not.
    #[error("loading names {site}, which the specification does not have")]
    UnknownSite {
        /// The station or tank.
        site: String,
    },
    /// Malformed or truncated `pilotage.bridge.v1` protobuf wire bytes.
    #[error("video bridge wire failure: {detail}")]
    VideoWire {
        /// Diagnostic detail.
        detail: String,
    },
    /// Underlying TCP sidecar socket I/O error.
    #[error("video bridge socket I/O failed: {0}")]
    VideoIo(#[from] std::io::Error),
    /// A camera pose or zoom detent could not be resolved honestly.
    #[error("camera view resolution failed: {detail}")]
    InvalidCameraView {
        /// Why the camera view was rejected.
        detail: String,
    },
    /// A view pose named a vertical datum whose reference surface was not supplied.
    #[error("vertical datum {datum} cannot be resolved without its reference surface")]
    UnsupportedVerticalDatum {
        /// Wire datum code (`1=ellipsoid`, `2=msl`, `3=agl`, or unknown).
        datum: u32,
    },
}

impl From<BlockError> for PilotageBridgeError {
    fn from(source: BlockError) -> Self {
        Self::Block(Box::new(source))
    }
}

impl From<PackError> for PilotageBridgeError {
    fn from(source: PackError) -> Self {
        Self::Pack(Box::new(source))
    }
}

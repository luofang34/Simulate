use sim_core::BlockError;
use syspack::SyspackError;

/// Why a system package could not become a simulated one.
#[derive(Debug, thiserror::Error)]
pub enum PackError {
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
}

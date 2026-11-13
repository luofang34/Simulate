//! Typed errors for `link-replay`.

/// Failure modes for CLI parsing, tile caching, package sealing, rendering,
/// chain-trial execution, and telemetry serialization.
#[derive(Debug, thiserror::Error)]
pub enum ReplayError {
    /// Invalid command-line flags or arguments.
    #[error("{0}\n{usage}", usage = crate::cli::USAGE)]
    Usage(String),
    /// Invalid or refused `#v=2` shareable simulation link.
    #[error("link refused: {0}")]
    Link(#[from] Box<sim_link::Refusal>),
    /// Link-to-trial conversion or camera world opening failed.
    #[error(transparent)]
    Host(#[from] Box<sim_link_host::HostError>),
    /// Map package planning, fetching, or sealing failed.
    #[error("map package error: {0}")]
    Package(#[from] Box<sim_navdata::NavdataError>),
    /// Chain trial setup or execution failed.
    #[error("chain trial error: {0}")]
    Trial(#[from] Box<sim_chain_trial::ChainTrialError>),
    /// Headless GPU terrain rendering failed.
    #[error("terrain camera render error: {0}")]
    Render(#[from] Box<sim_render::RenderError>),
    /// JSON parsing or serialization failed.
    #[error("JSON error ({what}): {source}")]
    Json {
        /// Context being serialized or parsed.
        what: String,
        /// Underlying `serde_json` error.
        #[source]
        source: serde_json::Error,
    },
    /// Filesystem I/O failure on `path`.
    #[error("I/O error on {path}: {source}")]
    Io {
        /// Path that failed.
        path: String,
        /// Underlying I/O error.
        #[source]
        source: std::io::Error,
    },
    /// Requested tile was absent from the local disk cache under `--offline`.
    #[error("offline cache miss for {url} (looked up {path})")]
    OfflineMiss {
        /// Tile URL requested.
        url: String,
        /// Expected cache file path.
        path: String,
    },
}

impl From<sim_link::Refusal> for ReplayError {
    fn from(err: sim_link::Refusal) -> Self {
        Self::Link(Box::new(err))
    }
}

impl From<sim_link_host::HostError> for ReplayError {
    fn from(err: sim_link_host::HostError) -> Self {
        Self::Host(Box::new(err))
    }
}

impl From<sim_navdata::NavdataError> for ReplayError {
    fn from(err: sim_navdata::NavdataError) -> Self {
        Self::Package(Box::new(err))
    }
}

impl From<sim_chain_trial::ChainTrialError> for ReplayError {
    fn from(err: sim_chain_trial::ChainTrialError) -> Self {
        Self::Trial(Box::new(err))
    }
}

impl From<sim_render::RenderError> for ReplayError {
    fn from(err: sim_render::RenderError) -> Self {
        Self::Render(Box::new(err))
    }
}

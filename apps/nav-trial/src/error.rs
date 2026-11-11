use std::path::PathBuf;

use sim_navigate::trial::TrialError;

/// Why the trial binary stopped.
#[derive(Debug, thiserror::Error)]
pub enum NavTrialError {
    /// The command line is not one [`crate::cli`] describes.
    #[error(
        "usage: nav-trial (--request <request.json> | --route <route.json> [--defaults <file>] [--epoch <utc>] [--imu <grade>] [--guidance truth|estimate]) --out <result.json> [--progress <file|->] ({detail})"
    )]
    Usage {
        /// What was wrong.
        detail: String,
    },
    /// A file could not be read or written.
    #[error("{action} {path}")]
    Io {
        /// Reading or writing.
        action: &'static str,
        /// The file.
        path: PathBuf,
        /// The system's refusal.
        #[source]
        source: std::io::Error,
    },
    /// The trial refused the request or failed.
    #[error("trial")]
    Trial(#[from] TrialError),
}

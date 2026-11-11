//! Why a map package was not planned, fetched or sealed.

use navigate_data::DataError;
use thiserror::Error;

/// A refusal or failure of the package host.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum NavdataError {
    /// Navigate's data contract refuses a value or a store failed.
    #[error("{what}: {source}")]
    Data {
        /// What the host was doing.
        what: &'static str,
        /// The refusal.
        #[source]
        source: DataError,
    },
    /// A source could not be reached or answered with a status that is
    /// neither a tile nor "no such tile".
    #[error("fetching {url} failed: {message}")]
    Transport {
        /// The address fetched.
        url: String,
        /// The transport's or the server's message.
        message: String,
    },
    /// A tile's bytes are not the image its layer says.
    #[error("tile {tile} does not decode: {detail}")]
    Decode {
        /// The tile.
        tile: String,
        /// The decoder's message.
        detail: String,
    },
    /// The plan's upper size bound is over the storage budget. Judged
    /// before any package tile is fetched.
    #[error(
        "the map package needs up to {needed_bytes} bytes ({tiles} tiles) and the budget is \
         {budget_bytes} bytes"
    )]
    OverBudget {
        /// The plan's upper bound, bytes.
        needed_bytes: u64,
        /// The storage budget, bytes.
        budget_bytes: u64,
        /// Tiles planned.
        tiles: usize,
    },
    /// The tiles fetched reached the storage budget before the plan was
    /// complete. Nothing is sealed.
    #[error(
        "the map package reached its budget of {budget_bytes} bytes after {done} of {total} \
         tiles ({bytes} bytes); nothing was sealed"
    )]
    BudgetReached {
        /// The storage budget, bytes.
        budget_bytes: u64,
        /// Bytes of the tiles held when the budget was reached.
        bytes: u64,
        /// Tiles held.
        done: usize,
        /// Tiles planned.
        total: usize,
    },
    /// A planned tile is neither in the cache nor known absent: the package
    /// was sealed without prefetching it.
    #[error("tile {tile} was not prefetched")]
    NotPrefetched {
        /// The tile.
        tile: String,
    },
}

impl NavdataError {
    /// Wraps a refusal of the data contract met while doing `what`.
    pub(crate) fn data(what: &'static str) -> impl Fn(DataError) -> Self {
        move |source| Self::Data { what, source }
    }
}

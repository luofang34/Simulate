//! The one thing a platform supplies to reach a tile source: an HTTP GET.

use std::future::Future;

use navigate_data::{Fetched, Sha256Digest};

use crate::{NavdataError, TileSource};

/// What a server answered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HttpAnswer {
    /// The HTTP status.
    pub status: u16,
    /// The body.
    pub body: Vec<u8>,
}

/// An HTTP client. An error is a failure to get any answer; a status the
/// host does not like is still an answer.
pub trait TileTransport {
    /// Gets `url`.
    fn get(&self, url: &str) -> impl Future<Output = Result<HttpAnswer, String>>;
}

/// The tile an answer of `source` carries: status 200 is the tile unless
/// its body is one of the source's no-data bodies, 204 and 404 mean the
/// source does not have it, anything else is a failure. Navigate's package
/// tool has the same rule for statuses; it asks Esri for 404s instead of
/// the no-data picture, which a page cannot read.
///
/// # Errors
///
/// Returns [`NavdataError::Transport`] for any other status.
pub fn fetched(
    source: &TileSource,
    url: &str,
    answer: HttpAnswer,
) -> Result<Fetched, NavdataError> {
    match answer.status {
        200 if no_data(source, &answer.body) => Ok(Fetched::Absent),
        200 => Ok(Fetched::Tile(answer.body)),
        204 | 404 => Ok(Fetched::Absent),
        status => Err(NavdataError::Transport {
            url: url.to_owned(),
            message: format!("HTTP status {status}"),
        }),
    }
}

fn no_data(source: &TileSource, body: &[u8]) -> bool {
    !source.no_data_bodies.is_empty() && source.no_data_bodies.contains(&Sha256Digest::of(body))
}

//! An atmosphere's canonical content, for hashing by content.

/// An atmosphere that can state everything that shapes its samples.
///
/// A run's configuration hash covers the air about each body by this
/// content, so two atmospheres that can give different samples must give
/// different content, whatever they are called.
pub trait AtmosphereContent {
    /// Calls `f` with consecutive byte slices whose concatenation is the
    /// atmosphere's canonical content: a tag naming the kind of model, then
    /// every number that shapes a sample, in a fixed order, each `f64` as
    /// its little-endian IEEE-754 bits. How the content is cut into slices
    /// carries no meaning.
    fn visit_content(&self, f: &mut dyn FnMut(&[u8]));
}

/// Feeds the tag `kind`, length first, so no kind's content is another's
/// prefix.
pub(crate) fn feed_kind(f: &mut dyn FnMut(&[u8]), kind: &str) {
    f(&(kind.len() as u64).to_le_bytes());
    f(kind.as_bytes());
}

/// Feeds a counted run of numbers.
pub(crate) fn feed_f64s(f: &mut dyn FnMut(&[u8]), values: &[f64]) {
    f(&(values.len() as u64).to_le_bytes());
    for value in values {
        f(&value.to_bits().to_le_bytes());
    }
}

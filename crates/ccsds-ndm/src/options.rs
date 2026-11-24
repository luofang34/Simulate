//! How strictly to read, and how to write.

/// How strictly a reader holds the input to the standard.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ParseMode {
    /// Everything the standard requires: mandatory keywords present and
    /// non-empty, no undefined keywords, units exactly as defined, JSON
    /// numbers as numbers.
    #[default]
    Strict,
    /// What real producers send: mandatory header and metadata values may be
    /// absent or empty (CelesTrak leaves `CREATION_DATE` and `ORIGINATOR`
    /// blank), undefined keywords are kept as [`Extension`](crate::Extension)s,
    /// units match without regard to case, JSON numbers may arrive as strings,
    /// and a Fortran `D` exponent is read as `E`.
    ///
    /// Lenient mode never guesses a value: what is absent stays `None`.
    Lenient,
}

/// Choices a writer makes that the data does not determine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub struct WriteOptions {
    /// Restate each value's unit (`[km]` in KVN, `units="km"` in XML).
    pub units: bool,
}

impl WriteOptions {
    /// Sets whether each value's unit is restated.
    #[must_use]
    pub fn with_units(mut self, units: bool) -> Self {
        self.units = units;
        self
    }
}

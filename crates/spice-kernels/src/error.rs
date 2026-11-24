//! The crate's single typed error.

use alloc::string::String;

/// Why building, encoding, parsing or evaluating a kernel failed.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// The byte slice ends before a record or array the file refers to.
    #[error("kernel truncated: {len} bytes, need at least {needed}")]
    Truncated {
        /// Length of the byte slice.
        len: usize,
        /// Bytes the file's own structure requires.
        needed: usize,
    },
    /// The identification word does not start with `DAF/`.
    #[error("not a DAF file: identification word `{found}`")]
    NotDaf {
        /// The eight bytes found, as text.
        found: String,
    },
    /// The file is a DAF but not of the expected kernel type.
    #[error("unexpected kernel type `{found}`, expected `{expected}`")]
    WrongKernelType {
        /// Identification word the caller needs.
        expected: &'static str,
        /// Identification word found.
        found: String,
    },
    /// The binary format word names neither IEEE byte order.
    #[error("unsupported DAF binary format `{found}` (supported: LTL-IEEE, BIG-IEEE)")]
    UnsupportedBinaryFormat {
        /// The eight bytes found, as text.
        found: String,
    },
    /// The FTP validation string is present but damaged, which is how a
    /// text-mode transfer of a binary kernel shows up.
    #[error("DAF FTP validation string is damaged; the file was probably transferred in text mode")]
    FtpCorruption,
    /// `ND`/`NI` are outside what a DAF summary record can hold.
    #[error("invalid DAF summary format ND={nd}, NI={ni}")]
    InvalidSummaryFormat {
        /// Double precision components per summary.
        nd: i64,
        /// Integer components per summary.
        ni: i64,
    },
    /// The summary record chain leaves the file, loops, or holds an
    /// impossible summary count.
    #[error("corrupt DAF summary record chain at record {record}")]
    CorruptSummaryChain {
        /// One-based record number.
        record: usize,
    },
    /// An array's word address range is empty, reversed or outside the file.
    #[error("invalid DAF array address range {start}..={end} for `{name}`")]
    InvalidAddressRange {
        /// Array name.
        name: String,
        /// One-based first word address.
        start: i64,
        /// One-based last word address.
        end: i64,
    },
    /// An array does not carry the number of summary components its file needs.
    #[error(
        "array `{name}` has {doubles} double and {integers} integer summary components; the file needs {nd} and {ni}"
    )]
    SummaryShapeMismatch {
        /// Array name.
        name: String,
        /// Double components supplied.
        doubles: usize,
        /// Integer components supplied, addresses excluded.
        integers: usize,
        /// Double components the file needs.
        nd: usize,
        /// Integer components the file needs, addresses excluded.
        ni: usize,
    },
    /// An array has no data words; a DAF array holds at least one.
    #[error("array `{name}` is empty")]
    EmptyArray {
        /// Array name.
        name: String,
    },
    /// A text field is longer than its fixed slot in the file.
    #[error("{field} is {len} characters, limit {max}")]
    TextTooLong {
        /// Which field.
        field: &'static str,
        /// Characters supplied.
        len: usize,
        /// Characters the slot holds.
        max: usize,
    },
    /// A text field holds a character the format cannot store.
    #[error("{field} holds a character the kernel format cannot store: {character:?}")]
    InvalidCharacter {
        /// Which field.
        field: &'static str,
        /// The offending character.
        character: char,
    },
    /// The file would need word addresses beyond the format's 32-bit range.
    #[error("kernel too large: word address {words} exceeds the DAF 32-bit address range")]
    TooLarge {
        /// Word address that does not fit.
        words: usize,
    },
    /// A segment's stored words do not form a valid segment of its type.
    #[error("malformed SPK type {data_type} segment `{name}`: {reason}")]
    MalformedSegment {
        /// Segment name.
        name: String,
        /// SPK data type.
        data_type: i32,
        /// What is wrong.
        reason: &'static str,
    },
    /// Epoch and state counts differ.
    #[error("{epochs} epochs but {states} states")]
    LengthMismatch {
        /// Number of epochs.
        epochs: usize,
        /// Number of states.
        states: usize,
    },
    /// Fewer states than one interpolation window.
    #[error("{found} states cannot fill an interpolation window of {window}")]
    TooFewStates {
        /// Window size in states.
        window: usize,
        /// States supplied.
        found: usize,
    },
    /// The window size is outside what the segment type allows.
    #[error(
        "interpolation window of {window} states is outside {min}..={max} for SPK type {data_type}"
    )]
    InvalidWindow {
        /// SPK data type.
        data_type: i32,
        /// Window size supplied.
        window: usize,
        /// Smallest window the type allows.
        min: usize,
        /// Largest window the type allows.
        max: usize,
    },
    /// Epochs are not strictly increasing.
    #[error("epoch {epoch} at index {index} is not after its predecessor {previous}")]
    EpochsOutOfOrder {
        /// Zero-based index of the offending epoch.
        index: usize,
        /// The offending epoch.
        epoch: f64,
        /// The epoch before it.
        previous: f64,
    },
    /// An epoch or state component is NaN or infinite.
    #[error("{what} at index {index} is not finite")]
    NonFinite {
        /// Which quantity.
        what: &'static str,
        /// Zero-based index.
        index: usize,
    },
    /// The coverage interval is empty, reversed, or reaches outside the data.
    #[error(
        "segment coverage {start}..={end} is empty or outside the data interval {data_start}..={data_end}"
    )]
    InvalidCoverage {
        /// Requested coverage start.
        start: f64,
        /// Requested coverage end.
        end: f64,
        /// First epoch the data supports.
        data_start: f64,
        /// Last epoch the data supports.
        data_end: f64,
    },
    /// The epoch is outside the segment's coverage.
    #[error("epoch {et} is outside segment coverage {start}..={end}")]
    OutOfCoverage {
        /// Requested epoch.
        et: f64,
        /// Coverage start.
        start: f64,
        /// Coverage end.
        end: f64,
    },
    /// No segment gives the target relative to the centre at the epoch.
    #[error("no segment covers target {target} relative to centre {center} at epoch {et}")]
    NoCoverage {
        /// Target body code.
        target: i32,
        /// Centre body code.
        center: i32,
        /// Requested epoch.
        et: f64,
    },
    /// The segment is carried verbatim and this crate cannot evaluate its type.
    #[error("SPK type {data_type} segments are carried verbatim and cannot be evaluated")]
    UnsupportedDataType {
        /// SPK data type.
        data_type: i32,
    },
    /// Reading or writing the kernel file failed.
    #[cfg(feature = "std")]
    #[error("kernel file `{path}`: {source}")]
    Io {
        /// File path.
        path: std::path::PathBuf,
        /// Underlying I/O error.
        #[source]
        source: std::io::Error,
    },
}

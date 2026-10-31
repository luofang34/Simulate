use sim_core::EndpointError;

/// Why an external XIL wire frame or transport exchange failed.
#[derive(Debug, thiserror::Error)]
pub enum XilError {
    /// A wire buffer was shorter or longer than the protocol allows.
    #[error("invalid {protocol} packet length: expected {expected} bytes, got {actual}")]
    InvalidPacketLength {
        /// Wire protocol name.
        protocol: &'static str,
        /// Expected byte length.
        expected: usize,
        /// Actual byte length.
        actual: usize,
    },
    /// Checksum or CRC verification failed on a received frame.
    #[error("{protocol} checksum mismatch: expected {expected:#06x}, got {actual:#06x}")]
    ChecksumMismatch {
        /// Wire protocol name.
        protocol: &'static str,
        /// Computed checksum.
        expected: u16,
        /// Checksum read from the wire.
        actual: u16,
    },
    /// A simulation stimulus contained non-finite kinematics or atmosphere values.
    #[error("non-finite XIL stimulus field: {field}")]
    InvalidStimulus {
        /// Name of the offending field.
        field: &'static str,
    },
    /// A protocol field carried an unsupported version, direction, or value.
    #[error("{protocol} protocol violation: {detail}")]
    ProtocolViolation {
        /// Wire protocol name.
        protocol: &'static str,
        /// Diagnostic description.
        detail: String,
    },
    /// Underlying socket or stream I/O failure.
    #[error("XIL transport I/O failed: {0}")]
    Io(#[from] std::io::Error),
}

impl From<XilError> for EndpointError {
    fn from(error: XilError) -> Self {
        Self {
            detail: error.to_string(),
        }
    }
}

use aviate_xil_contract::AttachError;
use aviate_xil_direct::DirectSynthesisError;

/// Which model failed when initialising an [`aviate_xil_direct::DirectXilBoard`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DirectInitStage {
    /// WGS84 origin latitude/longitude/altitude validation failed.
    Origin,
    /// Geomagnetic field evaluation failed at the origin and epoch.
    EarthField,
    /// Atmosphere evaluation failed at the origin.
    Atmosphere,
}

impl From<DirectSynthesisError> for DirectInitStage {
    fn from(err: DirectSynthesisError) -> Self {
        match err {
            DirectSynthesisError::Origin(_) => Self::Origin,
            DirectSynthesisError::EarthField(_) => Self::EarthField,
            DirectSynthesisError::Atmosphere(_) => Self::Atmosphere,
        }
    }
}

/// Why the shared block or a command in it could not be trusted.
///
/// [`AttachError`] is a `no_std` type with no [`std::error::Error`] impl, so
/// it is carried as a field rather than as a source.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum AviateBridgeError {
    /// The header failed the contract's attach rules.
    #[error("shared block failed attach validation: {0:?}")]
    Attach(AttachError),
    /// The header passed, but no writer has published readiness.
    #[error("shared block has plugin_ready = 0")]
    NotReady,
    /// The model-state sequence is odd at publish time: an earlier write never
    /// finished.
    #[error("model state seqlock stuck mid-write at seq {seq}")]
    ModelStateInFlight {
        /// The sequence found.
        seq: u32,
    },
    /// The motor command stayed in flight for the whole retry budget.
    #[error("motor command seqlock gave no consistent read, seq {seq}")]
    MotorCommandTorn {
        /// The sequence found.
        seq: u32,
    },
    /// A command claimed more lanes than the block has.
    #[error("motor command claims {count} lanes")]
    MotorCountOutOfRange {
        /// The count found.
        count: u32,
    },
    /// A populated lane does not hold a finite rotor speed.
    #[error("motor lane {lane} holds non-finite bits {bits:#018x}")]
    MotorSpeedNotFinite {
        /// The lane.
        lane: usize,
        /// The bits found.
        bits: u64,
    },
    /// Lockstep is armed and the flight controller did not acknowledge the
    /// step it was handed.
    #[error("flight controller acknowledged step {acknowledged}, expected {expected}")]
    StepNotAcknowledged {
        /// The step published.
        expected: u64,
        /// The acknowledgement found.
        acknowledged: u64,
    },
    /// The master clock no longer fits the block's microsecond field.
    #[error("simulation time of {seconds} s does not fit in u64 microseconds")]
    TimeOutOfRange {
        /// Whole seconds at the failed instant.
        seconds: u64,
    },
    /// A ready flight controller drives a different number of rotors than the
    /// vehicle has.
    #[error("flight controller drives {commanded} rotors, the vehicle has {fitted}")]
    RotorCountMismatch {
        /// Populated lanes.
        commanded: usize,
        /// Rotors on the vehicle.
        fitted: usize,
    },
    /// Initialising the in-process [`aviate_xil_direct::DirectXilBoard`] failed.
    #[error("direct Aviate board initialisation failed in {stage:?}")]
    DirectInit {
        /// Failing environment model stage.
        stage: DirectInitStage,
    },
    /// A stimulus contained non-finite kinematics before reaching `aviate-xil-direct`.
    #[error("direct Aviate stimulus field {field} is not finite")]
    DirectNonFinite {
        /// Name of the non-finite field.
        field: &'static str,
    },
}

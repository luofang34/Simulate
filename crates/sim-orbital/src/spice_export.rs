//! Adapters exporting Simulate trajectories (`OrbitalTrajectoryProduct`,
//! `TranslationalTrajectorySource`, `AttitudeTrajectorySource`, and sampled states)
//! into deterministic binary NAIF `DAF/SPK` (Types 2, 3, 9, 13) and `DAF/CK`
//! (Types 2, 3) kernels via [`spice_kernels`].
//!
//! # Unit and Frame Conventions
//!
//! - **SPK**: Simulate translational states (`m`, `m/s` in [`FrameAxesSpec::IcrfInertial`])
//!   are scaled by `0.001` into SPICE native `km` and `km/s` in `FRAME_J2000` (`1`).
//! - **CK**: Simulate `q_body_to_icrf` rotates body coordinates into ICRF, whereas SPICE CK
//!   stores the scalar-first C-matrix quaternion `[w, x, y, z]` representing `R_{ref -> inst}`
//!   (`v_inst = Q2M(q) * v_ref`), which is `q_body_to_icrf.conjugate()`. Angular velocity
//!   `omega_body_rad_s` is rotated from body into `J2000 / ICRF` reference axes.

mod builders;

use glam::{DQuat, DVec3};
use sim_spacetime::{
    AttitudeSample, AttitudeTrajectorySource, FrameAxesSpec, NaifId, TdbEpoch,
    TrajectoryQueryError, TranslationalTrajectorySource,
};
use spice_kernels::{CkFile, CkPointing, CkPointingInstance, SpkFile};

use crate::{KinematicStateIcrf, OrbitalTrajectoryProduct};
use builders::{
    attitude_to_ck_instance, build_ck_file, build_spk_from_samples, build_spk_from_source,
    ck_pointing_to_attitude, kinematic_to_spice, spice_to_kinematic, uniform_epoch_grid,
};

const M_TO_KM: f64 = 0.001;
const KM_TO_M: f64 = 1_000.0;
const NANOS_PER_SECOND_F64: f64 = 1_000_000_000.0;
const NANOS_PER_SECOND_I128: i128 = 1_000_000_000;
const MAX_GRID_RECORDS: f64 = 1_000_000.0;

/// Why exporting a Simulate trajectory into a NAIF SPK or CK kernel failed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum SpiceExportError {
    /// Querying the underlying trajectory source failed.
    #[error("trajectory query failed during SPICE export: {0}")]
    TrajectoryQuery(#[from] TrajectoryQueryError),
    /// Constructing or validating the SPICE kernel failed.
    #[error("SPICE kernel construction failed: {0}")]
    Kernel(String),
    /// Source trajectory covered zero or negative duration.
    #[error("trajectory covers zero or negative duration")]
    EmptySpan,
    /// Trajectory coordinates were not expressed in `ICRF / J2000` inertial axes.
    #[error("SPICE export requires ICRF inertial axes, found {found:?}")]
    NonIcrfAxes {
        /// Coordinate axes carried by the queried trajectory state.
        found: FrameAxesSpec,
    },
    /// Invalid SPK or CK export parameters (step size, degree, window, or sample count).
    #[error("invalid SPICE export configuration: {0}")]
    InvalidConfiguration(&'static str),
    /// Invalid linear SCLK mapping parameters.
    #[error("invalid linear SCLK mapping: {0}")]
    InvalidSclkMapping(&'static str),
    /// Quaternion norm was zero or non-finite.
    #[error("attitude quaternion has zero or non-finite norm")]
    InvalidUnitQuaternion,
}

impl From<spice_kernels::Error> for SpiceExportError {
    fn from(err: spice_kernels::Error) -> Self {
        Self::Kernel(err.to_string())
    }
}

/// SPK representation and sampling parameters for trajectory export.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SpkExportKind {
    /// SPK Type 2 (equal-interval Chebyshev polynomials of position only; velocity by derivative).
    Type2Chebyshev {
        /// Nominal granule length in TDB seconds (`> 0`).
        granule_duration_s: f64,
        /// Chebyshev polynomial degree per component.
        degree: usize,
    },
    /// SPK Type 3 (equal-interval Chebyshev polynomials of position and velocity).
    Type3Chebyshev {
        /// Nominal granule length in TDB seconds (`> 0`).
        granule_duration_s: f64,
        /// Chebyshev polynomial degree per component.
        degree: usize,
    },
    /// SPK Type 9 (discrete states interpolated by Lagrange polynomials).
    Type9Lagrange {
        /// Nominal sample spacing in TDB seconds (`>= 1e-6`).
        step_s: f64,
        /// Lagrange interpolation window width (`>= 2`).
        window: usize,
    },
    /// SPK Type 13 (discrete states interpolated by Hermite polynomials).
    Type13Hermite {
        /// Nominal sample spacing in TDB seconds (`>= 1e-6`).
        step_s: f64,
        /// Hermite window width in states (`>= 2`, even).
        window: usize,
    },
}

/// Metadata and representation settings for exporting a translational trajectory to `DAF/SPK`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpkExportConfig<'a> {
    /// Internal DAF file name (`<= 60` printable ASCII characters).
    pub internal_name: &'a str,
    /// Segment descriptor name (`<= 40` printable ASCII characters).
    pub segment_name: &'a str,
    /// NAIF target spacecraft or body identifier.
    pub target: NaifId,
    /// NAIF center-of-motion body or barycenter identifier.
    pub center: NaifId,
    /// Target SPK data type and discretization settings.
    pub kind: SpkExportKind,
}

impl<'a> SpkExportConfig<'a> {
    /// Creates an SPK export configuration for `target` relative to `center` in `FRAME_J2000`.
    #[must_use]
    pub const fn new(
        internal_name: &'a str,
        segment_name: &'a str,
        target: NaifId,
        center: NaifId,
        kind: SpkExportKind,
    ) -> Self {
        Self {
            internal_name,
            segment_name,
            target,
            center,
            kind,
        }
    }

    /// Exports any [`TranslationalTrajectorySource`] using this configuration.
    pub fn export_translational(
        &self,
        source: &dyn TranslationalTrajectorySource,
    ) -> Result<SpkFile, SpiceExportError> {
        export_translational_spk(source, self)
    }

    /// Exports a propagated [`OrbitalTrajectoryProduct`] using this configuration.
    pub fn export_orbital_product(
        &self,
        product: &OrbitalTrajectoryProduct,
    ) -> Result<SpkFile, SpiceExportError> {
        export_translational_spk(product, self)
    }

    /// Exports discrete `(TdbEpoch, KinematicStateIcrf)` samples (`m`, `m/s`) using this configuration.
    pub fn export_sampled_states(
        &self,
        samples: &[(TdbEpoch, KinematicStateIcrf)],
    ) -> Result<SpkFile, SpiceExportError> {
        export_sampled_states_spk(samples, self)
    }

    /// Converts a Simulate `ICRF` kinematic state (`m`, `m/s`) to SPICE `[x, y, z, vx, vy, vz]` (`km`, `km/s`).
    #[must_use]
    pub fn kinematic_to_spice(state: &KinematicStateIcrf) -> [f64; 6] {
        kinematic_to_spice(state)
    }

    /// Converts a SPICE state `[x, y, z, vx, vy, vz]` (`km`, `km/s`) into Simulate `ICRF` (`m`, `m/s`, zero acceleration).
    #[must_use]
    pub fn spice_to_kinematic(state: &[f64; 6]) -> KinematicStateIcrf {
        spice_to_kinematic(state)
    }
}

/// Linear correspondence between `TdbEpoch` and encoded SPICE `SCLK` ticks (`sclkdp`).
///
/// Defined by `sclk(t) = reference_sclk + (t - reference_epoch) * ticks_per_second`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearSclkMapping {
    /// Reference TDB epoch where encoded clock reads `reference_sclk`.
    pub reference_epoch: TdbEpoch,
    /// Encoded SCLK value at `reference_epoch`.
    pub reference_sclk: f64,
    /// Positive monotonic clock tick rate in encoded ticks per TDB second.
    pub ticks_per_second: f64,
}

impl LinearSclkMapping {
    /// Builds a custom linear SCLK mapping.
    #[must_use]
    pub const fn new(
        reference_epoch: TdbEpoch,
        reference_sclk: f64,
        ticks_per_second: f64,
    ) -> Self {
        Self {
            reference_epoch,
            reference_sclk,
            ticks_per_second,
        }
    }

    /// Maps `TdbEpoch` linearly to seconds (`ticks_per_second = 1.0`) elapsed since `origin`.
    #[must_use]
    pub const fn seconds_since(origin: TdbEpoch) -> Self {
        Self {
            reference_epoch: origin,
            reference_sclk: 0.0,
            ticks_per_second: 1.0,
        }
    }

    /// Validates finiteness and positive rate.
    pub fn validate(&self) -> Result<(), SpiceExportError> {
        if !self.reference_sclk.is_finite() {
            return Err(SpiceExportError::InvalidSclkMapping(
                "reference_sclk must be finite",
            ));
        }
        if !self.ticks_per_second.is_finite() || self.ticks_per_second <= 0.0 {
            return Err(SpiceExportError::InvalidSclkMapping(
                "ticks_per_second must be finite and positive",
            ));
        }
        Ok(())
    }

    /// Converts a `TdbEpoch` into encoded SCLK ticks (`sclkdp`).
    pub fn epoch_to_sclk(&self, epoch: TdbEpoch) -> Result<f64, SpiceExportError> {
        self.validate()?;
        let sclk = self.reference_sclk
            + epoch.signed_offset_seconds_from(self.reference_epoch) * self.ticks_per_second;
        if !sclk.is_finite() {
            return Err(SpiceExportError::InvalidSclkMapping(
                "mapped SCLK overflowed f64",
            ));
        }
        Ok(sclk)
    }

    /// Returns seconds per encoded SCLK tick (`1.0 / ticks_per_second`).
    pub fn seconds_per_tick(&self) -> Result<f64, SpiceExportError> {
        self.validate()?;
        let spt = 1.0 / self.ticks_per_second;
        if !spt.is_finite() || spt <= 0.0 {
            return Err(SpiceExportError::InvalidSclkMapping(
                "seconds_per_tick underflowed",
            ));
        }
        Ok(spt)
    }

    /// Converts an encoded SCLK timestamp back to `TdbEpoch` at nanosecond resolution.
    #[must_use]
    pub fn sclk_to_epoch(&self, sclk: f64) -> Option<TdbEpoch> {
        if self.validate().is_err() || !sclk.is_finite() {
            return None;
        }
        let dt_s = (sclk - self.reference_sclk) / self.ticks_per_second;
        if !dt_s.is_finite() {
            return None;
        }
        let whole_s = libm::trunc(dt_s);
        let frac_ns = libm::round((dt_s - whole_s) * NANOS_PER_SECOND_F64) as i128;
        let whole_ns = (whole_s as i128).checked_mul(NANOS_PER_SECOND_I128)?;
        let total_ns = self
            .reference_epoch
            .to_j2000_nanos_floor()
            .checked_add(whole_ns)?
            .checked_add(frac_ns)?;
        Some(TdbEpoch::from_j2000_nanos(total_ns))
    }

    /// Converts a Simulate attitude `(q_body_to_icrf, omega_body_rad_s)` at `epoch` into a CK pointing instance.
    pub fn attitude_to_ck(
        &self,
        epoch: TdbEpoch,
        q_body_to_icrf: DQuat,
        omega_body_rad_s: Option<DVec3>,
    ) -> Result<CkPointingInstance, SpiceExportError> {
        attitude_to_ck_instance(epoch, q_body_to_icrf, omega_body_rad_s, self)
    }

    /// Converts an evaluated SPICE [`CkPointing`] (`R_{J2000 -> inst}`) back into a Simulate [`AttitudeSample`].
    #[must_use]
    pub fn ck_to_attitude(&self, pointing: CkPointing) -> Option<AttitudeSample> {
        let epoch = self.sclk_to_epoch(pointing.sclk)?;
        Some(ck_pointing_to_attitude(epoch, pointing))
    }
}

/// CK representation and sampling parameters for attitude export.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CkExportKind {
    /// CK Type 2: piecewise-constant angular velocity intervals sampled uniformly at `step_s`.
    Type2ConstantRate {
        /// Sampling interval step size in seconds (`>= 1e-6`).
        step_s: f64,
    },
    /// CK Type 3: discrete pointing instances with SLERP orientation and optional linear AV.
    Type3Slerp {
        /// Sampling step size in seconds (`>= 1e-6`).
        step_s: f64,
        /// Record angular velocity vectors (`avflag = 1`) when `true`.
        include_angular_velocity: bool,
    },
}

/// Metadata and SCLK settings for exporting an attitude trajectory to `DAF/CK`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CkExportConfig<'a> {
    /// Internal DAF file name (`<= 60` printable ASCII characters).
    pub internal_name: &'a str,
    /// Segment descriptor name (`<= 40` printable ASCII characters).
    pub segment_name: &'a str,
    /// NAIF instrument or spacecraft body frame identifier (`inst`).
    pub instrument_id: i32,
    /// Target CK data type and step sizing.
    pub kind: CkExportKind,
    /// Linear TDB-to-SCLK clock conversion.
    pub sclk: LinearSclkMapping,
}

impl<'a> CkExportConfig<'a> {
    /// Creates a CK export configuration in base frame `FRAME_J2000`.
    #[must_use]
    pub const fn new(
        internal_name: &'a str,
        segment_name: &'a str,
        instrument_id: i32,
        kind: CkExportKind,
        sclk: LinearSclkMapping,
    ) -> Self {
        Self {
            internal_name,
            segment_name,
            instrument_id,
            kind,
            sclk,
        }
    }

    /// Exports any [`AttitudeTrajectorySource`] using this configuration.
    pub fn export_attitude(
        &self,
        source: &dyn AttitudeTrajectorySource,
    ) -> Result<CkFile, SpiceExportError> {
        export_attitude_ck(source, self)
    }

    /// Exports a propagated [`OrbitalTrajectoryProduct`] attitude trajectory using this configuration.
    pub fn export_orbital_product(
        &self,
        product: &OrbitalTrajectoryProduct,
    ) -> Result<CkFile, SpiceExportError> {
        export_attitude_ck(product, self)
    }

    /// Exports discrete `(TdbEpoch, q_body_to_icrf, omega_body_rad_s)` samples using this configuration.
    pub fn export_sampled_attitude(
        &self,
        samples: &[(TdbEpoch, DQuat, DVec3)],
    ) -> Result<CkFile, SpiceExportError> {
        export_sampled_attitude_ck(samples, self)
    }

    /// Converts an evaluated SPICE [`CkPointing`] (`R_{J2000 -> inst}`) at `epoch` back into a Simulate [`AttitudeSample`].
    #[must_use]
    pub fn ck_to_sim_attitude(epoch: TdbEpoch, pointing: CkPointing) -> AttitudeSample {
        ck_pointing_to_attitude(epoch, pointing)
    }
}

/// Exports any [`TranslationalTrajectorySource`] into an in-memory [`SpkFile`].
pub fn export_translational_spk(
    source: &dyn TranslationalTrajectorySource,
    config: &SpkExportConfig<'_>,
) -> Result<SpkFile, SpiceExportError> {
    build_spk_from_source(source, config)
}

/// Exports discrete `(TdbEpoch, KinematicStateIcrf)` samples (`m`, `m/s`) into an [`SpkFile`].
pub fn export_sampled_states_spk(
    samples: &[(TdbEpoch, KinematicStateIcrf)],
    config: &SpkExportConfig<'_>,
) -> Result<SpkFile, SpiceExportError> {
    build_spk_from_samples(samples, config)
}

/// Exports any [`AttitudeTrajectorySource`] into an in-memory [`CkFile`].
pub fn export_attitude_ck(
    source: &dyn AttitudeTrajectorySource,
    config: &CkExportConfig<'_>,
) -> Result<CkFile, SpiceExportError> {
    config.sclk.validate()?;
    let valid = source.attitude_valid_interval();
    let step_s = match config.kind {
        CkExportKind::Type2ConstantRate { step_s } | CkExportKind::Type3Slerp { step_s, .. } => {
            step_s
        }
    };
    let epochs = uniform_epoch_grid(valid.start_epoch, valid.end_epoch, step_s)?;
    let include_av = match config.kind {
        CkExportKind::Type2ConstantRate { .. } => true,
        CkExportKind::Type3Slerp {
            include_angular_velocity,
            ..
        } => include_angular_velocity,
    };
    let mut instances = Vec::with_capacity(epochs.len());
    for epoch in epochs {
        let sample = source.sample_attitude_icrf(epoch)?;
        let omega = include_av.then_some(sample.omega_body_rad_s);
        instances.push(attitude_to_ck_instance(
            sample.epoch,
            sample.q_body_to_icrf,
            omega,
            &config.sclk,
        )?);
    }
    build_ck_file(&instances, config)
}

/// Exports discrete `(TdbEpoch, q_body_to_icrf, omega_body_rad_s)` samples into a [`CkFile`].
pub fn export_sampled_attitude_ck(
    samples: &[(TdbEpoch, DQuat, DVec3)],
    config: &CkExportConfig<'_>,
) -> Result<CkFile, SpiceExportError> {
    config.sclk.validate()?;
    let include_av = match config.kind {
        CkExportKind::Type2ConstantRate { .. } => true,
        CkExportKind::Type3Slerp {
            include_angular_velocity,
            ..
        } => include_angular_velocity,
    };
    let mut instances = Vec::with_capacity(samples.len());
    for &(epoch, q_body_to_icrf, omega_body_rad_s) in samples {
        let omega = include_av.then_some(omega_body_rad_s);
        instances.push(attitude_to_ck_instance(
            epoch,
            q_body_to_icrf,
            omega,
            &config.sclk,
        )?);
    }
    build_ck_file(&instances, config)
}

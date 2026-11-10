//! Evaluated outcome payload, binding verification errors, and sealed [`AnalysisResultArtifact`].

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    ContentHash, DomainHasher,
    artifact::{
        ANALYSIS_CONTRACT_SCHEMA_VERSION, AnalysisJobStatus, AnalysisRequestArtifact,
        AnalysisWorkloadSpec, CoordinateFrameContract, DesignRevisionIdentity,
        NumericalAccuracyContract, ReproducibilityStamp, TimeScaleContract,
    },
    relay::{EndpointLinkReport, GeometricRelayReport, VerificationScope},
    taxonomy::AccessAnalysisError,
};

/// Evaluated payload inside an [`AnalysisResultArtifact`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum AnalysisOutcomePayload {
    /// Two-endpoint geometric link report.
    EndpointPairReport(EndpointLinkReport),
    /// Aircraft – Satellite – Ground Station geometric relay report.
    GeometricRelayReport(GeometricRelayReport),
    /// Structured failure when evaluation fails closed (`DataCoverage` or `ModelNotApplicable`).
    EvaluationFailed(AccessAnalysisError),
}

/// Reason why verifying an [`AnalysisResultArtifact`] against a registered [`AnalysisRequestArtifact`] failed.
#[derive(Clone, Debug, PartialEq, Eq, Error)]
pub enum ArtifactBindingError {
    /// Schema version mismatch.
    #[error("unsupported schema_version {0}")]
    UnsupportedSchemaVersion(u32),
    /// Result `result_digest` does not match `compute_expected_digest()`.
    #[error("result_digest mismatch: artifact was modified after sealing")]
    ResultDigestMismatch,
    /// Result `request_id` does not match the registered request's `request_id`.
    #[error("request_id mismatch: expected {expected}, got {actual}")]
    RequestIdMismatch {
        /// Registered request ID.
        expected: String,
        /// Result's request ID.
        actual: String,
    },
    /// Result `request_digest` does not match `registered_request.content_hash()`.
    #[error(
        "request_digest mismatch: result was computed from a modified or different request payload"
    )]
    RequestDigestMismatch,
    /// Result `bound_design_revision` does not match the registered request's `design_revision`.
    #[error("bound_design_revision mismatch")]
    RevisionBindingMismatch,
    /// Result `execution_candidate_digest` does not match the registered request's `execution_candidate_digest`.
    #[error("execution_candidate_digest mismatch")]
    ExecutionCandidateDigestMismatch,
    /// Result time-scale, coordinate-frame, or numerical-accuracy contract does not match the registered request.
    #[error(
        "time_scale, coordinate_frames, or numerical_accuracy contract mismatch with registered request"
    )]
    ContractMetadataMismatch,
    /// Result `job_status` or `outcome` variant is inconsistent with the request workload.
    #[error("outcome variant or job_status is inconsistent with the registered request workload")]
    OutcomeWorkloadMismatch,
}

/// Versioned external analysis result artifact returned by `Simulate` and ingested by `Sokoly`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AnalysisResultArtifact {
    /// Contract schema version ([`ANALYSIS_CONTRACT_SCHEMA_VERSION`]).
    pub schema_version: u32,
    /// Unique result artifact identifier.
    pub result_id: String,
    /// Request identifier that produced this result.
    pub request_id: String,
    /// Design revision identity to which this result is cryptographically bound.
    pub bound_design_revision: DesignRevisionIdentity,
    /// Optional execution candidate digest echoed from the request.
    #[serde(default)]
    pub execution_candidate_digest: Option<ContentHash>,
    /// SHA-256 digest of the evaluated [`AnalysisRequestArtifact`].
    pub request_digest: ContentHash,
    /// Terminal job status (`Completed` or `Failed`).
    pub job_status: AnalysisJobStatus,
    /// Explicit verification scope (`rf_link_budget_verified = false`, `store_and_forward_modeled = false`).
    pub verification_scope: VerificationScope,
    /// Time-scale contract verified and echoed from the request.
    pub time_scale: TimeScaleContract,
    /// Coordinate-frame contract verified and echoed from the request.
    pub coordinate_frames: CoordinateFrameContract,
    /// Numerical accuracy contract including evaluator-populated `proven_*` bounds.
    pub numerical_accuracy: NumericalAccuracyContract,
    /// Cryptographic reproducibility stamp recording evaluator version and catalog hashes.
    pub reproducibility: ReproducibilityStamp,
    /// Evaluated report or structured fail-closed error.
    pub outcome: AnalysisOutcomePayload,
    /// SHA-256 digest sealing EVERY field of this result artifact.
    pub result_digest: ContentHash,
}

impl AnalysisResultArtifact {
    /// Computes the canonical SHA-256 digest sealing ALL fields of this result artifact (except `result_digest`).
    #[must_use]
    pub fn compute_expected_digest(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-analysis:result-artifact:v2");
        h.write_u32(self.schema_version);
        h.write_bytes(self.result_id.as_bytes());
        h.write_bytes(self.request_id.as_bytes());
        self.bound_design_revision.write_to_hasher(&mut h);
        match self.execution_candidate_digest {
            None => h.write_u8(0),
            Some(d) => {
                h.write_u8(1);
                h.write_hash(d);
            }
        }
        h.write_hash(self.request_digest);
        self.job_status.write_to_hasher(&mut h);
        h.write_u8(u8::from(
            self.verification_scope.geometric_los_and_fov_verified,
        ));
        h.write_u8(u8::from(self.verification_scope.rf_link_budget_verified));
        h.write_u8(u8::from(self.verification_scope.store_and_forward_modeled));
        self.time_scale.write_to_hasher(&mut h);
        self.coordinate_frames.write_to_hasher(&mut h);
        self.numerical_accuracy.write_to_hasher(&mut h);
        self.reproducibility.write_to_hasher(&mut h);
        if let Ok(outcome_bytes) = serde_json::to_vec(&self.outcome) {
            h.write_bytes(&outcome_bytes);
        }
        h.finish()
    }

    /// Verifies that this result artifact's cryptographic digest is intact and matches `expected_revision`.
    #[must_use]
    pub fn verify_revision_binding(&self, expected_revision: &DesignRevisionIdentity) -> bool {
        self.schema_version == ANALYSIS_CONTRACT_SCHEMA_VERSION
            && &self.bound_design_revision == expected_revision
            && self.compute_expected_digest() == self.result_digest
    }

    /// Strictly verifies this result artifact against the immutable registered [`AnalysisRequestArtifact`]
    /// that initiated the evaluation.
    pub fn verify_against_request(
        &self,
        expected_request: &AnalysisRequestArtifact,
    ) -> Result<(), ArtifactBindingError> {
        if self.schema_version != ANALYSIS_CONTRACT_SCHEMA_VERSION {
            return Err(ArtifactBindingError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        if self.compute_expected_digest() != self.result_digest {
            return Err(ArtifactBindingError::ResultDigestMismatch);
        }
        if self.request_id != expected_request.request_id {
            return Err(ArtifactBindingError::RequestIdMismatch {
                expected: expected_request.request_id.clone(),
                actual: self.request_id.clone(),
            });
        }
        if self.request_digest != expected_request.content_hash() {
            return Err(ArtifactBindingError::RequestDigestMismatch);
        }
        if self.bound_design_revision != expected_request.design_revision {
            return Err(ArtifactBindingError::RevisionBindingMismatch);
        }
        if self.execution_candidate_digest != expected_request.execution_candidate_digest {
            return Err(ArtifactBindingError::ExecutionCandidateDigestMismatch);
        }
        if self.time_scale != expected_request.time_scale
            || self.coordinate_frames != expected_request.coordinate_frames
            || !self
                .numerical_accuracy
                .matches_request(&expected_request.numerical_accuracy)
        {
            return Err(ArtifactBindingError::ContractMetadataMismatch);
        }
        let outcome_ok = matches!(
            (&expected_request.workload, &self.job_status, &self.outcome),
            (
                AnalysisWorkloadSpec::EndpointPairLos { .. },
                AnalysisJobStatus::Completed,
                AnalysisOutcomePayload::EndpointPairReport(_),
            ) | (
                AnalysisWorkloadSpec::AircraftSatelliteGroundRelay { .. },
                AnalysisJobStatus::Completed,
                AnalysisOutcomePayload::GeometricRelayReport(_),
            ) | (
                _,
                AnalysisJobStatus::Failed { .. },
                AnalysisOutcomePayload::EvaluationFailed(_),
            )
        );
        if !outcome_ok {
            return Err(ArtifactBindingError::OutcomeWorkloadMismatch);
        }
        Ok(())
    }
}

//! Reusable, model-agnostic closed-loop POH and empirical flight-test
//! calibration & Jacobian sensitivity identification engine for packaged aircraft.
//!
//! Any [`LoadedSyspack`] declaring [`CALIBRATION_SUITE_ASSET`] (`performance/poh_benchmarks.json`)
//! — and optionally [`crate::PITOT_STATIC_CALIBRATION_ASSET`] (`performance/pitot_static_calibration.json`)
//! — can be evaluated headlessly via [`evaluate_package`] or identified via
//! [`identify_package_sensitivities`].

mod ace_bridge;
mod climb_time;
mod cruise;
mod design;
mod engines;
mod markdown;
mod mission;
mod mixture;
mod results;
mod schema;
mod sensitivity;
mod solver;
mod takeoff;
mod trades;

pub use ace_bridge::{
    AceFeedbackReport, build_ace_feedback_report, load_syspack_from_ace_vehicle_json,
    write_syspack_with_manual_assets,
};
pub use climb_time::{BrakeReleaseStart, ClimbTimeBenchmark};
pub use design::estimate_design_performance;
pub use mission::{default_manual_mission_profiles, evaluate_mission_profile};
pub use results::{
    CalibrationReport, ClimbResult, ClimbTimeResult, CruiseResult, DesignClimbPoint,
    DesignCruisePoint, DesignPerformanceReport, EmpiricalSegmentResult, MissionProfileResult,
    MissionSegmentResult, ParameterAdjustment, PayloadRangePoint, SensitivityReport, StallResult,
    TakeoffResult,
};
pub use schema::{
    CalibrationSuite, CalibrationTolerances, ClimbBenchmark, CruiseBenchmark,
    EmpiricalFlightSegment, EmpiricalSegmentMode, HandbookProperty, MissionProfileSpec,
    MissionSegmentKind, MissionSegmentSpec, StallBenchmark, StaticRunupBenchmark, TailProfile,
    TakeoffBenchmark,
};
pub use trades::{
    DesignTradeCandidateResult, DesignTradeSpec, DesignTradeStudyReport, apply_design_trade,
    default_design_trades, run_design_trade_study,
};

use crate::{LoadedSyspack, PackError};
use solver::{ParameterOverrides, run_suite_with_overrides};

/// Canonical asset path inside a fixed-wing system package declaring its
/// POH and empirical flight-test calibration suite.
pub const CALIBRATION_SUITE_ASSET: &str = "performance/poh_benchmarks.json";

/// Runs the package's embedded [`CalibrationSuite`] in closed-loop 6-DoF JSBSim
/// simulation and returns a structured [`CalibrationReport`].
pub fn evaluate_package(package: &LoadedSyspack) -> Result<CalibrationReport, PackError> {
    let suite = package
        .calibration_suite()
        .ok_or_else(|| PackError::InvalidCalibration {
            path: CALIBRATION_SUITE_ASSET.to_owned(),
            detail: "package does not ship performance/poh_benchmarks.json".to_owned(),
        })?;
    evaluate_package_against(package, suite)
}

/// Evaluates `package` against an explicit [`CalibrationSuite`] (enabling external
/// benchmark files or iterative calibration sweeps).
pub fn evaluate_package_against(
    package: &LoadedSyspack,
    suite: &CalibrationSuite,
) -> Result<CalibrationReport, PackError> {
    suite.validate()?;
    let raw = run_suite_with_overrides(package, suite, ParameterOverrides::default())?;
    Ok(results::build_report(suite, raw))
}

/// Evaluates `package` alongside finite-difference Jacobian sensitivities and
/// damped Gauss-Newton parameter adjustment recommendations.
pub fn identify_package_sensitivities(
    package: &LoadedSyspack,
) -> Result<SensitivityReport, PackError> {
    let suite = package
        .calibration_suite()
        .ok_or_else(|| PackError::InvalidCalibration {
            path: CALIBRATION_SUITE_ASSET.to_owned(),
            detail: "package does not ship performance/poh_benchmarks.json".to_owned(),
        })?;
    suite.validate()?;
    sensitivity::compute_sensitivities(package, suite)
}

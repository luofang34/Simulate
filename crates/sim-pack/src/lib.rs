//! The simulator's side of system packages. A built-in vehicle and a
//! downloaded one are the same thing: a `syspack.json` manifest plus
//! SHA-256-verified asset files, verified by [`syspack`] and bound here by
//! [`LoadedSyspack`] to the blocks that fly them.
//!
//! Loading fails closed: anything [`syspack`] refuses, or a device that cannot
//! be built, is a typed [`PackError`], and nothing of the package is usable.

mod builtin;
pub mod calibration;
mod envelope;
mod error;
mod instruments;
mod landmarks;
mod livery;
mod loaded;
mod pec;

pub use builtin::{
    builtin_c172_package, builtin_c172_source, builtin_p38g_package, builtin_p38g_source,
};
pub use calibration::{
    AceFeedbackReport, CALIBRATION_SUITE_ASSET, CalibrationReport, CalibrationSuite,
    CalibrationTolerances, ClimbBenchmark, CruiseBenchmark, DesignClimbPoint, DesignCruisePoint,
    DesignPerformanceReport, DesignTradeCandidateResult, DesignTradeSpec, DesignTradeStudyReport,
    EmpiricalFlightSegment, EmpiricalSegmentMode, HandbookProperty, MissionProfileResult,
    MissionProfileSpec, MissionSegmentKind, MissionSegmentResult, MissionSegmentSpec,
    PayloadRangePoint, SensitivityReport, StallBenchmark, StaticRunupBenchmark, TailProfile,
    TakeoffBenchmark, TakeoffResult, apply_design_trade, build_ace_feedback_report,
    default_design_trades, default_manual_mission_profiles, estimate_design_performance,
    evaluate_mission_profile, evaluate_package, evaluate_package_against,
    identify_package_sensitivities, load_syspack_from_ace_vehicle_json, run_design_trade_study,
    write_syspack_with_manual_assets,
};
pub use envelope::{AIRSPEED_ENVELOPE_ASSET, AirspeedEnvelope, ResolvedAirspeedEnvelope};
pub use error::PackError;
pub use instruments::{BUILTIN_INSTRUMENT_PACKS, builtin_instrument_pack_source};
pub use landmarks::{
    HeightReference, MeshFeature, STRUCTURAL_LANDMARKS_ASSET, StructuralLandmark,
    StructuralLandmarks,
};
pub use livery::{LIVERY_CATALOG_ASSET, LiveryCatalog, VehicleLivery};
pub use loaded::{CotsDevice, LoadedSyspack, MountedCotsDevice, PackageCamera};
pub use pec::{
    AirspeedCalibrationPoint, PITOT_STATIC_CALIBRATION_ASSET, PitotStaticCalibration,
    PitotStaticCurve,
};
#[cfg(not(target_arch = "wasm32"))]
pub use syspack::DirectoryPackageSource;
pub use syspack::{
    AssetDigest, ComponentRecord, ControlsSpec, DeviceKindSpec, DynamicsSpec,
    INITIAL_COMPONENT_TEMPERATURE_K, MANIFEST_FILE, MemoryPackageSource, MountedDeviceSpec,
    PackageSource, SCHEMA_VERSION, SyspackError, SyspackManifest, SystemKind, VerifiedSyspack,
};

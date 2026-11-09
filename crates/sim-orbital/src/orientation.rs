//! Tiered planetary orientation models (`IAU` text PCK vs. binary lunar PA/ME and Earth `ITRF93` PCKs).
//!
//! Explicitly tags every evaluated rotation with [`SurfaceNavGrade`] so approximate
//! IAU trigonometric models (`pck00011.tpc`) are never silently treated as geodetic-grade
//! Earth `ITRF93` or Lunar `DE440 PA / ME` frames.

mod bpc_frames;
mod iau_pck;

pub use bpc_frames::{LunarTargetFrame, StackedOrientationCatalog};
pub use iau_pck::IauPckCatalog;
pub use sim_spacetime::{
    BodyOrientationCatalog, BodyRotationState, OrientationError, OrientationPolicy,
    SurfaceNavGrade, coverage_for_grade, default_iau_frame_code,
};

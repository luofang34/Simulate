//! Bridge from Simulate's master clock and visual harness into Navigate's real
//! visual-fix admission and navigation filter.
//!
//! [`MasterClockBridge`] maps [`sim_core::SimInstant`] onto Navigate's monotonic
//! stamps. [`VisualFixAdapter`] turns harness fixes into
//! [`navigate_visual::Estimate`]s, labels each anchor in a real
//! [`navigate_visual_session::VisualSession`], takes the camera lever arm off,
//! and admits only first-in-cell anchors through
//! [`navigate_visual_fusion::VisualFixSource`]. [`NavigationLoop`] replays those
//! fixes and [`KinematicAiding`] velocities in acquisition order into
//! [`navigate_fusion::NavigationFilter`], and [`compare_with_truth`] scores each
//! solution against simulation truth.
//!
//! The estimates the corridor scenarios supply are truth plus declared noise,
//! so a run proves admission, timing, and covariance discipline, not image
//! matching. Navigate's filter publishes no protection levels yet; nothing here
//! invents them.

mod aiding;
mod clock;
mod error;
mod estimate;
mod filter_loop;
mod truth;
mod visual_fix;

pub use aiding::KinematicAiding;
pub use clock::{MASTER_CLOCK_DOMAIN, MasterClockBridge};
pub use error::NavigateBridgeError;
pub use estimate::CorridorFrame;
pub use filter_loop::{NavigationLoop, airborne_fusion_config};
pub use truth::{TruthComparison, compare_with_truth, solution_position_ecef};
pub use visual_fix::{FixDisposition, VisualFixAdapter, VisualFixConfig};

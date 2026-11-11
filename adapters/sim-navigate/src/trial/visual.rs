//! Visual terrain navigation in the trial: the simulated vehicle camera
//! renders the world imagery at the true pose ([`LiveCamera`]); Navigate
//! renders a reference from the onboard package at the navigator's estimated
//! pose, matches, and converts the pose into an observation ([`VisualAid`]).

mod aid;
mod camera;
mod matching;
mod report;
mod setup;
mod spec;

pub use aid::{FixRecord, FrameTiming, VISUAL_SOURCE, VisualAid, VisualTally};
pub use camera::{CameraFrame, CameraFrameTruth, LiveCamera};
pub use report::{MapIdentity, ScoredFix, VisualMapsReport, VisualReport, visual_report};
pub use setup::{
    load_onboard_package, maps_report, mean_ground_distance_m, package_specs, visual_aid_for,
};
pub use spec::{
    CameraSpec, MapsSpec, PriorSpec, RadiometrySpec, RenderSpec, VisualBudgetSpec, VisualSpec,
};

//! A route navigation trial: a waypoint route flown on truth by Navigate's
//! plan executor and guidance, sensed once, and replayed through Navigate's
//! inertial filter under each sensor set.
//!
//! Three stages keep truth where it belongs. [`fly_route`] and the sensing
//! in [`record_trial`] read truth, as the vehicle and its sensors' physics.
//! [`SetRun`] replays a [`SensorLog`] and sees only the log and its own
//! solution: the star tracker's prior is the navigator's estimate.
//! [`evaluate`] meets the estimates with truth to score them. Every set sees the same sensor
//! realisations; a sensor a set lacks sends no observations.
//!
//! Visual terrain navigation ([`VisualSpec`]) adds a vehicle camera that
//! Simulate renders from the world imagery at the true pose, and Navigate's
//! visual chain, which renders references from the onboard
//! [`crate::package::MapPackage`] at the navigator's estimated pose.

mod closed;
mod contribution;
mod epoch;
mod error;
mod evaluate;
mod gnss;
mod models;
mod record;
mod replay;
mod report;
mod request;
mod route;
mod score;
mod sky;
mod stars;
mod visual;

pub use closed::{
    ClosedLoopOutcome, ClosedLoopReport, CrossTrackSample, Pilot, PilotEvent, PilotInput,
    PilotOutput, TerminationReport, WaypointOutcome,
};
pub use contribution::{Contribution, ablation_origin, default_sensor_sets};
pub use error::TrialError;
pub use evaluate::{
    ClosedTrialOutcome, RecordedTrial, SetTimings, TrialOutcome, evaluate, record_trial,
    run_closed_trial, run_trial, run_trial_with_timings,
};
pub use gnss::{BaroReading, GnssFix};
use models::model_statements;
pub use record::{Logged, SensorLog};
pub use replay::{
    ObservationTally, SensorTallies, SetRun, SetSelection, StarAvailability, trial_inertial_config,
};
pub use report::{
    ConsistencyNote, ErrorStats, InputIdentity, ModelStatement, OutageReport, REPORT_SCHEMA,
    RouteReport, SeriesSample, SetReport, SetSummary, TrackPoint, TrialReport,
};
pub use request::{
    AlignmentSpec, BaroSpec, GnssSpec, GuidanceSource, ImuGradeName, OutageSpec, OutputSpec,
    QualityFloor, SeedSpec, SensorSetSpec, StarSpec, TerminationSpec, TrialRequest, VehicleSpec,
    WaypointSpec,
};
pub use route::{RouteEvent, RouteFlight, execution_config, fly_route, plan_waypoints};
pub use sky::SkySample;
pub use stars::sighting::{SightingOutcome, StarSighting, pair_table};
pub use stars::{
    StarExposure, StarFrame, StarFrameTruth, StarImage, StarScene, StarSky, render_star_frames,
};
pub use visual::{
    CameraFrame, CameraFrameTruth, CameraSpec, FixRecord, FrameTiming, LiveCamera, MapIdentity,
    MapsSpec, PriorSpec, RadiometrySpec, RenderSpec, ScoredFix, VISUAL_SOURCE, VisualAid,
    VisualBudgetSpec, VisualMapsReport, VisualReport, VisualSpec, VisualTally, package_specs,
};

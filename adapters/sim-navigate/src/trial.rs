//! A route navigation trial: a waypoint route flown on truth by Navigate's
//! plan executor and guidance, sensed once, and replayed through Navigate's
//! inertial filter under each sensor set.
//!
//! Three stages keep truth where it belongs. [`fly_route`] and the sensing
//! in [`record_trial`] read truth, as the vehicle and its sensors' physics.
//! [`SetRun`] replays a [`SensorLog`] and sees only the log and its own
//! solution: the star tracker's prior is the navigator's estimate.
//! [`evaluate`] meets the estimates with truth to score them. Every set sees
//! the same sensor realisations; a sensor a set lacks, or one an
//! [`AvailabilitySchedule`] withholds, sends no observations. A
//! [`ProgressSink`] watches the sets advance together in route time.

mod availability;
mod closed;
mod comparison;
mod defaults;
mod epoch;
mod error;
mod evaluate;
mod gnss;
mod models;
mod progress;
mod record;
mod replay;
mod report;
mod request;
mod route;
mod score;
mod sky;
mod stars;
mod vertical;

pub use availability::{
    AvailabilitySchedule, ScheduledInterval, SensorKind, SensorState, SetAvailability,
    UnavailableInterval,
};
pub use comparison::{Baseline, ComparisonRow, ComparisonTable};
pub use defaults::{DEFAULT_EPOCH_UTC, RoutePoint, default_sensor_sets, request_from_route};
pub use progress::{
    EventKind, EventRecord, FinishedRecord, NoProgress, PROGRESS_SCHEMA, ProgressPosition,
    ProgressRecord, ProgressRoute, ProgressSet, ProgressSink, ProgressWaypoint, SampleRecord,
    SensorStates, StartedRecord,
};
pub use vertical::{FlownPoint, LegVertical, ProfilePoint, VerticalReport, vertical_plan};

pub use closed::{
    ClosedLoopOutcome, ClosedLoopReport, CrossTrackSample, Pilot, PilotEvent, PilotInput,
    PilotOutput, TerminationReport, WaypointOutcome,
};
pub use error::TrialError;
pub use evaluate::{
    ClosedTrialOutcome, RecordedTrial, TrialOutcome, evaluate, evaluate_with_progress,
    record_trial, replay_set, run_closed_trial, run_trial, run_trial_with_progress,
    star_pair_table,
};
pub use gnss::{BaroReading, GnssFix};
use models::model_statements;
pub use record::{Logged, SensorLog};
pub use replay::{
    ObservationTally, SensorTallies, SetRun, SetSelection, StarAvailability, trial_inertial_config,
};
pub use report::{
    ConsistencyNote, ErrorStats, InputIdentity, IntervalErrors, IntervalReport, ModelStatement,
    REPORT_SCHEMA, RouteReport, SeriesSample, SetReport, SetSummary, TrackPoint, TrialReport,
};
pub use request::{
    AlignmentSpec, BaroSpec, GnssSpec, GuidanceSource, ImuGradeName, OutputSpec, QualityFloor,
    SeedSpec, SensorSetSpec, StarSpec, TerminationSpec, TrialRequest, VehicleSpec, WaypointSpec,
};
pub use route::{RouteEvent, RouteFlight, execution_config, fly_route, plan_waypoints};
pub use sky::SkySample;
pub use stars::sighting::{SightingOutcome, StarSighting, pair_table};
pub use stars::{
    StarExposure, StarFrame, StarFrameTruth, StarImage, StarScene, StarSky, render_star_frames,
};

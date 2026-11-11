//! A route navigation trial: a waypoint route flown by a linked flight
//! director on truth, sensed once, and replayed through a linked navigator
//! under each sensor set. The trial links Navigate's onboard composition as
//! `navigate_onboard::Navigator` and `navigate_onboard::FlightDirector` trait
//! objects in one place, [`trial_navigator_blocking`] and [`trial_director`], and
//! otherwise only simulates the world, the vehicle and the sensors, hands
//! sensor inputs over, and scores.
//!
//! Three stages keep truth where it belongs. [`fly_route`] and the sensing
//! in [`record_trial_blocking`] read truth, as the vehicle and its sensors' physics.
//! [`SetRun`] replays a [`SensorLog`] and sees only the log and its own
//! solution.
//! [`evaluate_blocking`] meets the estimates with truth to score them. Every set sees
//! the same sensor realisations; a sensor a set lacks, or one an
//! [`AvailabilitySchedule`] withholds, sends no observations. A
//! [`ProgressSink`] watches the sets advance together in route time.

mod availability;
mod closed;
mod comparison;
mod defaults;
mod devices;
mod epoch;
mod error;
mod evaluate;
mod map_store;
mod maps;
mod models;
mod onboard;
mod progress;
mod record;
mod replay;
mod report;
mod request;
mod route;
mod score;
mod sky;
mod stars;
mod terrain_camera;
mod terrain_report;
mod timing;
mod vertical;

pub use availability::{
    AvailabilitySchedule, ScheduledInterval, SensorKind, SensorState, SetAvailability,
    UnavailableInterval,
};
pub use comparison::{Baseline, ComparisonRow, ComparisonTable};
pub use defaults::{DEFAULT_EPOCH_UTC, RoutePoint, default_sensor_sets, request_from_route};
pub use map_store::{DirectoryStores, MapFile, MapStore, MapStores, NamedStores};
pub use maps::{
    AssumedAccuracy, GeoidChoice, MapsReport, MapsSpec, OnboardMapSpec, OpenedPackage, PackageRef,
    PackageRefusal, PackageReport, WORLD_GEOID, maps_report_blocking,
};
pub use progress::{
    EventKind, EventRecord, FinishedRecord, NoProgress, PROGRESS_SCHEMA, ProgressPosition,
    ProgressRecord, ProgressRoute, ProgressSet, ProgressSink, ProgressWaypoint, SampleRecord,
    SensorStates, StartedRecord,
};
pub use terrain_camera::{
    CameraBand, CameraMountSpec, RadiometrySpec, TerrainCamera, TerrainCameraSpec, TerrainExposer,
    TerrainExposure, TerrainFrame, TerrainFrameTruth,
};
pub use terrain_report::{MapLayerReport, MapLoadReport, TerrainSetReport, WorkingCameraReport};
pub use timing::{SetTiming, TrialTiming};
pub use vertical::{FlownPoint, LegVertical, ProfilePoint, VerticalReport, vertical_plan};

pub use closed::{
    Autopilot, ClosedLoopOutcome, ClosedLoopReport, CrossTrackSample, PilotInput, PilotOutput,
    TerminationReport, WaypointOutcome,
};
pub use epoch::parse_utc;
pub use error::TrialError;
pub use evaluate::{
    ClosedTrialOutcome, RecordedTrial, TrialOutcome, evaluate_blocking,
    evaluate_with_progress_blocking, record_trial_blocking, record_trial_with_stores,
    replay_set_blocking, replay_set_with_stores, run_closed_trial, run_trial_blocking,
    run_trial_timed_blocking, run_trial_with_progress_blocking, run_trial_with_stores,
};
use models::model_statements;
pub use onboard::{
    BARO_SOURCE, GNSS_SOURCE, IMU_SOURCE, STAR_SOURCE, sensor_suite, star_catalogue,
    trial_director, trial_navigator_blocking, trial_navigator_with_stores,
};
pub use record::{Logged, SensorLog};
pub use replay::{ObservationTally, SensorTallies, SetRun, SetSelection, StarAvailability};
pub use report::{
    ConsistencyNote, ErrorStats, InputIdentity, IntervalErrors, IntervalReport, ModelStatement,
    REPORT_SCHEMA, RouteReport, SeriesSample, SetReport, SetSummary, TrackPoint, TrialReport,
};
pub use request::{
    AlignmentSpec, BaroSpec, GnssSpec, GuidanceSource, ImuGradeName, OutputSpec, QualityFloor,
    SeedSpec, SensorSetSpec, StarSpec, TerminationSpec, TrialRequest, VehicleSpec, WaypointSpec,
};
pub use route::{RouteEvent, RouteFlight, execution_config, fly_route, plan_waypoints, step_span};
pub use sky::SkySample;
pub use stars::{
    StarExposure, StarFrame, StarFrameTruth, StarImage, StarScene, StarSky, render_star_frames,
    roof_star_mount, star_camera_config,
};

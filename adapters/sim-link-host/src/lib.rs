//! Hosting a simulation link: what a browser page or a headless tool does
//! with a decoded [`sim_link::SimulationLink`] before anything flies.
//!
//! - [`flyable_route`] takes the legs the vehicle's software flies today and
//!   refuses every other leg by its number and code.
//! - [`hosted_request`] maps the link onto the one request every host
//!   shares, [`sim_navigate::trial::TrialRequest`], converting heights to
//!   the ellipsoid and outages to availability schedules. What the trial
//!   request has no field for rides beside it in [`HostedRequest`], so
//!   nothing a link says is dropped.
//! - [`plan_summary`] gives the legs, the flight time and the Sun's
//!   elevation along the route, from Navigate's route geometry and
//!   ephemeris.
//! - [`plan_lines`] words the link and its summary in plain language.
//! - [`package_route_msl`] and [`package_camera`] say what the map package
//!   is planned for, and [`package_plan_lines`], [`package_lines`] and
//!   [`sealed_lines`] word its plan, its progress and its result;
//!   [`world_memory_bytes`] is what the camera's world of the package takes
//!   in the tab once decoded, checked against [`WORLD_MEMORY_CEILING_BYTES`].
//! - [`chain_spec`] maps the link onto the vehicle's own control chain, and
//!   [`ChainFlight`] steps that chain as the page's flight, rendering its
//!   terrain camera's frames when asked.
//! - [`SwitchLog`] writes the visitor's switches into the link of the run,
//!   so the link reopens the same flight.
//! - [`Monitor`] is what the page shows while the link flies, fed by the
//!   flight; [`run_slice`] and [`Pacer`] step a [`FlightStepper`] in slices
//!   a page can draw between, and [`Replay`] flies a recorded run again.

mod chain;
mod error;
mod flyable;
mod monitor;
mod package;
mod request;
mod run_link;
mod summary;
mod wording;
mod world;

#[cfg(test)]
mod tests;

pub use chain::{
    ASSUMED_ACCURACY, C172_CRUISE_M_S, ChainFlight, MAX_STEP_S, OVERRUN_S, PAGE_STORE, chain_spec,
    link_outages, link_spoof, page_stores,
};
pub use error::HostError;
pub use flyable::{
    FlyableFix, FlyableRoute, HOLD_COURSE_TOLERANCE_DEG, TerminalHold, final_course_deg,
    flyable_route, path_words,
};
pub use monitor::{
    ABSENT, CameraLog, CameraShot, Clock, Column, EVENTS_KEPT, FLIGHT_SHARE, FixMark,
    FlightStepper, GreyImage, HoldState, MAX_FRAME_S, MatchLine, MatchShot, MeansSwitch, Monitor,
    MonitorFeed, OfferedPosition, PaceLimit, Pacer, RUN_LINES, Replay, RouteEnd, SAMPLES_KEPT,
    SLICE_BUDGET_S, SensorControl, SensorFault, SetTrace, Slice, SolvedFit, SpoofDrag, Spoofing,
    StepState, TracePoint, clock_words, entry_words, event_words, leg_words, outcome_words,
    pattern_words, run_lines, run_slice, sensor_controls, sensor_words, starting_lines,
    state_words, stopped_lines, terrain_words,
};
pub use package::{
    BYTES_PER_MB, PACKAGE_LINES, PackageStage, budget_bytes, budget_refusal_lines, error_words,
    package_camera, package_lines, package_plan_lines, package_route_msl, sealed_lines,
};
pub use request::{
    CameraGeometry, HostedHold, HostedRequest, StartState, VEHICLE_CAMERA, hosted_request,
    own_cruise_speed_m_s,
};
pub use run_link::{SpoofRun, SwitchLog};
pub use summary::{LegSummary, Light, PlanSummary, SunSample, plan_summary};
pub use wording::{PlanLine, outage_words, plan_lines, refusal_lines, spoof_words};
pub use world::{
    CameraWorld, MapView, WORLD_MEMORY_CEILING_BYTES, mercator, unmercator, world_line,
    world_memory_bytes, world_memory_refusal,
};

//! Hosting a simulation link: what a browser page or a headless tool does
//! with a decoded [`sim_link::SimulationLink`] before anything flies.
//!
//! - [`hosted_request`] maps the link onto the one request every host
//!   shares, [`sim_navigate::trial::TrialRequest`], converting heights to
//!   the ellipsoid and outages to availability schedules. What the trial
//!   request has no field for rides beside it in [`HostedRequest`], so
//!   nothing a link says is dropped.
//! - [`plan_summary`] gives the legs, the flight time and the Sun's
//!   elevation along the route, from Navigate's route geometry and
//!   ephemeris.
//! - [`plan_lines`] words the link and its summary in plain language.

mod error;
mod request;
mod summary;
mod wording;

pub use error::HostError;
pub use request::{
    CameraGeometry, HostedRequest, StartState, UNTIL_END_S, VEHICLE_CAMERA, hosted_request,
    own_cruise_speed_m_s, sensor_seeds,
};
pub use summary::{LegSummary, Light, PlanSummary, SunSample, plan_summary};
pub use wording::{PlanLine, outage_words, plan_lines, refusal_lines};

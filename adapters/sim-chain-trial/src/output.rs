//! What a step gives back.

use std::collections::BTreeMap;

use sim_flight_chain::FcAvionics;
use sim_navigate::trial::{GnssCheck, ProgressRecord};

use crate::TruthSnapshot;

/// How a step ended.
#[derive(Debug, Clone, PartialEq)]
pub enum StepOutcome {
    /// The simulation advanced; what happened on the way.
    Advanced(Outputs),
    /// A terrain camera frame is wanted: rendered now, or the simulation
    /// cannot pass its readout. What happened before the request rides
    /// along.
    FrameWanted(FrameRequest, Outputs),
}

/// What happened over one or more cycles, in order.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Outputs {
    /// The navigation trial's progress records.
    pub records: Vec<ProgressRecord>,
    /// What Pilotage's mission did, when it did anything.
    pub mission: Vec<MissionUpdate>,
    /// Each set's terrain camera fixes so far, when they changed.
    pub terrain_fixes: Vec<(String, BTreeMap<String, u64>)>,
    /// What each GNSS set's navigator says of its receiver, by set, at the
    /// progress stream's rate.
    pub gnss_checks: Vec<(String, GnssCheck)>,
    /// The flight controller's account at the feed rate the host asked
    /// for, in order; empty unless it asked.
    pub avionics: Vec<FcAvionics>,
    /// The truth at the same instants as [`Self::avionics`], for the world
    /// outside the aeroplane only.
    pub truth: Vec<TruthSnapshot>,
}

impl Outputs {
    /// Whether nothing happened.
    pub fn is_empty(&self) -> bool {
        self.records.is_empty()
            && self.mission.is_empty()
            && self.terrain_fixes.is_empty()
            && self.gnss_checks.is_empty()
            && self.avionics.is_empty()
            && self.truth.is_empty()
    }

    /// Appends `later` to these.
    pub fn extend(&mut self, later: Self) {
        self.records.extend(later.records);
        self.mission.extend(later.mission);
        self.terrain_fixes.extend(later.terrain_fixes);
        self.gnss_checks.extend(later.gnss_checks);
        self.avionics.extend(later.avionics);
        self.truth.extend(later.truth);
    }
}

/// The mission's state and what it reported, in Pilotage's own words.
#[derive(Debug, Clone, PartialEq)]
pub struct MissionUpdate {
    /// Route time, seconds.
    pub t_s: f64,
    /// The mission phase as Pilotage names it, once a mission is loaded.
    pub phase: Option<String>,
    /// Pilotage's events since the last update, as it prints them.
    pub events: Vec<String>,
}

/// A terrain camera exposure the host is to render.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrameRequest {
    /// The exposure's index in the run, from 1.
    pub index: u64,
    /// Route time of the middle of the exposure, seconds.
    pub mid_s: f64,
    /// Route time the frame is read out, seconds: the simulation does not
    /// pass it before the frame is delivered.
    pub readout_s: f64,
    /// The camera's optical centre, ECEF metres.
    pub eye_ecef: [f64; 3],
    /// The camera's axes (+X right, +Y up, −Z forward) into ECEF, as a
    /// quaternion `[w, x, y, z]`.
    pub camera_to_ecef_wxyz: [f64; 4],
    /// Image width and height, pixels.
    pub size: (u32, u32),
    /// Focal lengths and principal point, pixels: `[fx, fy, cx, cy]`.
    pub pinhole: [f64; 4],
    /// The Sun's elevation at the eye, degrees.
    pub sun_elevation_deg: f64,
}

/// What a chain trial's log keeps: bounded by how far its slowest reader
/// lags, not by the flight's length.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Retained {
    /// Navigation steps logged, forgotten ones included.
    pub steps: usize,
    /// Steps whose instant, IMU increment or truth is still kept.
    pub steps_kept: usize,
    /// Receiver fixes and air-data samples still kept.
    pub readings_kept: usize,
    /// Star and terrain camera frames still kept.
    pub frames_kept: usize,
    /// Pixel bytes the kept terrain frames hold.
    pub terrain_pixel_bytes: usize,
    /// Roughly the bytes all of it takes.
    pub approximate_bytes: usize,
}

//! Sensing a flown route once: every sensor's readings along the truth, so
//! that every sensor set replays the same realisations.

use glam::DVec3;
use sim_core::{SimInstant, VehicleState};
use sim_devices::{AirDataSample, GnssReading, InertialIncrement};

use super::devices::{StaticPort, receiver};
use super::epoch::parse_utc;
use super::route::{STEP_HZ, step_seconds};
use super::stars::{StarExposure, StarFrame, StarScene, StarSky, render_star_frames};
use super::terrain_camera::TerrainFrame;
use super::{TrialError, TrialRequest};
use crate::{AlignmentFix, InitialAlignment, SimulatedImu, Truth};

/// A reading delivered at one simulation step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Logged<T> {
    /// The step at which the reading is acquired and delivered.
    pub step: usize,
    /// The reading.
    pub reading: T,
}

/// Everything the sensors reported along one flight. The only truth in it is
/// each star frame's scene record, behind [`Truth`], which a replay never
/// reads.
#[derive(Debug, Clone)]
pub struct SensorLog {
    /// The instant of every simulation step.
    pub instants: Vec<SimInstant>,
    /// The initial alignment at step 0.
    pub alignment: AlignmentFix,
    /// The IMU increment ending at each step from step 1.
    pub imu: Vec<InertialIncrement>,
    /// GNSS receiver fixes.
    pub gnss: Vec<Logged<GnssReading>>,
    /// Static-port samples.
    pub baro: Vec<Logged<AirDataSample>>,
    /// Star camera frames; their truth part is for scoring only.
    pub stars: Vec<StarFrame>,
    /// The mid-exposure step of every frame, in order, known from the
    /// camera's schedule before the frame is read out.
    pub star_mid_steps: Vec<usize>,
    /// Terrain camera frames; their truth part is for scoring only.
    pub terrain: Vec<TerrainFrame>,
}

/// Steps between readings of a sensor at `rate_hz`.
pub(super) fn period_steps(field: &'static str, rate_hz: f64) -> Result<usize, TrialError> {
    let steps = STEP_HZ as f64 / rate_hz;
    let whole = steps.round();
    if !(rate_hz > 0.0 && (steps - whole).abs() < 1e-9 && whole >= 1.0) {
        return Err(TrialError::InvalidRequest {
            field,
            detail: format!("{rate_hz} Hz does not divide {STEP_HZ} Hz"),
        });
    }
    Ok(whole as usize)
}

/// Senses `truth` with every sensor of `request`, each from its own seed.
pub fn record(
    request: &TrialRequest,
    truth: &[Truth<(SimInstant, VehicleState)>],
) -> Result<SensorLog, TrialError> {
    let Some(first) = truth.first() else {
        return Err(TrialError::InvalidRequest {
            field: "waypoints",
            detail: "the route was not flown".into(),
        });
    };
    let (start, initial) = *first.get();
    let alignment = initial_alignment(request).measure(&initial);
    let mut imu = SimulatedImu::new(request.imu_grade.grade().spec(), request.seeds.imu)?;
    imu.sense(start, &initial)?;
    let mut increments = Vec::with_capacity(truth.len());
    for sample in truth.iter().skip(1) {
        let (at, state) = sample.get();
        if let Some(increment) = imu.sense(*at, state)? {
            increments.push(increment);
        }
    }
    let mut gnss_receiver = receiver(request)?;
    let gnss_period = period_steps("gnss", request.gnss.rate_hz)?;
    let gnss = periodic(truth, gnss_period, |_, s| {
        Ok(gnss_receiver.measure(s, step_seconds(gnss_period)))
    })?;
    let mut port = StaticPort::new(request)?;
    let baro_period = period_steps("baro", request.baro.rate_hz)?;
    let baro = periodic(truth, baro_period, |step, s| {
        port.sense(s, step_seconds(step), step_seconds(baro_period))
            .map(Some)
    })?;
    let stars = if request.uses_stars() {
        render_star_frames(truth, &star_scene(request)?)?
    } else {
        Vec::new()
    };
    Ok(SensorLog {
        instants: truth.iter().map(|sample| sample.get().0).collect(),
        alignment,
        imu: increments,
        gnss,
        baro,
        star_mid_steps: stars.iter().map(|frame| frame.image.mid_step).collect(),
        terrain: Vec::new(),
        stars,
    })
}

/// The request's initial alignment, with its seed.
pub(super) fn initial_alignment(request: &TrialRequest) -> InitialAlignment {
    InitialAlignment {
        position_sigma_m: DVec3::splat(request.alignment.position_sigma_m),
        velocity_sigma_m_s: DVec3::splat(request.alignment.velocity_sigma_m_s),
        attitude_sigma_rad: DVec3::splat(request.alignment.attitude_sigma_rad),
        seed: request.seeds.alignment,
    }
}

/// The request's star camera along the route, with the Sun-lit sky.
pub fn star_scene(request: &TrialRequest) -> Result<StarScene, TrialError> {
    let steps = request.stars.period_s * STEP_HZ as f64;
    if !(steps.is_finite() && steps >= 1.0 && (steps - steps.round()).abs() < 1e-9) {
        return Err(TrialError::InvalidRequest {
            field: "stars.period_s",
            detail: format!(
                "{} s is not a whole number of steps",
                request.stars.period_s
            ),
        });
    }
    Ok(StarScene {
        start: parse_utc(&request.epoch_utc)?,
        step_hz: STEP_HZ as f64,
        period_steps: steps.round() as usize,
        exposure: StarExposure::Moving,
        sky: StarSky::SunLit,
        seed: request.seeds.star_camera,
    })
}

/// A reading every `period` steps from step `period`, in order; a sample
/// whose device produced nothing is absent.
fn periodic<T>(
    truth: &[Truth<(SimInstant, VehicleState)>],
    period: usize,
    mut measure: impl FnMut(usize, &VehicleState) -> Result<Option<T>, TrialError>,
) -> Result<Vec<Logged<T>>, TrialError> {
    let mut log = Vec::new();
    for (step, sample) in truth.iter().enumerate().skip(period).step_by(period) {
        if let Some(reading) = measure(step, &sample.get().1)? {
            log.push(Logged { step, reading });
        }
    }
    Ok(log)
}

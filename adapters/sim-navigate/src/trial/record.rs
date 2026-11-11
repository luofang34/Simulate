//! Sensing a flown route once: every sensor's readings along the truth, so
//! that every sensor set replays the same realisations.

use std::path::Path;
use std::sync::Arc;

use glam::DVec3;
use sim_core::{SimInstant, VehicleState};
use sim_devices::InertialIncrement;
use sim_scenery::CatalogScenery;

use super::epoch::parse_utc;
use super::gnss::{BaroAltimeter, BaroReading, GnssFix, GnssReceiver};
use super::route::STEP_HZ;
use super::stars::{StarExposure, StarFrame, StarScene, StarSky, render_star_frames};
use super::visual::{CameraFrame, LiveCamera, VisualSpec};
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
    /// GNSS fixes.
    pub gnss: Vec<Logged<GnssFix>>,
    /// Barometric altitudes.
    pub baro: Vec<Logged<BaroReading>>,
    /// Star camera frames; their truth part is for scoring only.
    pub stars: Vec<StarFrame>,
    /// The mid-exposure step of every frame, in order, known from the
    /// camera's schedule before the frame is read out.
    pub star_mid_steps: Vec<usize>,
    /// Vehicle camera frames; their truth part is for scoring only.
    pub camera_frames: Vec<CameraFrame>,
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
    let mut receiver = GnssReceiver::new(request.gnss, request.seeds.gnss)?;
    let gnss = periodic(truth, period_steps("gnss", request.gnss.rate_hz)?, |s| {
        receiver.measure(s)
    });
    let mut altimeter = BaroAltimeter::new(request.baro, request.seeds.baro)?;
    let baro = periodic(truth, period_steps("baro", request.baro.rate_hz)?, |s| {
        altimeter.measure(s)
    });
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
        stars,
        camera_frames: camera_frames(request, truth)?,
    })
}

/// The vehicle camera of `request` over its world imagery, when any set
/// takes visual fixes.
pub(super) fn live_camera(request: &TrialRequest) -> Result<Option<LiveCamera>, TrialError> {
    let Some(visual) = request.visual.as_ref().filter(|_| request.uses_visual()) else {
        return Ok(None);
    };
    let world = CatalogScenery::from_catalog_dir_blocking(Path::new(&visual.world_dir)).map_err(
        |error| TrialError::Visual {
            what: format!("world imagery {}: {error}", visual.world_dir),
        },
    )?;
    Ok(Some(LiveCamera::new(
        Arc::new(world),
        visual.camera,
        visual.radiometry,
        request.seeds.camera.unwrap_or_default(),
        parse_utc(&request.epoch_utc)?,
    )))
}

/// Steps between vehicle camera frames.
pub(super) fn camera_period_steps(visual: &VisualSpec) -> Result<usize, TrialError> {
    let steps = visual.camera.period_s * STEP_HZ as f64;
    if !(steps.is_finite() && steps >= 1.0 && (steps - steps.round()).abs() < 1e-9) {
        return Err(TrialError::InvalidRequest {
            field: "visual.camera.period_s",
            detail: format!(
                "{} s is not a whole number of steps",
                visual.camera.period_s
            ),
        });
    }
    Ok(steps.round() as usize)
}

fn camera_frames(
    request: &TrialRequest,
    truth: &[Truth<(SimInstant, VehicleState)>],
) -> Result<Vec<CameraFrame>, TrialError> {
    let (Some(camera), Some(visual)) = (live_camera(request)?, request.visual.as_ref()) else {
        return Ok(Vec::new());
    };
    let period = camera_period_steps(visual)?;
    truth
        .iter()
        .enumerate()
        .skip(period)
        .step_by(period)
        .map(|(step, sample)| {
            let (at, state) = sample.get();
            camera.capture((step / period) as u64, step, *at, state)
        })
        .collect()
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

/// A reading every `period` steps from step `period`, in order.
fn periodic<T>(
    truth: &[Truth<(SimInstant, VehicleState)>],
    period: usize,
    mut measure: impl FnMut(&VehicleState) -> T,
) -> Vec<Logged<T>> {
    truth
        .iter()
        .enumerate()
        .skip(period)
        .step_by(period)
        .map(|(step, sample)| Logged {
            step,
            reading: measure(&sample.get().1),
        })
        .collect()
}

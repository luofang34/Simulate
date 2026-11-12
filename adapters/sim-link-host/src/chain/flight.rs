//! The chain trial as a page's flight: stepped toward the pace's target in
//! short calls, each terrain-camera frame rendered and delivered within
//! the same call, and the run ended where the link's flight ends.

use glam::{DQuat, DVec3};
use sim_chain_trial::{ChainTrial, FrameRequest, StepOutcome};
use sim_geo::ecef_to_geodetic;
use sim_navigate::trial::{ProgressRecord, SensorKind};

use super::{feeds, finished, intrinsics};
use crate::{CameraShot, CameraWorld, Clock, FlightStepper, HostError, MonitorFeed, StepState};

/// Route time one call of the chain may cover, seconds: a call returns to
/// the page's slice no later than this, whatever the target.
pub const MAX_STEP_S: f64 = 0.25;
/// Route time the run may go on past half again its planned duration,
/// seconds, before the page ends it: a mission that never completes still
/// ends. One that completes flies on in its hold for as long as the page
/// is open.
pub const OVERRUN_S: f64 = 120.0;

/// The chain trial flown in a page.
pub struct ChainFlight<C> {
    trial: ChainTrial,
    world: Option<CameraWorld>,
    clock: C,
    started_wall_s: f64,
    camera_wall_s: f64,
    end_s: f64,
    ended: bool,
}

impl<C: Clock> ChainFlight<C> {
    /// `trial` flown with `world` as what its terrain camera sees, timed on
    /// `clock`.
    pub fn new(trial: ChainTrial, world: Option<CameraWorld>, clock: C) -> Self {
        Self {
            trial,
            world,
            started_wall_s: clock.now_s(),
            clock,
            camera_wall_s: 0.0,
            end_s: f64::INFINITY,
            ended: false,
        }
    }

    /// The trial being flown, for a host that shows more of it than the
    /// monitor's feed carries.
    pub fn trial(&self) -> &ChainTrial {
        &self.trial
    }

    /// Renders and delivers the frame `request` asks for, and hands the
    /// monitor a copy.
    fn develop(
        &mut self,
        request: &FrameRequest,
        out: &mut Vec<MonitorFeed>,
    ) -> Result<(), HostError> {
        let world = self.world.as_ref().ok_or_else(|| HostError::Render {
            detail: format!("frame {} wanted and no world to render", request.index),
        })?;
        let started = self.clock.now_s();
        let [w, x, y, z] = request.camera_to_ecef_wxyz;
        let eye = DVec3::from_array(request.eye_ecef);
        let frame = world.render(eye, DQuat::from_xyzw(x, y, z, w), &intrinsics(request))?;
        self.trial.deliver_frame(request.index, frame.pixels())?;
        self.camera_wall_s += (self.clock.now_s() - started).max(0.0);
        let at = ecef_to_geodetic(eye);
        out.push(MonitorFeed::Frame(CameraShot {
            index: request.index,
            mid_s: request.mid_s,
            size: request.size,
            luma: frame.pixels().to_vec(),
            at_deg: (at.lat_rad.to_degrees(), at.lon_rad.to_degrees()),
        }));
        Ok(())
    }

    /// Notes what the new feeds say of the run's end.
    fn watch(&mut self, feeds: &[MonitorFeed]) {
        for feed in feeds {
            let MonitorFeed::Progress(record) = feed else {
                continue;
            };
            if let ProgressRecord::Started(started) = record {
                self.end_s = 1.5 * started.route.planned_duration_s + OVERRUN_S;
            }
            if super::completes_plan(record) {
                self.end_s = f64::INFINITY;
            }
            self.ended |= super::ends_flight(record);
        }
        self.ended |= self.trial.now_s() >= self.end_s;
    }
}

impl<C: Clock> FlightStepper for ChainFlight<C> {
    type Error = HostError;

    fn sim_time_s(&self) -> f64 {
        self.trial.now_s()
    }

    fn step(&mut self, target_s: f64, out: &mut Vec<MonitorFeed>) -> Result<StepState, HostError> {
        if self.ended {
            return Ok(StepState::Finished);
        }
        let until_s = target_s.min(self.trial.now_s() + MAX_STEP_S);
        let from = out.len();
        match self.trial.step_until(until_s)? {
            StepOutcome::Advanced(outputs) => feeds(outputs, out),
            StepOutcome::FrameWanted(request, outputs) => {
                feeds(outputs, out);
                self.develop(&request, out)?;
            }
        }
        self.watch(out.get(from..).unwrap_or_default());
        if self.ended {
            out.push(MonitorFeed::Progress(finished(
                self.clock.now_s() - self.started_wall_s,
            )));
            return Ok(StepState::Finished);
        }
        Ok(StepState::Running)
    }

    fn take_camera_wall_s(&mut self) -> f64 {
        std::mem::take(&mut self.camera_wall_s)
    }

    fn switch(&mut self, sensor: SensorKind, on: bool) {
        self.trial.set_sensor(sensor, on);
    }

    fn outages(&self) -> Vec<(SensorKind, f64, Option<f64>)> {
        super::link_outages(self.trial.outages())
    }
}

//! The chain trial, stepped one flight-controller cycle at a time.

mod camera;
mod feed;
mod holding;
mod records;
mod sets;
mod setup;
#[cfg(test)]
mod tests;

use aviate_core::control::fixed_wing::FixedWingController;
use aviate_core::mixer::fixed_wing::FixedWingMixer;
use sim_core::SimInstant;
use sim_flight_chain::{C172Airframe, DeviceOutages, FcAvionics, HostChain, NavigationDisplay};
use sim_navigate::NavMeans;
use sim_navigate::trial::{
    LiveStarCamera, LogPositions, MapStores, SensorKind, TerrainExposer, TerrainMatch,
    replay_set_inspecting,
};

use crate::nav_log::NavLog;
use crate::{
    ChainTrialError, ChainTrialSpec, DeviceOutage, FrameRequest, Outputs, Steering, StepOutcome,
};
use camera::HostCamera;
use sets::{SetSlot, has_sensor};

pub use feed::TruthSnapshot;
pub use sets::FLIGHT_CONTROLLER_SET;

/// The C172 under Pilotage's host core.
pub type C172Chain = HostChain<C172Airframe, FixedWingController, FixedWingMixer>;

/// A navigation trial flown by the vehicle's own control chain.
///
/// Pilotage's host core flies the C172 through Aviate, steering on the
/// navigation the spec names, while every Navigate set navigates on the
/// same devices the flight controller reads. The host steps it one
/// flight-controller cycle at a time, or up to a route time, and renders
/// the terrain camera's frames when asked.
pub struct ChainTrial {
    chain: Box<C172Chain>,
    nav: NavLog,
    sets: Vec<SetSlot>,
    steering: Option<usize>,
    outages: Vec<DeviceOutage>,
    camera: Option<HostCamera>,
    /// The star camera, when a set navigates on the stars.
    stars: Option<LiveStarCamera>,
    announced: Option<FrameRequest>,
    progress_steps: usize,
    pending: Outputs,
    records: records::Recorder,
    feed: Option<feed::FeedClock>,
    holding: holding::HoldingWatch,
}

impl ChainTrial {
    /// The trial of `spec`, its onboard packages read from `stores`: the
    /// aeroplane trimmed at the start, every set aligned, nothing flown.
    pub fn new(spec: &ChainTrialSpec, stores: &dyn MapStores) -> Result<Self, ChainTrialError> {
        Self::new_inspecting(spec, stores, false)
    }

    /// [`Self::new`], whose visual sets keep a record of their latest
    /// terrain match when `inspect` is on ([`Self::terrain_matches`]). The
    /// flight and every navigator's output are the same either way.
    pub fn new_inspecting(
        spec: &ChainTrialSpec,
        stores: &dyn MapStores,
        inspect: bool,
    ) -> Result<Self, ChainTrialError> {
        let resolved = setup::resolve(spec)?;
        let chain = setup::chain(spec, &resolved)?;
        let start = chain.true_state();
        let alignment = setup::alignment(&resolved.request, &start, chain.board().suite());
        let nav = NavLog::new(start, alignment);
        let request = &resolved.request;
        let sets = request
            .sensor_sets
            .iter()
            .map(|set| {
                let run = replay_set_inspecting(request, set, &nav.log, (stores, inspect))?;
                Ok(SetSlot::new(set.name.clone(), run, |kind| {
                    has_sensor(&set.means, kind)
                }))
            })
            .collect::<Result<Vec<_>, ChainTrialError>>()?;
        let steering = match &spec.steering {
            Steering::FlightController => None,
            Steering::Set(name) => Some(
                sets.iter()
                    .position(|slot| &slot.name == name)
                    .ok_or_else(|| ChainTrialError::Invalid {
                        detail: format!("no set `{name}` to steer on"),
                    })?,
            ),
        };
        let camera = match request.terrain_camera {
            Some(camera) => {
                let seed = request.seeds.terrain_camera.unwrap_or(request.seeds.imu);
                Some(HostCamera::new(TerrainExposer::new(
                    camera,
                    seed,
                    resolved.utc,
                )?)?)
            }
            None => None,
        };
        let stars = match request
            .sensor_sets
            .iter()
            .any(|set| set.means.contains(&NavMeans::Celestial))
        {
            true => Some(LiveStarCamera::new(request)?),
            false => None,
        };
        let step_s = sim_navigate::trial::step_span().as_seconds();
        let progress_steps = (request.output.progress_period_s / step_s).round().max(1.0) as usize;
        Ok(Self {
            records: records::Recorder::new(request, &resolved.digest, spec),
            chain: Box::new(chain),
            nav,
            sets,
            steering,
            outages: spec.outages.clone(),
            camera,
            stars,
            announced: None,
            progress_steps,
            pending: Outputs::default(),
            feed: None,
            holding: holding::HoldingWatch::default(),
        })
    }

    /// The control chain the trial flies: the board, its airframe and
    /// Pilotage's host, for drawing and readouts.
    pub fn chain(&self) -> &C172Chain {
        &self.chain
    }

    /// The flight controller's account of its last cycle: what the
    /// aeroplane's own displays may show.
    pub fn avionics(&self) -> FcAvionics {
        feed::avionics(&self.chain)
    }

    /// What the trial's log keeps: as much as its slowest reader has yet to
    /// take, however long the flight.
    pub fn retained(&self) -> crate::Retained {
        self.nav.retained()
    }

    /// What Pilotage's navigation display shows of the mission now: the
    /// plan, the leg flown and the holding pattern after it. It is the
    /// flight-management side, apart from the flight controller's account.
    pub fn navigation(&self) -> Option<&NavigationDisplay> {
        self.chain.status().navigation.as_ref()
    }

    /// Each visual set's latest terrain match, by set name, for a trial built
    /// inspecting them.
    pub fn terrain_matches(&self) -> impl Iterator<Item = (&str, &TerrainMatch)> {
        self.sets.iter().filter_map(|slot| {
            slot.run
                .last_terrain_match()
                .map(|found| (slot.name.as_str(), found))
        })
    }

    /// The true aeroplane now, for drawing the world outside it only.
    pub fn truth(&self) -> TruthSnapshot {
        feed::truth(&self.chain)
    }

    /// From the next cycle on, every output carries the flight
    /// controller's account and the truth `rate_hz` times a second of route
    /// time, or never with `None`. A sample falls at the end of the first
    /// cycle at or past each due instant.
    pub fn set_feed_rate_hz(&mut self, rate_hz: Option<f64>) -> Result<(), ChainTrialError> {
        self.feed = rate_hz
            .map(|hz| feed::FeedClock::new(hz, self.chain.now()))
            .transpose()?;
        Ok(())
    }

    /// Route time, seconds.
    pub fn now_s(&self) -> f64 {
        seconds(self.chain.now())
    }

    /// One flight-controller cycle, unless a frame must come first.
    pub fn step(&mut self) -> Result<StepOutcome, ChainTrialError> {
        if let Some(started) = self.records.start() {
            self.pending.records.push(started);
        }
        let next = self.nav.last_step().wrapping_add(1);
        if let Some(request) = self.camera.as_ref().and_then(|c| c.blocking(next)) {
            return Ok(StepOutcome::FrameWanted(request, self.take_outputs()));
        }
        self.cycle()?;
        if let Some(request) = self.announced.take() {
            return Ok(StepOutcome::FrameWanted(request, self.take_outputs()));
        }
        Ok(StepOutcome::Advanced(self.take_outputs()))
    }

    /// Cycles until route time `until_s`, or until a frame is wanted, with
    /// everything that happened on the way.
    pub fn step_until(&mut self, until_s: f64) -> Result<StepOutcome, ChainTrialError> {
        let mut outputs = Outputs::default();
        while self.now_s() < until_s {
            match self.step()? {
                StepOutcome::Advanced(more) => outputs.extend(more),
                StepOutcome::FrameWanted(request, more) => {
                    outputs.extend(more);
                    return Ok(StepOutcome::FrameWanted(request, outputs));
                }
            }
        }
        Ok(StepOutcome::Advanced(outputs))
    }

    /// Takes `sensor`'s device out of service, or returns it, from the next
    /// cycle on, and returns that cycle's route time, seconds: a whole
    /// number of cycles. It is recorded as an outage beginning or ending
    /// then, the same as an outage the spec states.
    pub fn set_sensor(&mut self, sensor: SensorKind, on: bool) -> f64 {
        let at_s = self.now_s();
        let open = self
            .outages
            .iter_mut()
            .find(|o| o.sensor == sensor && o.end_s.is_none());
        match (open, on) {
            (Some(outage), true) => outage.end_s = Some(at_s),
            (None, false) => self.outages.push(DeviceOutage {
                sensor,
                start_s: at_s,
                end_s: None,
            }),
            _ => {}
        }
        at_s
    }

    /// The outages so far, those the spec stated and those switched.
    pub fn outages(&self) -> &[DeviceOutage] {
        &self.outages
    }

    /// The host's grey image for exposure `index`: exactly width × height
    /// bytes, row-major, top row first.
    pub fn deliver_frame(&mut self, index: u64, luma: &[u8]) -> Result<(), ChainTrialError> {
        let camera = self
            .camera
            .as_mut()
            .ok_or(ChainTrialError::UnexpectedFrame { index })?;
        let frame = camera.develop(index, luma)?;
        self.nav.log.terrain.push(frame);
        Ok(())
    }

    fn take_outputs(&mut self) -> Outputs {
        core::mem::take(&mut self.pending)
    }

    /// One cycle of the chain, and the navigation step it completed.
    fn cycle(&mut self) -> Result<(), ChainTrialError> {
        let t_s = self.now_s();
        let out = |kind| {
            self.outages
                .iter()
                .any(|o| o.sensor == kind && o.contains(t_s))
        };
        self.chain.board_mut().set_outages(DeviceOutages {
            gnss: out(SensorKind::Gnss),
            air_data: out(SensorKind::Baro),
        });
        let from = self.chain.now();
        let before = self.chain.true_state();
        self.chain.advance_tick()?;
        let after = self.chain.true_state();
        if self.feed.as_mut().is_some_and(|f| f.due(self.chain.now())) {
            self.pending.avionics.push(feed::avionics(&self.chain));
            self.pending.truth.push(feed::truth(&self.chain));
        }
        let samples = *self.chain.board().last_samples();
        let completed = self
            .nav
            .on_cycle((from, self.chain.now()), (&before, &after), &samples);
        if let Some(step) = completed {
            self.navigate(step)?;
        }
        let events = self.chain.drain_events();
        let sequenced = self.records.sequenced(&events);
        self.pending.records.extend(sequenced);
        let now_s = self.now_s();
        let held = self
            .holding
            .records(now_s, &events, self.chain.status().navigation.as_ref());
        self.pending.records.extend(held);
        let phase = self.chain.status().mission_state.map(|s| format!("{s:?}"));
        if let Some(update) = self.records.mission(self.now_s(), phase, &events) {
            self.pending.mission.push(update);
        }
        Ok(())
    }

    /// Every set to `step`, then the log released up to the slowest set and
    /// the truth up to the star camera's open exposure.
    fn advance_sets(&mut self, step: usize) -> Result<(), ChainTrialError> {
        for slot in &mut self.sets {
            slot.run.advance_to(&self.nav.log, step)?;
        }
        let Some(positions) = self
            .sets
            .iter()
            .map(|slot| slot.run.positions())
            .reduce(LogPositions::earliest)
        else {
            return Ok(());
        };
        let looking_back = self
            .stars
            .as_ref()
            .map_or(0, LiveStarCamera::exposure_steps);
        self.nav
            .release_before(&positions, step.saturating_sub(looking_back));
        Ok(())
    }

    /// Navigation step `step`: the camera, every set, the steering, and the
    /// records due.
    fn navigate(&mut self, step: usize) -> Result<(), ChainTrialError> {
        let t_s = self.nav.seconds(step);
        let instant = self
            .nav
            .log
            .instants
            .get(step)
            .copied()
            .unwrap_or(SimInstant::START);
        let truth = self
            .nav
            .truth
            .get(step)
            .map_or(self.chain.true_state(), |sample| sample.get().1);
        let camera_out = self
            .outages
            .iter()
            .any(|o| o.sensor == SensorKind::TerrainCamera && o.covers_up_to(t_s));
        if let Some(camera) = self.camera.as_mut()
            && let Some(request) = camera.expose(step, instant, &truth, camera_out)?
        {
            self.announced = Some(request);
        }
        if let Some(stars) = &self.stars {
            let step_s = sim_navigate::trial::step_span().as_seconds();
            let outages = &self.outages;
            let in_service = |open: usize| {
                !outages
                    .iter()
                    .any(|o| o.sensor == SensorKind::Stars && o.covers_up_to(open as f64 * step_s))
            };
            stars.sense(&self.nav.truth, step, &mut self.nav.log, in_service)?;
        }
        self.advance_sets(step)?;

        if let Some(slot) = self.steering.and_then(|index| self.sets.get_mut(index))
            && let Some(solution) = slot.run.solution(&self.nav.log)?
        {
            let solution = records::on_mission_clock(solution.get());
            self.chain.offer_navigation(&solution);
        }
        // Before the first step nothing was out, so an outage from the
        // start begins at the first step.
        let previous_s = match step {
            1 => f64::NEG_INFINITY,
            _ => self.nav.seconds(step.saturating_sub(1)),
        };
        self.pending.records.extend(records::availability(
            &self.sets,
            &self.outages,
            (previous_s, t_s),
        ));
        for slot in &mut self.sets {
            if let Some(fixes) = slot.changed_fixes() {
                self.pending.terrain_fixes.push((slot.name.clone(), fixes));
            }
        }
        if step.is_multiple_of(self.progress_steps) {
            // The flight controller's estimate is of the cycle's end, so it
            // meets the truth of that instant.
            let controller = self
                .chain
                .board()
                .estimated_state()
                .map(|estimate| (self.chain.true_state(), estimate));
            let samples = records::samples(
                (t_s, &truth),
                &mut self.sets,
                &self.nav.log,
                controller.as_ref(),
                &self.outages,
            )?;
            self.pending.records.extend(samples);
        }
        Ok(())
    }
}

fn seconds(at: SimInstant) -> f64 {
    at.duration_since(SimInstant::START).as_seconds()
}

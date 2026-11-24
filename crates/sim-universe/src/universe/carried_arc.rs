//! Predictor-corrector orbital arc preparation for carried articulated plants.
//!
//! When a carried plant has moving joints, its canonical internal momentum
//! `H = L - I_eff ω` evolves across an exchange interval `[t_0, t_N]`. A two-pass
//! preview runs the plant against an initial orbital arc, fits a linear ramp
//! `H(t)` on `sim-orbital` across `[t_0, t_N]`, and resamples the orbital base
//! trajectory under that continuous gyroscopic rate before advancing to the next
//! exchange.

use glam::DVec3;
use sim_core::{SimInstant, TickSpan, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_orbital::{ContinuousPlantState, DerivativeOrder, ExternalLoads, NaifId, VesselId};

use super::{Flight, Universe};
use crate::UniverseError;
use crate::handoff::to_body_fixed;

struct CarriedPreview {
    id: VesselId,
    body: BodyEllipsoid,
    plant_start: SimInstant,
    ramp_start: SimInstant,
    end: SimInstant,
    step: TickSpan,
}

impl Universe {
    /// Prepares linear internal-momentum ramps and step-sampled base trajectories
    /// for articulated carried plants entering or extending an exchange interval.
    pub(super) fn prepare_carried_intervals(&mut self) -> Result<(), UniverseError> {
        let pending = self.collect_carried_previews();
        for item in pending {
            self.prepare_one_carried_interval(&item)?;
        }
        Ok(())
    }

    fn collect_carried_previews(&self) -> Vec<CarriedPreview> {
        self.vessels
            .iter()
            .filter_map(|(&id, vessel)| {
                let Flight::Orbital {
                    carried: Some(plant),
                    samples,
                } = &vessel.flight
                else {
                    return None;
                };
                let step = plant.period();
                if !plant.has_moving_joints() || step.ticks_u128() == 0 {
                    return None;
                }
                let end = Self::exchange_instant(plant.as_ref())?;
                if end <= self.now {
                    return None;
                }
                let already_prepared = samples.last().is_some_and(|&(last_t, _)| last_t == end);
                (!already_prepared).then(|| CarriedPreview {
                    id,
                    body: plant.body(),
                    plant_start: plant.now(),
                    ramp_start: self.now,
                    end,
                    step,
                })
            })
            .collect()
    }

    fn prepare_one_carried_interval(&mut self, item: &CarriedPreview) -> Result<(), UniverseError> {
        let kept_prefix = self.take_kept_prefix(item.id, item.ramp_start)?;
        let future_grid = build_future_grid(item, &kept_prefix);
        let mut traj_0 = kept_prefix.clone();
        traj_0.extend(self.sample_carried_arc(item.id, item.body, &future_grid)?);
        if let Some(h_end) = self.dry_run_carried_momentum(item, traj_0)?
            && let Some(updated_loads) = self.ramped_loads_for(item, h_end)?
        {
            self.orbital
                .set_external_loads(item.id, updated_loads)
                .map_err(|e| UniverseError::Scheduler(Box::new(e)))?;
        }
        let mut traj_1 = kept_prefix;
        traj_1.extend(self.sample_carried_arc(item.id, item.body, &future_grid)?);
        if let Flight::Orbital { samples, .. } = &mut self.vessel_mut(item.id)?.flight {
            *samples = traj_1;
        }
        Ok(())
    }

    fn take_kept_prefix(
        &mut self,
        id: VesselId,
        now: SimInstant,
    ) -> Result<Vec<(SimInstant, VehicleState)>, UniverseError> {
        let Flight::Orbital { samples, .. } = &mut self.vessel_mut(id)?.flight else {
            return Ok(Vec::new());
        };
        let taken = std::mem::take(samples);
        Ok(taken.into_iter().filter(|&(t, _)| t <= now).collect())
    }

    fn dry_run_carried_momentum(
        &mut self,
        item: &CarriedPreview,
        traj: Vec<(SimInstant, VehicleState)>,
    ) -> Result<Option<DVec3>, UniverseError> {
        let Some(mut plant) = self.take_carried(item.id) else {
            return Ok(None);
        };
        let Some(snapshot) = plant.snapshot_internals() else {
            if let Flight::Orbital { carried: slot, .. } = &mut self.vessel_mut(item.id)?.flight {
                *slot = Some(plant);
            }
            return Ok(None);
        };
        let end_state = match traj.last() {
            Some((_, state)) => *state,
            None => self.carried_base_state(item.id, item.body)?,
        };
        let held_end = self.orbital.committed_checkpoint(item.id).map(|cp| {
            cp.external
                .loads
                .internal_momentum_at(self.epoch.tdb_at(item.end))
        });
        let carried_res = self
            .carry_context(item.body, end_state, item.end)
            .map(|ctx| {
                let ctx = ctx.with_trajectory(traj).with_preview_ramp_target();
                match held_end {
                    Some(h) => ctx.with_carried_ramp_end(h),
                    None => ctx,
                }
            })
            .and_then(|ctx| plant.carry(&ctx));
        let _ = plant.restore_internals(&snapshot);
        if let Flight::Orbital { carried: slot, .. } = &mut self.vessel_mut(item.id)?.flight {
            *slot = Some(plant);
        }
        carried_res.map(|loads| Some(loads.internal_momentum_body_nms))
    }

    fn ramped_loads_for(
        &self,
        item: &CarriedPreview,
        h_end: DVec3,
    ) -> Result<Option<ExternalLoads>, UniverseError> {
        let duration_s = item.end.duration_since(item.ramp_start).as_seconds();
        if duration_s <= 0.0 {
            return Ok(None);
        }
        let checkpoint = self
            .orbital
            .committed_checkpoint(item.id)
            .ok_or(UniverseError::UnknownVessel(item.id))?;
        let start_epoch = self.epoch.tdb_at(item.ramp_start);
        let end_epoch = self.epoch.tdb_at(item.end);
        let base_loads = checkpoint.external.loads;
        let h_start = base_loads.internal_momentum_at(start_epoch);
        let rate_body_nm = (h_end - h_start) / duration_s;
        Ok(Some(
            base_loads
                .with_internal_momentum(h_start)
                .with_momentum_ramp(rate_body_nm, start_epoch, end_epoch),
        ))
    }

    fn sample_carried_arc(
        &self,
        id: VesselId,
        body: BodyEllipsoid,
        grid: &[SimInstant],
    ) -> Result<Vec<(SimInstant, VehicleState)>, UniverseError> {
        let naif = self
            .bodies()
            .naif_of(&body)
            .ok_or(UniverseError::UnmodelledBody)?;
        let arc = self
            .orbital
            .sample_vessel_arc(id, grid)
            .map_err(|e| UniverseError::Scheduler(Box::new(e)))?;
        let mut samples = Vec::with_capacity(grid.len());
        for (&at, plant) in grid.iter().zip(arc) {
            let state = self.plant_state_to_body_fixed(body, naif, at, plant)?;
            samples.push((at, state));
        }
        Ok(samples)
    }

    pub(super) fn plant_state_to_body_fixed(
        &self,
        body: BodyEllipsoid,
        naif: NaifId,
        at: SimInstant,
        mut plant: ContinuousPlantState,
    ) -> Result<VehicleState, UniverseError> {
        if plant.primary_body != naif {
            let tdb = self.epoch.tdb_at(at);
            let current = self.ephemeris.geometric_state_icrf(
                plant.primary_body,
                NaifId::SSB,
                tdb,
                DerivativeOrder::Velocity,
            )?;
            let target = self.ephemeris.geometric_state_icrf(
                naif,
                NaifId::SSB,
                tdb,
                DerivativeOrder::Velocity,
            )?;
            plant.primary_body = naif;
            plant.pos_rel_icrf_m += current.pos_m - target.pos_m;
            plant.vel_rel_icrf_m_s += current.vel_m_s - target.vel_m_s;
        }
        let (_, rotation) = self.rotation(body, at)?;
        Ok(to_body_fixed(&plant, naif, &rotation)?.0)
    }
}

fn build_future_grid(
    item: &CarriedPreview,
    kept_prefix: &[(SimInstant, VehicleState)],
) -> Vec<SimInstant> {
    let mut full = vec![item.plant_start];
    let mut t = item.plant_start;
    while t.advanced(item.step) <= item.end {
        t = t.advanced(item.step);
        full.push(t);
    }
    if t < item.end {
        full.push(item.end);
    }
    full.into_iter()
        .filter(|inst| {
            *inst >= item.ramp_start && kept_prefix.binary_search_by_key(inst, |(k, _)| *k).is_err()
        })
        .collect()
}

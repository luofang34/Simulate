//! How far each vessel speculates in one proposal.
//!
//! Every proposal ends on the quantum grid, so where segments end never depends
//! on how a host slices time. A vessel whose integrator would step further than
//! one quantum ends its proposal on the first grid point its carried step reaches
//! instead of the next one, so a quiescent deep-space vessel takes its natural
//! steps rather than one clipped step per quantum. The horizon depends only on the
//! vessel's frontier checkpoint (its carried step is part of it) and the others'
//! frontiers, so it is the same however the host advances. Everything that is decided
//! at segment ends keeps a short cadence where it matters:
//!
//! - sphere-of-influence switching is detected at segment ends, so a vessel
//!   switching automatically speculates no further than it could fly towards
//!   the nearest switching radius;
//! - reachability tubes are built per segment, so a vessel near another vessel
//!   keeps one-quantum segments;
//! - only full 6-DoF integration takes long proposals; contact, rest, Kepler and
//!   reduced-manifold regimes keep the grid.
//!
//! Commands and events inside a long segment still land at their instants: the
//! scheduler replays the containing step up to the event instant.

use glam::DVec3;
use sim_spacetime::{SimInstant, TICKS_PER_SECOND, TickSpan};

use crate::{
    checkpoint::{PropagatorKind, VesselCheckpoint},
    ephemeris::{DerivativeOrder, EphemerisCatalog},
    time::TdbEpoch,
    universe::{
        OrbitalUniverse, VesselTimeline,
        worker::{MAX_PROPOSAL_CHUNK_SECONDS, soi_switch_radii_m},
    },
};

/// Vessels closer than this (`m`), plus what their relative speed closes in one
/// longest proposal, keep one-quantum segments so reachability tubes stay tight.
pub const PROXIMITY_GUARD_M: f64 = 100_000.0;

/// Share of the straight-line time to the nearest switching radius that a long
/// proposal may span; the rest absorbs the vessel speeding up on the way.
const SOI_LEAD_SHARE: f64 = 0.5;

impl OrbitalUniverse {
    /// Where `timeline`'s next proposal ends: the next grid point, or for a vessel
    /// free to take its natural steps the first grid point at or after its carried
    /// trial step, never past the last grid point within
    /// [`MAX_PROPOSAL_CHUNK_SECONDS`] (or the sphere-of-influence lead).
    pub(crate) fn speculation_horizon(&self, timeline: &VesselTimeline) -> SimInstant {
        let from = timeline.frontier_instant();
        let next_grid = self.next_canonical_horizon(from);
        let frontier = &timeline.frontier_checkpoint;
        if frontier.integrator_hint.propagator != PropagatorKind::FullCoupled6Dof
            || self.near_another_vessel(frontier)
        {
            return next_grid;
        }
        let Some(step_s) = frontier.integrator_hint.h_trial_s else {
            return next_grid;
        };
        let mut limit_s = MAX_PROPOSAL_CHUNK_SECONDS as f64;
        if timeline.config.auto_soi_switch {
            let epoch = self.clock.instant_to_tdb(from);
            limit_s = limit_s.min(soi_lead_s(frontier, epoch, self.ephemeris.as_ref()));
        }
        if !(step_s.is_finite() && limit_s.is_finite()) || step_s <= 0.0 || limit_s <= 0.0 {
            return next_grid;
        }
        let q = self.horizon_quantum_ticks.max(1);
        let ticks_after = |seconds: f64| {
            from.ticks_u128()
                .saturating_add((seconds * TICKS_PER_SECOND as f64) as u128)
        };
        let step_end = ticks_after(step_s).div_ceil(q).saturating_mul(q);
        let limit = (ticks_after(limit_s) / q).saturating_mul(q);
        let end = step_end.min(limit);
        SimInstant::START
            .advanced(TickSpan::from_ticks_u128(end))
            .max(next_grid)
    }

    fn near_another_vessel(&self, cp: &VesselCheckpoint) -> bool {
        let chunk_s = MAX_PROPOSAL_CHUNK_SECONDS as f64;
        self.vessels.values().any(|other| {
            let o = &other.frontier_checkpoint;
            if o.vessel_id == cp.vessel_id || o.plant.primary_body != cp.plant.primary_body {
                return false;
            }
            let apart = (o.plant.pos_rel_icrf_m - cp.plant.pos_rel_icrf_m).length();
            let closing = (o.plant.vel_rel_icrf_m_s - cp.plant.vel_rel_icrf_m_s).length();
            apart < PROXIMITY_GUARD_M + closing * chunk_s
        })
    }
}

/// Shortest time (`s`) in which `cp`, flying straight at its speed relative to
/// each body whose switching radius applies to its primary, could reach that
/// radius, scaled by [`SOI_LEAD_SHARE`]; infinite when no radius applies, and zero
/// (keep the grid) when a body cannot be located.
fn soi_lead_s(cp: &VesselCheckpoint, epoch: TdbEpoch, ephemeris: &dyn EphemerisCatalog) -> f64 {
    let primary = cp.plant.primary_body;
    let mut lead = f64::INFINITY;
    for (body, radius_m) in soi_switch_radii_m(primary) {
        let (offset, rel_vel) = if body == primary {
            (cp.plant.pos_rel_icrf_m, cp.plant.vel_rel_icrf_m_s)
        } else {
            let Ok(st) =
                ephemeris.geometric_state_icrf(body, primary, epoch, DerivativeOrder::Velocity)
            else {
                return 0.0;
            };
            (
                cp.plant.pos_rel_icrf_m - st.pos_m,
                cp.plant.vel_rel_icrf_m_s - st.vel_m_s,
            )
        };
        lead = lead.min(time_to_radius_s(offset, rel_vel, radius_m));
    }
    lead * SOI_LEAD_SHARE
}

fn time_to_radius_s(offset: DVec3, rel_vel: DVec3, radius_m: f64) -> f64 {
    let gap = (offset.length() - radius_m).abs();
    let speed = rel_vel.length();
    if speed > 0.0 {
        gap / speed
    } else {
        f64::INFINITY
    }
}

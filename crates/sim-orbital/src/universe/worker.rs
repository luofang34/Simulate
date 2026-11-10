//! Pure, side-effect-free speculative trajectory worker proposals.
//!
//! Workers consume immutable [`VesselCheckpoint`]s and return [`SpeculativeProposal`]s
//! without mutating shared universe state or advancing discrete GNC state inside RK stages.

use serde::{Deserialize, Serialize};
use sim_spacetime::{SimInstant, TickSpan};

use crate::{
    checkpoint::{PropagatorKind, VesselCheckpoint, VesselConfig, sim_instant_serde},
    dense_segment::PlantTrajectorySegment,
    dop853::{Dop853Stats, propagate_plant_interval},
    ephemeris::{DerivativeOrder, EphemerisCatalog},
    events::{EventOrderKey, ScheduledEvent, UniverseEventPayload, phase_ranks},
    gravity::ForceModel,
    ids::{NaifId, VesselId},
    orientation::BodyOrientationCatalog,
    time::{SimClockAdapter, TdbEpoch},
    universe::SchedulerError,
};

/// Hard cap on speculative lookahead segments buffered per vessel (`O(log S)` rollback bound).
pub const MAX_SPECULATIVE_SEGMENTS: usize = 64;

/// Maximum continuous segment-local propagation chunk (`<= 3,600 s <= 86,400 s` sub-ulp limit).
pub const MAX_PROPOSAL_CHUNK_SECONDS: u64 = 3_600;

const DETECTOR_DISCRETE_GNC: u32 = 1;
const DETECTOR_SOI_SWITCH: u32 = 2;

const MOON_SOI_RADIUS_M: f64 = 66_183_000.0;
const EARTH_SOI_RADIUS_M: f64 = 924_600_000.0;
const MARS_SOI_RADIUS_M: f64 = 577_200_000.0;
const SOI_ENTRY_FACTOR: f64 = 0.95;
const SOI_EXIT_FACTOR: f64 = 1.05;

/// One accepted continuous trajectory segment paired with exact start/end checkpoints
/// so rollback to any interior instant replays at most one single integration step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpeculativeSegmentEntry {
    /// Segment start timestamp (`== start_checkpoint.instant`).
    #[serde(with = "sim_instant_serde")]
    pub start_instant: SimInstant,
    /// Segment end timestamp (`== end_checkpoint.instant`).
    #[serde(with = "sim_instant_serde")]
    pub end_instant: SimInstant,
    /// Degree-7 `CONTD8` dense-output trajectory polynomial over `[start_instant, end_instant]`.
    pub segment: PlantTrajectorySegment,
    /// Complete checkpoint at `start_instant` for single-step interior rollback.
    pub start_checkpoint: VesselCheckpoint,
    /// Complete checkpoint at `end_instant`.
    pub end_checkpoint: VesselCheckpoint,
}

/// Pure worker proposal returned to [`crate::OrbitalUniverse`] for deterministic commit/ingest.
#[derive(Clone, Debug, PartialEq)]
pub struct SpeculativeProposal {
    /// Target vessel identifier.
    pub vessel_id: VesselId,
    /// `state_gen` of the base checkpoint from which this proposal was computed.
    pub base_state_gen: u64,
    /// Proposal start timestamp.
    pub from_instant: SimInstant,
    /// Latest timestamp covered by this proposal.
    pub through_instant: SimInstant,
    /// Accepted dense segments (at most `MAX_SPECULATIVE_SEGMENTS - existing_len`).
    pub entries: Vec<SpeculativeSegmentEntry>,
    /// Candidate events discovered at segment/GNC boundaries.
    pub proposed_events: Vec<ScheduledEvent>,
    /// Updated checkpoint at `through_instant` (including emitted sequence numbers).
    pub end_checkpoint: VesselCheckpoint,
    /// Integrator statistics accumulated across this proposal.
    pub stats: Dop853Stats,
}

/// Computes a pure [`SpeculativeProposal`] from `base_checkpoint` toward `horizon_target`
/// without mutating any shared state.
#[allow(clippy::too_many_arguments)]
pub fn propose_vessel_horizon(
    base_checkpoint: &VesselCheckpoint,
    config: &VesselConfig,
    horizon_target: SimInstant,
    existing_speculative_len: usize,
    clock: &SimClockAdapter,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<SpeculativeProposal, SchedulerError> {
    let vid = base_checkpoint.vessel_id;
    let remaining_slots = MAX_SPECULATIVE_SEGMENTS.saturating_sub(existing_speculative_len);
    if horizon_target <= base_checkpoint.instant || remaining_slots == 0 {
        return Ok(SpeculativeProposal {
            vessel_id: vid,
            base_state_gen: base_checkpoint.generations.state_gen,
            from_instant: base_checkpoint.instant,
            through_instant: base_checkpoint.instant,
            entries: Vec::new(),
            proposed_events: Vec::new(),
            end_checkpoint: base_checkpoint.clone(),
            stats: Dop853Stats::default(),
        });
    }

    let capped_end = clamp_proposal_end(base_checkpoint, horizon_target);
    if base_checkpoint.integrator_hint.propagator == PropagatorKind::PinnedSurfaceRest {
        return build_pinned_rest_proposal(base_checkpoint, capped_end, clock, orientation);
    }
    if base_checkpoint.integrator_hint.propagator == PropagatorKind::DormantKepler {
        return build_dormant_kepler_proposal(
            base_checkpoint,
            config,
            capped_end,
            clock,
            force_model,
            ephemeris,
        );
    }

    let start_epoch = clock.instant_to_tdb(base_checkpoint.instant);
    let end_epoch = clock.instant_to_tdb(capped_end);
    let mut chunk_tol = config.tolerances;
    chunk_tol.max_steps = chunk_tol
        .max_steps
        .min((remaining_slots as u32).saturating_mul(64).max(64));

    let out = propagate_checkpoint_interval(
        base_checkpoint,
        start_epoch,
        end_epoch,
        base_checkpoint.integrator_hint.h_trial_s,
        &chunk_tol,
        config,
        force_model,
        ephemeris,
        orientation,
    )?;

    package_segments_and_events(
        base_checkpoint,
        config,
        capped_end,
        remaining_slots,
        clock,
        ephemeris,
        out,
    )
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn propagate_checkpoint_interval(
    checkpoint: &VesselCheckpoint,
    start_epoch: TdbEpoch,
    end_epoch: TdbEpoch,
    h_init_s: Option<f64>,
    tolerances: &crate::dop853::Dop853Tolerances,
    config: &VesselConfig,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<crate::dop853::PlantPropagationOutput, SchedulerError> {
    let flux = sim_spacetime::SolarFluxModel::standard();
    let srp_law =
        checkpoint
            .gnc
            .srp_surface
            .map(|surface| crate::plant::SolarRadiationPressureLaw {
                inner: &checkpoint.gnc.latched_law,
                surface,
                flux: &flux,
                ephemeris,
            });
    let control: &dyn crate::plant::ContinuousControlLaw = match &srp_law {
        Some(law) => law,
        None => &checkpoint.gnc.latched_law,
    };
    propagate_plant_interval(
        &checkpoint.plant,
        start_epoch,
        end_epoch,
        h_init_s,
        tolerances,
        &config.mass_model,
        config.propellant_mode,
        config.rw_mode,
        force_model,
        ephemeris,
        orientation,
        control,
    )
    .map_err(|source| SchedulerError::Propagation {
        vessel_id: checkpoint.vessel_id,
        source,
    })
}

fn clamp_proposal_end(base: &VesselCheckpoint, horizon_target: SimInstant) -> SimInstant {
    let max_chunk = base
        .instant
        .advanced(TickSpan::from_seconds_and_subsec_ticks(
            MAX_PROPOSAL_CHUNK_SECONDS,
            0,
        ));
    let mut end = horizon_target.min(max_chunk);
    if let Some(gnc_ticks) = base.gnc.next_gnc_ticks {
        let gnc_inst = SimInstant::START.advanced(TickSpan::from_ticks_u128(gnc_ticks));
        if gnc_inst > base.instant && gnc_inst < end {
            end = gnc_inst;
        }
    }
    end
}

fn build_pinned_rest_proposal(
    base: &VesselCheckpoint,
    end_instant: SimInstant,
    clock: &SimClockAdapter,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<SpeculativeProposal, SchedulerError> {
    let mut end_cp = base
        .at_pinned_surface_instant(end_instant, clock, orientation)
        .map_err(|source| SchedulerError::SurfaceRest {
            vessel_id: base.vessel_id,
            source,
        })?;
    let mut events = Vec::new();
    maybe_emit_gnc_boundary_event(&mut end_cp, end_instant, &mut events);
    Ok(SpeculativeProposal {
        vessel_id: base.vessel_id,
        base_state_gen: base.generations.state_gen,
        from_instant: base.instant,
        through_instant: end_instant,
        entries: Vec::new(),
        proposed_events: events,
        end_checkpoint: end_cp,
        stats: Dop853Stats::default(),
    })
}

fn build_dormant_kepler_proposal(
    base: &VesselCheckpoint,
    config: &VesselConfig,
    end_instant: SimInstant,
    clock: &SimClockAdapter,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
) -> Result<SpeculativeProposal, SchedulerError> {
    let start_epoch = clock.instant_to_tdb(base.instant);
    let end_epoch = clock.instant_to_tdb(end_instant);
    let dt_s = end_epoch.signed_offset_seconds_from(start_epoch).max(0.0);
    let mu_p = force_model
        .constants()
        .require_mu(base.plant.primary_body)
        .unwrap_or(3.986_004_418e14);
    let (r_end, v_end) = crate::kepler::propagate_kepler_universal(
        base.plant.pos_rel_icrf_m,
        base.plant.vel_rel_icrf_m_s,
        dt_s,
        mu_p,
    );
    let mut end_plant = base.plant;
    end_plant.pos_rel_icrf_m = r_end;
    end_plant.vel_rel_icrf_m_s = v_end;
    let mut end_cp = base.advanced_continuous(
        end_instant,
        end_plant,
        dt_s.max(1.0),
        crate::plant::PropagationAudit::default(),
    );
    let events = collect_boundary_events(&mut end_cp, config, end_instant, clock, ephemeris);
    Ok(SpeculativeProposal {
        vessel_id: base.vessel_id,
        base_state_gen: base.generations.state_gen,
        from_instant: base.instant,
        through_instant: end_instant,
        entries: Vec::new(),
        proposed_events: events,
        end_checkpoint: end_cp,
        stats: Dop853Stats {
            accepted_steps: 1,
            rejected_steps: 0,
            rhs_evals: 1,
            next_h_hint_s: dt_s.max(1.0),
        },
    })
}

fn package_segments_and_events(
    base: &VesselCheckpoint,
    config: &VesselConfig,
    requested_end: SimInstant,
    remaining_slots: usize,
    clock: &SimClockAdapter,
    ephemeris: &dyn EphemerisCatalog,
    out: crate::dop853::PlantPropagationOutput,
) -> Result<SpeculativeProposal, SchedulerError> {
    let take_count = out.segments.len().min(remaining_slots);
    let mut entries = Vec::with_capacity(take_count);
    let mut events = Vec::new();
    let mut cursor_cp = base.clone();

    for seg in out.segments.into_iter().take(take_count) {
        let seg_end_inst = cursor_cp
            .instant
            .advanced(TickSpan::from_ticks_u128(seg.duration_ticks))
            .min(requested_end);
        let mut next_cp = cursor_cp.advanced_continuous(
            seg_end_inst,
            seg.end_state,
            seg.next_h_hint_s,
            seg.audit,
        );
        let seg_events =
            collect_boundary_events(&mut next_cp, config, seg_end_inst, clock, ephemeris);
        let stop_here = !seg_events.is_empty();
        events.extend(seg_events);
        entries.push(SpeculativeSegmentEntry {
            start_instant: cursor_cp.instant,
            end_instant: seg_end_inst,
            segment: seg,
            start_checkpoint: cursor_cp,
            end_checkpoint: next_cp.clone(),
        });
        cursor_cp = next_cp;
        if stop_here {
            break;
        }
    }

    let through_instant = cursor_cp.instant;
    Ok(SpeculativeProposal {
        vessel_id: base.vessel_id,
        base_state_gen: base.generations.state_gen,
        from_instant: base.instant,
        through_instant,
        entries,
        proposed_events: events,
        end_checkpoint: cursor_cp,
        stats: out.stats,
    })
}

fn collect_boundary_events(
    cursor_cp: &mut VesselCheckpoint,
    config: &VesselConfig,
    through_instant: SimInstant,
    clock: &SimClockAdapter,
    ephemeris: &dyn EphemerisCatalog,
) -> Vec<ScheduledEvent> {
    let mut events = Vec::new();
    maybe_emit_gnc_boundary_event(cursor_cp, through_instant, &mut events);
    if config.auto_soi_switch {
        let ep = clock.instant_to_tdb(through_instant);
        if let Some(target_body) = detect_soi_switch_target(cursor_cp, ep, ephemeris) {
            let seq = cursor_cp.generations.next_emit_seq();
            let key = EventOrderKey::single(
                through_instant,
                phase_ranks::PRIMARY_FRAME_SWITCH,
                cursor_cp.vessel_id,
                DETECTOR_SOI_SWITCH,
                seq,
            );
            events.push(ScheduledEvent::single(
                key,
                cursor_cp.vessel_id,
                Some(cursor_cp.generations.state_gen),
                UniverseEventPayload::SwitchPrimaryBody {
                    new_primary: target_body,
                },
            ));
        }
    }
    events
}

fn maybe_emit_gnc_boundary_event(
    cp: &mut VesselCheckpoint,
    through_instant: SimInstant,
    events: &mut Vec<ScheduledEvent>,
) {
    if let Some(gnc_ticks) = cp.gnc.next_gnc_ticks
        && through_instant.ticks_u128() >= gnc_ticks
    {
        let seq = cp.generations.next_emit_seq();
        let inst = SimInstant::START.advanced(TickSpan::from_ticks_u128(gnc_ticks));
        let key = EventOrderKey::single(
            inst,
            phase_ranks::GNC_DISCRETE_UPDATE,
            cp.vessel_id,
            DETECTOR_DISCRETE_GNC,
            seq,
        );
        events.push(ScheduledEvent::single(
            key,
            cp.vessel_id,
            Some(cp.generations.state_gen),
            UniverseEventPayload::DiscreteGncUpdate,
        ));
    }
}

fn detect_soi_switch_target(
    cp: &VesselCheckpoint,
    epoch: TdbEpoch,
    ephemeris: &dyn EphemerisCatalog,
) -> Option<NaifId> {
    let p = cp.plant.primary_body;
    let r_norm = cp.plant.pos_rel_icrf_m.length();
    if p == NaifId::EARTH {
        if let Ok(moon_rel) = ephemeris.geometric_state_icrf(
            NaifId::MOON,
            NaifId::EARTH,
            epoch,
            DerivativeOrder::Position,
        ) && (cp.plant.pos_rel_icrf_m - moon_rel.pos_m).length()
            < MOON_SOI_RADIUS_M * SOI_ENTRY_FACTOR
        {
            return Some(NaifId::MOON);
        }
        if r_norm > EARTH_SOI_RADIUS_M * SOI_EXIT_FACTOR {
            return Some(NaifId::SSB);
        }
    } else if p == NaifId::MOON {
        if r_norm > MOON_SOI_RADIUS_M * SOI_EXIT_FACTOR {
            return Some(NaifId::EARTH);
        }
    } else if p == NaifId::MARS {
        if r_norm > MARS_SOI_RADIUS_M * SOI_EXIT_FACTOR {
            return Some(NaifId::SSB);
        }
    } else if p == NaifId::SSB {
        if let Ok(earth_st) = ephemeris.geometric_state_icrf(
            NaifId::EARTH,
            NaifId::SSB,
            epoch,
            DerivativeOrder::Position,
        ) && (cp.plant.pos_rel_icrf_m - earth_st.pos_m).length()
            < EARTH_SOI_RADIUS_M * SOI_ENTRY_FACTOR
        {
            return Some(NaifId::EARTH);
        }
        if let Ok(mars_st) = ephemeris.geometric_state_icrf(
            NaifId::MARS,
            NaifId::SSB,
            epoch,
            DerivativeOrder::Position,
        ) && (cp.plant.pos_rel_icrf_m - mars_st.pos_m).length()
            < MARS_SOI_RADIUS_M * SOI_ENTRY_FACTOR
        {
            return Some(NaifId::MARS);
        }
    }
    None
}

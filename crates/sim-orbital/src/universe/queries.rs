//! Read-only render, telemetry, and inspection queries on [`crate::OrbitalUniverse`].

use sim_spacetime::SimInstant;

use crate::{
    checkpoint::VesselCheckpoint,
    collision::{
        RendezvousGroup, SweptReachabilityTube, TubeUncertaintyBudget, form_rendezvous_groups,
    },
    ephemeris::KinematicStateIcrf,
    events::{AuthoritativeCommandLog, EventOrderKey},
    ids::VesselId,
    manifest::SimulationManifestDigest,
    plant::ContinuousPlantState,
    time::SimClockAdapter,
    universe::{OrbitalUniverse, SchedulerError},
};

impl OrbitalUniverse {
    /// Authoritative global committed timestamp.
    #[must_use]
    pub const fn committed_instant(&self) -> SimInstant {
        self.committed_instant
    }

    /// Active model manifest digest.
    #[must_use]
    pub const fn manifest(&self) -> &SimulationManifestDigest {
        &self.manifest
    }

    /// Simulation clock (`SimInstant <-> TdbEpoch`) adapter.
    #[must_use]
    pub const fn clock(&self) -> &SimClockAdapter {
        &self.clock
    }

    /// Active camera focus vessel (pure view state; never alters simulation trajectories).
    #[must_use]
    pub const fn active_camera_vessel(&self) -> Option<VesselId> {
        self.active_camera_vessel
    }

    /// Selects or clears the active camera focus vessel without mutating any physics state.
    pub fn set_active_camera_vessel(&mut self, vessel_id: Option<VesselId>) {
        self.active_camera_vessel = vessel_id;
    }

    /// Registered vessel IDs in canonical ascending order.
    pub fn vessel_ids(&self) -> impl Iterator<Item = VesselId> + '_ {
        self.vessels.keys().copied()
    }

    /// Last completed step-boundary checkpoint for `vessel_id`.
    #[must_use]
    pub fn committed_checkpoint(&self, vessel_id: VesselId) -> Option<&VesselCheckpoint> {
        self.vessels
            .get(&vessel_id)
            .map(|v| &v.committed_checkpoint)
    }

    /// Number of buffered speculative segments currently held for `vessel_id`.
    #[must_use]
    pub fn speculative_segment_count(&self, vessel_id: VesselId) -> Option<usize> {
        self.vessels.get(&vessel_id).map(|v| v.speculative.len())
    }

    /// Pure `&self` render/telemetry sample at `render_instant` (never perturbs step boundaries or RNG).
    #[must_use]
    pub fn sample_vessel_state(
        &self,
        vessel_id: VesselId,
        render_instant: SimInstant,
    ) -> Option<ContinuousPlantState> {
        self.vessels.get(&vessel_id).map(|v| {
            v.sample_plant_and_accel(render_instant, &self.clock, self.orientation.as_ref())
                .0
        })
    }

    /// Pure `&self` barycentric (`SSB / ICRF`) position, velocity, and acceleration sample.
    pub fn sample_vessel_barycentric(
        &self,
        vessel_id: VesselId,
        render_instant: SimInstant,
    ) -> Result<KinematicStateIcrf, SchedulerError> {
        let tl = self
            .vessels
            .get(&vessel_id)
            .ok_or(SchedulerError::UnknownVessel(vessel_id))?;
        let (plant, acc_rel) =
            tl.sample_plant_and_accel(render_instant, &self.clock, self.orientation.as_ref());
        let ep = self.clock.instant_to_tdb(render_instant);
        self.force_model
            .reconstruct_barycentric_state(
                plant.primary_body,
                plant.pos_rel_icrf_m,
                plant.vel_rel_icrf_m_s,
                acc_rel,
                ep,
                self.ephemeris.as_ref(),
            )
            .map_err(|source| SchedulerError::FrameSwitch { vessel_id, source })
    }

    /// Chronological log of all `EventOrderKey`s executed since creation or snapshot restore.
    #[must_use]
    pub fn fired_events(&self) -> &[EventOrderKey] {
        &self.fired_events_log
    }

    /// Authoritative player command log.
    #[must_use]
    pub const fn command_log(&self) -> &AuthoritativeCommandLog {
        &self.command_log
    }

    /// Builds conservative [`SweptReachabilityTube`] envelopes across all buffered speculative segments.
    #[must_use]
    pub fn speculative_reachability_tubes(
        &self,
        rigid_radius_m: f64,
        max_control_accel_delta_m_s2: f64,
    ) -> Vec<SweptReachabilityTube> {
        let mut tubes = Vec::new();
        for (&vid, tl) in &self.vessels {
            for entry in &tl.speculative {
                let mu = self
                    .force_model
                    .constants()
                    .get(entry.segment.primary_body)
                    .map_or(0.0, |b| b.mu_m3_s2);
                let budget = TubeUncertaintyBudget {
                    rigid_radius_m,
                    interp_error_m: tl.config.tolerances.atol_pos_m + 1.0e-3,
                    primary_mu_m3_s2: mu,
                    max_control_accel_delta_m_s2,
                };
                tubes.push(SweptReachabilityTube::from_plant_segment(
                    vid,
                    entry.start_instant,
                    entry.end_instant,
                    &entry.segment,
                    &budget,
                ));
            }
        }
        tubes
    }

    /// Deterministically forms [`RendezvousGroup`]s from overlapping speculative reachability envelopes.
    #[must_use]
    pub fn active_rendezvous_groups(
        &self,
        rigid_radius_m: f64,
        max_control_accel_delta_m_s2: f64,
    ) -> Vec<RendezvousGroup> {
        let tubes =
            self.speculative_reachability_tubes(rigid_radius_m, max_control_accel_delta_m_s2);
        form_rendezvous_groups(&tubes)
    }
}

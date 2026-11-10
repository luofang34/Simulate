//! Self-contained, hashable per-vessel transactional checkpoints and pure latched steering laws.
//!
//! A [`VesselCheckpoint`] captures every piece of state needed to evolve a vessel forward
//! bit-identically without mutating shared controller or RNG state inside rejected RK stages.

pub mod burn_law;

use serde::{Deserialize, Serialize};
pub(crate) use sim_spacetime::sim_instant_serde;
use sim_spacetime::{SimInstant, TickSpan};

use crate::{
    contact::PinnedSurfaceState,
    dop853::Dop853Tolerances,
    gravity::GravityError,
    hash::{ContentHash, DomainHasher},
    ids::VesselId,
    orientation::BodyOrientationCatalog,
    plant::{
        ContinuousPlantState, MassPropertiesModel, PropagationAudit, PropellantMode,
        ReactionWheelMode, SrpSurface,
    },
    time::SimClockAdapter,
};

pub use burn_law::{ActuatorState, LatchedBurnLaw};

/// Explicit per-vessel `Xoshiro256**` deterministic PRNG state (zero thread-local or OS RNG).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct DeterministicRngState {
    /// Four 64-bit state words (guaranteed not all zero via `SplitMix64` seeding).
    pub state: [u64; 4],
}

impl DeterministicRngState {
    /// Seeds a non-degenerate 256-bit `Xoshiro256**` state from `seed` via `SplitMix64`.
    #[must_use]
    pub const fn from_seed(seed: u64) -> Self {
        let (s0, z0) = splitmix64(seed);
        let (s1, z1) = splitmix64(s0);
        let (s2, z2) = splitmix64(s1);
        let (_, z3) = splitmix64(s2);
        Self {
            state: [z0, z1, z2, z3],
        }
    }

    /// Advances the generator and returns the next 64-bit pseudorandom word.
    pub fn next_u64(&mut self) -> u64 {
        let [mut s0, mut s1, mut s2, mut s3] = self.state;
        let result = s1.wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = s1 << 17;
        s2 ^= s0;
        s3 ^= s1;
        s1 ^= s2;
        s0 ^= s3;
        s2 ^= t;
        s3 = s3.rotate_left(45);
        self.state = [s0, s1, s2, s3];
        result
    }

    /// Draws a uniform `f64` in `[0.0, 1.0)` using the top 53 bits of [`Self::next_u64`].
    pub fn next_f64_unit(&mut self) -> f64 {
        let bits53 = self.next_u64() >> 11;
        (bits53 as f64) * (1.0 / ((1_u64 << 53) as f64))
    }

    fn write_to_hasher(&self, h: &mut DomainHasher) {
        for &w in &self.state {
            h.write_u64(w);
        }
    }
}

const fn splitmix64(state: u64) -> (u64, u64) {
    let next = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
    let mut z = (next ^ (next >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    (next, z ^ (z >> 31))
}

/// Monotonic generation and sequence counters invalidating stale queued events.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct EventGenerationState {
    /// Increments on every discrete plant/burn/frame/contact mutation (`invalidates` stale events).
    pub state_gen: u64,
    /// Increments on every discrete GNC computer update.
    pub gnc_gen: u64,
    /// Monotonic per-vessel sequence counter stamping [`crate::EventOrderKey::sequence_no`].
    pub emit_seq: u64,
}

impl EventGenerationState {
    /// Bumps [`Self::state_gen`] via wrapping addition.
    pub fn bump_state_gen(&mut self) {
        self.state_gen = self.state_gen.wrapping_add(1);
    }

    /// Bumps [`Self::gnc_gen`] via wrapping addition.
    pub fn bump_gnc_gen(&mut self) {
        self.gnc_gen = self.gnc_gen.wrapping_add(1);
    }

    /// Allocates and returns the next monotonic event emission sequence number.
    pub fn next_emit_seq(&mut self) -> u64 {
        let seq = self.emit_seq;
        self.emit_seq = self.emit_seq.wrapping_add(1);
        seq
    }

    pub(crate) fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_u64(self.state_gen);
        h.write_u64(self.gnc_gen);
        h.write_u64(self.emit_seq);
    }
}

/// Active continuous or contact propagation regime for a vessel.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PropagatorKind {
    /// Full 17-state coupled 6-DoF `DOP853` integration.
    #[default]
    FullCoupled6Dof,
    /// High-warp analytic attitude-manifold thrust propagation (`Gate 5`).
    ReducedManifoldThrust,
    /// Rectified Stumpff universal-variable Encke propagator (`Gate 5`).
    EnckeKepler,
    /// Unperturbed universal-variable Kepler segment cache (`Gate 5`).
    DormantKepler,
    /// Fixed-substep (`1 ms`) compliant ground contact with bristle stiction (`Gate 4`).
    ContactSubstep,
    /// Body-fixed static equilibrium rest (`Gate 4`).
    PinnedSurfaceRest,
}

impl PropagatorKind {
    pub(crate) const fn code(self) -> u8 {
        match self {
            Self::FullCoupled6Dof => 1,
            Self::ReducedManifoldThrust => 2,
            Self::EnckeKepler => 3,
            Self::DormantKepler => 4,
            Self::ContactSubstep => 5,
            Self::PinnedSurfaceRest => 6,
        }
    }
}

/// Integrator continuation metadata preserved across checkpoints for exact step resumption.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct IntegratorContinuation {
    /// Next trial step size $h_{\text{trial}}$ (`s`), or `None` to use configured `h_init_s`.
    pub h_trial_s: Option<f64>,
    /// Consecutive step-rejection / stiffness counter.
    pub stiffness_count: u32,
    /// Active propagation regime.
    pub propagator: PropagatorKind,
}

impl IntegratorContinuation {
    fn write_to_hasher(&self, h: &mut DomainHasher) {
        match self.h_trial_s {
            Some(v) => {
                h.write_u8(1);
                h.write_f64_canonical(v);
            }
            None => h.write_u8(0),
        }
        h.write_u32(self.stiffness_count);
        h.write_u8(self.propagator.code());
    }
}

/// Discrete GNC computer state updated strictly at transactional event boundaries.
///
/// Stores only immutable mission-plan version references (`active_plan_id`, `active_plan_version`)
/// and an execution cursor (`maneuver_cursor`), keeping mutable planning graphs and mission
/// runner execution states outside `DiscreteGncState`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DiscreteGncState {
    /// Active guidance mode identifier (`0 = standby/coast`).
    pub mode_code: u32,
    /// Optional identifier of the bound immutable mission plan intent.
    pub active_plan_id: Option<u64>,
    /// Monotonic version of the bound mission plan (`0` when unbound).
    pub active_plan_version: u64,
    /// Index of the next or currently active maneuver node in the vessel's bound plan.
    pub maneuver_cursor: u32,
    /// Discrete GNC update period in master-clock ticks (`0` = event-driven / analytic).
    pub gnc_period_ticks: u128,
    /// Next scheduled discrete GNC tick (`Some(ticks)` when periodic GNC is active).
    pub next_gnc_ticks: Option<u128>,
    /// Integrated attitude-error state in body axes for zero-steady-error SAS (`rad·s`).
    pub integral_att_error_body: [f64; 3],
    /// Pure continuous steering/burn law latched across continuous integration steps.
    pub latched_law: LatchedBurnLaw,
    /// Optional cannonball solar radiation pressure surface evaluated via [`crate::SolarRadiationPressureLaw`].
    #[serde(default)]
    pub srp_surface: Option<SrpSurface>,
}

impl DiscreteGncState {
    fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_u32(self.mode_code);
        match self.active_plan_id {
            Some(id) => {
                h.write_u8(1);
                h.write_u64(id);
            }
            None => h.write_u8(0),
        }
        h.write_u64(self.active_plan_version);
        h.write_u32(self.maneuver_cursor);
        h.write_u128(self.gnc_period_ticks);
        match self.next_gnc_ticks {
            Some(t) => {
                h.write_u8(1);
                h.write_u128(t);
            }
            None => h.write_u8(0),
        }
        for v in self.integral_att_error_body {
            h.write_f64_canonical(v);
        }
        self.latched_law.write_to_hasher(h);
        if let Some(srp) = self.srp_surface {
            h.write_u8(1);
            h.write_f64_canonical(srp.area_m2);
            h.write_f64_canonical(srp.reflectivity_coefficient);
        }
    }
}

/// Static mass, actuator-mode, tolerance, and SOI-switching configuration for one vessel.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct VesselConfig {
    /// Mass and inertia tensor model.
    pub mass_model: MassPropertiesModel,
    /// Propellant depletion semantics.
    pub propellant_mode: PropellantMode,
    /// Reaction-wheel momentum semantics.
    pub rw_mode: ReactionWheelMode,
    /// Adaptive `DOP853` tolerances and step limits.
    pub tolerances: Dop853Tolerances,
    /// Outer rigid-body bounding sphere radius $R_{\text{bound}}$ ($\text{m}$).
    pub bounding_radius_m: f64,
    /// Whether the scheduler proposes primary-body switches across Hill/SOI boundaries.
    pub auto_soi_switch: bool,
}

impl VesselConfig {
    /// Canonical SHA-256 content hash of this vessel's static configuration.
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-orbital:vessel-config:v1");
        self.write_to_hasher(&mut h);
        h.finish()
    }

    pub(crate) fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_f64_canonical(self.mass_model.dry_mass_kg);
        for col in self.mass_model.dry_inertia_cols {
            for v in col {
                h.write_f64_canonical(v);
            }
        }
        for col in self.mass_model.propellant_inertia_per_kg_cols {
            for v in col {
                h.write_f64_canonical(v);
            }
        }
        h.write_f64_canonical(self.mass_model.uncompensated_idot_factor);
        match self.propellant_mode {
            PropellantMode::FiniteVariableMass => h.write_u8(0),
            PropellantMode::InfiniteConstantMass => h.write_u8(1),
        }
        match self.rw_mode {
            ReactionWheelMode::ConserveMomentum {
                h_max_nms,
                tau_max_nm,
            } => {
                h.write_u8(0);
                h.write_f64_canonical(h_max_nms);
                h.write_f64_canonical(tau_max_nm);
            }
            ReactionWheelMode::ExternalMagicTorque { tau_max_nm } => {
                h.write_u8(1);
                h.write_f64_canonical(tau_max_nm);
            }
        }
        h.write_hash(self.tolerances.content_hash());
        h.write_f64_canonical(self.bounding_radius_m);
        h.write_u8(u8::from(self.auto_soi_switch));
    }
}

/// Complete, self-contained state required to reproduce future evolution of one vessel bit-identically.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VesselCheckpoint {
    /// Authoritative vessel identifier.
    pub vessel_id: VesselId,
    /// Exact simulation timestamp of this checkpoint.
    #[serde(with = "sim_instant_serde")]
    pub instant: SimInstant,
    /// Continuous translational, rotational, mass, and wheel-momentum state.
    pub plant: ContinuousPlantState,
    /// Discrete GNC computer state and latched continuous steering law.
    pub gnc: DiscreteGncState,
    /// Fin, nosecone, and throttle actuator states.
    pub actuators: ActuatorState,
    /// Explicit per-vessel deterministic PRNG state.
    pub rng: DeterministicRngState,
    /// Monotonic generation and sequence counters.
    pub generations: EventGenerationState,
    /// Integrator continuation hint (`h_trial_s`, stiffness count, propagator regime).
    pub integrator_hint: IntegratorContinuation,
    /// Body-fixed surface anchor when `integrator_hint.propagator == PropagatorKind::PinnedSurfaceRest`.
    #[serde(default)]
    pub pinned_surface: Option<PinnedSurfaceState>,
    /// Cumulative idealization/exceedance audit flags.
    pub audit: PropagationAudit,
}

impl VesselCheckpoint {
    /// Creates an initial checkpoint at `instant` seeded deterministically from `(vessel_id, seed)`.
    #[must_use]
    pub fn new(
        vessel_id: VesselId,
        instant: SimInstant,
        plant: ContinuousPlantState,
        seed: u64,
    ) -> Self {
        Self {
            vessel_id,
            instant,
            plant,
            gnc: DiscreteGncState::default(),
            actuators: ActuatorState::default(),
            rng: DeterministicRngState::from_seed(seed ^ vessel_id.0.rotate_left(19)),
            generations: EventGenerationState::default(),
            integrator_hint: IntegratorContinuation::default(),
            pinned_surface: None,
            audit: PropagationAudit::default(),
        }
    }

    /// Attaches a solar radiation pressure surface evaluated via [`crate::SolarRadiationPressureLaw`].
    #[must_use]
    pub const fn with_srp_surface(mut self, surface: SrpSurface) -> Self {
        self.gnc.srp_surface = Some(surface);
        self
    }

    fn write_physics_state_to_hasher(&self, h: &mut DomainHasher) {
        h.write_u64(self.vessel_id.0);
        h.write_u128(self.instant.ticks_u128());
        h.write_i32(self.plant.primary_body.0);
        for v in self.plant.to_ode_array() {
            h.write_f64_canonical(v);
        }
        self.actuators.write_to_hasher(h);
        match self.pinned_surface {
            Some(p) => {
                h.write_u8(1);
                h.write_i32(p.body.0);
                for v in p.pos_fixed_m {
                    h.write_f64_canonical(v);
                }
                for v in p.q_body_to_fixed_xyzw {
                    h.write_f64_canonical(v);
                }
            }
            None => h.write_u8(0),
        }
    }

    /// Canonical SHA-256 hash of instantaneous physical and actuator state only
    /// (`vessel_id`, `instant`, `plant`, `actuators`, and `pinned_surface`).
    #[must_use]
    pub fn physics_state_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-orbital:vessel-physics-state:v1");
        self.write_physics_state_to_hasher(&mut h);
        h.finish()
    }

    /// Canonical SHA-256 content hash of the full per-vessel replay checkpoint
    /// (covering physical state, GNC state, RNG, generations, integrator continuation, and audit).
    #[must_use]
    pub fn checkpoint_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-orbital:vessel-checkpoint:v2");
        self.write_physics_state_to_hasher(&mut h);
        self.gnc.write_to_hasher(&mut h);
        self.rng.write_to_hasher(&mut h);
        self.generations.write_to_hasher(&mut h);
        self.integrator_hint.write_to_hasher(&mut h);
        h.write_u8(u8::from(self.audit.mass_conservation_exempt));
        h.write_u8(u8::from(self.audit.angular_momentum_exempt));
        h.write_u8(u8::from(self.audit.proper_accel_exceedance));
        h.finish()
    }

    /// Returns a copy stamped at `instant` with updated continuous plant state, continuation hint, and audit.
    #[must_use]
    pub fn advanced_continuous(
        &self,
        instant: SimInstant,
        plant: ContinuousPlantState,
        next_h_hint_s: f64,
        step_audit: PropagationAudit,
    ) -> Self {
        let mut next = self.clone();
        next.instant = instant;
        next.plant = plant;
        next.integrator_hint.h_trial_s = Some(next_h_hint_s);
        next.audit = next.audit.merged(step_audit);
        next
    }

    /// Resolves and latches the body-fixed [`PinnedSurfaceState`] and evaluates the co-rotating
    /// `ICRF` plant state at `target_instant`.
    pub fn at_pinned_surface_instant(
        &self,
        target_instant: SimInstant,
        clock: &SimClockAdapter,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<Self, GravityError> {
        let pinned = match self.pinned_surface {
            Some(p) => p,
            None => PinnedSurfaceState::from_icrf_plant_state(
                &self.plant,
                clock.instant_to_tdb(self.instant),
                orientation,
            )?,
        };
        let target_epoch = clock.instant_to_tdb(target_instant);
        let new_plant = pinned.to_icrf_plant_state(
            target_epoch,
            self.plant.propellant_mass_kg,
            self.plant.h_rw_body_nms,
            orientation,
        )?;
        let mut next = self.clone();
        next.instant = target_instant;
        next.plant = new_plant;
        next.pinned_surface = Some(pinned);
        Ok(next)
    }

    /// Advances `instant` by `span` without modifying plant or discrete states.
    #[must_use]
    pub fn at_advanced_span(&self, span: TickSpan) -> Self {
        let mut next = self.clone();
        next.instant = self.instant.advanced(span);
        next
    }
}

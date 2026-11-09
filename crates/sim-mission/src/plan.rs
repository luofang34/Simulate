//! Mission plan representation (`MissionPlan`, `MissionSegmentNode`, `SegmentBurnLaw`,
//! `SegmentStoppingCondition`, `PlanBudgetScope`) with complete cryptographic content identity.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_orbital::{ContentHash, DomainHasher, NaifId, VesselId, serde_dvec3};
use sim_spacetime::DynamicalModelIdentity;

/// Attitude / thrust-pointing steering frame for a finite-burn mission segment.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum BurnSteeringFrame {
    /// Fixed unit direction in `ICRF` (`J2000`).
    InertialIcrfUnit {
        /// Desired unit thrust vector in `ICRF`.
        #[serde(with = "serde_dvec3")]
        dir_icrf: DVec3,
    },
    /// Local Velocity–Normal–Binormal (`VNB`) orbital frame relative to `primary`:
    /// $\hat{\mathbf{V}} = \mathbf{v}/\|\mathbf{v}\|$, $\hat{\mathbf{N}} = (\mathbf{r}\times\mathbf{v})/\|\mathbf{r}\times\mathbf{v}\|$, $\hat{\mathbf{B}} = \hat{\mathbf{V}}\times\hat{\mathbf{N}}$.
    LocalVnbUnit {
        /// Reference primary body for computing $(\mathbf{r}, \mathbf{v})$.
        primary: NaifId,
        /// Unit direction $[c_V, c_N, c_B]$ in `VNB` coordinates (`[1, 0, 0]` = prograde, `[-1, 0, 0]` = retrograde).
        #[serde(with = "serde_dvec3")]
        dir_vnb: DVec3,
    },
    /// Fixed thrust direction in spacecraft body coordinates (e.g. `+Y` main engine axis).
    BodyAxisUnit {
        /// Unit thrust vector in body coordinates.
        #[serde(with = "serde_dvec3")]
        dir_body: DVec3,
    },
}

impl BurnSteeringFrame {
    fn write_to_hasher(&self, h: &mut DomainHasher) {
        match *self {
            Self::InertialIcrfUnit { dir_icrf } => {
                h.write_u8(1);
                h.write_vec3(dir_icrf);
            }
            Self::LocalVnbUnit { primary, dir_vnb } => {
                h.write_u8(2);
                h.write_i32(primary.0);
                h.write_vec3(dir_vnb);
            }
            Self::BodyAxisUnit { dir_body } => {
                h.write_u8(3);
                h.write_vec3(dir_body);
            }
        }
    }
}

/// Continuous propulsion law active during a [`MissionSegmentNode`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum SegmentBurnLaw {
    /// Ballistic unpowered coast ($T = 0$).
    Coast,
    /// Constant-thrust finite burn with specified thrust magnitude (`N`), exhaust velocity (`m/s`), and steering frame.
    ConstantThrust {
        /// Thrust magnitude $T \ge 0$ (`N`).
        thrust_n: f64,
        /// Effective exhaust velocity $v_e = I_{\text{sp}} g_0 > 0$ (`m/s`).
        exhaust_vel_m_s: f64,
        /// Thrust pointing frame.
        steering: BurnSteeringFrame,
    },
}

impl SegmentBurnLaw {
    fn write_to_hasher(&self, h: &mut DomainHasher) {
        match *self {
            Self::Coast => h.write_u8(0),
            Self::ConstantThrust {
                thrust_n,
                exhaust_vel_m_s,
                steering,
            } => {
                h.write_u8(1);
                h.write_f64_canonical(thrust_n);
                h.write_f64_canonical(exhaust_vel_m_s);
                steering.write_to_hasher(h);
            }
        }
    }
}

/// Direction of a zero-crossing event $g(\tau) = 0$.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CrossingDirection {
    /// Crosses from $g < 0$ to $g \ge 0$ ($\dot{g} > 0$).
    Rising,
    /// Crosses from $g > 0$ to $g \le 0$ ($\dot{g} < 0$).
    Falling,
    /// Either rising or falling crossing.
    Any,
}

impl CrossingDirection {
    const fn tag(self) -> u8 {
        match self {
            Self::Rising => 1,
            Self::Falling => 2,
            Self::Any => 3,
        }
    }
}

/// Scalar orbital event function $g(t, \mathbf{y}) = 0$ used to terminate a [`MissionSegmentNode`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum OrbitalScalarEventKind {
    /// Radial velocity $\mathbf{r}\cdot\mathbf{v} = 0$ with $\frac{d}{dt}(\mathbf{r}\cdot\mathbf{v}) > 0$ (periapsis passage around `body`).
    PeriapsisPassage {
        /// Central body for $(\mathbf{r}, \mathbf{v})$.
        body: NaifId,
    },
    /// Radial velocity $\mathbf{r}\cdot\mathbf{v} = 0$ with $\frac{d}{dt}(\mathbf{r}\cdot\mathbf{v}) < 0$ (apoapsis passage around `body`).
    ApoapsisPassage {
        /// Central body for $(\mathbf{r}, \mathbf{v})$.
        body: NaifId,
    },
    /// Distance $\|\mathbf{r}_{\text{rel}}\| - R_{\text{target}} = 0$ relative to `body`.
    RadiusCrossing {
        /// Central body for $\|\mathbf{r}_{\text{rel}}\|$.
        body: NaifId,
        /// Target radius (`m`).
        target_radius_m: f64,
    },
    /// Remaining propellant mass $m_p(t) - m_{p,\text{target}} = 0$.
    RemainingPropellantCrossing {
        /// Target remaining propellant mass (`kg`).
        target_propellant_kg: f64,
    },
    /// Synodic $x - x_{\text{target}} = 0$ in the `(primary, secondary)` rotating frame relative to `secondary`.
    SynodicXCrossing {
        /// Primary body ID.
        primary: NaifId,
        /// Secondary body ID.
        secondary: NaifId,
        /// Target synodic $x$ coordinate relative to `secondary` (`m`).
        target_x_rel_secondary_m: f64,
    },
}

impl OrbitalScalarEventKind {
    fn write_to_hasher(&self, h: &mut DomainHasher) {
        match *self {
            Self::PeriapsisPassage { body } => {
                h.write_u8(1);
                h.write_i32(body.0);
            }
            Self::ApoapsisPassage { body } => {
                h.write_u8(2);
                h.write_i32(body.0);
            }
            Self::RadiusCrossing {
                body,
                target_radius_m,
            } => {
                h.write_u8(3);
                h.write_i32(body.0);
                h.write_f64_canonical(target_radius_m);
            }
            Self::RemainingPropellantCrossing {
                target_propellant_kg,
            } => {
                h.write_u8(4);
                h.write_f64_canonical(target_propellant_kg);
            }
            Self::SynodicXCrossing {
                primary,
                secondary,
                target_x_rel_secondary_m,
            } => {
                h.write_u8(5);
                h.write_i32(primary.0);
                h.write_i32(secondary.0);
                h.write_f64_canonical(target_x_rel_secondary_m);
            }
        }
    }
}

/// Stopping condition that terminates a [`MissionSegmentNode`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum SegmentStoppingCondition {
    /// Propagate for a fixed duration $\Delta t \ge 0$ (`s`).
    FixedDuration {
        /// Duration (`s`).
        duration_s: f64,
    },
    /// Propagate until the earliest root of `event` in `[min_search_offset_s, max_duration_s]`
    /// isolated on the dense output polynomial to `time_tolerance_s`.
    ScalarEvent {
        /// Scalar event function $g(t, \mathbf{y}) = 0$.
        event: OrbitalScalarEventKind,
        /// Required crossing direction (`Rising`, `Falling`, or `Any`).
        direction: CrossingDirection,
        /// Maximum allowed propagation horizon before failing with `EventNotTriggered` (`s`).
        max_duration_s: f64,
        /// Minimum elapsed segment time before event roots are armed (`s`, e.g. `1.0` to skip $t = 0$).
        min_search_offset_s: f64,
        /// Root-isolation time tolerance (`s`, e.g. `1e-6`).
        time_tolerance_s: f64,
    },
}

impl SegmentStoppingCondition {
    fn write_to_hasher(&self, h: &mut DomainHasher) {
        match *self {
            Self::FixedDuration { duration_s } => {
                h.write_u8(1);
                h.write_f64_canonical(duration_s);
            }
            Self::ScalarEvent {
                event,
                direction,
                max_duration_s,
                min_search_offset_s,
                time_tolerance_s,
            } => {
                h.write_u8(2);
                event.write_to_hasher(h);
                h.write_u8(direction.tag());
                h.write_f64_canonical(max_duration_s);
                h.write_f64_canonical(min_search_offset_s);
                h.write_f64_canonical(time_tolerance_s);
            }
        }
    }
}

/// One sequential segment (coast or finite burn) in a [`MissionPlan`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MissionSegmentNode {
    /// Stable segment identifier within the plan.
    pub segment_id: u32,
    /// Human-readable segment label (e.g. `"TLI Burn"`, `"Coast to Apoapsis"`).
    pub label: String,
    /// Active propulsion and steering law during this segment.
    pub burn_law: SegmentBurnLaw,
    /// Stopping condition terminating this segment.
    pub stopping: SegmentStoppingCondition,
}

impl MissionSegmentNode {
    /// Writes the complete definition of this segment node into a [`DomainHasher`].
    pub fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_u32(self.segment_id);
        h.write_bytes(self.label.as_bytes());
        self.burn_law.write_to_hasher(h);
        self.stopping.write_to_hasher(h);
    }
}

/// How propellant and $\Delta v$ budgets are enforced across a [`MissionPlan`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PlanBudgetScope {
    /// Enforce `max_fuel_budget_kg` and `delta_v_budget_m_s` over the cumulative plan execution.
    FullPlanCumulative,
}

/// Ordered sequence of mission segments (`Coast` / `ConstantThrust` with `FixedDuration` / `ScalarEvent`
/// stopping conditions) bound to a specific [`VesselId`], `version`, and [`DynamicalModelIdentity`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MissionPlan {
    /// Unique mission plan identifier.
    pub plan_id: u64,
    /// Monotonically increasing revision number bumped on every edit or targeter commit.
    pub version: u64,
    /// Target spacecraft `VesselId`.
    pub vessel_id: VesselId,
    /// Dynamical model under which this plan is defined.
    pub model_identity: DynamicalModelIdentity,
    /// Ordered sequence of mission segments.
    pub segments: Vec<MissionSegmentNode>,
    /// Maximum cumulative propellant mass allowed across the plan (`kg`, `None` = unlimited up to tank).
    pub max_fuel_budget_kg: Option<f64>,
    /// Maximum cumulative ideal rocket $\Delta v$ allowed across the plan (`m/s`, `None` = unlimited).
    pub delta_v_budget_m_s: Option<f64>,
    /// Budget enforcement scope.
    pub budget_scope: PlanBudgetScope,
}

impl MissionPlan {
    /// Computes the full domain-separated SHA-256 [`ContentHash`] of this mission plan,
    /// covering `plan_id`, `version`, `vessel_id`, `model_identity`, budget limits,
    /// `budget_scope`, and every segment's `segment_id`, `label`, `burn_law`, and `stopping` condition in order.
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-mission:mission-plan-content:v1");
        h.write_u64(self.plan_id);
        h.write_u64(self.version);
        h.write_u64(self.vessel_id.0);
        self.model_identity.write_to_hasher(&mut h);
        match self.max_fuel_budget_kg {
            Some(v) => {
                h.write_u8(1);
                h.write_f64_canonical(v);
            }
            None => h.write_u8(0),
        }
        match self.delta_v_budget_m_s {
            Some(v) => {
                h.write_u8(1);
                h.write_f64_canonical(v);
            }
            None => h.write_u8(0),
        }
        match self.budget_scope {
            PlanBudgetScope::FullPlanCumulative => h.write_u8(1),
        }
        h.write_u64(self.segments.len() as u64);
        for seg in &self.segments {
            seg.write_to_hasher(&mut h);
        }
        h.finish()
    }
}

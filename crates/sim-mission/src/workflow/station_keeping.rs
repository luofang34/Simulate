//! Keeping a spacecraft about a collinear libration point under the full
//! ephemeris and solar radiation pressure.
//!
//! An orbit about $L_2$ is unstable: left alone, a spacecraft falls back
//! towards the secondary or drifts away from it within a few months. A
//! mission holds it with a small burn every few weeks, along the line of the
//! two bodies. A bounded orbit about the point crosses the plane through
//! that line and the orbit normal moving squarely across it; a spacecraft
//! about to fall or escape crosses it already drifting inward or outward.
//! Each burn is therefore sized so that, a chosen number of crossings ahead,
//! the spacecraft's velocity along the line is zero.

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use sim_orbital::{
    BodyOrientationCatalog, CoastControlLaw, CollinearLagrangePoint, ContinuousPlantState,
    Dop853Tolerances, EphemerisCatalog, ForceModel, LagrangeError, MassPropertiesModel, NaifId,
    PropellantMode, ReactionWheelMode, SolarRadiationPressureLaw, SrpSurface, SynodicFrame,
    TdbEpoch, propagate_plant_interval,
};
use sim_spacetime::SolarFluxModel;

const DAY_S: f64 = 86_400.0;
/// Steps allowed to solve for one burn.
const SOLVER_STEPS: u32 = 40;
/// A burn is found once its bracket is this narrow (`m/s`).
const BURN_RESOLUTION_M_S: f64 = 1.0e-5;
/// A burn is found once it leaves this little velocity along the line at the crossing
/// (`m/s`).
const CROSSING_RATE_TOLERANCE_M_S: f64 = 1.0e-3;
/// How often the lookahead checks the box and looks for a crossing (`s`).
const LOOKAHEAD_STEP_S: f64 = 2.0 * DAY_S;

/// A spacecraft to hold about $L_2$ and how to hold it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct L2StationKeepingSpec {
    /// The larger body (`NaifId::SUN` for Sun–Earth $L_2$).
    pub primary: NaifId,
    /// The smaller body, which the state is given about.
    pub secondary: NaifId,
    /// Epoch of the initial state.
    pub start_epoch: TdbEpoch,
    /// Initial position in the rotating frame of the two bodies, from the secondary (`m`).
    #[serde(with = "sim_orbital::serde_dvec3")]
    pub initial_pos_syn_m: DVec3,
    /// Initial velocity in that rotating frame (`m/s`).
    #[serde(with = "sim_orbital::serde_dvec3")]
    pub initial_vel_syn_m_s: DVec3,
    /// How long to fly (`s`).
    pub duration_s: f64,
    /// Time between burns (`s`).
    pub maintenance_interval_s: f64,
    /// Which crossing of the plane ahead the burn is aimed at; `2` is one revolution.
    pub crossings_ahead: u32,
    /// Longest time to wait for that crossing (`s`).
    pub lookahead_s: f64,
    /// Distance from $L_2$ along the line of the two bodies beyond which the
    /// spacecraft has fallen or escaped (`m`).
    pub x_bound_m: f64,
    /// Largest burn searched for (`m/s`).
    pub max_burn_m_s: f64,
    /// Burns smaller than this are not made (`m/s`).
    pub min_burn_m_s: f64,
    /// Spacecraft mass (`kg`).
    pub mass_kg: f64,
    /// The spacecraft as sunlight pushes on it, if radiation pressure is modelled.
    pub srp: Option<SrpSurface>,
}

/// One station-keeping burn.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct StationKeepingBurn {
    /// When it was made.
    pub epoch: TdbEpoch,
    /// Velocity change along the line from the primary through the secondary (`m/s`),
    /// positive away from the primary.
    pub delta_v_m_s: f64,
}

/// A flight held about $L_2$.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct L2StationKeepingReport {
    /// The burns made, in order; a maintenance epoch that needed none is absent.
    pub burns: Vec<StationKeepingBurn>,
    /// Sum of the burns' magnitudes (`m/s`).
    pub total_delta_v_m_s: f64,
    /// Largest departure from $L_2$ along the line of the two bodies (`m`).
    pub max_x_excursion_m: f64,
    /// Largest distance from $L_2$ (`m`).
    pub max_l2_distance_m: f64,
    /// Daily positions in the rotating frame, from $L_2$ (`m`).
    pub synodic_l2_samples_m: Vec<[f64; 3]>,
}

/// Why a spacecraft could not be held.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum StationKeepingError {
    /// The frame, the ephemeris or the propagation failed.
    #[error("libration-point dynamics failed")]
    Dynamics(#[from] LagrangeError),
    /// No burn within the limit turns the spacecraft's drift around.
    #[error(
        "no burn within {max_burn_m_s} m/s holds the spacecraft at {epoch:?}: it is already too far from the libration point"
    )]
    CannotHold {
        /// The maintenance epoch.
        epoch: TdbEpoch,
        /// The limit searched to.
        max_burn_m_s: f64,
    },
    /// A time span, bound, limit or count in the specification is out of range.
    #[error(
        "station-keeping spans, bound, burn limits, mass and crossing count must be positive and finite"
    )]
    InvalidSpec,
}

/// How a trajectory ends up, looking ahead from a maintenance epoch.
#[derive(Clone, Copy, PartialEq)]
enum Outlook {
    /// It left the box towards the secondary before the crossing.
    FellInward,
    /// It left the box away from the secondary before the crossing.
    DriftedOutward,
    /// It reached the crossing with this velocity along the line of the two bodies
    /// (`m/s`), positive away from the primary.
    Crossed(f64),
    /// It stayed in the box and did not reach the crossing in the time allowed.
    StillBounded,
}

impl Outlook {
    /// Negative when the spacecraft needs a push away from the primary.
    fn drift(self) -> Option<f64> {
        match self {
            Self::FellInward => Some(f64::NEG_INFINITY),
            Self::DriftedOutward => Some(f64::INFINITY),
            Self::Crossed(rate) => Some(rate),
            Self::StillBounded => None,
        }
    }
}

struct Keeper<'a> {
    spec: &'a L2StationKeepingSpec,
    mu_primary: f64,
    mu_secondary: f64,
    force_model: &'a ForceModel,
    flux: &'a SolarFluxModel,
    ephemeris: &'a dyn EphemerisCatalog,
    orientation: &'a dyn BodyOrientationCatalog,
}

impl Keeper<'_> {
    fn frame(&self, epoch: TdbEpoch) -> Result<SynodicFrame, LagrangeError> {
        Ok(SynodicFrame::from_ephemeris(
            self.spec.primary,
            self.spec.secondary,
            epoch,
            self.ephemeris,
        )?)
    }

    fn coast(
        &self,
        state: &ContinuousPlantState,
        from: TdbEpoch,
        to: TdbEpoch,
    ) -> Result<ContinuousPlantState, LagrangeError> {
        let mass = MassPropertiesModel::diagonal(self.spec.mass_kg, [1.0e4; 3], [0.0; 3]);
        let wheels = ReactionWheelMode::ConserveMomentum {
            h_max_nms: 0.0,
            tau_max_nm: 0.0,
        };
        let fly = |law: &dyn sim_orbital::ContinuousControlLaw| {
            propagate_plant_interval(
                state,
                from,
                to,
                Some(3_600.0),
                &tolerances(),
                &mass,
                PropellantMode::InfiniteConstantMass,
                wheels,
                self.force_model,
                self.ephemeris,
                self.orientation,
                law,
            )
        };
        let out = match self.spec.srp {
            Some(surface) => fly(&SolarRadiationPressureLaw {
                inner: &CoastControlLaw,
                surface,
                flux: self.flux,
                ephemeris: self.ephemeris,
            }),
            None => fly(&CoastControlLaw),
        }?;
        Ok(out.final_state)
    }

    /// `state` with `delta_v_m_s` added along the line of the two bodies.
    fn burned(
        &self,
        state: &ContinuousPlantState,
        epoch: TdbEpoch,
        delta_v_m_s: f64,
    ) -> Result<ContinuousPlantState, LagrangeError> {
        let along = self
            .frame(epoch)?
            .transform_position_synodic_to_icrf(DVec3::X);
        Ok(ContinuousPlantState {
            vel_rel_icrf_m_s: state.vel_rel_icrf_m_s + along * delta_v_m_s,
            ..*state
        })
    }

    /// Position from $L_2$ and velocity, both in the rotating frame.
    fn synodic_from_l2(
        &self,
        state: &ContinuousPlantState,
        epoch: TdbEpoch,
    ) -> Result<(DVec3, DVec3), LagrangeError> {
        let frame = self.frame(epoch)?;
        let l2 = frame.lagrange_point_synodic_m(
            self.mu_primary,
            self.mu_secondary,
            CollinearLagrangePoint::L2,
        );
        let (position, velocity) =
            frame.icrf_rel_secondary_to_synodic(state.pos_rel_icrf_m, state.vel_rel_icrf_m_s);
        Ok((position - l2, velocity))
    }

    /// Where `state` is headed over the lookahead.
    fn outlook(
        &self,
        state: &ContinuousPlantState,
        epoch: TdbEpoch,
    ) -> Result<Outlook, LagrangeError> {
        let mut state = *state;
        let (mut before, _) = self.synodic_from_l2(&state, epoch)?;
        let mut before_rate = 0.0;
        let mut crossings = 0_u32;
        let mut flown_s = 0.0;
        while flown_s < self.spec.lookahead_s {
            let step_s = LOOKAHEAD_STEP_S.min(self.spec.lookahead_s - flown_s);
            let to = epoch.advanced_seconds(flown_s + step_s);
            state = self.coast(&state, epoch.advanced_seconds(flown_s), to)?;
            flown_s += step_s;
            let (here, velocity) = self.synodic_from_l2(&state, to)?;
            if here.x > self.spec.x_bound_m {
                return Ok(Outlook::DriftedOutward);
            }
            if here.x < -self.spec.x_bound_m {
                return Ok(Outlook::FellInward);
            }
            if before.y * here.y < 0.0 {
                crossings = crossings.wrapping_add(1);
                if crossings >= self.spec.crossings_ahead {
                    let fraction = before.y / (before.y - here.y);
                    return Ok(Outlook::Crossed(
                        before_rate + fraction * (velocity.x - before_rate),
                    ));
                }
            }
            before = here;
            before_rate = velocity.x;
        }
        Ok(Outlook::StillBounded)
    }

    /// The burn at `epoch` that leaves `state` crossing the plane with no velocity
    /// along the line of the two bodies; zero if it is too small to make.
    fn holding_burn(
        &self,
        state: &ContinuousPlantState,
        epoch: TdbEpoch,
    ) -> Result<f64, StationKeepingError> {
        let drift_after = |delta_v: f64| -> Result<Option<f64>, LagrangeError> {
            Ok(self
                .outlook(&self.burned(state, epoch, delta_v)?, epoch)?
                .drift())
        };
        let limit = self.spec.max_burn_m_s;
        let cannot_hold = StationKeepingError::CannotHold {
            epoch,
            max_burn_m_s: limit,
        };
        let Some(drift_now) = drift_after(0.0)? else {
            return Ok(0.0);
        };
        // Bracket the burn between one that leaves the spacecraft drifting inward and
        // one that leaves it drifting outward, then close in by false position.
        let far = if drift_now < 0.0 { limit } else { -limit };
        let Some(drift_far) = drift_after(far)? else {
            return Ok(0.0);
        };
        if drift_now * drift_far > 0.0 {
            return Err(cannot_hold);
        }
        let (mut a, mut drift_a, mut b, mut drift_b) = (0.0, drift_now, far, drift_far);
        for _ in 0..SOLVER_STEPS {
            let both_reach_the_crossing = drift_a.is_finite() && drift_b.is_finite();
            let next = if both_reach_the_crossing {
                a - drift_a * (b - a) / (drift_b - drift_a)
            } else {
                0.5 * (a + b)
            };
            let Some(drift) = drift_after(next)? else {
                return Ok(self.worth_making(next));
            };
            if drift * drift_a > 0.0 {
                (a, drift_a) = (next, drift);
            } else {
                (b, drift_b) = (next, drift);
            }
            if drift.abs() < CROSSING_RATE_TOLERANCE_M_S || (b - a).abs() < BURN_RESOLUTION_M_S {
                return Ok(self.worth_making(next));
            }
        }
        Err(cannot_hold)
    }

    fn worth_making(&self, delta_v_m_s: f64) -> f64 {
        if delta_v_m_s.abs() < self.spec.min_burn_m_s {
            0.0
        } else {
            delta_v_m_s
        }
    }
}

/// Flies `spec`'s spacecraft about $L_2$, burning at each maintenance epoch whatever
/// keeps it inside the box for the lookahead.
pub fn fly_l2_station_keeping(
    spec: &L2StationKeepingSpec,
    force_model: &ForceModel,
    flux: &SolarFluxModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<L2StationKeepingReport, StationKeepingError> {
    let positive = [
        spec.duration_s,
        spec.maintenance_interval_s,
        spec.lookahead_s,
        spec.x_bound_m,
        spec.max_burn_m_s,
        spec.mass_kg,
        f64::from(spec.crossings_ahead),
    ];
    let deadband_ok = spec.min_burn_m_s.is_finite() && spec.min_burn_m_s >= 0.0;
    if !deadband_ok || positive.iter().any(|v| !v.is_finite() || *v <= 0.0) {
        return Err(StationKeepingError::InvalidSpec);
    }
    let constants = force_model.constants();
    let keeper = Keeper {
        spec,
        mu_primary: constants
            .require_mu(spec.primary)
            .map_err(LagrangeError::from)?,
        mu_secondary: constants
            .require_mu(spec.secondary)
            .map_err(LagrangeError::from)?,
        force_model,
        flux,
        ephemeris,
        orientation,
    };
    let frame0 = keeper.frame(spec.start_epoch)?;
    let (position, velocity) =
        frame0.synodic_to_icrf_rel_secondary(spec.initial_pos_syn_m, spec.initial_vel_syn_m_s);
    let mut state = ContinuousPlantState {
        primary_body: spec.secondary,
        pos_rel_icrf_m: position,
        vel_rel_icrf_m_s: velocity,
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let mut report = L2StationKeepingReport {
        burns: Vec::new(),
        total_delta_v_m_s: 0.0,
        max_x_excursion_m: 0.0,
        max_l2_distance_m: 0.0,
        synodic_l2_samples_m: Vec::new(),
    };
    let mut flown_s = 0.0;
    let mut next_burn_s = 0.0;
    while flown_s < spec.duration_s {
        let epoch = spec.start_epoch.advanced_seconds(flown_s);
        if flown_s >= next_burn_s {
            let delta_v_m_s = keeper.holding_burn(&state, epoch)?;
            if delta_v_m_s != 0.0 {
                state = keeper.burned(&state, epoch, delta_v_m_s)?;
                report.burns.push(StationKeepingBurn { epoch, delta_v_m_s });
                report.total_delta_v_m_s += delta_v_m_s.abs();
            }
            next_burn_s += spec.maintenance_interval_s;
        }
        let step_s = DAY_S
            .min(spec.duration_s - flown_s)
            .min(next_burn_s - flown_s);
        let to = epoch.advanced_seconds(step_s);
        state = keeper.coast(&state, epoch, to)?;
        flown_s += step_s;
        let (from_l2, _) = keeper.synodic_from_l2(&state, to)?;
        report.max_x_excursion_m = report.max_x_excursion_m.max(from_l2.x.abs());
        report.max_l2_distance_m = report.max_l2_distance_m.max(from_l2.length());
        report.synodic_l2_samples_m.push(from_l2.to_array());
    }
    Ok(report)
}

fn tolerances() -> Dop853Tolerances {
    Dop853Tolerances {
        rtol: 1.0e-10,
        atol_pos_m: 1.0e-3,
        atol_vel_m_s: 1.0e-8,
        h_min_s: 1.0,
        h_max_s: 43_200.0,
        h_init_s: 3_600.0,
        max_steps: 50_000,
        ..Dop853Tolerances::default()
    }
}

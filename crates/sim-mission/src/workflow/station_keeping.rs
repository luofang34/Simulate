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
    BodyOrientationCatalog, ContinuousPlantState, EphemerisCatalog, ForceModel, L2HoldingDynamics,
    L2HoldingTarget, LagrangeError, MassPropertiesModel, NaifId, PropellantMode, ReactionWheelMode,
    SrpSurface, TdbEpoch, l2_holding_tolerances,
};
use sim_spacetime::SolarFluxModel;

const DAY_S: f64 = 86_400.0;

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

impl L2StationKeepingSpec {
    const fn holding_target(&self) -> L2HoldingTarget {
        L2HoldingTarget {
            maintenance_interval_s: self.maintenance_interval_s,
            crossings_ahead: self.crossings_ahead,
            lookahead_s: self.lookahead_s,
            x_bound_m: self.x_bound_m,
            max_burn_m_s: self.max_burn_m_s,
            min_burn_m_s: self.min_burn_m_s,
        }
    }
}

impl StationKeepingError {
    fn from_holding(error: LagrangeError) -> Self {
        match error {
            LagrangeError::NoHoldingBurn {
                epoch,
                max_burn_m_s,
            } => Self::CannotHold {
                epoch,
                max_burn_m_s,
            },
            LagrangeError::InvalidHoldingTarget => Self::InvalidSpec,
            other => Self::Dynamics(other),
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
    let target = spec.holding_target();
    let durations_ok = [spec.duration_s, spec.mass_kg]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0);
    if !durations_ok || !target.is_valid() {
        return Err(StationKeepingError::InvalidSpec);
    }
    let mass_model = MassPropertiesModel::diagonal(spec.mass_kg, [1.0e4; 3], [0.0; 3]);
    let tolerances = l2_holding_tolerances();
    let keeper = L2HoldingDynamics {
        primary: spec.primary,
        secondary: spec.secondary,
        force_model,
        flux,
        ephemeris,
        orientation,
        mass_model: &mass_model,
        propellant_mode: PropellantMode::InfiniteConstantMass,
        rw_mode: ReactionWheelMode::ConserveMomentum {
            h_max_nms: 0.0,
            tau_max_nm: 0.0,
        },
        tolerances: &tolerances,
        srp: spec.srp,
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
            let delta_v_m_s = keeper
                .holding_burn_m_s(&target, &state, epoch)
                .map_err(StationKeepingError::from_holding)?;
            if delta_v_m_s != 0.0 {
                state = keeper.burned(&state, epoch, delta_v_m_s)?;
                report.burns.push(StationKeepingBurn { epoch, delta_v_m_s });
                report.total_delta_v_m_s += delta_v_m_s.abs();
            }
            next_burn_s += target.maintenance_interval_s;
        }
        let step_s = DAY_S
            .min(spec.duration_s - flown_s)
            .min(next_burn_s - flown_s);
        let to = epoch.advanced_seconds(step_s);
        state = keeper.coast(&state, epoch, to)?;
        flown_s += step_s;
        let (from_l2, _) = keeper.offset_from_l2(&state, to)?;
        report.max_x_excursion_m = report.max_x_excursion_m.max(from_l2.x.abs());
        report.max_l2_distance_m = report.max_l2_distance_m.max(from_l2.length());
        report.synodic_l2_samples_m.push(from_l2.to_array());
    }
    Ok(report)
}

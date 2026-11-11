//! The burn that keeps a spacecraft about a collinear $L_2$ point.
//!
//! An orbit about $L_2$ is unstable: left alone, a spacecraft falls back
//! towards the secondary or drifts away from it within a few months. A bounded
//! orbit about the point crosses the plane through the line of the two bodies
//! and the orbit normal moving squarely across it; a spacecraft about to fall
//! or escape crosses it already drifting inward or outward. A holding burn,
//! made along that line, is sized so that a chosen number of crossings ahead
//! the spacecraft's velocity along the line is zero. The burn is made every
//! maintenance interval, whether or not the spacecraft is at a crossing, so the
//! unstable drift never grows for long between corrections.
//!
//! The prediction flies exactly the forces the vessel's propagator applies —
//! gravity, solar radiation pressure on the vessel's own surface and mass —
//! so a burn solved here holds the vessel the propagator actually carries.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_spacetime::SolarFluxModel;

use super::{CollinearLagrangePoint, LagrangeError, SynodicFrame};
use crate::{
    BodyOrientationCatalog, CoastControlLaw, ContinuousControlLaw, ContinuousPlantState,
    Dop853Tolerances, EphemerisCatalog, ForceModel, MassPropertiesModel, NaifId, PropellantMode,
    ReactionWheelMode, SolarRadiationPressureLaw, SrpSurface, TdbEpoch, propagate_plant_interval,
};

mod search;

pub use search::HoldingBurnSearch;

/// First integration step of each lookahead leg (`s`).
const FIRST_STEP_S: f64 = 3_600.0;

/// Integrator tolerances a holding-burn prediction flies with: a few metres over a
/// half-year lookahead, far inside what a burn's sensitivity can resolve, and steps
/// long enough that a solve stays cheap.
#[must_use]
pub fn l2_holding_tolerances() -> Dop853Tolerances {
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

/// What a holding burn aims for and how far it may go.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct L2HoldingTarget {
    /// Time between holding burns (`s`); the first is made at once.
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
}

impl L2HoldingTarget {
    /// Whether every span, bound and limit is positive and finite and the deadband is
    /// non-negative.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        let positive = [
            self.maintenance_interval_s,
            self.lookahead_s,
            self.x_bound_m,
            self.max_burn_m_s,
            f64::from(self.crossings_ahead),
        ];
        positive.iter().all(|v| v.is_finite() && *v > 0.0)
            && self.min_burn_m_s.is_finite()
            && self.min_burn_m_s >= 0.0
    }
}

/// The dynamics a spacecraft held about $L_2$ is flown in; it must be the model its
/// propagator integrates, or the burns aim at an orbit the vessel is not on.
#[derive(Clone, Copy)]
pub struct L2HoldingDynamics<'a> {
    /// The larger body (`NaifId::SUN` for Sun–Earth $L_2$).
    pub primary: NaifId,
    /// The smaller body, which states are given about.
    pub secondary: NaifId,
    /// Gravity.
    pub force_model: &'a ForceModel,
    /// The Sun and what eclipses it.
    pub flux: &'a SolarFluxModel,
    /// Body positions.
    pub ephemeris: &'a dyn EphemerisCatalog,
    /// Body orientations.
    pub orientation: &'a dyn BodyOrientationCatalog,
    /// The spacecraft's mass, which radiation pressure is divided by.
    pub mass_model: &'a MassPropertiesModel,
    /// How the spacecraft's propellant is accounted.
    pub propellant_mode: PropellantMode,
    /// How its reaction wheels are accounted.
    pub rw_mode: ReactionWheelMode,
    /// Integrator tolerances for the prediction.
    pub tolerances: &'a Dop853Tolerances,
    /// The spacecraft as sunlight pushes on it, if radiation pressure is modelled.
    pub srp: Option<SrpSurface>,
}

impl L2HoldingDynamics<'_> {
    /// The rotating frame of the two bodies at `epoch`.
    pub fn frame(&self, epoch: TdbEpoch) -> Result<SynodicFrame, LagrangeError> {
        Ok(SynodicFrame::from_ephemeris(
            self.primary,
            self.secondary,
            epoch,
            self.ephemeris,
        )?)
    }

    /// `state` flown without burns from `from` to `to`.
    pub fn coast(
        &self,
        state: &ContinuousPlantState,
        from: TdbEpoch,
        to: TdbEpoch,
    ) -> Result<ContinuousPlantState, LagrangeError> {
        let fly = |law: &dyn ContinuousControlLaw| {
            propagate_plant_interval(
                state,
                from,
                to,
                Some(FIRST_STEP_S),
                self.tolerances,
                self.mass_model,
                self.propellant_mode,
                self.rw_mode,
                self.force_model,
                self.ephemeris,
                self.orientation,
                law,
            )
        };
        let out = match self.srp {
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

    /// Unit vector in ICRF axes along the line from the primary through the
    /// secondary at `epoch`: the direction a positive holding burn pushes.
    pub fn burn_direction_icrf(&self, epoch: TdbEpoch) -> Result<DVec3, LagrangeError> {
        Ok(self
            .frame(epoch)?
            .transform_position_synodic_to_icrf(DVec3::X))
    }

    /// `state` with `delta_v_m_s` added along the line of the two bodies.
    pub fn burned(
        &self,
        state: &ContinuousPlantState,
        epoch: TdbEpoch,
        delta_v_m_s: f64,
    ) -> Result<ContinuousPlantState, LagrangeError> {
        let along = self.burn_direction_icrf(epoch)?;
        Ok(ContinuousPlantState {
            vel_rel_icrf_m_s: state.vel_rel_icrf_m_s + along * delta_v_m_s,
            ..*state
        })
    }

    /// `state`'s position from $L_2$ and its velocity, both in the rotating frame.
    /// The state must be given about the secondary.
    pub fn offset_from_l2(
        &self,
        state: &ContinuousPlantState,
        epoch: TdbEpoch,
    ) -> Result<(DVec3, DVec3), LagrangeError> {
        if state.primary_body != self.secondary {
            return Err(LagrangeError::WrongCentralBody {
                expected: self.secondary,
                found: state.primary_body,
            });
        }
        let constants = self.force_model.constants();
        let frame = self.frame(epoch)?;
        let l2 = frame.lagrange_point_synodic_m(
            constants.require_mu(self.primary)?,
            constants.require_mu(self.secondary)?,
            CollinearLagrangePoint::L2,
        );
        let (position, velocity) =
            frame.icrf_rel_secondary_to_synodic(state.pos_rel_icrf_m, state.vel_rel_icrf_m_s);
        Ok((position - l2, velocity))
    }

    /// The burn at `epoch` along the line of the two bodies (`m/s`, positive away
    /// from the primary) that leaves `state` crossing the plane `target`'s number of
    /// crossings ahead with no velocity along that line; zero if it is too small to
    /// make or the spacecraft stays bounded through the whole lookahead.
    ///
    /// # Errors
    /// [`LagrangeError::NoHoldingBurn`] when no burn within the limit turns the drift
    /// around, [`LagrangeError::InvalidHoldingTarget`] for an out-of-range target, and
    /// the frame, ephemeris and propagation errors of the prediction.
    pub fn holding_burn_m_s(
        &self,
        target: &L2HoldingTarget,
        state: &ContinuousPlantState,
        epoch: TdbEpoch,
    ) -> Result<f64, LagrangeError> {
        let mut search = HoldingBurnSearch::new(*target, *state, epoch)?;
        loop {
            if let Some(burn) = search.advance(self)? {
                return Ok(burn);
            }
        }
    }
}

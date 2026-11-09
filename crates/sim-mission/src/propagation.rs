//! Stage-evaluated finite-burn control law (`StageEvaluatedBurnLaw`) bridging
//! [`SegmentBurnLaw`](crate::SegmentBurnLaw) into `sim-orbital::ContinuousControlLaw`.

use std::sync::Mutex;

use glam::DVec3;
use sim_orbital::{
    ContinuousControlLaw, ContinuousPlantState, DerivativeOrder, EphemerisCatalog, GravityError,
    NaifId, PlantStageWrench, PropagationAudit, TdbEpoch, quat_normalize_portable,
    quat_rotate_vec_portable,
};
use sim_spacetime::sqrt;

use crate::{
    error::MissionDesignError,
    plan::{BurnSteeringFrame, SegmentBurnLaw},
};

/// Stage-evaluated continuous propulsion law for a [`MissionSegmentNode`](crate::MissionSegmentNode).
pub(crate) struct StageEvaluatedBurnLaw<'a> {
    plan_id: u64,
    segment_id: u32,
    law: SegmentBurnLaw,
    ephemeris: &'a dyn EphemerisCatalog,
    stage_error: Mutex<Option<MissionDesignError>>,
}

impl<'a> StageEvaluatedBurnLaw<'a> {
    pub(crate) const fn new(
        plan_id: u64,
        segment_id: u32,
        law: SegmentBurnLaw,
        ephemeris: &'a dyn EphemerisCatalog,
    ) -> Self {
        Self {
            plan_id,
            segment_id,
            law,
            ephemeris,
            stage_error: Mutex::new(None),
        }
    }

    pub(crate) fn take_error(&self) -> Option<MissionDesignError> {
        self.stage_error
            .lock()
            .ok()
            .and_then(|mut slot| slot.take())
    }

    fn record_error(&self, err: MissionDesignError) {
        if let Ok(mut slot) = self.stage_error.lock()
            && slot.is_none()
        {
            *slot = Some(err);
        }
    }
}

impl ContinuousControlLaw for StageEvaluatedBurnLaw<'_> {
    fn evaluate_stage(
        &self,
        tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> PlantStageWrench {
        self.evaluate_stage_checked(tau_s, epoch, state)
            .unwrap_or_default()
    }

    fn evaluate_stage_checked(
        &self,
        _tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> Result<PlantStageWrench, GravityError> {
        match self.law {
            SegmentBurnLaw::Coast => Ok(PlantStageWrench::default()),
            SegmentBurnLaw::ConstantThrust {
                thrust_n,
                exhaust_vel_m_s,
                steering,
            } => {
                if thrust_n <= 0.0 || state.propellant_mass_kg <= 0.0 {
                    return Ok(PlantStageWrench::default());
                }
                let q_b2i = quat_normalize_portable(state.q_body_to_icrf);
                let dir_body = match steering {
                    BurnSteeringFrame::BodyAxisUnit { dir_body } => {
                        self.normalize_checked(dir_body, state.primary_body, "BodyAxisUnit")?
                    }
                    BurnSteeringFrame::InertialIcrfUnit { dir_icrf } => {
                        let u_icrf = self.normalize_checked(
                            dir_icrf,
                            state.primary_body,
                            "InertialIcrfUnit",
                        )?;
                        quat_rotate_vec_portable(q_b2i.conjugate(), u_icrf)
                    }
                    BurnSteeringFrame::LocalVnbUnit { primary, dir_vnb } => {
                        let (r_rel, v_rel) = self.resolve_vnb_relative_pv(state, primary, epoch)?;
                        let u_icrf =
                            self.vnb_to_icrf_unit_checked(r_rel, v_rel, dir_vnb, primary)?;
                        quat_rotate_vec_portable(q_b2i.conjugate(), u_icrf)
                    }
                };
                Ok(PlantStageWrench {
                    main_thrust_body_n: dir_body * thrust_n,
                    main_exhaust_vel_m_s: exhaust_vel_m_s,
                    stage_audit: PropagationAudit::default(),
                    ..PlantStageWrench::default()
                })
            }
        }
    }
}

impl StageEvaluatedBurnLaw<'_> {
    fn resolve_vnb_relative_pv(
        &self,
        state: &ContinuousPlantState,
        primary: NaifId,
        epoch: TdbEpoch,
    ) -> Result<(DVec3, DVec3), GravityError> {
        if state.primary_body == primary {
            return Ok((state.pos_rel_icrf_m, state.vel_rel_icrf_m_s));
        }
        let shift = self
            .ephemeris
            .geometric_state_icrf(
                state.primary_body,
                primary,
                epoch,
                DerivativeOrder::Velocity,
            )
            .map_err(|err| {
                self.record_error(MissionDesignError::Ephemeris(err.clone()));
                GravityError::Ephemeris(err)
            })?;
        Ok((
            state.pos_rel_icrf_m + shift.pos_m,
            state.vel_rel_icrf_m_s + shift.vel_m_s,
        ))
    }

    fn normalize_checked(
        &self,
        v: DVec3,
        body: NaifId,
        context: &'static str,
    ) -> Result<DVec3, GravityError> {
        let n_sq = v.length_squared();
        let n = sqrt(n_sq);
        if !n.is_finite() || n <= 1.0e-15 {
            self.record_error(MissionDesignError::InvalidPlan {
                plan_id: self.plan_id,
                reason: format!(
                    "segment {} has zero or non-finite steering direction in {context}",
                    self.segment_id
                ),
            });
            return Err(GravityError::SingularRadius(body));
        }
        Ok(v / n)
    }

    fn vnb_to_icrf_unit_checked(
        &self,
        r_icrf: DVec3,
        v_icrf: DVec3,
        dir_vnb: DVec3,
        primary: NaifId,
    ) -> Result<DVec3, GravityError> {
        let v_norm = sqrt(v_icrf.length_squared());
        let h = r_icrf.cross(v_icrf);
        let h_norm = sqrt(h.length_squared());
        if !v_norm.is_finite() || v_norm <= 1.0e-12 || !h_norm.is_finite() || h_norm <= 1.0e-12 {
            self.record_error(MissionDesignError::InvalidPlan {
                plan_id: self.plan_id,
                reason: format!(
                    "segment {} encountered degenerate LocalVnbUnit basis around body {primary}",
                    self.segment_id
                ),
            });
            return Err(GravityError::SingularRadius(primary));
        }
        let u_v = v_icrf / v_norm;
        let u_n = h / h_norm;
        let u_b = u_v.cross(u_n);
        self.normalize_checked(
            dir_vnb.x * u_v + dir_vnb.y * u_n + dir_vnb.z * u_b,
            primary,
            "LocalVnbUnit",
        )
    }
}

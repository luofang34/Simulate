//! The two ends of a link at the instants a signal leaves one and reaches the other.

use sim_spacetime::{SPEED_OF_LIGHT_M_S, TdbEpoch, sqrt};

use crate::{
    access::AccessAnalysisError,
    models::SignalTimeModel,
    relay::{
        evaluator::EndpointPairCtx,
        kinematics::{EndpointWorldState, evaluate_endpoint_world_state},
    },
};

/// Light-time iterations: each gains a factor of the source's speed over light, so a few
/// reach the rounding of an interplanetary range.
const LIGHT_TIME_ITERATIONS: usize = 5;

/// A link's source at emission and its target at reception.
pub(super) struct SignalEvents {
    pub(super) source: EndpointWorldState,
    pub(super) target: EndpointWorldState,
    pub(super) emit_epoch: TdbEpoch,
    pub(super) light_time_s: Option<f64>,
}

/// The ends of `ctx`'s link for a signal arriving at the target at `epoch`: both at
/// `epoch` under [`SignalTimeModel::Geometric`], the source one light time earlier under
/// [`SignalTimeModel::LightTime`].
pub(super) fn resolve_signal_events(
    ctx: &EndpointPairCtx<'_>,
    epoch: TdbEpoch,
) -> Result<SignalEvents, AccessAnalysisError> {
    let dem = ctx.terrain_dem.map(|(d, _)| d);
    let at = |endpoint, when| {
        evaluate_endpoint_world_state(endpoint, when, dem, ctx.ephemeris, ctx.orientation)
    };
    let target = at(&ctx.target, epoch)?;
    let mut source = at(&ctx.source, epoch)?;
    if ctx.query.signal_time == SignalTimeModel::Geometric {
        return Ok(SignalEvents {
            source,
            target,
            emit_epoch: epoch,
            light_time_s: None,
        });
    }
    let mut light_time_s = 0.0_f64;
    let mut emit_epoch = epoch;
    for _ in 0..LIGHT_TIME_ITERATIONS {
        let range_m = sqrt((target.pos_ssb_m - source.pos_ssb_m).length_squared());
        light_time_s = range_m / SPEED_OF_LIGHT_M_S;
        emit_epoch = epoch.advanced_seconds(-light_time_s);
        source = at(&ctx.source, emit_epoch)?;
    }
    Ok(SignalEvents {
        source,
        target,
        emit_epoch,
        light_time_s: Some(light_time_s),
    })
}

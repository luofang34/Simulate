//! Link budgets over the epochs a geometric access analysis accepts.

use sim_analysis_contract::{
    BudgetUnavailableReason, LinkBudgetReport, LinkBudgetSample, RfLinkSpec, VerificationScope,
};
use sim_spacetime::{BodyOrientationCatalog, EphemerisCatalog};

use super::{
    antenna::AntennaPattern,
    budget::{LinkBudgetInputs, compute_link_budget},
    path::{PathGeometry, PropagationLoss},
};
use crate::{
    access::AccessAnalysisError,
    models::{AccessQuerySpec, OccultationEnvironmentModel},
    relay::{AnalysisEndpoint, EndpointLinkSample, evaluate_endpoint_link_access},
    terrain::{TerrainElevationSource, TerrainLosConfig},
};

/// The antenna and propagation models a budget is evaluated with.
///
/// [`LinkBudgetModels::from_spec`] takes them from the link description; replace a field or
/// push to `propagation` to bring a model the description cannot express.
pub struct LinkBudgetModels<'a> {
    /// Pattern of the transmit antenna.
    pub transmit_antenna: &'a dyn AntennaPattern,
    /// Pattern of the receive antenna.
    pub receive_antenna: &'a dyn AntennaPattern,
    /// Extra propagation losses.
    pub propagation: Vec<&'a dyn PropagationLoss>,
}

impl<'a> LinkBudgetModels<'a> {
    /// Models named by `link`.
    #[must_use]
    pub fn from_spec(link: &'a RfLinkSpec) -> Self {
        Self {
            transmit_antenna: &link.transmitter.antenna,
            receive_antenna: &link.receiver.antenna,
            propagation: link
                .propagation
                .iter()
                .map(|m| m as &dyn PropagationLoss)
                .collect(),
        }
    }
}

const fn is_surface(endpoint: &AnalysisEndpoint<'_>) -> bool {
    matches!(endpoint, AnalysisEndpoint::SurfaceStation { .. })
}

fn path_of(
    sample: &EndpointLinkSample,
    link: &RfLinkSpec,
    source_on_surface: bool,
    target_on_surface: bool,
) -> PathGeometry {
    PathGeometry {
        epoch: sample.epoch,
        carrier_frequency_hz: link.carrier_frequency_hz,
        source_position_icrf_m: sample.source_pos_icrf_m,
        target_position_icrf_m: sample.target_pos_icrf_m,
        slant_range_m: sample.slant_range_m,
        range_rate_m_s: sample.range_rate_m_s,
        light_time_s: sample.light_time_s,
        source_surface_elevation_rad: source_on_surface.then_some(sample.elevation_from_source_rad),
        target_surface_elevation_rad: target_on_surface.then_some(sample.elevation_from_target_rad),
    }
}

/// Evaluates access from `source` to `target` and, on every epoch where it holds, the radio
/// budget of `link` with `source` transmitting.
///
/// The returned access report states `rf_link_budget_verified`. An epoch that is occulted,
/// masked, outside a field of view, or outside a model's validity has no budget and says why.
#[allow(clippy::too_many_arguments)]
pub fn evaluate_endpoint_link_budget(
    source: AnalysisEndpoint<'_>,
    target: AnalysisEndpoint<'_>,
    link: &RfLinkSpec,
    models: &LinkBudgetModels<'_>,
    occultation_env: &OccultationEnvironmentModel,
    terrain_dem: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)>,
    query: &AccessQuerySpec,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<LinkBudgetReport, AccessAnalysisError> {
    link.validate()?;
    let (src_surface, dst_surface) = (is_surface(&source), is_surface(&target));
    let mut access = evaluate_endpoint_link_access(
        source,
        target,
        occultation_env,
        terrain_dem,
        query,
        ephemeris,
        orientation,
    )?;
    access.verification_scope = VerificationScope::geometric_visibility_with_rf_link_budget();
    let samples = access
        .samples
        .iter()
        .map(|sample| budget_sample(sample, link, models, src_surface, dst_surface))
        .collect();
    Ok(LinkBudgetReport {
        link: link.clone(),
        access,
        samples,
    })
}

pub(crate) fn budget_sample(
    sample: &EndpointLinkSample,
    link: &RfLinkSpec,
    models: &LinkBudgetModels<'_>,
    src_surface: bool,
    dst_surface: bool,
) -> LinkBudgetSample {
    let outcome = if sample.accessible {
        let path = path_of(sample, link, src_surface, dst_surface);
        compute_link_budget(&LinkBudgetInputs {
            link,
            transmit_antenna: models.transmit_antenna,
            receive_antenna: models.receive_antenna,
            propagation: &models.propagation,
            path: &path,
            transmit_look_in_mount: sample.source_look_direction_mount,
            receive_look_in_mount: sample.target_look_direction_mount,
        })
    } else {
        Err(BudgetUnavailableReason::NotAccessible)
    };
    match outcome {
        Ok(budget) => LinkBudgetSample {
            epoch: sample.epoch,
            budget: Some(budget),
            unavailable: None,
        },
        Err(reason) => LinkBudgetSample {
            epoch: sample.epoch,
            budget: None,
            unavailable: Some(reason),
        },
    }
}

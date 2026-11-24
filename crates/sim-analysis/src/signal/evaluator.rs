//! Link budgets over the epochs a geometric access analysis accepts.

use sim_analysis_contract::{
    BudgetUnavailableReason, LinkBudgetReport, LinkBudgetSample, RfLinkSpec, VerificationScope,
};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_spacetime::{
    BodyOrientationCatalog, DerivativeOrder, EphemerisCatalog, NaifId, TdbEpoch, acos_clamped, sin,
};
use world_propagation::{Polarisation, ProfilePoint};

use super::{
    antenna::AntennaPattern,
    budget::{LinkBudgetInputs, compute_link_budget},
    path::{ExceedancePercentages, PathGeometry, PropagationLoss},
};
use crate::{
    access::AccessAnalysisError,
    models::{AccessQuerySpec, OccultationEnvironmentModel},
    relay::{
        AnalysisEndpoint, EndpointLinkSample, EndpointWorldState, evaluate_endpoint_link_access,
        evaluate_endpoint_world_state,
    },
    terrain::{TerrainElevationSource, TerrainLosConfig},
};

const MAX_TERRAIN_PROFILE_STEPS: u32 = 1_024;

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
    /// Optional wave polarisation supplied to terrestrial propagation models.
    pub polarisation: Option<Polarisation>,
    /// Optional exceedance percentages `(time, location, situation)` supplied to ITM.
    pub exceedance: Option<ExceedancePercentages>,
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
            polarisation: None,
            exceedance: None,
        }
    }

    /// Sets the wave polarisation passed to [`PathGeometry::polarisation`].
    #[must_use]
    pub const fn with_polarisation(mut self, polarisation: Polarisation) -> Self {
        self.polarisation = Some(polarisation);
        self
    }

    /// Sets the exceedance quantiles passed to [`PathGeometry::exceedance`].
    #[must_use]
    pub const fn with_exceedance(mut self, exceedance: ExceedancePercentages) -> Self {
        self.exceedance = Some(exceedance);
        self
    }
}

const fn is_surface(endpoint: &AnalysisEndpoint<'_>) -> bool {
    matches!(endpoint, AnalysisEndpoint::SurfaceStation { .. })
}

struct TerrainContext {
    source_geodetic: Option<Geodetic>,
    target_geodetic: Option<Geodetic>,
    terrain_profile: Option<Vec<ProfilePoint>>,
}

impl TerrainContext {
    const EMPTY: Self = Self {
        source_geodetic: None,
        target_geodetic: None,
        terrain_profile: None,
    };
}

fn fixed_geodetic_of(
    state: &EndpointWorldState,
    epoch: TdbEpoch,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Option<Geodetic> {
    let body_ssb = ephemeris
        .geometric_state_icrf(
            state.host_body,
            NaifId::SSB,
            epoch,
            DerivativeOrder::Position,
        )
        .ok()?
        .pos_m;
    let rot = orientation.body_rotation(state.host_body, epoch).ok()?;
    let pos_fixed = rot.r_icrf_to_fixed * (state.pos_ssb_m - body_ssb);
    Some(state.ellipsoid.fixed_to_geodetic(pos_fixed))
}

fn sample_arc_terrain_profile(
    src_geo: &Geodetic,
    dst_geo: &Geodetic,
    ellipsoid: BodyEllipsoid,
    dem: &dyn TerrainElevationSource,
    cfg: &TerrainLosConfig,
    epoch: TdbEpoch,
) -> Option<Vec<ProfilePoint>> {
    if !cfg.sample_step_m.is_finite() || cfg.sample_step_m <= 0.0 {
        return None;
    }
    let p0_surf = ellipsoid.geodetic_to_fixed(&Geodetic {
        lat_rad: src_geo.lat_rad,
        lon_rad: src_geo.lon_rad,
        height_m: 0.0,
    });
    let p1_surf = ellipsoid.geodetic_to_fixed(&Geodetic {
        lat_rad: dst_geo.lat_rad,
        lon_rad: dst_geo.lon_rad,
        height_m: 0.0,
    });
    let u0 = p0_surf.try_normalize()?;
    let u1 = p1_surf.try_normalize()?;
    let psi = acos_clamped(u0.dot(u1));
    let arc_m = psi * 0.5 * (p0_surf.length() + p1_surf.length());
    if !arc_m.is_finite() || arc_m <= 1.0 {
        return None;
    }
    let step_m = dem
        .min_horizontal_cell_spacing_m(ellipsoid)
        .map_or(cfg.sample_step_m, |cell| {
            cfg.sample_step_m.min((0.5 * cell).max(1.0))
        });
    let max_steps = u64::from(cfg.max_ray_samples.clamp(2, MAX_TERRAIN_PROFILE_STEPS));
    let n_steps = (libm::ceil(arc_m / step_m) as u64).clamp(2, max_steps) as usize;
    let xi = arc_m / (n_steps as f64);
    let sin_psi = sin(psi);
    let mut profile = Vec::with_capacity(n_steps.saturating_add(1));

    for idx in 0..=n_steps {
        let t = (idx as f64) / (n_steps as f64);
        let dir = if sin_psi > 1.0e-7 {
            let w0 = sin((1.0 - t) * psi) / sin_psi;
            let w1 = sin(t * psi) / sin_psi;
            (u0 * w0 + u1 * w1).try_normalize()?
        } else {
            (u0 * (1.0 - t) + u1 * t).try_normalize()?
        };
        let geo = ellipsoid.fixed_to_geodetic(dir * ellipsoid.semi_major_m);
        let h_m = dem
            .elevation_above_ellipsoid_m(dem.body(), geo.lat_rad, geo.lon_rad, epoch)
            .ok()?;
        profile.push(ProfilePoint {
            distance_m: (idx as f64) * xi,
            height_m: h_m,
        });
    }
    Some(profile)
}

#[allow(clippy::too_many_arguments)]
fn resolve_terrain_context(
    source: AnalysisEndpoint<'_>,
    target: AnalysisEndpoint<'_>,
    sample: &EndpointLinkSample,
    models: &LinkBudgetModels<'_>,
    terrain_dem: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)>,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> TerrainContext {
    if !sample.accessible || (terrain_dem.is_none() && models.propagation.is_empty()) {
        return TerrainContext::EMPTY;
    }
    let dem_src = terrain_dem.map(|(dem, _)| dem);
    let emit_epoch = sample
        .light_time_s
        .map_or(sample.epoch, |lt| sample.epoch.advanced_seconds(-lt));
    let Ok(src_state) =
        evaluate_endpoint_world_state(&source, emit_epoch, dem_src, ephemeris, orientation)
    else {
        return TerrainContext::EMPTY;
    };
    let Ok(dst_state) =
        evaluate_endpoint_world_state(&target, sample.epoch, dem_src, ephemeris, orientation)
    else {
        return TerrainContext::EMPTY;
    };
    let source_geodetic = fixed_geodetic_of(&src_state, emit_epoch, ephemeris, orientation);
    let target_geodetic = fixed_geodetic_of(&dst_state, sample.epoch, ephemeris, orientation);
    let terrain_profile = match (terrain_dem, source_geodetic, target_geodetic) {
        (Some((dem, cfg)), Some(src_geo), Some(dst_geo))
            if src_state.host_body == dem.body() && dst_state.host_body == dem.body() =>
        {
            sample_arc_terrain_profile(
                &src_geo,
                &dst_geo,
                src_state.ellipsoid,
                dem,
                cfg,
                sample.epoch,
            )
        }
        _ => None,
    };
    TerrainContext {
        source_geodetic,
        target_geodetic,
        terrain_profile,
    }
}

fn path_of(
    sample: &EndpointLinkSample,
    link: &RfLinkSpec,
    models: &LinkBudgetModels<'_>,
    src_surface: bool,
    dst_surface: bool,
    terrain: TerrainContext,
) -> PathGeometry {
    PathGeometry {
        epoch: sample.epoch,
        carrier_frequency_hz: link.carrier_frequency_hz,
        source_position_icrf_m: sample.source_pos_icrf_m,
        target_position_icrf_m: sample.target_pos_icrf_m,
        slant_range_m: sample.slant_range_m,
        range_rate_m_s: sample.range_rate_m_s,
        light_time_s: sample.light_time_s,
        source_surface_elevation_rad: src_surface.then_some(sample.elevation_from_source_rad),
        target_surface_elevation_rad: dst_surface.then_some(sample.elevation_from_target_rad),
        polarisation: models.polarisation,
        exceedance: models.exceedance,
        source_geodetic: terrain.source_geodetic,
        target_geodetic: terrain.target_geodetic,
        terrain_profile: terrain.terrain_profile,
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
        .map(|sample| {
            let terrain = resolve_terrain_context(
                source,
                target,
                sample,
                models,
                terrain_dem,
                ephemeris,
                orientation,
            );
            budget_sample_with_terrain(sample, link, models, src_surface, dst_surface, terrain)
        })
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
    budget_sample_with_terrain(
        sample,
        link,
        models,
        src_surface,
        dst_surface,
        TerrainContext::EMPTY,
    )
}

fn budget_sample_with_terrain(
    sample: &EndpointLinkSample,
    link: &RfLinkSpec,
    models: &LinkBudgetModels<'_>,
    src_surface: bool,
    dst_surface: bool,
    terrain: TerrainContext,
) -> LinkBudgetSample {
    let outcome = if sample.accessible {
        let path = path_of(sample, link, models, src_surface, dst_surface, terrain);
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

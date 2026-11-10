//! General two-endpoint trajectory line-of-sight evaluation (`evaluate_endpoint_link_access`)
//! and Aircraft – Satellite – Ground Station simultaneous geometric relay window analysis
//! (`evaluate_geometric_relay_access`).

mod evaluator;
mod kinematics;
mod windows;

use sim_geo::BodyEllipsoid;
use sim_spacetime::{
    AttitudeTrajectorySource, BodyOrientationCatalog, EphemerisCatalog, NaifId, TdbEpoch,
    TranslationalTrajectorySource,
};

pub use sim_analysis_contract::{
    EndpointLinkReport, EndpointLinkSample, GeometricRelayReport, RelayAccessSample, RelayHopKind,
    RelayRestrictionEntry, VerificationScope,
};

use crate::{
    access::{
        AccessAnalysisError, StationAccessReport, WindowSearchCompleteness,
        append_environment_provenance, build_query_grid_epochs, evaluate_station_access,
    },
    models::{
        AccessQuerySpec, GroundStationModel, OccultationEnvironmentModel, VehicleSensorModel,
    },
    terrain::{AltitudeSpec, TerrainElevationSource, TerrainLosConfig},
};

use evaluator::{
    EndpointPairCtx, RelayEvalCtx, collect_endpoint_provenance, evaluate_pair_at_epoch,
    evaluate_relay_at_epoch, validate_endpoint, validate_query_and_terrain,
};
use windows::{extract_pair_windows, extract_relay_windows};

/// General endpoint in a two-endpoint or multi-hop geometric link analysis.
#[derive(Clone, Copy)]
pub enum AnalysisEndpoint<'a> {
    /// Planetary surface station (`GroundStationModel`) with optional vertical-datum altitude specification.
    SurfaceStation {
        /// Ground station model.
        station: &'a GroundStationModel,
        /// Optional vertical-datum altitude specification overriding `station.geodetic.height_m`.
        altitude_spec: Option<&'a AltitudeSpec>,
    },
    /// Moving vehicle or spacecraft (`SampledKinematicTrajectory`, `OrbitalTrajectoryProduct`, etc.).
    MovingPlatform {
        /// Unique numeric endpoint identifier.
        endpoint_id: u32,
        /// Human-readable platform label (e.g. `"Recon Aircraft"`, `"Relay LEO-1"`).
        label: &'a str,
        /// Primary reference body used for local horizon (`ENU`) and DEM terrain checks.
        host_body: NaifId,
        /// Reference ellipsoid of `host_body` (`BodyEllipsoid::WGS84`, etc.).
        ellipsoid: BodyEllipsoid,
        /// Translational trajectory source.
        trajectory: &'a dyn TranslationalTrajectorySource,
        /// Optional attitude trajectory source (mandatory when `sensor.require_attitude == true`).
        attitude: Option<&'a dyn AttitudeTrajectorySource>,
        /// Vehicle-mounted sensor or antenna model.
        sensor: &'a VehicleSensorModel,
        /// Optional minimum local topocentric elevation mask (`rad`) at this platform.
        min_local_elevation_rad: Option<f64>,
    },
}

impl<'a> AnalysisEndpoint<'a> {
    /// Returns the numeric identifier of this endpoint.
    #[must_use]
    pub const fn endpoint_id(&self) -> u32 {
        match *self {
            Self::SurfaceStation { station, .. } => station.station_id,
            Self::MovingPlatform { endpoint_id, .. } => endpoint_id,
        }
    }

    /// Returns the human-readable label of this endpoint.
    #[must_use]
    pub fn label(&self) -> &'a str {
        match *self {
            Self::SurfaceStation { station, .. } => &station.name,
            Self::MovingPlatform { label, .. } => label,
        }
    }
}

/// Evaluates general two-endpoint geometric line-of-sight, planetary body occultation,
/// optional local DEM terrain occultation, and endpoint sensor mount `FOV` constraints.
#[allow(clippy::too_many_arguments)]
pub fn evaluate_endpoint_link_access(
    source: AnalysisEndpoint<'_>,
    target: AnalysisEndpoint<'_>,
    occultation_env: &OccultationEnvironmentModel,
    terrain_dem: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)>,
    query: &AccessQuerySpec,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<EndpointLinkReport, AccessAnalysisError> {
    validate_query_and_terrain(query, terrain_dem)?;
    validate_endpoint(&source, query)?;
    validate_endpoint(&target, query)?;

    let ctx = EndpointPairCtx {
        source,
        target,
        occultation_env,
        terrain_dem,
        query,
        ephemeris,
        orientation,
    };

    let epochs = build_query_grid_epochs(query.query_interval, query.sample_step_s)?;
    let mut samples = Vec::with_capacity(epochs.len());
    for &ep in &epochs {
        samples.push(evaluate_pair_at_epoch(&ctx, ep)?);
    }

    let refine = query.boundary_time_tolerance_s < query.sample_step_s;
    let windows = extract_pair_windows(&ctx, &samples, refine)?;
    let completeness = completeness_for_query(query, refine);

    let mut provenance = Vec::new();
    collect_endpoint_provenance(&source, &mut provenance);
    collect_endpoint_provenance(&target, &mut provenance);
    append_environment_provenance(
        &mut provenance,
        NaifId::EARTH,
        query.query_interval,
        ephemeris,
        orientation,
    );

    Ok(EndpointLinkReport {
        source_endpoint_id: source.endpoint_id(),
        source_label: source.label().to_owned(),
        target_endpoint_id: target.endpoint_id(),
        target_label: target.label().to_owned(),
        valid_interval: query.query_interval,
        completeness,
        verification_scope: VerificationScope::geometric_visibility_only(),
        provenance,
        samples,
        windows,
    })
}

/// Evaluates simultaneous two-hop geometric relay (`Aircraft <-> Relay Satellite <-> Ground Station`)
/// plus optional direct `Aircraft <-> Ground Station` line of sight (including DEM terrain occultation).
#[allow(clippy::too_many_arguments)]
pub fn evaluate_geometric_relay_access<'a>(
    aircraft_for_satellite_hop: AnalysisEndpoint<'a>,
    satellite_for_aircraft_hop: AnalysisEndpoint<'a>,
    satellite_for_ground_hop: AnalysisEndpoint<'a>,
    ground_station: AnalysisEndpoint<'a>,
    aircraft_for_direct_hop: Option<AnalysisEndpoint<'a>>,
    occultation_env: &'a OccultationEnvironmentModel,
    terrain_dem: Option<(&'a dyn TerrainElevationSource, &'a TerrainLosConfig)>,
    query: &'a AccessQuerySpec,
    ephemeris: &'a dyn EphemerisCatalog,
    orientation: &'a dyn BodyOrientationCatalog,
) -> Result<GeometricRelayReport, AccessAnalysisError> {
    validate_query_and_terrain(query, terrain_dem)?;
    for ep in [
        &aircraft_for_satellite_hop,
        &satellite_for_aircraft_hop,
        &satellite_for_ground_hop,
        &ground_station,
    ] {
        validate_endpoint(ep, query)?;
    }
    if let Some(direct_air) = &aircraft_for_direct_hop {
        validate_endpoint(direct_air, query)?;
    }

    let make_ctx =
        |source: AnalysisEndpoint<'a>, target: AnalysisEndpoint<'a>| -> EndpointPairCtx<'a> {
            EndpointPairCtx {
                source,
                target,
                occultation_env,
                terrain_dem,
                query,
                ephemeris,
                orientation,
            }
        };
    let hop1_ctx = make_ctx(aircraft_for_satellite_hop, satellite_for_aircraft_hop);
    let hop2_ctx = make_ctx(satellite_for_ground_hop, ground_station);
    let direct_ctx = aircraft_for_direct_hop.map(|air| make_ctx(air, ground_station));
    let relay_ctx = RelayEvalCtx {
        hop1_ctx: &hop1_ctx,
        hop2_ctx: &hop2_ctx,
        direct_ctx: direct_ctx.as_ref(),
        query,
    };
    build_relay_report(
        &relay_ctx,
        &aircraft_for_satellite_hop,
        &satellite_for_aircraft_hop,
        &ground_station,
        query,
        ephemeris,
        orientation,
    )
}

fn build_relay_report(
    relay_ctx: &RelayEvalCtx<'_>,
    aircraft_for_satellite_hop: &AnalysisEndpoint<'_>,
    satellite_for_aircraft_hop: &AnalysisEndpoint<'_>,
    ground_station: &AnalysisEndpoint<'_>,
    query: &AccessQuerySpec,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<GeometricRelayReport, AccessAnalysisError> {
    let epochs = build_query_grid_epochs(query.query_interval, query.sample_step_s)?;
    let (samples, hop1_samples, hop2_samples, direct_samples) =
        sample_relay_grid(relay_ctx, &epochs)?;
    let refine = query.boundary_time_tolerance_s < query.sample_step_s;
    let relay_windows = extract_relay_windows(relay_ctx, &samples, refine)?;
    let hop1_windows = extract_pair_windows(relay_ctx.hop1_ctx, &hop1_samples, refine)?;
    let hop2_windows = extract_pair_windows(relay_ctx.hop2_ctx, &hop2_samples, refine)?;
    let direct_windows = relay_ctx
        .direct_ctx
        .map(|d| extract_pair_windows(d, &direct_samples, refine))
        .transpose()?
        .unwrap_or_default();

    let mut provenance = Vec::new();
    collect_endpoint_provenance(aircraft_for_satellite_hop, &mut provenance);
    collect_endpoint_provenance(satellite_for_aircraft_hop, &mut provenance);
    append_environment_provenance(
        &mut provenance,
        NaifId::EARTH,
        query.query_interval,
        ephemeris,
        orientation,
    );

    Ok(GeometricRelayReport {
        aircraft_endpoint_id: aircraft_for_satellite_hop.endpoint_id(),
        satellite_endpoint_id: satellite_for_aircraft_hop.endpoint_id(),
        ground_station_id: ground_station.endpoint_id(),
        valid_interval: query.query_interval,
        completeness: completeness_for_query(query, refine),
        verification_scope: VerificationScope::geometric_visibility_only(),
        provenance,
        samples,
        relay_windows,
        hop1_windows,
        hop2_windows,
        direct_windows,
    })
}

type RelayGridSamples = (
    Vec<RelayAccessSample>,
    Vec<EndpointLinkSample>,
    Vec<EndpointLinkSample>,
    Vec<EndpointLinkSample>,
);

fn sample_relay_grid(
    relay_ctx: &RelayEvalCtx<'_>,
    epochs: &[TdbEpoch],
) -> Result<RelayGridSamples, AccessAnalysisError> {
    let mut samples = Vec::with_capacity(epochs.len());
    let mut hop1_samples = Vec::with_capacity(epochs.len());
    let mut hop2_samples = Vec::with_capacity(epochs.len());
    let mut direct_samples = Vec::new();
    for &ep in epochs {
        let s = evaluate_relay_at_epoch(relay_ctx, ep)?;
        hop1_samples.push(s.hop1_aircraft_to_satellite.clone());
        hop2_samples.push(s.hop2_satellite_to_ground.clone());
        if let Some(d) = &s.direct_aircraft_to_ground {
            direct_samples.push(d.clone());
        }
        samples.push(s);
    }
    Ok((samples, hop1_samples, hop2_samples, direct_samples))
}

const fn completeness_for_query(query: &AccessQuerySpec, refine: bool) -> WindowSearchCompleteness {
    if refine {
        WindowSearchCompleteness::BracketRefinedOnGrid {
            sample_step_s: query.sample_step_s,
            boundary_tolerance_s: query.boundary_time_tolerance_s,
            may_miss_subgrid_windows_shorter_than_s: query.sample_step_s,
        }
    } else {
        WindowSearchCompleteness::SampledGridOnly {
            sample_step_s: query.sample_step_s,
        }
    }
}

/// Multi-analysis suite evaluated over a single shared materialized vehicle trajectory
/// (`TranslationalTrajectorySource` + `AttitudeTrajectorySource`), combining:
/// 1. Omnidirectional ground-station visibility (`station_visibility`),
/// 2. Directional vehicle sensor `FOV` + DEM link access (`sensor_fov_link`), and
/// 3. Optional multi-hop satellite relay + direct link (`relay_access`).
#[derive(Clone, Debug, PartialEq)]
pub struct SharedTrajectoryAccessSuite {
    /// Ground-station visibility report using an omnidirectional vehicle reference sensor.
    pub station_visibility: StationAccessReport,
    /// Two-endpoint link report enforcing the directional vehicle sensor `FOV` and optional DEM terrain LOS.
    pub sensor_fov_link: EndpointLinkReport,
    /// Optional two-hop `Aircraft <-> Satellite <-> Ground Station` relay report reusing the same trajectory.
    pub relay_access: Option<GeometricRelayReport>,
}

/// Evaluates station visibility, directional sensor `FOV` link access, and optional satellite relay
/// access over the **same** materialized platform trajectory without re-sampling or duplicating trajectory state.
#[allow(clippy::too_many_arguments)]
pub fn evaluate_shared_trajectory_access_suite<'a>(
    endpoint_id: u32,
    platform_label: &'a str,
    host_body: NaifId,
    ellipsoid: BodyEllipsoid,
    trajectory: &'a dyn TranslationalTrajectorySource,
    attitude: Option<&'a dyn AttitudeTrajectorySource>,
    directional_sensor: &'a VehicleSensorModel,
    station: &'a GroundStationModel,
    station_altitude_spec: Option<&'a AltitudeSpec>,
    relay_satellite: Option<(
        AnalysisEndpoint<'a>,
        AnalysisEndpoint<'a>,
        &'a VehicleSensorModel,
    )>,
    occultation_env: &'a OccultationEnvironmentModel,
    terrain_dem: Option<(&'a dyn TerrainElevationSource, &'a TerrainLosConfig)>,
    query: &'a AccessQuerySpec,
    ephemeris: &'a dyn EphemerisCatalog,
    orientation: &'a dyn BodyOrientationCatalog,
) -> Result<SharedTrajectoryAccessSuite, AccessAnalysisError> {
    let omni_sensor = VehicleSensorModel::omnidirectional_at_cg(0, "Omni-Visibility-Ref");
    let station_visibility = evaluate_station_access(
        trajectory,
        attitude,
        station,
        &omni_sensor,
        occultation_env,
        query,
        ephemeris,
        orientation,
    )?;
    let make_platform = |sensor: &'a VehicleSensorModel| AnalysisEndpoint::MovingPlatform {
        endpoint_id,
        label: platform_label,
        host_body,
        ellipsoid,
        trajectory,
        attitude,
        sensor,
        min_local_elevation_rad: None,
    };
    let platform_ep = make_platform(directional_sensor);
    let station_ep = AnalysisEndpoint::SurfaceStation {
        station,
        altitude_spec: station_altitude_spec,
    };
    let sensor_fov_link = evaluate_endpoint_link_access(
        platform_ep,
        station_ep,
        occultation_env,
        terrain_dem,
        query,
        ephemeris,
        orientation,
    )?;
    let relay_access = relay_satellite
        .map(|(sat_to_air_ep, sat_to_gnd_ep, air_to_sat_sensor)| {
            evaluate_geometric_relay_access(
                make_platform(air_to_sat_sensor),
                sat_to_air_ep,
                sat_to_gnd_ep,
                station_ep,
                Some(platform_ep),
                occultation_env,
                terrain_dem,
                query,
                ephemeris,
                orientation,
            )
        })
        .transpose()?;
    Ok(SharedTrajectoryAccessSuite {
        station_visibility,
        sensor_fov_link,
        relay_access,
    })
}

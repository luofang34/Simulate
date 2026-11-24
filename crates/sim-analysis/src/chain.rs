//! General N-hop communication and visibility chains ($E_0 \to E_1 \to \dots \to E_M$, $M \ge 1$)
//! with forward/backward causal iterative light-time propagation, planetary/DEM occultation,
//! sensor boresight/elevation constraints, Doppler composition, and per-hop RF link budgets.

mod solver;
mod windows;

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_analysis_contract::{
    BudgetUnavailableReason, LinkBudget, RfLinkSpec, VerificationScope, serde_dvec3,
};
use sim_geo::BodyEllipsoid;
use sim_spacetime::{
    AttitudeTrajectorySource, BodyOrientationCatalog, DataProvenance, EphemerisCatalog, NaifId,
    TdbEpoch, TranslationalTrajectorySource, ValidEpochInterval,
};
use thiserror::Error;

use crate::{
    access::{
        AccessAnalysisError, AccessRestrictionReason, AccessWindow, WindowSearchCompleteness,
    },
    models::{
        GroundStationModel, OccultationEnvironmentModel, SignalTimeModel, VehicleSensorModel,
    },
    relay::AnalysisEndpoint,
    signal::LinkBudgetModels,
    terrain::{AltitudeSpec, TerrainElevationSource, TerrainLosConfig},
};

use solver::{evaluate_chain_epoch_internal, validate_chain_inputs};
use windows::{run_chain_windows, validate_window_sampling_params};

/// Direction along an N-hop chain in which causal light-time epochs are propagated.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ChainPropagationDirection {
    /// Query `epoch` is the reception epoch at the final node $E_M$; each hop solves
    /// retarded emission time $t_k = t_{k+1} - \tau_k$ backward from $E_M$ to $E_0$.
    #[default]
    BackwardFromRx,
    /// Query `epoch` is the transmission epoch at the initial node $E_0$; each hop solves
    /// advanced reception time $t_{k+1} = t_k + \tau_k$ forward from $E_0$ to $E_M$.
    ForwardFromTx,
}

/// Moving vehicle or spacecraft node in an N-hop communication chain.
#[derive(Clone, Copy)]
pub struct ChainAssetNode<'a> {
    /// Unique numeric endpoint identifier.
    pub endpoint_id: u32,
    /// Human-readable node label (e.g. `"Mars-Relay-1"`).
    pub label: &'a str,
    /// Host body for local topocentric horizon and DEM calculations.
    pub host_body: NaifId,
    /// Reference ellipsoid of `host_body`.
    pub ellipsoid: BodyEllipsoid,
    /// Translational trajectory source (`SiMetric`, `IcrfInertial`).
    pub trajectory: &'a dyn TranslationalTrajectorySource,
    /// Optional attitude trajectory source (required when `antenna.require_attitude == true`).
    pub attitude: Option<&'a dyn AttitudeTrajectorySource>,
    /// Vehicle-mounted antenna/sensor model.
    pub antenna: &'a VehicleSensorModel,
    /// Optional minimum local topocentric elevation mask (`rad`).
    pub min_local_elevation_rad: Option<f64>,
}

impl<'a> ChainAssetNode<'a> {
    /// Creates a [`ChainAssetNode`] with `min_local_elevation_rad = None`.
    #[must_use]
    pub const fn new(
        endpoint_id: u32,
        label: &'a str,
        host_body: NaifId,
        ellipsoid: BodyEllipsoid,
        trajectory: &'a dyn TranslationalTrajectorySource,
        attitude: Option<&'a dyn AttitudeTrajectorySource>,
        antenna: &'a VehicleSensorModel,
    ) -> Self {
        Self {
            endpoint_id,
            label,
            host_body,
            ellipsoid,
            trajectory,
            attitude,
            antenna,
            min_local_elevation_rad: None,
        }
    }
}

/// One node in an N-hop causal link chain ($E_0 \to E_1 \to \dots \to E_M$).
#[derive(Clone, Copy)]
pub enum ChainNodeSpec<'a> {
    /// Fixed planetary surface station with optional vertical-datum altitude override.
    GroundStation {
        /// Ground station model.
        station: &'a GroundStationModel,
        /// Optional vertical-datum altitude specification.
        altitude_spec: Option<&'a AltitudeSpec>,
    },
    /// Moving spacecraft or atmospheric platform node.
    Asset(ChainAssetNode<'a>),
}

impl<'a> ChainNodeSpec<'a> {
    /// Wraps a [`GroundStationModel`] without an [`AltitudeSpec`] override.
    #[must_use]
    pub const fn ground_station(station: &'a GroundStationModel) -> Self {
        Self::GroundStation {
            station,
            altitude_spec: None,
        }
    }

    /// Converts an [`AnalysisEndpoint`] into a [`ChainNodeSpec`].
    #[must_use]
    pub const fn from_endpoint(endpoint: AnalysisEndpoint<'a>) -> Self {
        match endpoint {
            AnalysisEndpoint::SurfaceStation {
                station,
                altitude_spec,
            } => Self::GroundStation {
                station,
                altitude_spec,
            },
            AnalysisEndpoint::MovingPlatform {
                endpoint_id,
                label,
                host_body,
                ellipsoid,
                trajectory,
                attitude,
                sensor,
                min_local_elevation_rad,
            } => Self::Asset(ChainAssetNode {
                endpoint_id,
                label,
                host_body,
                ellipsoid,
                trajectory,
                attitude,
                antenna: sensor,
                min_local_elevation_rad,
            }),
        }
    }

    /// Views this node as an [`AnalysisEndpoint`] for shared kinematic and occultation checks.
    #[must_use]
    pub const fn as_analysis_endpoint(&self) -> AnalysisEndpoint<'a> {
        match *self {
            Self::GroundStation {
                station,
                altitude_spec,
            } => AnalysisEndpoint::SurfaceStation {
                station,
                altitude_spec,
            },
            Self::Asset(asset) => AnalysisEndpoint::MovingPlatform {
                endpoint_id: asset.endpoint_id,
                label: asset.label,
                host_body: asset.host_body,
                ellipsoid: asset.ellipsoid,
                trajectory: asset.trajectory,
                attitude: asset.attitude,
                sensor: asset.antenna,
                min_local_elevation_rad: asset.min_local_elevation_rad,
            },
        }
    }
}

/// Optional RF link specification and antenna/propagation models attached to a single hop.
pub struct ChainHopRfSpec<'a> {
    /// Radio link specification for this hop.
    pub link: &'a RfLinkSpec,
    /// Optional custom antenna/propagation models; defaults to [`LinkBudgetModels::from_spec`] when `None`.
    pub models: Option<LinkBudgetModels<'a>>,
}

impl<'a> ChainHopRfSpec<'a> {
    /// Attaches an [`RfLinkSpec`] using [`LinkBudgetModels::from_spec`].
    #[must_use]
    pub const fn from_link(link: &'a RfLinkSpec) -> Self {
        Self { link, models: None }
    }

    /// Attaches an [`RfLinkSpec`] with custom [`LinkBudgetModels`].
    #[must_use]
    pub const fn with_models(link: &'a RfLinkSpec, models: LinkBudgetModels<'a>) -> Self {
        Self {
            link,
            models: Some(models),
        }
    }
}

/// Query configuration for N-hop chain evaluation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChainQuerySpec {
    /// Causal light-time propagation direction (`BackwardFromRx` or `ForwardFromTx`).
    pub propagation_direction: ChainPropagationDirection,
    /// Signal time model (`Geometric` or `LightTime`).
    pub signal_time: SignalTimeModel,
    /// Celestial bodies checked for line-of-sight occultation on every hop.
    pub occultation_env: OccultationEnvironmentModel,
    /// Optional minimum slant range constraint per hop (`m`).
    pub min_range_m: Option<f64>,
    /// Optional maximum slant range constraint per hop (`m`).
    pub max_range_m: Option<f64>,
    /// Fixed-point light-time convergence tolerance (`s`, `> 0`, default `1e-9`).
    pub light_time_tolerance_s: f64,
    /// Maximum fixed-point light-time iterations per hop (default `16`).
    pub max_light_time_iterations: usize,
}

impl ChainQuerySpec {
    /// Default light-time solver tolerance (`1 ns`).
    pub const DEFAULT_LIGHT_TIME_TOLERANCE_S: f64 = 1.0e-9;
    /// Default maximum fixed-point light-time iterations per hop.
    pub const DEFAULT_MAX_LIGHT_TIME_ITERATIONS: usize = 16;

    /// Builds a causal [`SignalTimeModel::LightTime`] chain query with default solver tolerances.
    #[must_use]
    pub const fn light_time(
        occultation_env: OccultationEnvironmentModel,
        propagation_direction: ChainPropagationDirection,
    ) -> Self {
        Self {
            propagation_direction,
            signal_time: SignalTimeModel::LightTime,
            occultation_env,
            min_range_m: None,
            max_range_m: None,
            light_time_tolerance_s: Self::DEFAULT_LIGHT_TIME_TOLERANCE_S,
            max_light_time_iterations: Self::DEFAULT_MAX_LIGHT_TIME_ITERATIONS,
        }
    }

    /// Builds an instantaneous [`SignalTimeModel::Geometric`] chain query.
    #[must_use]
    pub const fn geometric(occultation_env: OccultationEnvironmentModel) -> Self {
        Self {
            propagation_direction: ChainPropagationDirection::BackwardFromRx,
            signal_time: SignalTimeModel::Geometric,
            occultation_env,
            min_range_m: None,
            max_range_m: None,
            light_time_tolerance_s: Self::DEFAULT_LIGHT_TIME_TOLERANCE_S,
            max_light_time_iterations: Self::DEFAULT_MAX_LIGHT_TIME_ITERATIONS,
        }
    }
}

/// Errors returned by N-hop chain validation and evaluation.
#[derive(Clone, Debug, PartialEq, Error, Serialize, Deserialize)]
pub enum ChainEvaluationError {
    /// Fewer than 2 nodes were provided (`0` hops).
    #[error("chain requires at least 2 nodes (1 hop), got {node_count} node(s)")]
    TooFewNodes {
        /// Supplied node count.
        node_count: usize,
    },
    /// Supplied `hop_rf` slice length differs from `nodes.len() - 1`.
    #[error("hop RF specification count ({actual}) does not match hop count ({expected})")]
    HopRfSpecCountMismatch {
        /// Expected hop count (`nodes.len() - 1`).
        expected: usize,
        /// Actual supplied RF slice length.
        actual: usize,
    },
    /// Light-time convergence tolerance or iteration cap is non-positive or non-finite.
    #[error(
        "invalid light-time solver parameters: tolerance_s={tolerance_s}, max_iterations={max_iterations}"
    )]
    InvalidLightTimeParameters {
        /// Supplied tolerance (`s`).
        tolerance_s: f64,
        /// Supplied iteration cap.
        max_iterations: usize,
    },
    /// Range constraint bounds are negative, non-finite, or inverted.
    #[error("invalid range bounds: min_range_m={min_range_m:?}, max_range_m={max_range_m:?}")]
    InvalidRangeBounds {
        /// Minimum slant range (`m`).
        min_range_m: Option<f64>,
        /// Maximum slant range (`m`).
        max_range_m: Option<f64>,
    },
    /// Fixed-point light-time solver did not converge within `max_iterations`.
    #[error(
        "hop {hop_index} light-time solver did not converge within {max_iterations} iteration(s) (last delta {last_delta_s:.3e} s > {tolerance_s:.3e} s)"
    )]
    LightTimeDidNotConverge {
        /// Zero-based hop index that failed to converge.
        hop_index: usize,
        /// Maximum configured iterations.
        max_iterations: usize,
        /// Last fixed-point step delta (`s`).
        last_delta_s: f64,
        /// Configured convergence tolerance (`s`).
        tolerance_s: f64,
    },
    /// Underlying trajectory, attitude, ephemeris, orientation, RF, or terrain error.
    #[error("access evaluation error: {0}")]
    Access(#[from] AccessAnalysisError),
}

/// Evaluation outcome for one hop $E_k \to E_{k+1}$ inside an N-hop chain.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HopEvaluation {
    /// Zero-based hop index (`0..hops.len()`).
    pub hop_index: usize,
    /// Transmitting endpoint numeric identifier.
    pub source_endpoint_id: u32,
    /// Receiving endpoint numeric identifier.
    pub target_endpoint_id: u32,
    /// Causal transmission epoch at `source` (`TDB`).
    pub tx_epoch: TdbEpoch,
    /// Causal reception epoch at `target` (`TDB`).
    pub rx_epoch: TdbEpoch,
    /// One-way light time along this hop (`s`; `0.0` for `SignalTimeModel::Geometric`).
    pub light_time_s: f64,
    /// Euclidean slant range between source at `tx_epoch` and target at `rx_epoch` (`m`).
    pub slant_range_m: f64,
    /// Line-of-sight range rate (`m/s`, positive when receding).
    pub range_rate_m_s: f64,
    /// First-order Doppler frequency multiplier $1 - \dot{\rho}/c$.
    pub doppler_factor: f64,
    /// Source antenna phase-center position at `tx_epoch` in `SSB ICRF` (`m`).
    #[serde(with = "serde_dvec3")]
    pub source_pos_icrf_m: DVec3,
    /// Target antenna phase-center position at `rx_epoch` in `SSB ICRF` (`m`).
    #[serde(with = "serde_dvec3")]
    pub target_pos_icrf_m: DVec3,
    /// Local horizon elevation angle from source toward target (`rad`).
    pub elevation_from_source_rad: f64,
    /// Local horizon elevation angle from target toward source (`rad`).
    pub elevation_from_target_rad: f64,
    /// Off-boresight angle in source mount (`rad`, if mount orientation is defined).
    pub source_off_boresight_rad: Option<f64>,
    /// Off-boresight angle in target mount (`rad`, if mount orientation is defined).
    pub target_off_boresight_rad: Option<f64>,
    /// `true` iff every elevation, FOV, range, body occultation, and terrain constraint holds.
    pub accessible: bool,
    /// Active restriction reasons blocking this hop (`empty` iff `accessible == true`).
    pub active_restrictions: Vec<AccessRestrictionReason>,
    /// Full RF link budget when RF specs are attached and the hop is accessible.
    pub link_budget: Option<LinkBudget>,
    /// Achieved $E_b/N_0$ (`dB`) when `link_budget` is present.
    pub eb_n0_db: Option<f64>,
    /// Link margin $E_b/N_0 - (E_b/N_0)_{\text{req}}$ (`dB`) when `link_budget` is present.
    pub margin_db: Option<f64>,
    /// Reason no link budget was produced when RF specs are attached.
    pub budget_unavailable: Option<BudgetUnavailableReason>,
}

/// Complete causal evaluation of an N-hop chain at a single anchor epoch.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChainEvaluationReport {
    /// Anchor epoch at which the chain was queried (`rx_epoch` if `BackwardFromRx`, `tx_epoch` if `ForwardFromTx`).
    pub query_epoch: TdbEpoch,
    /// Causal light-time propagation direction.
    pub propagation_direction: ChainPropagationDirection,
    /// Signal time model used.
    pub signal_time: SignalTimeModel,
    /// `true` iff every hop in the chain is geometrically accessible.
    pub end_to_end_accessible: bool,
    /// Transmission epoch at initial node $E_0$ (`TDB`).
    pub tx_epoch: TdbEpoch,
    /// Reception epoch at final node $E_M$ (`TDB`).
    pub rx_epoch: TdbEpoch,
    /// Cumulative end-to-end light time $\sum_k \tau_k$ (`s`).
    pub total_light_time_s: f64,
    /// Composite coherent Doppler frequency multiplier $\prod_k (1 - \dot{\rho}_k / c)$.
    pub net_doppler_factor: f64,
    /// Bottleneck RF margin $\min_k (\text{margin}_k)$ (`dB`) when `end_to_end_accessible` and RF specs are attached.
    pub min_margin_db: Option<f64>,
    /// Per-hop evaluations ordered along $E_0 \to E_1 \to \dots \to E_M$.
    pub hops: Vec<HopEvaluation>,
    /// Lowest zero-based hop index that failed geometric accessibility, or `None` when accessible.
    pub blocking_hop: Option<usize>,
}

/// Time-series N-hop chain report with continuous end-to-end [`AccessWindow`]s.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChainWindowReport {
    /// Evaluated `TDB` query interval (in the anchor clock defined by `propagation_direction`).
    pub valid_interval: ValidEpochInterval,
    /// Search completeness declaration.
    pub completeness: WindowSearchCompleteness,
    /// Explicit verification scope.
    pub verification_scope: VerificationScope,
    /// Provenance of node trajectories, ephemeris, and orientation catalogs.
    pub provenance: Vec<DataProvenance>,
    /// Uniform grid samples across `valid_interval`.
    pub samples: Vec<ChainEvaluationReport>,
    /// Continuous end-to-end access windows where all hops are simultaneously accessible.
    pub windows: Vec<AccessWindow>,
}

/// Evaluates an N-hop chain ($E_0 \to E_1 \to \dots \to E_M$) with causal light-time propagation at `epoch`.
pub fn evaluate_chain_at_epoch(
    nodes: &[ChainNodeSpec<'_>],
    hop_rf: Option<&[Option<ChainHopRfSpec<'_>>]>,
    epoch: TdbEpoch,
    query: &ChainQuerySpec,
    terrain_dem: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)>,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<ChainEvaluationReport, ChainEvaluationError> {
    validate_chain_inputs(nodes, hop_rf, query, terrain_dem)?;
    evaluate_chain_epoch_internal(
        nodes,
        hop_rf,
        epoch,
        query,
        terrain_dem,
        ephemeris,
        orientation,
    )
}

/// Samples an N-hop chain over `query_interval` and extracts continuous end-to-end access windows.
#[allow(clippy::too_many_arguments)]
pub fn evaluate_chain_windows(
    nodes: &[ChainNodeSpec<'_>],
    hop_rf: Option<&[Option<ChainHopRfSpec<'_>>]>,
    query_interval: ValidEpochInterval,
    sample_step_s: f64,
    boundary_time_tolerance_s: f64,
    query: &ChainQuerySpec,
    terrain_dem: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)>,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<ChainWindowReport, ChainEvaluationError> {
    validate_window_sampling_params(query_interval, sample_step_s, boundary_time_tolerance_s)?;
    validate_chain_inputs(nodes, hop_rf, query, terrain_dem)?;
    run_chain_windows(
        nodes,
        hop_rf,
        query_interval,
        sample_step_s,
        boundary_time_tolerance_s,
        query,
        terrain_dem,
        ephemeris,
        orientation,
    )
}

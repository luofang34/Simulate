//! Terrestrial and reference propagation loss adapters backed by `world-propagation`.

mod antenna;
mod terrain_models;
#[cfg(test)]
mod tests;

pub use antenna::{F1245Pattern, MainBeamPolicy, S465Pattern, S580Pattern};
pub use terrain_models::{DeltaBullingtonPropagationLoss, ItmPropagationLoss};

use sim_geo::{BodyEllipsoid, Geodetic};
use sim_spacetime::acos_clamped;
use world_propagation::p526::{
    DEFAULT_EFFECTIVE_EARTH_RADIUS_M, KnifeEdge, KnifeEdgeGeometry, KnifeEdgeLoss, KnifeEdgeMethod,
    SmoothEarth, SmoothEarthLoss, TerminalPath, diffraction_parameter,
};
use world_propagation::{Frequency, Ground, Loss, PathLoss, Polarisation, ProfilePoint, Signal};

use super::path::{PathGeometry, PropagationLoss, PropagationLossError};

pub(super) fn signal_from_path(
    model_name: &str,
    path: &PathGeometry,
    default_polarisation: Polarisation,
) -> Result<Signal, PropagationLossError> {
    let frequency = Frequency::from_hz(path.carrier_frequency_hz).map_err(|source| {
        PropagationLossError::Model {
            model: model_name.to_owned(),
            source,
        }
    })?;
    Ok(Signal {
        frequency,
        polarisation: path.polarisation.unwrap_or(default_polarisation),
    })
}

fn great_circle_surface_distance_m(src: &Geodetic, dst: &Geodetic) -> f64 {
    let p0 = BodyEllipsoid::WGS84.geodetic_to_fixed(&Geodetic {
        lat_rad: src.lat_rad,
        lon_rad: src.lon_rad,
        height_m: 0.0,
    });
    let p1 = BodyEllipsoid::WGS84.geodetic_to_fixed(&Geodetic {
        lat_rad: dst.lat_rad,
        lon_rad: dst.lon_rad,
        height_m: 0.0,
    });
    let psi = acos_clamped(p0.normalize().dot(p1.normalize()));
    let r_mean = 0.5 * (p0.length() + p1.length());
    psi * r_mean
}

pub(super) fn require_terrain_profile<'a>(
    model_name: &str,
    path: &'a PathGeometry,
) -> Result<(&'a [ProfilePoint], ProfilePoint, ProfilePoint), PropagationLossError> {
    let profile =
        path.terrain_profile
            .as_deref()
            .ok_or_else(|| PropagationLossError::MissingPathData {
                model: model_name.to_owned(),
                field: "terrain_profile",
            })?;
    let (Some(first), Some(last)) = (profile.first().copied(), profile.last().copied()) else {
        return Err(PropagationLossError::MissingPathData {
            model: model_name.to_owned(),
            field: "terrain_profile",
        });
    };
    Ok((profile, first, last))
}

pub(super) fn resolve_heights_above_ground(
    model_name: &str,
    path: &PathGeometry,
    first_ground_m: f64,
    last_ground_m: f64,
    default_tx_height_m: Option<f64>,
    default_rx_height_m: Option<f64>,
) -> Result<(f64, f64), PropagationLossError> {
    let tx_height_m = path
        .source_geodetic
        .map(|g| (g.height_m - first_ground_m).max(0.0))
        .or(default_tx_height_m)
        .ok_or_else(|| PropagationLossError::MissingPathData {
            model: model_name.to_owned(),
            field: "source_geodetic",
        })?;
    let rx_height_m = path
        .target_geodetic
        .map(|g| (g.height_m - last_ground_m).max(0.0))
        .or(default_rx_height_m)
        .ok_or_else(|| PropagationLossError::MissingPathData {
            model: model_name.to_owned(),
            field: "target_geodetic",
        })?;
    Ok((tx_height_m, rx_height_m))
}

/// Source of knife-edge obstacle geometry for [`KnifeEdgePropagationLoss`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum KnifeEdgeSource {
    /// Extract the dominant Fresnel obstruction (`argmax nu`) from `PathGeometry::terrain_profile`.
    TerrainProfile {
        /// Effective Earth radius for Earth-curvature bulge (`m`, typically `8_500_000.0`).
        effective_earth_radius_m: f64,
    },
    /// Evaluate a caller-supplied single knife-edge obstacle across any path frequency.
    Fixed(KnifeEdgeGeometry),
}

/// ITU-R P.526-16 single knife-edge diffraction loss adapter behind [`PropagationLoss`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KnifeEdgePropagationLoss {
    method: KnifeEdgeMethod,
    source: KnifeEdgeSource,
}

impl KnifeEdgePropagationLoss {
    /// Derives the dominant single knife-edge obstacle from `PathGeometry::terrain_profile`.
    #[must_use]
    pub const fn from_terrain_profile(method: KnifeEdgeMethod) -> Self {
        Self {
            method,
            source: KnifeEdgeSource::TerrainProfile {
                effective_earth_radius_m: DEFAULT_EFFECTIVE_EARTH_RADIUS_M,
            },
        }
    }

    /// Evaluates an explicit [`KnifeEdgeGeometry`] obstacle at the path's carrier frequency.
    #[must_use]
    pub const fn from_geometry(method: KnifeEdgeMethod, geometry: KnifeEdgeGeometry) -> Self {
        Self {
            method,
            source: KnifeEdgeSource::Fixed(geometry),
        }
    }

    /// Overrides the effective Earth radius used when extracting curvature bulge from a terrain profile.
    #[must_use]
    pub const fn with_effective_earth_radius_m(mut self, ae_m: f64) -> Self {
        if let KnifeEdgeSource::TerrainProfile { .. } = self.source {
            self.source = KnifeEdgeSource::TerrainProfile {
                effective_earth_radius_m: ae_m,
            };
        }
        self
    }

    /// Evaluates the knife-edge diffraction loss and dimensionless parameter `nu`.
    ///
    /// # Errors
    ///
    /// Returns [`PropagationLossError`] if terrain profile inputs are missing or model parameters are invalid.
    pub fn evaluate(&self, path: &PathGeometry) -> Result<KnifeEdgeLoss, PropagationLossError> {
        let name = self.name();
        let signal = signal_from_path(&name, path, Polarisation::Vertical)?;
        let geometry = match self.source {
            KnifeEdgeSource::Fixed(g) => g,
            KnifeEdgeSource::TerrainProfile {
                effective_earth_radius_m,
            } => dominant_edge_from_profile(&name, path, effective_earth_radius_m, signal)?,
        };
        KnifeEdge {
            method: self.method,
        }
        .path_loss(&geometry, &signal)
        .map_err(|source| PropagationLossError::Model {
            model: name,
            source,
        })
    }
}

fn dominant_edge_from_profile(
    model_name: &str,
    path: &PathGeometry,
    effective_earth_radius_m: f64,
    signal: Signal,
) -> Result<KnifeEdgeGeometry, PropagationLossError> {
    if !effective_earth_radius_m.is_finite() || effective_earth_radius_m <= 0.0 {
        return Err(PropagationLossError::Model {
            model: model_name.to_owned(),
            source: world_propagation::PropagationError::InvalidParameter {
                name: "effective_earth_radius_m",
                value: effective_earth_radius_m,
            },
        });
    }
    let (profile, first, last) = require_terrain_profile(model_name, path)?;
    if profile.len() < 3 {
        return Err(PropagationLossError::MissingPathData {
            model: model_name.to_owned(),
            field: "terrain_profile",
        });
    }
    let total_dist_m = last.distance_m - first.distance_m;
    if !total_dist_m.is_finite() || total_dist_m <= 0.0 {
        return Err(PropagationLossError::MissingPathData {
            model: model_name.to_owned(),
            field: "terrain_profile",
        });
    }
    let h_tx = path.source_geodetic.map_or(first.height_m, |g| g.height_m);
    let h_rx = path.target_geodetic.map_or(last.height_m, |g| g.height_m);
    let wavelength_m = signal.frequency.wavelength_m();

    let mut best: Option<(f64, KnifeEdgeGeometry)> = None;
    for pt in profile {
        let d1 = pt.distance_m - first.distance_m;
        let d2 = last.distance_m - pt.distance_m;
        if d1 <= 0.0 || d2 <= 0.0 {
            continue;
        }
        let h_los = h_tx + (h_rx - h_tx) * (d1 / total_dist_m);
        let bulge = (d1 * d2) / (2.0 * effective_earth_radius_m);
        let height_m = pt.height_m + bulge - h_los;
        let nu = diffraction_parameter(height_m, d1, d2, wavelength_m).map_err(|source| {
            PropagationLossError::Model {
                model: model_name.to_owned(),
                source,
            }
        })?;
        if best.is_none_or(|(best_nu, _)| nu > best_nu) {
            best = Some((
                nu,
                KnifeEdgeGeometry {
                    height_m,
                    d1_m: d1,
                    d2_m: d2,
                },
            ));
        }
    }
    best.map(|(_, geom)| geom)
        .ok_or_else(|| PropagationLossError::MissingPathData {
            model: model_name.to_owned(),
            field: "terrain_profile",
        })
}

impl PropagationLoss for KnifeEdgePropagationLoss {
    fn name(&self) -> String {
        "itu-p526-knife-edge".to_owned()
    }

    fn loss_db(&self, path: &PathGeometry) -> Result<f64, PropagationLossError> {
        self.evaluate(path).map(|out| out.loss_db())
    }
}

/// ITU-R P.526-16 smooth spherical Earth diffraction loss adapter behind [`PropagationLoss`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SmoothEarthPropagationLoss {
    inner: SmoothEarth,
    default_polarisation: Polarisation,
}

impl SmoothEarthPropagationLoss {
    /// Builds a smooth spherical Earth model with `effective_earth_radius_m` and surface `ground`.
    ///
    /// # Errors
    ///
    /// Returns `PropagationError` if `effective_earth_radius_m` or `ground` constants are invalid.
    pub fn new(
        effective_earth_radius_m: f64,
        ground: Ground,
    ) -> Result<Self, world_propagation::PropagationError> {
        Ok(Self {
            inner: SmoothEarth::new(effective_earth_radius_m, ground)?,
            default_polarisation: Polarisation::Vertical,
        })
    }

    /// Overrides the fallback polarisation used when `PathGeometry::polarisation` is `None`.
    #[must_use]
    pub const fn with_default_polarisation(mut self, polarisation: Polarisation) -> Self {
        self.default_polarisation = polarisation;
        self
    }

    /// Evaluates smooth-Earth diffraction loss and line-of-sight horizon flag.
    ///
    /// # Errors
    ///
    /// Returns [`PropagationLossError`] if sites/heights are absent or model inputs are out of range.
    pub fn evaluate(&self, path: &PathGeometry) -> Result<SmoothEarthLoss, PropagationLossError> {
        let name = self.name();
        let signal = signal_from_path(&name, path, self.default_polarisation)?;
        let src = path
            .source_geodetic
            .ok_or_else(|| PropagationLossError::MissingPathData {
                model: name.clone(),
                field: "source_geodetic",
            })?;
        let dst = path
            .target_geodetic
            .ok_or_else(|| PropagationLossError::MissingPathData {
                model: name.clone(),
                field: "target_geodetic",
            })?;
        let (distance_m, g0, g1) = match path.terrain_profile.as_deref() {
            Some(profile) if profile.len() >= 2 => {
                if let (Some(f), Some(l)) = (profile.first(), profile.last()) {
                    (l.distance_m - f.distance_m, f.height_m, l.height_m)
                } else {
                    (great_circle_surface_distance_m(&src, &dst), 0.0, 0.0)
                }
            }
            _ => (great_circle_surface_distance_m(&src, &dst), 0.0, 0.0),
        };
        let geom = TerminalPath {
            distance_m,
            tx_height_m: (src.height_m - g0).max(0.0),
            rx_height_m: (dst.height_m - g1).max(0.0),
        };
        self.inner
            .path_loss(&geom, &signal)
            .map_err(|source| PropagationLossError::Model {
                model: name,
                source,
            })
    }
}

impl PropagationLoss for SmoothEarthPropagationLoss {
    fn name(&self) -> String {
        "itu-p526-smooth-earth".to_owned()
    }

    fn loss_db(&self, path: &PathGeometry) -> Result<f64, PropagationLossError> {
        self.evaluate(path).map(|out| out.loss_db())
    }
}

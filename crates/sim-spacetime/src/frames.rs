//! Unified spatiotemporal coordinate frame specifications, transformation directions,
//! unit semantics, dynamical model identities, and separated state samples (`FramedState`,
//! `AttitudeSample`, `VehicleInternalSample`).

use glam::DVec3;
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};

mod types;

pub use types::{
    AttitudeSample, CrossDomainAxesSpec, DynamicalModelIdentity, FrameAxesSpec, FramedState,
    SpatiotemporalOrigin, StateFrameSpec, TransformDirection, UnitSemantics, VehicleInternalSample,
    geodetic_serde,
};

use crate::{
    catalogs::{
        BodyOrientationCatalog, DerivativeOrder, EphemerisCatalog, EphemerisError,
        OrientationError, SynodicError, SynodicFrame,
    },
    ids::NaifId,
    time::TdbEpoch,
};

/// Errors returned by spacetime coordinate frame transformations.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum SpacetimeTransformError {
    /// Attempted to use a normalized `CR3BP` state where an SI metric state is required (or vice versa).
    #[error("incompatible unit semantics: expected {expected:?}, got {actual:?}")]
    IncompatibleUnitSemantics {
        /// Expected unit convention.
        expected: UnitSemantics,
        /// Actual unit convention on the state.
        actual: UnitSemantics,
    },
    /// Unsupported axis variant for pure planetary/orbital transformation without a host vehicle attitude.
    #[error("axes {axes:?} require a host vehicle attitude sample for transformation")]
    RequiresHostAttitude {
        /// Requested axes.
        axes: FrameAxesSpec,
    },
    /// Underlying ephemeris lookup error.
    #[error("ephemeris error: {0}")]
    Ephemeris(#[from] EphemerisError),
    /// Underlying body-orientation error.
    #[error("orientation error: {0}")]
    Orientation(#[from] OrientationError),
    /// Underlying synodic frame error.
    #[error("synodic frame error: {0}")]
    Synodic(#[from] SynodicError),
}

/// Extension trait adding ephemeris- and orientation-backed coordinate transformations to [`FramedState`].
pub trait FramedStateTransformExt: Sized {
    /// Verifies that this state uses [`UnitSemantics::SiMetric`], returning
    /// [`SpacetimeTransformError::IncompatibleUnitSemantics`] if it is `Cr3bpNormalized`.
    fn require_si_metric(&self) -> Result<(), SpacetimeTransformError>;

    /// Explicitly maps a normalized `CR3BP` synodic state into an ephemeris-backed `IcrfInertial`
    /// `SiMetric` state around `target_origin` at `self.spec.epoch`.
    fn map_cr3bp_to_ephemeris_icrf(
        &self,
        target_origin: NaifId,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<Self, SpacetimeTransformError>;

    /// Transforms only the position vector to `target_axes` around the same `origin`.
    fn transform_position_only(
        &self,
        target_axes: FrameAxesSpec,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<DVec3, SpacetimeTransformError>;

    /// Transforms both position and velocity into `(target_origin, target_axes)` at `self.spec.epoch`,
    /// applying full origin translation and rotating-frame $\boldsymbol{\omega} \times \mathbf{r}$ velocity transport.
    fn transform_to(
        &self,
        target_origin: NaifId,
        target_axes: FrameAxesSpec,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<Self, SpacetimeTransformError>;
}

impl FramedStateTransformExt for FramedState {
    fn require_si_metric(&self) -> Result<(), SpacetimeTransformError> {
        if self.spec.units != UnitSemantics::SiMetric {
            return Err(SpacetimeTransformError::IncompatibleUnitSemantics {
                expected: UnitSemantics::SiMetric,
                actual: self.spec.units,
            });
        }
        Ok(())
    }

    fn map_cr3bp_to_ephemeris_icrf(
        &self,
        target_origin: NaifId,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<Self, SpacetimeTransformError> {
        if matches!(self.spec.units, UnitSemantics::SiMetric) {
            return Err(SpacetimeTransformError::IncompatibleUnitSemantics {
                expected: UnitSemantics::Cr3bpNormalized {
                    l_star_m: 1.0,
                    t_star_s: 1.0,
                    mu_star: 0.0,
                },
                actual: self.spec.units,
            });
        }
        self.to_si_metric().transform_to(
            target_origin,
            FrameAxesSpec::IcrfInertial,
            ephemeris,
            orientation,
        )
    }

    fn transform_position_only(
        &self,
        target_axes: FrameAxesSpec,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<DVec3, SpacetimeTransformError> {
        self.require_si_metric()?;
        let (pos_icrf, _) = axes_to_icrf(self, ephemeris, orientation)?;
        let (pos_out, _) = icrf_to_axes(
            self.spec.epoch,
            pos_icrf,
            DVec3::ZERO,
            target_axes,
            ephemeris,
            orientation,
        )?;
        Ok(pos_out)
    }

    fn transform_to(
        &self,
        target_origin: NaifId,
        target_axes: FrameAxesSpec,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<Self, SpacetimeTransformError> {
        self.require_si_metric()?;
        let (pos_icrf_orig, vel_icrf_orig) = axes_to_icrf(self, ephemeris, orientation)?;
        let (pos_icrf_target, vel_icrf_target) = if self.spec.origin == target_origin {
            (pos_icrf_orig, vel_icrf_orig)
        } else {
            let shift = ephemeris.geometric_state_icrf(
                self.spec.origin,
                target_origin,
                self.spec.epoch,
                DerivativeOrder::Velocity,
            )?;
            (pos_icrf_orig + shift.pos_m, vel_icrf_orig + shift.vel_m_s)
        };
        let (pos_out, vel_out) = icrf_to_axes(
            self.spec.epoch,
            pos_icrf_target,
            vel_icrf_target,
            target_axes,
            ephemeris,
            orientation,
        )?;
        Ok(Self {
            spec: StateFrameSpec {
                epoch: self.spec.epoch,
                origin: target_origin,
                axes: target_axes,
                units: UnitSemantics::SiMetric,
            },
            pos: pos_out,
            vel: vel_out,
        })
    }
}

fn axes_to_icrf(
    state: &FramedState,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<(DVec3, DVec3), SpacetimeTransformError> {
    match state.spec.axes {
        FrameAxesSpec::IcrfInertial => Ok((state.pos, state.vel)),
        FrameAxesSpec::BodyFixed { body } => {
            let rot = orientation.body_rotation(body, state.spec.epoch)?;
            Ok(rot.fixed_to_icrf_pv(state.pos, state.vel))
        }
        FrameAxesSpec::RotatingSynodic { primary, secondary } => {
            let syn =
                SynodicFrame::from_ephemeris(primary, secondary, state.spec.epoch, ephemeris)?;
            Ok(syn.synodic_to_icrf_rel_secondary(state.pos, state.vel))
        }
        FrameAxesSpec::TopocentricEnu { body, geodetic } => {
            let enu = LocalFrame::at(&geodetic);
            let pos_fixed = enu.enu_to_ecef() * state.pos;
            let vel_fixed = enu.enu_to_ecef() * state.vel;
            let rot = orientation.body_rotation(body, state.spec.epoch)?;
            Ok(rot.fixed_to_icrf_pv(pos_fixed, vel_fixed))
        }
        FrameAxesSpec::MountedSensor { .. } => Err(SpacetimeTransformError::RequiresHostAttitude {
            axes: state.spec.axes,
        }),
    }
}

fn icrf_to_axes(
    epoch: TdbEpoch,
    pos_icrf: DVec3,
    vel_icrf: DVec3,
    axes: FrameAxesSpec,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<(DVec3, DVec3), SpacetimeTransformError> {
    match axes {
        FrameAxesSpec::IcrfInertial => Ok((pos_icrf, vel_icrf)),
        FrameAxesSpec::BodyFixed { body } => {
            let rot = orientation.body_rotation(body, epoch)?;
            Ok(rot.icrf_to_fixed_pv(pos_icrf, vel_icrf))
        }
        FrameAxesSpec::RotatingSynodic { primary, secondary } => {
            let syn = SynodicFrame::from_ephemeris(primary, secondary, epoch, ephemeris)?;
            Ok(syn.icrf_rel_secondary_to_synodic(pos_icrf, vel_icrf))
        }
        FrameAxesSpec::TopocentricEnu { body, geodetic } => {
            let rot = orientation.body_rotation(body, epoch)?;
            let (pos_fixed, vel_fixed) = rot.icrf_to_fixed_pv(pos_icrf, vel_icrf);
            let enu = LocalFrame::at(&geodetic);
            Ok((
                enu.ecef_to_enu_vector(pos_fixed),
                enu.ecef_to_enu_vector(vel_fixed),
            ))
        }
        FrameAxesSpec::MountedSensor { .. } => {
            Err(SpacetimeTransformError::RequiresHostAttitude { axes })
        }
    }
}

/// Evaluates the `ICRF` position and velocity of a fixed geodetic station on `body` at `epoch`.
pub fn surface_station_state_icrf(
    body: NaifId,
    ellipsoid: BodyEllipsoid,
    geodetic: &Geodetic,
    epoch: TdbEpoch,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<(DVec3, DVec3, LocalFrame), OrientationError> {
    let sta_fixed = ellipsoid.geodetic_to_fixed(geodetic);
    let local_enu = LocalFrame::at(geodetic);
    let rot = orientation.body_rotation(body, epoch)?;
    let (pos_icrf, vel_icrf) = rot.fixed_to_icrf_pv(sta_fixed, DVec3::ZERO);
    Ok((pos_icrf, vel_icrf, local_enu))
}

//! Core spatiotemporal coordinate frame definitions (`FrameAxesSpec`, `StateFrameSpec`,
//! `FramedState`, `AttitudeSample`, `VehicleInternalSample`) and unit/model semantics.

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use sim_geo::Geodetic;

use crate::{
    hash::{ContentHash, DomainHasher},
    ids::{NaifId, VesselId},
    mount::SensorMount,
    provenance::ValidEpochInterval,
    scalar_math::{serde_dquat, serde_dvec3},
    time::TdbEpoch,
};

pub use crate::scalar_math::geodetic_serde;

/// Explicit direction of a coordinate frame or attitude rotation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TransformDirection {
    /// Active / passive transformation mapping coordinates from the local/mount/body frame into the reference/world frame.
    LocalToReference,
    /// Inverse transformation mapping coordinates from the reference/world frame into the local/mount/body frame.
    ReferenceToLocal,
}

/// Unit semantics carried by a [`FramedState`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum UnitSemantics {
    /// Standard SI metric units (`m` for position, `m/s` for velocity, `s` for time).
    SiMetric,
    /// Dimensionless `CR3BP` normalized units parameterized by characteristic length $L^*$,
    /// characteristic time $T^*$, and mass ratio $\mu^* = \mu_2 / (\mu_1 + \mu_2)$.
    Cr3bpNormalized {
        /// Characteristic primary–secondary distance $L^*$ (`m`).
        l_star_m: f64,
        /// Characteristic inverse mean-motion time scale $T^*$ (`s`).
        t_star_s: f64,
        /// Dimensionless mass parameter $\mu^* \in (0, 0.5]$.
        mu_star: f64,
    },
}

/// Explicit dynamical model identity for propagation and analysis.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum DynamicalModelIdentity {
    /// Full N-body solar-system ephemeris force model identified by its `ForceModel::content_hash()`.
    SolarSystemEphemerisNBody {
        /// Canonical SHA-256 hash of the active force model.
        force_model_hash: ContentHash,
    },
    /// Atmospheric / terrestrial kinematic or 6-DoF flight dynamics model on a planetary body.
    PlanetaryKinematicOrFlight {
        /// Host planetary body (`NaifId::EARTH`, `MARS`, etc.).
        body: NaifId,
    },
    /// Idealized autonomous Circular Restricted Three-Body Problem (`CR3BP`).
    AutonomousCr3bp {
        /// Primary body ID.
        primary: NaifId,
        /// Secondary body ID.
        secondary: NaifId,
        /// Dimensionless mass ratio $\mu^* = \mu_2 / (\mu_1 + \mu_2)$.
        mu_star: f64,
    },
}

impl DynamicalModelIdentity {
    /// Writes this model identity into a [`DomainHasher`].
    pub fn write_to_hasher(&self, h: &mut DomainHasher) {
        match *self {
            Self::SolarSystemEphemerisNBody { force_model_hash } => {
                h.write_u8(1);
                h.write_hash(force_model_hash);
            }
            Self::PlanetaryKinematicOrFlight { body } => {
                h.write_u8(2);
                h.write_i32(body.0);
            }
            Self::AutonomousCr3bp {
                primary,
                secondary,
                mu_star,
            } => {
                h.write_u8(3);
                h.write_i32(primary.0);
                h.write_i32(secondary.0);
                h.write_f64_canonical(mu_star);
            }
        }
    }

    /// Returns `true` if this model represents physical SI-metric trajectories in the real solar/planetary system.
    #[must_use]
    pub const fn supports_physical_ephemeris_analysis(&self) -> bool {
        matches!(
            self,
            Self::SolarSystemEphemerisNBody { .. } | Self::PlanetaryKinematicOrFlight { .. }
        )
    }
}

/// Unified origin entity across orbital, planetary-surface, and vehicle/sensor domains.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum SpatiotemporalOrigin {
    /// Natural solar-system body (`SSB`, `SUN`, `EARTH`, `MOON`, `MARS`, etc.).
    CelestialBody(NaifId),
    /// Fixed geodetic ground station anchored on a rotating planetary body.
    SurfaceStation {
        /// Unique ground station identifier.
        station_id: u64,
        /// Central planetary body hosting the station.
        body: NaifId,
        /// Geodetic latitude, longitude (`rad`), and ellipsoidal height (`m`).
        #[serde(with = "geodetic_serde")]
        geodetic: Geodetic,
    },
    /// Simulated spacecraft or vehicle center of mass (with optional rigid sensor mount offset).
    Vessel {
        /// Host vessel identifier.
        vessel_id: VesselId,
        /// Rigid sensor mount relative to the vessel CG.
        mount: SensorMount,
    },
}

/// Coordinate axis orientation convention for a [`FramedState`] or cross-domain query.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum FrameAxesSpec {
    /// Inertial `ICRF / J2000` axes.
    IcrfInertial,
    /// Body-fixed planetary axes (`IAU_BODY` / `ITRF` / `MOON_ME`) for `body`.
    BodyFixed {
        /// Rotating body whose prime-meridian and pole define the axes.
        body: NaifId,
    },
    /// Instantaneous two-body rotating synodic axes (`+X` primary-to-secondary, `+Z` orbital angular momentum).
    RotatingSynodic {
        /// Primary attractor $P_1$.
        primary: NaifId,
        /// Secondary attractor $P_2$.
        secondary: NaifId,
    },
    /// Local topocentric East–North–Up (`+X` East, `+Y` North, `+Z` Up) axes at a surface station.
    TopocentricEnu {
        /// Planetary body hosting the topocentric origin.
        body: NaifId,
        /// Station geodetic coordinates defining the local tangent plane.
        #[serde(with = "geodetic_serde")]
        geodetic: Geodetic,
    },
    /// Vehicle-mounted instrument/camera/antenna axes defined by a [`SensorMount`].
    MountedSensor {
        /// Host vessel identifier.
        vessel_id: VesselId,
        /// Rigid sensor mount (`offset_body_m` and `body_to_mount`).
        mount: SensorMount,
        /// Direction of the quaternion transform (`LocalToReference` = mount-to-body).
        direction: TransformDirection,
    },
}

/// Unified alias for cross-domain axis specifications.
pub type CrossDomainAxesSpec = FrameAxesSpec;

/// Complete epoch, validity interval, origin, axes, and unit descriptor for a 6-DoF translational state.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct StateFrameSpec {
    /// Exact `TDB` epoch at which the state and time-varying axes are evaluated.
    pub epoch: TdbEpoch,
    /// Coordinate origin body (`NaifId::SSB`, `EARTH`, `MOON`, `MARS`, etc.).
    pub origin: NaifId,
    /// Orientation of the coordinate axes.
    pub axes: FrameAxesSpec,
    /// Physical or normalized unit semantics.
    pub units: UnitSemantics,
}

impl StateFrameSpec {
    /// Returns the instantaneous single-epoch [`ValidEpochInterval`] `[epoch, epoch]` of this state.
    #[must_use]
    pub fn valid_interval(&self) -> ValidEpochInterval {
        ValidEpochInterval::new(self.epoch, self.epoch)
    }
}

/// Translational state vector paired with explicit [`StateFrameSpec`] semantics.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FramedState {
    /// Frame, epoch, origin, and unit specification.
    pub spec: StateFrameSpec,
    /// Position vector in `spec` (`m` if `SiMetric`, dimensionless if `Cr3bpNormalized`).
    #[serde(with = "serde_dvec3")]
    pub pos: DVec3,
    /// Velocity vector in `spec` (`m/s` if `SiMetric`, dimensionless if `Cr3bpNormalized`).
    #[serde(with = "serde_dvec3")]
    pub vel: DVec3,
}

impl FramedState {
    /// Explicitly dimensionalizes a `Cr3bpNormalized` state into `SiMetric` (`m` and `m/s`)
    /// using its embedded $(L^*, T^*)$ characteristic scales while keeping the same coordinate axes.
    #[must_use]
    pub fn to_si_metric(&self) -> Self {
        match self.spec.units {
            UnitSemantics::SiMetric => *self,
            UnitSemantics::Cr3bpNormalized {
                l_star_m, t_star_s, ..
            } => {
                let v_scale = l_star_m / t_star_s.max(1.0e-15);
                Self {
                    spec: StateFrameSpec {
                        units: UnitSemantics::SiMetric,
                        ..self.spec
                    },
                    pos: self.pos * l_star_m,
                    vel: self.vel * v_scale,
                }
            }
        }
    }
}

/// Propagator-agnostic attitude and angular velocity sample of a vehicle at `epoch`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AttitudeSample {
    /// Evaluation `TDB` epoch.
    pub epoch: TdbEpoch,
    /// Unit quaternion rotating vehicle body coordinates (`+X` Fwd, `+Y` Right, `+Z` Down) into `ICRF`.
    #[serde(with = "serde_dquat")]
    pub q_body_to_icrf: DQuat,
    /// Angular velocity of the vehicle body relative to `ICRF`, expressed in body coordinates (`rad/s`).
    #[serde(with = "serde_dvec3")]
    pub omega_body_rad_s: DVec3,
}

/// Propagator-agnostic internal vehicle state sample (propellant mass, dry mass, wheel momentum) at `epoch`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct VehicleInternalSample {
    /// Evaluation `TDB` epoch.
    pub epoch: TdbEpoch,
    /// Remaining onboard propellant mass (`kg`).
    pub propellant_mass_kg: f64,
    /// Dry structural mass (`kg`).
    pub dry_mass_kg: f64,
    /// Reaction-wheel internal angular momentum in body coordinates (`N*m*s`).
    #[serde(with = "serde_dvec3")]
    pub wheel_momentum_body_nms: DVec3,
}

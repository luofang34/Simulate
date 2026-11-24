use glam::{DMat3, DVec3};
use sim_devices::GroundTrackerStationSpec;
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_spacetime::{
    ASTRONOMICAL_UNIT_M, BodyOrientationCatalog, BodyRotationState, ContentHash, CoverageSet,
    DataProvenance, DerivativeOrder, DynamicalModelIdentity, EphemerisCatalog, EphemerisError,
    FrameAxesSpec, FrameCode, FramedState, KinematicStateIcrf, NaifId, OrientationError,
    StateFrameSpec, SurfaceNavGrade, TdbEpoch, TdbSpanNs, TrajectoryQueryError,
    TranslationalTrajectorySource, UnitSemantics, ValidEpochInterval,
};

pub const MU_EARTH: f64 = BodyEllipsoid::WGS84.mu_m3_s2;
pub const MU_SUN: f64 = 1.327_124_400_18e20;
const EARTH_SPIN: f64 = BodyEllipsoid::WGS84.omega_rad_s;
const MOON_ORBIT_R: f64 = 384_400_000.0;

pub struct AnalyticSolarCatalog {
    pub ref_epoch: TdbEpoch,
    pub earth_orbits_sun: bool,
    pub include_moon: bool,
}

impl AnalyticSolarCatalog {
    fn earth_ssb_state(&self, epoch: TdbEpoch) -> KinematicStateIcrf {
        if !self.earth_orbits_sun {
            return KinematicStateIcrf::ZERO;
        }
        let dt = epoch.signed_offset_seconds_from(self.ref_epoch);
        let r = ASTRONOMICAL_UNIT_M;
        let omega = (MU_SUN / (r * r * r)).sqrt();
        let (s, c) = (omega * dt).sin_cos();
        let v = r * omega;
        let a = r * omega * omega;
        KinematicStateIcrf {
            pos_m: DVec3::new(r * c, r * s, 0.0),
            vel_m_s: DVec3::new(-v * s, v * c, 0.0),
            acc_m_s2: DVec3::new(-a * c, -a * s, 0.0),
        }
    }

    fn moon_rel_earth(&self, epoch: TdbEpoch) -> KinematicStateIcrf {
        let dt = epoch.signed_offset_seconds_from(self.ref_epoch);
        let r = MOON_ORBIT_R;
        let omega = (MU_EARTH / (r * r * r)).sqrt();
        let (s, c) = (omega * dt).sin_cos();
        let v = r * omega;
        let a = r * omega * omega;
        KinematicStateIcrf {
            pos_m: DVec3::new(r * c, r * s, 0.0),
            vel_m_s: DVec3::new(-v * s, v * c, 0.0),
            acc_m_s2: DVec3::new(-a * c, -a * s, 0.0),
        }
    }
}

impl EphemerisCatalog for AnalyticSolarCatalog {
    fn content_hash(&self) -> ContentHash {
        ContentHash::of_bytes(b"analytic-solar-catalog-v1")
    }

    fn coverage(&self, _: NaifId, _: NaifId, _: DerivativeOrder) -> Vec<TdbSpanNs> {
        CoverageSet::unbounded().spans().to_vec()
    }

    fn geometric_state_icrf(
        &self,
        target: NaifId,
        center: NaifId,
        epoch: TdbEpoch,
        _: DerivativeOrder,
    ) -> Result<KinematicStateIcrf, EphemerisError> {
        let ssb_state = |body| match body {
            NaifId::SSB => Ok(KinematicStateIcrf::ZERO),
            NaifId::EARTH => Ok(self.earth_ssb_state(epoch)),
            NaifId::MOON if self.include_moon => {
                Ok(self.earth_ssb_state(epoch).plus(self.moon_rel_earth(epoch)))
            }
            _ => Err(EphemerisError::UnconnectedBodies {
                target: body,
                center: NaifId::SSB,
            }),
        };
        Ok(ssb_state(target)?.plus(ssb_state(center)?.negated()))
    }

    fn segment_knots_in(&self, _: NaifId, _: NaifId, _: TdbEpoch, _: TdbEpoch) -> Vec<TdbEpoch> {
        Vec::new()
    }
}

impl BodyOrientationCatalog for AnalyticSolarCatalog {
    fn content_hash(&self) -> ContentHash {
        ContentHash::of_bytes(b"analytic-orientation-v1")
    }

    fn coverage(&self, body: NaifId, _: SurfaceNavGrade) -> Vec<TdbSpanNs> {
        if body == NaifId::EARTH || (body == NaifId::MOON && self.include_moon) {
            CoverageSet::unbounded().spans().to_vec()
        } else {
            Vec::new()
        }
    }

    fn body_rotation(
        &self,
        body: NaifId,
        epoch: TdbEpoch,
    ) -> Result<BodyRotationState, OrientationError> {
        if body == NaifId::EARTH {
            let angle = EARTH_SPIN * epoch.signed_offset_seconds_from(self.ref_epoch);
            return Ok(BodyRotationState {
                r_icrf_to_fixed: DMat3::from_rotation_z(-angle),
                omega_body_rad_s: DVec3::new(0.0, 0.0, EARTH_SPIN),
                alpha_body_rad_s2: DVec3::ZERO,
                frame_code: FrameCode::ITRF93,
                grade: SurfaceNavGrade::PrecisionGeodetic,
            });
        }
        if body == NaifId::MOON && self.include_moon {
            return Ok(BodyRotationState {
                r_icrf_to_fixed: DMat3::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
                alpha_body_rad_s2: DVec3::ZERO,
                frame_code: FrameCode::MOON_ME,
                grade: SurfaceNavGrade::PrecisionGeodetic,
            });
        }
        Err(OrientationError::MissingBodyModel(body))
    }
}

pub struct CircularOrbitTrajectory {
    id: u64,
    origin: NaifId,
    ref_epoch: TdbEpoch,
    radius_m: f64,
    inclination_rad: f64,
    phase0_rad: f64,
    mu_m3_s2: f64,
    pub units: UnitSemantics,
    provenance: DataProvenance,
}

impl CircularOrbitTrajectory {
    pub fn new(
        origin: NaifId,
        ref_epoch: TdbEpoch,
        radius_m: f64,
        inclination_rad: f64,
        phase0_rad: f64,
        mu_m3_s2: f64,
        span_s: f64,
    ) -> Self {
        let valid = ValidEpochInterval::new(
            ref_epoch.advanced_seconds(-span_s),
            ref_epoch.advanced_seconds(span_s),
        );
        let provenance = DataProvenance::new(
            "analytic-circular",
            "1",
            valid,
            ContentHash::of_bytes(b"analytic-circular"),
        );
        Self {
            id: 7,
            origin,
            ref_epoch,
            radius_m,
            inclination_rad,
            phase0_rad,
            mu_m3_s2,
            units: UnitSemantics::SiMetric,
            provenance,
        }
    }
}

impl TranslationalTrajectorySource for CircularOrbitTrajectory {
    fn trajectory_id(&self) -> u64 {
        self.id
    }

    fn model_identity(&self) -> DynamicalModelIdentity {
        DynamicalModelIdentity::PlanetaryKinematicOrFlight { body: self.origin }
    }

    fn provenance(&self) -> &DataProvenance {
        &self.provenance
    }

    fn sample_translational_icrf(
        &self,
        epoch: TdbEpoch,
    ) -> Result<FramedState, TrajectoryQueryError> {
        let valid = self.provenance.valid_interval;
        if !valid.contains(epoch) {
            return Err(TrajectoryQueryError::EpochOutOfCoverage {
                requested_epoch: epoch,
                valid_start: valid.start_epoch,
                valid_end: valid.end_epoch,
            });
        }
        let dt = epoch.signed_offset_seconds_from(self.ref_epoch);
        let n = (self.mu_m3_s2 / (self.radius_m * self.radius_m * self.radius_m)).sqrt();
        let theta = self.phase0_rad + n * dt;
        let (s, c) = theta.sin_cos();
        let (si, ci) = self.inclination_rad.sin_cos();
        let v = self.radius_m * n;
        Ok(FramedState {
            spec: StateFrameSpec {
                epoch,
                origin: self.origin,
                axes: FrameAxesSpec::IcrfInertial,
                units: self.units,
            },
            pos: DVec3::new(
                self.radius_m * c,
                self.radius_m * s * ci,
                self.radius_m * s * si,
            ),
            vel: DVec3::new(-v * s, v * c * ci, v * c * si),
        })
    }
}

pub struct FixedTarget {
    origin: NaifId,
    pos_icrf: DVec3,
    provenance: DataProvenance,
}

impl FixedTarget {
    pub fn at(origin: NaifId, ref_epoch: TdbEpoch, pos_icrf: DVec3) -> Self {
        let valid = ValidEpochInterval::new(
            ref_epoch.advanced_seconds(-3600.0),
            ref_epoch.advanced_seconds(3600.0),
        );
        Self {
            origin,
            pos_icrf,
            provenance: DataProvenance::new(
                "fixed",
                "1",
                valid,
                ContentHash::of_bytes(b"fixed-target"),
            ),
        }
    }
}

impl TranslationalTrajectorySource for FixedTarget {
    fn trajectory_id(&self) -> u64 {
        11
    }

    fn model_identity(&self) -> DynamicalModelIdentity {
        DynamicalModelIdentity::PlanetaryKinematicOrFlight { body: self.origin }
    }

    fn provenance(&self) -> &DataProvenance {
        &self.provenance
    }

    fn sample_translational_icrf(
        &self,
        epoch: TdbEpoch,
    ) -> Result<FramedState, TrajectoryQueryError> {
        Ok(FramedState {
            spec: StateFrameSpec {
                epoch,
                origin: self.origin,
                axes: FrameAxesSpec::IcrfInertial,
                units: UnitSemantics::SiMetric,
            },
            pos: self.pos_icrf,
            vel: DVec3::ZERO,
        })
    }
}

pub fn goldstone_station(min_el_rad: f64) -> GroundTrackerStationSpec {
    GroundTrackerStationSpec::new(
        14,
        NaifId::EARTH,
        BodyEllipsoid::WGS84,
        Geodetic::from_degrees(35.426, -116.89, 1000.0),
        min_el_rad,
        8.4e9,
    )
}

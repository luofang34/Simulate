use glam::{DMat3, DQuat, DVec3};
use pilotage_adapter_api::{
    AvionicsAttitudeSample, AvionicsBaroSample, AvionicsKinematicsSample, AvionicsSample,
    FcStateSample, GeodeticFixSample, MeasurementClock, MeasurementStamp, Pose2d, SimTruthSample,
    SourceIncarnation, SourceIntegrity, SourceRole, TelemetrySample,
};
use pilotage_geo::{
    BaroSettingId, DatumRealizationId, GeodeticPosition, GeoidModelId, HorizontalDatum,
    LocalOriginId, PositionQuality, TerrainRefId, VerticalDatum, VerticalPosition,
};
use pilotage_protocol::VehicleId;
use pilotage_timing::SimTick;
use sim_atmosphere::geometric_to_geopotential_m;
use sim_core::{SimInstant, VehicleState};
use sim_devices::WhiteNoise;
use sim_geo::{LocalFrame, ecef_to_geodetic};

const ATTITUDE_SIGMA_RAD: f64 = 0.002;
const RATE_SIGMA_RAD_S: f64 = 0.001;
const POSITION_SIGMA_M: f64 = 1.5;
const VERTICAL_SIGMA_M: f64 = 3.0;
const VELOCITY_SIGMA_M_S: f64 = 0.1;
const BARO_SIGMA_M: f64 = 0.5;
const METRES_TO_MM: f64 = 1_000.0;
/// Attitude, rates, position, and velocity, as Pilotage numbers them.
const ALL_ESTIMATE_GROUPS: u32 = 0b1111;
/// Attitude, position, and velocity: a truth sample carries no rates.
const TRUTH_GROUPS: u32 = 0b1101;
const QUALITY_GOOD: u32 = 0;
/// Arm-state codes of [`FcStateSample::arm_state`].
const DISARMED: u32 = 1;
const ARMED: u32 = 2;
const ESTIMATOR_SOURCE: u64 = 1;
const TRUTH_SOURCE: u64 = 2;
const FC_STATE_SOURCE: u64 = 3;

/// Who produced this adapter's measurements.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct SourceIdentity {
    incarnation: SourceIncarnation,
}

impl SourceIdentity {
    pub(super) fn new(vehicle: VehicleId, seed: u64) -> Self {
        let mut bytes = [0; 16];
        let (head, tail) = bytes.split_at_mut(8);
        head.copy_from_slice(&vehicle.as_u64().to_le_bytes());
        tail.copy_from_slice(&seed.to_le_bytes());
        Self {
            incarnation: SourceIncarnation::new(bytes),
        }
    }
}

/// Stands in for a flight controller's estimator: truth plus seeded Gaussian
/// error of a stated size. Pilotage bars truth from the avionics group, so
/// even a simulator's estimate must come from somewhere else. It is a noise
/// model, not an estimator: its error does not drift, and sensor failures do
/// not reach it.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct AvionicsModel {
    noise: WhiteNoise,
    origin_ecef: DVec3,
    origin: LocalFrame,
    identity: SourceIdentity,
    epoch: u32,
    sequence: u32,
}

/// Body-to-NED rotation at `frame` from body-to-ECEF.
fn body_to_ned(frame: &LocalFrame, attitude: DQuat) -> DQuat {
    DQuat::from_mat3(&(frame.ned_to_ecef().transpose() * DMat3::from_quat(attitude))).normalize()
}

fn quat_wxyz(rotation: DQuat) -> [f32; 4] {
    let DQuat { x, y, z, w } = rotation;
    [w, x, y, z].map(|value| value as f32)
}

fn metres_f32(value: DVec3) -> [f32; 3] {
    value.to_array().map(|component| component as f32)
}

fn millimetres(sigma_m: f64) -> u32 {
    (sigma_m * METRES_TO_MM) as u32
}

/// A WGS84 ellipsoidal position, or `None` if a value could not state one:
/// Pilotage reads absence as no fix.
fn geodetic_fix(
    position_ecef: DVec3,
    quality: PositionQuality,
    stamp: MeasurementStamp,
) -> Option<GeodeticFixSample> {
    let position = ecef_to_geodetic(position_ecef);
    let vertical = VerticalPosition::new(
        position.height_m,
        VerticalDatum::Ellipsoid,
        GeoidModelId::UNDECLARED,
        TerrainRefId::UNDECLARED,
        BaroSettingId::UNDECLARED,
        LocalOriginId::UNDECLARED,
    )
    .ok()?;
    Some(GeodeticFixSample {
        position: GeodeticPosition {
            latitude_deg: position.lat_deg(),
            longitude_deg: (position.lon_deg() + 180.0).rem_euclid(360.0) - 180.0,
            horizontal_datum: HorizontalDatum::Wgs84,
            realization: DatumRealizationId::UNDECLARED,
            vertical,
        },
        quality,
        stamp,
    })
}

impl AvionicsModel {
    pub(super) const fn new(
        seed: u64,
        origin_ecef: DVec3,
        origin: LocalFrame,
        identity: SourceIdentity,
    ) -> Self {
        Self {
            noise: WhiteNoise::from_seed(seed),
            origin_ecef,
            origin,
            identity,
            epoch: 0,
            sequence: 0,
        }
    }

    /// A source restart: the noise stream begins again and the epoch moves on,
    /// so nothing from before the reset orders against what follows.
    pub(super) fn restart(&mut self, seed: u64) {
        self.noise = WhiteNoise::from_seed(seed);
        self.epoch = self.epoch.wrapping_add(1);
        self.sequence = 0;
    }

    fn stamp(&self, role: SourceRole, source_id: u64, at: SimInstant) -> MeasurementStamp {
        MeasurementStamp {
            role,
            integrity: SourceIntegrity::Unprotected,
            source_id,
            source_incarnation: self.identity.incarnation,
            source_epoch: self.epoch,
            sequence: self.sequence,
            acquired_at_ns: at.to_nanos().unwrap_or(u64::MAX),
            clock: MeasurementClock::Simulation,
        }
    }

    fn truth_sample(&self, state: &VehicleState, at: SimInstant) -> SimTruthSample {
        let stamp = self.stamp(SourceRole::SimulationTruth, TRUTH_SOURCE, at);
        let ned = self
            .origin
            .ecef_to_ned_vector(state.position_ecef - self.origin_ecef);
        SimTruthSample {
            quat_wxyz: quat_wxyz(body_to_ned(&self.origin, state.attitude)),
            pos_ned_m: metres_f32(ned),
            vel_ned_mps: metres_f32(self.origin.ecef_to_ned_vector(state.velocity_ecef)),
            valid_flags: TRUTH_GROUPS,
            geodetic: geodetic_fix(
                state.position_ecef,
                PositionQuality {
                    horizontal_mm: 0,
                    vertical_mm: 0,
                },
                stamp,
            ),
            stamp,
        }
    }

    fn estimate(&mut self, state: &VehicleState, at: SimInstant) -> (AvionicsSample, Pose2d, f64) {
        let stamp = self.stamp(SourceRole::OperationalEstimate, ESTIMATOR_SOURCE, at);
        let tilt = self.noise.next_vec3(DVec3::splat(ATTITUDE_SIGMA_RAD));
        let attitude = (state.attitude * DQuat::from_scaled_axis(tilt)).normalize();
        let rates = state.angular_rate_body + self.noise.next_vec3(DVec3::splat(RATE_SIGMA_RAD_S));
        let local = self.origin.ned_to_ecef();
        let position_sigma = DVec3::new(POSITION_SIGMA_M, POSITION_SIGMA_M, VERTICAL_SIGMA_M);
        let position = state.position_ecef + local * self.noise.next_vec3(position_sigma);
        let velocity = self.origin.ecef_to_ned_vector(state.velocity_ecef)
            + self.noise.next_vec3(DVec3::splat(VELOCITY_SIGMA_M_S));
        let ned = self.origin.ecef_to_ned_vector(position - self.origin_ecef);
        let height = ecef_to_geodetic(state.position_ecef).height_m;
        let pressure_alt =
            geometric_to_geopotential_m(height) + self.noise.next_gaussian() * BARO_SIGMA_M;
        let forward = self.origin.ecef_to_ned_vector(attitude * DVec3::X);
        let quality = PositionQuality {
            horizontal_mm: millimetres(POSITION_SIGMA_M),
            vertical_mm: millimetres(VERTICAL_SIGMA_M),
        };
        let sample = AvionicsSample {
            attitude: Some(AvionicsAttitudeSample::Canonical {
                quat_wxyz: quat_wxyz(body_to_ned(&self.origin, attitude)),
                rates_rps: Some(metres_f32(rates)),
                stamp,
            }),
            kinematics: Some(AvionicsKinematicsSample {
                pos_ned_m: metres_f32(ned),
                vel_ned_mps: metres_f32(velocity),
                stamp,
            }),
            baro: Some(AvionicsBaroSample {
                pressure_alt_m: pressure_alt as f32,
                stamp,
            }),
            geodetic: geodetic_fix(position, quality, stamp),
            magnetic_variation: None,
            estimator_status_stamp: Some(stamp),
            valid_flags: ALL_ESTIMATE_GROUPS,
            quality: QUALITY_GOOD,
        };
        let pose = Pose2d {
            x: ned.x,
            y: ned.y,
            heading: forward.y.atan2(forward.x),
        };
        (sample, pose, velocity.length())
    }

    /// One new measurement of every group. Sequence numbers advance here and
    /// nowhere else, so re-reading a sample never claims a new measurement.
    pub(super) fn sample(
        &mut self,
        vehicle: VehicleId,
        tick: SimTick,
        at: SimInstant,
        state: &VehicleState,
        armed: bool,
    ) -> TelemetrySample {
        self.sequence = self.sequence.wrapping_add(1);
        let (avionics, pose, speed) = self.estimate(state, at);
        TelemetrySample {
            flight_procedure: None,
            payload: None,
            vehicle,
            tick,
            pose: Some(pose),
            speed: Some(speed),
            avionics: Some(avionics),
            sim_truth: Some(self.truth_sample(state, at)),
            fc_state: Some(FcStateSample {
                arm_state: if armed { ARMED } else { DISARMED },
                last_command: None,
                landed_state: None,
                motor_state: None,
                landing_approach: None,
                stamp: Some(self.stamp(SourceRole::FcState, FC_STATE_SOURCE, at)),
            }),
            gimbal: None,
            battery: None,
        }
    }
}

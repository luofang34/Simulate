use glam::{DQuat, DVec3};
use indicate_instrument_feeder::nav_guidance::NavGuidanceTracker;
use indicate_instrument_state::abi::v9::{DecodeReport, decode_state, encode_state};
use indicate_instrument_state::{
    AirData, AircraftState, AltitudeClass, AltitudeDeclaration, Attitude, DynSample,
    EstimateQuality, Kinematics, OriginId, SnapshotCoherence, Stamped, TurnBasis, TurnSample,
    ValidFlags,
};
use sim_apollo::CockpitInstrumentSnapshot;
use sim_core::{SimInstant, SourceValidity, StampedReading, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, InertialTransform, LocalFrame};

use crate::clock::age_ms;
use crate::frames::{heading_rate_rad_s, indicate_quat, true_heading_rad};
use crate::{
    AirDataReading, DisplayAxes, FeedError, LocalLevel, indicated_altitude_m, instant_ms,
    pressure_altitude_m,
};

mod guidance;
#[cfg(test)]
mod tests;

pub use guidance::{
    AutopilotModeSummary, HeadingMode, HostFlightGuidance, HostNavGuidance, PilotSelections,
    RunwayNavGuidance,
};

/// Largest spread between the stamps of one frame's groups that still reads as
/// one instant, milliseconds. A 25 Hz barometer beside a 120 Hz flight model
/// sits inside it.
const COHERENCE_BUDGET_MS: u64 = 50;

/// Where the displayed altitude comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AltitudeSource {
    /// Height above [`FeedConfig::origin`], which the tape labels `REL` so it
    /// cannot pass for a barometric altitude. The only choice with no
    /// atmosphere to read.
    OriginRelative,
    /// The air-data sample's static pressure, as indicated altitude under its
    /// altimeter setting or pressure altitude without one. Indicate fails the
    /// tape if the sample cannot supply it; nothing falls back.
    Barometric,
}

/// What stays fixed across a flight's frames.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeedConfig {
    /// The central body the vehicle flies over.
    pub body: BodyEllipsoid,
    /// How the vehicle's axes sit against a display's.
    pub axes: DisplayAxes,
    /// The local origin positions are measured from.
    pub origin: Geodetic,
    /// Identity of `origin`, carried with origin-relative altitude.
    pub origin_id: u32,
    /// Where the displayed altitude comes from.
    pub altitude: AltitudeSource,
    /// North reference convention stamped on primary headings.
    pub heading_mode: HeadingMode,
}

/// One instant of flight, as the panels should be told it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightSample {
    /// The instant being displayed.
    pub timestamp: SimInstant,
    /// The vehicle. Without [`Self::cockpit`], its attitude and rates are the
    /// sensed ones; with it, this supplies position and velocity alone, as the
    /// navigation solution the display trusts.
    pub vehicle: VehicleState,
    /// Sensed attitude and rates with their source health.
    pub cockpit: Option<CockpitInstrumentSnapshot>,
    /// How the central body stood, needed for an Apollo snapshot's body to
    /// J2000 attitudes.
    pub central_body: Option<InertialTransform>,
    /// The pitot-static system, on a vehicle that has one.
    pub air_data: Option<AirDataReading>,
    /// Specific force in vehicle axes at the turn coordinator, m/s².
    pub specific_force_body_m_s2: Option<DVec3>,
    /// What the flight-guidance host reports, when a guidance panel is fitted.
    pub guidance: Option<HostFlightGuidance>,
}

impl FlightSample {
    /// A sample reporting `vehicle` exactly, with no other sensor fitted.
    pub fn truth(timestamp: SimInstant, vehicle: VehicleState) -> Self {
        Self {
            timestamp,
            vehicle,
            cockpit: None,
            central_body: None,
            air_data: None,
            specific_force_body_m_s2: None,
            guidance: None,
        }
    }

    fn tick(&self) -> u128 {
        self.timestamp.ticks_u128()
    }

    /// The attitude being flown, read through `Failed` as well as healthy.
    fn flown_attitude(&self) -> StampedReading<DQuat> {
        match &self.cockpit {
            Some(cockpit) => cockpit.attitude,
            None => {
                StampedReading::new(self.vehicle.attitude, self.timestamp, SourceValidity::Valid)
            }
        }
    }

    /// The body rate sensed, vehicle axes.
    fn body_rate(&self) -> StampedReading<DVec3> {
        match &self.cockpit {
            Some(cockpit) => cockpit.body_rate_rad_s,
            None => StampedReading::new(
                self.vehicle.angular_rate_body,
                self.timestamp,
                SourceValidity::Valid,
            ),
        }
    }

    /// Whether a cockpit attitude is body to J2000 rather than body to fixed.
    fn reads_inertial(&self) -> bool {
        self.cockpit
            .is_some_and(|cockpit| cockpit.guidance_source.is_some())
    }

    /// Display-to-NED attitude of one of this sample's attitude readings.
    pub(crate) fn display_attitude(
        &self,
        rotation: DQuat,
        local: &LocalLevel,
        axes: &DisplayAxes,
    ) -> Result<DQuat, FeedError> {
        if !self.reads_inertial() {
            return Ok(local.attitude_from_fixed(rotation, axes));
        }
        let central_body = self
            .central_body
            .as_ref()
            .ok_or(FeedError::MissingBodyFrame { tick: self.tick() })?;
        Ok(local.attitude_from_inertial(rotation, central_body, axes))
    }

    fn require(&self, field: &'static str, finite: bool) -> Result<(), FeedError> {
        if finite {
            return Ok(());
        }
        Err(FeedError::NonFinite {
            field,
            tick: self.tick(),
        })
    }

    fn validate(&self) -> Result<(), FeedError> {
        let vehicle = &self.vehicle;
        self.require("vehicle.position_ecef", vehicle.position_ecef.is_finite())?;
        self.require("vehicle.velocity_ecef", vehicle.velocity_ecef.is_finite())?;
        let attitude = self.flown_attitude();
        if attitude.validity != SourceValidity::Missing {
            self.require("attitude", attitude.value.is_finite())?;
        }
        let rate = self.body_rate();
        if rate.validity != SourceValidity::Missing {
            self.require("body_rate", rate.value.is_finite())?;
        }
        if let Some(force) = self.specific_force_body_m_s2 {
            self.require("specific_force_body_m_s2", force.is_finite())?;
        }
        Ok(())
    }
}

/// The attitude group and what later groups read from it.
struct AttitudeGroup {
    stamped: Stamped<Attitude>,
    display_to_ned: Option<DQuat>,
    rate_display: Option<DVec3>,
    usable: bool,
    degraded: bool,
    rates_usable: bool,
}

/// Turns [`FlightSample`]s into Indicate's input state. Between frames it
/// carries the snapshot generation and the admission state of the host's
/// guidance lane.
#[derive(Debug)]
pub struct StateFeed {
    config: FeedConfig,
    origin_fixed_m: DVec3,
    origin_frame: LocalFrame,
    generation: u32,
    nav_tracker: NavGuidanceTracker,
}

impl StateFeed {
    /// A feed that has produced nothing yet.
    pub fn new(config: FeedConfig) -> Self {
        Self {
            origin_fixed_m: config.body.geodetic_to_fixed(&config.origin),
            origin_frame: LocalFrame::at(&config.origin),
            config,
            generation: 0,
            nav_tracker: NavGuidanceTracker::new(),
        }
    }

    /// Forgets the guidance lane's admission state, so the next sample is
    /// accepted against a fresh clock. Call it when the host's guidance clock
    /// goes back, as across a rewind; the tracker refuses a clock that runs
    /// backwards and shows no guidance until it does.
    pub fn reset_guidance_clock(&mut self) {
        self.nav_tracker = NavGuidanceTracker::new();
    }

    /// The configuration.
    pub fn config(&self) -> &FeedConfig {
        &self.config
    }

    /// The local level the vehicle is at in `sample`.
    pub(crate) fn local_level(&self, sample: &FlightSample) -> LocalLevel {
        LocalLevel::at(&self.config.body, sample.vehicle.position_ecef)
    }

    /// The state to display for `sample`. A reading that stopped updating
    /// shows through its age, and one with nothing to report leaves its group
    /// out, so Indicate decides how each is shown. Non-finite values that
    /// frame conversion depends on are refused.
    pub fn state(&mut self, sample: &FlightSample) -> Result<AircraftState, FeedError> {
        sample.validate()?;
        let local = self.local_level(sample);
        let attitude = self.attitude_group(sample, &local)?;
        let air = air_group(sample);
        let stamps = group_stamps(sample, &attitude, &air);
        let display_to_ned = attitude.display_to_ned;
        let heading = display_to_ned.and_then(true_heading_rad);
        let (heading_sample, variation) = guidance::heading_and_variation(
            self.config.heading_mode,
            &self.config.body,
            local.geodetic(),
            heading,
        );
        let turn = display_to_ned
            .zip(attitude.rate_display)
            .and_then(|(attitude, rate)| heading_rate_rad_s(attitude, rate));
        let lateral = sample
            .specific_force_body_m_s2
            .map(|force| self.config.axes.vector_in_display(force).y as f32);
        self.generation = self.generation.wrapping_add(1);
        let altitude = self.altitude_declaration(sample);
        let air_setting = air.data.and_then(|air| air.baro_setting_hpa);
        let selections = sample
            .guidance
            .map(|host| guidance::indicate_selections(&host.selections, &altitude, air_setting))
            .unwrap_or_default();
        let nav = sample.guidance.map_or_else(Stamped::default, |host| {
            guidance::host_nav(
                &mut self.nav_tracker,
                &host.nav,
                &self.config.body,
                sample.vehicle.position_ecef,
            )
        });
        Ok(AircraftState {
            attitude: attitude.stamped,
            kinematics: fresh(self.kinematics(sample, &local)),
            air,
            quality: if attitude.degraded {
                EstimateQuality::Degraded
            } else {
                EstimateQuality::Good
            },
            valid: ValidFlags {
                attitude: attitude.usable,
                rates: attitude.rates_usable,
                position: true,
                velocity_horizontal: true,
                velocity_vertical: true,
                heading: heading_sample.is_some() && attitude.usable,
                variation: variation.is_some(),
                turn: turn.is_some() && attitude.rates_usable,
                slip: lateral.is_some(),
                ..ValidFlags::default()
            },
            altitude,
            selections,
            nav,
            heading: stamped_if(heading_sample),
            variation: stamped_if(variation),
            dynamics: dynamics_group(turn, lateral),
            snapshot: indicate_instrument_state::SnapshotMeta {
                generation: self.generation,
                coherence: coherence(&stamps),
            },
            ..AircraftState::default()
        })
    }

    fn attitude_group(
        &self,
        sample: &FlightSample,
        local: &LocalLevel,
    ) -> Result<AttitudeGroup, FeedError> {
        let reading = sample.flown_attitude();
        let rate = sample.body_rate();
        let rate_present = rate.validity != SourceValidity::Missing;
        let rate_display = rate_present.then(|| self.config.axes.vector_in_display(rate.value));
        let rates_usable = rate_present && rate.validity != SourceValidity::Failed;
        if reading.validity == SourceValidity::Missing {
            return Ok(AttitudeGroup {
                stamped: Stamped::default(),
                display_to_ned: None,
                rate_display,
                usable: false,
                degraded: false,
                rates_usable,
            });
        }
        let display_to_ned = sample.display_attitude(reading.value, local, &self.config.axes)?;
        let DVec3 { x, y, z } = rate_display.unwrap_or(DVec3::ZERO);
        Ok(AttitudeGroup {
            stamped: Stamped {
                data: Some(Attitude {
                    quat: indicate_quat(display_to_ned),
                    rates_rps: [x as f32, y as f32, z as f32],
                }),
                age_ms: Some(age_ms(sample.timestamp, reading.timestamp)),
            },
            display_to_ned: Some(display_to_ned),
            rate_display,
            usable: reading.validity != SourceValidity::Failed,
            degraded: reading.validity == SourceValidity::Degraded,
            rates_usable,
        })
    }

    /// Position against the origin's tangent plane. Down is the height
    /// difference, so the curve of the body does not read as a climb.
    fn kinematics(&self, sample: &FlightSample, local: &LocalLevel) -> Kinematics {
        let offset = sample.vehicle.position_ecef - self.origin_fixed_m;
        let DVec3 { x, y, .. } = self.origin_frame.ecef_to_ned_vector(offset);
        let height_m = local.geodetic().height_m;
        let down = self.config.origin.height_m - height_m;
        let velocity = local.vector_in_ned(sample.vehicle.velocity_ecef);
        Kinematics {
            pos_ned_m: [x as f32, y as f32, down as f32],
            vel_ned_mps: velocity.to_array().map(|component| component as f32),
        }
    }

    fn altitude_declaration(&self, sample: &FlightSample) -> AltitudeDeclaration {
        let base = AltitudeDeclaration {
            origin: OriginId(self.config.origin_id),
            ..AltitudeDeclaration::default()
        };
        if self.config.altitude == AltitudeSource::OriginRelative {
            return base;
        }
        let reading = usable_air_data(sample);
        let pressure = reading.and_then(|air| air.static_pressure_pa);
        let setting = reading.and_then(|air| air.altimeter_setting_hpa);
        let (reference_class, altitude) = match reading.map(|air| air.altimeter_setting_hpa) {
            Some(None) => (
                AltitudeClass::Pressure,
                pressure.and_then(pressure_altitude_m),
            ),
            _ => (
                AltitudeClass::BaroIndicated,
                pressure
                    .zip(setting)
                    .and_then(|(pressure, setting)| indicated_altitude_m(pressure, setting)),
            ),
        };
        AltitudeDeclaration {
            reference_class,
            sample_m: altitude.map(|metres| metres as f32),
            ..base
        }
    }
}

fn fresh<T>(data: T) -> Stamped<T> {
    Stamped {
        data: Some(data),
        age_ms: Some(0.0),
    }
}

fn stamped_if<T>(data: Option<T>) -> Stamped<T> {
    data.map_or_else(Stamped::default, fresh)
}

fn usable_air_data(sample: &FlightSample) -> Option<crate::AirDataSample> {
    sample
        .air_data
        .filter(|reading| reading.validity.is_usable() || reading.validity == SourceValidity::Stale)
        .map(|reading| reading.value)
}

/// The air-data group. Indicate has no validity flag for this group, so a
/// failed source offers no value and the tapes go to dashes.
fn air_group(sample: &FlightSample) -> Stamped<AirData> {
    let (Some(reading), Some(air)) = (sample.air_data, usable_air_data(sample)) else {
        return Stamped::default();
    };
    Stamped {
        data: Some(AirData {
            ias_mps: Some(air.indicated_airspeed_m_s() as f32),
            baro_setting_hpa: air.altimeter_setting_hpa.map(|hpa| hpa as f32),
            tas_mps: Some(air.true_airspeed_m_s as f32),
            // The head-up display carries no rate-of-climb instrument.
            vsi_mps: None,
        }),
        age_ms: Some(age_ms(sample.timestamp, reading.timestamp)),
    }
}

fn dynamics_group(turn: Option<f64>, lateral: Option<f32>) -> Stamped<DynSample> {
    if turn.is_none() && lateral.is_none() {
        return Stamped::default();
    }
    fresh(DynSample {
        turn: turn.map(|rate| TurnSample {
            rate_rps: rate as f32,
            basis: TurnBasis::HeadingRate,
        }),
        lateral_mps2: lateral,
        ias_trend_mps2: None,
    })
}

/// When each stamped group in the frame was acquired, milliseconds.
fn group_stamps(
    sample: &FlightSample,
    attitude: &AttitudeGroup,
    air: &Stamped<AirData>,
) -> Vec<u64> {
    let now = instant_ms(sample.timestamp);
    let mut stamps = vec![now];
    if attitude.stamped.data.is_some() {
        stamps.push(instant_ms(sample.flown_attitude().timestamp));
    }
    if let (Some(reading), Some(_)) = (sample.air_data, air.data) {
        stamps.push(instant_ms(reading.timestamp));
    }
    stamps
}

fn coherence(stamps: &[u64]) -> SnapshotCoherence {
    let (Some(oldest), Some(newest)) = (stamps.iter().min(), stamps.iter().max()) else {
        return SnapshotCoherence::Insufficient;
    };
    if stamps.len() < 2 {
        SnapshotCoherence::Insufficient
    } else if newest - oldest > COHERENCE_BUDGET_MS {
        SnapshotCoherence::ExcessiveSkew
    } else {
        SnapshotCoherence::Coherent
    }
}

/// Writes `state` as a tagged binary frame into `buffer`, returning its
/// length. [`indicate_instrument_state::abi::v9::CAPACITY`] always suffices.
pub fn encode_frame(state: &AircraftState, buffer: &mut [u8]) -> Result<usize, FeedError> {
    encode_state(state, buffer).map_err(|source| FeedError::Encode {
        capacity: buffer.len(),
        source,
    })
}

/// Reads a tagged binary frame back, as a panel across the ABI boundary does.
pub fn decode_frame(frame: &[u8]) -> Result<DecodeReport, FeedError> {
    decode_state(frame).map_err(|source| FeedError::Decode {
        length: frame.len(),
        source,
    })
}

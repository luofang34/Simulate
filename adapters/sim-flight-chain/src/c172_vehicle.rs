//! The Cessna 172 as a vehicle a mission can command: the JSBSim airframe,
//! its sensor suite and Aviate's fixed-wing kernel, and the airframe
//! Pilotage's adapter knows it as.

use aviate_core::control::fixed_wing::FixedWingController;
use aviate_core::mixer::fixed_wing::FixedWingMixer;
use glam::DVec3;
use pilotage_adapter_aviate::AviateAirframe;
use pilotage_control_feel::{FixedWingEnvelope, ValidatedFixedWingEnvelope};
use pilotage_protocol::VehicleId;
use sim_aviate::{build_direct_fixed_wing_kernel, c172_airframe};
use sim_core::{TICKS_PER_SECOND, TickSpan};
use sim_devices::{
    GaussMarkov, GnssErrorSpec, ImuGrade, MagneticEnvironment, MagnetometerSpec, PressureErrorSpec,
    SensorSuiteSpec,
};
use sim_geo::{Geodetic, UtcTime};

use crate::{C172Airframe, C172Start, C172Variant, ChainError, HostedBoard, SharedAir};

/// Cycle rate of the C172's flight controller, hertz: the one rate Aviate's
/// fixed-wing law has flown at.
pub const C172_RATE_HZ: u64 = 250;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(C172_RATE_HZ));

/// The C172's board on its airframe.
pub type C172Board = HostedBoard<C172Airframe, FixedWingController, FixedWingMixer>;

/// One C172 in the air.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct C172Setup {
    /// The vehicle Pilotage addresses.
    pub vehicle: VehicleId,
    /// Origin of the flight controller's local frame.
    pub origin: Geodetic,
    /// Where the aeroplane starts, in trimmed level flight.
    pub position: Geodetic,
    /// Heading at the start from true north, radians.
    pub heading_rad: f64,
    /// True airspeed at the start, m/s.
    pub speed_m_s: f64,
    /// Instant of the start.
    pub utc: UtcTime,
    /// Date of the geomagnetic field, as a decimal year.
    pub decimal_year: f64,
    /// Seed of the device errors.
    pub seed: u64,
}

/// How far the C172's accelerometers were calibrated before flight.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum AccelCalibration {
    /// Calibrated: [`ImuGrade::ConsumerMemsCalibrated`].
    #[default]
    Calibrated,
    /// Uncalibrated, as shipped: [`ImuGrade::ConsumerMems`], whose 20 mg
    /// 1-sigma initial offset makes a stress configuration.
    Uncalibrated20Mg,
}

impl AccelCalibration {
    /// The inertial unit's grade: the one statement of its errors that the
    /// devices simulate and the navigators declare.
    pub const fn grade(self) -> ImuGrade {
        match self {
            Self::Calibrated => ImuGrade::ConsumerMemsCalibrated,
            Self::Uncalibrated20Mg => ImuGrade::ConsumerMems,
        }
    }
}

/// [`c172_sensor_suite_with`] with calibrated accelerometers.
pub fn c172_sensor_suite(decimal_year: f64, seed: u64) -> SensorSuiteSpec {
    c172_sensor_suite_with(decimal_year, seed, AccelCalibration::Calibrated)
}

/// The C172's devices at the flight controller's cycle: inertial every
/// cycle, air data at 25 Hz, magnetometer at 50 Hz, receiver at 10 Hz.
///
/// The error figures are typical of the device classes, not measured on an
/// aeroplane: a consumer MEMS inertial unit whose accelerometer bias is as
/// `calibration` says, pressure sensors with a few pascals of noise over a
/// slowly wandering bias, a magnetometer with an uncalibrated
/// half-microtesla hard-iron residue, and a single-frequency receiver in
/// open sky. The declared local magnetic anomaly is zero.
pub fn c172_sensor_suite_with(
    decimal_year: f64,
    seed: u64,
    calibration: AccelCalibration,
) -> SensorSuiteSpec {
    let pressure = |white_sigma_pa, sigma| PressureErrorSpec {
        white_sigma_pa,
        bias: GaussMarkov {
            tau_s: 600.0,
            sigma,
        },
    };
    SensorSuiteSpec {
        imu: calibration.grade().spec(),
        static_port: pressure(3.0, 20.0),
        pitot: Some((pressure(3.0, 10.0), DVec3::X)),
        magnetometer: MagnetometerSpec {
            white_sigma_ut: 0.2,
            hard_iron_sigma_ut: 0.5,
            scale_factor_sigma: 0.0,
        },
        gnss: GnssErrorSpec::OPEN_SKY,
        magnetic: MagneticEnvironment::main_field(decimal_year),
        air_data_every: 10,
        magnetometer_every: 5,
        gnss_every: 25,
        seed,
    }
}

/// The C172S of `setup` in `air` with the devices of `suite`, trimmed and
/// flying, its flight controller booting.
pub fn c172_board(
    setup: &C172Setup,
    suite: &SensorSuiteSpec,
    air: SharedAir,
) -> Result<C172Board, ChainError> {
    c172_board_variant(C172Variant::C172s, setup, suite, air)
}

/// The selected C-172 `variant` of `setup` in `air` with the devices of
/// `suite`, trimmed and flying, its flight controller booting.
pub fn c172_board_variant(
    variant: C172Variant,
    setup: &C172Setup,
    suite: &SensorSuiteSpec,
    air: SharedAir,
) -> Result<C172Board, ChainError> {
    let start = C172Start {
        position: setup.position,
        heading_rad: setup.heading_rad,
        speed_m_s: setup.speed_m_s,
        period: TickSpan::from_ticks(TICKS_PER_SECOND / C172_RATE_HZ),
        utc: setup.utc,
    };
    let airframe = C172Airframe::trimmed_variant(variant, &start, air)?;
    let kernel = build_direct_fixed_wing_kernel(&c172_airframe()).ok_or(ChainError::KernelBuild)?;
    HostedBoard::new(airframe, suite, kernel, setup.origin, setup.decimal_year)
}

/// The bank Pilotage's mission plans the C172's turns at, degrees: 3°
/// under the flight controller's 25° route bank, so the roll-in and
/// disturbances keep authority left.
pub const C172_PLANNING_BANK_DEG: f64 = 22.0;

/// The C172 as Pilotage's Aviate adapter knows it: a fixed wing flown by
/// the flight controller's own law, its demands bounded by Pilotage's C172
/// envelope.
pub fn c172_pilotage_airframe() -> Result<AviateAirframe, ChainError> {
    ValidatedFixedWingEnvelope::new(FixedWingEnvelope::C172)
        .map(AviateAirframe::FixedWing)
        .map_err(|source| ChainError::Envelope {
            detail: source.to_string(),
        })
}

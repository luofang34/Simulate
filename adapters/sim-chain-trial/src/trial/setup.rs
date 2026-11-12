//! Building a chain trial from its spec: the aeroplane where the spec puts
//! it, its control chain under Pilotage's host, and every set's navigator
//! aligned on the declared start.

use glam::DVec3;
use sha2::{Digest, Sha256};
use sim_atmosphere::{AtmosphereField, GeoidStandardAtmosphere};
use sim_core::VehicleState;
use sim_devices::SensorSuite;
use sim_flight_chain::{
    C172Setup, ChainRoute, HoldingTurn, OwnshipFeed, RouteFix, SharedAir, TerminalHoldingPattern,
    VehicleClass, VehicleId, c172_board, c172_pilotage_airframe, c172_sensor_suite_with,
};
use sim_geo::{Geodetic, UtcTime};
use sim_navigate::trial::{TrialRequest, parse_utc};
use sim_navigate::{GroundCalibration, InitialAlignment};

use super::C172Chain;
use crate::{ChainTrialError, ChainTrialSpec, HoldSpec, HoldTurn, Steering};

/// Sea-level standard density, kg/m³.
const ISA_RHO0_KG_M3: f64 = 1.225;
/// Seconds of a mean Julian year, for the geomagnetic date.
const SECONDS_PER_YEAR: f64 = 31_557_600.0;
/// When the crew hands the plan to Pilotage's host after the flight
/// controller powers up, seconds.
const PLAN_LOADED_AT_S: f64 = 2.0;
/// Climb rate the mission may ask for toward a higher fix, m/s.
const CLIMB_RATE_M_S: f64 = 2.0;

/// What the spec resolves to before anything flies.
pub(super) struct Resolved {
    pub(super) request: TrialRequest,
    pub(super) utc: UtcTime,
    pub(super) start: Geodetic,
    pub(super) digest: String,
}

/// The spec's request as the vehicle's devices make it: the navigators
/// declare the very grade of inertial unit the C172's devices simulate.
pub(super) fn resolve(spec: &ChainTrialSpec) -> Result<Resolved, ChainTrialError> {
    let mut request = spec.request.clone();
    request.imu_grade = spec.calibration.grade_name();
    let utc = parse_utc(&request.epoch_utc)?;
    let s = spec.start;
    let start = Geodetic::from_degrees(s.lat_deg, s.lon_deg, s.height_m);
    let bytes = serde_json_digest_input(spec)?;
    Ok(Resolved {
        digest: Sha256::digest(&bytes)
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect(),
        request,
        utc,
        start,
    })
}

fn serde_json_digest_input(spec: &ChainTrialSpec) -> Result<Vec<u8>, ChainTrialError> {
    serde_json::to_vec(spec).map_err(|error| ChainTrialError::Invalid {
        detail: format!("the spec does not serialise: {error}"),
    })
}

/// The C172 at the declared start under Pilotage's host core.
pub(super) fn chain(
    spec: &ChainTrialSpec,
    resolved: &Resolved,
) -> Result<C172Chain, ChainTrialError> {
    let true_m_s = true_airspeed_m_s(spec.start.indicated_m_s, &resolved.start, resolved.utc)?;
    let decimal_year = 1970.0 + resolved.utc.unix_seconds() / SECONDS_PER_YEAR;
    let seed = resolved.request.seeds.imu;
    let setup = C172Setup {
        vehicle: VehicleId::new(1),
        origin: resolved.start,
        position: resolved.start,
        heading_rad: spec.start.heading_deg.to_radians(),
        speed_m_s: true_m_s,
        utc: resolved.utc,
        decimal_year,
        seed,
    };
    let suite = c172_sensor_suite_with(decimal_year, seed, spec.calibration.calibration());
    // The trial's air: the standard atmosphere entered at mean sea level,
    // the air the navigation trial's devices sense and its navigators read
    // their altimeters in.
    let air = SharedAir::new(GeoidStandardAtmosphere::new());
    let board = c172_board(&setup, &suite, air)?;
    let route = ChainRoute {
        fixes: resolved
            .request
            .waypoints
            .iter()
            .map(|w| RouteFix {
                ident: Some(w.ident.clone()),
                lat_deg: w.lat_deg,
                lon_deg: w.lon_deg,
                altitude_m: w.altitude_m,
            })
            .collect(),
        cruise_mps: spec.start.indicated_m_s,
        route_bank_deg: sim_flight_chain::C172_PLANNING_BANK_DEG,
        climb_rate_mps: CLIMB_RATE_M_S,
        max_horizontal_mps: spec.start.indicated_m_s,
        ownship: match spec.steering {
            Steering::FlightController => OwnshipFeed::FlightController,
            Steering::Set(_) => OwnshipFeed::Navigator,
        },
        class: VehicleClass::FixedWing,
        airframe: c172_pilotage_airframe()?,
        plan_loaded_at_s: PLAN_LOADED_AT_S,
        utc: resolved.utc,
        terminal_hold: terminal_hold(spec.hold)?,
    };
    Ok(C172Chain::new(board, &route)?)
}

/// The hold the mission flies at its last fix: the spec's, or Pilotage's
/// standard.
fn terminal_hold(hold: Option<HoldSpec>) -> Result<TerminalHoldingPattern, ChainTrialError> {
    let Some(hold) = hold else {
        return Ok(TerminalHoldingPattern::default());
    };
    if !(hold.leg_time_s.is_finite() && hold.leg_time_s > 0.0) {
        return Err(ChainTrialError::Invalid {
            detail: format!("a hold leg time of {} s", hold.leg_time_s),
        });
    }
    Ok(TerminalHoldingPattern {
        altitude_m: hold.altitude_m,
        turn: match hold.turn {
            HoldTurn::Left => HoldingTurn::Left,
            HoldTurn::Right => HoldingTurn::Right,
        },
        leg_time_s: Some(hold.leg_time_s),
    })
}

/// The true airspeed that reads `indicated_m_s` at `start` in the standard
/// atmosphere, the air the aeroplane starts in.
fn true_airspeed_m_s(
    indicated_m_s: f64,
    start: &Geodetic,
    utc: UtcTime,
) -> Result<f64, ChainTrialError> {
    let air = GeoidStandardAtmosphere::new()
        .sample(start, utc)
        .map_err(|error| ChainTrialError::Invalid {
            detail: format!("no standard air at the start: {error}"),
        })?;
    Ok(indicated_m_s * (ISA_RHO0_KG_M3 / air.density_kg_m3).sqrt())
}

/// The sets' alignment: the declared start with the request's stated
/// uncertainty, its error drawn from the alignment seed, and the biases of
/// a ground calibration of `devices`' inertial unit.
///
/// The aeroplane starts in the air, but its crew ran the engine up at rest
/// first: the gyroscopes' mean rate over that minute, less the Earth's spin
/// at the surveyed start and the magnetic heading, is their turn-on bias,
/// which no set could otherwise learn before it turned the heading away.
/// One calibration of the one unit serves every set.
pub(super) fn alignment(
    request: &TrialRequest,
    start: &VehicleState,
    devices: &SensorSuite,
) -> sim_navigate::AlignmentFix {
    let mut fix = InitialAlignment {
        position_sigma_m: DVec3::splat(request.alignment.position_sigma_m),
        velocity_sigma_m_s: DVec3::splat(request.alignment.velocity_sigma_m_s),
        attitude_sigma_rad: DVec3::splat(request.alignment.attitude_sigma_rad),
        seed: request.seeds.alignment,
    }
    .measure(start);
    let calibration = GroundCalibration::at_rest(request.seeds.alignment);
    fix.imu_bias_prior = calibration.prior(&devices.spec().imu, devices.imu().gyro_bias_rad_s());
    fix
}

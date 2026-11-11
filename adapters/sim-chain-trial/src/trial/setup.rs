//! Building a chain trial from its spec: the aeroplane where the spec puts
//! it, its control chain under Pilotage's host, and every set's navigator
//! aligned on the declared start.

use glam::DVec3;
use sha2::{Digest, Sha256};
use sim_atmosphere::{GeoidStandardAtmosphere, geometric_to_geopotential_m, isa_at_geopotential};
use sim_core::VehicleState;
use sim_flight_chain::{
    C172Setup, ChainRoute, OwnshipFeed, RouteFix, SharedAir, VehicleClass, VehicleId, c172_board,
    c172_pilotage_airframe, c172_sensor_suite_with,
};
use sim_geo::{Geodetic, UtcTime, egm96_undulation_m};
use sim_navigate::InitialAlignment;
use sim_navigate::trial::{ImuGradeName, TrialRequest, parse_utc};

use super::C172Chain;
use crate::{ChainTrialError, ChainTrialSpec, Steering};

/// Sea-level standard density, kg/m³.
const ISA_RHO0_KG_M3: f64 = 1.225;
/// Seconds of a mean Julian year, for the geomagnetic date.
const SECONDS_PER_YEAR: f64 = 31_557_600.0;
/// Climb rate the mission may ask for toward a higher fix, m/s.
const CLIMB_RATE_M_S: f64 = 2.0;

/// What the spec resolves to before anything flies.
pub(super) struct Resolved {
    pub(super) request: TrialRequest,
    pub(super) utc: UtcTime,
    pub(super) start: Geodetic,
    pub(super) undulation_m: f64,
    pub(super) digest: String,
}

/// The spec's request as the vehicle's devices make it: the navigators
/// declare the inertial unit the C172 carries.
pub(super) fn resolve(spec: &ChainTrialSpec) -> Result<Resolved, ChainTrialError> {
    let mut request = spec.request.clone();
    request.imu_grade = ImuGradeName::ConsumerMems;
    let utc = parse_utc(&request.epoch_utc)?;
    let s = spec.start;
    let start = Geodetic::from_degrees(s.lat_deg, s.lon_deg, s.height_m);
    let bytes = serde_json_digest_input(spec)?;
    Ok(Resolved {
        undulation_m: egm96_undulation_m(start.lat_rad, start.lon_rad),
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
    let true_m_s = true_airspeed_m_s(
        spec.start.indicated_m_s,
        &resolved.start,
        resolved.undulation_m,
    )?;
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
    // The air the devices sense is the standard atmosphere entered at mean
    // sea level, so the altimeter's pressure altitude is a height above the
    // geoid, as it is in flight.
    let air = SharedAir::new(GeoidStandardAtmosphere::default());
    let board = c172_board(&setup, &suite, air)?;
    let route = ChainRoute {
        fixes: resolved
            .request
            .waypoints
            .iter()
            .map(|w| RouteFix {
                lat_deg: w.lat_deg,
                lon_deg: w.lon_deg,
                altitude_m: w.altitude_m,
            })
            .collect(),
        cruise_mps: spec.start.indicated_m_s,
        climb_rate_mps: CLIMB_RATE_M_S,
        max_horizontal_mps: spec.start.indicated_m_s,
        ownship: match spec.steering {
            Steering::FlightController => OwnshipFeed::FlightController,
            Steering::Set(_) => OwnshipFeed::Navigator,
        },
        class: VehicleClass::FixedWing,
        airframe: c172_pilotage_airframe()?,
        utc: resolved.utc,
    };
    Ok(C172Chain::new(board, &route)?)
}

/// The true airspeed that reads `indicated_m_s` at `start` in the standard
/// atmosphere, the air the aeroplane starts in.
fn true_airspeed_m_s(
    indicated_m_s: f64,
    start: &Geodetic,
    undulation_m: f64,
) -> Result<f64, ChainTrialError> {
    let geopotential = geometric_to_geopotential_m(start.height_m - undulation_m);
    let air = isa_at_geopotential(geopotential).map_err(|error| ChainTrialError::Invalid {
        detail: format!("no standard air at the start: {error}"),
    })?;
    Ok(indicated_m_s * (ISA_RHO0_KG_M3 / air.density_kg_m3).sqrt())
}

/// The sets' alignment: the declared start with the request's stated
/// uncertainty, its error drawn from the alignment seed.
pub(super) fn alignment(
    request: &TrialRequest,
    start: &VehicleState,
) -> sim_navigate::AlignmentFix {
    InitialAlignment {
        position_sigma_m: DVec3::splat(request.alignment.position_sigma_m),
        velocity_sigma_m_s: DVec3::splat(request.alignment.velocity_sigma_m_s),
        attitude_sigma_rad: DVec3::splat(request.alignment.attitude_sigma_rad),
        seed: request.seeds.alignment,
    }
    .measure(start)
}

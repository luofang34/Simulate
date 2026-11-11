//! The X500 on the vehicle's own control chain: Pilotage's mission engine flies a route
//! from a pad, commanding the Aviate flight controller in MAVLink.
//!
//! Nobody flies this vehicle by hand; the app steps the chain and draws its true state.

use glam::DVec3;
use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_flight_chain::{
    ChainSetup, DirectNoiseTier, FLIGHT_RATE_HZ, FlightChain, MissionSetup, MissionState, RouteFix,
    VehicleId,
};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use tracing::info;

use crate::error::AppSimError;

/// Corners of the demonstration route, metres ahead and to the right of the pad heading.
const ROUTE_AHEAD_RIGHT_M: [(f64, f64); 5] = [
    (0.0, 0.0),
    (800.0, 0.0),
    (800.0, 600.0),
    (0.0, 600.0),
    (0.0, 0.0),
];
/// Height of the route above the pad, metres.
const CRUISE_HEIGHT_M: f64 = 60.0;
/// Speed asked along the route, m/s; the flight controller's envelope caps it.
const CRUISE_M_S: f64 = 12.0;
/// Climb rate asked to the cruise height, m/s; the flight controller's envelope caps it.
const CLIMB_M_S: f64 = 3.0;
const NOISE_SEED: u64 = 11;
const MISSION_VEHICLE: VehicleId = VehicleId::new(1);

/// The flight chain and the mission phase last logged.
pub struct MissionRig {
    chain: FlightChain,
    logged: Option<MissionState>,
}

/// The X500 on a pad at `pad`, nose toward `heading_rad`, with the demonstration route laid
/// out ahead of it.
pub(super) fn build_mission_rig(
    pad: Geodetic,
    heading_rad: f64,
) -> Result<MissionRig, AppSimError> {
    let frame = LocalFrame::at(&pad);
    let (sin, cos) = heading_rad.sin_cos();
    let fixes = ROUTE_AHEAD_RIGHT_M
        .iter()
        .map(|&(ahead, right)| {
            let ned = DVec3::new(ahead * cos - right * sin, ahead * sin + right * cos, 0.0);
            let at = ecef_to_geodetic(geodetic_to_ecef(&pad) + frame.ned_to_ecef() * ned);
            RouteFix {
                lat_deg: at.lat_deg(),
                lon_deg: at.lon_deg(),
            }
        })
        .collect();
    let chain = FlightChain::new(&ChainSetup {
        mission: MissionSetup {
            vehicle: MISSION_VEHICLE,
            anchor: pad,
            fixes,
            cruise_height_m: CRUISE_HEIGHT_M,
            cruise_mps: CRUISE_M_S,
            climb_rate_mps: CLIMB_M_S,
        },
        heading_rad,
        noise: DirectNoiseTier::Industrial,
        noise_seed: NOISE_SEED,
    })?;
    Ok(MissionRig {
        chain,
        logged: None,
    })
}

impl MissionRig {
    /// One cycle of the flight controller and the airframe.
    pub(super) fn period(&self) -> TickSpan {
        TickSpan::from_ticks(TICKS_PER_SECOND / FLIGHT_RATE_HZ)
    }

    /// The true vehicle state.
    pub(super) fn state(&self) -> VehicleState {
        self.chain.true_state()
    }

    /// The true specific force in body forward-right-down axes, m/s².
    pub(super) fn specific_force_frd_m_s2(&self) -> DVec3 {
        self.chain.vehicle().true_specific_force_frd_m_s2()
    }

    /// Whether the flight controller's last heartbeat reported it armed.
    pub(super) fn armed(&self) -> bool {
        self.chain.vehicle().reports_armed() == Some(true)
    }

    /// One flight-controller cycle, after a mission frame when one is due.
    pub(super) fn step(&mut self) -> Result<(), AppSimError> {
        let Some(report) = self.chain.advance_tick()? else {
            return Ok(());
        };
        let at_s = self
            .chain
            .now()
            .duration_since(SimInstant::START)
            .as_seconds();
        for event in &report.events {
            info!(at_s, ?event, "Pilotage mission");
        }
        if self.logged != Some(report.state) {
            info!(at_s, state = ?report.state, mode = ?self.chain.flight_mode(), "mission phase");
            self.logged = Some(report.state);
        }
        Ok(())
    }
}

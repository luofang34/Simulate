//! The displays of an aeroplane on the real control chain, drawn from its flight
//! controller's own account ([`FcAvionics`]) and nothing else.
//!
//! The C172 mission's onboard readings ([`crate::flight::Flight::onboard_sensors`]) and its
//! guidance panel come from here, so every host that flies the mission, desktop or page,
//! draws the same displays. The flight controller holds no altimeter setting, so the air
//! data state none and the displays show pressure altitude, labelled `STD`: in the trial's
//! standard atmosphere on the geoid it is the height above mean sea level.

use aviate_core::control::ControlMode;
use glam::DVec3;
use sim_core::{SourceValidity, StampedReading, VehicleState};
use sim_flight_chain::{FcAvionics, NavigationDisplay};
use sim_geo::{LocalFrame, ecef_to_geodetic, egm96_undulation_m};
use sim_indicate::{
    AirDataReading, AirDataSample, AutopilotModeSummary, HostFlightGuidance, HostNavGuidance,
    IdentStr, PilotSelections,
};

mod navigation;
#[cfg(test)]
mod tests;

pub use navigation::navigation_lane;

/// Sea-level standard density, kg/m³, which defines indicated airspeed.
const SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225;

/// The flight controller's pitot-static reading as the displays take it: its true
/// airspeed, the density its indicated and true airspeeds imply, and its static
/// pressure, with no altimeter setting. A held reading the controller no longer flies
/// on is stale.
#[must_use]
pub fn air_data_reading(avionics: &FcAvionics) -> Option<AirDataReading> {
    let air = avionics.air?;
    let usable = air.true_m_s.is_finite() && air.true_m_s > 0.0 && air.indicated_m_s.is_finite();
    usable.then(|| {
        let ratio = air.indicated_m_s / air.true_m_s;
        StampedReading::new(
            AirDataSample {
                true_airspeed_m_s: air.true_m_s,
                density_kg_m3: SEA_LEVEL_DENSITY_KG_M3 * ratio * ratio,
                static_pressure_pa: air.static_pressure_pa,
                altimeter_setting_hpa: None,
                pec_indicated_airspeed_m_s: None,
            },
            avionics.at,
            if air.fresh {
                SourceValidity::Valid
            } else {
                SourceValidity::Stale
            },
        )
    })
}

/// What the flight controller flies toward, as the guidance panel shows it: its heading
/// target (else the heading it holds), its height target above mean sea level, the
/// standard altimeter setting, and its mode, beside Pilotage's `navigation` on the HSI.
/// `estimate` is the controller's.
#[must_use]
pub fn guidance(
    avionics: &FcAvionics,
    estimate: &VehicleState,
    navigation: Option<&NavigationDisplay>,
    source_epoch: u32,
) -> HostFlightGuidance {
    let position = ecef_to_geodetic(estimate.position_ecef);
    let undulation_m = egm96_undulation_m(position.lat_rad, position.lon_rad);
    let targets = avionics.targets;
    HostFlightGuidance {
        selections: PilotSelections {
            heading_rad: targets.heading_rad.unwrap_or_else(|| heading_rad(estimate)),
            altitude_m: targets.altitude_m.map(|height| height - undulation_m),
            baro_setting_hpa: None,
        },
        nav: navigation.map_or(HostNavGuidance::None, |display| {
            navigation_lane(display, avionics.at, source_epoch)
        }),
        autopilot: AutopilotModeSummary {
            lateral: ident(lateral_label(avionics)),
            vertical: ident(vertical_label(avionics)),
            armed_approach: false,
            engaged: avionics.armed,
        },
        source_epoch,
    }
}

/// True heading of the nose, radians.
fn heading_rad(vehicle: &VehicleState) -> f64 {
    let local = LocalFrame::at(&ecef_to_geodetic(vehicle.position_ecef));
    let nose = vehicle.attitude * DVec3::X;
    nose.dot(local.east()).atan2(nose.dot(local.north()))
}

fn lateral_label(avionics: &FcAvionics) -> &'static str {
    match (avionics.mode, avionics.targets.heading_rad) {
        (ControlMode::DeviationTracking, _) => "TRK",
        (ControlMode::PositionHold, _) => "POS",
        (_, Some(_)) => "HDG",
        (ControlMode::Rate | ControlMode::Attitude, None) => "ATT",
        (ControlMode::AltitudeHold | ControlMode::VelocityControl, None) => "LVL",
    }
}

fn vertical_label(avionics: &FcAvionics) -> &'static str {
    match (
        avionics.targets.altitude_m,
        avionics.targets.vertical_speed_m_s,
    ) {
        (Some(_), _) => "ALT",
        (None, Some(_)) => "VS",
        (None, None) => "PIT",
    }
}

fn ident(label: &str) -> Option<IdentStr> {
    IdentStr::new(label).ok()
}

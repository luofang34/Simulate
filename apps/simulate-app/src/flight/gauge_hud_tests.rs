//! The P-38G's panel needles and its head-up display read the same air in
//! the same frame: each needle stands where its dial's own scale puts the
//! value the head-up display shows, give or take what the mechanism really
//! adds (bench instrument error, the static port's position error, lag).

use std::f64::consts::TAU;

use mbd_airdata::impact_pressure_from_cas_pa;
use sim_core::{ControlInputs, InstrumentNeedles};
use sim_indicate::PointerValues;
use sim_indicate::{AltitudeSource, DisplayAxes, FeedConfig, FlightSample, HeadingMode, StateFeed};
use sim_scenario::SimTimeScale;
use sim_scenery::{AerodromeId, OsmAerodromeDiagram};

use super::*;
use crate::vehicles::FreeFlightVehicle;

const M_S_TO_MPH: f64 = 1.0 / 0.447_04;
const M_TO_FT: f64 = 1.0 / 0.3048;
/// The pack's bench instrument error on the fighter's airspeed indicator.
const ASI_INSTRUMENT_ERROR_MPH: f64 = 2.0;

/// Signed difference `a - b` of two angles, degrees in `(-180, 180]`.
fn angle_diff_deg(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(TAU).to_degrees();
    if d > 180.0 { d - 360.0 } else { d }
}

/// Every pointer at `value`, the drum at `window`.
fn primary(value: f32, window: Option<f32>) -> PointerValues {
    PointerValues {
        pointers: [Some(value); 4],
        window,
    }
}

fn airborne_p38() -> Flight {
    let mut flight = Flight::new().expect("flight");
    flight
        .select_vehicle(FreeFlightVehicle::JsbsimP38g)
        .expect("P-38G");
    let diagram = OsmAerodromeDiagram::builtin(AerodromeId::NewarkKewr).expect("KEWR");
    flight
        .spawn_at_aerodrome(&diagram, 0, SpawnMode::ShortFinal)
        .expect("short final");
    let cruise = ControlInputs {
        throttle: 0.8,
        ..ControlInputs::default()
    };
    for _ in 0..40 {
        flight
            .advance_by(0.05, cruise, SimTimeScale::RealTime)
            .expect("fly");
    }
    flight
}

#[test]
fn p38_needles_stand_where_their_scales_put_the_hud_airspeed_and_altitude() {
    let flight = airborne_p38();
    let air = flight.onboard_sensors().air_data.expect("air data");
    let mut feed = StateFeed::new(FeedConfig {
        body: sim_geo::BodyEllipsoid::WGS84,
        axes: DisplayAxes::AEROSPACE,
        origin: sim_geo::Geodetic::from_degrees(40.69, -74.17, 0.0),
        origin_id: 1,
        altitude: AltitudeSource::Barometric,
        heading_mode: HeadingMode::EarthMagnetic,
        deviation_scale: sim_indicate::DeviationScaleProfile::GNSS_NAVIGATOR,
    });
    let sample = FlightSample {
        air_data: Some(air),
        ..FlightSample::truth(air.timestamp, flight.vehicle_state())
    };
    let hud = feed.state(&sample).expect("HUD state");
    let hud_air = hud.air.data.expect("HUD air data");
    let hud_mph = f64::from(hud_air.ias_mps.expect("HUD airspeed")) * M_S_TO_MPH;
    let hud_ft = f64::from(hud.altitude.sample_m.expect("HUD altitude")) * M_TO_FT;
    assert!(hud_mph > 80.0, "airborne at {hud_mph} mph");

    let needles = flight.display_articulation().instrument_needles;
    let dials = flight.gauges.dials();
    let angle = |slot: usize, hand: usize| {
        needles.angle_rad(InstrumentNeedles::channel(slot, hand).expect("channel"))
    };

    // Airspeed: the long hand against the dial's own law at the HUD value.
    let asi = &dials.slot(0).expect("airspeed slot").dial;
    let at_hud = asi.pose(&primary(hud_mph as f32, None));
    let expected = f64::from(at_hud.pointers[0].angle_rad);
    let off = angle_diff_deg(angle(0, 0), expected);
    let slope_deg_per_mph = {
        let up = asi.pose(&primary(hud_mph as f32 + 1.0, None));
        angle_diff_deg(f64::from(up.pointers[0].angle_rad), expected)
    };
    let off_mph = off / slope_deg_per_mph;
    assert!(
        (off_mph - ASI_INSTRUMENT_ERROR_MPH).abs() < 1.0,
        "airspeed needle {off_mph:.2} mph from the HUD's {hud_mph:.1} mph"
    );

    // Altitude: all three hands against the HUD value, within the static
    // port's position error, which the HUD's ambient static does not carry.
    let ias = air.value.indicated_airspeed_m_s();
    let cas = air.value.calibrated_airspeed_m_s();
    let port_error_pa = impact_pressure_from_cas_pa(cas) - impact_pressure_from_cas_pa(ias);
    let port_error_ft = port_error_pa.abs() / (air.value.density_kg_m3 * 9.806_65) * M_TO_FT;
    let alt = &dials.slot(1).expect("altimeter slot").dial;
    let at_hud = alt.pose(&primary(hud_ft as f32, Some(29.92)));
    // Spec order, lowest first: ten-thousands, thousands, hundreds.
    for (hand, deg_per_ft) in [(2, 0.36), (1, 0.036), (0, 0.0036)] {
        let expected = f64::from(at_hud.pointers[hand].angle_rad);
        let off_ft = angle_diff_deg(angle(1, hand), expected) / deg_per_ft;
        assert!(
            off_ft.abs() < port_error_ft + 5.0,
            "hand {hand} reads {off_ft:.1} ft from the HUD's {hud_ft:.0} ft \
             (port error {port_error_ft:.1} ft)"
        );
    }
}

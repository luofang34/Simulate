//! Map-rig regressions that need no GPU: the vertical datum of drawn altitudes and the
//! ownship airframe chosen for what is flying.

use sim_core::{AirframeArticulation, VehicleState};
use sim_geo::{Geodetic, egm96_undulation_m};
use sim_maplibre::OwnshipVisualKind;
use sim_scenario::ScenarioProfile;
use sim_scenery::{CatalogScenery, CatalogVerticalDatum};

use super::aircraft::{ownship_aircraft, ownship_visual};
use super::stamps::geoid_undulation_m;
use crate::vehicles::FreeFlightVehicle;

/// Newark Liberty field reference point and its NASR field elevation, metres MSL.
const KEWR: (f64, f64, f64) = (40.692_5, -74.168_7, 5.33);

#[test]
fn real_world_ownship_sits_at_its_msl_altitude_not_its_ellipsoidal_height() {
    let (lat, lon, msl) = KEWR;
    let (lat_rad, lon_rad) = (lat.to_radians(), lon.to_radians());
    let n = geoid_undulation_m(None, lat_rad, lon_rad);
    assert!((n - egm96_undulation_m(lat_rad, lon_rad)).abs() < 1e-9);
    assert!((-36.0..-30.0).contains(&n), "KEWR undulation {n}");
    let ellipsoidal = msl + n;
    assert!(
        ellipsoidal < -25.0,
        "the C172's raw WGS84 height {ellipsoidal}"
    );
    let vehicle =
        VehicleState::level_flight(&Geodetic::from_degrees(lat, lon, ellipsoidal), 0.0, 0.0);
    let ownship = ownship_aircraft(
        &vehicle,
        glam::DVec3::ZERO,
        AirframeArticulation::default(),
        n,
        None,
    );
    let altitude = ownship.placement.altitude_meters;
    assert!(
        (altitude - msl).abs() < 0.05,
        "AboveMsl altitude {altitude}"
    );
}

#[test]
fn an_ellipsoidal_catalog_has_no_undulation_and_an_orthometric_one_does() {
    let (lat, lon, _) = KEWR;
    let (lat_rad, lon_rad) = (lat.to_radians(), lon.to_radians());
    let ellipsoid = Some(CatalogVerticalDatum::EllipsoidWgs84);
    assert!(geoid_undulation_m(ellipsoid, lat_rad, lon_rad).abs() < f64::EPSILON);
    let msl = Some(CatalogVerticalDatum::OrthometricMslEgm96);
    assert!(geoid_undulation_m(msl, lat_rad, lon_rad) < -30.0);
    let synthetic = CatalogScenery::synthetic_regional_network().expect("synthetic catalog");
    let n = geoid_undulation_m(Some(synthetic.vertical_datum()), lat_rad, lon_rad);
    let expected = match synthetic.vertical_datum() {
        CatalogVerticalDatum::EllipsoidWgs84 => 0.0,
        CatalogVerticalDatum::OrthometricMslEgm96 => egm96_undulation_m(lat_rad, lon_rad),
    };
    assert!((n - expected).abs() < 1e-9);
}

#[test]
fn the_ownship_airframe_follows_the_scenario_then_the_free_flight_plant() {
    use FreeFlightVehicle as V;
    use OwnshipVisualKind as K;
    use ScenarioProfile as P;
    for (profile, vehicle, kind) in [
        (Some(P::Apollo11CapsuleReentry), None, K::Hidden),
        (Some(P::Apollo11LunarLanding), None, K::LunarLander),
        (Some(P::CorridorNominalCruise), None, K::FixedWingUav),
        (
            Some(P::CorridorSameCellLoiter),
            Some(V::JsbsimC172),
            K::FixedWingUav,
        ),
        (Some(P::AirportKewrToweredPattern), None, K::C172),
        (Some(P::Airport47NUncontrolledPattern), None, K::C172),
        (None, Some(V::JsbsimC172), K::C172),
        (None, Some(V::JsbsimP38g), K::P38g),
        (None, Some(V::AviateFixedWingDirect), K::C172),
        (None, Some(V::Kinematic), K::FixedWingUav),
        (None, Some(V::AviateX500Direct), K::QuadrotorX500),
        (None, None, K::Raketa),
    ] {
        assert_eq!(
            ownship_visual(profile, vehicle),
            kind,
            "{profile:?} {vehicle:?}"
        );
    }
}

#[test]
fn the_globe_outlines_both_ends_of_every_runway_and_highlights_the_chosen_one() {
    let (_, kewr) = sim_scenery::AirportDatabase::builtin()
        .by_code("KEWR")
        .expect("KEWR");
    let overlays = super::globe::runway_overlays(kewr, Some("22L"));
    assert_eq!(overlays.len(), kewr.runways.len() * 2);
    let selected: Vec<&str> = overlays
        .iter()
        .filter(|o| o.selected)
        .map(|o| o.designator.as_str())
        .collect();
    assert_eq!(selected, ["22L"]);
    for overlay in &overlays {
        let twin = overlays
            .iter()
            .find(|o| o.threshold_lat_deg == overlay.opposite_lat_deg)
            .expect("the opposite end");
        assert_eq!(twin.opposite_lon_deg, overlay.threshold_lon_deg);
    }
}

#[test]
fn a_globe_click_reaches_further_when_zoomed_out_and_near_the_equator() {
    let near = super::globe::pick_radius_km(40.0, 8.0, 18.0);
    let far = super::globe::pick_radius_km(40.0, 3.0, 18.0);
    assert!(far > near * 20.0, "{near} {far}");
    assert!(super::globe::pick_radius_km(0.0, 8.0, 18.0) > near);
    assert!((0.5..=400.0).contains(&super::globe::pick_radius_km(89.9, 16.0, 18.0)));
}

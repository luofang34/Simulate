//! The C172 a link's page flies: the page steps the one trial, the app's plant only shows
//! what the page last published.

use sim_chain_trial::{ChainTrial, first_link};
use sim_core::ControlInputs;
use sim_navigate::trial::DirectoryStores;

use crate::vehicles::{FreeFlightPlant, PageFeed, PageSnapshot};
use crate::world_surface::WorldSurfaceHandle;

#[test]
fn the_page_mission_shows_what_the_page_published_and_steps_nothing() {
    let spec = first_link().unwrap();
    let mut trial = ChainTrial::new(&spec, &DirectoryStores).unwrap();
    let feed = PageFeed::new(1_781_524_800.0, PageSnapshot::of(&trial, None));
    let mut plant = FreeFlightPlant::page_mission(feed.clone());
    let world = WorldSurfaceHandle::new();
    let start = plant.state();

    // The plant steps nothing of its own: without a new snapshot it stays put.
    for _ in 0..50 {
        plant.step(ControlInputs::default(), &world).unwrap();
    }
    assert_eq!(plant.state(), start);
    assert_eq!(plant.stated_epoch_utc_s(), Some(1_781_524_800.0));

    // The page steps its trial and publishes; the plant, stepped to the published
    // instant one controller cycle at a time, shows the published chain.
    trial.step_until(5.0).unwrap();
    feed.publish(PageSnapshot::of(&trial, Some("Enroute".to_owned())));
    for _ in 50..1250 {
        plant.step(ControlInputs::default(), &world).unwrap();
    }
    assert_eq!(plant.state(), trial.chain().true_state());
    assert_ne!(plant.state().position_ecef, start.position_ecef);
    assert_eq!(
        plant.chain_readout().and_then(|(phase, _)| phase),
        Some("Enroute")
    );
    assert_eq!(
        plant.aviate_armed(),
        Some(trial.chain().board().controller().is_armed())
    );
}

#[test]
fn a_flight_shown_a_page_mission_flies_the_pages_chain() {
    let spec = first_link().unwrap();
    let mut trial = ChainTrial::new(&spec, &DirectoryStores).unwrap();
    trial.step_until(2.0).unwrap();
    let feed = PageFeed::new(1_781_524_800.0, PageSnapshot::of(&trial, None));
    let mut flight = crate::flight::Flight::new().unwrap();
    assert!(flight.onboard_avionics_shown());
    flight.show_page_mission(feed).unwrap();
    // The displays read the page's flight controller, never the page's truth.
    assert!(flight.onboard_avionics_shown());
    let (avionics, held) = flight.chain_avionics().unwrap();
    assert_eq!(avionics, trial.avionics());
    assert_eq!(
        held,
        trial
            .avionics()
            .estimate
            .unwrap_or(trial.chain().true_state())
    );
    assert_eq!(flight.vehicle_state(), trial.chain().true_state());
    assert!(flight.chain_readout().is_some());
}

/// A page paces its flight, so the app's clock trails the page's; the displays read one
/// instant, the flight controller's latest at the app's clock, so Indicate never sees a
/// frame's readings skewed apart nor a sample from the app's future.
#[test]
fn a_page_missions_displays_read_the_controllers_instant() {
    let spec = first_link().unwrap();
    let mut trial = ChainTrial::new(&spec, &DirectoryStores).unwrap();
    let feed = PageFeed::new(1_781_524_800.0, PageSnapshot::of(&trial, None));
    let mut flight = crate::flight::Flight::new().unwrap();
    flight.show_page_mission(feed.clone()).unwrap();
    // Past the arming, so the controller flies on its air data. The page publishes every
    // frame; the app's clock trails it, as a page's display time does.
    for frame in 0..250 {
        trial.step_until(trial.now_s() + 0.02).unwrap();
        feed.publish(PageSnapshot::of(&trial, Some("Enroute".to_owned())));
        if frame > 0 {
            flight
                .advance_by(
                    0.02,
                    ControlInputs::default(),
                    sim_scenario::SimTimeScale::RealTime,
                )
                .unwrap();
        }
    }
    assert!(
        flight.now() < trial.avionics().at,
        "the app's clock trails the page's"
    );
    let now_s = flight
        .now()
        .duration_since(sim_core::SimInstant::START)
        .as_seconds();
    let at = feed.at(now_s).avionics.at;
    assert!(at <= flight.now(), "no sample from the app's future");
    assert_eq!(flight.display_instant(), at);
    let onboard = flight.onboard_sensors();
    let cockpit = onboard.cockpit;
    for stamp in [
        cockpit.timestamp,
        cockpit.attitude.timestamp,
        cockpit.pgns_attitude.timestamp,
        cockpit.ags_attitude.timestamp,
        cockpit.body_rate_rad_s.timestamp,
    ] {
        assert_eq!(stamp, at);
    }
    assert_eq!(onboard.air_data.expect("air data").timestamp, at);
}

#[test]
fn a_page_flight_hands_no_optical_exposures_to_the_maps() {
    let spec = first_link().unwrap();
    let mut trial = ChainTrial::new(&spec, &DirectoryStores).unwrap();
    let feed = PageFeed::new(1_781_524_800.0, PageSnapshot::of(&trial, None));
    let mut page = crate::flight::Flight::new().unwrap();
    page.show_page_mission(feed.clone()).unwrap();
    let mut own = crate::flight::Flight::new().unwrap();
    for _ in 0..30 {
        trial.step_until(trial.now_s() + 0.02).unwrap();
        feed.publish(PageSnapshot::of(&trial, Some("Enroute".to_owned())));
        for flight in [&mut page, &mut own] {
            flight
                .advance_by(
                    0.02,
                    ControlInputs::default(),
                    sim_scenario::SimTimeScale::RealTime,
                )
                .unwrap();
        }
    }
    assert!(
        !own.drain_sensor_exposures().is_empty(),
        "the app's own flight exposes its optical sensor"
    );
    assert!(page.drain_sensor_exposures().is_empty());
}

/// The moving maps place the aeroplane where the navigator the mission steers on does,
/// which is neither the truth nor the flight controller's own estimate.
#[test]
fn a_page_missions_maps_draw_the_steered_navigators_fix() {
    let spec = first_link().unwrap();
    let mut trial = ChainTrial::new(&spec, &DirectoryStores).unwrap();
    assert_eq!(
        PageSnapshot::of(&trial, None).steered,
        None,
        "no solution yet"
    );
    trial.step_until(20.0).unwrap();
    let steered = trial.steered().unwrap();
    let solution = steered.solution.unwrap();
    let feed = PageFeed::new(1_781_524_800.0, PageSnapshot::of(&trial, None));
    let mut flight = crate::flight::Flight::new().unwrap();
    flight.show_page_mission(feed).unwrap();
    let fix = flight.chain_steered().unwrap();
    assert_eq!(fix.lat_deg, solution.position.latitude_rad.to_degrees());
    assert_eq!(fix.lon_deg, solution.position.longitude_rad.to_degrees());
    assert!(fix.sigma_horizontal_m > 0.0);
    let (_, held) = flight.chain_avionics().unwrap();
    let controller = sim_geo::ecef_to_geodetic(held.position_ecef);
    assert_ne!(
        (fix.lat_deg, fix.lon_deg),
        (
            controller.lat_rad.to_degrees(),
            controller.lon_rad.to_degrees()
        ),
        "the navigator's fix, not the flight controller's estimate"
    );
}

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

    // The page steps its trial and publishes; the plant shows the published chain.
    trial.step_until(5.0).unwrap();
    feed.publish(PageSnapshot::of(&trial, Some("Enroute".to_owned())));
    plant.step(ControlInputs::default(), &world).unwrap();
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

/// A page paces its flight, so the app's clock trails the page's by up to a frame; the
/// displays still read one instant, the flight controller's, so Indicate never sees a
/// frame's readings skewed apart.
#[test]
fn a_page_missions_displays_read_the_controllers_instant() {
    let spec = first_link().unwrap();
    let mut trial = ChainTrial::new(&spec, &DirectoryStores).unwrap();
    let feed = PageFeed::new(1_781_524_800.0, PageSnapshot::of(&trial, None));
    let mut flight = crate::flight::Flight::new().unwrap();
    flight.show_page_mission(feed.clone()).unwrap();
    // Past the arming, so the controller flies on its air data; the app's clock takes a
    // few cycles, enough to read the page's latest, while the page has flown on.
    trial.step_until(5.0).unwrap();
    feed.publish(PageSnapshot::of(&trial, Some("Enroute".to_owned())));
    flight
        .advance_by(
            0.02,
            ControlInputs::default(),
            sim_scenario::SimTimeScale::RealTime,
        )
        .unwrap();
    let at = trial.avionics().at;
    assert!(flight.now() < at, "the app's clock trails the page's");
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

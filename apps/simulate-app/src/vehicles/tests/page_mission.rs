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

use std::cell::Cell;

use navigate_data::Sha256Digest;
use sim_chain_trial::{ChainTrial, Steering};
use sim_link::decode_fragment;
use sim_navigate::trial::{ProgressRecord, SensorKind};

use super::{ASSUMED_ACCURACY, ChainFlight, PAGE_STORE, chain_spec, page_stores};
use crate::{Clock, FlightStepper, HostError, Monitor, MonitorFeed, hosted_request, run_slice};

const FIRST: &str = "v=1&r=41.700000,-1.050000,,,ZA1;41.700000,-0.800000,,,ZA2;41.800000,-0.800000,,,ZA3;\
41.800000,-1.000000,,,ZA4;41.740000,-1.050000,,,ZA5&h=2000.0&hd=msl&start=41.700000,-1.070000,,90.0\
&t=20260615T120400Z&veh=c172&seed=1&nav=ins.baro.visual;ins.baro;ins.baro.gnss&fly=ins.baro.gnss\
&go=1&mb=200";
/// The same route, no visual set, no start.
const BLIND: &str = "v=1&r=41.700000,-1.050000,,,ZA1;41.700000,-0.800000,,,ZA2;41.800000,-0.800000,,,ZA3\
&h=2000.0&hd=msl&t=20260615T120400Z&veh=c172&seed=1&nav=ins.baro;ins.baro.gnss&fly=ins.baro.gnss\
&out=gnss:30-60;baro:90-&mb=200";

#[test]
fn the_chain_flies_the_links_start_steering_and_package() {
    let link = decode_fragment(FIRST).unwrap();
    let hosted = hosted_request(&link).unwrap();
    let identity = Sha256Digest::of(b"package");
    let spec = chain_spec(&link, &hosted, Some(identity)).unwrap();
    assert_eq!(
        (
            spec.start.lat_deg,
            spec.start.lon_deg,
            spec.start.heading_deg
        ),
        (41.7, -1.07, 90.0)
    );
    assert_eq!(spec.start.height_m, hosted.start.unwrap().altitude_m);
    assert_eq!(spec.start.indicated_m_s, 55.0);
    assert_eq!(spec.steering, Steering::Set("ins.baro.gnss".to_owned()));
    let maps = spec.request.maps.unwrap();
    assert_eq!(maps.onboard.store, PAGE_STORE);
    assert_eq!(maps.onboard.package, identity.to_string());
    assert_eq!(maps.onboard.assumed_accuracy, Some(ASSUMED_ACCURACY));
    assert!(maps.world_is_onboard());
    assert!(spec.outages.is_empty());
    assert!(matches!(
        chain_spec(&link, &hosted, None),
        Err(HostError::Render { .. })
    ));
}

#[test]
fn outages_are_the_devices_and_a_link_without_a_start_begins_on_its_route() {
    let link = decode_fragment(BLIND).unwrap();
    let hosted = hosted_request(&link).unwrap();
    assert!(!hosted.trial.availability.is_empty());
    let spec = chain_spec(&link, &hosted, None).unwrap();
    assert!(spec.request.maps.is_none());
    assert!(spec.request.availability.is_empty());
    assert!(
        spec.request
            .sensor_sets
            .iter()
            .all(|set| set.availability.is_empty())
    );
    let outages: Vec<(SensorKind, f64, Option<f64>)> = spec
        .outages
        .iter()
        .map(|o| (o.sensor, o.start_s, o.end_s))
        .collect();
    assert_eq!(
        outages,
        [
            (SensorKind::Gnss, 30.0, Some(60.0)),
            (SensorKind::Baro, 90.0, None)
        ]
    );
    let first = &spec.request.waypoints[0];
    assert_eq!(
        (spec.start.lat_deg, spec.start.lon_deg),
        (first.lat_deg, first.lon_deg)
    );
    assert!(
        (spec.start.heading_deg - 90.0).abs() < 0.2,
        "{}",
        spec.start.heading_deg
    );
}

/// A wall clock that moves a millisecond each time it is read.
struct Ticking(Cell<f64>);

impl Clock for Ticking {
    fn now_s(&self) -> f64 {
        let now = self.0.get() + 0.001;
        self.0.set(now);
        now
    }
}

#[test]
fn the_chain_flies_as_a_page_flight_and_a_switch_reaches_the_link() {
    let link = decode_fragment(BLIND).unwrap();
    let hosted = hosted_request(&link).unwrap();
    let spec = chain_spec(&link, &hosted, None).unwrap();
    let stores = page_stores(navigate_data::MemoryStore::default());
    let trial = ChainTrial::new(&spec, &stores).unwrap();
    let mut flight = ChainFlight::new(trial, None, Ticking(Cell::new(0.0)));
    let clock = Ticking(Cell::new(0.0));
    let (mut out, mut monitor) = (Vec::new(), Monitor::default());
    while flight.sim_time_s() < 4.0 {
        run_slice(&mut flight, &clock, 0.05, 4.0, &mut out).unwrap();
    }
    flight.switch(SensorKind::Stars, false);
    flight.switch(SensorKind::Gnss, false);
    while flight.sim_time_s() < 6.0 {
        run_slice(&mut flight, &clock, 0.05, 6.0, &mut out).unwrap();
    }
    flight.switch(SensorKind::Gnss, true);
    assert_eq!(flight.take_camera_wall_s(), 0.0);
    let started = out
        .iter()
        .filter(|feed| matches!(feed, MonitorFeed::Progress(ProgressRecord::Started(_))))
        .count();
    assert_eq!(started, 1);
    for feed in out {
        monitor.apply(feed);
    }
    let names: Vec<&str> = monitor.traces().iter().map(|t| t.name.as_str()).collect();
    assert!(names.contains(&"ins.baro.gnss"), "{names:?}");
    assert!(monitor.traces().iter().all(|t| !t.points().is_empty()));
    let gnss: Vec<_> = flight
        .outages()
        .into_iter()
        .filter(|(sensor, ..)| *sensor == SensorKind::Gnss)
        .map(|(_, start_s, end_s)| (start_s, end_s))
        .collect();
    assert_eq!(gnss.len(), 2, "{gnss:?}");
}

#[test]
fn completing_the_plan_does_not_end_a_page_flight() {
    let event = |kind: &str| {
        sim_navigate::trial::ProgressRecord::from_json_line(&format!(
            r#"{{"type":"event","t_s":1.0,"set":null,"kind":{kind}}}"#
        ))
        .unwrap()
    };
    let complete =
        event(r#""waypoint_sequenced","waypoint_index":4,"ident":"ZA5","plan_complete":true"#);
    assert!(super::completes_plan(&complete));
    assert!(!super::ends_flight(&complete));
    let sequenced =
        event(r#""waypoint_sequenced","waypoint_index":1,"ident":"ZA2","plan_complete":false"#);
    assert!(!super::completes_plan(&sequenced));
    let terminated = event(r#""terminated","rule":"max_duration","detail":"1 s""#);
    assert!(super::ends_flight(&terminated));
}

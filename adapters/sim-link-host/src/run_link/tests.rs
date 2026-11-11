use sim_link::{Sensor, decode_fragment, decode_url};
use sim_navigate::trial::SensorKind;

use super::SwitchLog;
use crate::hosted_request;

const FIRST: &str = "v=1&r=41.700000,-1.050000,,,ZA1;41.700000,-0.800000,,,ZA2;41.800000,-0.800000,,,ZA3;\
41.800000,-1.000000,,,ZA4;41.740000,-1.050000,,,ZA5&h=2000.0&hd=msl&start=41.700000,-1.070000,,90.0\
&t=20260615T120400Z&veh=c172&seed=1&nav=ins.baro.visual;ins.baro;ins.baro.gnss&fly=ins.baro.gnss\
&go=1&mb=200";
const BASE: &str = "https://luofang34.github.io/Simulate/";

#[test]
fn a_run_without_switches_is_its_link() {
    let link = decode_fragment(FIRST).unwrap();
    let log = SwitchLog::default();
    assert!(log.is_empty());
    assert_eq!(log.link_of_run(&link), link);
    assert_eq!(log.url(&link, BASE).unwrap(), link.url(BASE).unwrap());
}

#[test]
fn switches_become_outages_the_reopened_link_carries() {
    let link = decode_fragment(FIRST).unwrap();
    let mut log = SwitchLog::default();
    log.off(SensorKind::Gnss, 120.004);
    log.off(SensorKind::Gnss, 130.0);
    log.on(SensorKind::Gnss, 300.5);
    log.off(SensorKind::TerrainCamera, 600.0);
    log.on(SensorKind::Baro, 10.0);
    let url = log.url(&link, BASE).unwrap();
    assert!(url.contains("&out=gnss:120.004-300.5;cam:600-&"), "{url}");
    let reopened = decode_url(&url).unwrap().unwrap();
    assert_eq!(reopened, log.link_of_run(&link));
    let schedules = hosted_request(&reopened).unwrap().trial.availability;
    let gnss = &schedules[0];
    assert_eq!(gnss.sensor, SensorKind::Gnss);
    assert_eq!(
        (gnss.unavailable[0].start_s, gnss.unavailable[0].end_s),
        (120.004, Some(300.5))
    );
    assert_eq!(schedules[1].unavailable[0].end_s, None);
}

#[test]
fn a_switch_inside_a_stated_outage_joins_it() {
    let stated = FIRST.replace("&go=1", "&out=gnss:100-200,200-300&go=1");
    let link = decode_fragment(&stated).unwrap();
    let mut log = SwitchLog::default();
    log.off(SensorKind::Gnss, 150.0);
    log.on(SensorKind::Gnss, 250.0);
    log.off(SensorKind::Gnss, 400.0);
    let run = log.link_of_run(&link);
    let gnss = run
        .outages
        .iter()
        .find(|o| o.sensor == Sensor::Gnss)
        .unwrap();
    let spans: Vec<(f64, Option<f64>)> = gnss
        .spans
        .iter()
        .map(|span| (span.start.as_f64(), span.end.map(|end| end.as_f64())))
        .collect();
    assert_eq!(spans, [(100.0, Some(300.0)), (400.0, None)]);
    assert!(log.url(&link, BASE).is_ok());
}

#[test]
fn a_switch_back_at_the_same_instant_leaves_no_outage() {
    let link = decode_fragment(FIRST).unwrap();
    let mut log = SwitchLog::default();
    log.off(SensorKind::Gnss, 50.0);
    log.on(SensorKind::Gnss, 50.0);
    assert!(log.is_empty());
    assert_eq!(log.link_of_run(&link), link);
}

use navigate_data::Sha256Digest;
use sim_link::{Sensor, decode_fragment, decode_url};
use sim_navigate::trial::SensorKind;

use super::{SpoofRun, SwitchLog};
use crate::{chain_spec, hosted_request, link_spoof};

const FIRST: &str = "v=2&r=IF,ZA1:41.7/-1.05;TF,ZA2:41.7/-0.8;TF,ZA3:41.8/-0.8;TF,ZA4:41.8/-1;TF,ZA5:41.74/-1.05&h=2000.0&hd=msl&start=41.700000,-1.070000,,90.0\
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

#[test]
fn a_flights_outages_rebuild_the_same_spans() {
    let link = decode_fragment(FIRST).unwrap();
    let log = SwitchLog::of_outages(&[
        (SensorKind::Gnss, 120.004, Some(300.5)),
        (SensorKind::TerrainCamera, 600.0, None),
    ]);
    assert!(
        log.url(&link, BASE)
            .unwrap()
            .contains("&out=gnss:120.004-300.5;cam:600-&")
    );
}

#[test]
fn the_spoofer_a_flight_flew_is_the_spoof_the_reopened_link_carries() {
    let link = decode_fragment(FIRST).unwrap();
    let run = SpoofRun {
        rate_m_s: 1.0,
        bearing_deg: 90.0,
        spans: vec![(120.004, Some(300.5)), (600.0, None)],
    };
    let log = SwitchLog::default().with_spoof(Some(&run));
    assert!(!log.is_empty());
    let url = log.url(&link, BASE).unwrap();
    assert!(
        url.contains("&spoof=gnss:1,90:120.004-300.5,600-&"),
        "{url}"
    );
    let reopened = decode_url(&url).unwrap().unwrap();
    assert_eq!(reopened, log.link_of_run(&link));
    let hosted = hosted_request(&reopened).unwrap();
    let flown = chain_spec(&reopened, &hosted, Some(Sha256Digest::of(b"package")))
        .unwrap()
        .gnss_spoof
        .unwrap();
    assert_eq!(link_spoof(&flown), Some(run));
}

#[test]
fn a_spoofer_that_never_held_the_receiver_leaves_the_link_as_it_is() {
    let stated = FIRST.replace("&go=1", "&spoof=gnss:2.5,45:10-20&go=1");
    let link = decode_fragment(&stated).unwrap();
    assert_eq!(SwitchLog::default().link_of_run(&link), link);
    let idle = SpoofRun {
        rate_m_s: 1.0,
        bearing_deg: 90.0,
        spans: Vec::new(),
    };
    assert_eq!(
        SwitchLog::default()
            .with_spoof(Some(&idle))
            .link_of_run(&link),
        link
    );
}

use sim_link::{SimulationLink, decode_fragment};
use sim_navigate::NavMeans;
use sim_navigate::trial::{GuidanceSource, SeedSpec, SensorKind};

use crate::{HostError, VEHICLE_CAMERA, hosted_request};

/// The reference encoder's Zaragoza route with every optional parameter.
const DAY: &str = "v=2&r=IF,LEZG:41.666242/-1.041553;TF,Z1:41.75/-0.8,o;TF,41.6/-0.62&h=1981.2&hd=msl&start=41.666250,-1.141550,,75.0&t=20260615T120000Z&veh=c172&seed=1\
&nav=ins.baro.visual;ins.baro;ins.baro.gnss&fly=ins.baro.gnss&out=gnss:300-600,600-700,900-;cam:5-6\
&cam=1600x1200,90,45.5,2&go=1&mb=200";
/// A route no set of which is visual, so the trial takes it without maps.
const PLAIN: &str = "v=2&r=IF,LEZG:41.666242/-1.041553;TF,Z1:41.75/-0.8;TF,Z2:41.6/-0.62&h=2000&hd=hae&t=20260615T120000Z&veh=c172&seed=7&nav=ins.baro;ins.baro.gnss\
&out=gnss:300-600,900-&mb=200";

fn link(fragment: &str) -> SimulationLink {
    decode_fragment(fragment).unwrap()
}

#[test]
fn a_link_without_a_visual_set_maps_to_a_request_the_trial_accepts() {
    let hosted = hosted_request(&link(PLAIN)).unwrap();
    hosted.trial.validate().unwrap();
    assert_eq!(hosted.trial.epoch_utc, "2026-06-15T12:00:00Z");
    assert_eq!(hosted.trial.vehicle.cruise_speed_m_s, 55.0);
    assert_eq!(hosted.trial.vehicle.guidance, GuidanceSource::Estimate);
    assert_eq!((hosted.camera, hosted.start), (None, None));
    assert_eq!(hosted.steering_set, "ins.baro");
}

#[test]
fn heights_above_mean_sea_level_become_heights_above_the_ellipsoid() {
    let hosted = hosted_request(&link(DAY)).unwrap();
    let first = &hosted.trial.waypoints[0];
    let undulation =
        sim_geo::egm96_undulation_m(41.666242_f64.to_radians(), (-1.041553_f64).to_radians());
    assert!(
        undulation.abs() > 1.0,
        "the geoid is not the ellipsoid here: {undulation}"
    );
    assert_eq!(first.altitude_m, 1981.2 + undulation);
    let start = hosted.start.unwrap();
    let at_start =
        sim_geo::egm96_undulation_m(41.66625_f64.to_radians(), (-1.14155_f64).to_radians());
    assert_eq!(start.altitude_m, 1981.2 + at_start);
    assert_eq!(start.heading_deg, Some(75.0));

    let ellipsoid = hosted_request(&link(PLAIN)).unwrap();
    assert!(
        ellipsoid
            .trial
            .waypoints
            .iter()
            .all(|waypoint| waypoint.altitude_m == 2000.0)
    );
}

#[test]
fn waypoints_keep_their_names_turns_and_order() {
    let hosted = hosted_request(&link(DAY)).unwrap();
    let idents: Vec<&str> = hosted
        .trial
        .waypoints
        .iter()
        .map(|w| w.ident.as_str())
        .collect();
    assert_eq!(idents, ["LEZG", "Z1", "WP3"]);
    let over: Vec<bool> = hosted.trial.waypoints.iter().map(|w| w.fly_over).collect();
    assert_eq!(over, [false, true, false]);
    assert_eq!(hosted.trial.waypoints[1].lat_deg, 41.75);
    assert_eq!(hosted.trial.waypoints[2].lon_deg, -0.62);
}

#[test]
fn sets_carry_their_means_and_only_the_outages_of_sensors_they_have() {
    let hosted = hosted_request(&link(DAY)).unwrap();
    let sets = &hosted.trial.sensor_sets;
    let names: Vec<&str> = sets.iter().map(|set| set.name.as_str()).collect();
    assert_eq!(names, ["ins.baro.visual", "ins.baro", "ins.baro.gnss"]);
    assert_eq!(
        sets[0].means,
        [
            NavMeans::DeadReckoning,
            NavMeans::Barometric,
            NavMeans::Visual
        ]
    );
    let scheduled =
        |index: usize| -> Vec<SensorKind> { sets[index].availability.keys().copied().collect() };
    assert_eq!(scheduled(0), [SensorKind::TerrainCamera]);
    assert_eq!(scheduled(1), []);
    assert_eq!(scheduled(2), [SensorKind::Gnss]);
    assert_eq!(hosted.steering_set, "ins.baro.gnss");
}

#[test]
fn outages_become_schedules_with_touching_spans_joined_and_an_open_end() {
    let hosted = hosted_request(&link(DAY)).unwrap();
    let [gnss, camera] = hosted.trial.availability.as_slice() else {
        panic!("two schedules");
    };
    assert_eq!(
        (gnss.name.as_str(), gnss.sensor),
        ("gnss", SensorKind::Gnss)
    );
    let spans: Vec<(f64, Option<f64>)> = gnss
        .unavailable
        .iter()
        .map(|i| (i.start_s, i.end_s))
        .collect();
    assert_eq!(spans, [(300.0, Some(700.0)), (900.0, None)]);
    assert_eq!(camera.sensor, SensorKind::TerrainCamera);
    assert_eq!(
        (camera.unavailable[0].start_s, camera.unavailable[0].end_s),
        (5.0, Some(6.0))
    );
}

#[test]
fn the_camera_is_the_links_or_the_vehicles_own_when_a_set_is_visual() {
    let stated = hosted_request(&link(DAY)).unwrap().camera.unwrap();
    assert_eq!((stated.width_px, stated.height_px), (1600, 1200));
    assert_eq!(
        (
            stated.horizontal_fov_deg,
            stated.depression_deg,
            stated.rate_hz
        ),
        (90.0, 45.5, 2.0)
    );
    let own = hosted_request(&link(&DAY.replace("&cam=1600x1200,90,45.5,2", ""))).unwrap();
    assert_eq!(own.camera, Some(VEHICLE_CAMERA));
    let unused = hosted_request(&link(&format!("{PLAIN}&cam=800x600,67.4,90,1"))).unwrap();
    assert_eq!(unused.camera, None);
}

#[test]
fn speed_is_the_links_or_the_vehicles_own() {
    let own = hosted_request(&link(&PLAIN.replace("veh=c172", "veh=x500"))).unwrap();
    assert_eq!(own.trial.vehicle.cruise_speed_m_s, 10.0);
    let stated = hosted_request(&link(&format!("{PLAIN}&spd=62.5"))).unwrap();
    assert_eq!(stated.trial.vehicle.cruise_speed_m_s, 62.5);
}

#[test]
fn the_link_seed_is_the_trials_master_seed() {
    let seeds = hosted_request(&link(DAY)).unwrap().trial.seeds;
    assert_eq!(seeds, SeedSpec::from_master(1));
    let blind = hosted_request(&link(PLAIN)).unwrap().trial.seeds;
    let master = SeedSpec::from_master(7);
    assert_eq!(blind.terrain_camera, None);
    assert_eq!(
        (blind.imu, blind.star_camera),
        (master.imu, master.star_camera)
    );
}

#[test]
fn the_trial_gets_the_camera_by_its_geometry() {
    let stated = hosted_request(&link(DAY))
        .unwrap()
        .trial
        .terrain_camera
        .unwrap();
    assert_eq!((stated.width, stated.height), (1600, 1200));
    assert_eq!(stated.horizontal_fov_deg, Some(90.0));
    assert_eq!(stated.mount.pitch_deg, -45.5);
    assert_eq!(stated.period_s, 0.5);
    stated.validate().unwrap();
    let own = hosted_request(&link(&DAY.replace("&cam=1600x1200,90,45.5,2", "")))
        .unwrap()
        .trial
        .terrain_camera
        .unwrap();
    assert_eq!((own.width, own.height, own.period_s), (400, 300, 1.0));
    assert_eq!(own.mount.pitch_deg, -90.0);
    own.validate().unwrap();
    assert!(
        hosted_request(&link(PLAIN))
            .unwrap()
            .trial
            .terrain_camera
            .is_none()
    );
}

#[test]
fn a_link_value_without_a_height_is_refused_not_flown_at_zero() {
    let mut edited = link(PLAIN);
    edited.default_height = None;
    let error = hosted_request(&edited).unwrap_err();
    assert!(matches!(error, HostError::MissingHeight { what } if what == "LEZG"));
}

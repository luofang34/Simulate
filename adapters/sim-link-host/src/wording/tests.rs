use sim_link::decode_fragment;

use crate::{PlanLine, hosted_request, outage_words, plan_lines, plan_summary, refusal_lines};

const DAY: &str = "v=2&r=IF,LEZG:41.666242/-1.041553;TF,Z1:41.75/-0.8,o;TF,41.6/-0.62,a2100.5&h=1981.2&hd=msl&t=20260615T120000Z&veh=c172&seed=1\
&nav=ins.baro.visual;ins.baro.stars;ins.baro.gnss&fly=ins.baro.gnss&out=gnss:300-600,900-;cam:5.5-6\
&go=1&mb=200";

fn lines(fragment: &str) -> Vec<PlanLine> {
    let link = decode_fragment(fragment).unwrap();
    let request = hosted_request(&link).unwrap();
    let summary = plan_summary(&link, &request).unwrap();
    plan_lines(&link, &request, &summary)
}

fn texts<'a>(lines: &'a [PlanLine], label: &str) -> Vec<&'a str> {
    lines
        .iter()
        .filter(|line| line.label == label)
        .map(|line| line.text.as_str())
        .collect()
}

#[test]
fn outages_read_as_what_the_sensor_does_not_give_and_when() {
    let link = decode_fragment(DAY).unwrap();
    assert_eq!(
        outage_words(&link.outages[0]),
        [
            "satellite receiver gives no position from 300 s to 600 s",
            "satellite receiver gives no position from 900 s to the end of the flight",
        ]
    );
    assert_eq!(
        outage_words(&link.outages[1]),
        ["terrain camera gives no frames from 5.5 s to 6 s"]
    );
}

#[test]
fn the_plan_says_what_the_link_asks_for() {
    let lines = lines(DAY);
    let route = texts(&lines, "Route");
    assert!(route[0].starts_with("3 legs to 3 fixes, 4"), "{route:?}");
    assert!(route[0].ends_with(" min at 55 m/s (c172)"), "{route:?}");
    assert_eq!(
        texts(&lines, "Leg 2"),
        ["TF Z1  41.750000, -0.800000  1981.2 m above mean sea level, fly-over"]
    );
    assert_eq!(
        texts(&lines, "Leg 3"),
        ["TF WP3  41.600000, -0.620000  2100.5 m above mean sea level, fly-by"]
    );
    assert_eq!(
        texts(&lines, "Starts"),
        ["2026-06-15 12:00:00 UTC, over the first fix, heading for the second"]
    );
    assert_eq!(
        texts(&lines, "Compares"),
        ["ins + baro + visual;  ins + baro + stars;  ins + baro + gnss"]
    );
    assert_eq!(texts(&lines, "Flies on"), ["ins + baro + gnss"]);
    assert_eq!(
        texts(&lines, "Hold"),
        ["the standard right-hand hold at the last fix, flown when the route ends"]
    );
    assert!(texts(&lines, "RNP").is_empty());
    assert_eq!(texts(&lines, "Outage").len(), 3);
    assert_eq!(texts(&lines, "Sensor seed"), ["1"]);
    assert_eq!(
        texts(&lines, "Map package"),
        ["at most 200 MB, built and flown without asking"]
    );
    assert_eq!(
        texts(&lines, "Camera"),
        [
            "400 x 300 pixels, 67.4° across, looking straight down, 1 frame a second (the vehicle's own)"
        ]
    );
    let stated = self::lines(&DAY.replace("&go=1", "&cam=1600x1200,90,45.5,2&go=1"));
    assert_eq!(
        texts(&stated, "Camera"),
        ["1600 x 1200 pixels, 90° across, looking 45.5° below the horizon, 2 frames a second"]
    );
}

#[test]
fn the_sun_line_says_what_daylight_and_night_mean_for_the_sets() {
    let day = lines(DAY);
    let sun = texts(&day, "Sun")[0];
    assert!(sun.contains("daylight for the whole route"), "{sun}");
    assert!(
        sun.contains("The terrain camera needs daylight and has it."),
        "{sun}"
    );
    assert!(
        sun.contains("The star camera needs night and has none."),
        "{sun}"
    );

    let night = lines(&DAY.replace("T120000Z", "T003000Z"));
    let sun = texts(&night, "Sun")[0];
    assert!(sun.contains("night for the whole route"), "{sun}");
    assert!(
        sun.contains("The terrain camera needs daylight and has none."),
        "{sun}"
    );
    assert!(
        sun.contains("The star camera needs night and has it."),
        "{sun}"
    );

    let unaffected = lines(&DAY.replace("ins.baro.visual;ins.baro.stars;", ""));
    let sun = texts(&unaffected, "Sun")[0];
    assert!(sun.ends_with("daylight for the whole route."), "{sun}");
}

#[test]
fn a_start_position_and_a_waiting_package_are_worded() {
    let lines = lines(
        &DAY.replace("&t=", "&start=41.5,-1.2,1500,75&t=")
            .replace("&go=1", ""),
    );
    assert_eq!(
        texts(&lines, "Starts"),
        [
            "2026-06-15 12:00:00 UTC, at 41.500000, -1.200000  1500 m above mean sea level, heading 75° true"
        ]
    );
    assert_eq!(
        texts(&lines, "Map package"),
        ["at most 200 MB, shown as a plan first, built on a click"]
    );
}

#[test]
fn a_refusal_shows_the_parameter_the_reason_and_a_caret_under_the_character() {
    let fragment = DAY.replace("Z1:41.75/-0.8", "Z1:97.75/-0.8");
    let refusal = decode_fragment(&fragment).unwrap_err();
    let lines = refusal_lines(&fragment, &refusal);
    assert_eq!(texts(&lines, "Parameter"), ["r"]);
    assert_eq!(texts(&lines, "Entry"), ["2"]);
    assert_eq!(
        texts(&lines, "Reason"),
        ["latitude 97.75 is outside -90 to 90 degrees"]
    );
    let place = texts(&lines, "Where")[0];
    let [number, excerpt, caret] = place.split('\n').collect::<Vec<_>>()[..] else {
        panic!("three lines: {place}");
    };
    assert_eq!(
        number,
        format!("character {}", fragment.find("97.75").unwrap() + 1)
    );
    let column = caret.find('^').unwrap();
    assert!(excerpt[column..].starts_with("97.75"), "{excerpt}\n{caret}");
}

#[test]
fn a_caret_near_the_start_of_the_link_keeps_its_column() {
    let refusal = decode_fragment("v=3&r=1,2").unwrap_err();
    let lines = refusal_lines("v=3&r=1,2", &refusal);
    assert_eq!(texts(&lines, "Where"), ["character 3\nv=3&r=1,2\n  ^"]);
}

#[test]
fn the_plan_words_a_stated_hold_roles_feet_and_rnp() {
    let fragment = DAY.replace(
        "TF,41.6/-0.62,a2100.5",
        "TF,41.6/-0.62,rFAF,a3000ft,p0.3nm;HM,41.6/-0.62,tL,a1500,c138.2T,l1.5min",
    );
    let lines = lines(&fragment);
    assert_eq!(
        texts(&lines, "Leg 3"),
        ["TF WP3  41.600000, -0.620000  914.4 m above mean sea level, fly-by, FAF"]
    );
    assert_eq!(
        texts(&lines, "Hold"),
        ["at the last fix, left turns, inbound course 138.2° true, 90 s legs, at 1500 m"]
    );
    assert_eq!(texts(&lines, "RNP").len(), 1);
}

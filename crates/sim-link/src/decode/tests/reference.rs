//! Links the reference encoder `fpl_to_simulate_url.py` printed, one a line.

use crate::{
    Height, HeightDatum, HeightUnit, Limit, Means, Milli, PathTerminator, Sensor, TurnDirection,
    decode_fragment, decode_url,
};

const REFERENCE_LINKS: &str = include_str!("../../../fixtures/reference-v2.txt");
/// Links in which the encoder passed the user's digits through, wrote the
/// attributes out of order, repeated a fix in full, or printed a negative
/// zero: valid, but not the written form.
const OTHER_FORMS: &str = include_str!("../../../fixtures/reference-v2-other-forms.txt");

fn links() -> Vec<&'static str> {
    let links: Vec<&str> = REFERENCE_LINKS
        .lines()
        .filter(|line| !line.is_empty())
        .collect();
    assert_eq!(links.len(), 7);
    links
}

#[test]
fn every_reference_link_decodes() {
    for link in links() {
        let decoded = decode_url(link).unwrap_or_else(|refusal| panic!("{link}: {refusal}"));
        let decoded = decoded.unwrap();
        assert!(
            decoded
                .route
                .iter()
                .all(|leg| leg.fix.as_ref().is_some_and(|fix| fix.name.is_some())),
            "{link}"
        );
        assert_eq!(decoded.route[0].path, PathTerminator::If, "{link}");
        assert!(decoded.default_height.is_some(), "{link}");
    }
}

#[test]
fn every_reference_link_is_already_in_the_written_form() {
    for link in links() {
        let (base, fragment) = link.split_once('#').unwrap();
        let decoded = decode_fragment(fragment).unwrap();
        assert_eq!(decoded.fragment().unwrap(), fragment);
        assert_eq!(decoded.url(base).unwrap(), link);
    }
}

#[test]
fn the_reference_links_say_what_the_encoder_was_asked() {
    let decoded: Vec<_> = links()
        .into_iter()
        .map(|link| decode_url(link).unwrap().unwrap())
        .collect();
    let [day, night, plain, foreflight, ..] = decoded.as_slice() else {
        panic!("seven links");
    };
    assert_eq!(day.default_height, Some(Milli::from_scaled(1_981_200)));
    assert_eq!(day.start.unwrap().heading, Some(Milli::from_whole(75)));
    assert!(day.auto_start && day.sets[0].contains(Means::Visual));

    assert_eq!(night.datum, HeightDatum::Ellipsoid);
    assert!(night.route[1].fly_over && !night.route[2].fly_over);
    assert_eq!((night.time.hour, night.time.minute), (0, 30));
    assert!(!night.auto_start && night.start.is_none() && night.outages.is_empty());

    assert!(plain.camera.is_none() && plain.cruise_speed.is_none());

    let feet = |feet: i64| {
        Some(Limit::At(Height {
            value: Milli::from_whole(feet),
            unit: HeightUnit::Foot,
        }))
    };
    assert_eq!(foreflight.route[0].altitude, feet(3000));
    assert_eq!(foreflight.route[1].altitude, None);
    assert!(foreflight.route[2].fly_over);
    assert_eq!(
        foreflight.route[3].fix.as_ref().unwrap().name.as_deref(),
        Some("WP4")
    );
    let hold = foreflight.route.last().unwrap();
    assert_eq!(
        (
            hold.path,
            hold.turn,
            hold.fix.as_ref().unwrap().name.as_deref()
        ),
        (PathTerminator::Hm, Some(TurnDirection::Left), Some("KGED"))
    );
    assert_eq!(
        (
            foreflight.seed,
            foreflight.budget_mb,
            foreflight.steering_set
        ),
        (u64::MAX, 65_536, 1)
    );
}

#[test]
fn the_reference_links_carry_outages_repeated_fixes_and_the_first_flight() {
    let decoded: Vec<_> = links()
        .into_iter()
        .map(|link| decode_url(link).unwrap().unwrap())
        .collect();
    let [.., busy, out_and_back, first_flight] = decoded.as_slice() else {
        panic!("seven links");
    };
    let sensors: Vec<Sensor> = busy.outages.iter().map(|outage| outage.sensor).collect();
    assert_eq!(
        sensors,
        [Sensor::Cam, Sensor::Gnss, Sensor::Baro, Sensor::Stars]
    );
    assert_eq!(busy.start.unwrap().height, Some(Milli::from_whole(1800)));
    assert_eq!(busy.camera.unwrap().rate_hz, Milli::from_scaled(29_970));

    // The same ident at two places stays two fixes; the plan's return to its
    // first fix is written by name.
    let fixes: Vec<_> = out_and_back
        .route
        .iter()
        .map(|leg| leg.fix.as_ref().unwrap())
        .collect();
    assert_eq!(fixes[1].name, fixes[3].name);
    assert_ne!(fixes[1].lat, fixes[3].lat);
    assert_eq!(fixes[0], fixes[5]);

    // The land route of the first closed-loop flight in the browser.
    let names: Vec<_> = first_flight
        .route
        .iter()
        .map(|leg| leg.fix.as_ref().unwrap().name.as_deref().unwrap())
        .collect();
    assert_eq!(names, ["ZA1", "ZA2", "ZA3", "ZA4", "ZA5"]);
    assert_eq!(first_flight.time.compact(), "20260615T120400Z");
    assert_eq!(first_flight.steering().unwrap().text(), "ins.baro.gnss");
    assert!(first_flight.outages.is_empty() && first_flight.auto_start);
    assert_eq!(first_flight.budget_mb, 200);
}

#[test]
fn reference_links_in_other_forms_decode_to_the_value_their_written_form_gives() {
    let links: Vec<&str> = OTHER_FORMS
        .lines()
        .filter(|line| !line.is_empty())
        .collect();
    let [digits, zero] = links.as_slice() else {
        panic!("two links");
    };
    for (link, given, written) in [
        (
            digits,
            "TF,Z1:41.750000/-0.800000,a3000.0ft,o;",
            "TF,Z1:41.75/-0.8,o,a3000ft;",
        ),
        (digits, "TF,LEZG:41.666242/-1.041553&", "TF,LEZG&"),
        (
            digits,
            "out=gnss:300.0-600.0&cam=800x600,90.0,45.00,1.0",
            "out=gnss:300-600&cam=800x600,90,45,1",
        ),
        (zero, "HM,Z1,l01min,c065.20T,tR&", "HM,Z1,tR,c65.2T,l1min&"),
        (zero, "h=-0.0&", "h=0.0&"),
        (
            zero,
            "fly=ins.baro&out=baro:007-8.500",
            "fly=ins.baro&out=baro:7-8.5",
        ),
    ] {
        let decoded = decode_url(link).unwrap().unwrap();
        let fragment = decoded.fragment().unwrap();
        assert!(
            link.contains(given) && fragment.contains(written),
            "{link} -> {fragment}"
        );
        assert_eq!(decode_fragment(&fragment), Ok(decoded));
    }
}

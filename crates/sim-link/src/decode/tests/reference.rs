//! Links the reference encoder `fpl_to_simulate_url.py` printed, one a line.

use crate::{HeightDatum, Means, Milli, Sensor, Turn, Vehicle, decode_fragment, decode_url};

const REFERENCE_LINKS: &str = include_str!("../../../fixtures/reference-v1.txt");
/// Links in which the encoder passed the user's digits through, or printed a
/// negative zero: valid, but not the written form.
const OTHER_FORMS: &str = include_str!("../../../fixtures/reference-v1-other-forms.txt");

fn links() -> Vec<&'static str> {
    let links: Vec<&str> = REFERENCE_LINKS
        .lines()
        .filter(|line| !line.is_empty())
        .collect();
    assert_eq!(links.len(), 6);
    links
}

#[test]
fn every_reference_link_decodes() {
    for link in links() {
        let decoded = decode_url(link).unwrap_or_else(|refusal| panic!("{link}: {refusal}"));
        let decoded = decoded.unwrap();
        assert_eq!(decoded.route.len(), 3, "{link}");
        let names: Vec<_> = decoded
            .route
            .iter()
            .map(|w| w.name.as_deref().unwrap())
            .collect();
        assert_eq!(names, ["LEZG", "Z1", "Z_2_A"]);
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
    let [day, night, plain, quad, busy, south] = decoded.as_slice() else {
        panic!("six links");
    };
    assert_eq!(day.default_height, Some(Milli::from_scaled(1_981_200)));
    assert_eq!(day.start.unwrap().heading, Some(Milli::from_whole(75)));
    assert!(day.auto_start && day.sets[0].contains(Means::Visual));

    assert_eq!(night.datum, HeightDatum::Ellipsoid);
    assert_eq!(night.route[1].turn, Turn::FlyOver);
    assert_eq!((night.time.hour, night.time.minute), (0, 30));
    assert!(!night.auto_start && night.start.is_none() && night.outages.is_empty());

    assert!(plain.camera.is_none() && plain.cruise_speed.is_none());

    assert_eq!(quad.vehicle, Vehicle::X500);
    assert_eq!(quad.cruise_speed, Some(Milli::from_scaled(12_500)));
    assert_eq!(
        (quad.seed, quad.budget_mb, quad.steering_set),
        (u64::MAX, 65_536, 1)
    );

    let sensors: Vec<Sensor> = busy.outages.iter().map(|outage| outage.sensor).collect();
    assert_eq!(
        sensors,
        [Sensor::Cam, Sensor::Gnss, Sensor::Baro, Sensor::Stars]
    );
    assert_eq!(busy.start.unwrap().height, Some(Milli::from_whole(1800)));
    assert_eq!(busy.camera.unwrap().rate_hz, Milli::from_scaled(29_970));

    assert_eq!(south.start.unwrap().lat_deg(), -41.5);
    assert_eq!(south.sets[0].0.len(), 5);
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
            "out=gnss:300.0-600.0&cam=800x600,90.0,45.00,1.0",
            "out=gnss:300-600&cam=800x600,90,45,1",
        ),
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

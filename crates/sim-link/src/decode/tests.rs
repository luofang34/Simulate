mod leg_refusals;
mod legs;
mod parameters;
mod reference;
mod sensors;
mod start;
mod structure;

use crate::{Parameter, Reason, Refusal, decode_fragment};

/// A link in the written form, with every parameter present.
const FULL: &str = "v=2&r=IF,LEZG:41.666242/-1.041553;TF,Z1:41.75/-0.8;TF,Z_2_A:41.6/-0.62\
&h=1981.2&hd=msl&start=41.666250,-1.041550,,75.0&t=20260615T120000Z&veh=c172&spd=55&seed=1\
&nav=ins.baro.visual;ins.baro;ins.baro.gnss&fly=ins.baro.visual&out=gnss:300-600,900-\
&cam=800x600,67.4,90,1&go=1&mb=200";

/// [`FULL`] with the value of `key` replaced.
fn with(key: &str, value: &str) -> String {
    let pairs: Vec<String> = FULL
        .split('&')
        .map(|pair| match pair.split_once('=') {
            Some((found, _)) if found == key => format!("{key}={value}"),
            _ => pair.to_owned(),
        })
        .collect();
    assert!(
        FULL.contains(&format!("&{key}=")) || key == "v",
        "{key} is not in FULL"
    );
    pairs.join("&")
}

/// [`FULL`] without `key`.
fn without(key: &str) -> String {
    let pairs: Vec<&str> = FULL
        .split('&')
        .filter(|pair| pair.split_once('=').is_none_or(|(found, _)| found != key))
        .collect();
    pairs.join("&")
}

fn refusal(fragment: &str) -> Refusal {
    match decode_fragment(fragment) {
        Ok(link) => panic!("{fragment} decoded to {link:?}"),
        Err(refusal) => refusal,
    }
}

/// One expected refusal: the value given to a parameter of [`FULL`], the
/// entry at fault, and what the reason must be.
type Case = (&'static str, Option<usize>, fn(&Reason) -> bool);

/// Checks each case refuses at `parameter`, pointing at text inside the
/// replaced value.
fn assert_refusals(key: &str, parameter: Parameter, cases: &[Case]) {
    for (value, item, expected) in cases {
        let fragment = with(key, value);
        let refusal = refusal(&fragment);
        assert_eq!(refusal.parameter, parameter, "{key}={value}: {refusal}");
        assert_eq!(refusal.item, *item, "{key}={value}: {refusal}");
        assert!(expected(&refusal.reason), "{key}={value}: {refusal:?}");
        let value_start = fragment.find(&format!("&{key}=")).unwrap() + key.len() + 2;
        let inside = value_start..=value_start + value.len();
        assert!(
            inside.contains(&refusal.offset),
            "{key}={value}: offset {}",
            refusal.offset
        );
    }
}

#[test]
fn the_full_link_decodes() {
    let link = decode_fragment(FULL).unwrap();
    assert_eq!(link.route.len(), 3);
    assert_eq!(link.steering_set, 0);
    assert!(link.auto_start);
}

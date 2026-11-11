use super::{FULL, refusal, with, without};
use crate::{MAX_LINK_CHARS, Parameter, Reason, decode_fragment, decode_url, resolve_escapes};

#[test]
fn another_version_is_refused_before_anything_else_is_judged() {
    let refused = refusal(&format!("{}&shape=new", with("v", "2")));
    assert_eq!(refused.parameter, Parameter::V);
    assert_eq!(refused.offset, 2);
    assert_eq!(
        refused.reason,
        Reason::UnsupportedVersion { found: "2".into() }
    );
    assert_eq!(refusal("v=1.0&r=x").parameter, Parameter::V);
    assert_eq!(refusal("v=&r=x").parameter, Parameter::V);
}

#[test]
fn a_link_without_a_version_is_refused() {
    let fragment = without("v");
    let refused = refusal(&fragment);
    assert_eq!(
        (refused.parameter, &refused.reason),
        (Parameter::V, &Reason::Missing)
    );
    assert_eq!(refused.offset, fragment.len());
}

#[test]
fn every_required_parameter_is_required() {
    for (key, parameter) in [
        ("r", Parameter::R),
        ("hd", Parameter::Hd),
        ("t", Parameter::T),
        ("veh", Parameter::Veh),
        ("seed", Parameter::Seed),
        ("nav", Parameter::Nav),
        ("mb", Parameter::Mb),
    ] {
        let refused = refusal(&without(key));
        assert_eq!(
            (refused.parameter, &refused.reason),
            (parameter, &Reason::Missing),
            "{key}"
        );
    }
}

#[test]
fn every_optional_parameter_may_be_left_out() {
    for key in ["start", "spd", "fly", "out", "cam", "go"] {
        let link = decode_fragment(&without(key)).unwrap();
        assert_eq!(link.steering_set, 0, "{key}");
    }
    let bare = decode_fragment(&without("go")).unwrap();
    assert!(!bare.auto_start);
}

#[test]
fn a_link_longer_than_the_limit_is_refused() {
    let padding = "A".repeat(MAX_LINK_CHARS);
    let refused = refusal(&format!("{FULL}{padding}"));
    assert_eq!(refused.parameter, Parameter::Link);
    assert!(matches!(
        refused.reason,
        Reason::TooLong {
            limit: MAX_LINK_CHARS,
            ..
        }
    ));

    let base = format!(
        "https://example.invalid/{}",
        "a".repeat(MAX_LINK_CHARS - FULL.len() - 20)
    );
    let url = format!("{base}#{FULL}");
    assert!(url.len() > MAX_LINK_CHARS && FULL.len() < MAX_LINK_CHARS);
    let refused = decode_url(&url).unwrap_err();
    assert_eq!(
        refused.reason,
        Reason::TooLong {
            length: url.len(),
            limit: MAX_LINK_CHARS
        }
    );
}

#[test]
fn a_url_without_a_fragment_is_no_link() {
    assert_eq!(decode_url("http://localhost:8080/"), Ok(None));
    assert_eq!(decode_url("http://localhost:8080/#"), Ok(None));
    assert_eq!(decode_url("http://localhost:8080/index.html?x=1"), Ok(None));
    let link = decode_url(&format!("http://localhost:8080/#{FULL}")).unwrap();
    assert_eq!(link, Some(decode_fragment(FULL).unwrap()));
}

#[test]
fn a_fragment_that_is_not_a_link_is_refused_not_ignored() {
    let refused = decode_url("http://localhost:8080/#about").unwrap_err();
    assert_eq!(refused.parameter, Parameter::Link);
    assert!(matches!(
        refused.reason,
        Reason::Malformed {
            expected: "key=value",
            ..
        }
    ));
}

#[test]
fn unknown_and_repeated_parameters_are_refused() {
    let fragment = format!("{FULL}&wind=5");
    let refused = refusal(&fragment);
    assert_eq!(
        refused.reason,
        Reason::UnknownParameter {
            name: "wind".into()
        }
    );
    assert_eq!(refused.offset, FULL.len() + 1);

    let fragment = format!("{FULL}&h=5");
    let refused = refusal(&fragment);
    assert_eq!(
        (refused.parameter, &refused.reason),
        (Parameter::H, &Reason::Repeated)
    );
    assert_eq!(refused.offset, FULL.len() + 1);
}

#[test]
fn a_pair_without_an_equals_sign_is_refused() {
    for fragment in [format!("{FULL}&go"), format!("{FULL}&"), format!("&{FULL}")] {
        let refused = refusal(&fragment);
        assert!(matches!(
            refused.reason,
            Reason::Malformed {
                expected: "key=value",
                ..
            }
        ));
    }
}

#[test]
fn percent_escapes_of_ascii_resolve_before_parsing() {
    let escaped = FULL
        .replace(';', "%3B")
        .replace(',', "%2c")
        .replace(':', "%3A");
    assert_ne!(escaped, FULL);
    assert_eq!(decode_fragment(&escaped), decode_fragment(FULL));
}

#[test]
fn a_refusal_points_into_the_resolved_fragment() {
    let escaped = with("mb", "0").replace(';', "%3B");
    let resolved = with("mb", "0");
    let refused = refusal(&escaped);
    assert_eq!(refused.parameter, Parameter::Mb);
    assert_eq!(refused.offset, resolved.len() - 1);
    assert_eq!(resolve_escapes(&escaped).unwrap(), resolved);
}

#[test]
fn broken_escapes_and_characters_outside_ascii_are_refused() {
    for (fragment, text) in [
        (format!("{FULL}%zz"), "%zz"),
        (format!("{FULL}%4"), "%4"),
        (format!("{FULL}%"), "%"),
        (format!("{FULL}%e4"), "%e4"),
        (format!("{FULL}é"), "é"),
    ] {
        let refused = refusal(&fragment);
        assert_eq!(
            refused.reason,
            Reason::BadEscape { text: text.into() },
            "{fragment}"
        );
        assert_eq!(refused.offset, FULL.len());
    }
}

#[test]
fn a_refusal_reads_as_parameter_place_and_reason() {
    let refused = refusal(&with("r", "41.5,2.0;95.5,2.0"));
    assert_eq!(
        refused.to_string(),
        "r, entry 2, character 16: latitude 95.5 is outside -90 to 90 degrees"
    );
    let refused = refusal(&without("mb"));
    assert!(refused.to_string().starts_with("mb, character "));
    assert!(refused.to_string().ends_with(": the parameter is missing"));
}

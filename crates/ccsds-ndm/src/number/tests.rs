use super::*;

#[test]
fn accepts_ccsds_real_forms() {
    for (text, value) in [
        ("6655.9942", 6655.9942),
        ("-0.00101495", -0.00101495),
        (".00073794", 0.00073794),
        ("5.", 5.0),
        ("+1.5e3", 1500.0),
        ("-.63E-6", -0.63e-6),
        ("-063.042", -63.042),
        ("0925", 925.0),
    ] {
        assert_eq!(parse_f64(text, ParseMode::Strict), Some(value), "{text}");
    }
}

#[test]
fn rejects_what_the_standard_does_not_allow() {
    for text in [
        "", ".", "-", "inf", "NaN", "infinity", "1e", "1e+", "0x10", "1.2.3", "1e999", " 1", "1 ",
        "1_000", "e5",
    ] {
        assert_eq!(parse_f64(text, ParseMode::Strict), None, "{text:?}");
        if !text.contains(['d', 'D']) {
            assert_eq!(parse_f64(text, ParseMode::Lenient), None, "{text:?}");
        }
    }
}

#[test]
fn fortran_exponent_only_in_lenient_mode() {
    assert_eq!(parse_f64("1.5D-03", ParseMode::Strict), None);
    assert_eq!(parse_f64("1.5D-03", ParseMode::Lenient), Some(1.5e-3));
}

#[test]
fn integers_need_plain_digits() {
    assert_eq!(parse_int::<u32>("0925"), Some(925));
    assert_eq!(parse_int::<i32>("-4"), Some(-4));
    assert_eq!(parse_int::<u32>("-4"), None);
    assert_eq!(parse_int::<u32>("4.0"), None);
    assert_eq!(parse_int::<u32>("99999999999"), None);
    assert_eq!(parse_int::<u32>(""), None);
}

#[test]
fn shortest_form_round_trips_bit_exactly() {
    let samples = [
        0.0,
        -0.0,
        1.0,
        6655.9942,
        3.331349476038534e-04,
        6.2244443e-10,
        1e300,
        f64::MIN_POSITIVE,
        f64::MAX,
        -f64::MAX,
        5e-324,
        0.1 + 0.2,
        123456789012345.6,
    ];
    for value in samples {
        let text = format_f64(value).unwrap();
        let back = parse_f64(&text, ParseMode::Strict).unwrap();
        assert_eq!(back.to_bits(), value.to_bits(), "{value:e} wrote {text}");
    }
    assert_eq!(format_f64(6.2244443e-10).unwrap(), "6.2244443e-10");
    assert_eq!(format_f64(6655.9942).unwrap(), "6655.9942");
    assert_eq!(format_f64(f64::NAN), None);
    assert_eq!(format_f64(f64::INFINITY), None);
}

#[test]
fn pseudo_random_bit_patterns_round_trip() {
    let mut state = 0x9E37_79B9_7F4A_7C15_u64;
    for _ in 0..20_000 {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        let value = f64::from_bits(state);
        if let Some(text) = format_f64(value) {
            let back = parse_f64(&text, ParseMode::Strict).unwrap();
            assert_eq!(back.to_bits(), value.to_bits(), "{text}");
        }
    }
}

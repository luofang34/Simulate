use super::{Micro, Milli, NumberFault};

#[test]
fn text_parses_to_the_scaled_integer() {
    assert_eq!(
        Micro::parse("41.666242", true),
        Ok(Micro::from_scaled(41_666_242))
    );
    assert_eq!(
        Micro::parse("-1.041553", true),
        Ok(Micro::from_scaled(-1_041_553))
    );
    assert_eq!(Micro::parse("-0.5", true), Ok(Micro::from_scaled(-500_000)));
    assert_eq!(
        Milli::parse("1981.2", true),
        Ok(Milli::from_scaled(1_981_200))
    );
    assert_eq!(Milli::parse("007", false), Ok(Milli::from_whole(7)));
    assert_eq!(Milli::parse("0.001", false), Ok(Milli::from_scaled(1)));
}

#[test]
fn forms_a_float_parser_would_take_are_refused() {
    for text in [
        "", "-", ".", "1.", ".5", "+1", "1e3", "nan", "inf", "0x10", "1,5", " 1", "1 ", "--1",
        "1.2.3", "１",
    ] {
        assert_eq!(
            Micro::parse(text, true),
            Err(NumberFault::NotDecimal),
            "{text:?}"
        );
    }
    assert_eq!(Milli::parse("-1", false), Err(NumberFault::NotDecimal));
}

#[test]
fn extra_digits_are_refused_not_rounded() {
    assert_eq!(
        Milli::parse("1.0001", true),
        Err(NumberFault::TooManyDecimals)
    );
    assert_eq!(
        Micro::parse("1.0000001", true),
        Err(NumberFault::TooManyDecimals)
    );
    assert_eq!(
        Micro::parse("1234567890123", true),
        Err(NumberFault::TooManyDigits)
    );
    assert_eq!(
        Micro::parse("999999999999.999999", true),
        Ok(Micro::from_scaled(999_999_999_999_999_999))
    );
}

#[test]
fn text_keeps_the_least_decimals_asked_and_no_spare_zeros() {
    assert_eq!(Micro::from_scaled(41_750_000).text(6), "41.750000");
    assert_eq!(Micro::from_scaled(-620_000).text(6), "-0.620000");
    assert_eq!(Milli::from_scaled(1_981_200).text(1), "1981.2");
    assert_eq!(Milli::from_whole(2_000).text(1), "2000.0");
    assert_eq!(Milli::from_whole(90).text(0), "90");
    assert_eq!(Milli::from_scaled(67_400).text(0), "67.4");
    assert_eq!(Milli::from_scaled(-1).text(0), "-0.001");
    assert_eq!(Milli::from_scaled(1_250).text(1), "1.25");
}

#[test]
fn the_float_is_the_one_the_text_parses_to() {
    for text in [
        "41.666242",
        "-1.041553",
        "0.000001",
        "179.999999",
        "-89.123457",
    ] {
        let fixed = Micro::parse(text, true).unwrap();
        assert_eq!(fixed.as_f64(), text.parse::<f64>().unwrap(), "{text}");
        assert_eq!(fixed.text(6), text);
    }
}

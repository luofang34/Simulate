use super::*;

#[test]
fn classifies_lines_with_positions() {
    let text =
        "CCSDS_OPM_VERS = 3.0\r\n\n  COMMENT  hello = world \nX = 1.5 [km]\nMETA_START\nCOMMENT\n";
    let lines = lex(text, ParseMode::Strict);
    assert_eq!(lines.len(), 5);
    let Line::Pair(version) = &lines[0] else {
        panic!("{:?}", lines[0]);
    };
    assert_eq!(version.keyword, "CCSDS_OPM_VERS");
    assert_eq!(version.value, "3.0");
    assert_eq!(version.at, Some(Location::new(1, 18)));
    let Line::Comment { text, at } = &lines[1] else {
        panic!("{:?}", lines[1]);
    };
    assert_eq!(text, "hello = world");
    assert_eq!(*at, Location::new(3, 3));
    let Line::Pair(x) = &lines[2] else {
        panic!("{:?}", lines[2]);
    };
    assert_eq!(x.value, "1.5 [km]");
    assert!(matches!(&lines[3], Line::Bare { text, .. } if text == "META_START"));
    assert!(matches!(&lines[4], Line::Comment { text, .. } if text.is_empty()));
}

#[test]
fn lower_case_keywords_only_in_lenient_mode() {
    let strict = lex("object_name = X", ParseMode::Strict);
    assert!(matches!(&strict[0], Line::Bare { .. }));
    let lenient = lex("object_name = X", ParseMode::Lenient);
    assert_eq!(lenient[0].keyword(), Some("OBJECT_NAME"));
}

#[test]
fn columns_count_characters() {
    let lines = lex("OBJECT_NAME = é = x", ParseMode::Strict);
    let Line::Pair(entry) = &lines[0] else {
        panic!();
    };
    assert_eq!(entry.value, "é = x");
    assert_eq!(entry.at, Some(Location::new(1, 15)));
}

#[test]
fn splits_concatenated_messages() {
    let text = "A = 1\nV = 1\nB = 2\nV = 2\nC = 3\n";
    let groups = split_messages(lex(text, ParseMode::Strict), "V");
    let keys: Vec<Vec<&str>> = groups
        .iter()
        .map(|g| g.iter().filter_map(Line::keyword).collect())
        .collect();
    assert_eq!(keys, vec![vec!["A"], vec!["V", "B"], vec!["V", "C"]]);
}

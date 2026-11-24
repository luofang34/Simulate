use super::*;
use crate::kvn::lexer::lex;

const SECTIONS: &[Section] = &[
    Section {
        element: "a",
        keywords: &["A"],
        prefix: None,
        repeats_at: None,
    },
    Section {
        element: "m",
        keywords: &["M1", "M2"],
        prefix: None,
        repeats_at: Some("M1"),
    },
    Section {
        element: "u",
        keywords: &[],
        prefix: Some("U_"),
        repeats_at: None,
    },
];

const SCHEMA: FlatSchema = FlatSchema {
    name: "TEST",
    root: "test",
    version_key: "V",
    sections: SECTIONS,
};

fn keywords(block: &Block) -> Vec<&str> {
    block.entries.iter().map(|e| e.keyword.as_str()).collect()
}

#[test]
fn comments_attach_to_the_block_that_follows() {
    let text = "V = 1\nCOMMENT a\nA = 1\nCOMMENT m\nM1 = 1\nM2 = 2\nCOMMENT n\nM1 = 3\nU_X = 4\nCOMMENT tail\n";
    let mut message = read(lex(text, ParseMode::Strict), &SCHEMA, ParseMode::Strict).unwrap();
    assert_eq!(message.version.as_ref().unwrap().value, "1");
    let first = message.take(0).unwrap();
    assert_eq!(first.comments, ["a"]);
    let maneuvers = message.take_all(1);
    assert_eq!(maneuvers.len(), 2);
    assert_eq!(maneuvers[0].comments, ["m"]);
    assert_eq!(keywords(&maneuvers[0]), ["M1", "M2"]);
    assert_eq!(maneuvers[1].comments, ["n"]);
    let user = message.take(2).unwrap();
    assert_eq!(user.comments, ["tail"]);
}

#[test]
fn repeated_keyword_opens_a_new_repeating_block() {
    let text = "V = 1\nM2 = 1\nM2 = 2\n";
    let mut message = read(lex(text, ParseMode::Strict), &SCHEMA, ParseMode::Strict).unwrap();
    assert_eq!(message.take_all(1).len(), 2);
}

#[test]
fn out_of_order_is_strict_error_and_lenient_merge() {
    let text = "V = 1\nA = 1\nM1 = 1\nA = 2\n";
    let error = read(lex(text, ParseMode::Strict), &SCHEMA, ParseMode::Strict)
        .err()
        .unwrap();
    assert!(matches!(error.kind(), ErrorKind::KeywordOutOfOrder));
    assert_eq!(error.keyword(), Some("A"));
    assert_eq!(error.location().unwrap().line, 4);
    let mut lenient = read(lex(text, ParseMode::Lenient), &SCHEMA, ParseMode::Lenient).unwrap();
    assert_eq!(keywords(&lenient.take(0).unwrap()), ["A", "A"]);
}

#[test]
fn rejects_data_lines_and_foreign_versions() {
    let bare = read(
        lex("V = 1\n1 2 3\n", ParseMode::Strict),
        &SCHEMA,
        ParseMode::Strict,
    );
    assert!(matches!(
        bare.err().unwrap().kind(),
        ErrorKind::UnexpectedLine { .. }
    ));
    let foreign = read(
        lex("W = 1\n", ParseMode::Strict),
        &SCHEMA,
        ParseMode::Strict,
    );
    assert!(matches!(
        foreign.err().unwrap().kind(),
        ErrorKind::WrongMessage { .. }
    ));
    let empty = read(Vec::new(), &SCHEMA, ParseMode::Strict);
    assert!(matches!(empty.err().unwrap().kind(), ErrorKind::Empty));
}

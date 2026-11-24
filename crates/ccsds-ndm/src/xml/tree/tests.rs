use super::*;

#[test]
fn builds_a_tree_with_positions_and_entities() {
    let text = "<?xml version=\"1.0\"?>\n<a x=\"1 &amp; 2\">\n  <b units=\"km\"> 5 </b>\n  <c>A &lt; B &#65;</c><d/>\n</a>\n<!-- trailing -->\n";
    let root = parse(text).unwrap();
    assert_eq!(root.name, "a");
    assert_eq!(root.attribute("x"), Some("1 & 2"));
    assert_eq!(root.at, Location::new(2, 1));
    let b = &root.children[0];
    assert_eq!(b.at, Location::new(3, 3));
    let entry = b.entry("B".to_owned());
    assert_eq!(entry.value, "5");
    assert_eq!(entry.unit.as_deref(), Some("km"));
    assert_eq!(root.children[1].text, "A < B A");
    assert_eq!(root.children[2].name, "d");
}

#[test]
fn strips_namespace_prefixes() {
    let root = parse("<ndm:opm xmlns:ndm=\"urn:x\" xsi:id=\"v\"><ndm:b/></ndm:opm>").unwrap();
    assert_eq!(root.name, "opm");
    assert_eq!(root.attribute("id"), Some("v"));
    assert_eq!(root.children[0].name, "b");
}

#[test]
fn rejects_malformed_documents() {
    for text in [
        "",
        "<a>",
        "<a></b>",
        "<a>&bogus;</a>",
        "<a/><b/>",
        "text<a/>",
        "<a x=1/>",
    ] {
        assert!(parse(text).is_err(), "{text:?}");
    }
}

#[test]
fn limits_nesting_depth() {
    let deep = "<a>".repeat(40) + &"</a>".repeat(40);
    let error = parse(&deep).err().unwrap();
    assert!(matches!(error.kind(), ErrorKind::XmlTooDeep { .. }));
}

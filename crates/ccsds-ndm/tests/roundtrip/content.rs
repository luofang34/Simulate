//! Notation-level content of a message, independent of the crate's own
//! parser: every keyword with its value normalised (numbers compared as
//! `f64`, units and spacing dropped), and every comment. Two texts with equal
//! content say the same thing.

use quick_xml::Reader;
use quick_xml::events::Event;

/// Sorted `(keyword, value)` pairs and sorted comments.
#[derive(Debug, PartialEq)]
pub struct Content {
    pub pairs: Vec<(String, String)>,
    pub comments: Vec<String>,
}

fn normalise(value: &str) -> String {
    let value = value.trim();
    let value = match value.strip_suffix(']').and_then(|v| v.rsplit_once('[')) {
        Some((number, _unit)) if number.trim().parse::<f64>().is_ok() => number.trim(),
        _ => value,
    };
    match value.parse::<f64>() {
        Ok(number) if !value.is_empty() => format!("{:e}", number),
        _ => value.split_whitespace().collect::<Vec<_>>().join(" "),
    }
}

/// KVN content. Data lines contribute their numbers under `DATA`.
pub fn kvn(text: &str) -> Content {
    let mut pairs = Vec::new();
    let mut comments = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix("COMMENT") {
            comments.push(normalise(comment));
        } else if let Some((key, value)) = line.split_once('=') {
            // A blank value says nothing; writers omit it.
            if !value.trim().is_empty() {
                pairs.push((key.trim().to_owned(), normalise(value)));
            }
        } else {
            for token in line.split_whitespace() {
                pairs.push(("DATA".to_owned(), normalise(token)));
            }
        }
    }
    pairs.sort();
    comments.sort();
    Content { pairs, comments }
}

/// XML content: leaf elements (with a `parameter` attribute appended to the
/// name), message `version` attributes, and comments.
pub fn xml(text: &str) -> Content {
    let mut reader = Reader::from_str(text);
    let mut pairs = Vec::new();
    let mut comments = Vec::new();
    let mut open: Option<(String, String)> = None;
    loop {
        match reader.read_event().unwrap() {
            Event::Start(tag) => {
                let mut name = String::from_utf8_lossy(tag.local_name().as_ref()).into_owned();
                for attribute in tag.attributes().flatten() {
                    let key = attribute.key.local_name();
                    let value = String::from_utf8_lossy(&attribute.value).into_owned();
                    if key.as_ref() == b"parameter" {
                        name = format!("USER_DEFINED_{value}");
                    } else if key.as_ref() == b"version" {
                        pairs.push((format!("{name}@version"), value));
                    }
                }
                open = Some((name, String::new()));
            }
            Event::Text(text) => {
                if let Some((_, buffer)) = open.as_mut() {
                    buffer.push_str(&text.decode().unwrap());
                }
            }
            Event::GeneralRef(reference) => {
                if let Some((_, buffer)) = open.as_mut() {
                    let name = reference.decode().unwrap();
                    buffer.push_str(match name.as_ref() {
                        "amp" => "&",
                        "lt" => "<",
                        "gt" => ">",
                        "quot" => "\"",
                        _ => "'",
                    });
                }
            }
            Event::End(_) => {
                if let Some((name, value)) = open.take() {
                    if name == "COMMENT" {
                        comments.push(normalise(&value));
                    } else if !value.trim().is_empty() {
                        pairs.push((name, normalise(&value)));
                    }
                }
            }
            Event::Eof => break,
            _ => {}
        }
    }
    pairs.sort();
    comments.sort();
    Content { pairs, comments }
}

//! Reads an XML document into a tree of elements with positions.
//!
//! NDM documents are small and shallow, so a tree is simpler than streaming
//! and lets every element report where it starts.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};

use crate::block::{Block, Entry};
use crate::error::{Error, ErrorKind, Location, Result};

/// Deeper nesting than any NDM schema uses (an NDM container holding an OEM
/// reaches six levels).
const MAX_DEPTH: usize = 32;

/// One element: its local name, attributes, children and text.
#[derive(Debug, Clone)]
pub(crate) struct Element {
    /// The local name (any namespace prefix removed).
    pub name: String,
    /// Attributes by local name, in document order.
    pub attributes: Vec<(String, String)>,
    /// Child elements.
    pub children: Vec<Element>,
    /// Concatenated character data, untrimmed.
    pub text: String,
    /// Where the start tag begins.
    pub at: Location,
}

impl Element {
    /// The value of an attribute.
    pub fn attribute(&self, name: &str) -> Option<&str> {
        self.attributes
            .iter()
            .find(|(key, _)| key == name)
            .map(|(_, value)| value.as_str())
    }

    /// An error located at this element.
    pub fn error(&self, kind: ErrorKind) -> Error {
        Error::new(kind).at(self.at)
    }

    /// The error for a child element this position does not allow.
    pub fn unexpected(&self) -> Error {
        self.error(ErrorKind::UnexpectedElement {
            name: self.name.clone(),
        })
    }

    /// The trimmed text as a block entry, with any `units` attribute.
    pub fn entry(&self, keyword: String) -> Entry {
        let mut entry = Entry::text(keyword, self.text.trim(), self.at);
        entry.unit = self.attribute("units").map(str::to_owned);
        entry
    }

    /// The single child with this name; `None` if absent, an error if repeated.
    pub fn child(&self, name: &'static str) -> Result<Option<&Element>> {
        let mut found = self.children.iter().filter(|c| c.name == name);
        let first = found.next();
        if let Some(second) = found.next() {
            return Err(second.unexpected());
        }
        Ok(first)
    }

    /// The single child with this name, which must be present.
    pub fn required_child(&self, name: &'static str) -> Result<&Element> {
        self.child(name)?
            .ok_or_else(|| self.error(ErrorKind::MissingElement { name }))
    }

    /// A block of `COMMENT` and leaf elements. A child with children of its
    /// own is an error.
    pub fn block(&self) -> Result<Block> {
        let mut block = Block::starting_at(Some(self.at));
        for child in &self.children {
            if !child.children.is_empty() {
                return Err(child.unexpected());
            }
            if child.name == "COMMENT" {
                block.comments.push(child.text.trim().to_owned());
            } else {
                block.entries.push(child.entry(child.name.clone()));
            }
        }
        Ok(block)
    }
}

/// Parses `text` and returns its root element.
pub(crate) fn parse(text: &str) -> Result<Element> {
    let mut reader = Reader::from_str(text);
    let mut stack: Vec<Element> = Vec::new();
    loop {
        let start = usize::try_from(reader.buffer_position()).unwrap_or(usize::MAX);
        let at = Location::of_offset(text, start);
        let event = reader.read_event().map_err(|source| {
            let offset = usize::try_from(reader.error_position()).unwrap_or(usize::MAX);
            Error::new(ErrorKind::Xml(source)).at(Location::of_offset(text, offset))
        })?;
        match event {
            Event::Start(tag) => {
                if stack.len() >= MAX_DEPTH {
                    return Err(Error::new(ErrorKind::XmlTooDeep { limit: MAX_DEPTH }).at(at));
                }
                stack.push(open(&tag, at)?);
            }
            Event::Empty(tag) => {
                let element = open(&tag, at)?;
                if let Some(root) = attach(&mut stack, element) {
                    return finish(&mut reader, text, root);
                }
            }
            Event::End(_) => {
                let element = stack.pop().ok_or_else(|| unexpected_end(at))?;
                if let Some(root) = attach(&mut stack, element) {
                    return finish(&mut reader, text, root);
                }
            }
            Event::Text(content) => {
                let decoded = content.decode().map_err(|e| xml_error(e.into(), at))?;
                append_text(&mut stack, &decoded, at)?;
            }
            Event::CData(content) => {
                let decoded = content.decode().map_err(|e| xml_error(e.into(), at))?;
                append_text(&mut stack, &decoded, at)?;
            }
            Event::GeneralRef(reference) => {
                let name = reference.decode().map_err(|e| xml_error(e.into(), at))?;
                let resolved =
                    resolve_reference(&name, reference.resolve_char_ref()).ok_or_else(|| {
                        Error::new(ErrorKind::UnknownEntity {
                            name: name.to_string(),
                        })
                        .at(at)
                    })?;
                append_text(&mut stack, &resolved.to_string(), at)?;
            }
            Event::Eof => {
                return Err(Error::new(ErrorKind::UnexpectedEnd {
                    expected: "a closing tag",
                })
                .at(at));
            }
            Event::Comment(_) | Event::Decl(_) | Event::PI(_) | Event::DocType(_) => {}
        }
    }
}

fn open(tag: &BytesStart<'_>, at: Location) -> Result<Element> {
    let name = String::from_utf8_lossy(tag.local_name().as_ref()).into_owned();
    let mut attributes = Vec::new();
    for attribute in tag.attributes() {
        let attribute = attribute.map_err(|e| xml_error(e.into(), at))?;
        let key = String::from_utf8_lossy(attribute.key.local_name().as_ref()).into_owned();
        let value = attribute
            .normalized_value(quick_xml::XmlVersion::Implicit1_0)
            .map_err(|e| xml_error(e, at))?
            .into_owned();
        attributes.push((key, value));
    }
    Ok(Element {
        name,
        attributes,
        children: Vec::new(),
        text: String::new(),
        at,
    })
}

/// Adds a finished element to its parent, or returns it if it is the root.
fn attach(stack: &mut [Element], element: Element) -> Option<Element> {
    match stack.last_mut() {
        Some(parent) => {
            parent.children.push(element);
            None
        }
        None => Some(element),
    }
}

fn append_text(stack: &mut [Element], text: &str, at: Location) -> Result<()> {
    match stack.last_mut() {
        Some(element) => {
            element.text.push_str(text);
            Ok(())
        }
        // Whitespace around the root is allowed; anything else is not.
        None if text.trim().is_empty() => Ok(()),
        None => Err(Error::new(ErrorKind::UnexpectedEnd {
            expected: "a root element around text",
        })
        .at(at)),
    }
}

/// After the root closes only whitespace, comments and instructions may follow.
fn finish(reader: &mut Reader<&[u8]>, text: &str, root: Element) -> Result<Element> {
    loop {
        let start = usize::try_from(reader.buffer_position()).unwrap_or(usize::MAX);
        let at = Location::of_offset(text, start);
        match reader.read_event().map_err(|e| xml_error(e, at))? {
            Event::Eof => return Ok(root),
            Event::Text(content) if content.decode().is_ok_and(|t| t.trim().is_empty()) => {}
            Event::Comment(_) | Event::PI(_) => {}
            _ => {
                return Err(Error::new(ErrorKind::UnexpectedLine {
                    text: "content after the root element".to_owned(),
                })
                .at(at));
            }
        }
    }
}

fn resolve_reference(
    name: &str,
    char_ref: std::result::Result<Option<char>, quick_xml::Error>,
) -> Option<char> {
    match name {
        "amp" => Some('&'),
        "lt" => Some('<'),
        "gt" => Some('>'),
        "quot" => Some('"'),
        "apos" => Some('\''),
        _ => char_ref.ok().flatten(),
    }
}

fn unexpected_end(at: Location) -> Error {
    Error::new(ErrorKind::UnexpectedEnd {
        expected: "a matching start tag",
    })
    .at(at)
}

fn xml_error(source: quick_xml::Error, at: Location) -> Error {
    Error::new(ErrorKind::Xml(source)).at(at)
}

#[cfg(test)]
mod tests;

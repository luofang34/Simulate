//! XML notation: a small element tree read with `quick-xml`, a writer, and
//! the mapping between elements and blocks.

pub(crate) mod flat;
pub(crate) mod tree;
pub(crate) mod writer;

use crate::header::Version;

/// The XML Schema instance namespace NDM roots declare.
pub(crate) const XSI_NAMESPACE: &str = "http://www.w3.org/2001/XMLSchema-instance";

/// Namespace and schema attributes for a message root. A message inside an
/// `<ndm>` container inherits them, so it gets none.
pub(crate) fn root_attributes(
    version: Version,
    standalone: bool,
) -> Vec<(&'static str, &'static str)> {
    if !standalone {
        return Vec::new();
    }
    let mut attributes = vec![("xmlns:xsi", XSI_NAMESPACE)];
    if let Some(location) = schema_location(version) {
        attributes.push(("xsi:noNamespaceSchemaLocation", location));
    }
    attributes
}

/// The SANA schema location for a message version, where one is published.
pub(crate) fn schema_location(version: Version) -> Option<&'static str> {
    match version {
        Version::V1 => None,
        Version::V2 => {
            Some("https://sanaregistry.org/r/ndmxml_unqualified/ndmxml-2.0.0-master-2.0.xsd")
        }
        Version::V3 => {
            Some("https://sanaregistry.org/r/ndmxml_unqualified/ndmxml-3.0.0-master-3.0.xsd")
        }
    }
}

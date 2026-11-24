//! The NDM combined instantiation: several messages in one `<ndm>` XML
//! document (ODM 502.0-B-3 annex G10).

use crate::error::Result;
use crate::header::Version;
use crate::xml::{self, tree, writer::XmlWriter};
use crate::{Oem, Omm, Opm, ParseMode, WriteOptions};

/// An `<ndm>` document.
#[derive(Debug, Clone, PartialEq)]
pub struct Ndm {
    /// Comments before the first message.
    pub comments: Vec<String>,
    /// The messages, in document order.
    pub messages: Vec<NdmMessage>,
}

/// One message of an [`Ndm`].
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
#[allow(clippy::large_enum_variant)]
pub enum NdmMessage {
    /// An Orbit Parameter Message.
    Opm(Opm),
    /// An Orbit Mean-Elements Message.
    Omm(Omm),
    /// An Orbit Ephemeris Message.
    Oem(Oem),
    /// An Attitude Parameter Message.
    Apm(crate::Apm),
    /// An Attitude Ephemeris Message.
    Aem(crate::Aem),
    /// A Conjunction Data Message.
    Cdm(crate::Cdm),
    /// A Tracking Data Message.
    Tdm(crate::Tdm),
}

impl NdmMessage {
    fn version(&self) -> Option<Version> {
        match self {
            Self::Opm(opm) => Some(opm.version),
            Self::Omm(omm) => omm.version,
            Self::Oem(oem) => Some(oem.version),
            Self::Apm(apm) => Some(apm.version),
            Self::Aem(aem) => Some(aem.version),
            Self::Cdm(cdm) => Some(cdm.version),
            Self::Tdm(tdm) => Some(tdm.version),
        }
    }
}

impl Ndm {
    /// Reads an `<ndm>` document. Message types this crate does not read
    /// (`<ocm>`, `<tdm>`, …) are an error naming the element.
    pub fn from_xml(text: &str, mode: ParseMode) -> Result<Self> {
        let root = tree::parse(text)?;
        if root.name != "ndm" {
            return Err(root.error(crate::ErrorKind::WrongMessage {
                expected: "NDM",
                found: root.name.clone(),
            }));
        }
        let mut comments = Vec::new();
        let mut messages = Vec::new();
        for child in &root.children {
            let index = messages.len();
            let message = match child.name.as_str() {
                "COMMENT" => {
                    comments.push(child.text.trim().to_owned());
                    continue;
                }
                "opm" => Opm::from_element(child, mode).map(NdmMessage::Opm),
                "omm" => Omm::from_element(child, mode).map(NdmMessage::Omm),
                "oem" => Oem::from_element(child, mode).map(NdmMessage::Oem),
                "apm" => crate::Apm::from_element(child, mode).map(NdmMessage::Apm),
                "aem" => crate::Aem::from_element(child, mode).map(NdmMessage::Aem),
                "cdm" => crate::Cdm::from_element(child, mode).map(NdmMessage::Cdm),
                "tdm" => crate::Tdm::from_element(child, mode).map(NdmMessage::Tdm),
                _ => Err(child.unexpected()),
            };
            messages.push(message.map_err(|e| e.in_message(index))?);
        }
        Ok(Self { comments, messages })
    }

    /// Writes the document. Every message needs its version.
    pub fn to_xml(&self, options: WriteOptions) -> Result<String> {
        let mut out = XmlWriter::document();
        let newest = self
            .messages
            .iter()
            .filter_map(NdmMessage::version)
            .max()
            .unwrap_or(Version::V3);
        out.open("ndm", &xml::root_attributes(newest, true));
        out.comments(&self.comments);
        for (index, message) in self.messages.iter().enumerate() {
            match message {
                NdmMessage::Opm(opm) => opm.write_element(&mut out, options, false),
                NdmMessage::Omm(omm) => omm.write_element(&mut out, options, false),
                NdmMessage::Oem(oem) => oem.write_element(&mut out, options, false),
                NdmMessage::Apm(apm) => apm.write_element(&mut out, options, false),
                NdmMessage::Aem(aem) => aem.write_element(&mut out, options, false),
                NdmMessage::Cdm(cdm) => cdm.write_element(&mut out, options, false),
                NdmMessage::Tdm(tdm) => tdm.write_element(&mut out, options, false),
            }
            .map_err(|e| e.in_message(index))?;
        }
        out.close("ndm");
        Ok(out.finish())
    }
}

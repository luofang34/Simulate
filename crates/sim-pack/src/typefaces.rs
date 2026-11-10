//! Shared typeface packs and how an instrument pack's pins on them are
//! resolved.
//!
//! A typeface is its own system package (`kind: TypefacePack`) holding the
//! face's bytes and its provenance. An instrument pack pins each face its
//! dials are lettered in by id, version and the SHA-256 of its
//! `syspack.json`; vehicles never pin a face. Faces ship inside the binary,
//! and a pin that names no shipped face, or another revision of one,
//! refuses the instrument pack rather than lettering it in another face.

use syspack::{InstrumentPackDocument, MemoryPackageSource, VerifiedSyspack};

use crate::PackError;

#[cfg(test)]
mod tests;

const US_MS33558_MANIFEST: &[u8] = include_bytes!("../../../typefaces/us-ms33558/syspack.json");
const US_MS33558_FACE: &[u8] = include_bytes!("../../../typefaces/us-ms33558/us-ms33558.idtf");
const US_MS33558_PROVENANCE: &[u8] = include_bytes!("../../../typefaces/us-ms33558/PROVENANCE.md");

/// Ids of the typeface packs shipped in the binary.
pub const BUILTIN_TYPEFACES: [&str; 1] = ["us-ms33558"];

/// The shipped typeface pack with `id`, as package files.
#[must_use]
pub fn builtin_typeface_source(id: &str) -> Option<MemoryPackageSource> {
    match id {
        "us-ms33558" => Some(MemoryPackageSource::from_embedded(
            US_MS33558_MANIFEST,
            &[
                ("us-ms33558.idtf", US_MS33558_FACE),
                ("PROVENANCE.md", US_MS33558_PROVENANCE),
            ],
        )),
        _ => None,
    }
}

/// Every typeface `document` pins, each loaded from the shipped packs and
/// verified against its pin, in pin order.
pub fn resolve_typefaces(
    document: &InstrumentPackDocument,
) -> Result<Vec<VerifiedSyspack>, PackError> {
    document
        .typefaces
        .iter()
        .map(|pin| {
            let source = builtin_typeface_source(&pin.id).ok_or_else(|| PackError::Typeface {
                typeface: pin.id.clone(),
                detail: "no shipped typeface pack has this id".to_owned(),
            })?;
            Ok(VerifiedSyspack::load_typeface(&source, pin)?)
        })
        .collect()
}

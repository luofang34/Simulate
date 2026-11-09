//! Reusable part packages (engines, propellers) and how a vehicle's pins on
//! them resolve.
//!
//! A part is its own system package, so two aircraft can fly the same engine
//! and a vehicle's variants can differ only in the parts they pin. Each pin
//! names a role (`engine0`, `propeller1`); a part's JSBSim documents are served
//! where the aircraft's `<engine file="engine0">`, `<thruster file="propeller0">`
//! and `<system file="engine0">` elements find them. Flight-control systems
//! address `propulsion/engine[n]`, which JSBSim cannot parameterise, so a part
//! ships one system document per engine position (`jsbsim/system-<n>.xml`) and
//! the role's trailing digits choose it. Served bytes are always verified bytes.

use syspack::{MemoryPackageSource, PackageSource, SystemKind, VerifiedSyspack};

use crate::PackError;

#[cfg(test)]
mod tests;

/// Where pinned part packages are found: the parts shipped in the binary, or
/// a library of parts a user downloaded.
pub trait PartLibrary {
    /// The package files of the part with `id`, if the library holds it.
    fn part_source(&self, id: &str) -> Option<Box<dyn PackageSource>>;
}

/// One part shipped in the binary: its manifest and assets.
struct BuiltinPart {
    id: &'static str,
    manifest: &'static [u8],
    assets: &'static [(&'static str, &'static [u8])],
}

const BUILTIN: &[BuiltinPart] = &[];

/// Ids of the parts shipped in the binary.
#[must_use]
pub fn builtin_part_ids() -> Vec<&'static str> {
    BUILTIN.iter().map(|part| part.id).collect()
}

/// The shipped part with `id`, as package files.
#[must_use]
pub fn builtin_part_source(id: &str) -> Option<MemoryPackageSource> {
    let part = BUILTIN.iter().find(|part| part.id == id)?;
    Some(MemoryPackageSource::from_embedded(
        part.manifest,
        part.assets,
    ))
}

/// The parts shipped in the binary, as a [`PartLibrary`].
#[derive(Debug, Clone, Copy, Default)]
pub struct BuiltinParts;

impl PartLibrary for BuiltinParts {
    fn part_source(&self, id: &str) -> Option<Box<dyn PackageSource>> {
        builtin_part_source(id).map(|source| Box::new(source) as Box<dyn PackageSource>)
    }
}

/// A part verified against the vehicle's pin, in the role the vehicle
/// installs it in.
#[derive(Debug, Clone, PartialEq)]
pub struct InstalledPart {
    /// The role the vehicle pins it in.
    pub role: String,
    /// The verified part package.
    pub package: VerifiedSyspack,
}

/// Every part `vehicle` pins, each verified against its pin.
pub(crate) fn resolve_parts(
    vehicle: &VerifiedSyspack,
    library: &dyn PartLibrary,
) -> Result<Vec<InstalledPart>, PackError> {
    vehicle
        .manifest()
        .parts
        .iter()
        .map(|pin| {
            let source = library
                .part_source(&pin.id)
                .ok_or_else(|| part_error(&pin.role, &pin.id, "no part library holds this id"))?;
            let package = VerifiedSyspack::load_part(source.as_ref(), pin)?;
            Ok(InstalledPart {
                role: pin.role.clone(),
                package,
            })
        })
        .collect()
}

/// The JSBSim document a part of `kind` contributes, as a package asset.
const fn fragment_asset(kind: SystemKind) -> Option<&'static str> {
    match kind {
        SystemKind::Engine => Some("jsbsim/engine.xml"),
        SystemKind::Propeller => Some("jsbsim/propeller.xml"),
        _ => None,
    }
}

/// The JSBSim virtual paths and verified bytes the installed parts serve.
pub(crate) fn jsbsim_part_files(
    parts: &[InstalledPart],
) -> Result<Vec<(String, &[u8])>, PackError> {
    let mut files = Vec::new();
    for part in parts {
        let manifest = part.package.manifest();
        if let Some(bytes) = fragment_asset(manifest.kind).and_then(|a| part.package.asset(a)) {
            files.push((format!("engine/{}.xml", part.role), bytes));
        }
        let has_systems = manifest
            .assets
            .keys()
            .any(|path| path.starts_with("jsbsim/system-"));
        if has_systems {
            let index = role_index(&part.role).ok_or_else(|| {
                part_error(
                    &part.role,
                    &manifest.id,
                    "the part ships per-engine systems, so its role must end in the engine index",
                )
            })?;
            let asset = format!("jsbsim/system-{index}.xml");
            let bytes = part.package.asset(&asset).ok_or_else(|| {
                part_error(
                    &part.role,
                    &manifest.id,
                    &format!("the part ships no system for engine {index} ({asset})"),
                )
            })?;
            files.push((format!("systems/{}.xml", part.role), bytes));
        }
    }
    Ok(files)
}

/// The engine index a role ends in: `engine0` → `0`, `propeller12` → `12`.
fn role_index(role: &str) -> Option<&str> {
    let start = role.trim_end_matches(|c: char| c.is_ascii_digit()).len();
    role.get(start..).filter(|digits| !digits.is_empty())
}

fn part_error(role: &str, id: &str, detail: &str) -> PackError {
    PackError::Part {
        role: role.to_owned(),
        id: id.to_owned(),
        detail: detail.to_owned(),
    }
}

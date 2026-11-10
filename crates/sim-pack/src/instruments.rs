//! Shared instrument packs and how a vehicle's pin on one is resolved.
//!
//! A pack is its own system package (`kind: InstrumentPack`) so several
//! aircraft fly the same instruments; the vehicle manifest pins it by id,
//! version and the SHA-256 of its `syspack.json`. Packs ship inside the
//! binary, and a pin that names no shipped pack, or a different revision of
//! one, refuses the vehicle rather than flying it with other instruments.

use syspack::{
    DeviceKindSpec, InstrumentPackDocument, MemoryPackageSource, MountedDeviceSpec, SourceBinding,
    VerifiedSyspack,
};

use crate::PackError;

#[cfg(test)]
mod tests;

const GA_ANALOG_SIX_PACK_MANIFEST: &[u8] =
    include_bytes!("../../../instruments/ga-analog-six-pack/syspack.json");
const GA_ANALOG_SIX_PACK_INSTRUMENTS: &[u8] =
    include_bytes!("../../../instruments/ga-analog-six-pack/instruments.json");
const USAAF_1943_FIGHTER_MANIFEST: &[u8] =
    include_bytes!("../../../instruments/usaaf-1943-fighter/syspack.json");
const USAAF_1943_FIGHTER_INSTRUMENTS: &[u8] =
    include_bytes!("../../../instruments/usaaf-1943-fighter/instruments.json");
const USAAF_1943_FIGHTER_PROVENANCE: &[u8] =
    include_bytes!("../../../instruments/usaaf-1943-fighter/PROVENANCE.md");

/// Ids of the instrument packs shipped in the binary.
pub const BUILTIN_INSTRUMENT_PACKS: [&str; 2] = ["ga-analog-six-pack", "usaaf-1943-fighter"];

/// The shipped instrument pack with `id`, as package files.
#[must_use]
pub fn builtin_instrument_pack_source(id: &str) -> Option<MemoryPackageSource> {
    match id {
        "ga-analog-six-pack" => Some(MemoryPackageSource::from_embedded(
            GA_ANALOG_SIX_PACK_MANIFEST,
            &[(syspack::INSTRUMENTS_ASSET, GA_ANALOG_SIX_PACK_INSTRUMENTS)],
        )),
        "usaaf-1943-fighter" => Some(MemoryPackageSource::from_embedded(
            USAAF_1943_FIGHTER_MANIFEST,
            &[
                (syspack::INSTRUMENTS_ASSET, USAAF_1943_FIGHTER_INSTRUMENTS),
                ("PROVENANCE.md", USAAF_1943_FIGHTER_PROVENANCE),
            ],
        )),
        _ => None,
    }
}

/// The pack `vehicle` pins, verified against the pin, with every pitot-static
/// binding checked against the vehicle's own devices; `None` when it pins none.
pub(crate) fn resolve_pinned_pack(
    vehicle: &VerifiedSyspack,
) -> Result<Option<VerifiedSyspack>, PackError> {
    let Some(pin) = &vehicle.manifest().instrument_pack else {
        return Ok(None);
    };
    let source =
        builtin_instrument_pack_source(&pin.id).ok_or_else(|| PackError::InstrumentPack {
            pack: pin.id.clone(),
            detail: "no shipped instrument pack has this id".to_owned(),
        })?;
    let pack = VerifiedSyspack::load_pinned(&source, pin)?;
    if let Some(document) = pack.instrument_pack() {
        check_bindings(vehicle, &pin.id, document)?;
        crate::resolve_typefaces(document)?;
    }
    Ok(Some(pack))
}

/// Every `pitot_static.<id>` source must name a pitot-static device the vehicle
/// mounts, so an instrument can never read plumbing the aircraft lacks.
fn check_bindings(
    vehicle: &VerifiedSyspack,
    pack_id: &str,
    document: &InstrumentPackDocument,
) -> Result<(), PackError> {
    for instrument in &document.instruments {
        if let Some(SourceBinding::PitotStatic { device }) =
            SourceBinding::parse(&instrument.source)
            && pitot_static_device(vehicle, &device).is_none()
        {
            return Err(PackError::InstrumentPack {
                pack: pack_id.to_owned(),
                detail: format!(
                    "instrument {:?} reads pitot-static device {device:?}, which {} does not mount",
                    instrument.id,
                    vehicle.manifest().id
                ),
            });
        }
    }
    Ok(())
}

/// The vehicle's pitot-static device with `id`.
pub(crate) fn pitot_static_device<'a>(
    vehicle: &'a VerifiedSyspack,
    id: &str,
) -> Option<&'a MountedDeviceSpec> {
    vehicle
        .manifest()
        .devices
        .iter()
        .find(|d| d.id == id && matches!(d.kind, DeviceKindSpec::PitotStatic { .. }))
}

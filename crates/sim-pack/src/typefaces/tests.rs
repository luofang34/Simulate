use syspack::InstrumentPackDocument;

use super::{BUILTIN_TYPEFACES, builtin_typeface_source, resolve_typefaces};
use crate::{PackError, builtin_p38g_package};

#[test]
fn the_fighter_pack_resolves_the_face_its_dials_are_lettered_in() {
    let p38 = builtin_p38g_package().unwrap();
    let pack = p38.instrument_pack().unwrap();
    let faces = resolve_typefaces(pack).unwrap();
    assert_eq!(faces.len(), 1);
    assert!(faces[0].typeface_face().is_some_and(|f| !f.is_empty()));
    for id in BUILTIN_TYPEFACES {
        assert!(builtin_typeface_source(id).is_some());
    }
}

#[test]
fn a_stale_or_unknown_typeface_pin_refuses_the_pack() {
    let p38 = builtin_p38g_package().unwrap();
    let mut pack: InstrumentPackDocument = p38.instrument_pack().unwrap().clone();
    pack.typefaces[0].manifest_sha256 = "0".repeat(64);
    assert!(resolve_typefaces(&pack).is_err());
    pack.typefaces[0].id = "din-1451".to_owned();
    assert!(matches!(
        resolve_typefaces(&pack),
        Err(PackError::Typeface { .. })
    ));
}

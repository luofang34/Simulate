use std::collections::BTreeMap;

use jsbsim::Resolver as _;
use serde_json::{Value, json};
use syspack::{MemoryPackageSource, PackageSource, PartRef, sha256_hex};

use super::*;
use crate::{LoadedSyspack, PackError, SyspackError};

fn component(key: &str) -> Value {
    json!({
        "key": key,
        "name": key,
        "parent": null,
        "translation_m": [0.0, 0.0, 0.0],
        "dry_mass_kg": 100.0,
        "dry_inertia_body": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        "initial_fluid_mass_kg": 0.0,
        "initial_payload_mass_kg": 0.0,
        "attached": true,
        "thermal_capacity_j_k": 0.0,
        "nominal_power_w": 0.0,
    })
}

/// A package of `kind` holding `files`, as its manifest bytes and source.
fn package(
    id: &str,
    kind: &str,
    dynamics: Value,
    files: &[(&str, &[u8])],
    parts: &[PartRef],
) -> MemoryPackageSource {
    let assets: serde_json::Map<String, Value> = files
        .iter()
        .map(|(path, bytes)| {
            let digest = json!({"sha256": sha256_hex(bytes), "size_bytes": bytes.len()});
            ((*path).to_owned(), digest)
        })
        .collect();
    let manifest = json!({
        "schema_version": 1,
        "id": id,
        "name": id,
        "version": "1.0.0",
        "kind": kind,
        "assets": assets,
        "components": [component("body")],
        "dynamics": dynamics,
        "devices": [],
        "parts": parts,
    });
    MemoryPackageSource::new(
        manifest.to_string().into_bytes(),
        files
            .iter()
            .map(|(path, bytes)| ((*path).to_owned(), bytes.to_vec()))
            .collect(),
    )
}

fn engine_part() -> MemoryPackageSource {
    package(
        "test-radial",
        "Engine",
        json!("None"),
        &[
            ("engine.json", b"{}"),
            ("jsbsim/engine.xml", b"<piston_engine/>"),
            ("jsbsim/system-0.xml", b"<system name=\"e0\"/>"),
            ("jsbsim/system-1.xml", b"<system name=\"e1\"/>"),
        ],
        &[],
    )
}

fn propeller_part() -> MemoryPackageSource {
    package(
        "test-propeller",
        "Propeller",
        json!("None"),
        &[
            ("propeller.json", b"{}"),
            ("jsbsim/propeller.xml", b"<propeller/>"),
        ],
        &[],
    )
}

struct Library(BTreeMap<&'static str, MemoryPackageSource>);

impl PartLibrary for Library {
    fn part_source(&self, id: &str) -> Option<Box<dyn PackageSource>> {
        self.0
            .get(id)
            .cloned()
            .map(|source| Box::new(source) as Box<dyn PackageSource>)
    }
}

fn library() -> Library {
    Library(BTreeMap::from([
        ("test-radial", engine_part()),
        ("test-propeller", propeller_part()),
    ]))
}

fn pin(source: &MemoryPackageSource, role: &str) -> PartRef {
    VerifiedSyspack::load(source).unwrap().part_ref(role)
}

fn twin(parts: &[PartRef]) -> MemoryPackageSource {
    let dynamics = json!({"JsbsimXml": {
        "aircraft_dir": "aircraft",
        "model_name": "twin",
        "entry_asset": "fdm/twin.xml",
        "virtual_path": "aircraft/twin/twin.xml",
        "additional_assets": [],
    }});
    package(
        "twin",
        "FixedWing",
        dynamics,
        &[("fdm/twin.xml", b"<fdm_config/>")],
        parts,
    )
}

fn served(vehicle: &LoadedSyspack, path: &str) -> Vec<u8> {
    vehicle
        .build_jsbsim_resolver()
        .unwrap()
        .resolve(path)
        .unwrap()
}

#[test]
fn each_role_serves_its_part_where_the_aircraft_looks() {
    let parts = [
        pin(&engine_part(), "engine0"),
        pin(&engine_part(), "engine1"),
        pin(&propeller_part(), "propeller0"),
    ];
    let vehicle = LoadedSyspack::load_with_parts(&twin(&parts), &library()).unwrap();
    assert_eq!(vehicle.parts().len(), 3);
    assert_eq!(served(&vehicle, "engine/engine0.xml"), b"<piston_engine/>");
    assert_eq!(served(&vehicle, "engine/engine1.xml"), b"<piston_engine/>");
    assert_eq!(
        served(&vehicle, "systems/engine0.xml"),
        b"<system name=\"e0\"/>"
    );
    assert_eq!(
        served(&vehicle, "systems/engine1.xml"),
        b"<system name=\"e1\"/>"
    );
    assert_eq!(served(&vehicle, "engine/propeller0.xml"), b"<propeller/>");
    assert_eq!(served(&vehicle, "aircraft/twin/twin.xml"), b"<fdm_config/>");
    let installed = vehicle.part("propeller0").unwrap();
    assert_eq!(installed.manifest().id, "test-propeller");
}

#[test]
fn a_part_no_library_holds_refuses_the_vehicle() {
    let parts = [pin(&engine_part(), "engine0")];
    let empty = Library(BTreeMap::new());
    assert!(matches!(
        LoadedSyspack::load_with_parts(&twin(&parts), &empty),
        Err(PackError::Part { role, .. }) if role == "engine0"
    ));
    assert!(builtin_part_source("test-radial").is_none());
}

#[test]
fn another_revision_of_a_part_refuses_the_vehicle() {
    let mut stale = pin(&engine_part(), "engine0");
    stale.manifest_sha256 = "c".repeat(64);
    assert!(matches!(
        LoadedSyspack::load_with_parts(&twin(&[stale]), &library()),
        Err(PackError::Syspack(SyspackError::PartPinMismatch {
            field: "manifest_sha256",
            ..
        }))
    ));
}

#[test]
fn per_engine_systems_need_an_indexed_role_with_a_system() {
    let unindexed = [pin(&engine_part(), "engine")];
    let vehicle = LoadedSyspack::load_with_parts(&twin(&unindexed), &library()).unwrap();
    assert!(matches!(
        vehicle.build_jsbsim_resolver(),
        Err(PackError::Part { .. })
    ));
    let third = [pin(&engine_part(), "engine2")];
    let vehicle = LoadedSyspack::load_with_parts(&twin(&third), &library()).unwrap();
    assert!(matches!(
        vehicle.build_jsbsim_resolver(),
        Err(PackError::Part { detail, .. }) if detail.contains("system-2")
    ));
}

#[test]
fn role_indices_are_the_trailing_digits() {
    assert_eq!(role_index("engine0"), Some("0"));
    assert_eq!(role_index("propeller12"), Some("12"));
    assert_eq!(role_index("engine"), None);
}

#[test]
fn every_shipped_part_loads_against_its_own_pin_with_its_documents() {
    for id in builtin_part_ids() {
        let source = builtin_part_source(id).unwrap();
        let pin = pin(&source, "engine0");
        let part = VerifiedSyspack::load_part(&source, &pin).unwrap();
        let kind = part.manifest().kind;
        let document = kind.part_document().unwrap();
        let parsed: Value = serde_json::from_slice(part.asset(document).unwrap()).unwrap();
        assert_eq!(parsed["id"], id);
        let catalog: Value =
            serde_json::from_slice(part.asset("ace/catalog-entry.json").unwrap()).unwrap();
        assert_eq!(catalog["document"]["type"], catalog["kind"], "{id}");
        let fragment = fragment_asset(kind).unwrap();
        let root = if kind == SystemKind::Engine {
            "<piston_engine"
        } else {
            "<propeller"
        };
        let text = std::str::from_utf8(part.asset(fragment).unwrap()).unwrap();
        assert!(text.contains(root), "{id}");
        for n in 0..4 {
            let system =
                std::str::from_utf8(part.asset(&format!("jsbsim/system-{n}.xml")).unwrap())
                    .unwrap();
            assert!(
                system.contains(&format!("propulsion/engine[{n}]")),
                "{id} {n}"
            );
        }
    }
    assert_eq!(builtin_part_ids().len(), 3);
}

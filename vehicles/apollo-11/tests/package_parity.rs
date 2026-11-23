//! A built-in vehicle is an ordinary package: Eagle and Columbia read from
//! their folders, as a user who downloaded them would, are the embedded ones.

use std::path::Path;

use glam::DVec3;
use mbd_aero::{AeroElementKind, LinkFlow};
use mbd_core::AmbientGas;
use sim_apollo::{apollo_csm_package, apollo_lm_package};
use sim_pack::{DirectoryPackageSource, LoadedSyspack, PackError};

fn from_disk(package: &str) -> Result<LoadedSyspack, PackError> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("packages")
        .join(package);
    LoadedSyspack::load(&DirectoryPackageSource::new(dir))
}

#[cfg(test)]
fn assert_same_vehicle(embedded: &LoadedSyspack, downloaded: &LoadedSyspack) {
    assert_eq!(embedded.manifest(), downloaded.manifest());
    assert_eq!(embedded.tree(), downloaded.tree());
    assert_eq!(
        embedded.tree().mass_properties().unwrap(),
        downloaded.tree().mass_properties().unwrap()
    );
    assert_eq!(
        embedded.build_aero_elements().unwrap(),
        downloaded.build_aero_elements().unwrap()
    );
}

#[test]
fn eagle_on_disk_is_the_embedded_eagle() {
    assert_same_vehicle(
        &apollo_lm_package().unwrap(),
        &from_disk("eagle-lm").unwrap(),
    );
}

#[test]
fn columbia_on_disk_is_the_embedded_columbia() {
    let embedded = apollo_csm_package().unwrap();
    assert_same_vehicle(&embedded, &from_disk("columbia-csm").unwrap());
    let aero = embedded.build_aero_elements().unwrap();
    let AeroElementKind::BluntCapsule(capsule) = &aero[0].kind else {
        panic!("Columbia's element is its capsule: {aero:?}");
    };
    let (sin, cos) = (-0.3_f64).sin_cos();
    let wrench = capsule
        .evaluate(&LinkFlow {
            velocity_air_link_m_s: DVec3::new(cos, 0.0, sin) * -7_000.0,
            angular_rate_link_rad_s: DVec3::new(0.0, 0.02, 0.0),
            ambient: AmbientGas::earth_air(100.0, 230.0).unwrap(),
            controls: &[],
        })
        .unwrap();
    assert!(wrench.force_link_n.length() > 0.0, "the capsule has drag");
}

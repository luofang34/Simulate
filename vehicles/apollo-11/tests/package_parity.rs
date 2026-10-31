//! A built-in vehicle is an ordinary package: Eagle and Columbia read from
//! their folders, as a user who downloaded them would, are the embedded ones.

use std::path::Path;

use glam::DVec3;
use mbd_core::{AeroFlowState, AmbientFlow};
use sim_apollo::{apollo_csm_package, apollo_lm_package};
use sim_pack::{DirectoryPackageSource, LoadedSyspack, PackError};

fn from_disk(package: &str) -> Result<LoadedSyspack, PackError> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("packages")
        .join(package);
    LoadedSyspack::load(&DirectoryPackageSource::new(dir))
}

fn flow(alpha_rad: f64) -> AeroFlowState {
    let (sin, cos) = alpha_rad.sin_cos();
    AeroFlowState {
        velocity_air_body_m_s: DVec3::new(cos, 0.0, sin) * -7_000.0,
        angular_rate_body_rad_s: DVec3::new(0.0, 0.02, 0.0),
        ambient: AmbientFlow {
            pressure_pa: 100.0,
            temperature_k: 230.0,
            density_kg_m3: 1.5e-3,
            sound_speed_m_s: 300.0,
        },
        control_surfaces_rad: [0.0; 8],
    }
}

#[cfg(test)]
fn assert_same_vehicle(embedded: &LoadedSyspack, downloaded: &LoadedSyspack) {
    assert_eq!(embedded.manifest(), downloaded.manifest());
    assert_eq!(embedded.tree(), downloaded.tree());
    assert_eq!(
        embedded.tree().mass_properties().unwrap(),
        downloaded.tree().mass_properties().unwrap()
    );
    let aero = (
        embedded.build_aero_model().unwrap(),
        downloaded.build_aero_model().unwrap(),
    );
    for alpha_deg in [-30.0, -20.0, 0.0, 15.0, 170.0_f64] {
        let flow = flow(alpha_deg.to_radians());
        assert_eq!(
            aero.0.evaluate(&flow).unwrap(),
            aero.1.evaluate(&flow).unwrap()
        );
    }
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
    let wrench = embedded
        .build_aero_model()
        .unwrap()
        .evaluate(&flow(-0.3))
        .unwrap();
    assert!(wrench.force_body_n.length() > 0.0, "the capsule has drag");
}

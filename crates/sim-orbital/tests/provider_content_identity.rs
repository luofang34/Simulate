//! The orbital configuration hash covers each body's magnetic field and
//! atmosphere by what they contain, never by what they are called.

use std::sync::Arc;

use glam::DVec3;
use sim_atmosphere::VleoThermosphereAtmosphere;
use sim_bodies::{BodyCatalog, BodyDescriptor, HarmonicMagnetism};
use sim_geo::{MagneticField, MagneticFieldError};
use sim_orbital::{
    BodyGravitySpec, ContentHash, DynamicalMode, ForceModel, GravityConstantsCatalog, NaifId,
};

/// The force-model hash of a run whose only described body is `earth`.
#[cfg(test)]
fn configuration_hash(earth: BodyDescriptor) -> ContentHash {
    let ellipsoid = earth.ellipsoid;
    let catalog = Arc::new(BodyCatalog::new().with(earth));
    let constants = GravityConstantsCatalog::from_specs([BodyGravitySpec {
        naif_id: NaifId::EARTH,
        mu_m3_s2: ellipsoid.mu_m3_s2,
        equatorial_radius_m: ellipsoid.semi_major_m,
        polar_radius_m: ellipsoid.semi_major_m,
        harmonics: None,
    }])
    .with_body_catalog(catalog);
    ForceModel::new(
        constants,
        vec![NaifId::EARTH],
        Vec::new(),
        DynamicalMode::PrescribedEphemeris,
        0,
    )
    .unwrap()
    .content_hash()
}

#[cfg(test)]
fn dipole(g10_nt: f64) -> Arc<dyn MagneticField> {
    Arc::new(HarmonicMagnetism::dipole(6_371_200.0, [g10_nt, 0.0, 0.0], 1900.0, 2100.0).unwrap())
}

#[cfg(test)]
fn with_field(name: &str, field: Arc<dyn MagneticField>) -> BodyDescriptor {
    BodyDescriptor::earth()
        .unwrap()
        .with_magnetic_field(name, field)
}

#[cfg(test)]
fn with_air(name: &str, f107_sfu: f64) -> BodyDescriptor {
    BodyDescriptor::earth().unwrap().with_atmosphere(
        name,
        Arc::new(VleoThermosphereAtmosphere::with_f107(f107_sfu)),
    )
}

#[test]
fn magnetic_fields_of_one_name_and_different_coefficients_hash_apart() {
    let weak = configuration_hash(with_field("main-field", dipole(-29_400.0)));
    let strong = configuration_hash(with_field("main-field", dipole(-29_401.0)));
    assert_ne!(weak, strong, "one name, two fields");
    assert_eq!(
        weak,
        configuration_hash(with_field("main-field", dipole(-29_400.0))),
        "the same field built twice is the same content"
    );
    assert_eq!(
        weak,
        configuration_hash(with_field("another-name", dipole(-29_400.0))),
        "a name is not content"
    );
    assert_ne!(
        weak,
        configuration_hash(BodyDescriptor::earth().unwrap()),
        "a dipole is not IGRF"
    );
    assert_eq!(
        configuration_hash(BodyDescriptor::earth().unwrap()),
        configuration_hash(with_field(
            "igrf",
            Arc::new(HarmonicMagnetism::igrf14().unwrap())
        )),
        "IGRF read twice is one content"
    );
}

#[test]
fn atmospheres_of_one_name_and_different_parameters_hash_apart() {
    let quiet = configuration_hash(with_air("thermosphere", 140.0));
    let active = configuration_hash(with_air("thermosphere", 141.0));
    assert_ne!(quiet, active, "one name, two atmospheres");
    assert_eq!(quiet, configuration_hash(with_air("thermosphere", 140.0)));
    assert_eq!(quiet, configuration_hash(with_air("renamed", 140.0)));
    assert_eq!(
        quiet,
        configuration_hash(BodyDescriptor::earth().unwrap()),
        "the Earth's own entry is that atmosphere at 140 sfu"
    );
    let airless = BodyDescriptor::earth()
        .unwrap()
        .with_atmosphere("thermosphere", Arc::new(sim_atmosphere::VacuumAtmosphere));
    assert_ne!(quiet, configuration_hash(airless), "a kind is content");
}

/// A uniform field that states its content in slices of `chunk` bytes.
#[derive(Debug)]
#[cfg(test)]
struct Sliced {
    field_nt: DVec3,
    chunk: usize,
}

#[cfg(test)]
impl MagneticField for Sliced {
    fn field_fixed_nt(&self, _fixed: DVec3, _year: f64) -> Result<DVec3, MagneticFieldError> {
        Ok(self.field_nt)
    }

    fn visit_content(&self, f: &mut dyn FnMut(&[u8])) {
        let mut bytes = b"uniform".to_vec();
        for value in self.field_nt.to_array() {
            bytes.extend(value.to_bits().to_le_bytes());
        }
        for slice in bytes.chunks(self.chunk) {
            f(slice);
        }
    }
}

/// How a provider cuts its content into slices is not content.
#[test]
fn the_hash_reads_the_concatenated_content_alone() {
    let sliced = |chunk, x| {
        let field_nt = DVec3::new(x, 2.0e4, -3.0e4);
        configuration_hash(with_field("uniform", Arc::new(Sliced { field_nt, chunk })))
    };
    assert_eq!(sliced(1, 1.0e4), sliced(7, 1.0e4));
    assert_eq!(sliced(7, 1.0e4), sliced(4_096, 1.0e4));
    assert_ne!(sliced(7, 1.0e4), sliced(7, 1.0e4 + 1.0e-9));
}

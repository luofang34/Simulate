use mbd_core::GasSpecies;
use sim_geo::{Geodetic, UtcTime};

use super::VleoThermosphereAtmosphere;
use crate::{AtmosphereField, VacuumAtmosphere};

fn sample_at(field: &VleoThermosphereAtmosphere, height_m: f64) -> crate::AtmosphereSample {
    field
        .sample(
            &Geodetic::from_degrees(0.0, 0.0, height_m),
            UtcTime::from_unix_seconds(0.0),
        )
        .unwrap()
}

#[test]
fn atomic_oxygen_overtakes_n2_in_vleo_and_dominates_at_350km() {
    let field = VleoThermosphereAtmosphere::new();

    let at_120 = sample_at(&field, 120_000.0).ambient_gas();
    assert!(at_120.mole_fraction(GasSpecies::N2) > at_120.mole_fraction(GasSpecies::AtomicO));

    let at_220 = sample_at(&field, 220_000.0).ambient_gas();
    assert!(
        at_220.mole_fraction(GasSpecies::AtomicO) > at_220.mole_fraction(GasSpecies::N2),
        "O={} N2={}",
        at_220.mole_fraction(GasSpecies::AtomicO),
        at_220.mole_fraction(GasSpecies::N2)
    );

    let at_350 = sample_at(&field, 350_000.0).ambient_gas();
    assert!(
        at_350.mole_fraction(GasSpecies::AtomicO) > 0.70,
        "O fraction at 350 km = {}",
        at_350.mole_fraction(GasSpecies::AtomicO)
    );
    assert!(at_350.number_density_m3(GasSpecies::AtomicO) > 1.0e13);
}

#[test]
fn higher_solar_flux_increases_vleo_density_and_vacuum_top_matches() {
    let low = VleoThermosphereAtmosphere::with_f107(70.0);
    let high = VleoThermosphereAtmosphere::with_f107(220.0);

    let r_low = sample_at(&low, 300_000.0).density_kg_m3;
    let r_high = sample_at(&high, 300_000.0).density_kg_m3;
    assert!(r_high > r_low * 1.5, "high={r_high} low={r_low}");

    assert_eq!(sample_at(&high, 600_000.0), VacuumAtmosphere::SAMPLE);
    assert_eq!(sample_at(&high, 800_000.0), VacuumAtmosphere::SAMPLE);
}

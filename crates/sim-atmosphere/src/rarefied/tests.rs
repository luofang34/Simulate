use super::*;
use crate::{AtmosphereField, FullColumnAtmosphere};
use sim_geo::{Geodetic, UtcTime};

/// A sensor-boom-sized body, small enough that the upper mesosphere slips.
const PROBE_LENGTH_M: f64 = 0.3;

fn at_msl(msl_m: f64) -> AtmosphereSample {
    let position = Geodetic::from_degrees(0.0, 0.0, 0.0);
    let undulation = sim_geo::egm96_undulation_m(position.lat_rad, position.lon_rad);
    FullColumnAtmosphere::new()
        .sample(
            &Geodetic {
                height_m: msl_m + undulation,
                ..position
            },
            UtcTime::from_unix_seconds(0.0),
        )
        .unwrap()
}

#[test]
fn sea_level_mean_free_path_and_viscosity_match_kinetic_theory() {
    let sea = at_msl(0.0);
    let lambda = sea.mean_free_path_m();
    assert!((lambda - 6.6e-8).abs() < 0.1e-8, "{lambda}");
    let mu = sea.dynamic_viscosity_pa_s();
    assert!((mu - 1.789e-5).abs() < 0.005e-5, "{mu}");
    assert_eq!(sea.flow_regime(PROBE_LENGTH_M), FlowRegime::Continuum);
}

#[test]
fn rarefaction_climbs_through_every_regime_to_vacuum() {
    for (msl_km, regime) in [
        (80.0, FlowRegime::Slip),
        (100.0, FlowRegime::Transitional),
        (150.0, FlowRegime::FreeMolecular),
        (190.0, FlowRegime::FreeMolecular),
    ] {
        let sample = at_msl(msl_km * 1_000.0);
        let kn = sample.knudsen_number(PROBE_LENGTH_M);
        assert_eq!(
            sample.flow_regime(PROBE_LENGTH_M),
            regime,
            "{msl_km} km Kn={kn}"
        );
    }
    let space = at_msl(250_000.0);
    assert_eq!(space.mean_free_path_m(), f64::INFINITY);
    assert_eq!(space.knudsen_number(PROBE_LENGTH_M), f64::INFINITY);
    assert_eq!(space.dynamic_viscosity_pa_s(), 0.0);
    assert_eq!(space.flow_regime(PROBE_LENGTH_M), FlowRegime::FreeMolecular);
}

#[test]
fn knudsen_thresholds_and_degenerate_lengths() {
    assert_eq!(FlowRegime::from_knudsen(0.009_9), FlowRegime::Continuum);
    assert_eq!(FlowRegime::from_knudsen(0.01), FlowRegime::Slip);
    assert_eq!(FlowRegime::from_knudsen(0.1), FlowRegime::Transitional);
    assert_eq!(FlowRegime::from_knudsen(10.0), FlowRegime::FreeMolecular);
    assert_eq!(
        FlowRegime::from_knudsen(f64::NAN),
        FlowRegime::FreeMolecular
    );
    let sea = at_msl(0.0);
    assert_eq!(sea.knudsen_number(0.0), f64::INFINITY);
    assert_eq!(sea.knudsen_number(f64::NAN), f64::INFINITY);
}

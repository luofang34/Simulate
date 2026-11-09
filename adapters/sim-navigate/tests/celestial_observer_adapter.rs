//! Verifies `CelestialObserverAdapter` and `PureEphemerisNavAdapter` bridging
//! `sim-spacetime` / `sim-orbital` catalogs into `navigate-celestial`.

use glam::DVec3;
use navigate_celestial::{Correction, FrameId, State, observe_body};
use sim_navigate::{CelestialObserverAdapter, PureEphemerisNavAdapter, tdb_to_hifitime_epoch};
use sim_orbital::{BuiltinSolarSystem2026_2028, FrameCode, NaifId, TdbEpoch};

#[test]
fn celestial_observer_and_pure_ephemeris_adapters_evaluate_sun_sights() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let ep = tdb_to_hifitime_epoch(TdbEpoch::new(840_000_000, 0));

    let adapter = CelestialObserverAdapter {
        ephemeris: &solar.ephemeris,
        orientation: &solar.orientation,
    };
    let obs_itrf = State {
        position_m: [6_378_137.0, 0.0, 0.0],
        velocity_m_s: [0.0, 0.0, 0.0],
        epoch: ep,
        frame: FrameId {
            center_naif_id: NaifId::EARTH.0,
            orientation_naif_id: FrameCode::ITRF93.0,
        },
    };
    let geom = observe_body(&adapter, obs_itrf, NaifId::SUN.0, Correction::Geometric)
        .expect("geometric sun sight");
    let app = observe_body(
        &adapter,
        obs_itrf,
        NaifId::SUN.0,
        Correction::VacuumReception,
    )
    .expect("aberrated sun sight");
    let lt_s = app.light_time.expect("has light time").to_seconds();
    assert!(
        (490.0..510.0).contains(&lt_s),
        "1 AU light time ~499 s, got {lt_s}"
    );
    let dot =
        (DVec3::from_array(geom.direction).dot(DVec3::from_array(app.direction))).clamp(-1.0, 1.0);
    let aberr_arcsec = dot.acos().to_degrees() * 3_600.0;
    assert!(
        (19.0..22.0).contains(&aberr_arcsec),
        "annual stellar/solar aberration ~20.5 arcsec, got {aberr_arcsec}"
    );

    let pure_adapter = PureEphemerisNavAdapter {
        ephemeris: &solar.ephemeris,
    };
    let obs_j2000 = State {
        position_m: [6_378_137.0, 0.0, 0.0],
        velocity_m_s: [0.0, 7_800.0, 0.0],
        epoch: ep,
        frame: FrameId {
            center_naif_id: NaifId::EARTH.0,
            orientation_naif_id: FrameCode::J2000.0,
        },
    };
    let j2000_sight = observe_body(
        &pure_adapter,
        obs_j2000,
        NaifId::SUN.0,
        Correction::Geometric,
    )
    .expect("J2000 sun sight");
    assert!((DVec3::from_array(j2000_sight.direction).length() - 1.0).abs() < 1e-12);
}

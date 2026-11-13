use jsbsim::MemoryResolver;
use sim_geo::{LocalFrame, ecef_to_geodetic};

use super::*;

const C172_XML: &str = include_str!("../../../../../vehicles/c172s/fdm/c172s.xml");
const C172_MODEL: &str = "c172s";

/// Newark's runway on the WGS84 ellipsoid, where the geoid is about 33 m
/// below it.
const KEWR_RUNWAY_M: f64 = -28.7;
/// Denver's field elevation, far above the ellipsoid.
const KDEN_RUNWAY_M: f64 = 1_650.0;
/// The C172's CG height above the ground resting on its compressed gear.
const PARKED_CG_AGL_M: f64 = 1.324;

fn c172_resolver() -> MemoryResolver {
    let mut files = MemoryResolver::new();
    files.insert("aircraft/c172s/c172s.xml", C172_XML);
    files
}

fn parked_config(files: &MemoryResolver, runway_m: f64) -> JsbsimFdmConfig<'_> {
    JsbsimFdmConfig {
        resolver: files,
        aircraft_path: "aircraft",
        model: C172_MODEL,
        period: JSBSIM_DEFAULT_PERIOD,
        initial: Geodetic::from_degrees(40.6925, -74.1687, runway_m + PARKED_CG_AGL_M),
        heading_rad: 0.0,
        speed_m_s: 0.0,
        alpha_rad: 0.0,
        engine_start: EngineStartMode::ColdAndDark,
    }
}

fn height_and_climb(block: &JsbsimFdmBlock) -> (f64, f64) {
    let state = block.output();
    let geo = ecef_to_geodetic(state.position_ecef);
    let climb = state.velocity_ecef.dot(LocalFrame::at(&geo).up());
    (geo.height_m, climb)
}

/// Steps a C172 parked on `runway_m` for five seconds and checks it neither
/// sinks through the ground nor is thrown off it, and is still from the start.
fn assert_parked_c172_stays_at_rest(runway_m: f64) {
    let files = c172_resolver();
    let mut block =
        JsbsimFdmBlock::new_on_terrain(&parked_config(&files, runway_m), runway_m).unwrap();
    assert_eq!(block.last_ground_sync_m(), Some(runway_m));
    let terrain_ft = block
        .fdm
        .read_node(block.nodes.terrain_elevation_asl_ft)
        .unwrap();
    assert!(
        (terrain_ft * METRES_PER_FOOT - runway_m).abs() < super::TERRAIN_READBACK_M,
        "the initial conditions kept the ground: {terrain_ft} ft"
    );

    let input = KinematicInput::default();
    for tick in 1..=600_u32 {
        block.step(&input).unwrap();
        let (height_m, climb_m_s) = height_and_climb(&block);
        let agl_m = height_m - runway_m;
        assert!(
            (1.10..1.40).contains(&agl_m),
            "tick {tick}: CG {agl_m} m above a {runway_m} m runway"
        );
        let limit_m_s = if tick >= 240 { 0.05 } else { 0.25 };
        assert!(
            climb_m_s.abs() < limit_m_s,
            "tick {tick}: vertical speed {climb_m_s} m/s on a {runway_m} m runway"
        );
    }
}

#[test]
fn a_c172_parked_below_the_ellipsoid_stays_on_its_gear() {
    assert_parked_c172_stays_at_rest(KEWR_RUNWAY_M);
}

#[test]
fn a_c172_parked_high_above_the_ellipsoid_stays_on_its_gear() {
    assert_parked_c172_stays_at_rest(KDEN_RUNWAY_M);
}

#[test]
fn starting_on_non_finite_terrain_is_refused() {
    let files = c172_resolver();
    let config = parked_config(&files, 0.0);
    assert!(JsbsimFdmBlock::new_on_terrain(&config, f64::NAN).is_err());
    let default_ground = JsbsimFdmBlock::new(&config).unwrap();
    assert_eq!(default_ground.last_ground_sync_m(), None);
}

#[test]
fn late_dem_refinement_lifts_parked_c172_without_belly_slam_and_leaves_airborne_untouched() {
    let files = c172_resolver();
    let mut parked =
        JsbsimFdmBlock::new_on_terrain(&parked_config(&files, KEWR_RUNWAY_M), KEWR_RUNWAY_M)
            .unwrap();
    parked.sync_ground_versioned_m(KEWR_RUNWAY_M, 0).unwrap();
    let idle = KinematicInput::default();
    for _ in 0..24 {
        parked.step(&idle).unwrap();
    }
    let refined_m = KEWR_RUNWAY_M + 3.8;
    parked.sync_ground_versioned_m(refined_m, 1).unwrap();
    let (immediate_h, _) = height_and_climb(&parked);
    assert!(
        (1.10..1.40).contains(&(immediate_h - refined_m)),
        "late DEM jump preserved parked gear height: {} m above {refined_m}",
        immediate_h - refined_m
    );
    for tick in 1..=120_u32 {
        parked.sync_ground_versioned_m(refined_m, 1).unwrap();
        parked.step(&idle).unwrap();
        let (height_m, climb_m_s) = height_and_climb(&parked);
        let agl_m = height_m - refined_m;
        assert!(
            (1.10..1.40).contains(&agl_m) && climb_m_s.abs() < 0.25,
            "tick {tick}: agl={agl_m} m, climb={climb_m_s} m/s"
        );
    }

    let mut airborne = super::c172(0.0);
    airborne.sync_ground_versioned_m(0.0, 0).unwrap();
    let before_h = ecef_to_geodetic(airborne.output().position_ecef).height_m;
    airborne.sync_ground_versioned_m(50.0, 1).unwrap();
    let after_h = ecef_to_geodetic(airborne.output().position_ecef).height_m;
    assert!((after_h - before_h).abs() < 1e-6, "{before_h} vs {after_h}");
}

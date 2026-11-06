use std::time::Duration;

use glam::{DQuat, DVec3};
use maplibre::headless::TerrainDrapeMode;
use sim_core::{CameraMount, SimInstant};

use super::{sample_scenery_and_runtime, try_host_gpu, verify_volumetric_patch_export};
use crate::{
    AvionicsMapSet, ChartCameraSpec, ChartFrameOutcome, ConformalMapFrame, InstrumentMapMode,
    InstrumentMapSpec, MapAdapterError, SceneryEndpoints, belief_aviation_chart_style,
    belief_real_svs_style, conformal_stamp_from_vehicle, symmetric_frustum,
    tests::sample_vehicle_at,
};

fn chart_spec(id: &str, size: [u32; 2]) -> InstrumentMapSpec {
    let geojson = sim_scenery::OsmAerodromeDiagram::builtin_all_merged_geojson().expect("osm");
    InstrumentMapSpec {
        id: id.to_owned(),
        style: belief_aviation_chart_style(&SceneryEndpoints::catalog(0, 0), geojson)
            .expect("chart style"),
        mode: InstrumentMapMode::Chart2d {
            min_interval: Duration::from_millis(66),
        },
        size,
    }
}

pub(super) fn svs_spec() -> InstrumentMapSpec {
    InstrumentMapSpec {
        id: "pfd_svs".to_owned(),
        style: belief_real_svs_style(&SceneryEndpoints::catalog(0, 0)).expect("svs style"),
        mode: InstrumentMapMode::Conformal3d {
            initial_reference_altitude_m: Some(600.0),
        },
        size: [120, 90],
    }
}

fn camera() -> ChartCameraSpec {
    ChartCameraSpec {
        center_lat_deg: 39.0,
        center_lon_deg: -75.5,
        zoom: 4.0,
        bearing_deg: 0.0,
        pitch_deg: 0.0,
        surveillance_revision: 1,
        notices_revision: 0,
        symbology_revision: 0,
    }
}

#[test]
fn a_preset_may_own_no_instrument_maps() {
    let Some(gpu) = try_host_gpu() else {
        return;
    };
    let (scenery, _, runtime) = sample_scenery_and_runtime(gpu);
    let mut maps = AvionicsMapSet::new(&runtime, Vec::new()).expect("empty set");
    assert!(maps.is_empty());
    assert_eq!(maps.supply_dem_synchronously(&scenery).expect("dem"), 0);
    maps.set_sun_direction_ecef(DVec3::X).expect("sun");
    assert!(!maps.needs_redraw());
}

#[test]
fn one_chart_surface_is_dirty_gated_and_refuses_conformal_frames() {
    let Some(gpu) = try_host_gpu() else {
        return;
    };
    let (_, _, runtime) = sample_scenery_and_runtime(gpu);
    let mut maps =
        AvionicsMapSet::new(&runtime, vec![chart_spec("mfd_nav_chart", [100, 100])]).expect("set");
    assert_eq!(maps.ids().collect::<Vec<_>>(), ["mfd_nav_chart"]);
    let chart = maps.get_mut("mfd_nav_chart").expect("chart");
    let first = chart
        .render_chart_if_dirty(Duration::from_millis(16), camera(), None)
        .expect("c1");
    assert_eq!(first, ChartFrameOutcome::Rendered);
    let second = chart
        .render_chart_if_dirty(Duration::from_millis(32), camera(), None)
        .expect("c2");
    assert_eq!(second, ChartFrameOutcome::ReusedCachedTexture);
    assert_eq!(chart.render_count(), 1);
    assert_eq!(chart.has_readback_buffer(), Some(false));

    let veh = sample_vehicle_at(39.0, -75.5, 600.0, DQuat::IDENTITY);
    let stamp = conformal_stamp_from_vehicle(
        SimInstant::START,
        &veh,
        &CameraMount::forward_nose(DVec3::ZERO),
        DVec3::ZERO,
        symmetric_frustum(1.0, 100, 100, 0.5, 250_000.0).expect("frustum"),
        0.0,
    )
    .expect("stamp");
    let refused = chart.render_conformal(ConformalMapFrame {
        timestamp: Duration::from_millis(48),
        stamp,
        ownship_msl_m: 600.0,
        runway: None,
        entities: &[],
    });
    assert!(matches!(
        refused,
        Err(MapAdapterError::InstrumentModeMismatch { .. })
    ));
}

#[test]
fn several_surfaces_with_their_own_styles_share_one_runtime() {
    let Some(gpu) = try_host_gpu() else {
        return;
    };
    let (scenery, tile_z0, runtime) = sample_scenery_and_runtime(gpu);
    let duplicate = AvionicsMapSet::new(
        &runtime,
        vec![chart_spec("chart", [64, 64]), chart_spec("chart", [64, 64])],
    );
    assert!(matches!(
        duplicate,
        Err(MapAdapterError::DuplicateInstrumentMap { .. })
    ));
    let mut maps = AvionicsMapSet::new(
        &runtime,
        vec![
            svs_spec(),
            chart_spec("mfd_nav_chart", [100, 100]),
            chart_spec("airport_surface", [80, 60]),
        ],
    )
    .expect("three surfaces");
    assert_eq!(maps.len(), 3);
    assert_eq!(maps.supply_dem_synchronously(&scenery).expect("dem"), 3);
    let svs = maps.get_mut("pfd_svs").expect("svs");
    assert!(matches!(
        svs.terrain_drape_mode(),
        TerrainDrapeMode::ProceduralRelief(_)
    ));
    let veh = sample_vehicle_at(39.0, -75.5, 600.0, DQuat::IDENTITY);
    let stamp = conformal_stamp_from_vehicle(
        SimInstant::START,
        &veh,
        &CameraMount::forward_nose(DVec3::ZERO),
        DVec3::ZERO,
        symmetric_frustum(1.0, 120, 90, 0.5, 250_000.0).expect("frustum"),
        0.0,
    )
    .expect("stamp");
    svs.render_conformal(ConformalMapFrame {
        timestamp: Duration::from_millis(16),
        stamp,
        ownship_msl_m: 600.0,
        runway: None,
        entities: &[],
    })
    .expect("svs frame");
    assert!(svs.color_view().is_some());
    for id in ["mfd_nav_chart", "airport_surface"] {
        let chart = maps.get_mut(id).expect("chart");
        chart
            .render_chart_if_dirty(Duration::from_millis(16), camera(), None)
            .expect("chart frame");
        assert!(chart.color_view().is_some());
    }
    let surface = maps.get("airport_surface").expect("surface");
    assert_eq!(surface.read_rgba().expect("rgba").len(), 80 * 60 * 4);
    verify_volumetric_patch_export(maps.get("pfd_svs").expect("svs"), tile_z0);
}

/// The PFD's synthetic vision sky is bright aviation blue in level flight even when the
/// host hands it a night-side sun: the SVS sky never depends on the sun.
#[test]
fn svs_sky_stays_daylight_blue_on_the_night_side() {
    let Some(gpu) = try_host_gpu() else {
        return;
    };
    let (scenery, _, runtime) = sample_scenery_and_runtime(gpu);
    let mut maps = AvionicsMapSet::new(&runtime, vec![svs_spec()]).expect("svs set");
    let _ = maps.supply_dem_synchronously(&scenery).expect("dem");
    let veh = sample_vehicle_at(39.0, -75.5, 450.0, DQuat::IDENTITY);
    let midnight_sun = -veh.position_ecef.normalize();
    maps.set_sun_direction_ecef(midnight_sun)
        .expect("night sun");
    let svs = maps.get_mut("pfd_svs").expect("svs");
    let [w, h] = svs.size();
    let stamp = conformal_stamp_from_vehicle(
        SimInstant::START,
        &veh,
        &CameraMount::forward_nose(DVec3::ZERO),
        DVec3::ZERO,
        symmetric_frustum(1.0, w, h, 0.5, 250_000.0).expect("frustum"),
        0.0,
    )
    .expect("stamp");
    svs.render_conformal(ConformalMapFrame {
        timestamp: Duration::from_millis(16),
        stamp,
        ownship_msl_m: 450.0,
        runway: None,
        entities: &[],
    })
    .expect("svs frame");
    let rgba = svs.read_rgba().expect("rgba");
    let row_bytes = w as usize * 4;
    let sky_rows = h as usize / 4;
    let mut checked = 0_usize;
    for pixel in rgba[..sky_rows * row_bytes].as_chunks::<4>().0 {
        let [r, _g, b, _a] = *pixel;
        assert!(b > 80 && b > r, "dark or non-blue sky pixel {pixel:?}");
        checked += 1;
    }
    assert_eq!(checked, sky_rows * w as usize);
}

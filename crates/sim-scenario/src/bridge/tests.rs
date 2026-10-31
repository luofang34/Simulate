use std::io::Write;

use glam::{DMat3, DQuat, DVec3};
use sim_core::{
    Block, CAMERA_DEFAULT_PERIOD, CameraMount, CameraSensorBlock, CameraSensorConfig,
    CameraSensorInput, EnuAnchor, VehicleState,
};
use sim_geo::{
    BodyEllipsoid, Geodetic, InertialTransform, SensorIntrinsics, UtcTime, ecef_to_geodetic,
    nav_star_by_octal_id,
};
use sim_render::{HeadlessGpu, reversed_z_range_m};
use sim_scenery::{
    CatalogScenery, ElevationSource, LunarTranquilityScenery, SyntheticCorridorSpec, WEST_CRATER,
};

use super::*;
use crate::{CorridorRun, FlightDynamics, ScenarioProfile, ScenarioWorld};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;
const NEAR_M: f64 = 1.0;
const INTRINSICS: SensorIntrinsics = SensorIntrinsics {
    width: WIDTH,
    height: HEIGHT,
    fx: 300.0,
    fy: 280.0,
    cx: 159.5,
    cy: 119.5,
};

/// A machine with no GPU cannot run these. A skip is reported on the raw
/// stream, which the harness does not capture, so it cannot pass unnoticed.
fn gpu() -> Option<HeadlessGpu> {
    match pollster::block_on(HeadlessGpu::new()) {
        Ok(gpu) => Some(gpu),
        Err(err) => {
            writeln!(std::io::stderr(), "GPU test skipped: {err}").ok();
            None
        }
    }
}

/// A belly camera's capture of `vehicle`, taken by the real sensor block.
fn belly_capture(body: &BodyEllipsoid, vehicle: VehicleState) -> CameraFrameCapture {
    let position = body.fixed_to_geodetic(vehicle.position_ecef);
    let mut camera = CameraSensorBlock::new(CameraSensorConfig {
        mount: CameraMount::nadir_belly(DVec3::ZERO),
        intrinsics: INTRINSICS,
        anchor: EnuAnchor::new(body, &position),
        period: CAMERA_DEFAULT_PERIOD,
    })
    .unwrap();
    camera
        .step(&CameraSensorInput {
            vehicle,
            cg_body_m: DVec3::ZERO,
        })
        .unwrap();
    camera.output().unwrap()
}

fn pass_for(gpu: &HeadlessGpu, world: ScenarioWorld) -> SensorAperturePass {
    let scene = world.scene().unwrap();
    pollster::block_on(SensorAperturePass::new(
        &gpu.device,
        &gpu.queue,
        &scene.mesh,
        scene.atlas.as_ref(),
    ))
    .unwrap()
}

fn exposure(world: ScenarioWorld, waveband: ApertureWaveband) -> CaptureExposure {
    CaptureExposure {
        lighting: SceneLighting {
            sun_direction_ecef: world.sun_direction_ecef(),
        },
        waveband,
        occlusion: OcclusionOverlayConfig::default(),
        near_m: NEAR_M,
    }
}

/// How many distinct red levels a frame holds.
fn levels(image: &sim_render::ApertureImage) -> usize {
    let mut seen = [false; 256];
    for [red, ..] in image.rgba8.as_chunks::<4>().0 {
        seen[usize::from(*red)] = true;
    }
    seen.iter().filter(|level| **level).count()
}

#[test]
fn the_frame_carries_the_capture_pose_unchanged() {
    let vehicle = CorridorRun::new(
        ScenarioProfile::CorridorNominalCruise,
        FlightDynamics::Jsbsim,
    )
    .unwrap()
    .vehicle();
    let capture = belly_capture(&BodyEllipsoid::WGS84, vehicle);
    let frame = aperture_frame(
        &capture,
        &exposure(
            ScenarioWorld::DelawareCorridor,
            ApertureWaveband::VisibleRgb {
                sky_linear_rgb: [0.4, 0.6, 0.9],
            },
        ),
    );
    assert_eq!(frame.pose.position_ecef, capture.pose.position_ecef);
    assert_eq!(frame.pose.orientation, capture.pose.orientation);
}

#[test]
fn a_corridor_capture_renders_the_draped_terrain_at_its_true_range() {
    let Some(gpu) = gpu() else { return };
    let mut run = CorridorRun::new(
        ScenarioProfile::CorridorNominalCruise,
        FlightDynamics::Jsbsim,
    )
    .unwrap();
    for _ in 0..600 {
        run.step().unwrap();
    }
    let capture = belly_capture(&BodyEllipsoid::WGS84, run.vehicle());
    let world = ScenarioWorld::DelawareCorridor;
    let mut pass = pass_for(&gpu, world);
    let waveband = ApertureWaveband::VisibleRgb {
        sky_linear_rgb: [0.4, 0.6, 0.9],
    };
    let target = render_capture(
        &gpu.device,
        &gpu.queue,
        &mut pass,
        &capture,
        &exposure(world, waveband),
    )
    .unwrap();
    let image = target.read_blocking(&gpu.device, &gpu.queue).unwrap();
    gpu.check_device().unwrap();

    assert_eq!((image.width, image.height), (WIDTH, HEIGHT));
    assert!(levels(&image) > 20, "the ortho drape: {}", levels(&image));

    // The optical axis strikes the ground the catalog says is there.
    let axis = capture.pose.orientation * DVec3::NEG_Z;
    let catalog = CatalogScenery::synthetic_corridor(&SyntheticCorridorSpec::delaware()).unwrap();
    let eye = capture.pose.position_ecef;
    let mut range = ecef_to_geodetic(eye).height_m;
    for _ in 0..4 {
        let ground = ecef_to_geodetic(eye + axis * range);
        let surface = catalog.ground_height_m(&ground).unwrap().unwrap();
        range += ground.height_m - surface;
    }
    let depth = image.depth_at(WIDTH / 2, HEIGHT / 2).unwrap();
    let rendered = reversed_z_range_m(depth, NEAR_M).unwrap();
    assert!((rendered - range).abs() < 8.0, "{rendered} vs {range}");
}

#[test]
fn a_lunar_capture_renders_west_crater_in_the_thermal_band() {
    let Some(gpu) = gpu() else { return };
    let scenery = LunarTranquilityScenery::apollo_11_landing();
    let body = BodyEllipsoid::MOON_IAU;
    let mut over = scenery.at_local(WEST_CRATER.east_m, WEST_CRATER.north_m);
    over.height_m = 600.0;
    let vehicle = VehicleState::level_flight(&over, 0.0, 0.0);
    // `level_flight` reads WGS84, so put the vehicle on the Moon by hand.
    let vehicle = VehicleState {
        position_ecef: body.geodetic_to_fixed(&over),
        ..vehicle
    };
    let capture = belly_capture(&body, vehicle);
    let world = ScenarioWorld::LunarTranquility;
    let mut pass = pass_for(&gpu, world);
    let waveband = ApertureWaveband::ThermalLwir(sim_render::ThermalScale {
        black_k: 90.0,
        white_k: 380.0,
        nedt_k: 0.0,
        noise_seed: 0,
    });
    let target = render_capture(
        &gpu.device,
        &gpu.queue,
        &mut pass,
        &capture,
        &exposure(world, waveband),
    )
    .unwrap();
    let image = target.read_blocking(&gpu.device, &gpu.queue).unwrap();
    gpu.check_device().unwrap();
    assert!(levels(&image) > 20, "warm sunlit slopes and cold shadow");
}

#[test]
fn a_capture_aimed_at_a_star_renders_it_against_black_space() {
    let Some(gpu) = gpu() else { return };
    let inertial = InertialTransform::moon_at(UtcTime::from_unix_seconds(-14_182_200.0));
    let forward = inertial.j2000_to_fixed(nav_star_by_octal_id(0o17).unwrap().direction_j2000);
    let right = forward.cross(DVec3::Z).normalize();
    let eye = forward * 3_000_000.0;
    // Body +x at the star, so the nose camera looks straight at it.
    let vehicle = VehicleState {
        position_ecef: eye,
        velocity_ecef: DVec3::ZERO,
        attitude: DQuat::from_mat3(&DMat3::from_cols(forward, right, forward.cross(right))),
        angular_rate_body: DVec3::ZERO,
    };
    let origin = Geodetic::from_degrees(0.0, 0.0, 0.0);
    let body = BodyEllipsoid::MOON_IAU;
    let mut camera = CameraSensorBlock::new(CameraSensorConfig {
        mount: CameraMount::forward_nose(DVec3::ZERO),
        intrinsics: INTRINSICS,
        anchor: EnuAnchor::new(&body, &origin),
        period: CAMERA_DEFAULT_PERIOD,
    })
    .unwrap();
    camera
        .step(&CameraSensorInput {
            vehicle,
            cg_body_m: DVec3::ZERO,
        })
        .unwrap();
    let capture = camera.output().unwrap();

    let world = ScenarioWorld::LunarTranquility;
    let mut pass = pass_for(&gpu, world);
    let waveband = ApertureWaveband::StarfieldOptical {
        inertial,
        reticle: None,
    };
    let target = render_capture(
        &gpu.device,
        &gpu.queue,
        &mut pass,
        &capture,
        &exposure(world, waveband),
    )
    .unwrap();
    let image = target.read_blocking(&gpu.device, &gpu.queue).unwrap();
    gpu.check_device().unwrap();

    let peak = image.pixel(WIDTH / 2, HEIGHT / 2).unwrap();
    assert!(peak[0] > 30, "the star: {peak:?}");
    assert_eq!(image.pixel(5, 5).unwrap()[..3], [0, 0, 0], "black space");
}

#[test]
fn unusable_optics_are_refused_before_anything_renders() {
    let Some(gpu) = gpu() else { return };
    let vehicle = CorridorRun::new(
        ScenarioProfile::CorridorNominalCruise,
        FlightDynamics::Kinematic,
    )
    .unwrap()
    .vehicle();
    let capture = belly_capture(&BodyEllipsoid::WGS84, vehicle);
    let world = ScenarioWorld::EarthGlobe;
    let mut pass = pass_for(&gpu, world);
    let mut bad = exposure(
        world,
        ApertureWaveband::VisibleRgb {
            sky_linear_rgb: [0.0; 3],
        },
    );
    bad.near_m = 0.0;
    assert!(matches!(
        render_capture(&gpu.device, &gpu.queue, &mut pass, &capture, &bad),
        Err(ScenarioError::Render(_))
    ));
}

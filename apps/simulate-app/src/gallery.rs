//! Headless `1280×720` composite screenshot gallery across airports, traffic-pattern
//! stages, cockpit instruments, 2D OSM airport diagrams, and 3D virtual cameras.

use std::path::Path;
use std::time::Duration;

use image::RgbaImage;
use sim_core::ControlInputs;
use sim_geo::ecef_to_geodetic;
use sim_maplibre::HostGpu;
use sim_render::{CompositorPass, CompositorTarget, HeadlessGpu};
use sim_scenario::{OperatorViewportMode, PatternLeg, ScenarioProfile, SimTimeScale};
use sim_scenery::AerodromeId;
use tracing::info;

use crate::camera_view::camera_from_virtual_spec;
use crate::cockpit_rig::CockpitRig;
use crate::error::AppSimError;
use crate::flight::Flight;

const GALLERY_WIDTH_PX: u32 = 1280;
const GALLERY_HEIGHT_PX: u32 = 720;
const BYTES_PER_PIXEL: u32 = 4;

enum StageTarget {
    Pattern {
        profile: ScenarioProfile,
        leg: PatternLeg,
        extra_s: f64,
    },
    FreeAirport {
        aerodrome: AerodromeId,
        advance_s: f64,
    },
}

struct GalleryShot {
    filename: &'static str,
    caption: &'static str,
    stage: StageTarget,
    viewport: OperatorViewportMode,
    virtual_camera_index: usize,
    show_instruments: bool,
    show_hud: bool,
}

/// Renders the multi-airport, multi-stage screenshot gallery into `out_dir` and returns
/// the number of PNG images written.
pub fn render_gallery(out_dir: &Path) -> Result<usize, AppSimError> {
    std::fs::create_dir_all(out_dir).map_err(|err| AppSimError::Screenshot {
        detail: format!("creating {}: {err}", out_dir.display()),
    })?;
    let gpu = pollster::block_on(HeadlessGpu::new())?;
    let host_gpu = HostGpu {
        instance: gpu.instance.clone(),
        adapter: gpu.adapter.clone(),
        device: gpu.device.clone(),
        queue: gpu.queue.clone(),
    };
    let size = [GALLERY_WIDTH_PX, GALLERY_HEIGHT_PX];
    let enable_maps = gpu.backend != wgpu::Backend::Gl;
    let mut rig = CockpitRig::new_with_gpu(host_gpu, size, enable_maps)?;
    let compositor = pollster::block_on(CompositorPass::new(
        &gpu.device,
        wgpu::TextureFormat::Rgba8UnormSrgb,
    ))?;
    let target = create_target_texture(&gpu.device, size);
    let target_view = target.create_view(&wgpu::TextureViewDescriptor::default());

    let shots = gallery_shots();
    let mut written = 0_usize;
    for shot in &shots {
        let mut flight = Flight::new()?;
        advance_to_stage(&mut flight, &mut rig, &shot.stage)?;
        rig.supervisor_mut().set_viewport_mode(shot.viewport);
        rig.supervisor_mut()
            .set_virtual_camera_index(shot.virtual_camera_index);
        rig.set_show_instruments(shot.show_instruments);
        rig.set_show_conformal_hud(shot.show_hud);
        rig.invalidate_chart();

        settle_and_composite(&gpu, &compositor, &target_view, &mut rig, &flight, size)?;
        let rgba = readback_target_rgba(&gpu.device, &gpu.queue, &target)?;
        save_png(&out_dir.join(shot.filename), size, rgba)?;
        log_shot_summary(shot, &flight);
        written = written.wrapping_add(1);
    }
    gpu.check_device()?;
    Ok(written)
}

fn gallery_shots() -> Vec<GalleryShot> {
    let mut shots = gallery_shots_47n();
    shots.extend(gallery_shots_kewr_and_kdov());
    shots
}

fn gallery_shots_47n() -> Vec<GalleryShot> {
    vec![
        GalleryShot {
            filename: "01_47n_upwind_cockpit.png",
            caption: "47N Rwy 07 — Upwind Climbout (Cockpit + Conformal HUD + SVS/Chart/HSI)",
            stage: StageTarget::Pattern {
                profile: ScenarioProfile::Airport47NUncontrolledPattern,
                leg: PatternLeg::Upwind,
                extra_s: 1.0,
            },
            viewport: OperatorViewportMode::Cockpit,
            virtual_camera_index: 0,
            show_instruments: true,
            show_hud: true,
        },
        GalleryShot {
            filename: "02_47n_downwind_left_window.png",
            caption: "47N Rwy 07 — Midfield Left Downwind (Left Pattern Window Cam)",
            stage: StageTarget::Pattern {
                profile: ScenarioProfile::Airport47NUncontrolledPattern,
                leg: PatternLeg::Downwind,
                extra_s: 16.0,
            },
            viewport: OperatorViewportMode::SensorAperture,
            virtual_camera_index: 2,
            show_instruments: true,
            show_hud: false,
        },
        GalleryShot {
            filename: "03_47n_short_final_cockpit.png",
            caption: "47N Rwy 07 — Stabilized 3 deg Short Final, Flaps 30 (Cockpit + HUD)",
            stage: StageTarget::Pattern {
                profile: ScenarioProfile::Airport47NUncontrolledPattern,
                leg: PatternLeg::Final,
                extra_s: 26.0,
            },
            viewport: OperatorViewportMode::Cockpit,
            virtual_camera_index: 0,
            show_instruments: true,
            show_hud: true,
        },
        GalleryShot {
            filename: "03b_47n_surface_diagram_chart.png",
            caption: "47N Central Jersey (CTAF) — Full 2D OSM Airport Surface Diagram (ChartMap)",
            stage: StageTarget::Pattern {
                profile: ScenarioProfile::Airport47NUncontrolledPattern,
                leg: PatternLeg::Upwind,
                extra_s: 1.0,
            },
            viewport: OperatorViewportMode::ChartMap,
            virtual_camera_index: 0,
            show_instruments: false,
            show_hud: false,
        },
    ]
}

fn gallery_shots_kewr_and_kdov() -> Vec<GalleryShot> {
    vec![
        GalleryShot {
            filename: "04_kewr_surface_diagram_chart.png",
            caption: "KEWR Newark Class B — Full 2D OSM Airport Surface Diagram (ChartMap)",
            stage: StageTarget::Pattern {
                profile: ScenarioProfile::AirportKewrToweredPattern,
                leg: PatternLeg::Upwind,
                extra_s: 2.0,
            },
            viewport: OperatorViewportMode::ChartMap,
            virtual_camera_index: 0,
            show_instruments: false,
            show_hud: false,
        },
        GalleryShot {
            filename: "05_kewr_final_tower_ptz.png",
            caption: "KEWR Rwy 22L — Short Final from Newark Control Tower (Tower PTZ Auto-Track)",
            stage: StageTarget::Pattern {
                profile: ScenarioProfile::AirportKewrToweredPattern,
                leg: PatternLeg::Final,
                extra_s: 31.0,
            },
            viewport: OperatorViewportMode::SensorAperture,
            virtual_camera_index: 3,
            show_instruments: true,
            show_hud: false,
        },
        GalleryShot {
            filename: "06_kewr_base_to_final_chase.png",
            caption: "KEWR Rwy 22L — Base-to-Final Turn (External Chase + 6-DoF Airframe + Dock)",
            stage: StageTarget::Pattern {
                profile: ScenarioProfile::AirportKewrToweredPattern,
                leg: PatternLeg::Final,
                extra_s: 3.0,
            },
            viewport: OperatorViewportMode::Chase,
            virtual_camera_index: 0,
            show_instruments: true,
            show_hud: false,
        },
        GalleryShot {
            filename: "07_kewr_short_final_cockpit.png",
            caption: "KEWR Rwy 22L — Stabilized Short Final, Flaps 30 (Cockpit + HUD + Dock)",
            stage: StageTarget::Pattern {
                profile: ScenarioProfile::AirportKewrToweredPattern,
                leg: PatternLeg::Final,
                extra_s: 25.0,
            },
            viewport: OperatorViewportMode::Cockpit,
            virtual_camera_index: 0,
            show_instruments: true,
            show_hud: true,
        },
        GalleryShot {
            filename: "08_kdov_overhead_belly_nadir.png",
            caption: "KDOV Dover — Overhead Runway Crossing (Belly Nadir Optical Sensor + Dock)",
            stage: StageTarget::FreeAirport {
                aerodrome: AerodromeId::DoverKdov,
                advance_s: 12.0,
            },
            viewport: OperatorViewportMode::SensorAperture,
            virtual_camera_index: 1,
            show_instruments: true,
            show_hud: false,
        },
    ]
}

fn advance_to_stage(
    flight: &mut Flight,
    rig: &mut CockpitRig,
    stage: &StageTarget,
) -> Result<(), AppSimError> {
    match *stage {
        StageTarget::Pattern {
            profile,
            leg,
            extra_s,
        } => {
            flight.select(Some(profile))?;
            let mut safety_steps = 0_u32;
            while flight.pattern_leg() != Some(leg) && safety_steps < 3_600 {
                flight.advance_by(0.10, ControlInputs::default(), SimTimeScale::RealTime)?;
                safety_steps = safety_steps.wrapping_add(1);
            }
            if extra_s > 0.0 {
                let ticks = (extra_s / 0.10).round() as u32;
                for _ in 0..ticks {
                    flight.advance_by(0.10, ControlInputs::default(), SimTimeScale::RealTime)?;
                }
            }
        }
        StageTarget::FreeAirport {
            aerodrome,
            advance_s,
        } => {
            rig.supervisor_mut().set_free_aerodrome(aerodrome);
            if let Some(diag) = rig.supervisor().diagram(aerodrome).cloned() {
                flight.respawn_at_diagram(&diag)?;
            }
            let cruise = ControlInputs {
                throttle: 0.72,
                ..ControlInputs::default()
            };
            let ticks = (advance_s / 0.05).round() as u32;
            for _ in 0..ticks {
                flight.advance_by(0.05, cruise, SimTimeScale::RealTime)?;
            }
        }
    }
    Ok(())
}

fn settle_and_composite(
    gpu: &HeadlessGpu,
    compositor: &CompositorPass,
    target_view: &wgpu::TextureView,
    rig: &mut CockpitRig,
    flight: &Flight,
    size: [u32; 2],
) -> Result<(), AppSimError> {
    let mode = rig.supervisor().viewport_mode();
    let mut camera = flight.camera(mode)?;
    if mode == OperatorViewportMode::SensorAperture
        && let Some(vcam) = camera_from_virtual_spec(
            &rig.supervisor().active_virtual_camera(flight.profile()),
            &flight.vehicle_state(),
        )
    {
        camera = vcam;
    }
    let world = flight.world();
    let sun = world.sun_direction_ecef();

    for pass in 0..36_u32 {
        rig.invalidate_chart();
        let _need_globe = rig.prepare_frame(&gpu.queue, size, flight, &camera, sun)?;
        gpu.device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|err| AppSimError::Screenshot {
                detail: err.to_string(),
            })?;
        if pass < 10 || rig.maps_need_redraw() {
            std::thread::sleep(Duration::from_millis(18));
        } else if pass >= 12 {
            break;
        }
    }

    let quads = rig.overlay_quads(world);
    let mut encoder = gpu
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("gallery composite frame"),
        });
    compositor.encode(
        &gpu.device,
        &gpu.queue,
        &mut encoder,
        CompositorTarget {
            view: target_view,
            clear: Some(wgpu::Color::BLACK),
            size_px: size,
        },
        &quads,
    );
    gpu.queue.submit([encoder.finish()]);
    gpu.device
        .poll(wgpu::PollType::wait_indefinitely())
        .map_err(|err| AppSimError::Screenshot {
            detail: err.to_string(),
        })?;
    Ok(())
}

fn create_target_texture(device: &wgpu::Device, size: [u32; 2]) -> wgpu::Texture {
    let [width, height] = size;
    device.create_texture(&wgpu::TextureDescriptor {
        label: Some("gallery srgb target"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    })
}

fn readback_target_rgba(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    texture: &wgpu::Texture,
) -> Result<Vec<u8>, AppSimError> {
    let ext = texture.size();
    let tight_row = ext.width.saturating_mul(BYTES_PER_PIXEL);
    let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let padded_row = tight_row.div_ceil(align).saturating_mul(align);
    let staging = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("gallery staging"),
        size: u64::from(padded_row).saturating_mul(u64::from(ext.height)),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("gallery readback copy"),
    });
    enc.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &staging,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded_row),
                rows_per_image: None,
            },
        },
        ext,
    );
    queue.submit([enc.finish()]);
    let raw = map_buffer_blocking(device, &staging)?;
    let mut out = Vec::with_capacity((tight_row as usize).saturating_mul(ext.height as usize));
    for row in raw.chunks(padded_row as usize).take(ext.height as usize) {
        out.extend_from_slice(row.get(..tight_row as usize).unwrap_or(row));
    }
    Ok(out)
}

fn map_buffer_blocking(
    device: &wgpu::Device,
    staging: &wgpu::Buffer,
) -> Result<Vec<u8>, AppSimError> {
    let (tx, rx) = std::sync::mpsc::channel();
    staging.map_async(wgpu::MapMode::Read, .., move |res| {
        tx.send(res).ok();
    });
    device
        .poll(wgpu::PollType::wait_indefinitely())
        .map_err(|err| AppSimError::Screenshot {
            detail: err.to_string(),
        })?;
    rx.recv()
        .map_err(|err| AppSimError::Screenshot {
            detail: err.to_string(),
        })?
        .map_err(|err| AppSimError::Screenshot {
            detail: err.to_string(),
        })?;
    let bytes = staging
        .get_mapped_range(..)
        .map_err(|err| AppSimError::Screenshot {
            detail: err.to_string(),
        })?
        .to_vec();
    staging.unmap();
    Ok(bytes)
}

fn save_png(path: &Path, size: [u32; 2], rgba: Vec<u8>) -> Result<(), AppSimError> {
    let [width, height] = size;
    let img = RgbaImage::from_raw(width, height, rgba).ok_or_else(|| AppSimError::Screenshot {
        detail: format!("invalid RGBA buffer size for {}", path.display()),
    })?;
    img.save(path).map_err(|err| AppSimError::Screenshot {
        detail: format!("saving {}: {err}", path.display()),
    })
}

fn log_shot_summary(shot: &GalleryShot, flight: &Flight) {
    let state = flight.vehicle_state();
    let geo = ecef_to_geodetic(state.position_ecef);
    let kts = state.velocity_ecef.length() * 1.943_844;
    info!(
        file = shot.filename,
        caption = shot.caption,
        lat = format_args!("{:.4}", geo.lat_rad.to_degrees()),
        lon = format_args!("{:.4}", geo.lon_rad.to_degrees()),
        alt_msl_ft = format_args!("{:.0}", geo.height_m * 3.280_84),
        gs_kt = format_args!("{kts:.0}"),
        leg = ?flight.pattern_leg(),
        "saved screenshot"
    );
}

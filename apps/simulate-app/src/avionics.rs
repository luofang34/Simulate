use glam::DVec3;
use sim_apollo::CockpitInstrumentSnapshot;
use sim_atmosphere::{AtmosphereField, StandardAtmosphere};
use sim_core::{SimInstant, SourceValidity, StampedReading, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, InertialTransform, UtcTime, ecef_to_geodetic};
use sim_indicate::{
    AirDataReading, AirDataSample, AltitudeSource, BackgroundMode, DisplayAxes, FeedConfig,
    FlightSample, FrameId, FramebufferDims, MonitoredFrame, PANEL_HEIGHT_PX, PANEL_RGBA_BYTES,
    PANEL_SCENE_BYTES, PANEL_WIDTH_PX, PanelBuffers, PfdConfig, SourceMonitor, StateFeed,
    SvsViewport, VOLUME_SCENE_BYTES, VSpeeds, ViewPose3D, VolumeInstrument, emit_volume_scene,
    render_hsi, render_pfd,
};
use sim_maplibre::{TerrainPatchSnapshot, TerrainVolumeRequest, emit_terrain_volume_with_patch};
use sim_render::RgbaPanelTexture;
use sim_scenario::{OperatorViewportMode, ScenarioProfile};

use crate::error::AppSimError;
use crate::supervisor_rig::default_delaware_spawn;

const APOLLO_LM_EPOCH_UNIX_S: f64 = -14_182_200.0;
const APOLLO_CM_EPOCH_UNIX_S: f64 = -13_854_300.0;
const EARTH_EPOCH_UNIX_S: f64 = 1_782_864_000.0;
const STANDARD_QNH_HPA: f64 = 1_013.25;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FeedDomain {
    EarthAircraft,
    ApolloLunarModule,
    ApolloCommandModule,
}

impl FeedDomain {
    const fn for_profile(profile: Option<ScenarioProfile>) -> Self {
        match profile {
            Some(ScenarioProfile::Apollo11LunarLanding) => Self::ApolloLunarModule,
            Some(ScenarioProfile::Apollo11CapsuleReentry) => Self::ApolloCommandModule,
            Some(_) | None => Self::EarthAircraft,
        }
    }

    fn feed_config(self) -> FeedConfig {
        match self {
            Self::EarthAircraft => FeedConfig {
                body: BodyEllipsoid::WGS84,
                axes: DisplayAxes::AEROSPACE,
                origin: default_delaware_spawn(),
                origin_id: 1,
                altitude: AltitudeSource::Barometric,
            },
            Self::ApolloLunarModule => FeedConfig {
                body: BodyEllipsoid::MOON_IAU,
                axes: DisplayAxes::apollo_lm(),
                origin: Geodetic::from_degrees(0.674_16, 23.473_15, 0.0),
                origin_id: 11,
                altitude: AltitudeSource::OriginRelative,
            },
            Self::ApolloCommandModule => FeedConfig {
                body: BodyEllipsoid::WGS84,
                axes: DisplayAxes::AEROSPACE,
                origin: Geodetic::from_degrees(-3.2, 171.9, 0.0),
                origin_id: 12,
                altitude: AltitudeSource::OriginRelative,
            },
        }
    }
}

/// Inputs required to rasterize one cockpit avionics frame.
pub struct AvionicsFrameInput<'a> {
    /// Active scripted scenario, or `None` in free flight.
    pub profile: Option<ScenarioProfile>,
    /// Current simulation clock instant.
    pub timestamp: SimInstant,
    /// True or navigation-estimated vehicle state.
    pub vehicle: VehicleState,
    /// Cockpit instrument health/attitude snapshot.
    pub cockpit: CockpitInstrumentSnapshot,
    /// Body FRD specific force (`m/s²`), if modeled.
    pub specific_force_frd_m_s2: Option<DVec3>,
    /// Whether `BeliefAvionicsMap` rendered a live `ConformalSvs3D` underlay this frame.
    pub svs_underlay_active: bool,
    /// Whether the conformal HUD overlay is active on the main viewport.
    pub render_hud: bool,
    /// Active operator viewport mode (triggers 3D `VolumeInstrument` emission on `VolumetricScene`).
    pub viewport_mode: OperatorViewportMode,
    /// Optional stitched `129×129` DEM patch from `BeliefAvionicsMap` for `terrain-volume`.
    pub terrain_patch: Option<&'a TerrainPatchSnapshot>,
}

/// Owns `Indicate` state feeds, source monitors, CPU raster buffers, and persistent GPU panel textures.
pub struct AvionicsRig {
    domain: FeedDomain,
    feed: StateFeed,
    monitor: SourceMonitor,
    scene_buf: Vec<u8>,
    rgba_buf: Vec<u8>,
    volume_buf: Vec<u8>,
    volume_pose: ViewPose3D,
    last_volume_bytes: usize,
    render_gen: u32,
    pfd_texture: RgbaPanelTexture,
    hsi_texture: RgbaPanelTexture,
    hud_texture: RgbaPanelTexture,
}

impl AvionicsRig {
    /// Allocates reusable `Indicate` scene/pixel buffers and `480 × 360` sRGB GPU textures.
    pub fn new(device: &wgpu::Device) -> Result<Self, AppSimError> {
        let domain = FeedDomain::EarthAircraft;
        Ok(Self {
            domain,
            feed: StateFeed::new(domain.feed_config()),
            monitor: SourceMonitor::new(),
            scene_buf: vec![0_u8; PANEL_SCENE_BYTES],
            rgba_buf: vec![0_u8; PANEL_RGBA_BYTES],
            volume_buf: vec![0_u8; VOLUME_SCENE_BYTES],
            volume_pose: ViewPose3D::default(),
            last_volume_bytes: 0,
            render_gen: 0,
            pfd_texture: RgbaPanelTexture::new(
                device,
                "indicate pfd",
                PANEL_WIDTH_PX,
                PANEL_HEIGHT_PX,
            )?,
            hsi_texture: RgbaPanelTexture::new(
                device,
                "indicate hsi",
                PANEL_WIDTH_PX,
                PANEL_HEIGHT_PX,
            )?,
            hud_texture: RgbaPanelTexture::new(
                device,
                "indicate hud",
                PANEL_WIDTH_PX,
                PANEL_HEIGHT_PX,
            )?,
        })
    }

    /// GPU texture holding the latest PFD frame (`BackgroundMode::Svs` or `BackgroundMode::Horizon`).
    #[must_use]
    pub const fn pfd_texture(&self) -> &RgbaPanelTexture {
        &self.pfd_texture
    }

    /// GPU texture holding the latest HSI frame.
    #[must_use]
    pub const fn hsi_texture(&self) -> &RgbaPanelTexture {
        &self.hsi_texture
    }

    /// GPU texture holding the latest transparent conformal HUD frame (`BackgroundMode::None`).
    #[must_use]
    pub const fn hud_texture(&self) -> &RgbaPanelTexture {
        &self.hud_texture
    }

    /// Byte count of the most recently emitted 3D volumetric instrument scene (`VOL-01`).
    #[cfg(test)]
    #[must_use]
    pub const fn last_volume_scene_bytes(&self) -> usize {
        self.last_volume_bytes
    }

    /// Steps `Indicate` state resolution, rasterizes PFD + HSI (+ optional HUD), uploads GPU textures,
    /// and emits the 3D volumetric instrument IR when in `VolumetricScene`.
    pub fn update(
        &mut self,
        queue: &wgpu::Queue,
        input: &AvionicsFrameInput<'_>,
    ) -> Result<MonitoredFrame, AppSimError> {
        let next_domain = FeedDomain::for_profile(input.profile);
        if next_domain != self.domain {
            self.domain = next_domain;
            self.feed = StateFeed::new(next_domain.feed_config());
            self.monitor = SourceMonitor::new();
        }

        let sample = build_flight_sample(self.domain, input);
        let monitored = self.monitor.step(&mut self.feed, &sample)?;
        self.render_gen = self.render_gen.wrapping_add(1);
        let frame_id = FrameId {
            frame_generation: monitored.state.snapshot.generation,
            render_generation: self.render_gen,
        };

        let pfd_cfg = pfd_config_for(self.domain, input.svs_underlay_active);
        self.rasterize_and_upload_pfd(queue, &monitored, &pfd_cfg, frame_id)?;
        self.rasterize_and_upload_hsi(queue, &monitored, frame_id)?;
        if input.render_hud {
            self.rasterize_and_upload_hud(queue, &monitored, frame_id)?;
        }
        if input.viewport_mode == OperatorViewportMode::VolumetricScene {
            self.emit_active_volume(&monitored, input.terrain_patch)?;
        }
        Ok(monitored)
    }

    fn rasterize_and_upload_pfd(
        &mut self,
        queue: &wgpu::Queue,
        monitored: &MonitoredFrame,
        config: &PfdConfig,
        frame_id: FrameId,
    ) -> Result<(), AppSimError> {
        let dims = FramebufferDims::tight(PANEL_WIDTH_PX, PANEL_HEIGHT_PX);
        let _stats = render_pfd(
            &monitored.panel,
            config,
            Some(&monitored.alerts),
            frame_id,
            &mut PanelBuffers {
                scene: &mut self.scene_buf,
                pixels: &mut self.rgba_buf,
                dims,
            },
        )?;
        self.pfd_texture.upload(queue, &self.rgba_buf)?;
        Ok(())
    }

    fn rasterize_and_upload_hsi(
        &mut self,
        queue: &wgpu::Queue,
        monitored: &MonitoredFrame,
        frame_id: FrameId,
    ) -> Result<(), AppSimError> {
        let dims = FramebufferDims::tight(PANEL_WIDTH_PX, PANEL_HEIGHT_PX);
        let _stats = render_hsi(
            &monitored.panel,
            Some(&monitored.alerts),
            frame_id,
            &mut PanelBuffers {
                scene: &mut self.scene_buf,
                pixels: &mut self.rgba_buf,
                dims,
            },
        )?;
        self.hsi_texture.upload(queue, &self.rgba_buf)?;
        Ok(())
    }

    fn rasterize_and_upload_hud(
        &mut self,
        queue: &wgpu::Queue,
        monitored: &MonitoredFrame,
        frame_id: FrameId,
    ) -> Result<(), AppSimError> {
        let dims = FramebufferDims::tight(PANEL_WIDTH_PX, PANEL_HEIGHT_PX);
        let hud_cfg = PfdConfig {
            background: BackgroundMode::None,
            v_speeds: None,
        };
        let _stats = render_pfd(
            &monitored.panel,
            &hud_cfg,
            Some(&monitored.alerts),
            frame_id,
            &mut PanelBuffers {
                scene: &mut self.scene_buf,
                pixels: &mut self.rgba_buf,
                dims,
            },
        )?;
        self.hud_texture.upload(queue, &self.rgba_buf)?;
        Ok(())
    }

    fn emit_active_volume(
        &mut self,
        monitored: &MonitoredFrame,
        terrain_patch: Option<&TerrainPatchSnapshot>,
    ) -> Result<(), AppSimError> {
        let report = match (self.domain, terrain_patch) {
            (FeedDomain::EarthAircraft, Some(patch)) => emit_terrain_volume_with_patch(
                &TerrainVolumeRequest {
                    snapshot: patch,
                    grid_side: 16,
                    meters_per_unit: 800.0,
                    reference_msl_m: 450.0,
                    data: &monitored.panel,
                    view: self.volume_pose,
                    alerts: Some(&monitored.alerts),
                    extent_m: None,
                },
                &mut self.volume_buf,
            )?,
            (FeedDomain::ApolloLunarModule, _) => emit_volume_scene(
                VolumeInstrument::Orrery,
                &monitored.panel,
                self.volume_pose,
                Some(&monitored.alerts),
                &[],
                None,
                &mut self.volume_buf,
            )?,
            _ => emit_volume_scene(
                VolumeInstrument::Terrain,
                &monitored.panel,
                self.volume_pose,
                Some(&monitored.alerts),
                &[],
                None,
                &mut self.volume_buf,
            )?,
        };
        self.last_volume_bytes = report.scene_bytes;
        Ok(())
    }
}

fn build_flight_sample(domain: FeedDomain, input: &AvionicsFrameInput<'_>) -> FlightSample {
    let elapsed_s = input
        .timestamp
        .duration_since(SimInstant::START)
        .as_seconds();
    let central_body = match domain {
        FeedDomain::EarthAircraft => None,
        FeedDomain::ApolloLunarModule => Some(InertialTransform::moon_at(
            UtcTime::from_unix_seconds(APOLLO_LM_EPOCH_UNIX_S + elapsed_s),
        )),
        FeedDomain::ApolloCommandModule => Some(InertialTransform::earth_at(
            UtcTime::from_unix_seconds(APOLLO_CM_EPOCH_UNIX_S + elapsed_s),
        )),
    };
    let air_data = match domain {
        FeedDomain::EarthAircraft => sample_earth_air_data(input.timestamp, &input.vehicle),
        FeedDomain::ApolloLunarModule | FeedDomain::ApolloCommandModule => None,
    };
    FlightSample {
        timestamp: input.timestamp,
        vehicle: input.vehicle,
        cockpit: Some(input.cockpit),
        central_body,
        air_data,
        specific_force_body_m_s2: input.specific_force_frd_m_s2,
    }
}

fn sample_earth_air_data(timestamp: SimInstant, vehicle: &VehicleState) -> Option<AirDataReading> {
    let geo = ecef_to_geodetic(vehicle.position_ecef);
    if geo.height_m > 19_500.0 {
        return None;
    }
    let atm = StandardAtmosphere
        .sample(&geo, UtcTime::from_unix_seconds(EARTH_EPOCH_UNIX_S))
        .ok()?;
    let sample = AirDataSample {
        true_airspeed_m_s: vehicle.velocity_ecef.length(),
        density_kg_m3: atm.density_kg_m3,
        static_pressure_pa: Some(atm.pressure_pa),
        altimeter_setting_hpa: Some(STANDARD_QNH_HPA),
    };
    Some(StampedReading::new(
        sample,
        timestamp,
        SourceValidity::Valid,
    ))
}

fn pfd_config_for(domain: FeedDomain, svs_underlay_active: bool) -> PfdConfig {
    let background = if domain == FeedDomain::EarthAircraft && svs_underlay_active {
        BackgroundMode::Svs {
            viewport: SvsViewport {
                x: 0.0,
                y: 0.0,
                width: PANEL_WIDTH_PX as f32,
                height: PANEL_HEIGHT_PX as f32,
            },
            quality: 1,
        }
    } else {
        BackgroundMode::Horizon
    };
    let v_speeds = (domain == FeedDomain::EarthAircraft).then_some(VSpeeds {
        vs0_kt: 40.0,
        vs_kt: 48.0,
        vfe_kt: 85.0,
        vno_kt: 129.0,
        vne_kt: 163.0,
    });
    PfdConfig {
        background,
        v_speeds,
    }
}

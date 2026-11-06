//! `Indicate` panels of the cockpit: PFD, HSI, full-window conformal HUD, and 3D volumes.

use glam::{DQuat, DVec3};
use sim_apollo::CockpitInstrumentSnapshot;
use sim_core::{SimInstant, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, InertialTransform, UtcTime};
use sim_indicate::{
    AirDataReading, AltitudeSource, BackgroundMode, DisplayAxes, FeedConfig, FlightSample, FrameId,
    FramebufferDims, HeadingMode, HostFlightGuidance, MonitoredFrame, PANEL_HEIGHT_PX,
    PANEL_RGBA_BYTES, PANEL_SCENE_BYTES, PANEL_WIDTH_PX, PanelBuffers, PfdConfig, SourceMonitor,
    StateFeed, SvsViewport, VOLUME_SCENE_BYTES, VSpeeds, VectorHudPanel, ViewPose3D,
    VolumeInstrument, emit_volume_scene, render_hsi, render_pfd,
};
use sim_maplibre::{
    DynamicEntity3D, TerrainPatchSnapshot, TerrainVolumeRequest, emit_terrain_volume_with_patch,
};
use sim_render::{Camera, RgbaPanelTexture, VectorScenePass, VectorTarget};
use sim_scenario::{OperatorViewportMode, ScenarioProfile};
use sim_scenery::{CatalogVerticalDatum, LandingRunwayGeometry};

use crate::error::AppSimError;
use crate::supervisor_rig::default_delaware_spawn;

mod hud;
pub use hud::ConformalSurface;
#[cfg(test)]
pub(crate) use hud::hud_runway;

const APOLLO_LM_EPOCH_UNIX_S: f64 = -14_182_200.0;
const APOLLO_CM_EPOCH_UNIX_S: f64 = -13_854_300.0;

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
                heading_mode: HeadingMode::EarthMagnetic,
            },
            Self::ApolloLunarModule => FeedConfig {
                body: BodyEllipsoid::MOON_IAU,
                axes: DisplayAxes::apollo_lm(),
                origin: Geodetic::from_degrees(0.674_16, 23.473_15, 0.0),
                origin_id: 11,
                altitude: AltitudeSource::OriginRelative,
                heading_mode: HeadingMode::SimLocalTrue,
            },
            Self::ApolloCommandModule => FeedConfig {
                body: BodyEllipsoid::WGS84,
                axes: DisplayAxes::AEROSPACE,
                origin: Geodetic::from_degrees(-3.2, 171.9, 0.0),
                origin_id: 12,
                altitude: AltitudeSource::OriginRelative,
                heading_mode: HeadingMode::SimLocalTrue,
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
    /// Navigation solution the displays draw (never simulation truth).
    pub vehicle: VehicleState,
    /// Cockpit instruments as the onboard attitude and rate sensors read them.
    pub cockpit: CockpitInstrumentSnapshot,
    /// Pitot-static air data, `None` where the static port reads nothing.
    pub air_data: Option<AirDataReading>,
    /// Accelerometer body FRD specific force (`m/s²`), if modeled.
    pub specific_force_frd_m_s2: Option<DVec3>,
    /// Whether a conformal SVS instrument map rendered an underlay this frame.
    pub svs_underlay_active: bool,
    /// The window the conformal HUD overlays, when it is shown.
    pub hud: Option<HudView<'a>>,
    /// Surveyed landing runway geometry (FAA/AIP), drawn conformally on the HUD.
    pub runway: Option<&'a LandingRunwayGeometry>,
    /// What the flight-guidance host reports; shown on Earth aircraft, and its epoch resets
    /// the display's guidance clock whenever the host's time goes back.
    pub guidance: &'a HostFlightGuidance,
    /// Active operator viewport mode (triggers 3D `VolumeInstrument` emission on `VolumetricScene`).
    pub viewport_mode: OperatorViewportMode,
    /// Optional stitched `129×129` DEM patch from a conformal instrument map for `terrain-volume`.
    pub terrain_patch: Option<&'a TerrainPatchSnapshot>,
}

/// The window a conformal HUD is drawn over.
#[derive(Clone, Copy, Debug)]
pub struct HudView<'a> {
    /// Window size, physical pixels; the HUD keeps its aspect.
    pub surface_size: [u32; 2],
    /// Camera drawing the window, whose field of view the HUD shares.
    pub camera: Camera,
    /// Belief traffic shown as HUD contacts.
    pub traffic: &'a [DynamicEntity3D],
    /// True attitude of the airframe the combiner glass is bolted to: with the window
    /// camera it fixes where the glass sits in view, while everything drawn on the glass
    /// comes from the navigation solution.
    pub airframe_attitude: DQuat,
    /// Lever arm (`m`, body FRD) from the aircraft reference point to the pilot's eye.
    pub eye_body_lever_m: DVec3,
    /// Body ellipsoid and rendered surface figure that conformal cues register against.
    pub conformal_surface: ConformalSurface,
    /// Earth vertical datum in which `traffic` carries `AboveMsl` heights (`None` off Earth).
    pub vertical_datum: Option<CatalogVerticalDatum>,
}

/// Owns `Indicate` state feeds, source monitors, CPU raster buffers, persistent GPU panel
/// textures, and the `4×` MSAA vector HUD target.
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
    hud_pass: VectorScenePass,
    hud_target: Option<VectorTarget>,
    hud_panel: VectorHudPanel,
    hud_shown: bool,
    guidance_epoch: Option<u32>,
    #[cfg(test)]
    last_heading_ref: Option<sim_indicate::HeadingReference>,
    #[cfg(test)]
    last_heading_and_course_rad: Option<(f32, Option<f32>)>,
}

impl AvionicsRig {
    /// Allocates reusable `Indicate` scene/pixel buffers and `480 × 360` sRGB GPU textures;
    /// the HUD draws through `hud_pass` into a target sized to the window on first use.
    pub fn new(device: &wgpu::Device, hud_pass: VectorScenePass) -> Result<Self, AppSimError> {
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
            hud_pass,
            hud_target: None,
            hud_panel: VectorHudPanel::new(),
            hud_shown: false,
            guidance_epoch: None,
            #[cfg(test)]
            last_heading_ref: None,
            #[cfg(test)]
            last_heading_and_course_rad: None,
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

    /// Premultiplied full-window HUD view, when the HUD drew this frame.
    #[must_use]
    pub fn hud_view(&self) -> Option<&wgpu::TextureView> {
        self.hud_target
            .as_ref()
            .filter(|_| self.hud_shown)
            .map(VectorTarget::view)
    }

    /// Byte count of the most recently emitted 3D volumetric instrument scene (`VOL-01`).
    #[cfg(test)]
    #[must_use]
    pub const fn last_volume_scene_bytes(&self) -> usize {
        self.last_volume_bytes
    }

    /// Active heading reference (`Magnetic` vs polar `True` reversion) resolved on the latest frame.
    #[cfg(test)]
    #[must_use]
    pub const fn last_heading_reference(&self) -> Option<sim_indicate::HeadingReference> {
        self.last_heading_ref
    }

    /// `(heading_rad, nav_course_rose_rad)` resolved on the latest frame.
    #[cfg(test)]
    #[must_use]
    pub const fn last_heading_and_course_rad(&self) -> Option<(f32, Option<f32>)> {
        self.last_heading_and_course_rad
    }

    /// Steps `Indicate` state resolution, rasterizes PFD + HSI (+ optional HUD), uploads GPU textures,
    /// and emits the 3D volumetric instrument IR when in `VolumetricScene`.
    pub fn update(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        input: &AvionicsFrameInput<'_>,
    ) -> Result<MonitoredFrame, AppSimError> {
        let next_domain = FeedDomain::for_profile(input.profile);
        if next_domain != self.domain {
            self.domain = next_domain;
            self.feed = StateFeed::new(next_domain.feed_config());
            self.monitor = SourceMonitor::new();
        }
        let epoch = input.guidance.source_epoch;
        if self.guidance_epoch.is_some_and(|seen| seen != epoch) {
            // The host's clock went back: the guidance lane and the source comparator both
            // refuse time that runs backwards, so they start over at the new epoch.
            self.feed.reset_guidance_clock();
            self.monitor = SourceMonitor::new();
        }
        self.guidance_epoch = Some(epoch);
        self.hud_shown = false;

        let sample = build_flight_sample(self.domain, input);
        let monitored = self.monitor.step(&mut self.feed, &sample)?;
        #[cfg(test)]
        {
            self.last_heading_ref = Some(monitored.panel.heading.reference);
            let crs = monitored
                .panel
                .nav
                .course_rose_rad
                .status
                .shows_value()
                .then_some(monitored.panel.nav.course_rose_rad.value);
            self.last_heading_and_course_rad = Some((monitored.panel.heading.value_rad.value, crs));
        }
        self.render_gen = self.render_gen.wrapping_add(1);
        let frame_id = FrameId {
            frame_generation: monitored.state.snapshot.generation,
            render_generation: self.render_gen,
        };

        let pfd_cfg = pfd_config_for(self.domain, input.svs_underlay_active);
        self.rasterize_and_upload_pfd(queue, &monitored, &pfd_cfg, frame_id)?;
        self.rasterize_and_upload_hsi(queue, &monitored, frame_id)?;
        if let Some(view) = input
            .hud
            .filter(|_| self.domain == FeedDomain::EarthAircraft)
        {
            self.render_vector_hud(device, queue, &monitored, input, &view)?;
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

    /// Tessellates the HUD at the window's physical resolution and draws it with `4×` MSAA;
    /// a window outside the HUD's aspect range shows no HUD rather than failing the frame.
    fn render_vector_hud(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        monitored: &MonitoredFrame,
        input: &AvionicsFrameInput<'_>,
        view: &HudView<'_>,
    ) -> Result<(), AppSimError> {
        let [w, h] = view.surface_size.map(|v| v.max(1));
        if VectorHudPanel::design_frame(w, h).is_err() {
            return Ok(());
        }
        let context = hud::hud_context(&input.vehicle, view, input.runway);
        let vertices =
            self.hud_panel
                .tessellate(&monitored.panel, &context, Some(&monitored.alerts), w, h)?;
        let target = match self.hud_target.as_mut() {
            Some(target) => {
                target.ensure_size(device, w, h)?;
                target
            }
            None => self
                .hud_target
                .insert(VectorTarget::new(device, "indicate hud", w, h)?),
        };
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("indicate hud"),
        });
        self.hud_pass
            .encode(device, queue, &mut encoder, target, &vertices)?;
        queue.submit([encoder.finish()]);
        self.hud_shown = true;
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
        FeedDomain::EarthAircraft => input.air_data,
        FeedDomain::ApolloLunarModule | FeedDomain::ApolloCommandModule => None,
    };
    let earth = domain == FeedDomain::EarthAircraft;
    FlightSample {
        timestamp: input.timestamp,
        vehicle: input.vehicle,
        cockpit: Some(input.cockpit),
        central_body,
        air_data,
        specific_force_body_m_s2: input.specific_force_frd_m_s2,
        guidance: earth.then_some(*input.guidance),
    }
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

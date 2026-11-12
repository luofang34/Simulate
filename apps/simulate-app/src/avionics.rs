//! `Indicate` panels of the cockpit: PFD, HSI, full-window conformal HUD, and 3D volumes.

use glam::{DQuat, DVec3};
use sim_apollo::CockpitInstrumentSnapshot;
use sim_core::{SimInstant, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, InertialTransform, UtcTime};
use sim_indicate::{
    AirDataReading, AltitudeSource, BackgroundMode, DeviationScaleProfile, DisplayAxes, FeedConfig,
    FlightSample, HeadingMode, HostFlightGuidance, MonitoredFrame, PANEL_HEIGHT_PX, PANEL_WIDTH_PX,
    PfdConfig, SourceMonitor, StateFeed, SvsViewport, VOLUME_SCENE_BYTES, VectorFlightPanels,
    VectorHudPanel, ViewPose3D, VolumeInstrument, emit_volume_scene,
};
use sim_maplibre::{
    DynamicEntity3D, TerrainPatchSnapshot, TerrainVolumeRequest, emit_terrain_volume_with_patch,
};
use sim_pack::ResolvedAirspeedEnvelope;
use sim_render::{Camera, VectorScenePass, VectorTarget};
use sim_scenario::{OperatorViewportMode, ScenarioProfile};
use sim_scenery::{CatalogVerticalDatum, LandingRunwayGeometry};

use crate::error::AppSimError;
use crate::supervisor_rig::default_delaware_spawn;

mod hud;
pub use hud::ConformalSurface;
#[cfg(test)]
pub(crate) use hud::hud_runway;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FeedDomain {
    EarthAircraft,
    ApolloLunarModule,
    ApolloCommandModule,
}

/// The feed of an aircraft flying over the Earth: what the panel's gyro, turn
/// and compass instruments read their flight state from.
pub(crate) fn earth_aircraft_feed_config() -> FeedConfig {
    FeedDomain::EarthAircraft.feed_config()
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
                deviation_scale: DeviationScaleProfile::GNSS_NAVIGATOR,
            },
            Self::ApolloLunarModule => FeedConfig {
                body: BodyEllipsoid::MOON_IAU,
                axes: DisplayAxes::apollo_lm(),
                origin: Geodetic::from_degrees(0.674_16, 23.473_15, 0.0),
                origin_id: 11,
                altitude: AltitudeSource::OriginRelative,
                heading_mode: HeadingMode::SimLocalTrue,
                deviation_scale: DeviationScaleProfile::GNSS_NAVIGATOR,
            },
            Self::ApolloCommandModule => FeedConfig {
                body: BodyEllipsoid::WGS84,
                axes: DisplayAxes::AEROSPACE,
                origin: Geodetic::from_degrees(-3.2, 171.9, 0.0),
                origin_id: 12,
                altitude: AltitudeSource::OriginRelative,
                heading_mode: HeadingMode::SimLocalTrue,
                deviation_scale: DeviationScaleProfile::GNSS_NAVIGATOR,
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
    /// UTC at `timestamp` on the flight's clock, which orients the Moon and
    /// Earth under Apollo's inertial displays.
    pub utc: Option<UtcTime>,
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
    /// Physical `[width, height]` of a docked PFD/HSI panel when instruments are shown (`None` skips both).
    pub dock_panel_px: Option<[u32; 2]>,
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
    /// Package-declared fixed-wing airspeed limits and reference V-speeds at live gross mass.
    pub airspeed_envelope: Option<ResolvedAirspeedEnvelope>,
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

/// Owns `Indicate` state feeds, source monitors, and `4×` MSAA GPU vector targets for
/// PFD, HSI, and the conformal HUD.
pub struct AvionicsRig {
    domain: FeedDomain,
    feed: StateFeed,
    monitor: SourceMonitor,
    panels: VectorFlightPanels,
    volume_buf: Vec<u8>,
    volume_pose: ViewPose3D,
    last_volume_bytes: usize,
    pfd_target: VectorTarget,
    hsi_target: VectorTarget,
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
    /// Allocates reusable `Indicate` scene encoders and `4×` MSAA GPU vector targets for PFD,
    /// HSI, and the HUD; panel targets resize automatically to match the dock's physical pixels.
    pub fn new(device: &wgpu::Device, hud_pass: VectorScenePass) -> Result<Self, AppSimError> {
        let domain = FeedDomain::EarthAircraft;
        Ok(Self {
            domain,
            feed: StateFeed::new(domain.feed_config()),
            monitor: SourceMonitor::new(),
            panels: VectorFlightPanels::new(),
            volume_buf: vec![0_u8; VOLUME_SCENE_BYTES],
            volume_pose: ViewPose3D::default(),
            last_volume_bytes: 0,
            pfd_target: VectorTarget::new(device, "indicate pfd", PANEL_WIDTH_PX, PANEL_HEIGHT_PX)?,
            hsi_target: VectorTarget::new(device, "indicate hsi", PANEL_WIDTH_PX, PANEL_HEIGHT_PX)?,
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

    /// Premultiplied `Rgba8UnormSrgb` view of the latest PFD frame (`BackgroundMode::Svs` or `Horizon`).
    #[must_use]
    pub fn pfd_view(&self) -> &wgpu::TextureView {
        self.pfd_target.view()
    }

    /// Premultiplied `Rgba8UnormSrgb` view of the latest HSI frame.
    #[must_use]
    pub fn hsi_view(&self) -> &wgpu::TextureView {
        self.hsi_target.view()
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
        let hud_view = input
            .hud
            .filter(|_| self.domain == FeedDomain::EarthAircraft);
        if input.dock_panel_px.is_some() || hud_view.is_some() {
            let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("indicate avionics vector"),
            });
            let mut recorded = false;
            if let Some([pw, ph]) = input.dock_panel_px {
                let pfd_cfg = pfd_config_for(
                    self.domain,
                    input.svs_underlay_active,
                    input.airspeed_envelope.as_ref(),
                );
                self.encode_vector_panels(
                    device,
                    queue,
                    &mut encoder,
                    &monitored,
                    &pfd_cfg,
                    [pw, ph],
                )?;
                recorded = true;
            }
            if let Some(view) = hud_view {
                recorded |=
                    self.encode_vector_hud(device, queue, &mut encoder, &monitored, input, &view)?;
            }
            if recorded {
                queue.submit([encoder.finish()]);
            }
        }
        if input.viewport_mode == OperatorViewportMode::VolumetricScene {
            self.emit_active_volume(&monitored, input.terrain_patch)?;
        }
        Ok(monitored)
    }

    fn encode_vector_panels(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        monitored: &MonitoredFrame,
        config: &PfdConfig,
        [raw_w, raw_h]: [u32; 2],
    ) -> Result<(), AppSimError> {
        let (w, h) = (raw_w.max(64), raw_h.max(48));
        let pfd_verts = self.panels.tessellate_pfd_slice(
            &monitored.panel,
            config,
            Some(&monitored.alerts),
            w,
            h,
        )?;
        self.pfd_target.ensure_size(device, w, h)?;
        self.hud_pass
            .encode(device, queue, encoder, &mut self.pfd_target, pfd_verts)?;

        let hsi_verts =
            self.panels
                .tessellate_hsi_slice(&monitored.panel, Some(&monitored.alerts), w, h)?;
        self.hsi_target.ensure_size(device, w, h)?;
        self.hud_pass
            .encode(device, queue, encoder, &mut self.hsi_target, hsi_verts)?;
        Ok(())
    }

    /// Tessellates the HUD at the window's physical resolution and records it into `encoder`
    /// with `4×` MSAA; a window outside the HUD's aspect range shows no HUD rather than failing.
    fn encode_vector_hud(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        monitored: &MonitoredFrame,
        input: &AvionicsFrameInput<'_>,
        view: &HudView<'_>,
    ) -> Result<bool, AppSimError> {
        let [w, h] = view.surface_size.map(|v| v.max(1));
        if VectorHudPanel::design_frame(w, h).is_err() {
            return Ok(false);
        }
        let context = hud::hud_context(
            &input.vehicle,
            view,
            input.runway,
            input.airspeed_envelope.as_ref(),
        );
        let vertices = self.hud_panel.tessellate_slice(
            &monitored.panel,
            &context,
            Some(&monitored.alerts),
            w,
            h,
        )?;
        let target = match self.hud_target.as_mut() {
            Some(target) => {
                target.ensure_size(device, w, h)?;
                target
            }
            None => self
                .hud_target
                .insert(VectorTarget::new(device, "indicate hud", w, h)?),
        };
        self.hud_pass
            .encode(device, queue, encoder, target, vertices)?;
        self.hud_shown = true;
        Ok(true)
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
    let central_body = match domain {
        FeedDomain::EarthAircraft => None,
        FeedDomain::ApolloLunarModule => input.utc.map(InertialTransform::moon_at),
        FeedDomain::ApolloCommandModule => input.utc.map(InertialTransform::earth_at),
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

fn pfd_config_for(
    domain: FeedDomain,
    svs_underlay_active: bool,
    airspeed_envelope: Option<&ResolvedAirspeedEnvelope>,
) -> PfdConfig {
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
    let (v_speeds, v_speed_bugs) = if domain == FeedDomain::EarthAircraft {
        hud::indicate_v_speeds(airspeed_envelope)
    } else {
        Default::default()
    };
    PfdConfig {
        background,
        v_speeds,
        v_speed_bugs,
    }
}

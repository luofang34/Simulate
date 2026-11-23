//! `Indicate` HWD (`indicate-instrument-hmd`), conformal HUD (`indicate-instrument-hud`),
//! PFD/HSI (`indicate-instrument-panels`), and 3D Volume (`indicate-instrument-volumes`) emitter.

mod state;

pub use self::state::{build_aircraft_state, build_aircraft_state_with_approach};

use crate::error::FlightDeckError;
use crate::planet::PlanetaryReference;
use crate::telemetry::OwnshipState;
use crate::terrain::runways::ApproachRunwaySelection;
use indicate_instrument_descriptor::DesignFrame;
use indicate_instrument_glyphs::{CELL_H, CELL_W, PANEL_GLYPHS};
use indicate_instrument_hmd::{DisplayContext, HMD_FRAME, directions, draw_hwd, use_compact};
use indicate_instrument_hud::{
    HudContext, HudProfile, HudProjection, Runway as HudRunway, TargetPicture, WorldSource,
    draw_hud,
};
use indicate_instrument_panels::{BUILTIN_FRAME, PfdConfig, draw_hsi, draw_pfd};
use indicate_instrument_scene::{MAX_SCENE_BYTES, SceneWriter};
use indicate_instrument_state::{FreshnessPolicy, PanelData, Quat, Sig, SignalStatus, resolve};
use indicate_interaction::ViewPose3D;
use sim_indicate::{VOLUME_SCENE_BYTES, VolumeInstrument, emit_volume_scene};

const HUD_DESIGN: DesignFrame = DesignFrame {
    width: 1280.0,
    height: 960.0,
};

/// Active outside-scene camera pose (`FPV` or `Chase`) used to align `Indicate`'s conformal HUD
/// `body_from_view` and runway projection 1:1 with the rendered Metal 3D viewport.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraViewPose {
    /// Camera eye planetographic latitude (`deg`).
    pub latitude_deg: f64,
    /// Camera eye planetographic longitude (`deg`).
    pub longitude_deg: f64,
    /// Camera eye altitude above the declared vertical datum (`m`).
    pub altitude_datum_m: f64,
    /// Camera true look heading (`deg`, `0` = North, `90` = East).
    pub true_heading_deg: f64,
    /// Camera pitch angle (`deg`, positive nose/look up).
    pub pitch_deg: f64,
    /// Camera roll angle (`deg`, positive right side down).
    pub roll_deg: f64,
}

/// Pilot headset orientation relative to aircraft forward cockpit axes (`radians`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HeadPose {
    /// Head yaw angle relative to cockpit nose, radians (positive right).
    pub yaw_rad: f32,
    /// Head pitch angle relative to cockpit waterline, radians (positive up).
    pub pitch_rad: f32,
    /// Head roll angle, radians (positive right ear down).
    pub roll_rad: f32,
}

impl HeadPose {
    /// Cosine of off-boresight look angle (`1.0` looking straight ahead).
    #[must_use]
    pub fn alignment_cosine(self) -> f32 {
        libm::cosf(self.yaw_rad) * libm::cosf(self.pitch_rad)
    }
}

/// Complete `Indicate` instrument scene bundle for one display frame.
#[derive(Clone, Debug, Default)]
pub struct InstrumentScenes {
    /// Conformal 3D unit-sphere HWD stroke segments (`[a_enu, b_enu]`) from `indicate-instrument-hmd`.
    pub hwd_3d_strokes_enu: Vec<[[f32; 3]; 2]>,
    /// Whether the HWD peripheral overlay automatically switched to compact off-axis glance mode.
    pub hwd_compact_mode: bool,
    /// 2D HWD peripheral/glance scene command buffer (`1200 x 600` `HMD_FRAME`).
    pub hwd_overlay_scene: Vec<u8>,
    /// Conformal 2D HUD scene command buffer (`960 x 720`).
    pub hud_scene: Vec<u8>,
    /// Primary Flight Display (`PFD`) scene command buffer (`480 x 360` `BUILTIN_FRAME`).
    pub pfd_scene: Vec<u8>,
    /// Horizontal Situation Indicator (`HSI`) scene command buffer (`480 x 360` `BUILTIN_FRAME`).
    pub hsi_scene: Vec<u8>,
    /// 3D Volumetric Terrain/Route instrument scene command buffer (`indicate-instrument-volumes`).
    pub terrain_volume_scene: Vec<u8>,
}

/// Verified `Indicate` bitmap glyph entry for `IndicateAppleDisplay`'s `GlyphAtlas`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedGlyph {
    /// Unicode scalar code point.
    pub scalar: u32,
    /// Advance width in cell columns.
    pub advance: u8,
    /// Bitmap rows top-to-bottom.
    pub rows: Vec<u8>,
}

/// Verified `Indicate` glyph pack metadata + bitmaps.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedGlyphPack {
    /// Cell width (`px`).
    pub cell_width: u8,
    /// Cell height (`px`).
    pub cell_height: u8,
    /// Recorded content hash.
    pub content_hash: [u8; 32],
    /// Covered glyphs.
    pub glyphs: Vec<VerifiedGlyph>,
}

/// Returns the verified `PANEL_GLYPHS` font pack used across all `Indicate` 2D/3D displays.
#[must_use]
pub fn verified_glyph_pack() -> VerifiedGlyphPack {
    VerifiedGlyphPack {
        cell_width: CELL_W as u8,
        cell_height: CELL_H as u8,
        content_hash: PANEL_GLYPHS.recorded_hash(),
        glyphs: PANEL_GLYPHS
            .iter()
            .map(|entry| VerifiedGlyph {
                scalar: entry.glyph.ch as u32,
                advance: entry.glyph.advance,
                rows: entry.glyph.rows.to_vec(),
            })
            .collect(),
    }
}

/// Stateful `Indicate` emitter tracking off-axis HWD hysteresis (`35°` entry / `28°` exit),
/// conformal HUD vertical field of view, dynamic `DesignFrame`, and 3D volumetric view pose.
#[derive(Clone, Debug)]
pub struct InstrumentSuite {
    was_compact: bool,
    hud_vertical_fov_rad: f32,
    hud_design_frame: DesignFrame,
    last_true_data: Option<PanelData>,
    last_hud_ctx: Option<HudContext>,
    volume_pose: ViewPose3D,
    scratch_scene: Vec<u8>,
    scratch_volume: Vec<u8>,
}

impl Default for InstrumentSuite {
    fn default() -> Self {
        Self::new()
    }
}

impl InstrumentSuite {
    /// Creates a new [`InstrumentSuite`] with reusable scene scratch buffers.
    #[must_use]
    pub fn new() -> Self {
        Self {
            was_compact: false,
            hud_vertical_fov_rad: 52.0_f32.to_radians(),
            hud_design_frame: HUD_DESIGN,
            last_true_data: None,
            last_hud_ctx: None,
            volume_pose: ViewPose3D::default(),
            scratch_scene: vec![0u8; MAX_SCENE_BYTES],
            scratch_volume: vec![0u8; VOLUME_SCENE_BYTES],
        }
    }

    /// Sets the conformal HUD vertical field of view (`radians`, clamped to `5°..=120°`),
    /// matching the inset HUD frame subtended on the outside-scene camera (`~/sokoly-app/app/src/ui/hud.rs`).
    pub fn set_hud_vertical_fov_rad(&mut self, fov_rad: f32) {
        if fov_rad.is_finite() {
            let clamped = fov_rad.clamp(5.0_f32.to_radians(), 120.0_f32.to_radians());
            self.hud_vertical_fov_rad = clamped;
            if let Some(ctx) = &mut self.last_hud_ctx {
                ctx.projection.value.vertical_fov_rad = clamped;
            }
        }
    }

    /// Current conformal HUD vertical field of view (`radians`).
    #[must_use]
    pub const fn hud_vertical_fov_rad(&self) -> f32 {
        self.hud_vertical_fov_rad
    }

    /// Re-emits the latest conformal HUD scene into the exact `(width, height)` `DesignFrame`
    /// requested by `IndicateAppleDisplay`'s `PanelDisplay`, guaranteeing `[width * 0.5, height * 0.5]`
    /// boresight centering across all screen sizes and orientations.
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::Instrument`] if encoding fails.
    pub fn emit_hud_for_frame(
        &mut self,
        width: f32,
        height: f32,
    ) -> Result<Vec<u8>, FlightDeckError> {
        let frame = clamp_hud_design_frame(width, height);
        self.hud_design_frame = frame;
        let (Some(data), Some(ctx)) = (self.last_true_data, self.last_hud_ctx) else {
            return Ok(Vec::new());
        };
        self.write_2d_scene("hud", |writer| {
            draw_hud(&data, &ctx, None, frame, writer).map_err(|e| format!("{e:?}"))
        })
    }

    /// Emits HWD (3D unit-sphere + 2D overlay), HUD, PFD, HSI, and Terrain Volume scenes on `planet`.
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::Instrument`] if any `Indicate` scene buffer fails to encode.
    #[allow(clippy::too_many_arguments)]
    pub fn emit(
        &mut self,
        ownship: &OwnshipState,
        planet: &PlanetaryReference,
        head_pose: HeadPose,
        targets: TargetPicture,
        approach_runway: Option<HudRunway>,
        ground_elev_m: f64,
        now_ms: u64,
    ) -> Result<InstrumentScenes, FlightDeckError> {
        self.emit_with_view(
            ownship,
            planet,
            head_pose,
            None,
            targets,
            None,
            approach_runway,
            ground_elev_m,
            now_ms,
        )
    }

    /// Emits all `Indicate` scenes aligned to an optional [`CameraViewPose`] (`FPV` or `Chase`)
    /// and [`ApproachRunwaySelection`].
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::Instrument`] if any `Indicate` scene buffer fails to encode.
    #[allow(clippy::too_many_arguments)]
    pub fn emit_with_view(
        &mut self,
        ownship: &OwnshipState,
        planet: &PlanetaryReference,
        head_pose: HeadPose,
        camera_view: Option<CameraViewPose>,
        targets: TargetPicture,
        approach_sel: Option<&ApproachRunwaySelection>,
        approach_runway: Option<HudRunway>,
        ground_elev_m: f64,
        now_ms: u64,
    ) -> Result<InstrumentScenes, FlightDeckError> {
        let policy = FreshnessPolicy::default();
        let panel_data = resolve(
            &build_aircraft_state_with_approach(ownship, now_ms, planet, false, approach_sel),
            &policy,
        );
        let true_data = resolve(
            &build_aircraft_state_with_approach(ownship, now_ms, planet, true, approach_sel),
            &policy,
        );

        let compact = use_compact(&true_data, head_pose.alignment_cosine(), self.was_compact);
        self.was_compact = compact;

        let angular = directions(&true_data);
        let stroke_len = (angular.length as usize).min(angular.strokes.len());
        let hwd_3d_strokes_enu = angular
            .strokes
            .get(..stroke_len)
            .unwrap_or(&[])
            .iter()
            .map(|st| [st.a, st.b])
            .collect();

        let hwd_overlay_scene = self.write_2d_scene("hwd", |writer| {
            draw_hwd(&true_data, DisplayContext::default(), compact, None, writer)
                .map_err(|e| format!("{e:?}"))
        })?;
        let body_from_view = compute_body_from_view(ownship, camera_view);
        let hud_ctx = build_hud_context(
            self.hud_vertical_fov_rad,
            body_from_view,
            targets,
            approach_runway,
            ground_elev_m,
        );
        let hud_frame = self.hud_design_frame;
        let hud_scene = self.write_2d_scene("hud", |writer| {
            draw_hud(&true_data, &hud_ctx, None, hud_frame, writer).map_err(|e| format!("{e:?}"))
        })?;
        self.last_true_data = Some(true_data);
        self.last_hud_ctx = Some(hud_ctx);

        let pfd_cfg = PfdConfig::default();
        let pfd_scene = self.write_2d_scene("pfd", |writer| {
            draw_pfd(&panel_data, &pfd_cfg, None, BUILTIN_FRAME, writer)
                .map_err(|e| format!("{e:?}"))
        })?;
        let hsi_scene = self.write_2d_scene("hsi", |writer| {
            draw_hsi(&panel_data, None, BUILTIN_FRAME, writer).map_err(|e| format!("{e:?}"))
        })?;
        let terrain_volume_scene = self.emit_terrain_volume(&true_data)?;

        Ok(InstrumentScenes {
            hwd_3d_strokes_enu,
            hwd_compact_mode: compact,
            hwd_overlay_scene,
            hud_scene,
            pfd_scene,
            hsi_scene,
            terrain_volume_scene,
        })
    }

    fn emit_terrain_volume(
        &mut self,
        true_data: &indicate_instrument_state::PanelData,
    ) -> Result<Vec<u8>, FlightDeckError> {
        let vol_render = emit_volume_scene(
            VolumeInstrument::Terrain,
            true_data,
            self.volume_pose,
            None,
            &[],
            None,
            &mut self.scratch_volume,
        )
        .map_err(|e| FlightDeckError::Instrument {
            panel: "terrain-volume",
            reason: e.to_string(),
        })?;
        Ok(self
            .scratch_volume
            .get(..vol_render.scene_bytes)
            .unwrap_or(&[])
            .to_vec())
    }

    fn write_2d_scene(
        &mut self,
        panel: &'static str,
        draw: impl FnOnce(&mut SceneWriter<'_>) -> Result<(), String>,
    ) -> Result<Vec<u8>, FlightDeckError> {
        let mut writer =
            SceneWriter::new(&mut self.scratch_scene).map_err(|e| FlightDeckError::Instrument {
                panel,
                reason: format!("{e:?}"),
            })?;
        draw(&mut writer).map_err(|reason| FlightDeckError::Instrument { panel, reason })?;
        let len = writer.finish();
        Ok(self.scratch_scene.get(..len).unwrap_or(&[]).to_vec())
    }
}

fn clamp_hud_design_frame(width: f32, height: f32) -> DesignFrame {
    let raw_w = if width.is_finite() { width } else { 1280.0 };
    let raw_h = if height.is_finite() { height } else { 960.0 };
    let mut w = raw_w.clamp(390.0, 2560.0).round();
    let mut h = raw_h.clamp(390.0, 1600.0).round();
    let aspect = w / h;
    if aspect < 0.45 {
        w = (h * 0.45).ceil().clamp(390.0, 2560.0);
    } else if aspect > 2.4 {
        h = (w / 2.4).ceil().clamp(390.0, 1600.0);
    }
    DesignFrame {
        width: w,
        height: h,
    }
}

fn compute_body_from_view(ownship: &OwnshipState, camera_view: Option<CameraViewPose>) -> Quat {
    let Some(cam) = camera_view else {
        return Quat::IDENTITY;
    };
    let (Some(own_roll_deg), Some(own_pitch_deg), Some(own_hdg_deg)) = (
        ownship.roll_deg,
        ownship.pitch_deg,
        ownship.true_heading_deg.or(ownship.ground_track_true_deg),
    ) else {
        return Quat::IDENTITY;
    };
    let ned_from_body = Quat::from_euler(
        (own_roll_deg as f32).to_radians(),
        (own_pitch_deg as f32).to_radians(),
        (own_hdg_deg as f32).to_radians(),
    );
    let ned_from_view = Quat::from_euler(
        (cam.roll_deg as f32).to_radians(),
        (cam.pitch_deg as f32).to_radians(),
        (cam.true_heading_deg as f32).to_radians(),
    );
    ned_from_body.inverse().compose(ned_from_view)
}

fn build_hud_context(
    vertical_fov_rad: f32,
    body_from_view: Quat,
    targets: TargetPicture,
    approach: Option<HudRunway>,
    ground_elev_m: f64,
) -> HudContext {
    HudContext {
        profile: HudProfile::Screen,
        projection: Sig::with_status(
            HudProjection {
                vertical_fov_rad,
                body_from_view,
                source: WorldSource::Synthetic,
            },
            SignalStatus::Valid,
        ),
        targets,
        approach,
        terrain_elevation_m: Sig::with_status(ground_elev_m as f32, SignalStatus::Valid),
        ..HudContext::default()
    }
}

/// Design frame dimensions `(width, height)` of the 2D HWD peripheral overlay.
pub const HWD_OVERLAY_DIMS: (f32, f32) = (HMD_FRAME.width, HMD_FRAME.height);
/// Design frame dimensions `(width, height)` of the 2D conformal HUD.
pub const HUD_FRAME_DIMS: (f32, f32) = (HUD_DESIGN.width, HUD_DESIGN.height);
/// Design frame dimensions `(width, height)` of the PFD and HSI panels.
pub const PANEL_FRAME_DIMS: (f32, f32) = (BUILTIN_FRAME.width, BUILTIN_FRAME.height);

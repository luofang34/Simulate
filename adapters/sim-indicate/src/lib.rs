//! Puts `Indicate`'s real instruments on Simulate's flight state.
//!
//! [`StateFeed`] turns a [`FlightSample`] into Indicate's `AircraftState` and
//! its tagged binary frames; [`SourceMonitor`] offers the Apollo PGNS and AGS
//! attitudes to Indicate's own comparator, which alone picks what is displayed;
//! [`PanelDials`] draws an instrument pack's round dials with Indicate's
//! analog set. [`render_pfd`], [`render_hsi`], and [`render_hud`] rasterize the resolved
//! panels into caller buffers. [`VectorHudPanel`] and [`tessellate_scene_cmds`]
//! instead tessellate scenes into anti-aliased triangles for `sim-render`'s
//! `VectorScenePass` at the display's physical resolution.
//!
//! Simulate works in `f64` on a body-fixed frame, or J2000 for Apollo's sensors,
//! with vehicle-specific axes. Indicate reads `f32` forward-right-down bodies
//! on a local north-east-down frame. Every crossing goes through [`frames`],
//! on the [`sim_geo::BodyEllipsoid`] the vehicle flies over, so the Moon works
//! as the Earth does.

mod air_data;
mod clock;
mod dials;
mod error;
mod feed;
pub mod frames;
mod monitor;
mod panel;
mod vector_scene;
mod volume;

pub use air_data::{AirDataReading, AirDataSample, indicated_altitude_m, pressure_altitude_m};
pub use clock::{age_ms, instant_ms};
pub use dials::{
    DialError, DialSource, EngineGauges, FaceBake, GaugeIndications, GaugeState, Lane,
    OFF_FLAG_TEXT, PanelDials, PanelLettering, PointerValues, SlotDial, TapeBake, bake_drum_strip,
    bake_face, bake_off_flag, bake_placard, bake_tape_strip,
};
pub use error::{FeedError, PanelError};
pub use feed::{
    AltitudeSource, AutopilotModeSummary, FeedConfig, FlightSample, HeadingMode,
    HostFlightGuidance, HostNavGuidance, PilotSelections, RunwayNavGuidance, StateFeed,
    decode_frame, encode_frame,
};
pub use frames::{DisplayAxes, LocalLevel};
pub use indicate_instrument_analog::{
    CardBuild, DialPose, DialReading, Lighting, Mark, OverlaySpec, Paint, PhysicalBuild,
    PointerBuild, Shape, TapeBuild, TapeSense, WindowBuild,
};
pub use indicate_instrument_descriptor::{decode_v_speeds_payload, encode_v_speeds_payload};
pub use indicate_instrument_feeder::{
    CLOCK_HOST_MONOTONIC, ROLE_NAVIGATION_SOLUTION, RawStamp,
    nav_guidance::Guidance as LaneGuidance,
};
pub use indicate_instrument_glyphs::{Face, FaceTable, Typeface};
pub use indicate_instrument_hud::{
    Contact as HudContact, Domain as HudContactDomain, HudContext, HudProfile, HudProjection,
    Identity as HudContactIdentity, Runway as HudRunway, TargetPicture, WorldSource,
};
pub use indicate_instrument_panels::{BackgroundMode, PfdConfig, SvsViewport, VSpeeds};
pub use indicate_instrument_raster::{FrameId, FramebufferDims};
pub use indicate_instrument_state::{
    AircraftState, HeadingReference, IdentStr, Quat as IndicateQuat, Sig, SignalStatus, VSpeedBug,
    VSpeedBugs,
};
pub use indicate_instrument_symbology::v_speeds::{
    DialVSpeedScale, LinearVSpeedScale, draw_dial_arcs, draw_dial_bugs, draw_linear_bands,
    draw_linear_bugs,
};
pub use indicate_interaction::{SpatialEvent, ViewPose3D};
pub use monitor::{AGS_SOURCE, LOCAL_NED_FRAME, MonitoredFrame, PGNS_SOURCE, SourceMonitor};
pub use panel::{
    HUD_MAX_HEIGHT_PX, HUD_MAX_WIDTH_PX, HUD_MIN_SIDE_PX, PANEL_HEIGHT_PX, PANEL_RGBA_BYTES,
    PANEL_SCENE_BYTES, PANEL_WIDTH_PX, PanelBuffers, PanelRender, render_hsi, render_hud,
    render_pfd,
};
pub use vector_scene::{
    VECTOR_HUD_DESIGN_HEIGHT, VectorFlightPanels, VectorHudPanel, VectorSceneError,
    tessellate_scene_cmds, tessellate_scene_cmds_into,
};
pub use volume::{
    VOLUME_SCENE_BYTES, VolumeInstrument, VolumeRender, apply_volume_gesture, emit_volume_scene,
};

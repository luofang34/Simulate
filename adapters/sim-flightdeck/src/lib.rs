//! Multi-body EFB, GCS, flight-deck avionics, 3D airspace, surveillance, and synthetic vision adapter.
//!
//! Combines:
//! 1. **Ownship GNSS, AHRS & ADS-B links**: `avionics-link` Connext inner-record framing and
//!    capture replay ([`connext`]) plus FAA GDL-90 `UDP :4000` + ForeFlight AHRS ([`gdl90`]).
//! 2. **3D Airspace & Live Operational Notices**: `Flightbag` [`flightbag_core::AirspaceVolume`]s,
//!    pure-Rust `PMTiles` v3 / MVT 2.1 decoding, and `world-notices` TFR / SUA schedules ([`airspace`]).
//! 3. **Multi-Source ADS-B Traffic & Callsigns**: `world-surveillance` track fusion, callsign
//!    placards, and CPA `TA`/`RA` threat classification ([`traffic`]).
//! 4. **Synthetic Terrain & `Indicate` Flight Instruments**: `sim-scenery` HTAWS terrain + 3D
//!    runway/HITS overlays ([`terrain`]), `indicate-instrument-hmd` 3D unit-sphere HWD + glance
//!    hysteresis, conformal `HUD`, `PFD`, `HSI`, and 3D `Terrain` Volume ([`instruments`]), governed
//!    by the Digital Crown virtual/reality blend floor ([`blend::MIN_PASSTHROUGH_FRACTION`]).

pub mod airspace;
pub mod blend;
pub mod connext;
pub mod error;
pub mod gdl90;
pub mod instruments;
pub mod kml;
pub mod planet;
pub mod scenarios;
pub mod session;
pub mod terrain;
pub mod traffic;
pub mod wire;

pub use airspace::{
    AirspaceAlert, AirspaceAlertLevel, AirspaceClassStyle, AirspaceCoordinator,
    AirspaceRenderVolume,
};
pub use blend::{
    BlendWeights, CrownBlend, DEFAULT_CROWN_TURN, MAX_VIRTUAL_OPACITY, MIN_PASSTHROUGH_FRACTION,
};
pub use connext::{ConnextBatch, ConnextEvent, ConnextIngestor, REAL_SESSION_TSV};
pub use error::FlightDeckError;
pub use gdl90::{Gdl90AhrsReport, Gdl90Decoder, Gdl90Message, Gdl90ParticipantReport};
pub use instruments::{
    CameraViewPose, HUD_FRAME_DIMS, HWD_OVERLAY_DIMS, HeadPose, InstrumentScenes, InstrumentSuite,
    PANEL_FRAME_DIMS, VerifiedGlyph, VerifiedGlyphPack, build_aircraft_state,
    build_aircraft_state_with_approach, verified_glyph_pack,
};
pub use kml::{KmlFlightTrack, KmlReplayStatus, KmlTrackPoint};
pub use planet::{
    PlanetaryBodyId, PlanetaryReference, STAR_PACKED_STRIDE_F32, SkyEvaluator, SkySnapshot,
};
pub use scenarios::FlightDeckScenario;
pub use session::{FlightDeckFrameSnapshot, FlightDeckSession, TelemetrySourceMode};
pub use telemetry::{
    ActiveRoute, OwnshipIdentity, OwnshipState, RouteWaypoint, TelemetryClock, TelemetryGroup,
};
pub use terrain::{RunwayOverlay3D, SyntheticTerrainGenerator, TerrainMesh};
pub use traffic::{TrafficCoordinator, TrafficDisplayTarget, TrafficThreat};
pub use wire::{
    DecodedFrameHeader, FRAME_WIRE_MAGIC, FRAME_WIRE_VERSION, GLYPH_WIRE_MAGIC, GLYPH_WIRE_VERSION,
    decode_frame_header, encode_frame_snapshot, encode_glyph_pack,
};

/// Ownship telemetry state, arrival aging clock, and WMM2025 geomagnetic heading conversion.
pub mod telemetry;

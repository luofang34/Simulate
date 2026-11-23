//! Typed error hierarchy for `sim-flightdeck`.

use thiserror::Error;

/// Failures surfaced by telemetry decoders, airspace loaders, and instrument emitters.
#[derive(Debug, Error)]
pub enum FlightDeckError {
    /// Connext inner-record framing or payload error.
    #[error("connext record error: {reason}")]
    ConnextRecord {
        /// Diagnostic description.
        reason: String,
    },
    /// GDL-90 frame check sequence or payload error.
    #[error("GDL-90 decode error: {reason}")]
    Gdl90 {
        /// Diagnostic description.
        reason: String,
    },
    /// Surveillance track store rejected an observation.
    #[error("surveillance store error: {reason}")]
    Surveillance {
        /// Diagnostic description.
        reason: String,
    },
    /// Static airspace volume or operational notice validation failed.
    #[error("airspace validation error: {reason}")]
    Airspace {
        /// Diagnostic description.
        reason: String,
    },
    /// `PMTiles` v3 or Mapbox Vector Tile decoding failed.
    #[error("PMTiles / MVT decode error: {reason}")]
    PmTiles {
        /// Diagnostic description.
        reason: String,
    },
    /// Terrain elevation query failed.
    #[error("terrain query error: {reason}")]
    Terrain {
        /// Diagnostic description.
        reason: String,
    },
    /// `Indicate` instrument state or scene emission failed.
    #[error("instrument emission error in {panel}: {reason}")]
    Instrument {
        /// Panel or instrument identifier.
        panel: &'static str,
        /// Diagnostic description.
        reason: String,
    },
    /// Binary snapshot wire buffer error.
    #[error("wire codec error: {reason}")]
    Wire {
        /// Diagnostic description.
        reason: String,
    },
    /// KML or GPX flight track replay parsing error.
    #[error("KML flight track error: {reason}")]
    KmlTrack {
        /// Diagnostic description.
        reason: String,
    },
}

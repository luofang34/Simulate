//! Named chart rasters (approach plates, airport diagrams) served to maps at
//! `https://sim.chart/{chart_id}.png` from memory, so chart overlays work offline and in
//! the browser without a network round trip.

use std::{
    collections::HashMap,
    sync::{Arc, Mutex, PoisonError},
};

/// URL prefix of charts served from a [`ChartAssetStore`].
pub const CHART_URL_PREFIX: &str = "https://sim.chart/";

/// The URL a map fetches chart `chart_id` from.
#[must_use]
pub fn chart_url(chart_id: &str) -> String {
    format!("{CHART_URL_PREFIX}{chart_id}.png")
}

/// Shared, cloneable registry of encoded PNG chart rasters keyed by chart id.
///
/// Clones share one registry: a loader serving the charts and the surfaces registering
/// them hold clones of the same store.
#[derive(Clone, Debug, Default)]
pub struct ChartAssetStore {
    charts: Arc<Mutex<HashMap<String, Arc<[u8]>>>>,
}

impl ChartAssetStore {
    /// An empty store.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers (or replaces) the PNG bytes of `chart_id`.
    pub fn put_chart_png(&self, chart_id: &str, bytes: Arc<[u8]>) {
        self.charts
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .insert(chart_id.to_owned(), bytes);
    }

    /// Forgets `chart_id`; returns whether it was registered.
    pub fn remove_chart(&self, chart_id: &str) -> bool {
        self.charts
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .remove(chart_id)
            .is_some()
    }

    /// The PNG bytes of `chart_id`, if registered.
    #[must_use]
    pub fn chart_png(&self, chart_id: &str) -> Option<Arc<[u8]>> {
        self.charts
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .get(chart_id)
            .cloned()
    }

    /// The bytes a chart URL (`https://sim.chart/{chart_id}.png`) names, if registered.
    #[must_use]
    pub fn chart_for_url(&self, url: &str) -> Option<Arc<[u8]>> {
        let path = url.split(['?', '#']).next()?;
        let chart_id = path.strip_prefix(CHART_URL_PREFIX)?.strip_suffix(".png")?;
        self.chart_png(chart_id)
    }
}

#[cfg(test)]
mod tests;

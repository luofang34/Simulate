//! The EFB as the map rig keeps it for the cockpit: built when a preset needs it, drawn
//! when due on the navigation solution, and commanded by the operator.

use std::time::Duration;

use sim_maplibre::{ChartSymbologyInput, EfbDisplayMode, ViewSlotMask};
use sim_scenario::OperatorViewportMode;
use sim_scenery::LandingRunwayGeometry;
use tracing::warn;
use web_time::Instant;

use super::super::{AvionicsPreset, MapFrameInput, MapLibreRig, charts};
use super::{EfbAirport, EfbCommand, EfbRig, EfbStatus};
use crate::cockpit_rig::efb_chrome::{efb_map_rect, rect_px};
use crate::error::AppSimError;

impl MapLibreRig {
    /// Builds the EFB the first time a map preset needs it; a failure leaves the cockpit
    /// without one.
    pub(in crate::map_rig) fn ensure_efb(&mut self) {
        if self.preset == AvionicsPreset::NoMapGauges || self.efb.is_some() {
            return;
        }
        let compact = rect_px(
            efb_map_rect(self.window_px, EfbDisplayMode::CompactCorner),
            self.window_px,
        );
        self.efb = EfbRig::new(
            &self.runtime,
            self.charts.clone(),
            &self.endpoints,
            &self.aerodrome_geojson,
            compact,
        )
        .inspect_err(|err| warn!(error = %err, "EFB map unavailable"))
        .ok()
        .map(|mut efb| {
            efb.set_pixel_ratio(self.pixel_ratio);
            efb
        });
    }

    /// The EFB, when the preset shows one.
    fn shown_efb(&self) -> Option<&EfbRig> {
        self.efb
            .as_ref()
            .filter(|_| self.preset != AvionicsPreset::NoMapGauges)
    }

    /// Whether the preset shows the EFB.
    #[must_use]
    pub fn efb_shown(&self) -> bool {
        self.shown_efb().is_some()
    }

    /// What the EFB shows, when the preset shows one.
    #[must_use]
    pub fn efb_status(&self) -> Option<EfbStatus> {
        self.shown_efb().map(EfbRig::status)
    }

    /// The EFB's last frame, when the preset shows one and it is not collapsed.
    #[must_use]
    pub fn efb_view(&self) -> Option<&wgpu::TextureView> {
        self.shown_efb().and_then(EfbRig::view)
    }

    /// Whether the EFB has chrome to display (either a rendered frame or its unfold button).
    #[must_use]
    pub fn efb_ready(&self) -> bool {
        self.shown_efb().is_some_and(EfbRig::ready)
    }

    /// Whether the shown EFB has tile arrivals or a command waiting to be drawn.
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub fn efb_needs_redraw(&self) -> bool {
        self.shown_efb().is_some_and(EfbRig::needs_redraw)
    }

    /// Applies an operator command to the EFB, if there is one.
    pub fn efb_command(&mut self, command: EfbCommand) -> Result<(), AppSimError> {
        self.efb.as_mut().map_or(Ok(()), |efb| efb.command(command))
    }

    /// Sizes the expanded EFB page to its on-screen `size_px`.
    pub fn set_efb_expanded_px(&mut self, size_px: [u32; 2]) -> Result<(), AppSimError> {
        self.efb
            .as_mut()
            .map_or(Ok(()), |efb| efb.resize_expanded(size_px))
    }

    /// The airport the EFB shows instead of the supervisor's, if one was set.
    #[must_use]
    pub fn efb_airport(&self) -> Option<EfbAirport> {
        self.efb_airport.clone()
    }

    /// Shows `airport` on the EFB instead of the supervisor's active aerodrome (`None`
    /// follows the supervisor again).
    pub fn set_efb_airport(&mut self, airport: Option<EfbAirport>) {
        self.efb_airport = airport;
    }

    /// Draws the EFB when it is shown and due, on the navigation solution.
    pub(in crate::map_rig) fn step_efb(
        &mut self,
        input: &MapFrameInput<'_>,
        sim_time: Duration,
        landing_runway: Option<&LandingRunwayGeometry>,
    ) -> Result<(), AppSimError> {
        if self.preset == AvionicsPreset::NoMapGauges
            || input.supervisor.viewport_mode() == OperatorViewportMode::ChartMap
        {
            return Ok(());
        }
        let airport = self.efb_airport.clone().or_else(|| {
            let code = input.supervisor.effective_airport_code(input.profile);
            let runway = input
                .supervisor
                .active_runway(input.profile)
                .map_or_else(String::new, |r| r.designator.as_str().to_owned());
            Some((code.to_owned(), runway))
        });
        let Some(efb) = self.efb.as_mut() else {
            return Ok(());
        };
        let now = Instant::now();
        if !efb.prepare_step(now, sim_time, airport)? {
            return Ok(());
        }
        let own = charts::chart_ownship(&input.estimated_vehicle);
        let traffic = input.supervisor.belief_entities(ViewSlotMask::ALL);
        efb.render_prepared(
            now,
            sim_time,
            &ChartSymbologyInput {
                camera: charts::moving_map_camera(&own, input.supervisor),
                viewport_px: [1, 1],
                ownship: Some(own),
                runway: landing_runway,
                traffic: &traffic,
            },
        )
    }
}

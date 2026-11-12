//! Hangar livery selection, external livery/variant pack import, and 3D turntable livery sync.

use sim_core::ControlInputs;
use tracing::{info, warn};

use super::super::FlightApp;
use crate::esc_menu::catalog::HangarModel;

impl FlightApp {
    /// Returns `(hangar_liveries, active_livery_index, active_livery_name)` for the airframe
    /// currently highlighted in the Hangar tab.
    pub(super) fn hangar_livery_status(
        &self,
    ) -> (Vec<(String, String)>, Option<usize>, Option<String>) {
        let Some(aircraft_id) = self
            .catalog
            .vehicles
            .get(self.esc_menu.vehicle())
            .and_then(|v| v.aircraft_id)
        else {
            return (Vec::new(), None, None);
        };
        let session = self.flight.liveries();
        let chips: Vec<(String, String)> = session
            .liveries_for(aircraft_id)
            .into_iter()
            .map(|l| (l.tail_code, l.name))
            .collect();
        let active = session.active_livery(aircraft_id);
        let active_idx = active.map(|(idx, _)| idx);
        let active_name = active.map(|(_, liv)| format!("{} ({})", liv.tail_code, liv.name));
        (chips, active_idx, active_name)
    }

    /// Renders the Hangar turntable with any selected or imported per-tail OpenUSD livery.
    pub(super) fn render_hangar_preview(
        &mut self,
        hangar_model: Option<([u32; 2], HangarModel)>,
        live: ControlInputs,
        elapsed_s: f64,
    ) {
        let Some(renderer) = self.renderer.as_ref() else {
            return;
        };
        let aircraft_id = self
            .catalog
            .vehicles
            .get(self.esc_menu.vehicle())
            .and_then(|v| v.aircraft_id);
        let custom = aircraft_id
            .and_then(|id| self.flight.liveries().active_usd_override(id))
            .map(|((_, liv_id, generation), asset)| ((liv_id, generation), asset));
        if let Some(hangar) = self.hangar.as_mut() {
            hangar.hide();
            if let Some((size, model)) = hangar_model
                && let Err(err) = hangar.render_with_livery(
                    renderer.device(),
                    renderer.queue(),
                    model,
                    custom,
                    size,
                    live,
                    elapsed_s,
                )
            {
                warn!(error = %err, "hangar preview unavailable");
            }
        }
    }

    /// Selects livery `index` on the currently highlighted Hangar airframe.
    pub(super) fn select_hangar_livery(&mut self, index: usize) {
        let Some(aircraft_id) = self
            .catalog
            .vehicles
            .get(self.esc_menu.vehicle())
            .and_then(|v| v.aircraft_id)
        else {
            return;
        };
        match self
            .flight
            .liveries_mut()
            .select_livery_index(aircraft_id, index)
        {
            Ok(liv) => info!(
                aircraft_id,
                tail = %liv.tail_code,
                name = %liv.name,
                "selected aircraft livery"
            ),
            Err(err) => warn!(aircraft_id, index, error = %err, "selecting livery failed"),
        }
    }

    /// Imports an external livery or variant pack configured in the environment (native) or
    /// pasted via a JSON prompt (`wasm32`).
    pub(super) fn import_hangar_livery(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            let before = self.flight.liveries().revision();
            self.flight.liveries_mut().apply_environment_imports();
            if self.flight.liveries().revision() == before {
                info!(
                    livery_env = crate::livery_session::ENV_IMPORT_LIVERY,
                    variant_env = crate::livery_session::ENV_IMPORT_VARIANT,
                    "set SIMULATE_IMPORT_LIVERY or SIMULATE_IMPORT_VARIANT to import external packs"
                );
            }
        }
        #[cfg(target_arch = "wasm32")]
        {
            let aircraft_id = self
                .catalog
                .vehicles
                .get(self.esc_menu.vehicle())
                .and_then(|v| v.aircraft_id);
            let Some(json) = web_sys::window()
                .and_then(|w| {
                    w.prompt_with_message("Paste external LiverySpec JSON to import:")
                        .ok()
                        .flatten()
                })
                .filter(|s| !s.trim().is_empty())
            else {
                return;
            };
            match self
                .flight
                .liveries_mut()
                .import_livery_bytes(aircraft_id, json.as_bytes(), None)
            {
                Ok((id, liv)) => info!(
                    aircraft_id = %id,
                    tail = %liv.tail_code,
                    "imported external browser livery"
                ),
                Err(err) => warn!(error = %err, "browser livery import failed"),
            }
        }
    }
}

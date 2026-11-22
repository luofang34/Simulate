//! Non-player aircraft spawns attached to [`super::Flight`].

use sim_geo::Geodetic;

use crate::error::AppSimError;
use crate::npc_fleet::EvaluatedNpc;

impl super::Flight {
    /// Active non-player aircraft count (ground ramp + scripted flying targets).
    #[must_use]
    pub fn npc_count(&self) -> usize {
        self.npc_fleet.count()
    }

    /// Spawns one `aircraft_id` NPC onto the next unoccupied apron stand at the active airport.
    pub fn spawn_ground_npc(
        &mut self,
        aircraft_id: &str,
        livery_id: Option<&str>,
    ) -> Result<bool, AppSimError> {
        self.npc_fleet
            .spawn_ground_apron(&mut self.liveries, aircraft_id, livery_id)
    }

    /// Clears all non-player aircraft spawns.
    pub fn clear_npcs(&mut self) {
        self.npc_fleet.clear();
    }

    /// Evaluates active NPC poses and articulations for `anchor_msl` at the current flight instant.
    #[must_use]
    pub fn evaluate_npcs(&self, anchor_msl: Geodetic) -> Vec<EvaluatedNpc> {
        self.npc_fleet
            .evaluate(self.now(), anchor_msl, &self.surface)
    }

    /// Replaces active NPCs from a UTF-8 JSON script.
    #[cfg(any(test, target_arch = "wasm32"))]
    pub fn load_npc_script_bytes(&mut self, bytes: &[u8]) -> Result<usize, AppSimError> {
        self.npc_fleet.load_json_bytes(&mut self.liveries, bytes)
    }

    /// Replaces active NPCs by loading a JSON script file on disk.
    #[cfg(all(test, not(target_arch = "wasm32")))]
    pub fn load_npc_script_path(&mut self, path: &std::path::Path) -> Result<usize, AppSimError> {
        self.npc_fleet.load_json_path(&mut self.liveries, path)
    }
}

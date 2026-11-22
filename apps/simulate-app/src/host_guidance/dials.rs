//! Pilot-dialed avionics selections: HSI heading bug, altitude preselect, and baro QNH.

/// Standard sea-level barometric altimeter setting, hectopascals (`29.92 inHg`).
pub const STANDARD_QNH_HPA: f64 = 1_013.25;
/// Lowest settable barometric pressure, hectopascals.
pub const MIN_BARO_HPA: f64 = 940.0;
/// Highest settable barometric pressure, hectopascals.
pub const MAX_BARO_HPA: f64 = 1_060.0;
/// Maximum altitude bug preselect, metres (`60,000 ft`).
const MAX_SELECTED_ALT_M: f64 = 18_288.0;
/// Exact international foot conversion factor (`ft / m`).
const FEET_PER_M: f64 = 3.280_839_895_013_123;
/// Exact conversion factor from inches of mercury to hectopascals (`hPa / inHg`).
const HPA_PER_INHG: f64 = 33.863_886_666_667;

/// Interactive pilot-dialed bugs and barometric setting layered over stand-in host defaults.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PilotDialedSelections {
    /// Manual HSI heading bug (`0.0..360.0` deg), or `None` to follow runway/heading default.
    pub heading_bug_deg: Option<f64>,
    /// Manual PFD/HUD altitude preselect (metres in active datum), or `None` for host default.
    pub altitude_bug_m: Option<f64>,
    /// Active Kollsman barometric reference (`hPa`), defaulting to [`STANDARD_QNH_HPA`].
    pub baro_hpa: f64,
}

impl Default for PilotDialedSelections {
    fn default() -> Self {
        Self {
            heading_bug_deg: None,
            altitude_bug_m: None,
            baro_hpa: STANDARD_QNH_HPA,
        }
    }
}

/// Rounded display-ready summary of active bug and barometric settings for touch/HUD UI.
#[cfg_attr(
    not(target_arch = "wasm32"),
    allow(dead_code, reason = "used by wasm32 touch controls and tests")
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DialedAvionicsReadout {
    /// Active heading bug in integer degrees (`0..360`), when finite.
    pub heading_bug_deg: Option<u16>,
    /// Active altitude preselect in feet, when finite.
    pub altitude_bug_ft: Option<i32>,
    /// Active altimeter setting in hectopascals (`hPa`).
    pub baro_hpa: f64,
    /// Active altimeter setting in inches of mercury (`inHg`).
    pub baro_inhg: f64,
}

impl PilotDialedSelections {
    /// Steps the heading bug by `delta_deg`, initializing from `fallback_heading_deg` if unset.
    pub fn step_heading_deg(&mut self, delta_deg: f64, fallback_heading_deg: f64) -> f64 {
        let base = self.heading_bug_deg.unwrap_or_else(|| {
            if fallback_heading_deg.is_finite() {
                wrap_deg(fallback_heading_deg).round()
            } else {
                0.0
            }
        });
        let delta = if delta_deg.is_finite() {
            delta_deg
        } else {
            0.0
        };
        let next = wrap_deg(base + delta);
        self.heading_bug_deg = Some(next);
        next
    }

    /// Locks the heading bug onto `current_heading_deg` (rounded to the nearest degree).
    pub fn sync_heading_deg(&mut self, current_heading_deg: f64) -> f64 {
        let next = if current_heading_deg.is_finite() {
            wrap_deg(current_heading_deg.round())
        } else {
            0.0
        };
        self.heading_bug_deg = Some(next);
        next
    }

    /// Steps the altitude bug by `delta_ft`, initializing from `fallback_alt_m` if unset.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "used by wasm32 touch controls and tests")
    )]
    pub fn step_altitude_ft(&mut self, delta_ft: f64, fallback_alt_m: f64) -> f64 {
        let base_m = self.altitude_bug_m.unwrap_or_else(|| {
            if fallback_alt_m.is_finite() {
                ((fallback_alt_m * FEET_PER_M / 100.0).round() * 100.0 / FEET_PER_M)
                    .clamp(0.0, MAX_SELECTED_ALT_M)
            } else {
                0.0
            }
        });
        let delta = if delta_ft.is_finite() { delta_ft } else { 0.0 };
        let next_m = ((base_m * FEET_PER_M + delta) / FEET_PER_M).clamp(0.0, MAX_SELECTED_ALT_M);
        self.altitude_bug_m = Some(next_m);
        next_m
    }

    /// Locks the altitude bug onto `current_alt_m` rounded to the nearest `100 ft`.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "used by wasm32 touch controls and tests")
    )]
    pub fn sync_altitude_m(&mut self, current_alt_m: f64) -> f64 {
        let next_m = if current_alt_m.is_finite() {
            ((current_alt_m * FEET_PER_M / 100.0).round() * 100.0 / FEET_PER_M)
                .clamp(0.0, MAX_SELECTED_ALT_M)
        } else {
            0.0
        };
        self.altitude_bug_m = Some(next_m);
        next_m
    }

    /// Steps barometric QNH by `delta_hpa` clamped to [`MIN_BARO_HPA`]..=[`MAX_BARO_HPA`].
    pub fn step_baro_hpa(&mut self, delta_hpa: f64) -> f64 {
        let delta = if delta_hpa.is_finite() {
            delta_hpa
        } else {
            0.0
        };
        let next =
            (((self.baro_hpa + delta) * 100.0).round() / 100.0).clamp(MIN_BARO_HPA, MAX_BARO_HPA);
        self.baro_hpa = next;
        next
    }

    /// Resets barometric pressure to standard `1013.25 hPa` (`29.92 inHg`).
    pub fn set_baro_std(&mut self) -> f64 {
        self.baro_hpa = STANDARD_QNH_HPA;
        STANDARD_QNH_HPA
    }

    /// Computes formatted bug and barometric readouts, using `fallback_*` while bugs are unset.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "used by wasm32 touch controls and tests")
    )]
    #[must_use]
    pub fn readout(&self, fallback_hdg_deg: f64, fallback_alt_m: f64) -> DialedAvionicsReadout {
        let hdg = self
            .heading_bug_deg
            .or_else(|| fallback_hdg_deg.is_finite().then_some(fallback_hdg_deg))
            .map(|deg| (wrap_deg(deg).round() as u16) % 360);
        let alt_ft = self
            .altitude_bug_m
            .or_else(|| fallback_alt_m.is_finite().then_some(fallback_alt_m))
            .map(|m| (m * FEET_PER_M).round() as i32);
        DialedAvionicsReadout {
            heading_bug_deg: hdg,
            altitude_bug_ft: alt_ft,
            baro_hpa: self.baro_hpa,
            baro_inhg: self.baro_hpa / HPA_PER_INHG,
        }
    }
}

fn wrap_deg(deg: f64) -> f64 {
    deg.rem_euclid(360.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heading_bug_wraps_cleanly_across_north() {
        let mut dials = PilotDialedSelections::default();
        assert!((dials.step_heading_deg(-10.0, 5.0) - 355.0).abs() < 1e-9);
        assert!((dials.step_heading_deg(15.0, 5.0) - 10.0).abs() < 1e-9);
        assert!((dials.sync_heading_deg(-90.0) - 270.0).abs() < 1e-9);
    }

    #[test]
    fn altitude_bug_steps_and_clamps_to_envelope() {
        let mut dials = PilotDialedSelections::default();
        let initial_m = dials.sync_altitude_m(3_500.0 / FEET_PER_M);
        assert!(((initial_m * FEET_PER_M) - 3_500.0).abs() < 1.0);
        let climbed = dials.step_altitude_ft(500.0, 0.0);
        assert!(((climbed * FEET_PER_M) - 4_000.0).abs() < 1.0);
        let clamped_low = dials.step_altitude_ft(-50_000.0, 0.0);
        assert_eq!(clamped_low, 0.0);
        let clamped_high = dials.step_altitude_ft(100_000.0, 0.0);
        assert_eq!(clamped_high, MAX_SELECTED_ALT_M);
    }

    #[test]
    fn baro_qnh_clamps_and_resets_to_standard() {
        let mut dials = PilotDialedSelections::default();
        assert!((dials.step_baro_hpa(1.0) - 1_014.25).abs() < 1e-9);
        assert!((dials.step_baro_hpa(-1.0) - STANDARD_QNH_HPA).abs() < 1e-9);
        assert_eq!(dials.step_baro_hpa(-500.0), MIN_BARO_HPA);
        assert_eq!(dials.step_baro_hpa(500.0), MAX_BARO_HPA);
        assert_eq!(dials.set_baro_std(), STANDARD_QNH_HPA);
        let readout = dials.readout(40.0, 100.0);
        assert!((readout.baro_inhg - 29.92).abs() < 0.01);
        assert_eq!(readout.heading_bug_deg, Some(40));
    }
}

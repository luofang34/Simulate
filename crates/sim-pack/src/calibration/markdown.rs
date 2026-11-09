//! Markdown report formatting for [`CalibrationReport`], [`SensitivityReport`],
//! and [`DesignPerformanceReport`].

use crate::calibration::results::{CalibrationReport, DesignPerformanceReport, SensitivityReport};

impl CalibrationReport {
    /// Formats a human-readable Markdown engineering report suitable for CLI output or artifacts.
    #[must_use]
    pub fn to_markdown(&self) -> String {
        let mut out = String::with_capacity(4_096);
        let verdict = if self.all_passed { "PASS" } else { "FAIL" };
        out.push_str(&format!(
            "# Closed-Loop Calibration Report — `{}` (`{verdict}`)\n\n- **Reference**: {}\n",
            self.type_designator, self.reference_document
        ));
        if let Some(tail) = &self.tail_profile {
            out.push_str(&format!(
                "- **Tail Anchor**: `{}` ({} S/N `{}`, base `{}`, BEW `{:.1} kg` @ `{:.1} in`)\n- **Powerplant**: {}\n- **Avionics**: {}\n",
                tail.registration,
                tail.model_year,
                tail.serial_number,
                tail.home_base_icao,
                tail.basic_empty_mass_kg,
                tail.basic_empty_cg_in,
                tail.powerplant_summary,
                tail.avionics_suite
            ));
        }
        out.push_str(&format!(
            "- **Summary RMSEs**: Stall KCAS `{:.2} kt` | Stall KIAS `{:.2} kt` | Climb `{:.1} fpm` | Cruise `{:.2} KTAS` | Fuel `{:.2} GPH` | Empirical GS `{:.2} kt`\n\n",
            self.stall_kcas_rmse_kt,
            self.stall_kias_rmse_kt,
            self.climb_rmse_fpm,
            self.cruise_rmse_ktas,
            self.cruise_rmse_gph,
            self.empirical_gs_rmse_kt
        ));
        self.append_runup_and_stalls(&mut out);
        self.append_climb_and_cruise(&mut out);
        self.append_takeoffs(&mut out);
        out
    }

    fn append_runup_and_stalls(&self, out: &mut String) {
        if let Some((rpm, lo, hi, ok)) = self.static_runup {
            out.push_str(&format!(
                "## Static Ground Runup\n- Simulated WOT Static RPM: **`{rpm:.0} RPM`** (certificated `{lo:.0}–{hi:.0} RPM`, `{}`)\n\n",
                pass_str(ok)
            ));
        }
        if !self.stalls.is_empty() {
            out.push_str(
                "## Power-Off Stalls\n| Condition | Sim KCAS | POH KCAS | Sim KIAS | POH KIAS | Status |\n|---|---:|---:|---:|---:|:---:|\n",
            );
            for s in &self.stalls {
                let sk = s
                    .sim_kias
                    .map_or_else(|| "-".to_owned(), |v| format!("{v:.1}"));
                let tk = s
                    .target_kias
                    .map_or_else(|| "-".to_owned(), |v| format!("{v:.1}"));
                out.push_str(&format!(
                    "| {} | {:.1} | {:.1} | {} | {} | {} |\n",
                    s.label,
                    s.sim_kcas,
                    s.target_kcas,
                    sk,
                    tk,
                    pass_str(s.passed)
                ));
            }
            out.push('\n');
        }
    }

    fn append_climb_and_cruise(&self, out: &mut String) {
        if !self.climbs.is_empty() {
            out.push_str(
                "## Full-Throttle Rate of Climb\n| Alt (ft) | Sim KCAS | RPM | Sim ROC (fpm) | POH ROC (fpm) | Residual (fpm) | Status |\n|---:|---:|---:|---:|---:|---:|:---:|\n",
            );
            for c in &self.climbs {
                out.push_str(&format!(
                    "| {:.0} | {:.1} | {:.0} | {:.0} | {:.0} | {:+.0} | {} |\n",
                    c.pressure_alt_ft,
                    c.sim_kcas,
                    c.sim_rpm,
                    c.sim_fpm,
                    c.target_fpm,
                    c.residual_fpm,
                    pass_str(c.passed)
                ));
            }
            out.push('\n');
        }
        if !self.cruises.is_empty() {
            out.push_str(
                "## Steady Level Cruise\n| Alt (ft) | Cmd RPM | Sim %BHP | Sim KTAS | POH KTAS | ΔKTAS | Sim GPH | POH GPH | Status |\n|---:|---:|---:|---:|---:|---:|---:|---:|:---:|\n",
            );
            for cr in &self.cruises {
                let bhp = cr
                    .sim_pct_bhp
                    .map_or_else(|| "-".to_owned(), |v| format!("{v:.1}%"));
                let sg = cr
                    .sim_gph
                    .map_or_else(|| "-".to_owned(), |v| format!("{v:.2}"));
                let tg = cr
                    .target_gph
                    .map_or_else(|| "-".to_owned(), |v| format!("{v:.2}"));
                out.push_str(&format!(
                    "| {:.0} | {:.0} | {} | {:.1} | {:.1} | {:+.1} | {} | {} | {} |\n",
                    cr.pressure_alt_ft,
                    cr.commanded_rpm,
                    bhp,
                    cr.sim_ktas,
                    cr.target_ktas,
                    cr.residual_ktas,
                    sg,
                    tg,
                    pass_str(cr.passed)
                ));
            }
            out.push('\n');
        }
        if !self.empirical_segments.is_empty() {
            out.push_str(
                "## Empirical Flight-Segment Closed Loop\n| Segment | Alt (ft) | Sim GS (kt) | Obs GS (kt) | ΔGS (kt) | Sim ROC (fpm) | Status |\n|---|---:|---:|---:|---:|---:|:---:|\n",
            );
            for e in &self.empirical_segments {
                out.push_str(&format!(
                    "| `{}` | {:.0} | {:.1} | {:.1} | {:+.1} | {:.0} | {} |\n",
                    e.segment_id,
                    e.pressure_alt_ft,
                    e.sim_gs_kt,
                    e.observed_gs_kt,
                    e.residual_gs_kt,
                    e.sim_roc_fpm,
                    pass_str(e.passed)
                ));
            }
        }
    }

    fn append_takeoffs(&self, out: &mut String) {
        if !self.takeoffs.is_empty() {
            out.push_str(
                "\n## Takeoff Distance\n| Condition | Sim roll (ft) | POH roll (ft) | Sim 50 ft (ft) | POH 50 ft (ft) | Status |\n|---|---:|---:|---:|---:|:---:|\n",
            );
            for t in &self.takeoffs {
                out.push_str(&format!(
                    "| {} | {:.0} | {:.0} | {:.0} | {:.0} | {} |\n",
                    t.label,
                    t.sim_ground_roll_ft,
                    t.target_ground_roll_ft,
                    t.sim_total_ft,
                    t.target_total_ft,
                    pass_str(t.passed)
                ));
            }
        }
    }
}

impl SensitivityReport {
    /// Formats both the baseline [`CalibrationReport`] and the Jacobian Gauss-Newton
    /// parameter-identification summary in Markdown.
    #[must_use]
    pub fn to_markdown(&self) -> String {
        let mut out = self.baseline.to_markdown();
        out.push_str(&format!(
            "\n## Jacobian Sensitivity & Damped Gauss-Newton Parameter Identification\n- **Normalized Weighted Cost**: `{:.4}` → linearized `{:.4}`\n\n| Parameter | Step | ∂ROC/∂θ (fpm) | ∂KTAS/∂θ (kt) | ∂GPH/∂θ | Recommended Δθ |\n|---|---:|---:|---:|---:|---:|\n",
            self.baseline_cost, self.linearized_predicted_cost
        ));
        for a in &self.adjustments {
            out.push_str(&format!(
                "| `{}` | {:+.4} | {:+.1} | {:+.2} | {:+.3} | {:+.5} |\n",
                a.parameter,
                a.perturbation_step,
                a.mean_d_climb_fpm,
                a.mean_d_cruise_ktas,
                a.mean_d_cruise_gph,
                a.recommended_delta
            ));
        }
        out
    }
}

impl DesignPerformanceReport {
    /// Formats the synthesized V-speed schedule, climb ceiling, glide polar,
    /// cruise specific range, endurance, and payload–range envelope in Markdown.
    #[must_use]
    pub fn to_markdown(&self) -> String {
        let mut out = String::with_capacity(4_096);
        out.push_str(&format!(
            "# New-Design Performance & Range Estimate — `{}` ({})\n\n",
            self.package_id, self.package_name
        ));
        out.push_str(&format!(
            "- **Mass Breakdown**: Empty `{:.1} kg` | Usable Fuel `{:.1} kg` (`{:.1} US gal`) | Nominal Payload `{:.1} kg` | Ref Gross `{:.1} kg`\n",
            self.empty_mass_kg, self.usable_fuel_kg, self.usable_fuel_gal, self.nominal_payload_kg, self.reference_gross_mass_kg
        ));
        out.push_str(&format!(
            "- **Synthesized V-Speeds (KCAS)**: $V_{{S0}} = {:.1}$ | $V_{{S1}} = {:.1}$ | $V_X = {:.1}$ ($\\gamma_{{\\max}} = {:.1}^\\circ$) | $V_Y = {:.1}$ (`{:.0} fpm` SL) | $V_G = {:.1}$ ($L/D = {:.2}$, `{:.2} NM / 1,000 ft`) | $V_A = {:.1}$ | $V_{{\\max,\\text{{SL}}}} = {:.1}\\text{{ KTAS}}$\n",
            self.vs0_kcas, self.vs1_kcas, self.vx_kcas, self.max_climb_angle_deg, self.vy_kcas, self.sea_level_roc_fpm, self.vg_kcas, self.max_lift_to_drag, self.glide_nm_per_1000ft, self.va_kcas, self.max_level_speed_sl_ktas
        ));
        out.push_str(&format!(
            "- **Estimated Ceilings**: Service Ceiling (`100 fpm`) **`{:.0} ft`** | Absolute Ceiling **`{:.0} ft`**\n\n",
            self.service_ceiling_ft, self.absolute_ceiling_ft
        ));
        self.append_climb_and_cruise_tables(&mut out);
        self.append_mission_profile_tables(&mut out);
        out
    }

    fn append_climb_and_cruise_tables(&self, out: &mut String) {
        out.push_str("## Climb Profile vs. Altitude\n| Alt (ft) | KCAS | KTAS | ROC (fpm) | Climb Angle (deg) | Gradient (ft/NM) |\n|---:|---:|---:|---:|---:|---:|\n");
        for c in &self.climb_profile {
            out.push_str(&format!(
                "| {:.0} | {:.1} | {:.1} | {:.0} | {:.2} | {:.0} |\n",
                c.pressure_alt_ft,
                c.kcas,
                c.ktas,
                c.roc_fpm,
                c.climb_angle_deg,
                c.gradient_ft_per_nm
            ));
        }
        out.push_str("\n## Cruise Polar, Specific Range & Endurance (`45-min` Reserve)\n| Regime | Alt (ft) | KTAS | Fuel (GPH) | Spec. Range (NM/gal) | Endurance (hr) | Range w/ Res (NM) | Dry Range (NM) |\n|---|---:|---:|---:|---:|---:|---:|---:|\n");
        for cr in &self.cruise_polar {
            out.push_str(&format!(
                "| {} | {:.0} | {:.1} | {:.2} | {:.2} | {:.2} | {:.0} | {:.0} |\n",
                cr.label,
                cr.pressure_alt_ft,
                cr.ktas,
                cr.fuel_flow_gph,
                cr.specific_range_nm_per_gal,
                cr.endurance_45min_res_hr,
                cr.range_45min_res_nm,
                cr.range_no_res_nm
            ));
        }
        out.push_str("\n## Payload–Range Envelope (`45-min` Reserve)\n| Mission Corner | Payload (kg) | Fuel (kg) | TOW (kg) | Cruise (KTAS) | Still-Air Range (NM) |\n|---|---:|---:|---:|---:|---:|\n");
        for p in &self.payload_range {
            out.push_str(&format!(
                "| {} | {:.1} | {:.1} | {:.1} | {:.1} | {:.0} |\n",
                p.label,
                p.payload_kg,
                p.fuel_kg,
                p.takeoff_mass_kg,
                p.cruise_ktas,
                p.range_45min_res_nm
            ));
        }
    }

    fn append_mission_profile_tables(&self, out: &mut String) {
        if self.mission_profiles.is_empty() {
            return;
        }
        out.push_str("\n## Multi-Altitude & Mid-Mission Payload-Drop Profiles\n| Profile | TOW (kg) | Dropped (kg) | Mission Fuel (kg) | Reserve (kg) | Block (hr) | Action Radius (NM) | No-Drop Radius (NM) | Drop Bonus (NM) | Post-Drop ROC Gain (fpm) |\n|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n");
        for m in &self.mission_profiles {
            let rad = m
                .action_radius_nm
                .map_or_else(|| "-".to_owned(), |v| format!("{v:.0}"));
            let nodrop = m
                .no_drop_action_radius_nm
                .map_or_else(|| "-".to_owned(), |v| format!("{v:.0}"));
            let bonus = m
                .drop_radius_bonus_nm
                .map_or_else(|| "-".to_owned(), |v| format!("+{v:.0}"));
            let roc_gain = m
                .post_drop_roc_gain_fpm
                .map_or_else(|| "-".to_owned(), |v| format!("+{v:.0}"));
            out.push_str(&format!(
                "| {} (`{}`) | {:.1} | {:.1} | {:.1} | {:.1} | {:.2} | **{}** | {} | {} | {} |\n",
                m.name,
                m.id,
                m.initial_mass_kg,
                m.total_payload_dropped_kg,
                m.total_fuel_burn_kg,
                m.reserve_fuel_kg,
                m.total_duration_hr,
                rad,
                nodrop,
                bonus,
                roc_gain
            ));
        }
        if let Some(hilo) = self
            .mission_profiles
            .iter()
            .find(|m| m.id == "hi_lo_drop_lo_hi")
        {
            out.push_str("\n### Segment Breakdown — `Hi-Lo-Drop-Lo-Hi` Profile\n| Segment | Kind | Alt (ft) | Start Mass (kg) | Dropped (kg) | End Mass (kg) | KTAS | Fuel (kg / gal) | Time (min) | Dist (NM) |\n|---|---|---:|---:|---:|---:|---:|---|---:|---:|\n");
            for s in &hilo.segments {
                out.push_str(&format!(
                    "| `{}` | {} | {:.0} | {:.1} | {:.1} | {:.1} | {:.1} | {:.1} / {:.2} | {:.1} | {:.1} |\n",
                    s.id,
                    s.kind_label,
                    s.alt_ft,
                    s.start_mass_kg,
                    s.payload_dropped_kg,
                    s.end_mass_kg,
                    s.ktas,
                    s.fuel_burn_kg,
                    s.fuel_burn_gal,
                    s.duration_min,
                    s.distance_nm
                ));
            }
        }
    }
}

const fn pass_str(ok: bool) -> &'static str {
    if ok { "PASS" } else { "FAIL" }
}

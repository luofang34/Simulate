//! Run-level statistical accumulation: `P50` / `P95` / `RMSE` / `Max`,
//! chi-square `NEES` bands, position-source dwell percentages, spoofing
//! `TTD` / `TTR`, and RNP containment fraction.

use std::collections::BTreeMap;

use super::nees::summarize_nees;
use super::types::{
    MetricSummary, NavigatorSummary, NavigatorTelemetry, SpoofingSummary, TELEMETRY_SCHEMA,
    TelemetrySample, TelemetrySummary,
};

/// Accumulates per-sample telemetry across a run to produce [`TelemetrySummary`].
#[derive(Debug, Default)]
pub(super) struct SummaryAccumulator {
    sample_count: usize,
    first_t_s: Option<f64>,
    last_t_s: f64,
    last_steered: String,
    last_rnp_nm: f64,
    last_rnp_m: f64,
    nse_abs_m: Vec<f64>,
    fte_abs_m: Vec<f64>,
    tse_abs_m: Vec<f64>,
    rnp_evaluated: usize,
    rnp_contained: usize,
    spoof_start_s: Option<f64>,
    spoof_end_s: Option<f64>,
    prev_spoof_active: bool,
    navs: BTreeMap<String, NavAccumulator>,
}

#[derive(Debug, Default)]
struct NavAccumulator {
    horiz_err_m: Vec<f64>,
    vert_err_m: Vec<f64>,
    nees_2d: Vec<f64>,
    nees_3d: Vec<f64>,
    source_counts: BTreeMap<String, usize>,
    detected_at_s: Option<f64>,
    excluded_at_s: Option<f64>,
    max_err_before_excl_m: Option<f64>,
    recovered_at_s: Option<f64>,
    degraded_during_spoof: bool,
}

impl SummaryAccumulator {
    pub(super) fn push(&mut self, sample: &TelemetrySample) {
        let t_s = sample.truth.t_s;
        self.sample_count = self.sample_count.wrapping_add(1);
        self.first_t_s.get_or_insert(t_s);
        self.last_t_s = t_s;
        self.last_steered
            .clone_from(&sample.guidance_rnp.steered_set);
        self.last_rnp_nm = sample.guidance_rnp.rnp_limit_nm;
        self.last_rnp_m = sample.guidance_rnp.rnp_limit_m;

        if let Some(nse) = sample.guidance_rnp.nse_cross_track_m {
            self.nse_abs_m.push(nse.abs());
        }
        if let Some(fte) = sample.guidance_rnp.fte_cross_track_m {
            self.fte_abs_m.push(fte.abs());
        }
        if let Some(tse) = sample.guidance_rnp.tse_cross_track_m {
            self.tse_abs_m.push(tse.abs());
        }
        if let Some(within) = sample.guidance_rnp.within_rnp {
            self.rnp_evaluated = self.rnp_evaluated.wrapping_add(1);
            if within {
                self.rnp_contained = self.rnp_contained.wrapping_add(1);
            }
        }

        let spoof_active = sample.inputs.spoof_active;
        if spoof_active && !self.prev_spoof_active {
            self.spoof_start_s.get_or_insert(t_s);
            // A subsequent spoofing window resets post-spoof recovery tracking.
            self.spoof_end_s = None;
            for nav in self.navs.values_mut() {
                nav.recovered_at_s = None;
            }
        } else if !spoof_active && self.prev_spoof_active {
            self.spoof_end_s = Some(t_s);
        }
        self.prev_spoof_active = spoof_active;

        for nav in &sample.navigators {
            let entry = self.navs.entry(nav.set.clone()).or_default();
            entry.push(t_s, spoof_active, self.spoof_start_s, self.spoof_end_s, nav);
        }
    }

    pub(super) fn finish(&self, seed: Option<u64>) -> TelemetrySummary {
        let duration_s = self
            .first_t_s
            .map_or(0.0, |first| (self.last_t_s - first).max(0.0));
        let mut per_navigator = BTreeMap::new();
        for (name, acc) in &self.navs {
            per_navigator.insert(
                name.clone(),
                acc.build(self.spoof_start_s, self.spoof_end_s),
            );
        }
        let steered = per_navigator.get(&self.last_steered);
        let rnp_containment_fraction = if self.rnp_evaluated == 0 {
            1.0
        } else {
            self.rnp_contained as f64 / self.rnp_evaluated as f64
        };
        TelemetrySummary {
            schema: TELEMETRY_SCHEMA,
            seed,
            sample_count: self.sample_count,
            duration_s,
            steered_set: self.last_steered.clone(),
            rnp_limit_nm: self.last_rnp_nm,
            rnp_limit_m: self.last_rnp_m,
            horizontal_error_m: steered
                .map_or_else(MetricSummary::default, |s| s.horizontal_error_m),
            nse_cross_track_m: summarize_series(&self.nse_abs_m),
            fte_cross_track_m: summarize_series(&self.fte_abs_m),
            tse_cross_track_m: summarize_series(&self.tse_abs_m),
            nees_horiz_2d: steered.and_then(|s| s.nees_horiz_2d),
            nees_pos_3d: steered.and_then(|s| s.nees_pos_3d),
            position_source_pct: steered
                .map_or_else(BTreeMap::new, |s| s.position_source_pct.clone()),
            rnp_containment_fraction,
            spoofing: steered.map_or_else(
                || SpoofingSummary {
                    spoof_injected: self.spoof_start_s.is_some(),
                    spoof_start_s: self.spoof_start_s,
                    spoof_end_s: self.spoof_end_s,
                    ..SpoofingSummary::default()
                },
                |s| s.spoofing,
            ),
            per_navigator,
        }
    }
}

impl NavAccumulator {
    fn push(
        &mut self,
        t_s: f64,
        spoof_active: bool,
        spoof_start_s: Option<f64>,
        spoof_end_s: Option<f64>,
        nav: &NavigatorTelemetry,
    ) {
        if nav.horizontal_error_m.is_finite() {
            self.horiz_err_m.push(nav.horizontal_error_m);
        }
        if nav.vertical_error_m.is_finite() {
            self.vert_err_m.push(nav.vertical_error_m);
        }
        if let Some(n2) = nav.nees_horiz_2d.filter(|v| v.is_finite()) {
            self.nees_2d.push(n2);
        }
        if let Some(n3) = nav.nees_3d_finite() {
            self.nees_3d.push(n3);
        }
        let count = self
            .source_counts
            .entry(nav.position_source.clone())
            .or_default();
        *count = count.wrapping_add(1);

        if let Some(start_s) = spoof_start_s
            && t_s >= start_s
        {
            let already_excluded = self.excluded_at_s.is_some();
            if !already_excluded && (spoof_active || self.degraded_during_spoof) {
                let prev = self.max_err_before_excl_m.unwrap_or(0.0);
                self.max_err_before_excl_m = Some(prev.max(nav.horizontal_error_m));
            }
            let non_normal = nav.gnss_integrity == "degraded" || nav.gnss_integrity == "excluded";
            if non_normal {
                self.detected_at_s.get_or_insert(t_s);
                self.degraded_during_spoof = true;
            }
            if nav.gnss_integrity == "excluded" {
                self.excluded_at_s.get_or_insert(t_s);
            }
        }
        if !spoof_active
            && let Some(end_s) = spoof_end_s
            && t_s >= end_s
            && self.degraded_during_spoof
            && nav.gnss_integrity == "normal"
        {
            self.recovered_at_s.get_or_insert(t_s);
        }
    }

    fn build(&self, spoof_start_s: Option<f64>, spoof_end_s: Option<f64>) -> NavigatorSummary {
        let total = self.source_counts.values().copied().sum::<usize>();
        let mut position_source_pct = BTreeMap::new();
        if total > 0 {
            for (src, count) in &self.source_counts {
                position_source_pct.insert(src.clone(), (*count as f64 * 100.0) / total as f64);
            }
        }
        let nees_2d_stats = summarize_series(&self.nees_2d);
        let nees_3d_stats = summarize_series(&self.nees_3d);
        let spoofing = SpoofingSummary {
            spoof_injected: spoof_start_s.is_some(),
            spoof_start_s,
            spoof_end_s,
            time_to_detect_s: spoof_start_s
                .zip(self.detected_at_s)
                .map(|(s, d)| (d - s).max(0.0)),
            time_to_exclude_s: spoof_start_s
                .zip(self.excluded_at_s)
                .map(|(s, e)| (e - s).max(0.0)),
            max_error_before_exclusion_m: self.max_err_before_excl_m,
            time_to_recover_s: spoof_end_s
                .zip(self.recovered_at_s)
                .map(|(end, rec)| (rec - end).max(0.0)),
        };
        NavigatorSummary {
            horizontal_error_m: summarize_series(&self.horiz_err_m),
            vertical_error_m: summarize_series(&self.vert_err_m),
            nees_horiz_2d: summarize_nees(&self.nees_2d, 2, nees_2d_stats.p95),
            nees_pos_3d: summarize_nees(&self.nees_3d, 3, nees_3d_stats.p95),
            position_source_pct,
            spoofing,
        }
    }
}

impl NavigatorTelemetry {
    fn nees_3d_finite(&self) -> Option<f64> {
        self.nees_pos_3d.filter(|v| v.is_finite())
    }
}

/// Computes `P50`, `P95`, `RMSE`, and `Max` for a finite non-negative sample slice.
pub(super) fn summarize_series(values: &[f64]) -> MetricSummary {
    let mut sorted: Vec<f64> = values.iter().copied().filter(|v| v.is_finite()).collect();
    if sorted.is_empty() {
        return MetricSummary::default();
    }
    sorted.sort_by(f64::total_cmp);
    let count = sorted.len();
    let sum_sq: f64 = sorted.iter().map(|v| v * v).sum();
    let rmse = (sum_sq / count as f64).sqrt();
    let max = sorted.last().copied().unwrap_or(0.0);
    MetricSummary {
        count,
        p50: percentile_sorted(&sorted, 0.50),
        p95: percentile_sorted(&sorted, 0.95),
        rmse,
        max,
    }
}

fn percentile_sorted(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let last = sorted.len().saturating_sub(1);
    let pos = (q.clamp(0.0, 1.0) * last as f64).round() as usize;
    sorted.get(pos).copied().unwrap_or(0.0)
}

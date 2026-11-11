//! What each means contributes: every aided set against the best set that
//! aids the inertial core with fewer of its means.

use serde::{Deserialize, Serialize};

use super::{SensorSetSpec, SetReport};
use crate::{AblationSet, NavMeans, NavigationOrigin};

/// One set's contribution, read off the sets of the same run.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Contribution {
    /// How Navigate's ablation scoring classes the set: one means on its own
    /// (`standalone`, the inertial core alone or with stars) or a fusion
    /// (`fused`).
    pub origin: String,
    /// The aiding means on top of the inertial core.
    pub aiding: Vec<NavMeans>,
    /// The set it is compared against: the best set aided by one of its
    /// means (for a fused set) or the inertial core alone (for a set aided
    /// by one means).
    pub baseline_set: Option<String>,
    /// The baseline's horizontal RMS error, metres.
    pub baseline_horizontal_rms_m: Option<f64>,
    /// This set's horizontal RMS minus the baseline's, metres: negative is
    /// better.
    pub horizontal_rms_change_m: Option<f64>,
    /// This set's horizontal RMS over the baseline's.
    pub horizontal_rms_ratio: Option<f64>,
}

/// Navigate's ablation origin of a set: a means on its own when the set is
/// that means' minimal unit, a fusion otherwise.
pub fn ablation_origin(set: AblationSet) -> NavigationOrigin {
    NavMeans::ALL
        .into_iter()
        .find(|means| means.minimal_unit() == set)
        .map_or(NavigationOrigin::Fused(set), NavigationOrigin::Standalone)
}

fn aiding(set: &SensorSetSpec) -> Vec<NavMeans> {
    set.ablation_set()
        .means()
        .filter(|m| *m != NavMeans::DeadReckoning)
        .collect()
}

/// Fills each report's contribution from its peers. Sets that ignore GNSS
/// outages are compared only with each other, and so are sets that honour
/// them.
pub fn fill_contributions(specs: &[SensorSetSpec], reports: &mut [SetReport]) {
    let rms: Vec<f64> = reports
        .iter()
        .map(|r| r.summary.horizontal_error_m.rms)
        .collect();
    let contributions: Vec<Option<Contribution>> = specs
        .iter()
        .map(|spec| {
            let mine = aiding(spec);
            if mine.is_empty() {
                return None;
            }
            let wanted = if mine.len() == 1 { 0 } else { 1 };
            let baseline = specs
                .iter()
                .zip(&rms)
                .filter(|(other, _)| other.gnss_outages == spec.gnss_outages)
                .filter(|(other, _)| {
                    let theirs = aiding(other);
                    theirs.len() == wanted && theirs.iter().all(|m| mine.contains(m))
                })
                .min_by(|a, b| a.1.total_cmp(b.1));
            let own = specs
                .iter()
                .position(|s| s.name == spec.name)
                .and_then(|i| rms.get(i).copied());
            let origin = match ablation_origin(spec.ablation_set()) {
                NavigationOrigin::Standalone(_) => "standalone",
                NavigationOrigin::Fused(_) => "fused",
            };
            Some(Contribution {
                origin: origin.into(),
                aiding: mine,
                baseline_set: baseline.map(|(s, _)| s.name.clone()),
                baseline_horizontal_rms_m: baseline.map(|(_, r)| *r),
                horizontal_rms_change_m: own.zip(baseline).map(|(o, (_, b))| o - b),
                horizontal_rms_ratio: own.zip(baseline).map(|(o, (_, b))| o / b),
            })
        })
        .collect();
    for (report, contribution) in reports.iter_mut().zip(contributions) {
        report.contribution = contribution;
    }
}

/// Every aiding means alone on the inertial core, and their fusions, for a
/// run with `visual` and stars available.
pub fn default_sensor_sets(visual: bool) -> Vec<SensorSetSpec> {
    use NavMeans::{Barometric, Celestial, DeadReckoning, Gnss, Visual};
    let set = |name: &str, means: &[NavMeans], gnss_outages: bool| SensorSetSpec {
        name: name.into(),
        means: means.to_vec(),
        gnss_outages,
    };
    let all = [DeadReckoning, Barometric, Gnss, Celestial, Visual];
    [
        set("ins", &[DeadReckoning], false),
        set("ins+baro", &[DeadReckoning, Barometric], false),
        set("ins+gnss", &[DeadReckoning, Gnss], false),
        set("ins+stars", &[DeadReckoning, Celestial], false),
        set("ins+visual", &[DeadReckoning, Visual], false),
        set(
            "ins+baro+stars",
            &[DeadReckoning, Barometric, Celestial],
            false,
        ),
        set(
            "ins+baro+visual",
            &[DeadReckoning, Barometric, Visual],
            false,
        ),
        set(
            "ins+baro+stars+visual",
            &[DeadReckoning, Barometric, Celestial, Visual],
            false,
        ),
        set("all", &all, false),
        set("all-gnss-outage", &all, true),
    ]
    .into_iter()
    .filter(|s| visual || !s.name.contains("visual"))
    .map(|mut s| {
        if !visual {
            s.means.retain(|m| *m != Visual);
        }
        s
    })
    .collect()
}

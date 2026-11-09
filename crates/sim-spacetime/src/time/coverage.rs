//! Disjoint sorted `TDB` coverage sets for segmented SPK and PCK chains.

use serde::{Deserialize, Serialize};

use super::TdbEpoch;

/// Closed `TDB` coverage span `[start_ns, end_ns]` in integer nanoseconds from `J2000.0 TDB`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct TdbSpanNs {
    /// Inclusive lower bound in nanoseconds from `J2000.0 TDB`.
    pub start_ns: i128,
    /// Inclusive upper bound in nanoseconds from `J2000.0 TDB` (`end_ns >= start_ns`).
    pub end_ns: i128,
}

impl TdbSpanNs {
    /// Constructs a validated closed span, returning `None` if `end_ns < start_ns`.
    #[must_use]
    pub const fn new(start_ns: i128, end_ns: i128) -> Option<Self> {
        if end_ns < start_ns {
            None
        } else {
            Some(Self { start_ns, end_ns })
        }
    }

    /// Constructs a conservative inner-rounded span from `f64` seconds past `J2000.0 TDB`.
    #[must_use]
    pub fn from_j2000_seconds_inner(start_s: f64, end_s: f64) -> Option<Self> {
        let start = TdbEpoch::from_j2000_seconds_f64(start_s)?.to_j2000_nanos_ceil();
        let end = TdbEpoch::from_j2000_seconds_f64(end_s)?.to_j2000_nanos_floor();
        Self::new(start, end)
    }

    /// Whether `epoch` falls inside `[start_ns, end_ns]`.
    #[must_use]
    pub const fn contains(self, epoch: TdbEpoch) -> bool {
        let lo = epoch.to_j2000_nanos_floor();
        let hi = epoch.to_j2000_nanos_ceil();
        lo >= self.start_ns && hi <= self.end_ns
    }

    /// Intersection of two closed intervals, if non-empty.
    #[must_use]
    pub fn intersect(self, other: Self) -> Option<Self> {
        let start_ns = self.start_ns.max(other.start_ns);
        let end_ns = self.end_ns.min(other.end_ns);
        Self::new(start_ns, end_ns)
    }
}

/// Normalized set of sorted, non-overlapping [`TdbSpanNs`] intervals.
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CoverageSet {
    spans: Vec<TdbSpanNs>,
}

impl CoverageSet {
    /// An empty coverage set.
    #[must_use]
    pub const fn empty() -> Self {
        Self { spans: Vec::new() }
    }

    /// Coverage spanning practically unbounded `i128` TDB time (for analytic models).
    #[must_use]
    pub fn unbounded() -> Self {
        Self {
            spans: vec![TdbSpanNs {
                start_ns: i128::MIN / 2,
                end_ns: i128::MAX / 2,
            }],
        }
    }

    /// Builds a normalized union of arbitrary `spans` (merging touching or overlapping ranges).
    #[must_use]
    pub fn from_spans(mut spans: Vec<TdbSpanNs>) -> Self {
        spans.retain(|s| s.end_ns >= s.start_ns);
        spans.sort_by_key(|s| (s.start_ns, s.end_ns));
        let mut merged: Vec<TdbSpanNs> = Vec::with_capacity(spans.len());
        for span in spans {
            if let Some(last) = merged.last_mut()
                && span.start_ns <= last.end_ns
            {
                last.end_ns = last.end_ns.max(span.end_ns);
                continue;
            }
            merged.push(span);
        }
        Self { spans: merged }
    }

    /// Sorted disjoint intervals in this coverage set.
    #[must_use]
    pub fn spans(&self) -> &[TdbSpanNs] {
        &self.spans
    }

    /// Whether the coverage set has zero valid intervals.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.spans.is_empty()
    }

    /// Whether `epoch` lies inside any interval of this set.
    #[must_use]
    pub fn contains(&self, epoch: TdbEpoch) -> bool {
        self.spans.iter().any(|span| span.contains(epoch))
    }

    /// Whether the entire closed interval `span` is covered without interior gaps.
    #[must_use]
    pub fn covers_span(&self, span: TdbSpanNs) -> bool {
        self.spans
            .iter()
            .any(|s| s.start_ns <= span.start_ns && s.end_ns >= span.end_ns)
    }

    /// Exact set union `self ∪ other`.
    #[must_use]
    pub fn union(&self, other: &Self) -> Self {
        let mut combined = Vec::with_capacity(self.spans.len().saturating_add(other.spans.len()));
        combined.extend_from_slice(&self.spans);
        combined.extend_from_slice(&other.spans);
        Self::from_spans(combined)
    }

    /// Exact set intersection `self ∩ other` (for multi-segment ephemeris chains).
    #[must_use]
    pub fn intersect(&self, other: &Self) -> Self {
        let mut out = Vec::new();
        let mut i = 0_usize;
        let mut j = 0_usize;
        while let (Some(&a), Some(&b)) = (self.spans.get(i), other.spans.get(j)) {
            if let Some(ov) = a.intersect(b) {
                out.push(ov);
            }
            if a.end_ns < b.end_ns {
                i = i.wrapping_add(1);
            } else {
                j = j.wrapping_add(1);
            }
        }
        Self::from_spans(out)
    }
}

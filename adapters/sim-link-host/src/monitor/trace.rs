//! One set's error over route time, and its reduction to the columns of a
//! plot: a flight of an hour at the trial's rate is thousands of samples,
//! and a plot a few hundred pixels wide draws one column per pixel.

use sim_navigate::NavMeans;
use sim_navigate::trial::ProgressSet;

/// One sample of a set.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TracePoint {
    /// Route time, seconds.
    pub t_s: f64,
    /// Horizontal distance from the truth, metres.
    pub horizontal_error_m: f64,
    /// The set's own one-sigma horizontal uncertainty, metres; `None` for
    /// a set that states none.
    pub sigma_horizontal_m: Option<f64>,
    /// The set's estimated latitude and longitude, degrees.
    pub estimate_deg: (f64, f64),
}

/// What one column of a plot covers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Column {
    /// The smallest error in the column, metres.
    pub min_m: f64,
    /// The largest error in the column, metres.
    pub max_m: f64,
    /// The largest one-sigma uncertainty stated in the column, metres.
    pub sigma_m: Option<f64>,
}

/// A set and its samples in route-time order.
#[derive(Debug, Clone, PartialEq)]
pub struct SetTrace {
    /// The set's name.
    pub name: String,
    /// Its means.
    pub means: Vec<NavMeans>,
    points: Vec<TracePoint>,
}

impl SetTrace {
    /// An empty trace of `set`.
    pub fn new(set: ProgressSet) -> Self {
        Self {
            name: set.name,
            means: set.means,
            points: Vec::new(),
        }
    }

    /// Adds a sample. One older than the last is dropped: the stream is in
    /// route-time order, and the columns rely on it.
    pub fn push(&mut self, point: TracePoint) {
        if self.points.last().is_none_or(|last| point.t_s >= last.t_s) {
            self.points.push(point);
        }
    }

    /// The samples.
    pub fn points(&self) -> &[TracePoint] {
        &self.points
    }

    /// The latest sample.
    pub fn latest(&self) -> Option<&TracePoint> {
        self.points.last()
    }

    /// The largest error so far, metres.
    pub fn max_error_m(&self) -> Option<f64> {
        self.points
            .iter()
            .map(|point| point.horizontal_error_m)
            .reduce(f64::max)
    }

    /// The samples from `t0_s` to `t1_s` in `count` equal columns; `None`
    /// for a column no sample falls in. A sample at `t1_s` belongs to the
    /// last column.
    pub fn columns(&self, t0_s: f64, t1_s: f64, count: usize) -> Vec<Option<Column>> {
        let mut columns = vec![None; count];
        let span = t1_s - t0_s;
        if count == 0 || span.is_nan() || span <= 0.0 {
            return columns;
        }
        let last = count.saturating_sub(1);
        let start = self.points.partition_point(|point| point.t_s < t0_s);
        let in_range = self.points.get(start..).unwrap_or_default();
        for point in in_range.iter().take_while(|point| point.t_s <= t1_s) {
            let at = ((point.t_s - t0_s) / span * count as f64).floor() as usize;
            let Some(column) = columns.get_mut(at.min(last)) else {
                continue;
            };
            let error = point.horizontal_error_m;
            *column = Some(match column.take() {
                None => Column {
                    min_m: error,
                    max_m: error,
                    sigma_m: point.sigma_horizontal_m,
                },
                Some(seen) => Column {
                    min_m: seen.min_m.min(error),
                    max_m: seen.max_m.max(error),
                    sigma_m: match (seen.sigma_m, point.sigma_horizontal_m) {
                        (Some(a), Some(b)) => Some(a.max(b)),
                        (a, b) => a.or(b),
                    },
                },
            });
        }
        columns
    }
}

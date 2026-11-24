//! CK type 2: piecewise-constant angular velocity intervals.

use alloc::vec::Vec;

use super::CkPointing;
use super::quaternion::{normalize_quaternion, propagate_constant_av};
use crate::error::Error;

const DIRECTORY_SPACING: usize = 100;
const WORDS_PER_RECORD: usize = 8;

/// One constant-angular-velocity interval of a CK type 2 segment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CkInterval {
    /// Interval start time in encoded SCLK ticks.
    pub start_sclk: f64,
    /// Interval stop time in encoded SCLK ticks (`stop_sclk > start_sclk`).
    pub stop_sclk: f64,
    /// Seconds per encoded SCLK tick (`> 0`).
    pub seconds_per_tick: f64,
    /// SPICE scalar-first quaternion `[q0, q1, q2, q3]` at `start_sclk` (`R_{ref -> inst}`).
    pub quaternion: [f64; 4],
    /// Constant angular velocity vector in the reference frame (`rad/s`).
    pub angular_velocity: [f64; 3],
}

/// Continuous attitude over disjoint constant-rate intervals (CK type 2).
#[derive(Clone, Debug, PartialEq)]
pub struct CkType2 {
    intervals: Vec<CkInterval>,
}

impl CkType2 {
    /// Validates and stores chronological disjoint constant-rate intervals.
    pub fn new(intervals: Vec<CkInterval>) -> Result<Self, Error> {
        if intervals.is_empty() {
            return Err(Error::InvalidCkInterval {
                index: 0,
                reason: "CK type 2 segment requires at least one interval",
            });
        }
        let mut prev_stop: Option<f64> = None;
        for (index, iv) in intervals.iter().enumerate() {
            check_interval(index, iv, prev_stop)?;
            prev_stop = Some(iv.stop_sclk);
        }
        Ok(Self { intervals })
    }

    /// Chronological constant-rate intervals.
    #[must_use]
    pub fn intervals(&self) -> &[CkInterval] {
        &self.intervals
    }

    /// Overall coverage span `[first.start_sclk, last.stop_sclk]`.
    #[must_use]
    pub fn span(&self) -> [f64; 2] {
        [
            self.intervals.first().map_or(f64::NAN, |i| i.start_sclk),
            self.intervals.last().map_or(f64::NAN, |i| i.stop_sclk),
        ]
    }

    /// Serializes to the NAIF CK type 2 DAF array layout:
    /// `N` 8-word records, `N` start times, `N` stop times, and `(N - 1) / 100`
    /// midpoint directory entries `(STOP(100*k) + START(100*k + 1)) / 2`.
    pub(super) fn to_words(&self) -> Vec<f64> {
        let n = self.intervals.len();
        let d = directory_len(n);
        let mut words = Vec::with_capacity(word_count(n));
        for iv in &self.intervals {
            words.extend_from_slice(&iv.quaternion);
            words.extend_from_slice(&iv.angular_velocity);
            words.push(iv.seconds_per_tick);
        }
        words.extend(self.intervals.iter().map(|iv| iv.start_sclk));
        words.extend(self.intervals.iter().map(|iv| iv.stop_sclk));
        for k in 1..=d {
            let stop_idx = k.saturating_mul(DIRECTORY_SPACING).saturating_sub(1);
            let start_idx = k.saturating_mul(DIRECTORY_SPACING);
            let stop = self.intervals.get(stop_idx).map_or(0.0, |i| i.stop_sclk);
            let start = self.intervals.get(start_idx).map_or(0.0, |i| i.start_sclk);
            words.push(0.5 * (stop + start));
        }
        words
    }

    /// Rebuilds from stored CK type 2 array words.
    pub(super) fn from_words(words: &[f64]) -> Result<Result<Self, Error>, &'static str> {
        let n = infer_record_count(words.len())
            .ok_or("word count does not match 10 * N + (N - 1) / 100")?;
        let rec_words = n.saturating_mul(WORDS_PER_RECORD);
        let records = words.get(..rec_words).ok_or("missing pointing records")?;
        let starts = words
            .get(rec_words..rec_words.saturating_add(n))
            .ok_or("missing interval start times")?;
        let stops_start = rec_words.saturating_add(n);
        let stops = words
            .get(stops_start..stops_start.saturating_add(n))
            .ok_or("missing interval stop times")?;
        let dirs = words
            .get(stops_start.saturating_add(n)..)
            .ok_or("missing directory")?;
        if dirs.iter().any(|x| !x.is_finite()) {
            return Err("non-finite directory entry");
        }
        let mut intervals = Vec::with_capacity(n);
        for ((&[q0, q1, q2, q3, a1, a2, a3, rate], &start_sclk), &stop_sclk) in records
            .as_chunks::<WORDS_PER_RECORD>()
            .0
            .iter()
            .zip(starts)
            .zip(stops)
        {
            intervals.push(CkInterval {
                start_sclk,
                stop_sclk,
                seconds_per_tick: rate,
                quaternion: [q0, q1, q2, q3],
                angular_velocity: [a1, a2, a3],
            });
        }
        Ok(Self::new(intervals))
    }

    /// Evaluates pointing and angular velocity at `sclk` within `tolerance` ticks.
    pub(super) fn evaluate(&self, sclk: f64, tolerance: f64, frame: i32) -> Option<CkPointing> {
        if !sclk.is_finite() || !tolerance.is_finite() || tolerance < 0.0 {
            return None;
        }
        // First interval whose start is strictly after `sclk`, so overlapping
        // boundaries (`stop[i] == start[i+1]`) select `i + 1` per NAIF ck.req.
        let after = self.intervals.partition_point(|iv| iv.start_sclk <= sclk);
        if let Some(prev) = after.checked_sub(1).and_then(|i| self.intervals.get(i)) {
            if sclk <= prev.stop_sclk {
                return Some(eval_interval(prev, sclk, frame));
            }
            let d_left = sclk - prev.stop_sclk;
            let next_opt = self.intervals.get(after);
            let d_right = next_opt.map_or(f64::INFINITY, |n| n.start_sclk - sclk);
            if d_right <= d_left && d_right <= tolerance {
                return next_opt.map(|n| eval_interval(n, n.start_sclk, frame));
            }
            if d_left <= tolerance {
                return Some(eval_interval(prev, prev.stop_sclk, frame));
            }
            return None;
        }
        let first = self.intervals.first()?;
        if first.start_sclk - sclk <= tolerance {
            Some(eval_interval(first, first.start_sclk, frame))
        } else {
            None
        }
    }
}

fn eval_interval(iv: &CkInterval, eval_sclk: f64, frame: i32) -> CkPointing {
    let dt_s = (eval_sclk - iv.start_sclk) * iv.seconds_per_tick;
    let q0 = normalize_quaternion(iv.quaternion).unwrap_or([1.0, 0.0, 0.0, 0.0]);
    let quaternion = propagate_constant_av(q0, iv.angular_velocity, dt_s);
    CkPointing {
        sclk: eval_sclk,
        frame,
        quaternion,
        angular_velocity: Some(iv.angular_velocity),
    }
}

fn check_interval(index: usize, iv: &CkInterval, prev_stop: Option<f64>) -> Result<(), Error> {
    if !iv.start_sclk.is_finite() || !iv.stop_sclk.is_finite() {
        return Err(Error::NonFinite {
            what: "CK interval SCLK bound",
            index,
        });
    }
    if iv.stop_sclk <= iv.start_sclk {
        return Err(Error::InvalidCkInterval {
            index,
            reason: "stop_sclk must be strictly greater than start_sclk",
        });
    }
    if let Some(prev) = prev_stop
        && iv.start_sclk < prev
    {
        return Err(Error::InvalidCkInterval {
            index,
            reason: "intervals must be non-overlapping and sorted",
        });
    }
    if !iv.seconds_per_tick.is_finite() || iv.seconds_per_tick <= 0.0 {
        return Err(Error::InvalidCkInterval {
            index,
            reason: "seconds_per_tick must be finite and positive",
        });
    }
    if normalize_quaternion(iv.quaternion).is_none() {
        return Err(Error::InvalidQuaternion { index });
    }
    if iv.angular_velocity.iter().any(|w| !w.is_finite()) {
        return Err(Error::NonFinite {
            what: "CK angular velocity",
            index,
        });
    }
    Ok(())
}

const fn directory_len(n: usize) -> usize {
    n.saturating_sub(1) / DIRECTORY_SPACING
}

const fn word_count(n: usize) -> usize {
    n.saturating_mul(10).saturating_add(directory_len(n))
}

fn infer_record_count(total_words: usize) -> Option<usize> {
    let max_n = total_words / 10;
    if max_n == 0 {
        return None;
    }
    let mut lo = 1_usize;
    let mut hi = max_n;
    while lo <= hi {
        let mid = lo.saturating_add(hi.saturating_sub(lo) / 2);
        let expected = word_count(mid);
        if expected == total_words {
            return Some(mid);
        }
        if expected < total_words {
            lo = mid.saturating_add(1);
        } else {
            hi = mid.saturating_sub(1);
        }
    }
    None
}

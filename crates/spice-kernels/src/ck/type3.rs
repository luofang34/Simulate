//! CK type 3: discrete pointing instances with linear SCLK interval SLERP.

use alloc::vec::Vec;

use super::CkPointing;
use super::quaternion::{normalize_quaternion, quaternion_slerp};
use crate::error::Error;

const DIRECTORY_SPACING: usize = 100;

/// One discrete pointing instance of a CK type 3 segment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CkPointingInstance {
    /// Encoded SCLK timestamp.
    pub sclk: f64,
    /// SPICE scalar-first quaternion `[q0, q1, q2, q3]` (`R_{ref -> inst}`).
    pub quaternion: [f64; 4],
    /// Optional reference-frame angular velocity vector (`rad/s`), present iff `avflag = 1`.
    pub angular_velocity: Option<[f64; 3]>,
}

/// Linearly interpolated discrete pointing partitioned into continuous intervals (CK type 3).
#[derive(Clone, Debug, PartialEq)]
pub struct CkType3 {
    instances: Vec<CkPointingInstance>,
    interval_starts: Vec<f64>,
    interval_start_indices: Vec<usize>,
    has_angular_velocity: bool,
}

impl CkType3 {
    /// Validates and builds type 3 data with explicit interpolation interval start times.
    ///
    /// Per NAIF CK type 3 rules, `interval_starts` must be strictly increasing,
    /// begin at `instances[0].sclk`, and every start time must equal an instance `sclk`.
    pub fn new(
        instances: Vec<CkPointingInstance>,
        interval_starts: Vec<f64>,
        has_angular_velocity: bool,
    ) -> Result<Self, Error> {
        if instances.is_empty() {
            return Err(Error::InvalidCkInterval {
                index: 0,
                reason: "CK type 3 segment requires at least one pointing instance",
            });
        }
        validate_instances(&instances, has_angular_velocity)?;
        let interval_start_indices = validate_intervals(&instances, &interval_starts)?;
        Ok(Self {
            instances,
            interval_starts,
            interval_start_indices,
            has_angular_velocity,
        })
    }

    /// Convenience constructor placing all instances into a single continuous
    /// interpolation interval starting at the first instance.
    pub fn single_interval(
        instances: Vec<CkPointingInstance>,
        has_angular_velocity: bool,
    ) -> Result<Self, Error> {
        let first = instances.first().map_or(f64::NAN, |p| p.sclk);
        Self::new(instances, alloc::vec![first], has_angular_velocity)
    }

    /// Pointing instances in chronological order.
    #[must_use]
    pub fn instances(&self) -> &[CkPointingInstance] {
        &self.instances
    }

    /// Encoded SCLK start times of each continuous interpolation interval.
    #[must_use]
    pub fn interval_starts(&self) -> &[f64] {
        &self.interval_starts
    }

    /// Whether angular velocity vectors are stored (`avflag == 1`).
    #[must_use]
    pub const fn has_angular_velocity(&self) -> bool {
        self.has_angular_velocity
    }

    /// Overall coverage span `[first.sclk, last.sclk]`.
    #[must_use]
    pub fn span(&self) -> [f64; 2] {
        [
            self.instances.first().map_or(f64::NAN, |i| i.sclk),
            self.instances.last().map_or(f64::NAN, |i| i.sclk),
        ]
    }

    /// Serializes to the NAIF CK type 3 DAF array layout:
    /// pointing records (`4*N` or `7*N`), `SCLKDP[N]`, SCLK directory,
    /// `STARTS[M]`, starts directory, `M`, `N`.
    pub(super) fn to_words(&self) -> Vec<f64> {
        let n = self.instances.len();
        let m = self.interval_starts.len();
        let mut words = Vec::with_capacity(word_count(n, m, self.has_angular_velocity));
        for inst in &self.instances {
            words.extend_from_slice(&inst.quaternion);
            if self.has_angular_velocity {
                words.extend_from_slice(&inst.angular_velocity.unwrap_or([0.0; 3]));
            }
        }
        words.extend(self.instances.iter().map(|i| i.sclk));
        for idx in (DIRECTORY_SPACING.saturating_sub(1)..n)
            .step_by(DIRECTORY_SPACING)
            .take(directory_len(n))
        {
            words.push(self.instances.get(idx).map_or(0.0, |i| i.sclk));
        }
        words.extend_from_slice(&self.interval_starts);
        for idx in (DIRECTORY_SPACING.saturating_sub(1)..m)
            .step_by(DIRECTORY_SPACING)
            .take(directory_len(m))
        {
            words.push(self.interval_starts.get(idx).copied().unwrap_or(0.0));
        }
        words.push(m as f64);
        words.push(n as f64);
        words
    }

    /// Rebuilds from stored CK type 3 array words.
    pub(super) fn from_words(
        words: &[f64],
        has_angular_velocity: bool,
    ) -> Result<Result<Self, Error>, &'static str> {
        let [.., nints_f, nrec_f] = *words else {
            return Err("fewer than two trailer words");
        };
        let m = nints_f as usize;
        let n = nrec_f as usize;
        if m as f64 != nints_f || m == 0 {
            return Err("interval count NUMINT is not a positive integer");
        }
        if n as f64 != nrec_f || n == 0 {
            return Err("record count NPREC is not a positive integer");
        }
        if word_count(n, m, has_angular_velocity) != words.len() {
            return Err("word count does not match CK type 3 layout");
        }
        let rec_size = if has_angular_velocity {
            7_usize
        } else {
            4_usize
        };
        let rec_words = n.saturating_mul(rec_size);
        let sclks_start = rec_words;
        let starts_start = sclks_start
            .saturating_add(n)
            .saturating_add(directory_len(n));
        let rec_slice = words.get(..rec_words).unwrap_or(&[]);
        let sclks = words
            .get(sclks_start..sclks_start.saturating_add(n))
            .unwrap_or(&[]);
        let starts = words
            .get(starts_start..starts_start.saturating_add(m))
            .unwrap_or(&[])
            .to_vec();

        let mut instances = Vec::with_capacity(n);
        for (chunk, &sclk) in rec_slice.chunks_exact(rec_size).zip(sclks) {
            let q = [
                chunk.first().copied().unwrap_or(0.0),
                chunk.get(1).copied().unwrap_or(0.0),
                chunk.get(2).copied().unwrap_or(0.0),
                chunk.get(3).copied().unwrap_or(0.0),
            ];
            let angular_velocity = if has_angular_velocity {
                Some([
                    chunk.get(4).copied().unwrap_or(0.0),
                    chunk.get(5).copied().unwrap_or(0.0),
                    chunk.get(6).copied().unwrap_or(0.0),
                ])
            } else {
                None
            };
            instances.push(CkPointingInstance {
                sclk,
                quaternion: q,
                angular_velocity,
            });
        }
        Ok(Self::new(instances, starts, has_angular_velocity))
    }

    /// Evaluates pointing (and angular velocity if present) at `sclk` within `tolerance`.
    pub(super) fn evaluate(&self, sclk: f64, tolerance: f64, frame: i32) -> Option<CkPointing> {
        if !sclk.is_finite() || !tolerance.is_finite() || tolerance < 0.0 {
            return None;
        }
        let iv_after = self.interval_starts.partition_point(|&t| t <= sclk);
        let Some(iv_idx) = iv_after.checked_sub(1) else {
            let first = self.instances.first()?;
            return (first.sclk - sclk <= tolerance).then(|| instance_pointing(first, frame));
        };
        let k_start = *self.interval_start_indices.get(iv_idx)?;
        let k_end = self
            .interval_start_indices
            .get(iv_idx.saturating_add(1))
            .map_or(self.instances.len().saturating_sub(1), |&next| {
                next.saturating_sub(1)
            });
        let end_inst = self.instances.get(k_end)?;
        if sclk <= end_inst.sclk {
            let slice = self.instances.get(k_start..=k_end)?;
            let local = slice.partition_point(|p| p.sclk <= sclk).saturating_sub(1);
            let p1 = slice.get(local)?;
            if sclk == p1.sclk || local.saturating_add(1) >= slice.len() {
                return Some(instance_pointing(p1, frame));
            }
            let p2 = slice.get(local.saturating_add(1))?;
            return Some(interpolate_instances(p1, p2, sclk, frame));
        }
        let d_left = sclk - end_inst.sclk;
        let next_start_inst = self
            .interval_start_indices
            .get(iv_after)
            .and_then(|&idx| self.instances.get(idx));
        let d_right = next_start_inst.map_or(f64::INFINITY, |p| p.sclk - sclk);
        if d_right <= d_left && d_right <= tolerance {
            return next_start_inst.map(|p| instance_pointing(p, frame));
        }
        (d_left <= tolerance).then(|| instance_pointing(end_inst, frame))
    }
}

fn instance_pointing(inst: &CkPointingInstance, frame: i32) -> CkPointing {
    CkPointing {
        sclk: inst.sclk,
        frame,
        quaternion: normalize_quaternion(inst.quaternion).unwrap_or([1.0, 0.0, 0.0, 0.0]),
        angular_velocity: inst.angular_velocity,
    }
}

fn interpolate_instances(
    p1: &CkPointingInstance,
    p2: &CkPointingInstance,
    sclk: f64,
    frame: i32,
) -> CkPointing {
    let dt = p2.sclk - p1.sclk;
    let w = if dt > 0.0 {
        ((sclk - p1.sclk) / dt).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let q1 = normalize_quaternion(p1.quaternion).unwrap_or([1.0, 0.0, 0.0, 0.0]);
    let q2 = normalize_quaternion(p2.quaternion).unwrap_or([1.0, 0.0, 0.0, 0.0]);
    let quaternion = quaternion_slerp(q1, q2, w);
    let angular_velocity = match (p1.angular_velocity, p2.angular_velocity) {
        (Some([a10, a11, a12]), Some([a20, a21, a22])) => Some([
            (1.0 - w) * a10 + w * a20,
            (1.0 - w) * a11 + w * a21,
            (1.0 - w) * a12 + w * a22,
        ]),
        _ => None,
    };
    CkPointing {
        sclk,
        frame,
        quaternion,
        angular_velocity,
    }
}

fn validate_instances(
    instances: &[CkPointingInstance],
    has_angular_velocity: bool,
) -> Result<(), Error> {
    let mut prev: Option<f64> = None;
    for (index, inst) in instances.iter().enumerate() {
        if !inst.sclk.is_finite() {
            return Err(Error::NonFinite {
                what: "CK SCLK",
                index,
            });
        }
        if let Some(p) = prev
            && inst.sclk <= p
        {
            return Err(Error::EpochsOutOfOrder {
                index,
                epoch: inst.sclk,
                previous: p,
            });
        }
        prev = Some(inst.sclk);
        if normalize_quaternion(inst.quaternion).is_none() {
            return Err(Error::InvalidQuaternion { index });
        }
        match (has_angular_velocity, inst.angular_velocity) {
            (true, Some(av)) if av.iter().any(|w| !w.is_finite()) => {
                return Err(Error::NonFinite {
                    what: "CK angular velocity",
                    index,
                });
            }
            (true, None) | (false, Some(_)) => {
                return Err(Error::InvalidCkInterval {
                    index,
                    reason: "instance angular velocity presence does not match segment avflag",
                });
            }
            _ => {}
        }
    }
    Ok(())
}

fn validate_intervals(
    instances: &[CkPointingInstance],
    interval_starts: &[f64],
) -> Result<Vec<usize>, Error> {
    if interval_starts.is_empty() {
        return Err(Error::InvalidCkInterval {
            index: 0,
            reason: "CK type 3 requires at least one interpolation interval",
        });
    }
    let first_sclk = instances.first().map_or(f64::NAN, |i| i.sclk);
    let mut indices = Vec::with_capacity(interval_starts.len());
    let mut prev: Option<f64> = None;
    for (idx, &start) in interval_starts.iter().enumerate() {
        if !start.is_finite() {
            return Err(Error::NonFinite {
                what: "CK interval start",
                index: idx,
            });
        }
        if idx == 0 && start != first_sclk {
            return Err(Error::InvalidCkInterval {
                index: 0,
                reason: "first interval start must equal first instance SCLK",
            });
        }
        if let Some(p) = prev
            && start <= p
        {
            return Err(Error::InvalidCkInterval {
                index: idx,
                reason: "interval start times must be strictly increasing",
            });
        }
        let pos = instances
            .binary_search_by(|inst| inst.sclk.total_cmp(&start))
            .map_err(|_| Error::InvalidCkInterval {
                index: idx,
                reason: "interval start time must match a pointing instance SCLK",
            })?;
        indices.push(pos);
        prev = Some(start);
    }
    Ok(indices)
}

const fn directory_len(n: usize) -> usize {
    n.saturating_sub(1) / DIRECTORY_SPACING
}

const fn word_count(n: usize, m: usize, has_av: bool) -> usize {
    let rec_size = if has_av { 7_usize } else { 4_usize };
    n.saturating_mul(rec_size.saturating_add(1))
        .saturating_add(directory_len(n))
        .saturating_add(m)
        .saturating_add(directory_len(m))
        .saturating_add(2)
}

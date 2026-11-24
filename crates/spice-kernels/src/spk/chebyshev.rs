//! Equal-interval Chebyshev position records for SPK type 2 and shared packing/fitting.

use alloc::vec;
use alloc::vec::Vec;

use super::chebyshev_fit::{
    check_finite_slice, check_header, check_mid_radius, check_sample_inputs,
    collect_granule_channels, eval_type2_record, fit_least_squares_chebyshev, fit_lobatto_1d,
    lobatto_nodes, parse_trailer, select_record, slice_vec, validate_flat_records,
};
use crate::error::Error;

/// One Chebyshev granule of a SPK type 2 segment (position only).
#[derive(Clone, Debug, PartialEq)]
pub struct ChebyshevPositionRecord {
    /// Interval midpoint epoch, TDB seconds past J2000.
    pub mid: f64,
    /// Interval half-width in seconds (`> 0`).
    pub radius: f64,
    /// Chebyshev coefficients `[x, y, z]` in kilometres, all of equal length `degree + 1`.
    pub coeffs: [Vec<f64>; 3],
}

/// Equal-interval Chebyshev position polynomials of SPK type 2.
///
/// Velocity is obtained analytically by differentiating the position polynomials.
#[derive(Clone, Debug, PartialEq)]
pub struct ChebyshevPositions {
    init: f64,
    intlen: f64,
    n_coeff: usize,
    n_records: usize,
    records: Vec<f64>,
}

impl ChebyshevPositions {
    /// Builds type 2 data from explicit per-granule Chebyshev records.
    pub fn from_records(
        init: f64,
        intlen: f64,
        records: &[ChebyshevPositionRecord],
    ) -> Result<Self, Error> {
        let mut flat = Vec::new();
        let n_coeff = pack_records(
            init,
            intlen,
            records.len(),
            3,
            &mut flat,
            records.iter().map(|r| (r.mid, r.radius, r.coeffs.iter())),
        )?;
        Ok(Self {
            init,
            intlen,
            n_coeff,
            n_records: records.len(),
            records: flat,
        })
    }

    /// Builds type 2 data from `[x, y, z]` coefficient blocks over uniform intervals
    /// starting at `init` with step `intlen`.
    pub fn from_uniform(init: f64, intlen: f64, coeffs: &[[Vec<f64>; 3]]) -> Result<Self, Error> {
        check_header(init, intlen, coeffs.len())?;
        let radius = 0.5 * intlen;
        let records: Vec<ChebyshevPositionRecord> = coeffs
            .iter()
            .enumerate()
            .map(|(k, c)| ChebyshevPositionRecord {
                mid: init + (k as f64 + 0.5) * intlen,
                radius,
                coeffs: c.clone(),
            })
            .collect();
        Self::from_records(init, intlen, &records)
    }

    /// Fits a type 2 Chebyshev series of `degree` on `n_records` granules by
    /// sampling `sample_pos(et)` (km) at Chebyshev-Gauss-Lobatto nodes.
    pub fn fit_nodes(
        init: f64,
        intlen: f64,
        n_records: usize,
        degree: usize,
        mut sample_pos: impl FnMut(f64) -> [f64; 3],
    ) -> Result<Self, Error> {
        match Self::try_fit_nodes::<core::convert::Infallible>(
            init,
            intlen,
            n_records,
            degree,
            |t| Ok(sample_pos(t)),
        ) {
            Ok(res) => res,
            Err(never) => match never {},
        }
    }

    /// Fallible variant of [`Self::fit_nodes`].
    pub fn try_fit_nodes<E>(
        init: f64,
        intlen: f64,
        n_records: usize,
        degree: usize,
        sample_pos: impl FnMut(f64) -> Result<[f64; 3], E>,
    ) -> Result<Result<Self, Error>, E> {
        let fitted = match fit_lobatto_channels(init, intlen, n_records, degree, sample_pos)? {
            Ok(data) => data,
            Err(err) => return Ok(Err(err)),
        };
        Ok(Self::from_uniform(init, intlen, &fitted))
    }

    /// Fits a type 2 Chebyshev series of `degree` on `n_records` granules via
    /// Chebyshev-basis least squares over dense sampled `(epochs, positions)` (km).
    pub fn fit_samples(
        init: f64,
        intlen: f64,
        n_records: usize,
        degree: usize,
        epochs: &[f64],
        positions: &[[f64; 3]],
    ) -> Result<Self, Error> {
        let coeffs = fit_sample_channels(init, intlen, n_records, degree, epochs, positions)?;
        Self::from_uniform(init, intlen, &coeffs)
    }

    /// Initial granule start epoch, TDB seconds past J2000.
    #[must_use]
    pub const fn init(&self) -> f64 {
        self.init
    }

    /// Equal granule interval length in seconds.
    #[must_use]
    pub const fn intlen(&self) -> f64 {
        self.intlen
    }

    /// Polynomial degree (`n_coeff - 1`).
    #[must_use]
    pub const fn degree(&self) -> usize {
        self.n_coeff.saturating_sub(1)
    }

    /// Number of Chebyshev coefficients per component (`degree + 1`).
    #[must_use]
    pub const fn n_coeff(&self) -> usize {
        self.n_coeff
    }

    /// Number of equal-interval granules.
    #[must_use]
    pub const fn n_records(&self) -> usize {
        self.n_records
    }

    /// Full span `[init, init + n_records * intlen]`.
    #[must_use]
    pub fn span(&self) -> [f64; 2] {
        [self.init, self.init + (self.n_records as f64) * self.intlen]
    }

    /// Reconstructs per-granule records.
    #[must_use]
    pub fn records(&self) -> Vec<ChebyshevPositionRecord> {
        let rsize = 2_usize.saturating_add(3_usize.saturating_mul(self.n_coeff));
        let n = self.n_coeff;
        self.records
            .chunks_exact(rsize)
            .map(|rec| ChebyshevPositionRecord {
                mid: rec.first().copied().unwrap_or(0.0),
                radius: rec.get(1).copied().unwrap_or(0.0),
                coeffs: [
                    slice_vec(rec, 2, n),
                    slice_vec(rec, 2_usize.saturating_add(n), n),
                    slice_vec(rec, 2_usize.saturating_add(2_usize.saturating_mul(n)), n),
                ],
            })
            .collect()
    }

    pub(super) fn to_words(&self) -> Vec<f64> {
        let rsize = 2_usize.saturating_add(3_usize.saturating_mul(self.n_coeff));
        let mut words = Vec::with_capacity(self.records.len().saturating_add(4));
        words.extend_from_slice(&self.records);
        words.extend_from_slice(&[self.init, self.intlen, rsize as f64, self.n_records as f64]);
        words
    }

    pub(super) fn from_words(words: &[f64]) -> Result<Result<Self, Error>, &'static str> {
        let (init, intlen, rsize, n_records, payload) = parse_trailer(words)?;
        if rsize < 5 || !rsize.saturating_sub(2).is_multiple_of(3) {
            return Err("type 2 record size is not 2 + 3 * n_coeff");
        }
        let n_coeff = rsize.saturating_sub(2) / 3;
        Ok(
            validate_flat_records(init, intlen, n_coeff, n_records, payload, 3).map(|()| Self {
                init,
                intlen,
                n_coeff,
                n_records,
                records: payload.to_vec(),
            }),
        )
    }

    pub(super) fn evaluate(&self, et: f64) -> Option<[f64; 6]> {
        let rsize = 2_usize.saturating_add(3_usize.saturating_mul(self.n_coeff));
        let idx = select_record(self.init, self.intlen, self.n_records, et);
        let start = idx.saturating_mul(rsize);
        let rec = self.records.get(start..start.saturating_add(rsize))?;
        eval_type2_record(rec, self.n_coeff, et)
    }
}

pub(super) fn pack_records<'a, I>(
    init: f64,
    intlen: f64,
    n_records: usize,
    channels: usize,
    flat: &mut Vec<f64>,
    records: impl Iterator<Item = (f64, f64, I)>,
) -> Result<usize, Error>
where
    I: Iterator<Item = &'a Vec<f64>>,
{
    check_header(init, intlen, n_records)?;
    let mut n_coeff: Option<usize> = None;
    for (idx, (mid, radius, axes)) in records.enumerate() {
        check_mid_radius(mid, radius, idx)?;
        flat.push(mid);
        flat.push(radius);
        for axis in axes {
            let expected = *n_coeff.get_or_insert(axis.len());
            if expected == 0 {
                return Err(Error::InvalidChebyshev {
                    reason: "Chebyshev degree requires at least one coefficient",
                });
            }
            if axis.len() != expected {
                return Err(Error::InvalidChebyshev {
                    reason: "all components and records must share the same coefficient count",
                });
            }
            check_finite_slice(axis, "Chebyshev coefficient", idx)?;
            flat.extend_from_slice(axis);
        }
    }
    let coeffs = n_coeff.unwrap_or(0);
    let expected_len =
        n_records.saturating_mul(2_usize.saturating_add(channels.saturating_mul(coeffs)));
    if coeffs == 0 || flat.len() != expected_len {
        return Err(Error::InvalidChebyshev {
            reason: "inconsistent Chebyshev channel count",
        });
    }
    Ok(coeffs)
}

pub(super) type FittedChannels<const C: usize> = Result<Vec<[Vec<f64>; C]>, Error>;

pub(super) fn fit_lobatto_channels<const C: usize, E>(
    init: f64,
    intlen: f64,
    n_records: usize,
    degree: usize,
    mut sample: impl FnMut(f64) -> Result<[f64; C], E>,
) -> Result<FittedChannels<C>, E> {
    if let Err(err) = check_header(init, intlen, n_records) {
        return Ok(Err(err));
    }
    let nodes = lobatto_nodes(degree);
    let radius = 0.5 * intlen;
    let mut out = Vec::with_capacity(n_records);
    for k in 0..n_records {
        let mid = init + (k as f64 + 0.5) * intlen;
        let mut taus = Vec::with_capacity(nodes.len());
        let mut vals = vec![Vec::with_capacity(nodes.len()); C];
        for &s in &nodes {
            let t_node = mid + radius * s;
            taus.push((t_node - mid) / radius);
            let y = sample(t_node)?;
            for (ch, &v) in vals.iter_mut().zip(&y) {
                ch.push(v);
            }
        }
        let max_drift = taus
            .iter()
            .zip(&nodes)
            .fold(0.0_f64, |acc, (&t, &s)| acc.max((t - s).abs()));
        let fitted: [Vec<f64>; C] = if max_drift > 1.0e-14
            && let Ok(ls) = fit_least_squares_chebyshev(k, &taus, &vals, degree)
        {
            core::array::from_fn(|c| ls.get(c).cloned().unwrap_or_default())
        } else {
            core::array::from_fn(|c| fit_lobatto_1d(vals.get(c).map_or(&[], Vec::as_slice), degree))
        };
        out.push(fitted);
    }
    Ok(Ok(out))
}

pub(super) fn fit_sample_channels<const C: usize>(
    init: f64,
    intlen: f64,
    n_records: usize,
    degree: usize,
    epochs: &[f64],
    values: &[[f64; C]],
) -> Result<Vec<[Vec<f64>; C]>, Error> {
    check_header(init, intlen, n_records)?;
    check_sample_inputs(epochs, values)?;
    let radius = 0.5 * intlen;
    let tol = 1.0e-12 * intlen;
    let mut out = Vec::with_capacity(n_records);
    for k in 0..n_records {
        let t_lo = init + (k as f64) * intlen;
        let t_hi = init + ((k.saturating_add(1)) as f64) * intlen;
        let mid = init + (k as f64 + 0.5) * intlen;
        let (taus, channels) =
            collect_granule_channels(epochs, values, t_lo, t_hi, mid, radius, tol);
        let fitted = fit_least_squares_chebyshev(k, &taus, &channels, degree)?;
        let arr: [Vec<f64>; C] =
            core::array::from_fn(|c| fitted.get(c).cloned().unwrap_or_default());
        out.push(arr);
    }
    Ok(out)
}

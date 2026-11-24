//! Equal-interval Chebyshev position and velocity records for SPK type 3.

use alloc::vec::Vec;

use super::chebyshev::{fit_lobatto_channels, fit_sample_channels, pack_records};
use super::chebyshev_fit::{
    check_header, eval_type3_record, parse_trailer, select_record, slice_vec, validate_flat_records,
};
use crate::error::Error;

/// One Chebyshev granule of a SPK type 3 segment (position and velocity).
#[derive(Clone, Debug, PartialEq)]
pub struct ChebyshevStateRecord {
    /// Interval midpoint epoch, TDB seconds past J2000.
    pub mid: f64,
    /// Interval half-width in seconds (`> 0`).
    pub radius: f64,
    /// Position Chebyshev coefficients `[x, y, z]` in kilometres, all of length `degree + 1`.
    pub pos: [Vec<f64>; 3],
    /// Velocity Chebyshev coefficients `[vx, vy, vz]` in km/s, all of length `degree + 1`.
    pub vel: [Vec<f64>; 3],
}

/// Equal-interval Chebyshev position and velocity polynomials of SPK type 3.
#[derive(Clone, Debug, PartialEq)]
pub struct ChebyshevStates {
    init: f64,
    intlen: f64,
    n_coeff: usize,
    n_records: usize,
    records: Vec<f64>,
}

impl ChebyshevStates {
    /// Builds type 3 data from explicit per-granule Chebyshev records.
    pub fn from_records(
        init: f64,
        intlen: f64,
        records: &[ChebyshevStateRecord],
    ) -> Result<Self, Error> {
        let mut flat = Vec::new();
        let n_coeff = pack_records(
            init,
            intlen,
            records.len(),
            6,
            &mut flat,
            records
                .iter()
                .map(|r| (r.mid, r.radius, r.pos.iter().chain(r.vel.iter()))),
        )?;
        Ok(Self {
            init,
            intlen,
            n_coeff,
            n_records: records.len(),
            records: flat,
        })
    }

    /// Builds type 3 data from `[x, y, z, vx, vy, vz]` coefficient blocks over
    /// uniform intervals starting at `init` with step `intlen`.
    pub fn from_uniform(init: f64, intlen: f64, coeffs: &[[Vec<f64>; 6]]) -> Result<Self, Error> {
        check_header(init, intlen, coeffs.len())?;
        let radius = 0.5 * intlen;
        let records: Vec<ChebyshevStateRecord> = coeffs
            .iter()
            .enumerate()
            .map(|(k, [x, y, z, vx, vy, vz])| ChebyshevStateRecord {
                mid: init + (k as f64 + 0.5) * intlen,
                radius,
                pos: [x.clone(), y.clone(), z.clone()],
                vel: [vx.clone(), vy.clone(), vz.clone()],
            })
            .collect();
        Self::from_records(init, intlen, &records)
    }

    /// Fits a type 3 Chebyshev series of `degree` on `n_records` granules by
    /// sampling `sample_state(et)` (`[x, y, z, vx, vy, vz]` in km, km/s) at
    /// Chebyshev-Gauss-Lobatto nodes.
    pub fn fit_nodes(
        init: f64,
        intlen: f64,
        n_records: usize,
        degree: usize,
        mut sample_state: impl FnMut(f64) -> [f64; 6],
    ) -> Result<Self, Error> {
        match Self::try_fit_nodes::<core::convert::Infallible>(
            init,
            intlen,
            n_records,
            degree,
            |t| Ok(sample_state(t)),
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
        sample_state: impl FnMut(f64) -> Result<[f64; 6], E>,
    ) -> Result<Result<Self, Error>, E> {
        let fitted = match fit_lobatto_channels(init, intlen, n_records, degree, sample_state)? {
            Ok(data) => data,
            Err(err) => return Ok(Err(err)),
        };
        Ok(Self::from_uniform(init, intlen, &fitted))
    }

    /// Fits a type 3 Chebyshev series of `degree` on `n_records` granules via
    /// Chebyshev-basis least squares over dense sampled `(epochs, states)` (km, km/s).
    pub fn fit_samples(
        init: f64,
        intlen: f64,
        n_records: usize,
        degree: usize,
        epochs: &[f64],
        states: &[[f64; 6]],
    ) -> Result<Self, Error> {
        let fitted = fit_sample_channels(init, intlen, n_records, degree, epochs, states)?;
        Self::from_uniform(init, intlen, &fitted)
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
    pub fn records(&self) -> Vec<ChebyshevStateRecord> {
        let rsize = 2_usize.saturating_add(6_usize.saturating_mul(self.n_coeff));
        let n = self.n_coeff;
        self.records
            .chunks_exact(rsize)
            .map(|rec| ChebyshevStateRecord {
                mid: rec.first().copied().unwrap_or(0.0),
                radius: rec.get(1).copied().unwrap_or(0.0),
                pos: [
                    slice_vec(rec, 2, n),
                    slice_vec(rec, 2_usize.saturating_add(n), n),
                    slice_vec(rec, 2_usize.saturating_add(2_usize.saturating_mul(n)), n),
                ],
                vel: [
                    slice_vec(rec, 2_usize.saturating_add(3_usize.saturating_mul(n)), n),
                    slice_vec(rec, 2_usize.saturating_add(4_usize.saturating_mul(n)), n),
                    slice_vec(rec, 2_usize.saturating_add(5_usize.saturating_mul(n)), n),
                ],
            })
            .collect()
    }

    pub(super) fn to_words(&self) -> Vec<f64> {
        let rsize = 2_usize.saturating_add(6_usize.saturating_mul(self.n_coeff));
        let mut words = Vec::with_capacity(self.records.len().saturating_add(4));
        words.extend_from_slice(&self.records);
        words.extend_from_slice(&[self.init, self.intlen, rsize as f64, self.n_records as f64]);
        words
    }

    pub(super) fn from_words(words: &[f64]) -> Result<Result<Self, Error>, &'static str> {
        let (init, intlen, rsize, n_records, payload) = parse_trailer(words)?;
        if rsize < 8 || !rsize.saturating_sub(2).is_multiple_of(6) {
            return Err("type 3 record size is not 2 + 6 * n_coeff");
        }
        let n_coeff = rsize.saturating_sub(2) / 6;
        Ok(
            validate_flat_records(init, intlen, n_coeff, n_records, payload, 6).map(|()| Self {
                init,
                intlen,
                n_coeff,
                n_records,
                records: payload.to_vec(),
            }),
        )
    }

    pub(super) fn evaluate(&self, et: f64) -> Option<[f64; 6]> {
        let rsize = 2_usize.saturating_add(6_usize.saturating_mul(self.n_coeff));
        let idx = select_record(self.init, self.intlen, self.n_records, et);
        let start = idx.saturating_mul(rsize);
        let rec = self.records.get(start..start.saturating_add(rsize))?;
        eval_type3_record(rec, self.n_coeff, et)
    }
}

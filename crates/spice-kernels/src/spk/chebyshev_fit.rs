//! Clenshaw evaluation and Chebyshev-basis fitting using pure `libm` arithmetic.

use alloc::vec;
use alloc::vec::Vec;
use core::f64::consts::PI;

use crate::error::Error;

/// Evaluates `sum_{k=0}^{n-1} c_k T_k(s)` via Clenshaw's recurrence.
pub(super) fn clenshaw_val(coeffs: &[f64], s: f64) -> Option<f64> {
    let c0 = *coeffs.first()?;
    if coeffs.len() == 1 {
        return Some(c0);
    }
    let two_s = 2.0 * s;
    let mut b1 = 0.0_f64;
    let mut b2 = 0.0_f64;
    for &ck in coeffs.iter().skip(1).rev() {
        let b0 = ck + two_s * b1 - b2;
        b2 = b1;
        b1 = b0;
    }
    Some(c0 + s * b1 - b2)
}

/// Evaluates `(p(s), dp/ds)` for `p(s) = sum_{k=0}^{n-1} c_k T_k(s)` via
/// Clenshaw's recurrence and its differentiated recurrence.
pub(super) fn clenshaw_val_deriv(coeffs: &[f64], s: f64) -> Option<(f64, f64)> {
    let c0 = *coeffs.first()?;
    if coeffs.len() == 1 {
        return Some((c0, 0.0));
    }
    let two_s = 2.0 * s;
    let mut b1 = 0.0_f64;
    let mut b2 = 0.0_f64;
    let mut db1 = 0.0_f64;
    let mut db2 = 0.0_f64;
    for &ck in coeffs.iter().skip(1).rev() {
        let db0 = 2.0 * b1 + two_s * db1 - db2;
        let b0 = ck + two_s * b1 - b2;
        db2 = db1;
        db1 = db0;
        b2 = b1;
        b1 = b0;
    }
    Some((c0 + s * b1 - b2, b1 + s * db1 - db2))
}

/// Evaluates a Type 2 Chebyshev record `[mid, radius, x.., y.., z..]` at `et`.
pub(super) fn eval_type2_record(record: &[f64], n_coeff: usize, et: f64) -> Option<[f64; 6]> {
    let mid = *record.first()?;
    let radius = *record.get(1)?;
    if !mid.is_finite() || !radius.is_finite() || radius <= 0.0 {
        return None;
    }
    let s = (et - mid) / radius;
    let inv_r = 1.0 / radius;
    let mut state = [0.0_f64; 6];
    for axis in 0..3_usize {
        let start = 2_usize.saturating_add(axis.saturating_mul(n_coeff));
        let coeffs = record.get(start..start.saturating_add(n_coeff))?;
        let (p, dp_ds) = clenshaw_val_deriv(coeffs, s)?;
        *state.get_mut(axis)? = p;
        *state.get_mut(axis.saturating_add(3))? = dp_ds * inv_r;
    }
    Some(state)
}

/// Evaluates a Type 3 Chebyshev record `[mid, radius, x.., y.., z.., vx.., vy.., vz..]` at `et`.
pub(super) fn eval_type3_record(record: &[f64], n_coeff: usize, et: f64) -> Option<[f64; 6]> {
    let mid = *record.first()?;
    let radius = *record.get(1)?;
    if !mid.is_finite() || !radius.is_finite() || radius <= 0.0 {
        return None;
    }
    let s = (et - mid) / radius;
    let mut state = [0.0_f64; 6];
    for comp in 0..6_usize {
        let start = 2_usize.saturating_add(comp.saturating_mul(n_coeff));
        let coeffs = record.get(start..start.saturating_add(n_coeff))?;
        *state.get_mut(comp)? = clenshaw_val(coeffs, s)?;
    }
    Some(state)
}

/// Normalized Chebyshev-Gauss-Lobatto nodes `cos(j * pi / degree)` on `[-1, 1]`.
pub(super) fn lobatto_nodes(degree: usize) -> Vec<f64> {
    if degree == 0 {
        return vec![0.0];
    }
    let deg_f = degree as f64;
    (0..=degree)
        .map(|j| {
            if j == 0 {
                1.0
            } else if j == degree {
                -1.0
            } else {
                libm::cos(PI * (j as f64) / deg_f)
            }
        })
        .collect()
}

/// Computes Chebyshev coefficients of degree `degree` from function values at
/// [`lobatto_nodes(degree)`].
pub(super) fn fit_lobatto_1d(values: &[f64], degree: usize) -> Vec<f64> {
    if degree == 0 {
        return vec![values.first().copied().unwrap_or(0.0)];
    }
    let deg_f = degree as f64;
    let scale = 2.0 / deg_f;
    let mut coeffs = Vec::with_capacity(degree.saturating_add(1));
    for k in 0..=degree {
        let mut sum = 0.0_f64;
        for (j, &val) in values.iter().take(degree.saturating_add(1)).enumerate() {
            let weight = if j == 0 || j == degree { 0.5 } else { 1.0 };
            let angle = PI * (k as f64) * (j as f64) / deg_f;
            sum += weight * val * libm::cos(angle);
        }
        let mut ck = scale * sum;
        if k == 0 || k == degree {
            ck *= 0.5;
        }
        coeffs.push(ck);
    }
    coeffs
}

/// Least-squares Chebyshev fit of `degree` across `channels` at normalized
/// coordinates `taus` inside `[-1, 1]` using Householder QR decomposition.
pub(super) fn fit_least_squares_chebyshev(
    granule: usize,
    taus: &[f64],
    channels: &[Vec<f64>],
    degree: usize,
) -> Result<Vec<Vec<f64>>, Error> {
    let n_coeff = degree.saturating_add(1);
    let m = taus.len();
    if m < n_coeff {
        return Err(Error::InsufficientFitSamples {
            granule,
            required: n_coeff,
            found: m,
        });
    }
    let mut a = build_chebyshev_design(taus, n_coeff);
    let mut rhs = channels.to_vec();
    householder_qr_reduce(granule, &mut a, &mut rhs, m, n_coeff)?;
    let mut out = Vec::with_capacity(rhs.len());
    for col in &rhs {
        out.push(back_substitute(&a, col, n_coeff));
    }
    Ok(out)
}

fn build_chebyshev_design(taus: &[f64], n_coeff: usize) -> Vec<Vec<f64>> {
    let m = taus.len();
    let mut cols = vec![vec![0.0_f64; m]; n_coeff];
    for (row, &tau) in taus.iter().enumerate() {
        if let Some(c0) = cols.first_mut().and_then(|c| c.get_mut(row)) {
            *c0 = 1.0;
        }
        if n_coeff > 1
            && let Some(c1) = cols.get_mut(1).and_then(|c| c.get_mut(row))
        {
            *c1 = tau;
        }
        let two_tau = 2.0 * tau;
        for k in 2..n_coeff {
            let t_prev2 = cols
                .get(k.saturating_sub(2))
                .and_then(|c| c.get(row))
                .copied()
                .unwrap_or(0.0);
            let t_prev1 = cols
                .get(k.saturating_sub(1))
                .and_then(|c| c.get(row))
                .copied()
                .unwrap_or(0.0);
            if let Some(ck) = cols.get_mut(k).and_then(|c| c.get_mut(row)) {
                *ck = two_tau * t_prev1 - t_prev2;
            }
        }
    }
    cols
}

fn householder_qr_reduce(
    granule: usize,
    cols: &mut [Vec<f64>],
    rhs: &mut [Vec<f64>],
    m: usize,
    n: usize,
) -> Result<(), Error> {
    for k in 0..n {
        let mut norm_sq = 0.0_f64;
        if let Some(col_k) = cols.get(k) {
            for &val in col_k.iter().skip(k) {
                norm_sq += val * val;
            }
        }
        let norm = libm::sqrt(norm_sq);
        if !norm.is_finite() || norm <= 1.0e-13 {
            return Err(Error::SingularFit { granule });
        }
        let x0 = cols.get(k).and_then(|c| c.get(k)).copied().unwrap_or(0.0);
        let alpha = if x0 >= 0.0 { -norm } else { norm };
        let v0 = x0 - alpha;
        let v_tail_sq = norm_sq - x0 * x0 + v0 * v0;
        if !v_tail_sq.is_finite() || v_tail_sq == 0.0 {
            return Err(Error::SingularFit { granule });
        }
        let beta = 2.0 / v_tail_sq;
        let v: Vec<f64> = cols
            .get(k)
            .map(|c| {
                c.iter()
                    .enumerate()
                    .skip(k)
                    .map(|(i, &val)| if i == k { v0 } else { val })
                    .collect()
            })
            .unwrap_or_default();
        if let Some(diag) = cols.get_mut(k).and_then(|c| c.get_mut(k)) {
            *diag = alpha;
        }
        for col in cols.iter_mut().skip(k.saturating_add(1)) {
            apply_reflector(col, k, m, &v, beta);
        }
        for col in rhs.iter_mut() {
            apply_reflector(col, k, m, &v, beta);
        }
    }
    Ok(())
}

fn apply_reflector(col: &mut [f64], k: usize, m: usize, v: &[f64], beta: f64) {
    let mut dot = 0.0_f64;
    for (offset, &vj) in v.iter().enumerate() {
        let row = k.saturating_add(offset);
        if row < m {
            dot += vj * col.get(row).copied().unwrap_or(0.0);
        }
    }
    let scale = beta * dot;
    for (offset, &vj) in v.iter().enumerate() {
        let row = k.saturating_add(offset);
        if let Some(slot) = col.get_mut(row) {
            *slot -= scale * vj;
        }
    }
}

fn back_substitute(cols: &[Vec<f64>], rhs: &[f64], n: usize) -> Vec<f64> {
    let mut x = vec![0.0_f64; n];
    for k in (0..n).rev() {
        let mut sum = rhs.get(k).copied().unwrap_or(0.0);
        for j in k.saturating_add(1)..n {
            let r_kj = cols.get(j).and_then(|c| c.get(k)).copied().unwrap_or(0.0);
            let x_j = x.get(j).copied().unwrap_or(0.0);
            sum -= r_kj * x_j;
        }
        let diag = cols.get(k).and_then(|c| c.get(k)).copied().unwrap_or(1.0);
        if let Some(slot) = x.get_mut(k) {
            *slot = sum / diag;
        }
    }
    x
}

pub(super) fn check_header(init: f64, intlen: f64, n_records: usize) -> Result<(), Error> {
    if !init.is_finite() {
        return Err(Error::InvalidChebyshev {
            reason: "initial epoch INIT is not finite",
        });
    }
    if !intlen.is_finite() || intlen <= 0.0 {
        return Err(Error::InvalidChebyshev {
            reason: "interval length INTLEN must be finite and positive",
        });
    }
    if n_records == 0 {
        return Err(Error::InvalidChebyshev {
            reason: "segment requires at least one Chebyshev record",
        });
    }
    Ok(())
}

pub(super) fn check_mid_radius(mid: f64, radius: f64, index: usize) -> Result<(), Error> {
    if !mid.is_finite() {
        return Err(Error::NonFinite {
            what: "Chebyshev record midpoint",
            index,
        });
    }
    if !radius.is_finite() || radius <= 0.0 {
        return Err(Error::InvalidChebyshev {
            reason: "Chebyshev record radius must be finite and positive",
        });
    }
    Ok(())
}

pub(super) fn check_finite_slice(
    values: &[f64],
    what: &'static str,
    index: usize,
) -> Result<(), Error> {
    if values.iter().any(|v| !v.is_finite()) {
        return Err(Error::NonFinite { what, index });
    }
    Ok(())
}

pub(super) fn check_sample_inputs<const C: usize>(
    epochs: &[f64],
    values: &[[f64; C]],
) -> Result<(), Error> {
    if epochs.len() != values.len() {
        return Err(Error::LengthMismatch {
            epochs: epochs.len(),
            states: values.len(),
        });
    }
    if let Some(index) = epochs.iter().position(|t| !t.is_finite()) {
        return Err(Error::NonFinite {
            what: "epoch",
            index,
        });
    }
    if let Some(index) = values.iter().position(|v| v.iter().any(|x| !x.is_finite())) {
        return Err(Error::NonFinite {
            what: "sample",
            index,
        });
    }
    for (index, pair) in epochs.windows(2).enumerate() {
        if let [prev, curr] = *pair
            && curr <= prev
        {
            return Err(Error::EpochsOutOfOrder {
                index: index.saturating_add(1),
                epoch: curr,
                previous: prev,
            });
        }
    }
    Ok(())
}

pub(super) fn collect_granule_channels<const C: usize>(
    epochs: &[f64],
    values: &[[f64; C]],
    t_lo: f64,
    t_hi: f64,
    mid: f64,
    radius: f64,
    tol: f64,
) -> (Vec<f64>, Vec<Vec<f64>>) {
    let mut taus = Vec::new();
    let mut channels = vec![Vec::new(); C];
    let start = epochs.partition_point(|&t| t < t_lo - tol);
    for (epoch, sample) in epochs.iter().zip(values).skip(start) {
        if *epoch > t_hi + tol {
            break;
        }
        taus.push((*epoch - mid) / radius);
        for (ch, &val) in channels.iter_mut().zip(sample) {
            ch.push(val);
        }
    }
    (taus, channels)
}

pub(super) fn parse_trailer(
    words: &[f64],
) -> Result<(f64, f64, usize, usize, &[f64]), &'static str> {
    let [.., init, intlen, rsize_f, n_rec_f] = *words else {
        return Err("fewer than four trailer words");
    };
    let rsize = rsize_f as usize;
    let n_records = n_rec_f as usize;
    if rsize as f64 != rsize_f || rsize < 5 {
        return Err("record size is not a valid integer");
    }
    if n_records as f64 != n_rec_f || n_records == 0 {
        return Err("record count is not a positive integer");
    }
    let payload_len = rsize
        .checked_mul(n_records)
        .filter(|&len| len.saturating_add(4) == words.len())
        .ok_or("word count does not match RSIZE * N + 4")?;
    let payload = words.get(..payload_len).unwrap_or(&[]);
    Ok((init, intlen, rsize, n_records, payload))
}

pub(super) fn validate_flat_records(
    init: f64,
    intlen: f64,
    n_coeff: usize,
    n_records: usize,
    payload: &[f64],
    channels: usize,
) -> Result<(), Error> {
    check_header(init, intlen, n_records)?;
    let rsize = 2_usize.saturating_add(channels.saturating_mul(n_coeff));
    for (idx, rec) in payload.chunks_exact(rsize).enumerate() {
        let mid = rec.first().copied().unwrap_or(0.0);
        let radius = rec.get(1).copied().unwrap_or(0.0);
        check_mid_radius(mid, radius, idx)?;
        check_finite_slice(rec.get(2..).unwrap_or(&[]), "Chebyshev coefficient", idx)?;
    }
    Ok(())
}

pub(super) fn select_record(init: f64, intlen: f64, n_records: usize, et: f64) -> usize {
    let raw = libm::floor((et - init) / intlen);
    if raw <= 0.0 {
        0
    } else {
        (raw as usize).min(n_records.saturating_sub(1))
    }
}

pub(super) fn slice_vec(slice: &[f64], start: usize, len: usize) -> Vec<f64> {
    slice
        .get(start..start.saturating_add(len))
        .unwrap_or(&[])
        .to_vec()
}

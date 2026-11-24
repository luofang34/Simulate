//! Polynomial interpolation through unequally spaced samples.
//!
//! Both routines return `None` only when their slices disagree in length,
//! which the callers rule out; it keeps the arithmetic free of panicking
//! index operations.

use alloc::vec::Vec;

fn at(values: &[f64], index: usize) -> Option<f64> {
    values.get(index).copied()
}

/// Value at `x` of the polynomial of degree `xs.len() - 1` through
/// `(xs[i], ys[i])`, by Neville's recurrence. `ys` is the work table and is
/// overwritten.
pub(super) fn lagrange(xs: &[f64], ys: &mut [f64], x: f64) -> Option<f64> {
    let n = xs.len();
    if ys.len() != n {
        return None;
    }
    for span in 1..n {
        for i in 0..n.saturating_sub(span) {
            let left = at(xs, i)?;
            let right = at(xs, i.saturating_add(span))?;
            let lower = at(ys, i)?;
            let upper = at(ys, i.saturating_add(1))?;
            *ys.get_mut(i)? = ((x - right) * lower + (left - x) * upper) / (left - right);
        }
    }
    at(ys, 0)
}

/// Value and derivative at `x` of the polynomial of degree `2 * xs.len() - 1`
/// that matches the values `ys` and derivatives `dys` at every `xs[i]`.
///
/// Newton form on doubled nodes: each node appears twice, the divided
/// difference across a doubled pair is the supplied derivative, and Horner's
/// rule gives the value together with the derivative.
pub(super) fn hermite(xs: &[f64], ys: &[f64], dys: &[f64], x: f64) -> Option<(f64, f64)> {
    let n = xs.len();
    if ys.len() != n || dys.len() != n || n == 0 {
        return None;
    }
    let nodes: Vec<f64> = xs.iter().flat_map(|&node| [node, node]).collect();
    let mut table: Vec<f64> = ys.iter().flat_map(|&value| [value, value]).collect();
    let m = nodes.len();
    for i in (1..m).rev() {
        let difference = if i % 2 == 1 {
            at(dys, i / 2)?
        } else {
            (at(&table, i)? - at(&table, i.saturating_sub(1))?)
                / (at(&nodes, i)? - at(&nodes, i.saturating_sub(1))?)
        };
        *table.get_mut(i)? = difference;
    }
    for order in 2..m {
        for i in (order..m).rev() {
            let difference = (at(&table, i)? - at(&table, i.saturating_sub(1))?)
                / (at(&nodes, i)? - at(&nodes, i.saturating_sub(order))?);
            *table.get_mut(i)? = difference;
        }
    }
    let mut value = at(&table, m.saturating_sub(1))?;
    let mut derivative = 0.0;
    for k in (0..m.saturating_sub(1)).rev() {
        let offset = x - at(&nodes, k)?;
        derivative = derivative * offset + value;
        value = value * offset + at(&table, k)?;
    }
    Some((value, derivative))
}

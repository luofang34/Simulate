//! Keeping a long run's series within a fixed size: a flight that holds after
//! its route goes on indefinitely, and the page cannot keep every sample of it.
//! When a series reaches its limit, each pair of neighbours becomes one and
//! from then on every two new samples become one, so every kept sample stands
//! for the same number of samples and the series keeps its whole span evenly.

#[cfg(test)]
mod tests;

/// Samples a series keeps at most.
pub const SAMPLES_KEPT: usize = 8192;

/// A series thinned to [`SAMPLES_KEPT`]; `keep` picks which of two
/// neighbours stands for both.
#[derive(Debug, Clone)]
pub struct Thinned<T> {
    items: Vec<T>,
    /// Samples each kept one stands for.
    stride: usize,
    /// The samples since the last kept one, merged, and how many they are.
    pending: Option<(T, usize)>,
    /// The latest sample as it came, merged into nothing.
    last: Option<T>,
    keep: fn(T, T) -> T,
}

impl<T: Copy> Thinned<T> {
    /// An empty series.
    pub const fn new(keep: fn(T, T) -> T) -> Self {
        Self {
            items: Vec::new(),
            stride: 1,
            pending: None,
            last: None,
            keep,
        }
    }

    /// Adds `item`.
    pub fn push(&mut self, item: T) {
        self.last = Some(item);
        let (merged, count) = match self.pending.take() {
            Some((held, count)) => ((self.keep)(held, item), count.saturating_add(1)),
            None => (item, 1),
        };
        if count < self.stride {
            self.pending = Some((merged, count));
            return;
        }
        self.items.push(merged);
        if self.items.len() >= SAMPLES_KEPT {
            let mut kept = Vec::with_capacity(SAMPLES_KEPT);
            let mut items = std::mem::take(&mut self.items).into_iter();
            while let Some(first) = items.next() {
                kept.push(
                    items
                        .next()
                        .map_or(first, |second| (self.keep)(first, second)),
                );
            }
            self.items = kept;
            self.stride = self.stride.saturating_mul(2);
        }
    }

    /// The kept samples, oldest first.
    pub fn items(&self) -> &[T] {
        &self.items
    }

    /// The latest sample as it came.
    pub const fn latest(&self) -> Option<&T> {
        self.last.as_ref()
    }
}

impl<T: Copy> Default for Thinned<T> {
    /// An empty series that keeps the older of two neighbours.
    fn default() -> Self {
        Self::new(|older, _| older)
    }
}

impl<T: PartialEq> PartialEq for Thinned<T> {
    /// Equal when the samples are; how two series thin is not compared.
    fn eq(&self, other: &Self) -> bool {
        self.items == other.items
            && self.stride == other.stride
            && self.pending == other.pending
            && self.last == other.last
    }
}

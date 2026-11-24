//! Discrete states at unequally spaced epochs, the data of SPK types 9 and 13.

use alloc::vec::Vec;
use core::ops::Range;

use super::interpolation;
use crate::error::Error;

/// Epochs between two entries of the epoch directory a segment ends with.
const DIRECTORY_SPACING: usize = 100;

/// States sampled at strictly increasing epochs, with the number of
/// consecutive samples one interpolation uses.
///
/// Epochs are TDB seconds past J2000; a state is position in kilometres
/// followed by velocity in kilometres per second.
#[derive(Clone, Debug, PartialEq)]
pub struct SampledStates {
    epochs: Vec<f64>,
    states: Vec<[f64; 6]>,
    window: usize,
}

impl SampledStates {
    /// Checks and stores the samples.
    ///
    /// `window` is the number of consecutive samples used per evaluation: a
    /// type 9 segment interpolates with a polynomial of degree `window - 1`,
    /// a type 13 segment with one of degree `2 * window - 1`.
    pub fn new(epochs: Vec<f64>, states: Vec<[f64; 6]>, window: usize) -> Result<Self, Error> {
        if epochs.len() != states.len() {
            return Err(Error::LengthMismatch {
                epochs: epochs.len(),
                states: states.len(),
            });
        }
        if window == 0 || epochs.len() < window {
            return Err(Error::TooFewStates {
                window,
                found: epochs.len(),
            });
        }
        if let Some(index) = epochs.iter().position(|epoch| !epoch.is_finite()) {
            return Err(Error::NonFinite {
                what: "epoch",
                index,
            });
        }
        let unfinished = |state: &[f64; 6]| state.iter().any(|component| !component.is_finite());
        if let Some(index) = states.iter().position(unfinished) {
            return Err(Error::NonFinite {
                what: "state",
                index,
            });
        }
        for (index, pair) in epochs.windows(2).enumerate() {
            if let [previous, epoch] = *pair
                && epoch <= previous
            {
                return Err(Error::EpochsOutOfOrder {
                    index: index.saturating_add(1),
                    epoch,
                    previous,
                });
            }
        }
        Ok(Self {
            epochs,
            states,
            window,
        })
    }

    /// Sample epochs, strictly increasing.
    #[must_use]
    pub fn epochs(&self) -> &[f64] {
        &self.epochs
    }

    /// One state per epoch.
    #[must_use]
    pub fn states(&self) -> &[[f64; 6]] {
        &self.states
    }

    /// Consecutive samples used per evaluation.
    #[must_use]
    pub const fn window(&self) -> usize {
        self.window
    }

    /// First and last epoch.
    pub(super) fn span(&self) -> [f64; 2] {
        [
            self.epochs.first().copied().unwrap_or(f64::NAN),
            self.epochs.last().copied().unwrap_or(f64::NAN),
        ]
    }

    /// The segment's stored words: states, epochs, every hundredth epoch as a
    /// search directory, the window size less one, and the state count.
    pub(super) fn to_words(&self) -> Vec<f64> {
        let n = self.epochs.len();
        let mut words = Vec::with_capacity(word_count(n));
        words.extend(self.states.iter().flatten());
        words.extend(&self.epochs);
        words.extend(
            self.epochs
                .iter()
                .skip(DIRECTORY_SPACING.saturating_sub(1))
                .step_by(DIRECTORY_SPACING)
                .take(directory_len(n)),
        );
        words.push(self.window.saturating_sub(1) as f64);
        words.push(n as f64);
        words
    }

    /// Rebuilds the samples from a segment's stored words. `Err(reason)` is a
    /// layout fault; value faults come back as the inner typed error.
    pub(super) fn from_words(words: &[f64]) -> Result<Result<Self, Error>, &'static str> {
        let [.., window_less_one, count] = *words else {
            return Err("fewer than two words");
        };
        let n = count as usize;
        if n as f64 != count || n == 0 {
            return Err("state count is not a positive integer");
        }
        let window = (window_less_one as usize).saturating_add(1);
        if window.saturating_sub(1) as f64 != window_less_one {
            return Err("window size is not a positive integer");
        }
        if n.checked_mul(7)
            .is_none_or(|_| word_count(n) != words.len())
        {
            return Err("word count does not match the state count");
        }
        let (state_words, rest) = words.split_at(n.saturating_mul(6));
        let states = state_words.as_chunks::<6>().0.to_vec();
        let epochs = rest.iter().take(n).copied().collect();
        Ok(Self::new(epochs, states, window))
    }

    /// Indices of the samples used at `et`.
    ///
    /// SPICE's rule: an even window is centred on the interval that brackets
    /// `et`, an odd one on the sample nearest `et` (the later one on a tie),
    /// and either is shifted inward when it would overrun the samples.
    pub(super) fn window_range(&self, et: f64) -> Range<usize> {
        let before = self.epochs.partition_point(|&epoch| epoch < et);
        let low = before.saturating_sub(1);
        let centre = if self.window.is_multiple_of(2) || before == 0 {
            low
        } else {
            match (self.epochs.get(low), self.epochs.get(before)) {
                (Some(&earlier), Some(&later)) if later - et <= et - earlier => before,
                _ => low,
            }
        };
        let first = centre
            .saturating_sub(self.window.saturating_sub(1) / 2)
            .min(self.epochs.len().saturating_sub(self.window));
        first..first.saturating_add(self.window)
    }

    /// One component of every state in `range`.
    fn component(&self, range: Range<usize>, index: usize) -> Vec<f64> {
        self.states
            .get(range)
            .unwrap_or(&[])
            .iter()
            .filter_map(|state| state.get(index).copied())
            .collect()
    }

    /// Type 9 evaluation: each of the six components is interpolated on its
    /// own, so velocity comes from the velocity samples.
    pub(super) fn lagrange(&self, et: f64) -> Option<[f64; 6]> {
        let range = self.window_range(et);
        let epochs = self.epochs.get(range.clone())?;
        let mut state = [0.0; 6];
        for (index, slot) in state.iter_mut().enumerate() {
            let mut values = self.component(range.clone(), index);
            *slot = interpolation::lagrange(epochs, &mut values, et)?;
        }
        Some(state)
    }

    /// Type 13 evaluation: each axis is interpolated from its position and
    /// velocity samples together, and velocity is the derivative of that
    /// polynomial.
    pub(super) fn hermite(&self, et: f64) -> Option<[f64; 6]> {
        let range = self.window_range(et);
        let epochs = self.epochs.get(range.clone())?;
        let mut state = [0.0; 6];
        for axis in 0..3_usize {
            let positions = self.component(range.clone(), axis);
            let velocities = self.component(range.clone(), axis.saturating_add(3));
            let (position, velocity) = interpolation::hermite(epochs, &positions, &velocities, et)?;
            *state.get_mut(axis)? = position;
            *state.get_mut(axis.saturating_add(3))? = velocity;
        }
        Some(state)
    }
}

fn directory_len(n: usize) -> usize {
    n.saturating_sub(1) / DIRECTORY_SPACING
}

fn word_count(n: usize) -> usize {
    n.saturating_mul(7)
        .saturating_add(directory_len(n))
        .saturating_add(2)
}

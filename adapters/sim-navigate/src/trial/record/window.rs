//! A log read by absolute position that may forget its oldest entries once
//! no reader will come back for them, so a log grown for as long as a
//! flight lasts keeps only what its readers still need.

use std::collections::VecDeque;

#[cfg(test)]
mod tests;

/// Entries at absolute positions `0..len()`, of which those from
/// [`LogWindow::first_kept`] on are kept.
#[derive(Debug, Clone, PartialEq)]
pub struct LogWindow<T> {
    first: usize,
    items: VecDeque<T>,
}

impl<T> Default for LogWindow<T> {
    fn default() -> Self {
        Self {
            first: 0,
            items: VecDeque::new(),
        }
    }
}

impl<T> From<Vec<T>> for LogWindow<T> {
    fn from(items: Vec<T>) -> Self {
        Self {
            first: 0,
            items: items.into(),
        }
    }
}

impl<T> FromIterator<T> for LogWindow<T> {
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        Self {
            first: 0,
            items: iter.into_iter().collect(),
        }
    }
}

impl<T> Extend<T> for LogWindow<T> {
    fn extend<I: IntoIterator<Item = T>>(&mut self, iter: I) {
        self.items.extend(iter);
    }
}

impl<T> LogWindow<T> {
    /// An empty log.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The entry at absolute position `index`, unless it is past the end or
    /// already forgotten.
    #[must_use]
    pub fn get(&self, index: usize) -> Option<&T> {
        self.items.get(index.checked_sub(self.first)?)
    }

    /// The entry at absolute position `index`, mutably.
    pub fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        self.items.get_mut(index.checked_sub(self.first)?)
    }

    /// The last entry, when one is kept.
    #[must_use]
    pub fn last(&self) -> Option<&T> {
        self.items.back()
    }

    /// Appends `item` at absolute position [`LogWindow::len`].
    pub fn push(&mut self, item: T) {
        self.items.push_back(item);
    }

    /// Entries ever logged, forgotten ones included: the next position.
    #[must_use]
    pub fn len(&self) -> usize {
        self.first.saturating_add(self.items.len())
    }

    /// Whether nothing was ever logged.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The first position still kept.
    #[must_use]
    pub const fn first_kept(&self) -> usize {
        self.first
    }

    /// How many entries are kept.
    #[must_use]
    pub fn kept(&self) -> usize {
        self.items.len()
    }

    /// The kept entries, oldest first.
    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.items.iter()
    }

    /// The kept entries, oldest first, mutably.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.items.iter_mut()
    }

    /// Forgets every entry before absolute position `index`; positions do
    /// not move.
    pub fn release_before(&mut self, index: usize) {
        let forget = index.saturating_sub(self.first).min(self.items.len());
        self.items.drain(..forget);
        self.first = self.first.saturating_add(forget);
    }
}

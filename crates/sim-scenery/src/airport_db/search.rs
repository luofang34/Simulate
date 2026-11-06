//! Lookup by ICAO, FAA or IATA code and ranked free-text search.

use super::AirportDatabase;
use super::model::{AirportIdx, AirportRecord, CompactCode};
use super::rank_records;

/// Airports an empty search lists, in order.
const CURATED: [&str; 16] = [
    "KEWR", "47N", "KDOV", "KJFK", "KSFO", "KORD", "KLAX", "KOSH", "EGLL", "LFPG", "LSZH", "EDDF",
    "RJTT", "YSSY", "VNLK", "LOWI",
];

/// How a code matched: an ICAO indicator ranks above the FAA and IATA codes
/// that collide across airports (`HND` is Haneda's IATA code and a Nevada
/// FAA identifier).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum CodeKind {
    Icao = 0,
    Other = 1,
}

/// Every code of every record, sorted by code, then kind, then prominence.
#[derive(Debug, Clone, Default)]
pub(crate) struct CodeIndex {
    entries: Vec<(CompactCode, CodeKind, AirportIdx)>,
}

impl CodeIndex {
    pub(crate) fn build(records: &[AirportRecord]) -> Self {
        let mut entries = Vec::new();
        for (i, record) in records.iter().enumerate() {
            let Ok(i) = u32::try_from(i) else { continue };
            for code in record.ids.codes() {
                let kind = if record.ids.icao == Some(code) {
                    CodeKind::Icao
                } else {
                    CodeKind::Other
                };
                entries.push((code, kind, AirportIdx(i)));
            }
        }
        entries.sort_unstable();
        Self { entries }
    }

    /// The entries holding exactly `code`.
    fn exact(&self, code: CompactCode) -> &[(CompactCode, CodeKind, AirportIdx)] {
        let start = self.entries.partition_point(|entry| entry.0 < code);
        let end = self.entries.partition_point(|entry| entry.0 <= code);
        self.entries.get(start..end).unwrap_or(&[])
    }

    /// The best match for `code`: an ICAO match, else the most prominent.
    fn best(&self, code: CompactCode, records: &[AirportRecord]) -> Option<AirportIdx> {
        self.exact(code)
            .iter()
            .min_by(|a, b| {
                let rank = match (records.get(a.2.index()), records.get(b.2.index())) {
                    (Some(x), Some(y)) => rank_records(x, y),
                    _ => a.2.cmp(&b.2),
                };
                a.1.cmp(&b.1).then(rank)
            })
            .map(|entry| entry.2)
    }

    /// Whether some code of an airport starts with `prefix`.
    fn prefixed<'a>(&'a self, prefix: &'a str) -> impl Iterator<Item = AirportIdx> + 'a {
        let start = self
            .entries
            .partition_point(|entry| entry.0.as_str() < prefix);
        self.entries
            .get(start..)
            .unwrap_or(&[])
            .iter()
            .take_while(move |entry| entry.0.as_str().starts_with(prefix))
            .map(|entry| entry.2)
    }
}

impl AirportDatabase {
    fn search_keys(&self) -> &[Box<str>] {
        self.search_keys.get_or_init(|| {
            self.records
                .iter()
                .map(|record| {
                    format!("{} {}", record.name, record.municipality)
                        .to_lowercase()
                        .into()
                })
                .collect()
        })
    }

    /// The airport answering to `code` — an ICAO (`KEWR`), FAA (`EWR`,
    /// `47N`) or IATA (`LHR`) code, any case. An ICAO match wins; among FAA
    /// and IATA codes shared by several airports, the most prominent wins.
    #[must_use]
    pub fn by_code(&self, code: &str) -> Option<(AirportIdx, &AirportRecord)> {
        let idx = self.codes.best(CompactCode::parse(code)?, &self.records)?;
        Some((idx, self.get(idx)?))
    }

    /// Up to `limit` airports for a search box. An empty query lists a
    /// curated set of prominent airports. Otherwise matches rank by kind —
    /// exact code, code prefix, a word of the name or municipality starting
    /// with every query word, then any substring of them — and within a kind
    /// by prominence.
    #[must_use]
    pub fn search(&self, query: &str, limit: usize) -> Vec<AirportIdx> {
        let query = query.trim();
        if query.is_empty() {
            return CURATED
                .iter()
                .filter_map(|code| self.by_code(code).map(|(idx, _)| idx))
                .take(limit)
                .collect();
        }
        let upper = query.to_ascii_uppercase();
        let lower = query.to_lowercase();
        let words: Vec<&str> = lower.split_whitespace().collect();
        let exact = CompactCode::parse(query);
        let mut best: Vec<Option<u8>> = vec![None; self.records.len()];
        let mut mark = |idx: AirportIdx, kind: u8| {
            if let Some(slot) = best.get_mut(idx.index()) {
                *slot = Some(slot.map_or(kind, |k| k.min(kind)));
            }
        };
        if let Some(code) = exact {
            for entry in self.codes.exact(code) {
                mark(entry.2, 0);
            }
        }
        for idx in self.codes.prefixed(&upper) {
            mark(idx, 1);
        }
        for (i, key) in self.search_keys().iter().enumerate() {
            let Ok(i) = u32::try_from(i) else { continue };
            if words.iter().all(|word| {
                key.split(|c: char| !c.is_alphanumeric())
                    .any(|w| w.starts_with(word))
            }) {
                mark(AirportIdx(i), 2);
            } else if key.contains(lower.as_str()) {
                mark(AirportIdx(i), 3);
            }
        }
        let mut hits: Vec<(u8, AirportIdx)> = self
            .indices()
            .filter_map(|idx| {
                best.get(idx.index())
                    .copied()
                    .flatten()
                    .map(|kind| (kind, idx))
            })
            .collect();
        hits.sort_by(|a, b| a.0.cmp(&b.0).then_with(|| self.rank(a.1, b.1)));
        hits.into_iter().take(limit).map(|(_, idx)| idx).collect()
    }
}

//! Parser for NAIF Kernel Pool (`KPL/PCK`, `KPL/FK`, `KPL/LSK`) text kernels.
//!
//! Reads variable assignments inside `\begindata` .. `\begintext` blocks,
//! supporting Fortran `D`/`d` exponents, multi-line parenthesized vectors,
//! `+=` append assignments, `'STRING'` literals, and `@YYYY-MON-DD` dates.

use std::collections::BTreeMap;

use crate::hash::ContentHash;

/// Why parsing a NAIF text kernel failed.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum KplError {
    /// Input bytes were not valid UTF-8 / ASCII text.
    #[error("NAIF text kernel is not valid UTF-8")]
    InvalidUtf8,
    /// Unclosed single-quoted string literal.
    #[error("unterminated string literal in NAIF text kernel")]
    UnterminatedString,
    /// Malformed assignment statement inside a data block.
    #[error("malformed KPL assignment near `{0}`")]
    MalformedAssignment(String),
    /// Expected a numeric variable that was missing or non-numeric.
    #[error("missing or non-numeric KPL variable `{0}`")]
    MissingNumeric(String),
}

/// One scalar element in a NAIF text-kernel variable value list.
#[derive(Clone, Debug, PartialEq)]
pub enum KplAtom {
    /// Floating-point scalar (parsed with `D`/`d` normalized to `E`).
    Float(f64),
    /// Single-quoted string value (with doubled `''` unescaped).
    Text(String),
    /// `@YYYY-MON-DD` calendar epoch token (without leading `@`).
    CalendarEpoch(String),
}

/// Parsed NAIF text-kernel variable pool and its source SHA-256 hash.
#[derive(Clone, Debug)]
pub struct KplPool {
    hash: ContentHash,
    vars: BTreeMap<String, Vec<KplAtom>>,
}

impl KplPool {
    /// Parses all `\begindata` blocks from `bytes` and records the source SHA-256 digest.
    pub fn parse_bytes(bytes: &[u8]) -> Result<Self, KplError> {
        let text = core::str::from_utf8(bytes).map_err(|_| KplError::InvalidUtf8)?;
        let hash = ContentHash::of_bytes(bytes);
        let data_src = extract_data_sections(text);
        let tokens = tokenize_data(&data_src)?;
        let vars = parse_assignments(&tokens)?;
        Ok(Self { hash, vars })
    }

    /// SHA-256 digest of the raw text kernel bytes.
    #[must_use]
    pub const fn content_hash(&self) -> ContentHash {
        self.hash
    }

    /// Raw atom slice for variable `name`, if present.
    #[must_use]
    pub fn atoms(&self, name: &str) -> Option<&[KplAtom]> {
        self.vars.get(name).map(Vec::as_slice)
    }

    /// Iterates over all `(variable_name, atoms)` entries in lexicographic order.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &[KplAtom])> {
        self.vars.iter().map(|(k, v)| (k.as_str(), v.as_slice()))
    }

    /// Returns numeric values for `name`, or `None` if absent or containing non-floats.
    #[must_use]
    pub fn floats(&self, name: &str) -> Option<Vec<f64>> {
        let list = self.vars.get(name)?;
        let mut out = Vec::with_capacity(list.len());
        for atom in list {
            match *atom {
                KplAtom::Float(v) => out.push(v),
                KplAtom::Text(_) | KplAtom::CalendarEpoch(_) => return None,
            }
        }
        Some(out)
    }

    /// Returns numeric values for `name`, failing closed if absent or non-numeric.
    pub fn require_floats(&self, name: &str) -> Result<Vec<f64>, KplError> {
        self.floats(name)
            .ok_or_else(|| KplError::MissingNumeric(name.to_owned()))
    }

    /// Returns a single scalar `f64` for `name`, failing closed if absent or non-scalar.
    pub fn require_scalar(&self, name: &str) -> Result<f64, KplError> {
        let vals = self.require_floats(name)?;
        if vals.len() != 1 {
            return Err(KplError::MissingNumeric(name.to_owned()));
        }
        vals.first()
            .copied()
            .ok_or_else(|| KplError::MissingNumeric(name.to_owned()))
    }

    /// Returns the first string value of `name`, if present.
    #[must_use]
    pub fn first_text(&self, name: &str) -> Option<&str> {
        match self.vars.get(name)?.first()? {
            KplAtom::Text(s) => Some(s.as_str()),
            KplAtom::Float(_) | KplAtom::CalendarEpoch(_) => None,
        }
    }
}

fn extract_data_sections(text: &str) -> String {
    let mut out = String::new();
    let mut in_data = false;
    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed == "\\begindata" {
            in_data = true;
            continue;
        }
        if trimmed == "\\begintext" {
            in_data = false;
            continue;
        }
        if in_data {
            out.push_str(line);
            out.push('\n');
        }
    }
    out
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum RawTok {
    LParen,
    RParen,
    Assign,
    PlusAssign,
    Quoted(String),
    Bare(String),
}

fn tokenize_data(src: &str) -> Result<Vec<RawTok>, KplError> {
    let mut out = Vec::new();
    let mut chars = src.chars().peekable();
    while let Some(&ch) = chars.peek() {
        if ch.is_ascii_whitespace() || ch == ',' {
            let _ = chars.next();
        } else if ch == '(' {
            let _ = chars.next();
            out.push(RawTok::LParen);
        } else if ch == ')' {
            let _ = chars.next();
            out.push(RawTok::RParen);
        } else if ch == '=' {
            let _ = chars.next();
            out.push(RawTok::Assign);
        } else if ch == '\'' {
            out.push(RawTok::Quoted(read_quoted(&mut chars)?));
        } else {
            let word = read_bare(&mut chars);
            if word == "+" && chars.peek() == Some(&'=') {
                let _ = chars.next();
                out.push(RawTok::PlusAssign);
            } else {
                out.push(RawTok::Bare(word));
            }
        }
    }
    Ok(out)
}

fn read_quoted(chars: &mut core::iter::Peekable<core::str::Chars<'_>>) -> Result<String, KplError> {
    let _ = chars.next();
    let mut buf = String::new();
    while let Some(ch) = chars.next() {
        if ch == '\'' {
            if chars.peek() == Some(&'\'') {
                let _ = chars.next();
                buf.push('\'');
            } else {
                return Ok(buf);
            }
        } else {
            buf.push(ch);
        }
    }
    Err(KplError::UnterminatedString)
}

fn read_bare(chars: &mut core::iter::Peekable<core::str::Chars<'_>>) -> String {
    let mut buf = String::new();
    while let Some(&ch) = chars.peek() {
        if ch.is_ascii_whitespace() || matches!(ch, ',' | '(' | ')' | '=' | '\'') {
            break;
        }
        buf.push(ch);
        let _ = chars.next();
    }
    buf
}

fn parse_assignments(tokens: &[RawTok]) -> Result<BTreeMap<String, Vec<KplAtom>>, KplError> {
    let mut map: BTreeMap<String, Vec<KplAtom>> = BTreeMap::new();
    let mut i = 0_usize;
    while let Some(tok) = tokens.get(i) {
        let RawTok::Bare(name) = tok else {
            return Err(KplError::MalformedAssignment(format!("{tok:?}")));
        };
        let op = tokens
            .get(i.wrapping_add(1))
            .ok_or_else(|| KplError::MalformedAssignment(name.clone()))?;
        let append = match op {
            RawTok::Assign => false,
            RawTok::PlusAssign => true,
            _ => return Err(KplError::MalformedAssignment(name.clone())),
        };
        i = i.wrapping_add(2);
        let (vals, next_i) = parse_rhs(tokens, i, name)?;
        i = next_i;
        if append {
            map.entry(name.clone()).or_default().extend(vals);
        } else {
            map.insert(name.clone(), vals);
        }
    }
    Ok(map)
}

fn parse_rhs(
    tokens: &[RawTok],
    start: usize,
    name: &str,
) -> Result<(Vec<KplAtom>, usize), KplError> {
    let first = tokens
        .get(start)
        .ok_or_else(|| KplError::MalformedAssignment(name.to_owned()))?;
    if *first == RawTok::LParen {
        let mut i = start.wrapping_add(1);
        let mut vals = Vec::new();
        while let Some(tok) = tokens.get(i) {
            if *tok == RawTok::RParen {
                return Ok((vals, i.wrapping_add(1)));
            }
            vals.push(atom_from_tok(tok, name)?);
            i = i.wrapping_add(1);
        }
        Err(KplError::MalformedAssignment(name.to_owned()))
    } else {
        Ok((vec![atom_from_tok(first, name)?], start.wrapping_add(1)))
    }
}

fn atom_from_tok(tok: &RawTok, name: &str) -> Result<KplAtom, KplError> {
    match tok {
        RawTok::Quoted(s) => Ok(KplAtom::Text(s.clone())),
        RawTok::Bare(s) => {
            if let Some(cal) = s.strip_prefix('@') {
                return Ok(KplAtom::CalendarEpoch(cal.to_owned()));
            }
            let normalized = s.replace(['D', 'd'], "E");
            let val: f64 = normalized
                .parse()
                .map_err(|_| KplError::MalformedAssignment(format!("{name}: {s}")))?;
            if !val.is_finite() {
                return Err(KplError::MalformedAssignment(format!("{name}: {s}")));
            }
            Ok(KplAtom::Float(val))
        }
        RawTok::LParen | RawTok::RParen | RawTok::Assign | RawTok::PlusAssign => {
            Err(KplError::MalformedAssignment(name.to_owned()))
        }
    }
}

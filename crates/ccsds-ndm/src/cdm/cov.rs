//! CDM 6x6 RTN covariance (with optional 7x7 drag, 8x8 SRP, and 9x9 thrust extensions).

use crate::block::{Block, Entry, Fields, ValueKind};
use crate::covariance::Covariance6;
use crate::error::{Error, ErrorKind, Result};
use crate::{ParseMode, WriteOptions, number};

/// Keyword metadata for one lower-triangular CDM RTN covariance entry.
#[derive(Clone, Copy)]
struct CovField {
    keyword: &'static str,
    unit: &'static str,
}

const COV_6X6: [CovField; 21] = [
    CovField {
        keyword: "CR_R",
        unit: "m**2",
    },
    CovField {
        keyword: "CT_R",
        unit: "m**2",
    },
    CovField {
        keyword: "CT_T",
        unit: "m**2",
    },
    CovField {
        keyword: "CN_R",
        unit: "m**2",
    },
    CovField {
        keyword: "CN_T",
        unit: "m**2",
    },
    CovField {
        keyword: "CN_N",
        unit: "m**2",
    },
    CovField {
        keyword: "CRDOT_R",
        unit: "m**2/s",
    },
    CovField {
        keyword: "CRDOT_T",
        unit: "m**2/s",
    },
    CovField {
        keyword: "CRDOT_N",
        unit: "m**2/s",
    },
    CovField {
        keyword: "CRDOT_RDOT",
        unit: "m**2/s**2",
    },
    CovField {
        keyword: "CTDOT_R",
        unit: "m**2/s",
    },
    CovField {
        keyword: "CTDOT_T",
        unit: "m**2/s",
    },
    CovField {
        keyword: "CTDOT_N",
        unit: "m**2/s",
    },
    CovField {
        keyword: "CTDOT_RDOT",
        unit: "m**2/s**2",
    },
    CovField {
        keyword: "CTDOT_TDOT",
        unit: "m**2/s**2",
    },
    CovField {
        keyword: "CNDOT_R",
        unit: "m**2/s",
    },
    CovField {
        keyword: "CNDOT_T",
        unit: "m**2/s",
    },
    CovField {
        keyword: "CNDOT_N",
        unit: "m**2/s",
    },
    CovField {
        keyword: "CNDOT_RDOT",
        unit: "m**2/s**2",
    },
    CovField {
        keyword: "CNDOT_TDOT",
        unit: "m**2/s**2",
    },
    CovField {
        keyword: "CNDOT_NDOT",
        unit: "m**2/s**2",
    },
];

const COV_DRG: [CovField; 7] = [
    CovField {
        keyword: "CDRG_R",
        unit: "m**3/kg",
    },
    CovField {
        keyword: "CDRG_T",
        unit: "m**3/kg",
    },
    CovField {
        keyword: "CDRG_N",
        unit: "m**3/kg",
    },
    CovField {
        keyword: "CDRG_RDOT",
        unit: "m**3/(kg*s)",
    },
    CovField {
        keyword: "CDRG_TDOT",
        unit: "m**3/(kg*s)",
    },
    CovField {
        keyword: "CDRG_NDOT",
        unit: "m**3/(kg*s)",
    },
    CovField {
        keyword: "CDRG_DRG",
        unit: "m**4/kg**2",
    },
];

const COV_SRP: [CovField; 8] = [
    CovField {
        keyword: "CSRP_R",
        unit: "m**3/kg",
    },
    CovField {
        keyword: "CSRP_T",
        unit: "m**3/kg",
    },
    CovField {
        keyword: "CSRP_N",
        unit: "m**3/kg",
    },
    CovField {
        keyword: "CSRP_RDOT",
        unit: "m**3/(kg*s)",
    },
    CovField {
        keyword: "CSRP_TDOT",
        unit: "m**3/(kg*s)",
    },
    CovField {
        keyword: "CSRP_NDOT",
        unit: "m**3/(kg*s)",
    },
    CovField {
        keyword: "CSRP_DRG",
        unit: "m**4/kg**2",
    },
    CovField {
        keyword: "CSRP_SRP",
        unit: "m**4/kg**2",
    },
];

const COV_THR: [CovField; 9] = [
    CovField {
        keyword: "CTHR_R",
        unit: "m**2/s**2",
    },
    CovField {
        keyword: "CTHR_T",
        unit: "m**2/s**2",
    },
    CovField {
        keyword: "CTHR_N",
        unit: "m**2/s**2",
    },
    CovField {
        keyword: "CTHR_RDOT",
        unit: "m**2/s**3",
    },
    CovField {
        keyword: "CTHR_TDOT",
        unit: "m**2/s**3",
    },
    CovField {
        keyword: "CTHR_NDOT",
        unit: "m**2/s**3",
    },
    CovField {
        keyword: "CTHR_DRG",
        unit: "m**3/(kg*s**2)",
    },
    CovField {
        keyword: "CTHR_SRP",
        unit: "m**3/(kg*s**2)",
    },
    CovField {
        keyword: "CTHR_THR",
        unit: "m**2/s**4",
    },
];

pub(super) const COVARIANCE_KEYWORDS: &[&str] = &[
    "CR_R",
    "CT_R",
    "CT_T",
    "CN_R",
    "CN_T",
    "CN_N",
    "CRDOT_R",
    "CRDOT_T",
    "CRDOT_N",
    "CRDOT_RDOT",
    "CTDOT_R",
    "CTDOT_T",
    "CTDOT_N",
    "CTDOT_RDOT",
    "CTDOT_TDOT",
    "CNDOT_R",
    "CNDOT_T",
    "CNDOT_N",
    "CNDOT_RDOT",
    "CNDOT_TDOT",
    "CNDOT_NDOT",
    "CDRG_R",
    "CDRG_T",
    "CDRG_N",
    "CDRG_RDOT",
    "CDRG_TDOT",
    "CDRG_NDOT",
    "CDRG_DRG",
    "CSRP_R",
    "CSRP_T",
    "CSRP_N",
    "CSRP_RDOT",
    "CSRP_TDOT",
    "CSRP_NDOT",
    "CSRP_DRG",
    "CSRP_SRP",
    "CTHR_R",
    "CTHR_T",
    "CTHR_N",
    "CTHR_RDOT",
    "CTHR_TDOT",
    "CTHR_NDOT",
    "CTHR_DRG",
    "CTHR_SRP",
    "CTHR_THR",
];

/// Object RTN covariance matrix (`<covarianceMatrix>`, `m**2`, `m**2/s`, `m**2/s**2`).
#[derive(Debug, Clone, PartialEq)]
pub struct CdmCovariance {
    /// Comments attached to the covariance block.
    pub comments: Vec<String>,
    /// Mandatory 6x6 RTN position-velocity covariance (`m`, `s`).
    pub matrix: Covariance6,
    /// Optional 7th row (`CDRG_R`..`CDRG_DRG`).
    pub drag_row: Option<[f64; 7]>,
    /// Optional 8th row (`CSRP_R`..`CSRP_SRP`).
    pub srp_row: Option<[f64; 8]>,
    /// Optional 9th row (`CTHR_R`..`CTHR_THR`).
    pub thrust_row: Option<[f64; 9]>,
}

impl CdmCovariance {
    /// Builds a 6x6 RTN CDM covariance with no solve-for extension rows.
    pub const fn new(matrix: Covariance6) -> Self {
        Self {
            comments: Vec::new(),
            matrix,
            drag_row: None,
            srp_row: None,
            thrust_row: None,
        }
    }

    pub(super) fn read(
        comments: Vec<String>,
        fields: &mut Fields,
        mode: ParseMode,
    ) -> Result<Self> {
        let lower = [0.0; 21];
        let mut vals = lower;
        for (dst, spec) in vals.iter_mut().zip(COV_6X6) {
            *dst = fields.required(spec.keyword, |e, m| parse_cov_entry(e, spec.unit, m))?;
        }
        let drag_row = read_opt_row(fields, &COV_DRG)?;
        let srp_row = read_opt_row(fields, &COV_SRP)?;
        let thrust_row = read_opt_row(fields, &COV_THR)?;
        let _ = mode;
        Ok(Self {
            comments,
            matrix: Covariance6::from_lower(vals),
            drag_row,
            srp_row,
            thrust_row,
        })
    }

    pub(super) fn write(&self, options: WriteOptions) -> Result<Block> {
        let mut block = Block {
            comments: self.comments.clone(),
            entries: Vec::with_capacity(45),
            at: None,
        };
        let lower = *self.matrix.lower();
        for (val, spec) in lower.into_iter().zip(COV_6X6) {
            push_cov_entry(&mut block, spec, val, options)?;
        }
        if let Some(row) = self.drag_row {
            for (val, spec) in row.into_iter().zip(COV_DRG) {
                push_cov_entry(&mut block, spec, val, options)?;
            }
        }
        if let Some(row) = self.srp_row {
            for (val, spec) in row.into_iter().zip(COV_SRP) {
                push_cov_entry(&mut block, spec, val, options)?;
            }
        }
        if let Some(row) = self.thrust_row {
            for (val, spec) in row.into_iter().zip(COV_THR) {
                push_cov_entry(&mut block, spec, val, options)?;
            }
        }
        Ok(block)
    }
}

fn read_opt_row<const N: usize>(
    fields: &mut Fields,
    specs: &[CovField; N],
) -> Result<Option<[f64; N]>> {
    let keys = specs.map(|s| s.keyword);
    if !fields.has_any(&keys) {
        return Ok(None);
    }
    let mut row = [0.0; N];
    for (dst, spec) in row.iter_mut().zip(*specs) {
        *dst = fields.required(spec.keyword, |e, m| parse_cov_entry(e, spec.unit, m))?;
    }
    Ok(Some(row))
}

fn parse_cov_entry(entry: &Entry, expected_unit: &'static str, mode: ParseMode) -> Result<f64> {
    if entry.is_empty() {
        return Err(entry.error(ErrorKind::EmptyValue));
    }
    let trimmed = entry.value.trim();
    let (num_str, parsed_unit) = if let Some(unit) = entry.unit.as_deref() {
        (trimmed, Some(unit))
    } else if let Some(inner) = trimmed.strip_suffix(']')
        && let Some((head, unit)) = inner.rsplit_once('[')
    {
        (head.trim(), Some(unit.trim()))
    } else {
        (trimmed, None)
    };
    if mode == ParseMode::Strict
        && let Some(unit) = parsed_unit
        && unit != expected_unit
    {
        return Err(entry.error(ErrorKind::UnitMismatch {
            expected: expected_unit,
            found: unit.to_owned(),
        }));
    }
    number::parse_f64(num_str, mode).ok_or_else(|| {
        entry.error(ErrorKind::InvalidNumber {
            text: num_str.to_owned(),
        })
    })
}

fn push_cov_entry(
    block: &mut Block,
    spec: CovField,
    value: f64,
    options: WriteOptions,
) -> Result<()> {
    let formatted = number::format_f64(value)
        .ok_or_else(|| Error::new(ErrorKind::NonFiniteNumber).with_keyword(spec.keyword))?;
    block.entries.push(Entry {
        keyword: spec.keyword.to_owned(),
        value: formatted,
        unit: options.units.then(|| spec.unit.to_owned()),
        at: None,
        kind: ValueKind::Text,
    });
    Ok(())
}

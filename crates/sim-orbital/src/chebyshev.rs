//! Exact fixed-interval Chebyshev polynomial evaluation up to second derivatives.
//!
//! Re-exported from [`navigate_astrodynamics::spk`] so `Navigate` and `Simulate`
//! share a single Chebyshev evaluator across SPK and PCK segments.

pub use navigate_astrodynamics::spk::{ChebyshevError, eval_type2_record_3d};

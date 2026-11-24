//! Scratch memory for one harmonic synthesis.
//!
//! A synthesis needs a workspace sized by its degree. Every evaluation
//! here owns its own, so a field shared between threads has no mutable
//! state and gives the same bits whichever thread asks.

use world_potential::{FieldError, Workspace, workspace_len};

/// Highest degree served from the small stack buffer: the low-degree
/// reference fields every flight step evaluates.
const SMALL_DEGREE: usize = 12;
/// Highest degree served from the stack at all; above it the buffer is a
/// heap allocation per evaluation.
const STACK_DEGREE: usize = 72;

/// Runs `f` with a workspace for `degree`, owned by this call.
pub(crate) fn with_workspace<T>(
    degree: usize,
    f: impl FnOnce(&mut Workspace<'_>) -> Result<T, FieldError>,
) -> Result<T, FieldError> {
    if degree <= SMALL_DEGREE {
        let mut buffer = [0.0_f64; workspace_len(SMALL_DEGREE)];
        f(&mut Workspace::new(&mut buffer, degree)?)
    } else if degree <= STACK_DEGREE {
        let mut buffer = [0.0_f64; workspace_len(STACK_DEGREE)];
        f(&mut Workspace::new(&mut buffer, degree)?)
    } else {
        let mut buffer = vec![0.0_f64; workspace_len(degree)];
        f(&mut Workspace::new(&mut buffer, degree)?)
    }
}

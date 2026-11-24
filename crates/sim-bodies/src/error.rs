//! Why a field could not be built from its data.

use world_potential::{FieldError, ParseError};

/// Why a gravity or magnetic field was not built.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum FieldBuildError {
    /// The constants or coefficients do not make a model.
    #[error("the {what} model was refused")]
    Model {
        /// Which model was being built.
        what: &'static str,
        /// What the model refused.
        #[source]
        source: FieldError,
    },
    /// A coefficient file could not be read.
    #[error("the {format} coefficient text could not be read")]
    Parse {
        /// The file format being read.
        format: &'static str,
        /// The line and what was wrong with it.
        #[source]
        source: ParseError,
    },
}

impl FieldBuildError {
    pub(crate) fn model(what: &'static str) -> impl Fn(FieldError) -> Self {
        move |source| Self::Model { what, source }
    }

    pub(crate) fn parse(format: &'static str) -> impl Fn(ParseError) -> Self {
        move |source| Self::Parse { format, source }
    }
}

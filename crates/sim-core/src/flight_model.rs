//! The shared machinery under the vehicle blocks: a multibody model loaded
//! by force elements, integrated one period at a time around a rotating
//! body, and read back for telemetry.

pub(crate) mod coupled_mechanism;
pub(crate) mod elements;
pub(crate) mod environment;
pub(crate) mod integration;
pub(crate) mod loads;
pub(crate) mod planet_frame;

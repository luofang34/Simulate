//! The truth fields of one body, as the handles its consumers share.

use std::sync::Arc;

use sim_geo::{GravityField, MagneticField};

/// One body's truth gravity and magnetic field. A vehicle's dynamics and
/// every device on it are built from the same `BodyFields`, so what its
/// sensors read is the field it flies through.
#[derive(Debug, Clone)]
pub struct BodyFields {
    /// The gravity dynamics integrate and inertial or gravimetric sensors
    /// read.
    pub gravity: Arc<dyn GravityField>,
    /// The field magnetometers read and magnetic torques act in.
    pub magnetic: Arc<dyn MagneticField>,
}

impl BodyFields {
    /// Whether `gravity` and `magnetic` are the very instances held here,
    /// not equal copies: what a host checks before it wires a device to a
    /// vehicle.
    #[must_use]
    pub fn are_instances(
        &self,
        gravity: &Arc<dyn GravityField>,
        magnetic: &Arc<dyn MagneticField>,
    ) -> bool {
        std::ptr::addr_eq(Arc::as_ptr(&self.gravity), Arc::as_ptr(gravity))
            && std::ptr::addr_eq(Arc::as_ptr(&self.magnetic), Arc::as_ptr(magnetic))
    }
}

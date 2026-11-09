use std::ops::Deref;

use jsbsim::MemoryResolver;
use syspack::{InstrumentPackDocument, MountedDeviceSpec, PackageSource, VerifiedSyspack};

use crate::instruments::{pitot_static_device, resolve_pinned_pack};
use crate::{
    AIRSPEED_ENVELOPE_ASSET, AirspeedEnvelope, CALIBRATION_SUITE_ASSET, CalibrationSuite,
    LIVERY_CATALOG_ASSET, LiveryCatalog, PITOT_STATIC_CALIBRATION_ASSET, PackError,
    PitotStaticCalibration, ResolvedAirspeedEnvelope,
};

mod devices;

pub use devices::{CotsDevice, MountedCotsDevice, PackageCamera};

#[cfg(test)]
mod tests;

/// A verified system package bound to this simulator: everything a
/// [`VerifiedSyspack`] holds, reached through `Deref`, plus the blocks built
/// from it. Holding one proves the devices, cameras, and any declared airspeed
/// envelope, pitot-static `PEC` table, or POH calibration suite are valid too.
#[derive(Debug, Clone, PartialEq)]
pub struct LoadedSyspack {
    verified: VerifiedSyspack,
    airspeed_envelope: Option<AirspeedEnvelope>,
    pitot_static_calibration: Option<PitotStaticCalibration>,
    calibration_suite: Option<CalibrationSuite>,
    livery_catalog: Option<LiveryCatalog>,
    instrument_pack: Option<VerifiedSyspack>,
}

impl Deref for LoadedSyspack {
    type Target = VerifiedSyspack;

    fn deref(&self) -> &VerifiedSyspack {
        &self.verified
    }
}

impl LoadedSyspack {
    /// Reads and verifies the package in `source`, then builds its devices and
    /// validates any declared airspeed, `PEC`, or POH calibration assets fail-closed.
    pub fn load(source: &dyn PackageSource) -> Result<Self, PackError> {
        let verified = VerifiedSyspack::load(source)?;
        let airspeed_envelope = match verified.asset(AIRSPEED_ENVELOPE_ASSET) {
            Some(bytes) => Some(AirspeedEnvelope::from_json_bytes(bytes)?),
            None => None,
        };
        let pitot_static_calibration = match verified.asset(PITOT_STATIC_CALIBRATION_ASSET) {
            Some(bytes) => Some(PitotStaticCalibration::from_json_bytes(bytes)?),
            None => None,
        };
        let calibration_suite = match verified.asset(CALIBRATION_SUITE_ASSET) {
            Some(bytes) => Some(CalibrationSuite::from_json_bytes(bytes)?),
            None => None,
        };
        let livery_catalog = match verified.asset(LIVERY_CATALOG_ASSET) {
            Some(bytes) => Some(LiveryCatalog::from_verified(&verified, bytes)?),
            None => None,
        };
        let instrument_pack = resolve_pinned_pack(&verified)?;
        let package = Self {
            verified,
            airspeed_envelope,
            pitot_static_calibration,
            calibration_suite,
            livery_catalog,
            instrument_pack,
        };
        package.instantiate_cots_devices()?;
        package.camera_mounts()?;
        Ok(package)
    }

    /// Published airspeed limitations and V-speeds when this package declares
    /// [`AIRSPEED_ENVELOPE_ASSET`].
    #[must_use]
    pub const fn airspeed_envelope(&self) -> Option<&AirspeedEnvelope> {
        self.airspeed_envelope.as_ref()
    }

    /// Flap-indexed pitot-static position error (`KIAS` ↔ `KCAS`) curves when
    /// this package declares [`PITOT_STATIC_CALIBRATION_ASSET`].
    #[must_use]
    pub const fn pitot_static_calibration(&self) -> Option<&PitotStaticCalibration> {
        self.pitot_static_calibration.as_ref()
    }

    /// Declarative POH and empirical flight-test suite when this package declares
    /// [`CALIBRATION_SUITE_ASSET`].
    #[must_use]
    pub const fn calibration_suite(&self) -> Option<&CalibrationSuite> {
        self.calibration_suite.as_ref()
    }

    /// Multi-livery and FlightGear `<PropertyList>` catalog when this package declares
    /// [`LIVERY_CATALOG_ASSET`].
    #[must_use]
    pub const fn livery_catalog(&self) -> Option<&LiveryCatalog> {
        self.livery_catalog.as_ref()
    }

    /// The instruments of the pack this vehicle pins, verified against the pin.
    #[must_use]
    pub fn instrument_pack(&self) -> Option<&InstrumentPackDocument> {
        self.instrument_pack.as_ref()?.instrument_pack()
    }

    /// The vehicle's pitot-static device with `id`, which instruments bound to
    /// `pitot_static.<id>` read.
    #[must_use]
    pub fn pitot_static_device(&self, id: &str) -> Option<&MountedDeviceSpec> {
        pitot_static_device(&self.verified, id)
    }

    /// Resolves the package's [`AirspeedEnvelope`] at `mass_kg` (`kg`).
    #[must_use]
    pub fn airspeed_envelope_at_mass(&self, mass_kg: f64) -> Option<ResolvedAirspeedEnvelope> {
        self.airspeed_envelope
            .as_ref()
            .map(|env| env.resolve_at_mass(mass_kg))
    }

    /// Resolves the package's [`AirspeedEnvelope`] at the [`mbd_core::ComponentTree`]'s
    /// loaded gross mass.
    #[must_use]
    pub fn resolved_airspeed_envelope(&self) -> Option<ResolvedAirspeedEnvelope> {
        let env = self.airspeed_envelope.as_ref()?;
        let mass_kg = self
            .verified
            .tree()
            .mass_properties()
            .map_or(env.reference_mass_kg, |props| props.total_mass_kg);
        Some(env.resolve_at_mass(mass_kg))
    }

    /// The aircraft directory and model name a [`sim_core::JsbsimFdmConfig`]
    /// needs.
    pub fn jsbsim_model(&self) -> Result<(&str, &str), PackError> {
        Ok(self.verified.jsbsim_model()?)
    }

    /// The verified aircraft files at their JSBSim virtual paths, so a
    /// [`sim_core::JsbsimFdmBlock`] flies the package with no file system.
    pub fn build_jsbsim_resolver(&self) -> Result<MemoryResolver, PackError> {
        let mut files = MemoryResolver::new();
        for (path, bytes) in self.jsbsim_files()? {
            files.insert(path, bytes);
        }
        Ok(files)
    }
}

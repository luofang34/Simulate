//! Package metadata extracted when an external `syspack.json` vehicle is imported into `LiverySession`.

use glam::DVec3;
use sim_maplibre::StructuralDatum;
#[cfg(not(target_arch = "wasm32"))]
use sim_maplibre::UsdAircraftAsset;
#[cfg(not(target_arch = "wasm32"))]
use sim_pack::LoadedSyspack;
use sim_pack::{AirspeedEnvelope, PitotStaticCalibration};
use syspack::InstrumentPackDocument;

#[cfg(not(target_arch = "wasm32"))]
use crate::camera_view::DEFAULT_CHASE_BACK_UP_M;
use crate::camera_view::VehicleCameraSuite;

#[cfg(not(target_arch = "wasm32"))]
const METRES_PER_INCH: f64 = 0.0254;
#[cfg(not(target_arch = "wasm32"))]
const KNOTS_TO_MPS: f64 = 1852.0 / 3600.0;

/// Metadata, airspeed envelope, camera suite, and structural datum for one imported external aircraft.
#[derive(Debug, Clone)]
pub struct ExternalVehicleMeta {
    /// Assignable slot index backing [`crate::vehicles::FreeFlightVehicle::External`] and [`crate::esc_menu::catalog::HangarModel::External`].
    pub slot: u16,
    /// Stable variant package identifier (e.g. `"tu-95ms"`, `"tu-160"`).
    pub aircraft_id: String,
    /// Human-readable aircraft display title.
    pub title: String,
    /// Optional family grouping title.
    pub family_title: Option<String>,
    /// Representative subsonic cruise airspeed (`kt`).
    pub cruise_kt: f64,
    /// Representative subsonic cruise true airspeed at low altitude (`m/s`).
    pub cruise_mps: f64,
    /// Design reference / loaded initial gross mass (`kg`).
    pub mass_kg: f64,
    /// Wingspan (`m`) from JSBSim `<metrics><wingspan>`.
    pub wingspan_m: f64,
    /// Least-squares fit locating the USD origin inside the JSBSim structural frame (`IN`).
    pub structural_datum: Option<StructuralDatum>,
    /// Pilot eye position in body FRD metres relative to the USD origin.
    pub pilot_eye_frd_m: DVec3,
    /// Parked landing-gear clearance below CG (`m`).
    pub parked_gear_height_m: f64,
    /// Chase offsets and cyclable package cameras for this airframe.
    pub camera_suite: VehicleCameraSuite,
    /// Optional mass-scaled airspeed envelope.
    pub envelope: Option<(AirspeedEnvelope, f64)>,
    /// Optional pitot-static position-error calibration.
    pub pec: Option<PitotStaticCalibration>,
    /// Optional instrument-pack pin.
    pub instruments: Option<InstrumentPackDocument>,
}

impl ExternalVehicleMeta {
    /// Builds [`ExternalVehicleMeta`] from an already verified external [`LoadedSyspack`] and
    /// its default [`UsdAircraftAsset`] (if OpenUSD layers exist).
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub fn from_package(
        slot: u16,
        aircraft_id: &str,
        title: &str,
        family_title: Option<String>,
        pkg: &LoadedSyspack,
        default_asset: Option<&UsdAircraftAsset>,
    ) -> Self {
        let mass_kg = pkg
            .tree()
            .mass_properties()
            .map_or(1_200.0, |props| props.total_mass_kg);
        let wingspan_m = pkg.jsbsim_wingspan_m().unwrap_or(15.0);
        let envelope = pkg.airspeed_envelope().cloned().map(|env| (env, mass_kg));
        let cruise_kt = envelope
            .as_ref()
            .map(|(env, m)| f64::from(env.resolve_at_mass(*m).vno_kt) * 0.78)
            .filter(|kt| kt.is_finite() && *kt > 25.0)
            .unwrap_or(140.0);
        let cruise_mps = (cruise_kt * KNOTS_TO_MPS).clamp(35.0, 240.0);
        let structural_datum = pkg
            .structural_landmarks()
            .zip(default_asset)
            .and_then(|(landmarks, asset)| StructuralDatum::fit(landmarks, asset).ok());
        let pilot_eye_frd_m = compute_pilot_eye_frd(pkg, structural_datum.as_ref());
        let parked_gear_height_m = pkg.jsbsim_parked_gear_height_m().unwrap_or(2.2);
        let views = pkg.camera_view_set();
        let camera_suite = VehicleCameraSuite {
            chase_back_up_m: views.chase_offsets_m().unwrap_or(DEFAULT_CHASE_BACK_UP_M),
            cameras: views.virtual_cameras(pilot_eye_frd_m),
        };
        Self {
            slot,
            aircraft_id: aircraft_id.to_owned(),
            title: title.to_owned(),
            family_title,
            cruise_kt,
            cruise_mps,
            mass_kg,
            wingspan_m,
            structural_datum,
            pilot_eye_frd_m,
            parked_gear_height_m,
            camera_suite,
            envelope,
            pec: pkg.pitot_static_calibration().cloned(),
            instruments: pkg.instrument_pack().cloned(),
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn compute_pilot_eye_frd(pkg: &LoadedSyspack, datum: Option<&StructuralDatum>) -> DVec3 {
    if let (Some([eye_x_in, eye_y_in, eye_z_in]), Some(datum)) =
        (pkg.jsbsim_eyepoint_structural_in(), datum)
    {
        let orig = datum.origin_in();
        DVec3::new(
            (orig.x - eye_x_in) * METRES_PER_INCH,
            (eye_y_in - orig.y) * METRES_PER_INCH,
            (orig.z - eye_z_in) * METRES_PER_INCH,
        )
    } else {
        DVec3::new(2.0, 0.0, -0.8)
    }
}

//! Outside-scene camera view alignment, dynamic `DesignFrame` HUD re-emission, and surveyed
//! airport runway GeoJSON queries for [`super::FlightDeckSession`].

use super::FlightDeckSession;
use crate::error::FlightDeckError;
use crate::instruments::CameraViewPose;
use crate::terrain::runways::{
    RunwayEyeAnchor, build_airport_runways_geojson, extract_route_airport_codes,
    lookup_airport_json,
};

impl FlightDeckSession {
    /// Sets or clears the pilot's active EFB/GCS flight-plan route string (`"KEWR → SBJ → N51"`),
    /// prioritizing its destination airport for destination airport runway highlighting and HUD guidance.
    pub fn set_flight_plan_route(&mut self, route_label: Option<&str>) {
        self.flight_plan_label = route_label
            .map(str::trim)
            .filter(|s| !s.is_empty())
            .map(str::to_owned);
    }

    /// Resolves an airport identifier (`"KHSV"`, `"KMEM"`, `"N51"`, `"KEWR"`) on `self.planet`
    /// from `AirportDatabase::builtin()`, returning UTF-8 JSON (`"{}"` if not found or non-Earth).
    #[must_use]
    pub fn lookup_airport(&self, code: &str) -> String {
        lookup_airport_json(&self.planet, code)
    }

    pub(super) fn effective_route_label(&self) -> Option<String> {
        if let Some(player) = &self.kml_player {
            let kml_lbl = player.route_label();
            if !kml_lbl.is_empty() {
                return Some(kml_lbl.to_owned());
            }
        }
        self.flight_plan_label.clone()
    }
    /// Updates the outside-scene 3D camera pose (`Some(...)` in `FPV` or `Chase`, `None` in `Globe`
    /// orbit) and optional onboard terrain DEM sample (`m` above datum) beneath the camera/ownship.
    pub fn set_camera_view_pose(
        &mut self,
        camera_view: Option<CameraViewPose>,
        onboard_dem_datum_m: Option<f64>,
    ) {
        self.camera_view = camera_view;
        self.onboard_dem_datum_m = onboard_dem_datum_m.filter(|v| v.is_finite());
    }

    /// Re-emits the conformal HUD scene into the exact `(width, height)` `DesignFrame` requested
    /// by `IndicateAppleDisplay`'s `PanelDisplay` so `Layout.centre` maps 1:1 to screen center.
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::Instrument`] if `Indicate` scene encoding fails.
    pub fn emit_hud_scene(&mut self, width: f32, height: f32) -> Result<Vec<u8>, FlightDeckError> {
        self.instruments.emit_hud_for_frame(width, height)
    }

    /// Generates a GeoJSON `FeatureCollection` of surveyed airport runway polygons, centerlines, approach
    /// lead-in feathers, and threshold designators from `sim_scenery::AirportDatabase::builtin()`
    /// on `WGS-84` Earth (empty `FeatureCollection` on non-Earth bodies).
    #[must_use]
    pub fn airport_runways_geojson(
        &self,
        center_lat_deg: f64,
        center_lon_deg: f64,
        radius_km: f64,
    ) -> String {
        let route_label = self.effective_route_label();
        let route_codes = extract_route_airport_codes(&self.ownship, route_label.as_deref());
        build_airport_runways_geojson(
            &self.planet,
            center_lat_deg,
            center_lon_deg,
            radius_km,
            &route_codes,
        )
    }

    pub(super) fn active_eye_override(&self) -> Option<RunwayEyeAnchor> {
        if let Some(cam) = self.camera_view {
            Some(RunwayEyeAnchor {
                latitude_deg: cam.latitude_deg,
                longitude_deg: cam.longitude_deg,
                altitude_datum_m: cam.altitude_datum_m,
                onboard_dem_datum_m: self.onboard_dem_datum_m,
            })
        } else if let Some(dem_m) = self.onboard_dem_datum_m
            && let (Some(lat), Some(lon)) = (self.ownship.latitude_deg, self.ownship.longitude_deg)
        {
            Some(RunwayEyeAnchor {
                latitude_deg: lat,
                longitude_deg: lon,
                altitude_datum_m: self.ownship.best_altitude_m().unwrap_or(dem_m + 2.5),
                onboard_dem_datum_m: Some(dem_m),
            })
        } else {
            None
        }
    }
}

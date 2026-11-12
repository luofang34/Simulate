//! Chart cameras and symbology for the 2D instrument maps.

use std::hash::{Hash, Hasher};
use std::time::Duration;

use glam::DVec3;
use sim_core::VehicleState;
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_maplibre::{
    ChartCameraSpec, ChartOwnship, ChartSymbologyInput, DynamicEntity3D, InstrumentMapSurface,
    build_chart_symbology_scaled,
};
use sim_scenario::{OperatorViewportMode, ScenarioProfile};
use sim_scenery::{AerodromeId, LandingRunwayGeometry};

use super::{AIRPORT_SURFACE_MAP, MapFrameInput, MapLibreRig};
use crate::error::AppSimError;
use crate::supervisor_rig::SupervisorRig;
use crate::vehicles::SteeredFix;

impl MapLibreRig {
    /// Samples orthometric elevation (`metres above MSL`) from the onboard `PFD_SVS_MAP`
    /// terrain cache (`None` when the tile has not arrived yet or SVS is inactive).
    #[cfg(test)]
    #[must_use]
    pub fn sample_svs_dem_msl(&self, lat_deg: f64, lon_deg: f64) -> Option<f64> {
        self.avionics
            .get(super::PFD_SVS_MAP)?
            .sample_resident_dem_msl(lat_deg, lon_deg)
    }

    /// The active landing runway laid on the runway datum every map lays into the terrain
    /// it draws ([`sim_scenery::RunwayDatum::touchdown_plane`]), so the conformal runway
    /// sits on the drawn one.
    #[must_use]
    pub(super) fn active_landing_runway(
        &self,
        input: &MapFrameInput<'_>,
    ) -> Option<LandingRunwayGeometry> {
        let datum = input.supervisor.vertical_datum();
        input
            .supervisor
            .active_landing_runway(input.profile)
            .map(|rwy| self.surface.runway_datum().touchdown_plane(&rwy, datum))
    }
}

/// Moving-map zoom of the dock chart.
const MOVING_MAP_ZOOM: f64 = 13.6;

/// True heading of `vehicle`'s nose, degrees.
pub fn heading_deg(vehicle: &VehicleState) -> f64 {
    let local = LocalFrame::at(&ecef_to_geodetic(vehicle.position_ecef));
    let nose = vehicle.attitude * DVec3::X;
    nose.dot(local.east())
        .atan2(nose.dot(local.north()))
        .to_degrees()
        .rem_euclid(360.0)
}

/// Ownship as charts show it.
pub fn chart_ownship(vehicle: &VehicleState) -> ChartOwnship {
    let geo = ecef_to_geodetic(vehicle.position_ecef);
    let local = LocalFrame::at(&geo);
    ChartOwnship {
        lat_deg: geo.lat_rad.to_degrees(),
        lon_deg: geo.lon_rad.to_degrees(),
        true_heading_deg: heading_deg(vehicle),
        ground_velocity_en_mps: [
            vehicle.velocity_ecef.dot(local.east()),
            vehicle.velocity_ecef.dot(local.north()),
        ],
    }
}

/// Ownship as the moving maps place it: where the `steered` navigator does when the
/// mission flies on one, with the heading of the instruments' own `estimated` solution,
/// else that solution's position too.
pub fn map_ownship(estimated: &VehicleState, steered: Option<SteeredFix>) -> ChartOwnship {
    let own = chart_ownship(estimated);
    steered.map_or(own, |fix| ChartOwnship {
        lat_deg: fix.lat_deg,
        lon_deg: fix.lon_deg,
        ground_velocity_en_mps: fix.ground_velocity_en_m_s,
        ..own
    })
}

/// Revision of the symbology: changes when ownship moves a few metres or turns half a degree.
pub fn symbology_revision(own: &ChartOwnship, surveillance_revision: u64) -> u64 {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    ((own.lat_deg * 2e4).round() as i64).hash(&mut hasher);
    ((own.lon_deg * 2e4).round() as i64).hash(&mut hasher);
    ((own.true_heading_deg * 2.0).round() as i64).hash(&mut hasher);
    surveillance_revision.hash(&mut hasher);
    hasher.finish()
}

/// Track-up moving map centred on ownship.
pub fn moving_map_camera(own: &ChartOwnship, supervisor: &SupervisorRig) -> ChartCameraSpec {
    let surveillance_revision = supervisor.belief_surveillance_revision();
    ChartCameraSpec {
        center_lat_deg: own.lat_deg,
        center_lon_deg: own.lon_deg,
        zoom: MOVING_MAP_ZOOM,
        bearing_deg: own.true_heading_deg,
        pitch_deg: 0.0,
        surveillance_revision,
        notices_revision: 0,
        symbology_revision: symbology_revision(own, surveillance_revision),
    }
}

/// North-up airport-surface diagram of the active aerodrome, or the moving map without one.
pub fn airport_surface_camera(
    own: &ChartOwnship,
    profile: Option<ScenarioProfile>,
    supervisor: &SupervisorRig,
) -> ChartCameraSpec {
    let (center_lat_deg, center_lon_deg, zoom) =
        if let Some(record) = supervisor.active_airport_record(profile) {
            if let Some(aero_id) = record.osm_aerodrome()
                && let Some(diagram) = supervisor.diagram(aero_id)
            {
                let zoom = match aero_id {
                    AerodromeId::CentralJersey47N => 15.1,
                    AerodromeId::NewarkKewr | AerodromeId::DoverKdov => 13.8,
                };
                (
                    diagram.surface_graph.reference_point.latitude_deg,
                    diagram.surface_graph.reference_point.longitude_deg,
                    zoom,
                )
            } else {
                (record.lat_deg, record.lon_deg, 13.8)
            }
        } else {
            let aero_id = supervisor.effective_aerodrome(profile);
            let Some(diagram) = supervisor.diagram(aero_id) else {
                return moving_map_camera(own, supervisor);
            };
            let zoom = match aero_id {
                AerodromeId::CentralJersey47N => 15.1,
                AerodromeId::NewarkKewr | AerodromeId::DoverKdov => 13.8,
            };
            (
                diagram.surface_graph.reference_point.latitude_deg,
                diagram.surface_graph.reference_point.longitude_deg,
                zoom,
            )
        };
    let surveillance_revision = supervisor.belief_surveillance_revision();
    ChartCameraSpec {
        center_lat_deg,
        center_lon_deg,
        zoom,
        bearing_deg: 0.0,
        pitch_deg: 0.0,
        surveillance_revision,
        notices_revision: 0,
        symbology_revision: symbology_revision(own, surveillance_revision),
    }
}

/// Renders one 2D moving-map or airport-surface chart surface when due and visible.
pub(super) fn render_chart_surface(
    surface: &mut InstrumentMapSurface,
    input: &MapFrameInput<'_>,
    sim_dur: Duration,
    own: ChartOwnship,
    landing_runway: Option<&LandingRunwayGeometry>,
    belief_entities: &[DynamicEntity3D],
) -> Result<(), AppSimError> {
    let full_window = surface.id() == AIRPORT_SURFACE_MAP;
    if full_window && input.supervisor.viewport_mode() != OperatorViewportMode::ChartMap {
        return Ok(());
    }
    let camera = if full_window {
        airport_surface_camera(&own, input.profile, input.supervisor)
    } else {
        moving_map_camera(&own, input.supervisor)
    };
    let symbology = build_chart_symbology_scaled(
        &ChartSymbologyInput {
            camera,
            viewport_px: surface.size(),
            ownship: Some(own),
            runway: landing_runway,
            traffic: belief_entities,
        },
        surface.pixel_ratio(),
    );
    let _outcome = surface.render_chart_if_dirty(sim_dur, camera, Some(symbology))?;
    Ok(())
}

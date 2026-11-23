//! 3D airspace volume extrusion (`Flightbag` + `world-notices`), TFR/SUA live status,
//! and ownship containment/proximity alerts in local East-North-Up coordinates.

pub mod catalog;
mod extrude;
pub mod pmtiles;

use crate::airspace::catalog::{builtin_ny_nj_airspaces, builtin_ny_nj_notices};
use crate::airspace::extrude::{
    VolumeExtrudeSpec, extrude_polygons_enu, extrude_tfr_enu, format_static_label, min_rim_horiz_m,
    resolve_limit_msl_m,
};
use crate::error::FlightDeckError;
use crate::planet::PlanetaryReference;
use crate::telemetry::OwnshipState;
use flightbag_core::{
    AirspaceClassification, AirspaceVolume, AltitudeDatum, ControlledAirspaceClass,
};
use world_core::{DecimalYear, GeodeticPosition, LocalTangentPlane};
use world_notices::{AirspaceKey, NoticesStore};

const DEFAULT_CULL_RADIUS_M: f64 = 46_300.0; // 25 NM

/// Visual portrayal class for 3D airspace curtains and rims.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AirspaceClassStyle {
    /// Class B terminal radar airspace (cyan-blue).
    ClassB = 0,
    /// Class C terminal radar airspace (magenta-violet).
    ClassC = 1,
    /// Class D airport tower cylinder (sky-blue).
    ClassD = 2,
    /// Class E controlled transition/surface airspace (soft magenta).
    ClassE = 3,
    /// Special Use Airspace currently active / hot via `NoticesStore` (amber-orange).
    SuaActive = 4,
    /// Special Use Airspace currently cold / inactive (muted slate-amber).
    SuaInactive = 5,
    /// Active Temporary Flight Restriction (`world-notices` `TfrNotice`, crimson-red).
    TemporaryFlightRestriction = 6,
}

impl AirspaceClassStyle {
    /// Canonical RGB tint (`0.0..=1.0`) for XR curtain walls and rim outlines.
    #[must_use]
    pub const fn rgb(self) -> [f32; 3] {
        match self {
            Self::ClassB => [0.18, 0.58, 0.98],
            Self::ClassC => [0.82, 0.24, 0.68],
            Self::ClassD => [0.28, 0.74, 0.96],
            Self::ClassE => [0.74, 0.38, 0.72],
            Self::SuaActive => [0.98, 0.60, 0.14],
            Self::SuaInactive => [0.62, 0.58, 0.44],
            Self::TemporaryFlightRestriction => [0.98, 0.20, 0.22],
        }
    }
}

/// Severity of ownship's relationship to surrounding controlled/restricted airspace.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum AirspaceAlertLevel {
    /// Outside controlled/restricted volumes with comfortable margin.
    #[default]
    Clear = 0,
    /// Within `1.5 NM` horizontally and `500 ft` vertically of Class B/C/D, active SUA, or TFR.
    Proximity = 1,
    /// Inside controlled Class B/C/D/E airspace.
    InsideControlled = 2,
    /// Penetrating an active Restricted/Prohibited SUA or Temporary Flight Restriction.
    RestrictedOrTfrPenetration = 3,
}

/// Summary banner describing ownship's highest-priority airspace state.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct AirspaceAlert {
    /// Alert severity tier.
    pub level: AirspaceAlertLevel,
    /// Short cockpit banner message (empty when [`AirspaceAlertLevel::Clear`]).
    pub message: String,
}

/// Extruded 3D airspace volume in ownship-centered East-North-Up metres ready for `RealityKit`.
#[derive(Clone, Debug, PartialEq)]
pub struct AirspaceRenderVolume {
    /// Stable airspace or NOTAM identifier.
    pub airspace_key: String,
    /// Human-readable chart label (`"NEWARK CLASS B 15-70 • 1500–7000 MSL"`).
    pub label: String,
    /// Portrayal style and color family.
    pub style: AirspaceClassStyle,
    /// Base RGB color (`0.0..=1.0`).
    pub rgb: [f32; 3],
    /// Whether the volume is currently active (`true` for controlled/TFR/hot SUA).
    pub active: bool,
    /// Whether ownship is currently inside this volume laterally and vertically.
    pub ownship_inside: bool,
    /// Triangle mesh vertices (`[east_m, north_m, up_m]`) for vertical boundary curtains.
    pub wall_positions_enu_m: Vec<[f32; 3]>,
    /// Triangle indices (`3 * tri_count`) with double-sided winding for XR interior/exterior views.
    pub wall_indices: Vec<u32>,
    /// 3D floor and ceiling perimeter line segments (`[[e0, n0, u0], [e1, n1, u1]]`).
    pub rim_segments_enu_m: Vec<[[f32; 3]; 2]>,
    /// 3D anchor (`[east_m, north_m, up_m]`) on the nearest boundary edge for the floating label.
    pub label_anchor_enu_m: [f32; 3],
}

/// Coordinator combining static `Flightbag` [`AirspaceVolume`]s with live [`NoticesStore`] TFRs/SUAs.
#[derive(Clone, Debug)]
pub struct AirspaceCoordinator {
    volumes: Vec<AirspaceVolume>,
    notices: NoticesStore,
    cull_radius_m: f64,
}

impl AirspaceCoordinator {
    /// Creates an [`AirspaceCoordinator`] pre-populated with the NY/NJ corridor airspaces and notices.
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::Airspace`] if built-in validation fails.
    pub fn builtin_ny_nj() -> Result<Self, FlightDeckError> {
        Ok(Self {
            volumes: builtin_ny_nj_airspaces()?,
            notices: builtin_ny_nj_notices()?,
            cull_radius_m: DEFAULT_CULL_RADIUS_M,
        })
    }

    /// Appends validated [`AirspaceVolume`]s (for example decoded from an MVT / `PMTiles` tile).
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::Airspace`] if any volume fails validation.
    pub fn extend_volumes(&mut self, extra: Vec<AirspaceVolume>) -> Result<(), FlightDeckError> {
        for vol in extra {
            vol.validate().map_err(|e| FlightDeckError::Airspace {
                reason: format!("invalid airspace {}: {e:?}", vol.airspace_key),
            })?;
            if !self
                .volumes
                .iter()
                .any(|existing| existing.airspace_key == vol.airspace_key)
            {
                self.volumes.push(vol);
            }
        }
        Ok(())
    }

    /// Mutable access to [`NoticesStore`] for live TFR or SUA updates.
    pub fn notices_mut(&mut self) -> &mut NoticesStore {
        &mut self.notices
    }

    /// Shared access to static [`AirspaceVolume`]s.
    #[must_use]
    pub fn volumes(&self) -> &[AirspaceVolume] {
        &self.volumes
    }

    /// Extrudes nearby static airspaces and active TFRs into ownship-centered ENU 3D meshes,
    /// returning `(render_volumes, highest_alert)` when `planet` is `WGS-84` Earth.
    #[must_use]
    pub fn snapshot(
        &self,
        ownship: &OwnshipState,
        planet: &PlanetaryReference,
        ground_elev_msl_m: f64,
        epoch: DecimalYear,
    ) -> (Vec<AirspaceRenderVolume>, AirspaceAlert) {
        let Some(own_geo) = ownship.valid_earth_wgs84_position(planet) else {
            return (Vec::new(), AirspaceAlert::default());
        };
        let own_alt_msl_m = ownship
            .best_altitude_m()
            .unwrap_or(own_geo.ellipsoidal_height_m);
        let flat_origin = GeodeticPosition::wgs84(own_geo.latitude_deg, own_geo.longitude_deg, 0.0);
        let Ok(plane) = LocalTangentPlane::new(flat_origin) else {
            return (Vec::new(), AirspaceAlert::default());
        };

        let mut out = Vec::new();
        let mut alert = AirspaceAlert::default();

        for vol in &self.volumes {
            let (style, active) = self.classify_static_volume(vol, own_alt_msl_m, epoch);
            let floor_msl_m = resolve_limit_msl_m(vol.lower, ground_elev_msl_m);
            let ceil_msl_m = resolve_limit_msl_m(vol.upper, ground_elev_msl_m);
            let label = format_static_label(vol);
            let is_sfc_floor = vol.lower.value_ft == 0
                && matches!(vol.lower.datum, AltitudeDatum::Surface | AltitudeDatum::Agl);
            let Some(rendered) = extrude_polygons_enu(
                &VolumeExtrudeSpec {
                    airspace_key: &vol.airspace_key,
                    label: &label,
                    sector_tag: vol.sector.as_deref(),
                    style,
                    active,
                    is_sfc_floor,
                    has_floating_floor: vol.lower.value_ft > 0,
                    polygons: &vol.polygons,
                    floor_msl_m,
                    ceil_msl_m,
                    own_alt_msl_m,
                },
                &plane,
                self.cull_radius_m,
            ) else {
                continue;
            };
            update_static_alert(
                &mut alert,
                vol,
                &rendered,
                floor_msl_m,
                ceil_msl_m,
                own_alt_msl_m,
            );
            out.push(rendered);
        }

        self.append_active_tfrs(
            own_geo,
            flat_origin,
            own_alt_msl_m,
            ground_elev_msl_m,
            epoch,
            &plane,
            &mut out,
            &mut alert,
        );

        (out, alert)
    }

    #[allow(clippy::too_many_arguments)]
    fn append_active_tfrs(
        &self,
        own_geo: GeodeticPosition,
        flat_origin: GeodeticPosition,
        own_alt_msl_m: f64,
        ground_elev_msl_m: f64,
        epoch: DecimalYear,
        plane: &LocalTangentPlane,
        out: &mut Vec<AirspaceRenderVolume>,
        alert: &mut AirspaceAlert,
    ) {
        for tfr in self
            .notices
            .tfrs_near(flat_origin, self.cull_radius_m, epoch)
        {
            let Some(rendered) = extrude_tfr_enu(
                tfr,
                own_geo,
                own_alt_msl_m,
                ground_elev_msl_m,
                epoch,
                plane,
                self.cull_radius_m,
            ) else {
                continue;
            };
            if rendered.ownship_inside {
                promote_alert(
                    alert,
                    AirspaceAlertLevel::RestrictedOrTfrPenetration,
                    format!("TFR PENETRATION: {}", tfr.id),
                );
            } else if min_rim_horiz_m(&rendered) <= 2_778.0 {
                promote_alert(
                    alert,
                    AirspaceAlertLevel::Proximity,
                    format!("TFR AHEAD: {}", tfr.id),
                );
            }
            out.push(rendered);
        }
    }

    fn classify_static_volume(
        &self,
        vol: &AirspaceVolume,
        own_alt_msl_m: f64,
        epoch: DecimalYear,
    ) -> (AirspaceClassStyle, bool) {
        match vol.classification {
            AirspaceClassification::Controlled { class, .. } => {
                let style = match class {
                    ControlledAirspaceClass::ClassB => AirspaceClassStyle::ClassB,
                    ControlledAirspaceClass::ClassC => AirspaceClassStyle::ClassC,
                    ControlledAirspaceClass::ClassD => AirspaceClassStyle::ClassD,
                    ControlledAirspaceClass::ClassE => AirspaceClassStyle::ClassE,
                };
                (style, true)
            }
            AirspaceClassification::SpecialUse(_) => {
                let hot = AirspaceKey::parse(&vol.ident)
                    .or_else(|_| AirspaceKey::parse(&vol.airspace_key))
                    .is_ok_and(|key| {
                        self.notices
                            .is_airspace_active(key, Some(own_alt_msl_m), epoch)
                            || self.notices.is_airspace_active(key, None, epoch)
                    });
                if hot {
                    (AirspaceClassStyle::SuaActive, true)
                } else {
                    (AirspaceClassStyle::SuaInactive, false)
                }
            }
        }
    }
}

fn update_static_alert(
    alert: &mut AirspaceAlert,
    vol: &AirspaceVolume,
    rendered: &AirspaceRenderVolume,
    floor_msl_m: f64,
    ceil_msl_m: f64,
    own_alt_msl_m: f64,
) {
    if rendered.ownship_inside {
        if rendered.style == AirspaceClassStyle::SuaActive {
            promote_alert(
                alert,
                AirspaceAlertLevel::RestrictedOrTfrPenetration,
                format!("ACTIVE SUA PENETRATION: {}", vol.name),
            );
        } else if rendered.active {
            promote_alert(
                alert,
                AirspaceAlertLevel::InsideControlled,
                format!("INSIDE {} ({})", vol.name, vol.classification.class_code()),
            );
        }
        return;
    }
    let vert_near = own_alt_msl_m >= floor_msl_m - 152.4 && own_alt_msl_m <= ceil_msl_m + 152.4;
    if rendered.active
        && vert_near
        && rendered.style != AirspaceClassStyle::ClassE
        && min_rim_horiz_m(rendered) <= 2_778.0
    {
        promote_alert(
            alert,
            AirspaceAlertLevel::Proximity,
            format!("AIRSPACE NEARBY: {}", vol.name),
        );
    }
}

fn promote_alert(alert: &mut AirspaceAlert, level: AirspaceAlertLevel, message: String) {
    if level > alert.level {
        alert.level = level;
        alert.message = message;
    }
}

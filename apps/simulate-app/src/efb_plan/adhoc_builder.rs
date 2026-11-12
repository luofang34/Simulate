//! Two-point ad-hoc runway delineation from either the 2D/globe EFB map or 3D terrain
//! raycast clicks, with live dimensions, magnetic designator preview, and instant world
//! terrain stamping.

use sim_geo::Geodetic;
use sim_scenery::{
    AdHocAerodrome, AdHocRunwayError, AdHocRunwaySpec, RunwaySurfaceKind,
    approx_magnetic_variation_deg, magnetic_heading_deg,
};

use super::navlog::distance_and_bearing_nm;
use crate::esc_menu::catalog::{HubCatalog, aviation_heading_deg};
use crate::world_surface::WorldSurfaceHandle;

const METRES_PER_NM: f64 = 1_852.0;

/// Active interactive threshold-picking mode for delineating a new ad-hoc runway.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AdHocPickMode {
    /// No active threshold pick.
    #[default]
    None,
    /// Waiting for Threshold A click on the Expanded EFB map.
    EfbMapThresholdA,
    /// Waiting for Threshold B click on the Expanded EFB map.
    EfbMapThresholdB,
    /// Waiting for Threshold A click directly on the 3D terrain/bridge world view.
    World3dThresholdA,
    /// Waiting for Threshold B click directly on the 3D terrain/bridge world view.
    World3dThresholdB,
}

impl AdHocPickMode {
    /// Whether the operator is picking on the 3D world viewport (while the EFB tablet is
    /// minimized out of the way).
    #[must_use]
    pub const fn is_world_3d(self) -> bool {
        matches!(self, Self::World3dThresholdA | Self::World3dThresholdB)
    }

    /// Whether the operator is picking on the Expanded EFB moving map.
    #[must_use]
    pub const fn is_efb_map(self) -> bool {
        matches!(self, Self::EfbMapThresholdA | Self::EfbMapThresholdB)
    }

    /// Short HUD/EFB prompt banner describing what click is expected next.
    #[must_use]
    pub const fn banner_prompt(self) -> Option<&'static str> {
        match self {
            Self::None => None,
            Self::EfbMapThresholdA => Some("EFB MAP PICK: CLICK RUNWAY THRESHOLD A"),
            Self::EfbMapThresholdB => Some("EFB MAP PICK: CLICK RECIPROCAL THRESHOLD B"),
            Self::World3dThresholdA => {
                Some("3D RUNWAY PICK: CLICK THRESHOLD A ON TERRAIN OR BRIDGE (ESC CANCEL)")
            }
            Self::World3dThresholdB => {
                Some("3D RUNWAY PICK: CLICK THRESHOLD B TO COMPLETE STRIP (ESC CANCEL)")
            }
        }
    }
}

/// Live geometric preview of the draft two-point ad-hoc runway before committing.
#[derive(Debug, Clone, PartialEq)]
pub struct AdHocDraftPreview {
    /// Proposed facility code (`ZZ01`, `ZZ02`, ...).
    pub code: String,
    /// Chord length between Threshold A and Threshold B, metres.
    pub length_m: f64,
    /// Configured strip width, metres.
    pub width_m: f32,
    /// True bearing A -> B, whole aviation degrees (`001..=360`).
    pub true_bearing_deg: u16,
    /// Magnetic bearing A -> B, whole aviation degrees (`001..=360`).
    pub mag_bearing_deg: u16,
    /// Computed bidirectional designator pair (`15/33`, `09/27`).
    pub designator_pair: String,
    /// Sampled or overridden field elevation, metres MSL.
    pub elevation_msl_m: f64,
    /// Short surface label (`ASPH`, `TURF`, `GRVL`).
    pub surface_label: &'static str,
}

/// Interactive state of the EFB `AD-HOC RWY` delineation tool.
#[derive(Debug, Clone, PartialEq)]
pub struct AdHocBuilder {
    /// Current interactive cursor-pick state.
    pub pick_mode: AdHocPickMode,
    /// Threshold A `[lat_deg, lon_deg]`.
    pub threshold_a_deg: Option<[f64; 2]>,
    /// Threshold B `[lat_deg, lon_deg]`.
    pub threshold_b_deg: Option<[f64; 2]>,
    /// Strip width in metres (`18`, `24`, `30`, `45`).
    pub width_m: f32,
    /// Landing surface material.
    pub surface: RunwaySurfaceKind,
    /// Optional fixed MSL elevation override (metres) for elevated bridge decks.
    pub override_elevation_msl_m: Option<f32>,
    /// Monotonic sequence counter for `ZZ01..ZZ99` codes.
    pub next_seq: u32,
    /// Code of the most recently committed or preset-selected ad-hoc aerodrome (`ZMRS`, `ZZ01`).
    pub active_code: String,
    /// Latest operator status line.
    pub status: String,
}

impl Default for AdHocBuilder {
    fn default() -> Self {
        // Default draft pre-populated with Bolshoy Moskvoretsky Bridge so the pilot can
        // inspect, tweak, or immediately delineate custom strips anywhere else.
        Self {
            pick_mode: AdHocPickMode::None,
            threshold_a_deg: Some([55.751_35, 37.623_62]),
            threshold_b_deg: Some([55.747_22, 37.627_68]),
            width_m: 24.0,
            surface: RunwaySurfaceKind::Asphalt,
            override_elevation_msl_m: None,
            next_seq: 1,
            active_code: "ZMRS".to_owned(),
            status: "ZMRS RED SQUARE BRIDGE READY (525X24M RWY 15/33)".to_owned(),
        }
    }
}

impl AdHocBuilder {
    /// Starts two-click threshold picking on the Expanded EFB map.
    pub fn start_efb_pick(&mut self) {
        self.pick_mode = AdHocPickMode::EfbMapThresholdA;
        self.override_elevation_msl_m = None;
        self.status = "CLICK THRESHOLD A ON THE EFB MAP".to_owned();
    }

    /// Starts two-click threshold picking directly on the 3D terrain/world view.
    pub fn start_world_pick(&mut self) {
        self.pick_mode = AdHocPickMode::World3dThresholdA;
        self.override_elevation_msl_m = None;
        self.status = "CLICK THRESHOLD A IN THE 3D VIEWPORT".to_owned();
    }

    /// Cancels any in-progress cursor pick.
    pub fn cancel_pick(&mut self) {
        self.pick_mode = AdHocPickMode::None;
    }

    /// Records a picked `[lat_deg, lon_deg]` threshold; returns `true` when Threshold B was
    /// just captured so the caller can restore the EFB view if picking from 3D world.
    pub fn record_point(&mut self, lat_deg: f64, lon_deg: f64) -> bool {
        match self.pick_mode {
            AdHocPickMode::EfbMapThresholdA => {
                self.threshold_a_deg = Some([lat_deg, lon_deg]);
                self.threshold_b_deg = None;
                self.pick_mode = AdHocPickMode::EfbMapThresholdB;
                self.status = format!("THR A SET ({lat_deg:.4}, {lon_deg:.4}) - CLICK THR B");
                false
            }
            AdHocPickMode::World3dThresholdA => {
                self.threshold_a_deg = Some([lat_deg, lon_deg]);
                self.threshold_b_deg = None;
                self.pick_mode = AdHocPickMode::World3dThresholdB;
                self.status = format!("3D THR A ({lat_deg:.4}, {lon_deg:.4}) - CLICK THR B");
                false
            }
            AdHocPickMode::EfbMapThresholdB | AdHocPickMode::World3dThresholdB => {
                self.threshold_b_deg = Some([lat_deg, lon_deg]);
                self.pick_mode = AdHocPickMode::None;
                self.status = format!("THR B SET ({lat_deg:.4}, {lon_deg:.4}) - READY TO CREATE");
                true
            }
            AdHocPickMode::None => false,
        }
    }

    /// Cycles strip width across `18 m -> 24 m -> 30 m -> 45 m`.
    pub fn cycle_width(&mut self) {
        self.width_m = if self.width_m < 20.0 {
            24.0
        } else if self.width_m < 27.0 {
            30.0
        } else if self.width_m < 38.0 {
            45.0
        } else {
            18.0
        };
    }

    /// Cycles strip surface across `Asphalt -> Turf -> Gravel`.
    pub fn cycle_surface(&mut self) {
        self.surface = match self.surface {
            RunwaySurfaceKind::Asphalt => RunwaySurfaceKind::Turf,
            RunwaySurfaceKind::Turf => RunwaySurfaceKind::Gravel,
            _ => RunwaySurfaceKind::Asphalt,
        };
    }

    /// Swaps Threshold A and Threshold B to flip primary landing direction.
    pub fn swap_thresholds(&mut self) {
        std::mem::swap(&mut self.threshold_a_deg, &mut self.threshold_b_deg);
        self.status = "SWAPPED THRESHOLD A <-> B".to_owned();
    }

    /// Short uppercase label of the selected [`RunwaySurfaceKind`].
    #[must_use]
    pub const fn surface_label(&self) -> &'static str {
        match self.surface {
            RunwaySurfaceKind::Turf | RunwaySurfaceKind::PavedTurf => "TURF",
            RunwaySurfaceKind::Gravel => "GRVL",
            _ => "ASPH",
        }
    }

    /// Computes live draft metrics when both thresholds are set.
    #[must_use]
    pub fn preview(&self, surface: &WorldSurfaceHandle) -> Option<AdHocDraftPreview> {
        let a = self.threshold_a_deg?;
        let b = self.threshold_b_deg?;
        let (nm, true_deg) = distance_and_bearing_nm(a, b);
        let length_m = nm * METRES_PER_NM;
        let mid_lat = 0.5 * (a.first().copied()? + b.first().copied()?);
        let mid_lon = 0.5 * (a.get(1).copied()? + b.get(1).copied()?);
        let mag_var = approx_magnetic_variation_deg(mid_lat, mid_lon);
        let mag_ab = f64::from(magnetic_heading_deg(true_deg as f32, mag_var));
        let mag_ba = (mag_ab + 180.0).rem_euclid(360.0);
        let num_a = ((mag_ab / 10.0).round() as u32).rem_euclid(36);
        let num_b = ((mag_ba / 10.0).round() as u32).rem_euclid(36);
        let da = if num_a == 0 { 36 } else { num_a };
        let db = if num_b == 0 { 36 } else { num_b };
        let elev = self
            .override_elevation_msl_m
            .map(f64::from)
            .or_else(|| {
                surface.orthometric_ground_m(&Geodetic::from_degrees(mid_lat, mid_lon, 0.0))
            })
            .unwrap_or(0.0);
        Some(AdHocDraftPreview {
            code: format!("ZZ{:02}", self.next_seq % 100),
            length_m,
            width_m: self.width_m,
            true_bearing_deg: aviation_heading_deg(true_deg),
            mag_bearing_deg: aviation_heading_deg(mag_ab),
            designator_pair: format!("{da:02}/{db:02}"),
            elevation_msl_m: elev,
            surface_label: self.surface_label(),
        })
    }

    /// Builds, registers, and stamps the delineated [`AdHocAerodrome`] into `catalog` and
    /// `surface`, advancing `next_seq` and returning `(catalog_row, aerodrome)`.
    pub fn commit(
        &mut self,
        catalog: &mut HubCatalog,
        surface: &WorldSurfaceHandle,
    ) -> Result<(usize, AdHocAerodrome), AdHocRunwayError> {
        let a = self
            .threshold_a_deg
            .ok_or(AdHocRunwayError::NonFiniteCoordinate)?;
        let b = self
            .threshold_b_deg
            .ok_or(AdHocRunwayError::NonFiniteCoordinate)?;
        let code = format!("ZZ{:02}", self.next_seq % 100);
        let mut spec = AdHocRunwaySpec::between(&code, format!("Ad-Hoc Strip {code}"), a, b);
        spec.width_m = self.width_m;
        spec.surface = self.surface;
        spec.override_elevation_msl_m = self.override_elevation_msl_m;
        let aerodrome = AdHocAerodrome::from_spec(&spec, |lat, lon| {
            surface.orthometric_ground_m(&Geodetic::from_degrees(lat, lon, 0.0))
        })?;
        let row = catalog.register_ad_hoc(aerodrome.clone());
        surface.set_ad_hoc_strips(catalog.ad_hoc_strips());
        self.next_seq = self.next_seq.wrapping_add(1);
        self.active_code.clone_from(&code);
        let rwy = aerodrome
            .airport
            .runways
            .first()
            .map_or("RWY", |r| r.id.as_str());
        self.status = format!(
            "CREATED {code} ({rwy}, {:.0}M {})",
            aerodrome.airport.longest_runway_m,
            self.surface_label()
        );
        Ok((row, aerodrome))
    }
}

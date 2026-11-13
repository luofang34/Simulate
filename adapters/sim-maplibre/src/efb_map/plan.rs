//! The plan on the EFB map: the flight plan and its progress, what a host draws over it,
//! the basemap under it, and where its fixes fall on each view, for labels and picking.

use serde_json::json;

use super::plan_style::{OUTLINES_SOURCE, ROUTE_SOURCE, SATELLITE_LAYER, TRACKS_SOURCE};
use super::{
    EfbDisplayMode, EfbFrameMarks, EfbMapSurface, EfbTracks, FixId, FlightPlan, RouteProgress,
};
use crate::{
    MapAdapterError,
    chart_symbology::{screen_positions, unproject_screen_position},
    runtime::sample_point_msl,
};

/// What lies under the plan.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EfbBasemap {
    /// The dark aeronautical chart.
    #[default]
    Chart,
    /// Satellite imagery.
    Satellite,
}

/// A fix under a point of a view.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FixPick {
    /// The fix.
    pub id: FixId,
    /// Its position in fly order.
    pub index: usize,
    /// How far the point is from it, physical pixels.
    pub distance_px: f32,
}

/// The plan as last given to the map.
#[derive(Clone, Debug, Default, PartialEq)]
pub(super) struct PlanState {
    pub(super) plan: FlightPlan,
    pub(super) progress: RouteProgress,
    pub(super) tracks: EfbTracks,
    pub(super) frame_marks: EfbFrameMarks,
    pub(super) basemap: EfbBasemap,
}

impl EfbMapSurface {
    /// The flight plan drawn.
    #[must_use]
    pub const fn flight_plan(&self) -> &FlightPlan {
        &self.plan.plan
    }

    /// The progress along it drawn.
    #[must_use]
    pub const fn route_progress(&self) -> RouteProgress {
        self.plan.progress
    }

    /// Draws `plan` at `progress`, the mission host's own report of its sequencing.
    pub fn set_route(
        &mut self,
        plan: &FlightPlan,
        progress: RouteProgress,
    ) -> Result<(), MapAdapterError> {
        if *plan == self.plan.plan && progress == self.plan.progress {
            return Ok(());
        }
        self.set_geojson_value(ROUTE_SOURCE, plan.geojson(progress))?;
        self.plan.plan = plan.clone();
        self.plan.progress = progress;
        Ok(())
    }

    /// Lays `marks` over the frames drawn from now on.
    pub const fn set_frame_marks(&mut self, marks: EfbFrameMarks) {
        self.plan.frame_marks = marks;
    }

    /// The marks laid over each frame.
    #[must_use]
    pub const fn frame_marks(&self) -> EfbFrameMarks {
        self.plan.frame_marks
    }

    /// What is drawn over the plan besides it.
    #[must_use]
    pub const fn tracks(&self) -> &EfbTracks {
        &self.plan.tracks
    }

    /// Draws `tracks` over the plan. Each change re-tessellates the tracks, so a host
    /// with growing tracks hands them over at a modest rate.
    pub fn set_tracks(&mut self, tracks: EfbTracks) -> Result<(), MapAdapterError> {
        if tracks == self.plan.tracks {
            return Ok(());
        }
        if tracks.lines != self.plan.tracks.lines || tracks.marks != self.plan.tracks.marks {
            self.set_geojson_value(TRACKS_SOURCE, tracks.geojson())?;
        }
        if tracks.outlines != self.plan.tracks.outlines {
            self.set_geojson_value(OUTLINES_SOURCE, tracks.outlines_geojson())?;
        }
        self.plan.tracks = tracks;
        Ok(())
    }

    /// The basemap under the plan.
    #[must_use]
    pub const fn basemap(&self) -> EfbBasemap {
        self.plan.basemap
    }

    /// Draws the plan over `basemap`.
    pub fn set_basemap(&mut self, basemap: EfbBasemap) -> Result<(), MapAdapterError> {
        let visibility = match basemap {
            EfbBasemap::Chart => "none",
            EfbBasemap::Satellite => "visible",
        };
        self.mutate(SATELLITE_LAYER, |style| {
            style.set_layout_property(SATELLITE_LAYER, "visibility", json!(visibility))
        })?;
        self.plan.basemap = basemap;
        Ok(())
    }

    /// Where each fix of the plan falls on `mode`'s view as it last drew, physical
    /// pixels from its top left, in fly order; `None` behind the globe.
    #[must_use]
    pub fn fix_positions(&self, mode: EfbDisplayMode) -> Vec<Option<[f32; 2]>> {
        let points: Vec<[f64; 2]> = self
            .plan
            .plan
            .fixes
            .iter()
            .map(|fix| [fix.lat_deg, fix.lon_deg])
            .collect();
        let Some(spec) = self.camera_spec(mode) else {
            return vec![None; points.len()];
        };
        let at = |lat: f64, lon: f64| sample_point_msl(&self.map, lat, lon);
        let ground = self.ground(&spec, &at);
        screen_positions(
            (&spec, self.size_of(mode), self.pixel_ratio()),
            ground,
            &points,
        )
    }

    /// Where the frame marks' true position falls on `mode`'s view as it last drew,
    /// physical pixels from its top left, for a host that labels the mark; `None`
    /// without the mark, and behind the globe.
    #[must_use]
    pub fn truth_mark_position(&self, mode: EfbDisplayMode) -> Option<[f32; 2]> {
        let truth_deg = self.plan.frame_marks.truth_deg?;
        let spec = self.camera_spec(mode)?;
        let at = |lat: f64, lon: f64| sample_point_msl(&self.map, lat, lon);
        let ground = self.ground(&spec, &at);
        screen_positions(
            (&spec, self.size_of(mode), self.pixel_ratio()),
            ground,
            &[truth_deg],
        )
        .into_iter()
        .next()
        .flatten()
    }

    /// Centres the focused view on `(lat_deg, lon_deg)` in [`super::EfbFollowMode::FreePan`],
    /// optionally setting its zoom within [`super::EFB_ZOOM_RANGE`].
    pub fn jump_to(&mut self, lat_deg: f64, lon_deg: f64, zoom: Option<f64>) {
        self.camera_mut().jump_to(lat_deg, lon_deg, zoom);
    }

    /// Sea-level `[lat_deg, lon_deg]` under physical pixel `at_px` on `mode`'s view;
    /// `None` beyond the globe limb or for non-finite coordinates.
    #[must_use]
    pub fn screen_to_geodetic(&self, mode: EfbDisplayMode, at_px: [f32; 2]) -> Option<[f64; 2]> {
        let spec = self.camera_spec(mode)?;
        unproject_screen_position((&spec, self.size_of(mode), self.pixel_ratio()), at_px)
    }

    /// The fix nearest `at_px` (physical pixels on `mode`'s view) within `radius_px`.
    #[must_use]
    pub fn pick_fix(
        &self,
        mode: EfbDisplayMode,
        at_px: [f32; 2],
        radius_px: f32,
    ) -> Option<FixPick> {
        let [x, y] = at_px;
        self.fix_positions(mode)
            .into_iter()
            .zip(&self.plan.plan.fixes)
            .enumerate()
            .filter_map(|(index, (position, fix))| {
                let [fx, fy] = position?;
                let distance_px = (fx - x).hypot(fy - y);
                (distance_px <= radius_px).then_some(FixPick {
                    id: fix.id,
                    index,
                    distance_px,
                })
            })
            .min_by(|a, b| a.distance_px.total_cmp(&b.distance_px))
    }

    fn set_geojson_value(
        &mut self,
        source_name: &str,
        document: serde_json::Value,
    ) -> Result<(), MapAdapterError> {
        self.map
            .set_geojson_source(source_name, document)
            .map_err(|source| MapAdapterError::GeoJsonSource {
                source_name: source_name.to_owned(),
                source,
            })
    }
}

//! The flight-guidance host seam.
//!
//! In real flight, autopilot modes, approach procedures, and the pilot's guidance selections
//! live in the `Pilotage` host; the avionics only display what it reports. When the app runs
//! standalone, [`StandInPilotageHost`] plays that host locally from the flight's own state:
//! the scripted pattern or corridor autopilot leg, the Aviate fly-by-wire mode, or manual
//! free flight, plus the receiver tuned to the active runway.

use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_indicate::{
    AutopilotModeSummary, HostFlightGuidance, HostNavGuidance, IdentStr, PilotSelections,
    RunwayNavGuidance,
};
use sim_scenario::PatternLeg;
use sim_scenery::LandingRunwayGeometry;

use crate::flight::Flight;
use crate::vehicles::FreeFlightVehicle;

const STANDARD_QNH_HPA: f64 = 1_013.25;
const PATTERN_HEIGHT_M: f64 = 304.8;
const HUNDRED_FT_M: f64 = 30.48;

/// Local stand-in for the `Pilotage` session host, used when no external host is connected.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct StandInPilotageHost {
    source_epoch: u32,
    sequence: u32,
    seen_discontinuity: Option<u32>,
}

impl StandInPilotageHost {
    /// A host that has published nothing yet.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The host's rewind/reset generation.
    #[must_use]
    pub const fn source_epoch(&self) -> u32 {
        self.source_epoch
    }

    /// Reports published in the current epoch.
    #[cfg(test)]
    #[must_use]
    pub const fn sequence(&self) -> u32 {
        self.sequence
    }

    /// Starts a new guidance epoch: the host's clock went back or its flight was replaced
    /// (rewind, restart, or a scenario, vehicle, or airport switch).
    pub fn bump_epoch(&mut self) {
        self.source_epoch = self.source_epoch.wrapping_add(1);
        self.sequence = 0;
    }

    /// The host's report for the flight's current instant. A change of the flight's timeline
    /// generation since the last report bumps the epoch first, so every rewind, restart, and
    /// switch is seen by the display whichever code path caused it. Like any onboard
    /// computer it reads the navigation solution, never simulation truth.
    pub fn publish(
        &mut self,
        flight: &Flight,
        runway: Option<&LandingRunwayGeometry>,
        heading_rad: f64,
    ) -> HostFlightGuidance {
        let discontinuity = flight.discontinuity();
        if self
            .seen_discontinuity
            .is_some_and(|seen| seen != discontinuity)
        {
            self.bump_epoch();
        }
        self.seen_discontinuity = Some(discontinuity);
        self.sequence = self.sequence.wrapping_add(1);
        let body = flight.world().body();
        let est_geo = body.fixed_to_geodetic(flight.estimated_vehicle_state().position_ecef);
        let undulation_m = if body == BodyEllipsoid::WGS84 {
            flight
                .world_surface()
                .geoid_undulation_m(est_geo.lat_rad, est_geo.lon_rad)
        } else {
            0.0
        };
        let est_datum_height_m = est_geo.height_m - undulation_m;
        HostFlightGuidance {
            selections: pilot_selections(&est_geo, est_datum_height_m, heading_rad, runway),
            nav: runway.map_or(HostNavGuidance::None, |rwy| {
                HostNavGuidance::RunwayReceiver(runway_nav(rwy))
            }),
            autopilot: autopilot_summary(flight),
            source_epoch: self.source_epoch,
        }
    }
}

/// Heading bug on the landing course (projected into ownship's local meridian across `±90°`)
/// and the altitude preselect at pattern altitude in the active vertical datum when a runway
/// is selected; otherwise on the current heading and datum altitude.
fn pilot_selections(
    est_geo: &Geodetic,
    est_datum_height_m: f64,
    heading_rad: f64,
    runway: Option<&LandingRunwayGeometry>,
) -> PilotSelections {
    let (heading_rad, altitude_m) = runway.map_or_else(
        || {
            (
                heading_rad,
                (est_datum_height_m / HUNDRED_FT_M).round() * HUNDRED_FT_M,
            )
        },
        |rwy| {
            (
                local_runway_course_rad(est_geo, rwy),
                rwy.threshold_elevation_msl_m + PATTERN_HEIGHT_M,
            )
        },
    );
    PilotSelections {
        heading_rad,
        altitude_m: Some(altitude_m),
        baro_setting_hpa: Some(STANDARD_QNH_HPA),
    }
}

/// Projects `rwy`'s 3D ECEF centreline into `est_geo`'s local tangent plane so polar
/// approaches across `±90°` report the runway course relative to ownship's meridian.
fn local_runway_course_rad(est_geo: &Geodetic, rwy: &LandingRunwayGeometry) -> f64 {
    let thr_frame = LocalFrame::at(&rwy.threshold);
    let (sin_c, cos_c) = rwy.true_bearing_rad.sin_cos();
    let dir_ecef = thr_frame.north() * cos_c + thr_frame.east() * sin_c;
    let own_frame = LocalFrame::at(est_geo);
    let north = dir_ecef.dot(own_frame.north());
    let east = dir_ecef.dot(own_frame.east());
    if north.hypot(east) > 1e-9 {
        east.atan2(north).rem_euclid(std::f64::consts::TAU)
    } else {
        rwy.true_bearing_rad.rem_euclid(std::f64::consts::TAU)
    }
}

/// Localizer and glide slope on `runway`'s surveyed landing threshold and glidepath.
fn runway_nav(runway: &LandingRunwayGeometry) -> RunwayNavGuidance {
    let ident = format!("RW{}", runway.designator);
    RunwayNavGuidance {
        threshold: runway.threshold,
        course_rad: runway.true_bearing_rad,
        length_m: runway.landing_length_m,
        glide_path_rad: runway.glidepath_rad,
        ident: IdentStr::new(&ident).unwrap_or_default(),
    }
}

/// The modes flying the aircraft: a scripted scenario's autopilot leg, the Aviate
/// fly-by-wire mode, or nothing engaged in manual free flight.
fn autopilot_summary(flight: &Flight) -> AutopilotModeSummary {
    if let Some(leg) = flight.pattern_leg() {
        return AutopilotModeSummary {
            lateral: ident(pattern_leg_label(leg)),
            vertical: ident(if leg == PatternLeg::Final {
                "GS"
            } else {
                "ALT"
            }),
            armed_approach: matches!(leg, PatternLeg::Base | PatternLeg::Final),
            engaged: true,
        };
    }
    if flight.profile().is_some() {
        return AutopilotModeSummary {
            lateral: ident("LNAV"),
            vertical: ident("VNAV"),
            armed_approach: false,
            engaged: true,
        };
    }
    match (flight.free_vehicle(), flight.aviate_armed()) {
        (Some(FreeFlightVehicle::AviateX500Direct), Some(engaged)) => AutopilotModeSummary {
            lateral: ident("POS HOLD"),
            vertical: ident("ALT HOLD"),
            armed_approach: false,
            engaged,
        },
        (Some(FreeFlightVehicle::X500Mission), Some(engaged)) => AutopilotModeSummary {
            lateral: ident("MSN"),
            vertical: ident("VEL"),
            armed_approach: false,
            engaged,
        },
        (Some(FreeFlightVehicle::AviateFixedWingDirect), Some(engaged)) => AutopilotModeSummary {
            lateral: ident("FBW ATT"),
            vertical: ident("FBW THR"),
            armed_approach: false,
            engaged,
        },
        _ => AutopilotModeSummary::default(),
    }
}

const fn pattern_leg_label(leg: PatternLeg) -> &'static str {
    match leg {
        PatternLeg::Upwind => "UPWIND",
        PatternLeg::Crosswind => "XWIND",
        PatternLeg::Downwind => "DNWIND",
        PatternLeg::Base => "BASE",
        PatternLeg::Final => "FINAL",
    }
}

fn ident(label: &str) -> Option<IdentStr> {
    IdentStr::new(label).ok()
}

//! Translation from [`OwnshipState`] to `Indicate` [`AircraftState`] on an explicit
//! [`PlanetaryReference`].

use crate::planet::PlanetaryReference;
use crate::telemetry::{FEET_TO_METRES, FPM_TO_MPS, KNOTS_TO_MPS, OwnshipState, TelemetryGroup};
use crate::terrain::runways::ApproachRunwaySelection;
use indicate_instrument_state::{
    AirData, AircraftState, AltitudeClass, AltitudeDeclaration, Attitude, EstimateQuality,
    GeoidModelId, HeadingReference, HeadingSample, IdentStr, Kinematics, MagneticVariation,
    NavData, NavFacility, NavFromTo, NavScale, NavSequencing, NavSource, OriginId,
    PositionIntegrity, PositionSource, Quat, Stamped, VariationSourceId,
};
use sim_geo::VerticalDatumKind;

/// Translates [`OwnshipState`] into `Indicate`'s [`AircraftState`] on `planet` while keeping
/// absent channels (`IAS`, `VSI` on Connext) unfed so they resolve honestly as `Missing`.
#[must_use]
pub fn build_aircraft_state(
    ownship: &OwnshipState,
    now_ms: u64,
    planet: &PlanetaryReference,
    prefer_true_heading: bool,
) -> AircraftState {
    build_aircraft_state_with_approach(ownship, now_ms, planet, prefer_true_heading, None)
}

/// Translates [`OwnshipState`] and an optional active [`ApproachRunwaySelection`] into
/// `Indicate`'s [`AircraftState`] on `planet`.
#[must_use]
pub fn build_aircraft_state_with_approach(
    ownship: &OwnshipState,
    now_ms: u64,
    planet: &PlanetaryReference,
    prefer_true_heading: bool,
    approach: Option<&ApproachRunwaySelection>,
) -> AircraftState {
    let mut state = AircraftState {
        quality: EstimateQuality::Good,
        ..AircraftState::default()
    };
    let nav_age_ms = ownship
        .clock
        .stalest_age_ms(
            &[
                TelemetryGroup::Position,
                TelemetryGroup::GroundVector,
                TelemetryGroup::PressureAltitude,
            ],
            now_ms,
        )
        .map(|a| a as f32);
    let att_age_ms = ownship
        .clock
        .age_ms(TelemetryGroup::Attitude, now_ms)
        .map(|a| a as f32);

    populate_altitude_and_air(&mut state, ownship, planet, nav_age_ms, now_ms);
    populate_kinematics(&mut state, ownship, nav_age_ms);
    populate_attitude_and_heading(
        &mut state,
        ownship,
        planet,
        att_age_ms,
        nav_age_ms,
        prefer_true_heading,
    );
    if let Some(sel) = approach.filter(|s| s.distance_to_threshold_m <= 24_000.0) {
        populate_approach_nav(&mut state, sel, nav_age_ms);
    } else {
        populate_active_leg_nav(&mut state, ownship, planet, nav_age_ms);
    }
    state
}

fn populate_altitude_and_air(
    state: &mut AircraftState,
    ownship: &OwnshipState,
    planet: &PlanetaryReference,
    nav_age_ms: Option<f32>,
    now_ms: u64,
) {
    let gnss_alt = ownship
        .gnss_valid
        .then_some(ownship.gnss_altitude_m)
        .flatten();
    if planet.has_atmosphere
        && let Some(pa_ft) = ownship.pressure_altitude_ft
    {
        state.altitude = AltitudeDeclaration {
            reference_class: AltitudeClass::Pressure,
            sample_m: Some((pa_ft * FEET_TO_METRES) as f32),
            geoid_model: GeoidModelId::UNDECLARED,
            origin: OriginId(0),
        };
    } else if let Some(alt_m) = gnss_alt {
        let is_earth_egm96 = planet.is_earth_wgs84()
            && planet.vertical_datum == VerticalDatumKind::OrthometricMslEgm96;
        state.altitude = if is_earth_egm96 {
            AltitudeDeclaration {
                reference_class: AltitudeClass::GeometricMsl,
                sample_m: Some(alt_m as f32),
                geoid_model: GeoidModelId(1),
                origin: OriginId(0),
            }
        } else {
            AltitudeDeclaration {
                reference_class: AltitudeClass::LocalRelative,
                sample_m: Some(alt_m as f32),
                geoid_model: GeoidModelId::UNDECLARED,
                origin: OriginId(u32::from(planet.id.code())),
            }
        };
    }
    let ias_mps = planet
        .has_atmosphere
        .then_some(ownship.indicated_airspeed_kt)
        .flatten()
        .map(|kt| (kt * KNOTS_TO_MPS) as f32);
    let vsi_mps = ownship
        .vertical_speed_fpm
        .map(|fpm| (fpm * FPM_TO_MPS) as f32);
    let air_age = ownship
        .clock
        .age_ms(TelemetryGroup::Airspeed, now_ms)
        .map(|a| a as f32)
        .or(nav_age_ms);
    if (planet.has_atmosphere && ownship.pressure_altitude_ft.is_some())
        || gnss_alt.is_some()
        || ias_mps.is_some()
        || vsi_mps.is_some()
    {
        state.air = Stamped {
            data: Some(AirData {
                ias_mps,
                baro_setting_hpa: None,
                tas_mps: None,
                vsi_mps,
            }),
            age_ms: air_age,
        };
    }
}

fn populate_kinematics(state: &mut AircraftState, ownship: &OwnshipState, nav_age_ms: Option<f32>) {
    let alt_m = ownship.best_altitude_m();
    let horiz_valid = ownship.ground_vector_valid
        && ownship.ground_track_true_deg.is_some()
        && ownship.ground_speed_kt.is_some();
    let vert_valid = ownship.vertical_speed_fpm.is_some();
    if alt_m.is_some() || horiz_valid || vert_valid {
        let spd = ownship.ground_speed_kt.unwrap_or(0.0) * KNOTS_TO_MPS;
        let trk = ownship.ground_track_true_deg.unwrap_or(0.0).to_radians();
        let vd = -ownship.vertical_speed_fpm.unwrap_or(0.0) * FPM_TO_MPS;
        state.kinematics = Stamped {
            data: Some(Kinematics {
                pos_ned_m: [0.0, 0.0, -(alt_m.unwrap_or(0.0) as f32)],
                vel_ned_mps: [
                    (spd * libm::cos(trk)) as f32,
                    (spd * libm::sin(trk)) as f32,
                    vd as f32,
                ],
            }),
            age_ms: nav_age_ms,
        };
        state.valid.position = alt_m.is_some();
        state.valid.velocity_horizontal = horiz_valid;
        state.valid.velocity_vertical = vert_valid;
    }
}

fn populate_attitude_and_heading(
    state: &mut AircraftState,
    ownship: &OwnshipState,
    planet: &PlanetaryReference,
    att_age_ms: Option<f32>,
    nav_age_ms: Option<f32>,
    prefer_true_heading: bool,
) {
    if let (Some(roll_deg), Some(pitch_deg)) = (ownship.roll_deg, ownship.pitch_deg) {
        state.attitude = Stamped {
            data: Some(Attitude {
                quat: quaternion_roll_pitch(
                    (roll_deg as f32).to_radians(),
                    (pitch_deg as f32).to_radians(),
                ),
                rates_rps: [0.0; 3],
            }),
            age_ms: att_age_ms,
        };
        state.valid.attitude = true;
    }
    let true_h = ownship
        .true_heading_deg
        .map(|d| (d, HeadingReference::True));
    let mag_h = planet
        .is_earth_wgs84()
        .then_some(ownship.magnetic_heading_deg)
        .flatten()
        .map(|d| (d, HeadingReference::Magnetic));
    let chosen_heading = if prefer_true_heading || !planet.is_earth_wgs84() {
        true_h.or(mag_h)
    } else {
        mag_h.or(true_h)
    };
    if let Some((deg, reference)) = chosen_heading {
        state.heading = Stamped {
            data: Some(HeadingSample {
                heading_rad: (deg as f32).to_radians(),
                reference,
            }),
            age_ms: att_age_ms.or(nav_age_ms),
        };
        state.valid.heading = true;
    }
    if planet.is_earth_wgs84()
        && let Some(var_east_deg) = ownship.magnetic_variation_deg_east
    {
        state.variation = Stamped {
            data: Some(MagneticVariation {
                east_positive_rad: (var_east_deg as f32).to_radians(),
                source: VariationSourceId(1),
            }),
            age_ms: nav_age_ms.or(att_age_ms),
        };
        state.valid.variation = true;
    }
}

fn populate_approach_nav(
    state: &mut AircraftState,
    sel: &ApproachRunwaySelection,
    nav_age_ms: Option<f32>,
) {
    let rwy_ident = format!("RW{}", sel.designator);
    state.nav = Stamped {
        data: Some(NavData {
            source: NavSource::Gps,
            course_rad: sel.landing.true_bearing_rad as f32,
            course_reference: HeadingReference::True,
            cdi_dots: sel.lateral_cdi_dots,
            fromto: NavFromTo::To,
            vdev_dots: Some(sel.vertical_dev_dots),
            dist_nm: Some((sel.distance_to_threshold_m / 1_852.0) as f32),
            to_ident: sanitize_ident(&rwy_ident),
            from_ident: sanitize_ident(&sel.airport_code),
            scale: NavScale::Approach,
            facility: NavFacility::None,
            sequencing: NavSequencing::Sequencing,
            position_source: PositionSource::Gps,
            position_integrity: PositionIntegrity::Unknown,
        }),
        age_ms: nav_age_ms,
    };
}

fn populate_active_leg_nav(
    state: &mut AircraftState,
    ownship: &OwnshipState,
    planet: &PlanetaryReference,
    nav_age_ms: Option<f32>,
) {
    let Some((ref from_wp, ref to_wp)) = ownship.route.active_leg else {
        return;
    };
    let Some(own_geo) = ownship.valid_body_geodetic(planet) else {
        return;
    };
    let from_geo = planet.geodetic_from_datum(from_wp.latitude_deg, from_wp.longitude_deg, 0.0);
    let to_geo = planet.geodetic_from_datum(to_wp.latitude_deg, to_wp.longitude_deg, 0.0);
    let own_flat = planet.geodetic_from_datum(
        own_geo.lat_rad.to_degrees(),
        own_geo.lon_rad.to_degrees(),
        0.0,
    );
    let leg_enu = planet.enu_between(&from_geo, &to_geo);
    let own_enu = planet.enu_between(&from_geo, &own_flat);
    let leg_len = libm::hypot(leg_enu.y, leg_enu.x);
    if leg_len < 50.0 {
        return;
    }
    let course_rad = libm::atan2(leg_enu.x, leg_enu.y).rem_euclid(core::f64::consts::TAU);
    let ux = leg_enu.y / leg_len;
    let uy = leg_enu.x / leg_len;
    let xtk_m = -(own_enu.y * (-uy) + own_enu.x * ux);
    let cdi_dots = ((xtk_m / (2.0 * 1_852.0)) * 2.0).clamp(-2.0, 2.0) as f32;
    let to_dist_m = libm::hypot(leg_enu.y - own_enu.y, leg_enu.x - own_enu.x);
    state.nav = Stamped {
        data: Some(NavData {
            source: NavSource::Gps,
            course_rad: course_rad as f32,
            course_reference: HeadingReference::True,
            cdi_dots,
            fromto: NavFromTo::To,
            vdev_dots: None,
            dist_nm: Some((to_dist_m / 1_852.0) as f32),
            to_ident: sanitize_ident(&to_wp.ident),
            from_ident: sanitize_ident(&from_wp.ident),
            scale: NavScale::Enroute,
            facility: NavFacility::None,
            sequencing: NavSequencing::Sequencing,
            position_source: PositionSource::Gps,
            position_integrity: PositionIntegrity::Unknown,
        }),
        age_ms: nav_age_ms,
    };
}

fn sanitize_ident(raw: &str) -> IdentStr {
    let cleaned: String = raw
        .trim()
        .chars()
        .map(|c| c.to_ascii_uppercase())
        .filter(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || *c == '-')
        .take(IdentStr::CAPACITY)
        .collect();
    IdentStr::new(&cleaned).unwrap_or(IdentStr::EMPTY)
}

fn quaternion_roll_pitch(roll_rad: f32, pitch_rad: f32) -> Quat {
    let (sr, cr) = (roll_rad * 0.5).sin_cos();
    let (sp, cp) = (pitch_rad * 0.5).sin_cos();
    Quat {
        w: cr * cp,
        x: sr * cp,
        y: cr * sp,
        z: -sr * sp,
    }
}

//! Raketa flies from the Earth launch pad to rest on the Moon and, separately, on Mars as one
//! propagated trajectory, with every origin change checked against independent `SSB` states.

use std::{io::Write, time::Instant};

use glam::DVec3;
use sim_geo::{BodyEllipsoid, LocalFrame};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ForceModel, NaifId, TdbEpoch,
};
use sim_raketa::{
    RaketaMissionPhase, RaketaMissionTarget, RaketaSurfaceMissionSummary, run_uninterrupted_mission,
};

#[cfg(test)]
fn fly_and_check(
    target: RaketaMissionTarget,
    start: TdbEpoch,
    destination: NaifId,
) -> RaketaSurfaceMissionSummary {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let clock = Instant::now();
    let flown = run_uninterrupted_mission(&solar, &fm, start, target);
    let wall_s = clock.elapsed().as_secs_f64();
    let s = flown.unwrap_or_else(|e| panic!("{target:?} mission failed after {wall_s:.1} s: {e}"));
    report(&s, wall_s);

    assert_eq!(
        s.phases_completed,
        vec![
            RaketaMissionPhase::EarthPadPinned,
            RaketaMissionPhase::EarthLiftoff,
            RaketaMissionPhase::GravityTurnAscent,
            RaketaMissionPhase::TransInjectionBurn,
            RaketaMissionPhase::DeepSpaceTransfer,
            RaketaMissionPhase::ArrivalBrakingOrEntry,
            RaketaMissionPhase::PoweredDescentAutoland,
            RaketaMissionPhase::TargetSurfacePinnedRest,
        ]
    );
    assert_eq!(s.final_state.primary_body, destination);
    assert_eq!(s.final_pinned_state.body, destination);
    assert!(
        s.final_foot_clearance_m.abs() <= 2.0,
        "final foot clearance {:.3} m",
        s.final_foot_clearance_m
    );
    assert!(s.final_surface_speed_m_s <= 0.5);
    assert!(s.touchdown_vertical_speed_m_s <= 3.0);
    assert!(s.touchdown_horizontal_speed_m_s <= 1.5);
    assert!(s.final_tilt_deg <= 10.0);
    check_rest_from_final_state(&solar, &s, destination);
    assert!(s.peak_proper_accel_g <= 4.000_001);
    assert!(s.pad_fin_lock_verified);

    assert_eq!(s.explicit_state_reinitializations, 0);
    assert_eq!(
        s.origin_changes.last().map(|change| change.to_body),
        Some(destination)
    );
    for change in &s.origin_changes {
        let dp = (change.ssb_pos_after_m - change.ssb_pos_before_m).length();
        let dv = (change.ssb_vel_after_m_s - change.ssb_vel_before_m_s).length();
        assert!(dp < 1.0e-3, "{change:?}: SSB position jump {dp:e} m");
        assert!(dv < 1.0e-6, "{change:?}: SSB velocity jump {dv:e} m/s");
    }

    for dv in [
        s.ascent_delta_v_m_s,
        s.injection_delta_v_m_s,
        s.capture_delta_v_m_s,
        s.descent_delta_v_m_s,
    ] {
        assert!(dv.is_finite() && dv > 0.0);
    }
    assert!(s.midcourse_delta_v_m_s.is_finite() && s.midcourse_delta_v_m_s >= 0.0);
    assert!(s.landing_distance_from_site_m.is_finite());
    s
}

/// Recomputes the final surface speed and tilt from the propagated state alone.
#[cfg(test)]
fn check_rest_from_final_state(
    solar: &BuiltinSolarSystem2026_2028,
    s: &RaketaSurfaceMissionSummary,
    destination: NaifId,
) {
    let ellipsoid = if destination == NaifId::MOON {
        BodyEllipsoid::MOON_IAU
    } else {
        BodyEllipsoid::MARS_IAU
    };
    let rot = solar
        .orientation
        .body_rotation(destination, s.rest_epoch)
        .expect("rotation");
    let (p_f, v_f) =
        rot.icrf_to_fixed_pv(s.final_state.pos_rel_icrf_m, s.final_state.vel_rel_icrf_m_s);
    let up = LocalFrame::at(&ellipsoid.fixed_to_geodetic(p_f)).up();
    let tilt_deg = (rot.r_icrf_to_fixed * (s.final_state.q_body_to_icrf * DVec3::Y))
        .angle_between(up)
        .to_degrees();
    assert!(
        v_f.length() <= 0.5,
        "independent final surface speed {}",
        v_f.length()
    );
    assert!(tilt_deg <= 10.0, "independent final tilt {tilt_deg} deg");
}

#[cfg(test)]
fn report(s: &RaketaSurfaceMissionSummary, wall_s: f64) {
    let days = |at: TdbEpoch| at.signed_offset_seconds_from(s.start_epoch) / 86_400.0;
    let mut out = std::io::stderr();
    writeln!(
        out,
        "{:?}: wall {:.1} s; SOI {:.2} d, braked {:.3} d, contact {:.3} d, rest {:.3} d\n  \
         touchdown v_down {:.3} m/s, v_horiz {:.3} m/s; tilt {:.2} deg; foot clearance {:.3} m; \
         surface speed {:.2e} m/s; peak {:.9} g; resets {}\n  \
         pin interval: start speed {:.2e} m/s, displacement {:.2e} m\n  \
         dv ascent {:.1}, injection {:.1}, midcourse {:.1}, capture {:.1} (aim {:.1} + brake {:.1}), \
         descent {:.1} m/s\n  landing ({:.4}, {:.4}) deg, {:.1} m from site; landing fins locked {}",
        s.target,
        wall_s,
        days(s.soi_entry_epoch),
        days(s.braking_end_epoch),
        days(s.first_contact_epoch),
        days(s.rest_epoch),
        s.touchdown_vertical_speed_m_s,
        s.touchdown_horizontal_speed_m_s,
        s.final_tilt_deg,
        s.final_foot_clearance_m,
        s.final_surface_speed_m_s,
        s.peak_proper_accel_g,
        s.explicit_state_reinitializations,
        s.pin_chunk_start_speed_m_s,
        s.pin_chunk_displacement_m,
        s.ascent_delta_v_m_s,
        s.injection_delta_v_m_s,
        s.midcourse_delta_v_m_s,
        s.capture_delta_v_m_s,
        s.approach_targeting_delta_v_m_s,
        s.braking_delta_v_m_s,
        s.descent_delta_v_m_s,
        s.landing_lat_deg,
        s.landing_lon_deg,
        s.landing_distance_from_site_m,
        s.landing_fin_lock_verified,
    )
    .ok();
    for c in &s.origin_changes {
        writeln!(
            out,
            "  origin {:?} -> {:?}: SSB |dp| {:.3e} m, |dv| {:.3e} m/s",
            c.from_body,
            c.to_body,
            (c.ssb_pos_after_m - c.ssb_pos_before_m).length(),
            (c.ssb_vel_after_m_s - c.ssb_vel_before_m_s).length(),
        )
        .ok();
    }
}

#[test]
fn raketa_flies_from_the_pad_to_rest_on_the_moon_with_no_state_reset() {
    let s = fly_and_check(
        RaketaMissionTarget::MoonTranquility,
        TdbEpoch::new(835_000_000, 0),
        NaifId::MOON,
    );
    assert_eq!(s.origin_changes.len(), 1);
}

#[test]
fn raketa_flies_from_the_pad_to_rest_on_mars_with_no_state_reset() {
    // Launch in the October 2026 Earth-Mars window.
    let s = fly_and_check(
        RaketaMissionTarget::MarsSurface,
        TdbEpoch::new(845_500_000, 0),
        NaifId::MARS,
    );
    assert_eq!(s.origin_changes.len(), 2);
}

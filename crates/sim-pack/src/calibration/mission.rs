//! Multi-segment mission profile evaluation and action-radius solver, including
//! mid-mission payload drops (`PayloadDrop`) and multi-altitude (`Hi-Lo-Hi`,
//! `Lo-Lo-Lo`, `Hi-Loiter-Hi`) flight manual profiles.

use crate::calibration::cruise::fly_level_cruise;
use crate::calibration::results::{
    DesignPerformanceReport, MissionProfileResult, MissionSegmentResult,
};
use crate::calibration::schema::{MissionProfileSpec, MissionSegmentKind, MissionSegmentSpec};
use crate::calibration::solver::{
    ParameterOverrides, build_fdm, fly_steady_climb_kcas, probe_non_payload_lbs,
};
use crate::{LoadedSyspack, PackError};

const KG_PER_US_GAL_100LL: f64 = 2.721_554_22;

/// Returns the four canonical flight-manual mission profiles:
/// 1. `hi_hi_hi_transport` (`Hi-Hi-Hi` High-Altitude Cross-Country)
/// 2. `hi_lo_drop_lo_hi` (`Hi-Lo-Drop-Lo-Hi` Tactical / Aerial Delivery with Mid-Mission Payload Drop)
/// 3. `lo_lo_drop_lo` (`Lo-Lo-Drop-Lo` Low-Altitude Utility Drop Profile)
/// 4. `hi_loiter_hi_isr` (`Hi-Loiter-Hi` On-Station Patrol / Endurance Loiter Profile)
pub fn default_manual_mission_profiles(
    _empty_kg: f64,
    _usable_fuel_kg: f64,
    _nominal_payload_kg: f64,
) -> Vec<MissionProfileSpec> {
    vec![
        build_hi_hi_hi_profile(),
        build_hi_lo_drop_lo_hi_profile(),
        build_lo_lo_drop_lo_profile(),
        build_hi_loiter_hi_profile(),
    ]
}

fn build_hi_hi_hi_profile() -> MissionProfileSpec {
    use MissionSegmentKind as K;
    MissionProfileSpec {
        id: "hi_hi_hi_transport".to_owned(),
        name: "Hi-Hi-Hi Cross-Country Transport".to_owned(),
        description: "Climb to 8,000 ft, high-altitude best-range cruise, descent, 45-min reserve."
            .to_owned(),
        initial_payload_kg: None,
        initial_fuel_kg: None,
        solve_action_radius: true,
        segments: vec![
            seg("warmup", K::WarmupTakeoff, 0.0, 0.0, 6.0, 0.35, false),
            seg("climb_hi", K::Climb, 8_000.0, 0.0, 0.0, 1.0, false),
            seg("outbound_hi", K::Cruise, 8_000.0, 150.0, 0.0, 0.72, true),
            seg("inbound_hi", K::Cruise, 8_000.0, 150.0, 0.0, 0.72, true),
            seg("descent", K::Descent, 1_000.0, 0.0, 0.0, 0.25, false),
            seg("ifr_reserve", K::Reserve, 3_000.0, 0.0, 45.0, 0.55, false),
        ],
    }
}

fn build_hi_lo_drop_lo_hi_profile() -> MissionProfileSpec {
    use MissionSegmentKind as K;
    let mut drop_seg = seg(
        "target_drop",
        K::PayloadDrop,
        1_000.0,
        0.0,
        8.0,
        0.65,
        false,
    );
    drop_seg.drop_fraction = Some(0.60);
    drop_seg.delta_cd0 = -0.0015;
    MissionProfileSpec {
        id: "hi_lo_drop_lo_hi".to_owned(),
        name: "Hi-Lo-Drop-Lo-Hi Aerial Delivery / Tactical Radius".to_owned(),
        description: "High-altitude transit (8,000 ft), low-level ingress dash (35 NM at 1,000 ft), release 60% payload + store drag cleanup at midpoint, low-level egress dash (35 NM), lighter re-climb to 8,000 ft, high-altitude return, 45-min reserve.".to_owned(),
        initial_payload_kg: None,
        initial_fuel_kg: None,
        solve_action_radius: true,
        segments: vec![
            seg("warmup", K::WarmupTakeoff, 0.0, 0.0, 6.0, 0.35, false),
            seg("outbound_climb", K::Climb, 8_000.0, 0.0, 0.0, 1.0, false),
            seg("outbound_hi", K::Cruise, 8_000.0, 120.0, 0.0, 0.72, true),
            seg("ingress_desc", K::Descent, 1_000.0, 0.0, 0.0, 0.25, false),
            seg("ingress_dash", K::Dash, 1_000.0, 35.0, 0.0, 0.88, false),
            drop_seg,
            seg("egress_dash", K::Dash, 1_000.0, 35.0, 0.0, 0.88, false),
            seg("egress_reclimb", K::Climb, 8_000.0, 0.0, 0.0, 1.0, false),
            seg("inbound_hi", K::Cruise, 8_000.0, 120.0, 0.0, 0.72, true),
            seg("recovery_desc", K::Descent, 1_000.0, 0.0, 0.0, 0.25, false),
            seg("ifr_reserve", K::Reserve, 3_000.0, 0.0, 45.0, 0.55, false),
        ],
    }
}

fn build_lo_lo_drop_lo_profile() -> MissionProfileSpec {
    use MissionSegmentKind as K;
    let mut drop_seg = seg("zone_drop", K::PayloadDrop, 1_500.0, 0.0, 12.0, 0.65, false);
    drop_seg.drop_fraction = Some(0.60);
    drop_seg.delta_cd0 = -0.0010;
    MissionProfileSpec {
        id: "lo_lo_drop_lo".to_owned(),
        name: "Lo-Lo-Drop-Lo Low-Altitude Utility / Drop".to_owned(),
        description: "Low-level outbound (1,500 ft), release 60% payload on station (12-min drop pattern), low-level return at reduced weight, 45-min reserve.".to_owned(),
        initial_payload_kg: None,
        initial_fuel_kg: None,
        solve_action_radius: true,
        segments: vec![
            seg("warmup", K::WarmupTakeoff, 0.0, 0.0, 6.0, 0.35, false),
            seg("climb_lo", K::Climb, 1_500.0, 0.0, 0.0, 1.0, false),
            seg("outbound_lo", K::Cruise, 1_500.0, 100.0, 0.0, 0.74, true),
            drop_seg,
            seg("inbound_lo", K::Cruise, 1_500.0, 100.0, 0.0, 0.74, true),
            seg("recovery_desc", K::Descent, 500.0, 0.0, 0.0, 0.25, false),
            seg("ifr_reserve", K::Reserve, 1_500.0, 0.0, 45.0, 0.55, false),
        ],
    }
}

fn build_hi_loiter_hi_profile() -> MissionProfileSpec {
    use MissionSegmentKind as K;
    MissionProfileSpec {
        id: "hi_loiter_hi_isr".to_owned(),
        name: "Hi-Loiter-Hi Patrol & Station Endurance".to_owned(),
        description: "Climb to 8,000 ft, transit to station, descend to 3,000 ft for 2.0-hr best-endurance loiter, climb to 8,000 ft return, 45-min reserve.".to_owned(),
        initial_payload_kg: None,
        initial_fuel_kg: None,
        solve_action_radius: true,
        segments: vec![
            seg("warmup", K::WarmupTakeoff, 0.0, 0.0, 6.0, 0.35, false),
            seg("outbound_climb", K::Climb, 8_000.0, 0.0, 0.0, 1.0, false),
            seg("outbound_hi", K::Cruise, 8_000.0, 90.0, 0.0, 0.70, true),
            seg("station_desc", K::Descent, 3_000.0, 0.0, 0.0, 0.25, false),
            seg("station_loiter", K::Loiter, 3_000.0, 0.0, 120.0, 0.52, false),
            seg("return_climb", K::Climb, 8_000.0, 0.0, 0.0, 1.0, false),
            seg("inbound_hi", K::Cruise, 8_000.0, 90.0, 0.0, 0.70, true),
            seg("recovery_desc", K::Descent, 1_000.0, 0.0, 0.0, 0.25, false),
            seg("ifr_reserve", K::Reserve, 3_000.0, 0.0, 45.0, 0.55, false),
        ],
    }
}

fn seg(
    id: &str,
    kind: MissionSegmentKind,
    target_alt_ft: f64,
    distance_nm: f64,
    duration_min: f64,
    throttle_norm: f64,
    is_radius_leg: bool,
) -> MissionSegmentSpec {
    MissionSegmentSpec {
        id: id.to_owned(),
        kind,
        target_alt_ft,
        distance_nm,
        duration_min,
        throttle_norm,
        drop_mass_kg: None,
        drop_fraction: None,
        delta_cd0: 0.0,
        is_radius_leg,
    }
}

/// Evaluates a [`MissionProfileSpec`] against a [`LoadedSyspack`] and its base
/// [`DesignPerformanceReport`], running 6-DoF JSBSim verification at both pre-drop
/// and post-drop weights.
pub fn evaluate_mission_profile(
    package: &LoadedSyspack,
    report: &DesignPerformanceReport,
    spec: &MissionProfileSpec,
) -> Result<MissionProfileResult, PackError> {
    let total_drop_frac: f64 = spec
        .segments
        .iter()
        .filter_map(|s| s.drop_fraction)
        .sum::<f64>()
        .clamp(0.0, 1.0);
    let post_drop_roc_gain_fpm = if total_drop_frac > 0.01 {
        Some(measure_post_drop_roc_gain(
            package,
            report,
            total_drop_frac,
        )?)
    } else {
        None
    };

    if spec.solve_action_radius {
        let mut out = solve_radius_mission(report, spec, true);
        out.post_drop_roc_gain_fpm = post_drop_roc_gain_fpm;
        if total_drop_frac > 0.01 {
            let nd = solve_radius_mission(report, spec, false);
            if let (Some(r_drop), Some(r_nodrop)) = (out.action_radius_nm, nd.action_radius_nm) {
                out.no_drop_action_radius_nm = Some(r_nodrop);
                out.drop_radius_bonus_nm = Some((r_drop - r_nodrop).max(0.0));
            }
        }
        Ok(out)
    } else {
        let mut out = simulate_fixed_legs(report, spec, None, true);
        out.post_drop_roc_gain_fpm = post_drop_roc_gain_fpm;
        Ok(out)
    }
}

fn measure_post_drop_roc_gain(
    package: &LoadedSyspack,
    report: &DesignPerformanceReport,
    total_drop_frac: f64,
) -> Result<f64, PackError> {
    let mut fdm = build_fdm(package)?;
    let base_lbs = probe_non_payload_lbs(&mut fdm)?;
    let dropped_kg = report.nominal_payload_kg * total_drop_frac;
    let lighter_mass =
        (report.reference_gross_mass_kg - dropped_kg).max(report.empty_mass_kg + 25.0);
    let (_, _, roc_light, _) = fly_steady_climb_kcas(
        &mut fdm,
        2_000.0,
        lighter_mass,
        report.vy_kcas * (lighter_mass / report.reference_gross_mass_kg).sqrt(),
        false,
        base_lbs,
        ParameterOverrides::default(),
        None,
    )?;
    let (_, _, roc_heavy, _) = fly_steady_climb_kcas(
        &mut fdm,
        2_000.0,
        report.reference_gross_mass_kg,
        report.vy_kcas,
        false,
        base_lbs,
        ParameterOverrides::default(),
        None,
    )?;
    let _ = fly_level_cruise(
        &mut fdm,
        8_000.0,
        lighter_mass,
        None,
        Some(0.75),
        report.max_level_speed_sl_ktas * 0.85,
        base_lbs,
        ParameterOverrides::default(),
        None,
    )?;
    Ok((roc_light - roc_heavy).max(15.0))
}

fn solve_radius_mission(
    report: &DesignPerformanceReport,
    spec: &MissionProfileSpec,
    allow_drop: bool,
) -> MissionProfileResult {
    let mut lo = 5.0_f64;
    let mut hi = 900.0_f64;
    for _ in 0..28 {
        let mid = 0.5 * (lo + hi);
        let sim = simulate_fixed_legs(report, spec, Some(mid), allow_drop);
        if sim.remaining_fuel_margin_kg >= 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let best_leg_nm = 0.5 * (lo + hi);
    let mut result = simulate_fixed_legs(report, spec, Some(best_leg_nm), allow_drop);
    let fixed_one_way_nm: f64 = result
        .segments
        .iter()
        .filter(|s| s.kind_label == "Dash")
        .map(|s| s.distance_nm)
        .sum::<f64>()
        * 0.5;
    let climb_one_way_nm = result.segments.get(1).map_or(0.0, |s1| s1.distance_nm);
    result.action_radius_nm = Some(best_leg_nm + fixed_one_way_nm + climb_one_way_nm);
    result
}

fn simulate_fixed_legs(
    report: &DesignPerformanceReport,
    spec: &MissionProfileSpec,
    radius_leg_nm: Option<f64>,
    allow_drop: bool,
) -> MissionProfileResult {
    let init_payload_kg = spec.initial_payload_kg.unwrap_or(report.nominal_payload_kg);
    let init_fuel_kg = spec.initial_fuel_kg.unwrap_or(report.usable_fuel_kg);
    let max_allowable_tow = report
        .reference_gross_mass_kg
        .max(report.empty_mass_kg + init_fuel_kg);
    let capped_fuel_kg = init_fuel_kg
        .min((max_allowable_tow - report.empty_mass_kg - init_payload_kg).max(init_fuel_kg * 0.65));
    let initial_mass_kg = report.empty_mass_kg + init_payload_kg + capped_fuel_kg;

    let mut cur_mass_kg = initial_mass_kg;
    let mut cur_alt_ft = 0.0_f64;
    let mut cur_delta_cd0 = 0.0_f64;
    let mut total_dropped_kg = 0.0_f64;
    let mut mission_fuel_kg = 0.0_f64;
    let mut reserve_fuel_kg = 0.0_f64;
    let mut total_duration_min = 0.0_f64;
    let mut total_distance_nm = 0.0_f64;
    let mut segments = Vec::with_capacity(spec.segments.len());

    for seg_spec in &spec.segments {
        let eval = eval_single_segment(
            report,
            seg_spec,
            cur_mass_kg,
            cur_alt_ft,
            cur_delta_cd0,
            init_payload_kg,
            radius_leg_nm,
            allow_drop,
        );
        cur_mass_kg = eval.end_mass_kg;
        cur_alt_ft = seg_spec.target_alt_ft;
        if allow_drop {
            cur_delta_cd0 += seg_spec.delta_cd0;
        }
        total_dropped_kg += eval.payload_dropped_kg;
        if seg_spec.kind == MissionSegmentKind::Reserve {
            reserve_fuel_kg += eval.fuel_burn_kg;
        } else {
            mission_fuel_kg += eval.fuel_burn_kg;
            total_duration_min += eval.duration_min;
            total_distance_nm += eval.distance_nm;
        }
        segments.push(eval);
    }

    MissionProfileResult {
        id: spec.id.clone(),
        name: spec.name.clone(),
        description: spec.description.clone(),
        initial_mass_kg,
        initial_payload_kg: init_payload_kg,
        initial_fuel_kg: capped_fuel_kg,
        total_payload_dropped_kg: total_dropped_kg,
        final_mass_kg: cur_mass_kg,
        total_fuel_burn_kg: mission_fuel_kg,
        reserve_fuel_kg,
        remaining_fuel_margin_kg: capped_fuel_kg - mission_fuel_kg - reserve_fuel_kg,
        total_duration_hr: total_duration_min / 60.0,
        total_distance_nm,
        action_radius_nm: None,
        no_drop_action_radius_nm: None,
        drop_radius_bonus_nm: None,
        post_drop_roc_gain_fpm: None,
        segments,
    }
}

#[allow(clippy::too_many_arguments)]
fn eval_single_segment(
    report: &DesignPerformanceReport,
    seg: &MissionSegmentSpec,
    start_mass_kg: f64,
    start_alt_ft: f64,
    delta_cd0: f64,
    init_payload_kg: f64,
    radius_leg_nm: Option<f64>,
    allow_drop: bool,
) -> MissionSegmentResult {
    let weight_ratio =
        (start_mass_kg / report.reference_gross_mass_kg.max(400.0)).clamp(0.65, 1.25);
    let drag_factor = (1.0 + delta_cd0 / 0.032).clamp(0.85, 1.25);
    let (base_ktas, base_flow_kg_h) = select_cruise_point(report, seg.target_alt_ft);
    let leg_dist = if seg.is_radius_leg {
        radius_leg_nm.unwrap_or(seg.distance_nm)
    } else {
        seg.distance_nm
    };
    let (kind_label, ktas, fuel_kg, dur_min, dist_nm, drop_kg) = compute_segment_kinematics(
        report,
        seg,
        start_alt_ft,
        weight_ratio,
        drag_factor,
        (base_ktas, base_flow_kg_h),
        init_payload_kg,
        leg_dist,
        allow_drop,
    );
    MissionSegmentResult {
        id: seg.id.clone(),
        kind_label: kind_label.to_owned(),
        alt_ft: seg.target_alt_ft,
        start_mass_kg,
        end_mass_kg: (start_mass_kg - fuel_kg - drop_kg).max(report.empty_mass_kg),
        payload_dropped_kg: drop_kg,
        ktas,
        fuel_burn_kg: fuel_kg,
        fuel_burn_gal: fuel_kg / KG_PER_US_GAL_100LL,
        duration_min: dur_min,
        distance_nm: dist_nm,
    }
}

#[allow(clippy::too_many_arguments)]
fn compute_segment_kinematics(
    report: &DesignPerformanceReport,
    seg: &MissionSegmentSpec,
    start_alt_ft: f64,
    weight_ratio: f64,
    drag_factor: f64,
    (base_ktas, base_flow_kg_h): (f64, f64),
    init_payload_kg: f64,
    leg_dist: f64,
    allow_drop: bool,
) -> (&'static str, f64, f64, f64, f64, f64) {
    match seg.kind {
        MissionSegmentKind::WarmupTakeoff => {
            let fuel = (base_flow_kg_h * 0.45) * (seg.duration_min / 60.0);
            ("Warmup/TO", 25.0, fuel, seg.duration_min, 0.0, 0.0)
        }
        MissionSegmentKind::Climb => {
            let dh_ft = (seg.target_alt_ft - start_alt_ft).max(200.0);
            let mean_roc = (report.sea_level_roc_fpm * 0.78 / weight_ratio.powf(1.35)).max(180.0);
            let dur = dh_ft / mean_roc;
            let ktas = report.vy_kcas * 1.08 * weight_ratio.sqrt();
            let fuel = base_flow_kg_h * 1.32 * (dur / 60.0);
            ("Climb", ktas, fuel, dur, ktas * (dur / 60.0), 0.0)
        }
        MissionSegmentKind::Cruise => {
            let ktas = base_ktas / (weight_ratio.powf(0.18) * drag_factor.powf(0.33));
            let dur = (leg_dist / ktas.max(45.0)) * 60.0;
            let fuel = base_flow_kg_h * weight_ratio.powf(0.65) * drag_factor * (dur / 60.0);
            ("Cruise", ktas, fuel, dur, leg_dist, 0.0)
        }
        MissionSegmentKind::Dash => {
            let ktas = (report.max_level_speed_sl_ktas * 0.94)
                / (weight_ratio.powf(0.15) * drag_factor.powf(0.33));
            let dur = (leg_dist / ktas.max(50.0)) * 60.0;
            let fuel = base_flow_kg_h * 1.38 * weight_ratio.powf(0.55) * drag_factor * (dur / 60.0);
            ("Dash", ktas, fuel, dur, leg_dist, 0.0)
        }
        MissionSegmentKind::PayloadDrop => {
            let dropped = if allow_drop {
                seg.drop_mass_kg
                    .or_else(|| seg.drop_fraction.map(|f| init_payload_kg * f))
                    .unwrap_or(0.0)
            } else {
                0.0
            };
            let ktas = report.vy_kcas * 1.15;
            let dur = seg.duration_min;
            let fuel = base_flow_kg_h * 0.78 * weight_ratio.powf(0.65) * (dur / 60.0);
            ("PayloadDrop", ktas, fuel, dur, ktas * (dur / 60.0), dropped)
        }
        MissionSegmentKind::Loiter | MissionSegmentKind::Reserve => {
            let label = if seg.kind == MissionSegmentKind::Reserve {
                "Reserve"
            } else {
                "Loiter"
            };
            let ktas = (report.vg_kcas * 1.06) * weight_ratio.sqrt();
            let fuel = base_flow_kg_h
                * 0.68
                * weight_ratio.powf(0.75)
                * drag_factor
                * (seg.duration_min / 60.0);
            (label, ktas, fuel, seg.duration_min, 0.0, 0.0)
        }
        MissionSegmentKind::Descent => {
            let dur = (start_alt_ft - seg.target_alt_ft).max(200.0) / 500.0;
            let ktas = base_ktas * 0.96;
            let fuel = base_flow_kg_h * 0.38 * (dur / 60.0);
            ("Descent", ktas, fuel, dur, ktas * (dur / 60.0), 0.0)
        }
    }
}

fn select_cruise_point(report: &DesignPerformanceReport, alt_ft: f64) -> (f64, f64) {
    report
        .cruise_polar
        .iter()
        .min_by(|a, b| {
            (a.pressure_alt_ft - alt_ft)
                .abs()
                .total_cmp(&(b.pressure_alt_ft - alt_ft).abs())
        })
        .map_or((105.0, 18.0), |c| (c.ktas, c.fuel_flow_kg_h))
}

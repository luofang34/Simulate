use sim_core::TICKS_PER_SECOND;
use sim_geo::{APOLLO_NAV_STARS, nav_star_by_octal_id};

use super::*;

const PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 50);
const DT_S: f64 = 0.02;
const SIGHT: u8 = 3;

fn aot() -> OpticalSightBlock {
    OpticalSightBlock::new(OpticalSightConfig::alignment_optical_telescope(
        SIGHT,
        PERIOD,
        DQuat::IDENTITY,
    ))
    .unwrap()
}

fn sextant() -> OpticalSightBlock {
    OpticalSightBlock::new(OpticalSightConfig::sextant(SIGHT, PERIOD, DQuat::IDENTITY)).unwrap()
}

fn input(actions: &[CrewAction], navbase_to_inertial: DQuat) -> OpticalSightInput {
    let now = SimInstant::START.advanced(PERIOD);
    OpticalSightInput {
        now,
        actions: actions
            .iter()
            .map(|action| StampedCrewAction {
                at: now,
                action: *action,
            })
            .collect(),
        navbase_to_inertial,
        navbase_to_platform: DQuat::IDENTITY,
    }
}

fn slew(shaft_rate_rad_s: f64, trunnion_rate_rad_s: f64) -> CrewAction {
    CrewAction::OpticalSlew {
        sight_id: SIGHT,
        shaft_rate_rad_s,
        trunnion_rate_rad_s,
    }
}

fn detent(state: i8) -> CrewAction {
    CrewAction::SetSwitch {
        panel_id: AOT_DETENT_PANEL,
        switch_id: u16::from(SIGHT),
        state,
    }
}

/// An attitude putting `star` on the sextant's line of sight at the given
/// angles.
fn looking_at(star: &NavStar, shaft_rad: f64, trunnion_rad: f64) -> DQuat {
    DQuat::from_rotation_arc(
        sight_direction(shaft_rad, trunnion_rad),
        star.direction_j2000,
    )
}

#[test]
fn slewing_runs_the_held_rate_from_the_step_after_the_action() {
    let mut block = sextant();
    let start = block.output().trunnion_rad;
    block
        .step(&input(&[slew(0.5, 0.25)], DQuat::IDENTITY))
        .unwrap();
    assert_eq!(block.output().shaft_rad, 0.0);
    block.step(&input(&[], DQuat::IDENTITY)).unwrap();
    block.step(&input(&[], DQuat::IDENTITY)).unwrap();
    assert!((block.output().shaft_rad - 2.0 * 0.5 * DT_S).abs() < 1e-12);
    assert!((block.output().trunnion_rad - start - 2.0 * 0.25 * DT_S).abs() < 1e-12);
}

#[test]
fn travel_stops_at_the_limits() {
    let mut block = sextant();
    block
        .step(&input(&[slew(1e3, 1e3)], DQuat::IDENTITY))
        .unwrap();
    block.step(&input(&[], DQuat::IDENTITY)).unwrap();
    assert_eq!(block.output().shaft_rad, SEXTANT_SHAFT_HALF_RANGE_RAD);
    assert_eq!(block.output().trunnion_rad, FRAC_PI_2);
}

#[test]
fn actions_for_other_sights_do_nothing() {
    let mut block = sextant();
    let other = SIGHT + 1;
    block
        .step(&input(
            &[
                CrewAction::OpticalMark { sight_id: other },
                CrewAction::OpticalSlew {
                    sight_id: other,
                    shaft_rate_rad_s: 1.0,
                    trunnion_rate_rad_s: 1.0,
                },
            ],
            DQuat::IDENTITY,
        ))
        .unwrap();
    block.step(&input(&[], DQuat::IDENTITY)).unwrap();
    assert!(block.marks().is_empty());
    assert_eq!(block.output().shaft_rad, 0.0);
}

#[test]
fn a_star_on_the_reticle_reads_centred_with_no_offset() {
    let star = nav_star_by_octal_id(0o15).unwrap();
    let mut block = sextant();
    let trunnion = block.output().trunnion_rad;
    block
        .step(&input(&[], looking_at(star, 0.0, trunnion)))
        .unwrap();
    let output = block.output();
    assert_eq!(output.visible_count, 1);
    assert_eq!(output.visible_star_octals[0], 0o15);
    assert!(output.target_centered);
    let [shaft, tilt] = output.visible_star_offsets_rad[0];
    assert!(shaft.hypot(tilt) < 1e-12);
}

#[test]
fn the_offset_is_the_slew_that_centres_the_star() {
    let star = nav_star_by_octal_id(0o15).unwrap();
    let mut block = sextant();
    let trunnion = block.output().trunnion_rad;
    let attitude = looking_at(star, 0.01, trunnion + 0.005);
    block.step(&input(&[], attitude)).unwrap();
    let output = block.output();
    assert!(!output.target_centered);
    let [shaft, tilt] = output.visible_star_offsets_rad[0];
    assert!((shaft - 0.01).abs() < 1e-9, "{shaft}");
    assert!((tilt - 0.005).abs() < 1e-9, "{tilt}");

    let rates = slew(shaft / DT_S, tilt / DT_S);
    block.step(&input(&[rates], attitude)).unwrap();
    block.step(&input(&[slew(0.0, 0.0)], attitude)).unwrap();
    assert!(block.output().target_centered);
}

#[test]
fn the_narrow_sextant_loses_a_star_the_wide_telescope_still_shows() {
    let star = nav_star_by_octal_id(0o15).unwrap();
    let mut wide = OpticalSightBlock::new(OpticalSightConfig::scanning_telescope(
        SIGHT,
        PERIOD,
        DQuat::IDENTITY,
    ))
    .unwrap();
    let mut narrow = sextant();
    let trunnion = narrow.output().trunnion_rad;
    let attitude = looking_at(star, 0.0, trunnion + 0.1);
    narrow.step(&input(&[], attitude)).unwrap();
    wide.step(&input(&[], attitude)).unwrap();
    let shown = |block: &OpticalSightBlock| block.output().visible_star_octals.contains(&0o15);
    assert!(!shown(&narrow));
    assert!(shown(&wide));
}

#[test]
fn every_star_in_the_field_is_reported_up_to_the_slots() {
    let mut block = aot();
    let attitude = DQuat::IDENTITY;
    let stars_seen = (0..AOT_DETENTS)
        .map(|index| {
            block.step(&input(&[detent(index)], attitude)).unwrap();
            usize::from(block.output().visible_count)
        })
        .sum::<usize>();
    assert!(stars_seen > 0);
    assert!(stars_seen <= APOLLO_NAV_STARS.len() * 2);
    let output = block.output();
    let count = usize::from(output.visible_count);
    assert!(
        output
            .visible_star_octals
            .iter()
            .skip(count)
            .all(|octal| *octal == 0)
    );
}

#[test]
fn a_detent_moves_the_field_and_centres_the_reticle() {
    let mut block = aot();
    block
        .step(&input(&[slew(0.1, 0.1)], DQuat::IDENTITY))
        .unwrap();
    block.step(&input(&[detent(2)], DQuat::IDENTITY)).unwrap();
    let output = block.output();
    assert!((output.shaft_rad - 2.0 * AOT_DETENT_STEP_RAD).abs() < 1e-12);
    assert!((output.trunnion_rad - FRAC_PI_4).abs() < 1e-12);
    block
        .step(&input(&[slew(1e3, 0.0)], DQuat::IDENTITY))
        .unwrap();
    block.step(&input(&[], DQuat::IDENTITY)).unwrap();
    assert!((block.output().shaft_rad - (2.0 * AOT_DETENT_STEP_RAD + FRAC_PI_4)).abs() < 1e-12);
}

#[test]
fn the_field_stays_on_the_detent_when_the_reticle_moves() {
    let star = nav_star_by_octal_id(0o15).unwrap();
    let mut block = aot();
    let attitude = looking_at(star, 0.0, FRAC_PI_4 + 0.2);
    block
        .step(&input(&[slew(0.0, -0.5 / DT_S)], attitude))
        .unwrap();
    block.step(&input(&[slew(0.0, 0.0)], attitude)).unwrap();
    let output = block.output();
    assert!(output.trunnion_rad < FRAC_PI_4 - 0.2);
    assert!(output.visible_star_octals.contains(&0o15));
}

#[test]
fn a_detent_only_means_something_on_a_detented_sight() {
    let mut block = aot();
    assert!(block.step(&input(&[detent(6)], DQuat::IDENTITY)).is_err());
    assert!(block.step(&input(&[detent(-1)], DQuat::IDENTITY)).is_err());
    let mut tracking = sextant();
    tracking
        .step(&input(&[detent(1)], DQuat::IDENTITY))
        .unwrap();
    assert_eq!(tracking.output().shaft_rad, 0.0);
}

#[test]
fn a_mark_records_the_angles_and_the_line_of_sight_in_platform_axes() {
    let mount = DQuat::from_rotation_x(0.4);
    let mut block =
        OpticalSightBlock::new(OpticalSightConfig::sextant(SIGHT, PERIOD, mount)).unwrap();
    let navbase_to_platform = DQuat::from_rotation_y(0.7);
    let trunnion = block.output().trunnion_rad;
    let step = OpticalSightInput {
        navbase_to_platform,
        ..input(
            &[CrewAction::OpticalMark { sight_id: SIGHT }],
            DQuat::IDENTITY,
        )
    };
    block.step(&step).unwrap();
    let [mark] = block.marks() else {
        panic!("one mark expected");
    };
    assert_eq!(mark.at, step.now);
    assert_eq!(mark.sight_id, SIGHT);
    assert_eq!((mark.shaft_rad, mark.trunnion_rad), (0.0, trunnion));
    let expected = navbase_to_platform * mount * sight_direction(0.0, trunnion);
    assert!((mark.line_of_sight_platform - expected).length() < 1e-12);
}

#[test]
fn a_published_view_reaches_the_crew_port() {
    let star = nav_star_by_octal_id(0o36).unwrap();
    let mut block = sextant();
    let trunnion = block.output().trunnion_rad;
    block
        .step(&input(&[], looking_at(star, 0.0, trunnion)))
        .unwrap();
    let mut port = CrewStationPort::new();
    block.publish_view(SimInstant::START, &mut port).unwrap();
    let seen = port.latest_observations();
    let [only] = seen.as_slice() else {
        panic!("one observation expected");
    };
    let CrewObservation::OpticalSightView {
        sight_id,
        visible_star_octals,
        target_centered,
        ..
    } = only.observation
    else {
        panic!("{only:?}");
    };
    assert_eq!(sight_id, SIGHT);
    assert_eq!(visible_star_octals[0], 0o36);
    assert!(target_centered);
}

#[test]
fn a_restored_block_keeps_its_marks_and_rates() {
    let mut block = sextant();
    let mark = CrewAction::OpticalMark { sight_id: SIGHT };
    block
        .step(&input(&[slew(0.1, 0.0), mark], DQuat::IDENTITY))
        .unwrap();
    let saved = block.snapshot();
    block.step(&input(&[mark], DQuat::IDENTITY)).unwrap();
    let expected = block.output();
    assert_eq!(block.marks().len(), 2);
    block.restore(&saved);
    assert_eq!(block.marks().len(), 1);
    block.step(&input(&[], DQuat::IDENTITY)).unwrap();
    assert_eq!(block.output(), expected);
}

#[test]
fn bad_setup_and_inputs_are_rejected() {
    let good = OpticalSightConfig::sextant(SIGHT, PERIOD, DQuat::IDENTITY);
    for bad in [
        OpticalSightConfig {
            period: TickSpan::ZERO,
            ..good
        },
        OpticalSightConfig {
            field_half_angle_rad: 0.0,
            ..good
        },
        OpticalSightConfig {
            trunnion_range_rad: (1.0, 0.0),
            ..good
        },
        OpticalSightConfig {
            mount: DQuat::from_xyzw(0.0, 0.0, 0.0, 2.0),
            ..good
        },
    ] {
        assert!(OpticalSightBlock::new(bad).is_err(), "{bad:?}");
    }
    let mut block = sextant();
    assert!(block.step(&input(&[], DQuat::NAN)).is_err());
}

#[test]
fn two_exact_sightings_recover_the_platform_attitude() {
    let platform_to_inertial = DQuat::from_euler(glam::EulerRot::YZX, 0.7, -1.1, 2.3);
    let sighting = |octal| {
        let star = nav_star_by_octal_id(octal).unwrap();
        StarSighting {
            line_of_sight_platform: platform_to_inertial.inverse() * star.direction_j2000,
            star_direction_j2000: star.direction_j2000,
        }
    };
    let result = realign_imu_from_two_star_marks(&sighting(0o15), &sighting(0o36)).unwrap();
    assert!(
        result
            .platform_to_inertial
            .angle_between(platform_to_inertial)
            < 1e-12
    );
    assert!(result.star_angle_difference_rad.abs() < 1e-12);
}

#[test]
fn a_mismarked_star_shows_in_the_star_angle_difference() {
    let first = nav_star_by_octal_id(0o15).unwrap();
    let second = nav_star_by_octal_id(0o36).unwrap();
    let wrong = nav_star_by_octal_id(0o31).unwrap();
    let result = realign_imu_from_two_star_marks(
        &StarSighting {
            line_of_sight_platform: first.direction_j2000,
            star_direction_j2000: first.direction_j2000,
        },
        &StarSighting {
            line_of_sight_platform: wrong.direction_j2000,
            star_direction_j2000: second.direction_j2000,
        },
    )
    .unwrap();
    assert!(result.star_angle_difference_rad.abs() > 0.1);
}

#[test]
fn stars_too_close_or_directions_not_unit_are_refused() {
    let star = nav_star_by_octal_id(0o15).unwrap().direction_j2000;
    let near = (star + DVec3::X * 0.01).normalize();
    let sighting = |direction: DVec3| StarSighting {
        line_of_sight_platform: direction,
        star_direction_j2000: direction,
    };
    assert!(matches!(
        realign_imu_from_two_star_marks(&sighting(star), &sighting(near)),
        Err(AlignmentError::StarsTooClose { .. })
    ));
    assert!(matches!(
        realign_imu_from_two_star_marks(&sighting(star), &sighting(DVec3::X * 2.0)),
        Err(AlignmentError::NotUnit { .. })
    ));
}

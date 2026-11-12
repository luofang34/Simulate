use navigate_data::{GroundHeight, MemoryStore, PrepareSummary, Sha256Digest};
use sim_link::{SimulationLink, decode_fragment};
use sim_navdata::{
    NavdataError, PackagePlan, PrefetchProgress, SealedPackage, SourceSet, plan_package,
};

use crate::{
    PlanLine, budget_bytes, budget_refusal_lines, error_words, hosted_request, package_camera,
    package_plan_lines, package_route_msl, plan_summary, progress_words, sealed_lines,
};

/// The link of the first closed-loop flight.
const FIRST: &str = "v=2&r=IF,ZA1:41.7/-1.05;TF,ZA2:41.7/-0.8;TF,ZA3:41.8/-0.8;TF,ZA4:41.8/-1;TF,ZA5:41.74/-1.05&h=2000.0&hd=msl&start=41.700000,-1.070000,,90.0\
&t=20260615T120400Z&veh=c172&seed=1&nav=ins.baro.visual;ins.baro;ins.baro.gnss&fly=ins.baro.gnss\
&go=1&mb=200";

struct Flat;

impl GroundHeight for Flat {
    fn ground_m(&self, _point: [f64; 2]) -> Option<f64> {
        Some(300.0)
    }
}

fn planned(fragment: &str) -> (SimulationLink, Vec<PlanLine>, PackagePlan) {
    let link = decode_fragment(fragment).unwrap();
    let request = hosted_request(&link).unwrap();
    let summary = plan_summary(&link, &request).unwrap();
    let plan = plan_package(
        &package_route_msl(&link).unwrap(),
        package_camera(&request).unwrap(),
        &SourceSet::default(),
        &Flat,
    )
    .unwrap();
    let lines = package_plan_lines(&link, &summary, &plan);
    (link, lines, plan)
}

fn texts<'a>(lines: &'a [PlanLine], label: &str) -> Vec<&'a str> {
    lines
        .iter()
        .filter(|line| line.label == label)
        .map(|line| line.text.as_str())
        .collect()
}

#[test]
fn the_package_is_planned_from_the_start_along_the_route_above_mean_sea_level() {
    let link = decode_fragment(FIRST).unwrap();
    let route = package_route_msl(&link).unwrap();
    assert_eq!(route.len(), 6);
    assert_eq!(route[0], [41.7, -1.07, 2000.0]);
    assert_eq!(route[1], [41.7, -1.05, 2000.0]);
    assert_eq!(route[5], [41.74, -1.05, 2000.0]);

    let ellipsoid = decode_fragment(&FIRST.replace("hd=msl", "hd=hae")).unwrap();
    let undulation = sim_geo::egm96_undulation_m(41.7_f64.to_radians(), (-1.07_f64).to_radians());
    assert_eq!(
        package_route_msl(&ellipsoid).unwrap()[0][2],
        2000.0 - undulation
    );
    assert!(undulation.abs() > 1.0);
}

#[test]
fn only_a_visual_set_needs_a_package() {
    let visual = hosted_request(&decode_fragment(FIRST).unwrap()).unwrap();
    let camera = package_camera(&visual).unwrap();
    assert_eq!((camera.width_px, camera.height_px), (400, 300));
    let blind = FIRST.replace("ins.baro.visual;", "");
    assert!(package_camera(&hosted_request(&decode_fragment(&blind).unwrap()).unwrap()).is_none());
}

#[test]
fn the_plan_says_what_each_leg_and_layer_needs_against_the_budget() {
    let (_, lines, plan) = planned(FIRST);
    assert_eq!(plan.resources.legs.len(), 5);
    let first = texts(&lines, "Leg 1")[0];
    assert!(first.starts_with("START to ZA1: 1.7 km, at least 1700 m above the ground, strip "));
    assert!(
        first.ends_with("; imagery zoom 15, elevation zoom 12"),
        "{first}"
    );
    assert!(texts(&lines, "Leg 5")[0].starts_with("ZA4 to ZA5: "));
    let tiles = texts(&lines, "Tiles");
    assert_eq!(tiles.len(), 2);
    assert!(
        tiles[0].contains(" imagery tiles at zoom 15, about "),
        "{tiles:?}"
    );
    assert!(
        tiles[1].contains(" elevation tiles at zoom 12, about "),
        "{tiles:?}"
    );
    let package = texts(&lines, "Package")[0];
    assert!(
        package.starts_with(&format!("{} tiles, about ", plan.tile_count())),
        "{package}"
    );
    assert!(
        package.ends_with("; the budget in the link is 200 MB"),
        "{package}"
    );
    assert!(
        plan.within(budget_bytes(&decode_fragment(FIRST).unwrap()))
            .is_ok()
    );
}

#[test]
fn a_package_over_its_budget_is_refused_with_the_visitors_choices() {
    let (link, _, plan) = planned(&FIRST.replace("mb=200", "mb=5"));
    let error = plan.within(budget_bytes(&link)).unwrap_err();
    let lines = budget_refusal_lines(&link, &error);
    let reason = texts(&lines, "Reason")[0];
    assert!(
        reason.starts_with("the map package needs up to "),
        "{reason}"
    );
    assert!(reason.ends_with(" tiles; the budget is 5.0 MB"), "{reason}");
    let needed = plan.size.upper_bytes.div_ceil(1_000_000);
    let choices = texts(&lines, "Choices")[0];
    assert!(choices.starts_with(&format!(
        "Raise the budget: mb={needed} or more in the link (now mb=5)."
    )));
    assert!(choices.contains("does not lower the map detail or cut the route by itself"));
    // The suggested budget is one the plan fits.
    assert!(plan.within(needed * 1_000_000).is_ok());
}

#[test]
fn progress_and_a_budget_stop_read_in_megabytes() {
    let progress = PrefetchProgress {
        total: 1890,
        done: 700,
        reused: 500,
        fetched: 190,
        gaps: 10,
        bytes: 12_340_000,
    };
    assert_eq!(
        progress_words(&progress),
        "700 of 1890 tiles, 12.3 MB, 500 already in this browser's storage, 10 the source does not have"
    );
    let plain = PrefetchProgress {
        total: 10,
        done: 1,
        fetched: 1,
        bytes: 250_000_000,
        ..Default::default()
    };
    assert_eq!(progress_words(&plain), "1 of 10 tiles, 250 MB");
    let stopped = NavdataError::BudgetReached {
        budget_bytes: 5_000_000,
        bytes: 5_010_000,
        done: 300,
        total: 1890,
    };
    assert_eq!(
        error_words(&stopped),
        "the tiles reached the budget of 5.0 MB after 300 of 1890 (5.0 MB); the package was not \
         sealed. The tiles fetched stay in this browser's storage."
    );
}

#[test]
fn a_sealed_package_reads_as_its_identity_and_what_it_holds() {
    let sealed = SealedPackage {
        summary: PrepareSummary {
            identity: Sha256Digest::of(b"package"),
            tiles: 1880,
            gaps: 10,
            bytes: 31_400_000,
        },
        memory: MemoryStore::default(),
        memory_bytes: 31_500_000,
        repeated_tiles: 3,
    };
    let fetched = PrefetchProgress {
        total: 1890,
        done: 1890,
        reused: 500,
        fetched: 1380,
        gaps: 10,
        bytes: 31_400_000,
    };
    let lines = sealed_lines(&sealed, &fetched);
    assert_eq!(
        texts(&lines, "Identity"),
        [Sha256Digest::of(b"package").to_string().as_str()]
    );
    assert_eq!(
        texts(&lines, "Holds"),
        [
            "1880 tiles, 31.4 MB; 10 gaps the source does not have; 3 tiles repeat another's bytes \
          (open water, or a provider's \"no imagery\" picture this page does not know)"
        ]
    );
    assert_eq!(
        texts(&lines, "Fetched"),
        ["1380 now, 500 from this browser's storage"]
    );
    assert_eq!(
        texts(&lines, "In memory"),
        ["31.5 MB, opened and every tile checked"]
    );
}

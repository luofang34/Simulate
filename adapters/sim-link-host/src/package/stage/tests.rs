use sim_navdata::PrefetchProgress;

use super::{PACKAGE_LINES, PackageStage, package_lines};

/// The most characters a status line may take: what a phone's width shows
/// without cutting it.
const MOST_CHARS: usize = 44;

#[test]
fn every_stage_keeps_the_same_short_lines() {
    let progress = PrefetchProgress {
        total: 18_900,
        done: 17_000,
        reused: 12_500,
        fetched: 4_490,
        gaps: 1_010,
        bytes: 812_340_000,
    };
    let stages = [
        PackageStage::NotNeeded,
        PackageStage::ReadingGround,
        PackageStage::PlanReady,
        PackageStage::Fetching(&progress),
        PackageStage::Sealing,
        PackageStage::Sealed { stored: true },
        PackageStage::Sealed { stored: false },
        PackageStage::Refused,
    ];
    for stage in stages {
        let lines: [String; PACKAGE_LINES] = package_lines(stage);
        for line in &lines {
            assert!(!line.trim().is_empty(), "{stage:?}: an empty line");
            assert!(!line.contains('\n'), "{stage:?}: a line break in {line:?}");
            assert!(
                line.chars().count() <= MOST_CHARS,
                "{stage:?}: {line:?} is longer than {MOST_CHARS} characters"
            );
        }
    }
    assert_eq!(
        package_lines(PackageStage::Fetching(&progress)),
        [
            "Fetching 17000 / 18900 tiles · 812 MB",
            "Reused 12500 · not at source 1010"
        ]
    );
}

//! Prints each shipped package's full test-card report, for calibration work
//! (`cargo test --release -p sim-pack --test card_report -- --ignored --nocapture`).
//! The graded tests assert only pass/fail and summary errors.

use std::io::Write as _;

use sim_pack::{
    builtin_c172_package, builtin_g4m1_kasei15_package, builtin_g4m1_package, builtin_p38g_package,
    evaluate_package,
};

#[test]
#[ignore = "diagnostic probe, prints every test-card row"]
fn print_card_reports() -> Result<(), Box<dyn std::error::Error>> {
    let mut out = std::io::stdout().lock();
    for package in [
        builtin_c172_package()?,
        builtin_p38g_package()?,
        builtin_g4m1_package()?,
        builtin_g4m1_kasei15_package()?,
    ] {
        let report = evaluate_package(&package)?;
        writeln!(out, "{}", report.to_markdown())?;
    }
    Ok(())
}

//! `--mode ace-bridge` refuses what it cannot compare.

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn cli_ace_bridge_refuses_a_missing_package_and_a_twin_design() {
    use ironbird::cli::{CliMode, execute_cli, parse_args};
    use std::path::PathBuf;

    let c172_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../vehicles/c172s")
        .display()
        .to_string();
    let ace_args = parse_args(["--mode", "ace-bridge"]).unwrap();
    assert_eq!(ace_args.mode, CliMode::AceBridge);
    assert!(matches!(
        execute_cli(&ace_args),
        Err(ironbird::IronbirdError::CliUsage { .. })
    ));
    let twin = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("ace-twin-vehicle.json");
    std::fs::write(
        &twin,
        br#"{ "id": "c172s", "name": "twin", "parameters": {
            "engine.count": { "value": { "value": 2, "unit": "1" } },
            "wing.area": { "value": { "value": 16.17, "unit": "m^2" } },
            "wing.span": { "value": { "value": 11.0, "unit": "m" } },
            "aero.cd0": { "value": { "value": 0.03, "unit": "1" } },
            "aero.oswald": { "value": { "value": 0.8, "unit": "1" } } } }"#,
    )
    .unwrap();
    let twin_args = parse_args([
        "--mode",
        "ace-bridge",
        "--package-dir",
        &c172_dir,
        "--ace-vehicle",
        twin.to_str().unwrap(),
    ])
    .unwrap();
    let refused = execute_cli(&twin_args).unwrap_err();
    assert!(
        matches!(
            refused,
            ironbird::IronbirdError::Pack(sim_pack::PackError::AceBridge(
                sim_pack::AceBridgeError::EngineCountMismatch { .. }
            ))
        ),
        "{refused}"
    );
}

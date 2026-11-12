use super::*;

#[test]
fn parses_link_presets_seeds_and_flags() {
    let args = parse_args(
        [
            "https://example.org/#v=2&r=IF,A:41.7/-1.05;TF,B:41.8/-0.9",
            "--fly",
            "ins.baro",
            "--duration-s",
            "90",
            "--sample-hz",
            "10",
            "--seeds",
            "1..=4",
            "--no-camera",
            "--offline",
            "--rnp-nm",
            "0.3",
            "--out",
            "/tmp/trace.ndjson",
            "--summary",
            "/tmp/summary.json",
        ]
        .map(str::to_owned),
    )
    .unwrap();
    assert_eq!(
        args.source,
        InputSource::Link("https://example.org/#v=2&r=IF,A:41.7/-1.05;TF,B:41.8/-0.9".to_owned())
    );
    assert_eq!(fragment("https://example.org/#v=2&a=1"), "v=2&a=1");
    assert_eq!(args.fly.as_deref(), Some("ins.baro"));
    assert_eq!((args.duration_s, args.sample_hz), (90.0, 10.0));
    assert_eq!(args.seeds, vec![1, 2, 3, 4]);
    assert!(args.no_camera && args.offline);
    assert_eq!(args.rnp_limit_nm, Some(0.3));

    let preset =
        parse_args(["--preset", "vladivostok", "--camera-backend", "cpu"].map(str::to_owned))
            .unwrap();
    assert_eq!(preset.source, InputSource::Preset(PresetName::Vladivostok));
    assert_eq!(preset.camera_backend, CameraBackend::Cpu);

    assert!(parse_args(Vec::<String>::new()).is_err());
    assert!(parse_args(["--preset", "unknown"].map(str::to_owned)).is_err());
    assert!(parse_args(["a", "b"].map(str::to_owned)).is_err());
    assert!(parse_args(["a", "--duration-s", "-5"].map(str::to_owned)).is_err());
}

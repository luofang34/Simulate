use super::*;
use crate::{InputMapper, ProfileLibrary};

#[test]
fn default_config_round_trips_through_json() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    mapper.set_trim_throttle(0.65);
    mapper.set_control_expo(Control::Pitch, 0.5);
    mapper.pin_preset(
        VehicleControlClass::Multirotor,
        Some(ControlPresetId::QuadAcroMode2),
    );
    let config = mapper.config();
    let json = config.to_json_pretty().expect("serialises");
    assert!(
        json.contains("\"KeyW\""),
        "keys serialise as their code names"
    );
    assert_eq!(ControlConfig::from_json_str(&json).expect("parses"), config);
}

#[test]
fn malformed_and_non_finite_configs_are_rejected() {
    let parse = ControlConfig::from_json_str("{").expect_err("truncated");
    assert!(matches!(parse, ConfigError::Parse { .. }));
    let mut config = InputMapper::new(ProfileLibrary::default()).config();
    config.trim_throttle = f64::NAN;
    let err = config.validate().expect_err("NaN trim");
    assert!(err.to_string().contains("trim_throttle"));
}

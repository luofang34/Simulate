use super::*;

#[test]
fn every_profile_and_builtin_aerodrome_is_listed_once() {
    let registry = ScenarioRegistry::with_builtins();
    assert_eq!(
        registry.len(),
        ScenarioProfile::ALL.len() + AerodromeId::ALL.len()
    );
    for profile in ScenarioProfile::ALL {
        let listed: Vec<_> = registry
            .iter()
            .filter(|descriptor| descriptor.profile == Some(profile))
            .collect();
        assert_eq!(listed.len(), 1, "{profile:?}");
    }
    assert_eq!(
        registry.in_category(ScenarioCategory::VisualCorridor).len(),
        4
    );
    assert_eq!(registry.in_category(ScenarioCategory::FreeFlight).len(), 3);
    assert!(
        registry
            .in_category(ScenarioCategory::XilVerification)
            .is_empty()
    );
}

#[test]
fn search_reaches_tags_and_aerodromes_and_runtime_entries_join() {
    let mut registry = ScenarioRegistry::with_builtins();
    let ids = |registry: &ScenarioRegistry, query: &str| -> Vec<String> {
        registry
            .search(query)
            .iter()
            .map(|descriptor| descriptor.id.clone())
            .collect()
    };
    assert_eq!(
        ids(&registry, "kewr"),
        vec!["free-flight/KEWR", "profile/AirportKewrToweredPattern"]
    );
    assert_eq!(ids(&registry, "APOLLO").len(), 2);

    let rig = ScenarioDescriptor {
        id: "xil/betaflight-hover".to_owned(),
        title: "Betaflight Hover".to_owned(),
        category: ScenarioCategory::XilVerification,
        summary: "A quadcopter hover against flight-controller firmware.".to_owned(),
        recommended_vehicle: "quadcopter".to_owned(),
        aerodrome: None,
        tags: vec!["sitl".to_owned()],
        profile: None,
    };
    assert!(registry.register(rig.clone()).is_none());
    assert_eq!(registry.search("sitl"), vec![&rig]);
    assert_eq!(
        registry.in_category(ScenarioCategory::XilVerification),
        vec![&rig]
    );
}

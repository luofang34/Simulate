use gilrs::Button;
use indicate_instrument_glyphs::PANEL_GLYPHS;
use sim_input::Control;
use sim_scenery::{AerodromeId, OsmAerodromeDiagram};
use winit::keyboard::KeyCode;

use super::catalog::{AerodromeKind, HangarModel, VehicleLaunch};
use super::keymap::menu_input_for_code;
use super::scene::{Highlights, tessellate_hub};
use super::view::WidgetStyle;
use super::*;
use crate::controls::{CaptureTarget, ControlPreset, ControlScheme, VehicleClass};
use crate::gamepad::menu_input;
use crate::vehicles::FreeFlightVehicle;

const SIZE: [u32; 2] = [2560, 1440];

pub(super) fn catalog() -> HubCatalog {
    let diagrams: Vec<OsmAerodromeDiagram> = AerodromeId::ALL
        .into_iter()
        .map(|id| OsmAerodromeDiagram::builtin(id).expect("diagram"))
        .collect();
    HubCatalog::new(&diagrams)
}

pub(super) fn status() -> MenuStatus {
    MenuStatus {
        history_s: 40.0,
        scenario_active: true,
        sky: "LIVE UTC".to_owned(),
        ..MenuStatus::default()
    }
}

pub(super) struct Hub {
    pub(super) menu: EscMenu,
    pub(super) catalog: HubCatalog,
    pub(super) status: MenuStatus,
}

impl Hub {
    pub(super) fn opened() -> Self {
        let mut hub = Self {
            menu: EscMenu::default(),
            catalog: catalog(),
            status: status(),
        };
        assert_eq!(hub.input(MenuInput::Toggle), MenuOutcome::Opened);
        hub
    }

    pub(super) fn view(&self) -> HubView {
        HubView::build(&self.menu, &self.catalog, &self.status, SIZE)
    }

    pub(super) fn input(&mut self, input: MenuInput) -> MenuOutcome {
        let view = self.view();
        self.menu.handle(input, &view, &self.catalog)
    }

    pub(super) fn click(&mut self, id: WidgetId) -> MenuOutcome {
        let view = self.view();
        let [x, y, w, h] = view
            .widget_rect(id)
            .unwrap_or_else(|| panic!("{id:?} laid out"));
        let [dw, dh] = view.design();
        let px = [
            f64::from((x + w * 0.5) / dw) * f64::from(SIZE[0]),
            f64::from((y + h * 0.5) / dh) * f64::from(SIZE[1]),
        ];
        self.menu.hover_at(px, &view);
        assert_eq!(self.menu.hovered(), Some(id), "hover hits {id:?}");
        self.menu.click_at(px, &view, &self.catalog)
    }

    fn type_text(&mut self, text: &str) {
        for ch in text.chars() {
            assert_eq!(self.input(MenuInput::Char(ch)), MenuOutcome::Idle);
        }
    }
}

#[test]
fn five_tabs_switch_by_keys_function_keys_bumpers_and_clicks() {
    let mut hub = Hub::opened();
    assert_eq!(hub.menu.tab(), HubTab::Flight);
    for expected in [
        HubTab::Hangar,
        HubTab::World,
        HubTab::Scenarios,
        HubTab::Controls,
        HubTab::Flight,
    ] {
        assert_eq!(hub.input(MenuInput::NextTab), MenuOutcome::Idle);
        assert_eq!(hub.menu.tab(), expected);
    }
    hub.input(MenuInput::PrevTab);
    assert_eq!(hub.menu.tab(), HubTab::Controls);
    assert_eq!(
        menu_input_for_code(KeyCode::F3, false),
        Some(MenuInput::Tab(HubTab::World))
    );
    hub.input(MenuInput::Tab(HubTab::World));
    assert_eq!(hub.menu.tab(), HubTab::World);
    assert_eq!(hub.click(WidgetId::Tab(HubTab::Hangar)), MenuOutcome::Idle);
    assert_eq!(hub.menu.tab(), HubTab::Hangar);
    assert_eq!(
        menu_input(Button::RightTrigger, true),
        Some(MenuInput::NextTab)
    );
    assert_eq!(
        menu_input(Button::LeftTrigger, true),
        Some(MenuInput::PrevTab)
    );
    assert_eq!(
        menu_input(Button::RightTrigger, false),
        None,
        "bumpers fly when closed"
    );
    assert_eq!(menu_input(Button::Start, false), Some(MenuInput::Toggle));
    assert_eq!(hub.input(MenuInput::Back), MenuOutcome::Closed);
    assert!(!hub.menu.is_open());
}

#[test]
fn every_tab_tessellates_at_window_and_retina_sizes_with_only_panel_glyphs() {
    let mut hub = Hub::opened();
    for tab in HubTab::ALL {
        hub.input(MenuInput::Tab(tab));
        for size in [[1280, 720], SIZE, [800, 1000]] {
            let view = HubView::build(&hub.menu, &hub.catalog, &hub.status, size);
            assert!(view.widgets().len() > HubTab::ALL.len(), "{tab:?}");
            let texts = view
                .widgets()
                .iter()
                .map(|w| w.label.as_str())
                .chain(view.labels().iter().map(|l| l.text.as_str()));
            for text in texts {
                for ch in text.chars() {
                    assert!(
                        PANEL_GLYPHS.glyph(ch).is_ok(),
                        "{tab:?}: {ch:?} in {text:?}"
                    );
                }
            }
            let vertices = tessellate_hub(&view, Highlights::default(), size).expect("tessellates");
            assert!(!vertices.is_empty(), "{tab:?} {size:?}");
        }
    }
}

#[test]
fn flight_tab_offers_takeover_rewind_rate_and_layout() {
    let mut hub = Hub::opened();
    let view = hub.view();
    let takeover = view.find(WidgetId::TakeControl).expect("takeover button");
    assert_eq!(
        takeover.style,
        WidgetStyle::Attention,
        "highlighted during a scenario"
    );
    assert_eq!(
        hub.click(WidgetId::TakeControl),
        MenuOutcome::Command(MenuCommand::TakeManualControl)
    );
    assert_eq!(
        hub.click(WidgetId::Rewind(15)),
        MenuOutcome::Command(MenuCommand::Rewind { seconds: 15 })
    );
    assert_eq!(
        hub.click(WidgetId::Rate(RateChoice::Quad)),
        MenuOutcome::Command(MenuCommand::Rate(RateChoice::Quad))
    );
    assert_eq!(
        hub.click(WidgetId::Layout(AvionicsLayout::HudOnly)),
        MenuOutcome::Command(MenuCommand::Layout(AvionicsLayout::HudOnly))
    );
    assert_eq!(
        hub.click(WidgetId::Sky(Step::Next)),
        MenuOutcome::Command(MenuCommand::Sky(Step::Next))
    );
    hub.status.scenario_active = false;
    let calm = hub.view().find(WidgetId::TakeControl).map(|w| w.style);
    assert_eq!(calm, Some(WidgetStyle::Button));
    assert_eq!(hub.click(WidgetId::Resume), MenuOutcome::Closed);
}

#[test]
fn keyboard_focus_moves_spatially_and_confirm_activates() {
    let mut hub = Hub::opened();
    hub.input(MenuInput::Down);
    assert_eq!(hub.menu.focus(), Some(WidgetId::Resume));
    hub.input(MenuInput::Down);
    assert_eq!(hub.menu.focus(), Some(WidgetId::TakeControl));
    hub.input(MenuInput::Right);
    assert!(
        matches!(hub.menu.focus(), Some(WidgetId::Rate(_) | WidgetId::Sky(_))),
        "{:?}",
        hub.menu.focus()
    );
    hub.input(MenuInput::Left);
    hub.input(MenuInput::Down);
    assert_eq!(hub.menu.focus(), Some(WidgetId::Restart));
    assert_eq!(
        hub.input(MenuInput::Confirm),
        MenuOutcome::Command(MenuCommand::Restart)
    );
}

#[test]
fn hangar_selects_previews_and_flies_registry_vehicles() {
    let mut hub = Hub::opened();
    hub.input(MenuInput::Tab(HubTab::Hangar));
    let view = hub.view();
    let preview = view.preview_viewport().expect("live preview card");
    assert!(preview.x > 0.0 && preview.y > 0.0 && preview.x + preview.width < 1.0);
    assert!(view.preview_px().is_some_and(|[w, h]| w > 400 && h > 200));
    let x500 = hub
        .catalog
        .vehicles
        .iter()
        .position(|v| v.class == VehicleClass::Multirotor)
        .expect("x500");
    assert_eq!(
        hub.click(WidgetId::VehicleRow(x500)),
        MenuOutcome::Command(MenuCommand::PreviewVehicle(x500))
    );
    assert_eq!(hub.menu.vehicle(), x500);
    assert_eq!(
        hub.catalog.vehicles.get(x500).map(|v| v.default_preset()),
        Some(ControlPreset::QuadCamDrone)
    );
    hub.input(MenuInput::Down);
    assert_eq!(
        hub.menu.focus(),
        Some(WidgetId::VehicleRow(x500 + 1)),
        "lists step by row"
    );
    assert_eq!(
        hub.input(MenuInput::Confirm),
        MenuOutcome::Command(MenuCommand::PreviewVehicle(x500 + 1))
    );
    assert_eq!(
        hub.click(WidgetId::ApplyVehicle),
        MenuOutcome::Command(MenuCommand::ApplyVehicle {
            vehicle: x500 + 1,
            mode: None,
        })
    );
    assert_eq!(
        hub.click(WidgetId::AutoRotate),
        MenuOutcome::Command(MenuCommand::ToggleAutoRotate)
    );
    assert_eq!(
        hub.click(WidgetId::ResetView),
        MenuOutcome::Command(MenuCommand::ResetHangarView)
    );
    let lander = hub
        .catalog
        .vehicles
        .iter()
        .find(|v| v.class == VehicleClass::Spacecraft);
    assert!(lander.is_some_and(|v| v.modes().is_empty()
        && v.launch_for(None)
            == Some(VehicleLaunch::Scenario(
                sim_scenario::ScenarioProfile::Apollo11LunarLanding
            ))));
}

#[test]
fn hangar_lists_distinct_airframes_and_keeps_every_plant_reachable() {
    let catalog = catalog();
    let models: Vec<HangarModel> = catalog.vehicles.iter().map(|v| v.model).collect();
    assert_eq!(catalog.vehicles.len(), 6);
    assert!(models.contains(&HangarModel::P38g));
    assert!(models.contains(&HangarModel::Raketa));
    for (i, model) in models.iter().enumerate() {
        assert!(!models.iter().skip(i + 1).any(|m| m == model), "{models:?}");
    }
    for plant in [
        FreeFlightVehicle::JsbsimC172,
        FreeFlightVehicle::JsbsimP38g,
        FreeFlightVehicle::AviateFixedWingDirect,
        FreeFlightVehicle::Kinematic,
        FreeFlightVehicle::AviateX500Direct,
        FreeFlightVehicle::X500Mission,
    ] {
        let index = catalog.airframe_of(plant).expect("plant has an airframe");
        let airframe = catalog.vehicles.get(index).expect("airframe");
        assert!(airframe.launch == VehicleLaunch::Free(plant), "{plant:?}");
        assert_eq!(
            airframe.launch_for(Some(plant)),
            Some(VehicleLaunch::Free(plant))
        );
    }

    let mut hub = Hub::opened();
    hub.input(MenuInput::Tab(HubTab::Hangar));
    let p38 = hub
        .catalog
        .airframe_of(FreeFlightVehicle::JsbsimP38g)
        .expect("p38g");
    let c172 = hub
        .catalog
        .airframe_of(FreeFlightVehicle::JsbsimC172)
        .expect("c172");
    hub.click(WidgetId::VehicleRow(p38));
    assert!(
        hub.view()
            .find(WidgetId::VehicleMode(FreeFlightVehicle::JsbsimP38g))
            .is_none(),
        "single-plant airframes show no mode chips"
    );
    hub.status.active_vehicle = Some(FreeFlightVehicle::AviateFixedWingDirect);
    hub.menu
        .sync_vehicle(&hub.catalog, FreeFlightVehicle::AviateFixedWingDirect);
    assert_eq!(hub.menu.vehicle(), c172);
    let active_modes = |hub: &Hub| -> Vec<FreeFlightVehicle> {
        hub.view()
            .widgets()
            .iter()
            .filter_map(|w| match w.id {
                WidgetId::VehicleMode(m) if w.active => Some(m),
                _ => None,
            })
            .collect()
    };
    assert_eq!(
        active_modes(&hub),
        [FreeFlightVehicle::AviateFixedWingDirect]
    );
    assert_eq!(
        hub.click(WidgetId::VehicleMode(FreeFlightVehicle::JsbsimC172)),
        MenuOutcome::Idle
    );
    assert_eq!(active_modes(&hub), [FreeFlightVehicle::JsbsimC172]);
    assert_eq!(
        hub.click(WidgetId::ApplyVehicle),
        MenuOutcome::Command(MenuCommand::ApplyVehicle {
            vehicle: c172,
            mode: Some(FreeFlightVehicle::JsbsimC172),
        })
    );
}

#[test]
fn aerodrome_search_runway_end_and_spawn_mode_build_the_spawn() {
    let mut hub = Hub::opened();
    hub.input(MenuInput::Tab(HubTab::World));
    hub.type_text("kewr");
    assert_eq!(hub.menu.search(), "KEWR");
    let kewr = hub.menu.aerodrome();
    let entry = hub.catalog.aerodromes.get(kewr).expect("entry").clone();
    assert_eq!(entry.kind, AerodromeKind::Osm(AerodromeId::NewarkKewr));
    assert!(entry.runways.len() >= 2, "{:?}", entry.runways);
    let rows: Vec<WidgetId> = hub
        .view()
        .widgets()
        .iter()
        .map(|w| w.id)
        .filter(|id| matches!(id, WidgetId::AerodromeRow(_)))
        .collect();
    assert_eq!(
        rows,
        [WidgetId::AerodromeRow(kewr)],
        "the filter leaves one row"
    );
    hub.click(WidgetId::RunwayEnd(1));
    hub.click(WidgetId::SpawnChoice(SpawnMode::ShortFinal));
    assert_eq!(
        hub.click(WidgetId::SpawnHere),
        MenuOutcome::Command(MenuCommand::Spawn {
            aerodrome: kewr,
            runway: 1,
            mode: SpawnMode::ShortFinal,
        })
    );
    assert_eq!(
        hub.view()
            .find(WidgetId::SpawnChoice(SpawnMode::Ramp))
            .is_some(),
        !entry.stands.is_empty()
    );

    for _ in 0..4 {
        hub.input(MenuInput::Backspace);
    }
    hub.type_text("tranq");
    let moon = hub
        .catalog
        .aerodromes
        .get(hub.menu.aerodrome())
        .map(|a| a.kind);
    assert_eq!(moon, Some(AerodromeKind::Tranquility));
    assert_eq!(hub.menu.runway(), 0, "a new aerodrome resets the runway");
    assert!(matches!(
        hub.click(WidgetId::SpawnHere),
        MenuOutcome::Command(MenuCommand::Spawn {
            mode: SpawnMode::Overhead,
            ..
        })
    ));
    hub.type_text("zzz");
    assert!(hub.catalog.filter_aerodromes(hub.menu.search()).is_empty());
    hub.input(MenuInput::Scroll(5));
    assert_eq!(hub.menu.aerodrome_scroll(), 0, "nothing to scroll");
}

#[test]
fn scenarios_launch_from_the_categorized_browser() {
    let mut hub = Hub::opened();
    hub.input(MenuInput::Tab(HubTab::Scenarios));
    let kewr = hub
        .catalog
        .scenarios
        .iter()
        .position(|s| s.profile == Some(sim_scenario::ScenarioProfile::AirportKewrToweredPattern))
        .expect("kewr pattern");
    hub.click(WidgetId::ScenarioRow(kewr));
    assert_eq!(
        hub.click(WidgetId::Launch),
        MenuOutcome::Command(MenuCommand::LaunchScenario(kewr))
    );
    let first = hub.catalog.scenarios.first().expect("free flight first");
    assert_eq!(first.profile, None);
    hub.input(MenuInput::Scroll(-3));
    assert_eq!(hub.menu.scenario_scroll(), 0);
}

#[test]
fn controls_tab_steps_presets_and_starts_rebinds() {
    let mut hub = Hub::opened();
    let mut scheme = ControlScheme::new(VehicleClass::FixedWing);
    hub.input(MenuInput::Tab(HubTab::Controls));
    hub.status.controls.rows = scheme.axis_rows();
    hub.status.controls.keys = crate::controls::KeyAction::ALL
        .into_iter()
        .map(|a| (a, crate::controls::key_label(scheme.keys().key_for(a))))
        .collect();
    let commands = [
        WidgetId::Preset(Step::Next),
        WidgetId::AxisRebind(Control::Yaw),
        WidgetId::AxisInvert(Control::Pitch),
        WidgetId::Deadzone(Control::Roll, Step::Next),
        WidgetId::Expo(Control::Roll, Step::Prev),
        WidgetId::KeyRebind(crate::controls::KeyAction::Brakes),
    ];
    for id in commands {
        let MenuOutcome::Command(command) = hub.click(id) else {
            panic!("{id:?} issues a command");
        };
        match command {
            MenuCommand::Preset(step) => {
                scheme.step_preset(step == Step::Next);
            }
            MenuCommand::RebindAxis(c) => scheme.begin_capture(CaptureTarget::Axis(c)),
            MenuCommand::InvertAxis(c) => scheme.toggle_invert(c),
            MenuCommand::Deadzone(c, s) => scheme.step_deadzone(c, s == Step::Next),
            MenuCommand::Expo(c, s) => scheme.step_expo(c, s == Step::Next),
            MenuCommand::RebindKey(a) => scheme.begin_capture(CaptureTarget::Key(a)),
            other => panic!("unexpected {other:?}"),
        }
    }
    assert_eq!(scheme.preset(), ControlPreset::FixedWingMode2);
    assert_eq!(
        scheme.capture(),
        Some(CaptureTarget::Key(crate::controls::KeyAction::Brakes))
    );
    assert!(scheme.capture_key(KeyCode::KeyN));
    hub.status.controls.capture = Some(CaptureTarget::Axis(Control::Yaw));
    let capturing = hub
        .view()
        .find(WidgetId::AxisRebind(Control::Yaw))
        .map(|w| w.label.clone());
    assert_eq!(capturing.as_deref(), Some("MOVE AXIS"));
    assert_eq!(
        hub.click(WidgetId::PresetAuto),
        MenuOutcome::Command(MenuCommand::PresetAuto)
    );
}

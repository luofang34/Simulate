use std::sync::Arc;

use sim_maplibre::{EfbTracks, FixId, FlightPlan, RouteFix, RouteProgress};

use super::gesture::{Gestures, PointerInput};
use super::{PageMapLink, PageMapView};
use crate::map_rig::EfbCommand;

fn all(gestures: &mut Gestures, inputs: &[PointerInput], ratio: f64) -> Vec<EfbCommand> {
    inputs
        .iter()
        .flat_map(|input| gestures.handle(*input, ratio))
        .collect()
}

#[test]
fn a_drag_pans_the_map_by_buffer_pixels_from_where_it_began() {
    let mut gestures = Gestures::default();
    let commands = all(
        &mut gestures,
        &[
            PointerInput::Down {
                id: 1,
                at: [100.0, 100.0],
                tilts: false,
            },
            PointerInput::Move {
                id: 1,
                at: [103.0, 100.0],
            },
            PointerInput::Move {
                id: 1,
                at: [120.0, 90.0],
            },
            PointerInput::Move {
                id: 1,
                at: [130.0, 90.0],
            },
            PointerInput::Up {
                id: 1,
                at: [130.0, 90.0],
            },
        ],
        2.0,
    );
    assert_eq!(
        commands,
        [
            EfbCommand::PanPixels([-40.0, 20.0]),
            EfbCommand::PanPixels([-20.0, 0.0]),
        ],
        "the first pan covers the tap's reach too; no pick after a drag"
    );
}

#[test]
fn a_tap_picks_at_its_buffer_pixel() {
    let mut gestures = Gestures::default();
    let commands = all(
        &mut gestures,
        &[
            PointerInput::Down {
                id: 7,
                at: [50.0, 40.0],
                tilts: false,
            },
            PointerInput::Move {
                id: 7,
                at: [52.0, 41.0],
            },
            PointerInput::Up {
                id: 7,
                at: [52.0, 41.0],
            },
        ],
        2.0,
    );
    assert_eq!(commands, [EfbCommand::Pick([104.0, 82.0])]);
}

#[test]
fn spreading_two_fingers_zooms_in_and_turning_them_rotates() {
    let mut gestures = Gestures::default();
    let commands = all(
        &mut gestures,
        &[
            PointerInput::Down {
                id: 1,
                at: [100.0, 100.0],
                tilts: false,
            },
            PointerInput::Down {
                id: 2,
                at: [200.0, 100.0],
                tilts: false,
            },
            PointerInput::Move {
                id: 2,
                at: [300.0, 100.0],
            },
        ],
        1.0,
    );
    assert_eq!(commands, [EfbCommand::ZoomBy(1.0)]);
    // Second finger moves up around the first: counter-clockwise on screen.
    let turned = gestures.handle(
        PointerInput::Move {
            id: 2,
            at: [100.0, -100.0],
        },
        1.0,
    );
    let rotate = turned.iter().find_map(|c| match c {
        EfbCommand::RotateBy(deg) => Some(*deg),
        _ => None,
    });
    assert!(
        rotate.is_some_and(|deg| (deg - 90.0).abs() < 1e-9),
        "{turned:?}"
    );
    let lifted = all(
        &mut gestures,
        &[
            PointerInput::Up {
                id: 2,
                at: [100.0, -100.0],
            },
            PointerInput::Up {
                id: 1,
                at: [100.0, 100.0],
            },
        ],
        1.0,
    );
    assert!(lifted.is_empty(), "a pinch never picks");
}

#[test]
fn a_cancelled_pointer_never_picks() {
    let mut gestures = Gestures::default();
    let commands = all(
        &mut gestures,
        &[
            PointerInput::Down {
                id: 3,
                at: [10.0, 10.0],
                tilts: false,
            },
            PointerInput::Cancel { id: 3 },
            PointerInput::Up {
                id: 3,
                at: [10.0, 10.0],
            },
        ],
        1.0,
    );
    assert!(commands.is_empty(), "{commands:?}");
}

#[test]
fn the_wheel_zooms_out_scrolling_down_and_at_most_a_level_a_notch() {
    let mut gestures = Gestures::default();
    assert_eq!(
        gestures.handle(PointerInput::Wheel { delta_y: 150.0 }, 2.0),
        [EfbCommand::ZoomBy(-0.5)]
    );
    assert_eq!(
        gestures.handle(PointerInput::Wheel { delta_y: -3000.0 }, 2.0),
        [EfbCommand::ZoomBy(1.0)]
    );
    assert!(
        gestures
            .handle(PointerInput::Wheel { delta_y: f64::NAN }, 2.0)
            .is_empty()
    );
}

#[test]
fn the_link_hands_each_change_over_once() {
    let link = PageMapLink::default();
    let plan = FlightPlan {
        origin: None,
        fixes: vec![RouteFix {
            id: FixId(0),
            ident: "ZA1".to_owned(),
            lat_deg: 41.7,
            lon_deg: -1.05,
            altitude_m: Some(2000.0),
        }],
    };
    let progress = RouteProgress::after_sequenced(None);
    link.command(EfbCommand::Recenter);
    link.publish_route(&plan, progress);
    link.publish_tracks(Arc::new(EfbTracks::default()));
    link.resize([800, 496]);
    let first = link.take();
    assert_eq!(first.size_px, Some([800, 496]));
    assert!(
        first.visible && link.in_sight(),
        "in sight until the page says otherwise"
    );
    assert_eq!(first.commands, [EfbCommand::Recenter]);
    assert_eq!(first.route, Some((plan.clone(), progress)));
    assert!(first.tracks.is_some());
    link.publish_route(&plan, progress);
    let again = link.take();
    assert!(again.commands.is_empty() && again.route.is_none() && again.tracks.is_none());
    assert_eq!(again.size_px, None);
    link.set_in_sight(false);
    assert!(!link.take().visible && !link.in_sight());
    link.show(PageMapView {
        live: true,
        ..PageMapView::default()
    });
    assert!(link.view().live);
}

#[test]
fn the_colour_of_ownships_set_stays_with_the_link_once_the_page_said() {
    let link = PageMapLink::default();
    assert_eq!(link.take().ownship_colour, None);
    link.set_ownship_colour([0x39, 0x87, 0xe5]);
    // The app takes its input every frame; the colour is the page's standing word.
    assert_eq!(link.take().ownship_colour, Some([0x39, 0x87, 0xe5]));
    assert_eq!(link.take().ownship_colour, Some([0x39, 0x87, 0xe5]));
}

#[test]
fn a_tilting_drag_tilts_with_up_and_down_and_turns_with_sideways() {
    let mut gestures = Gestures::default();
    let commands = all(
        &mut gestures,
        &[
            PointerInput::Down {
                id: 1,
                at: [100.0, 100.0],
                tilts: true,
            },
            PointerInput::Move {
                id: 1,
                at: [100.0, 80.0],
            },
            PointerInput::Move {
                id: 1,
                at: [110.0, 80.0],
            },
            PointerInput::Up {
                id: 1,
                at: [110.0, 80.0],
            },
        ],
        2.0,
    );
    assert_eq!(
        commands,
        [EfbCommand::PitchBy(8.0), EfbCommand::RotateBy(-3.0)],
        "the drag leaves the tap's reach in one move: up 20 px tilts 8°, then 10 px right turns"
    );
}

#[test]
fn two_fingers_moved_up_together_tilt_and_spread_apart_zoom() {
    let mut gestures = Gestures::default();
    let together = all(
        &mut gestures,
        &[
            PointerInput::Down {
                id: 1,
                at: [100.0, 200.0],
                tilts: false,
            },
            PointerInput::Down {
                id: 2,
                at: [200.0, 200.0],
                tilts: false,
            },
            PointerInput::Move {
                id: 1,
                at: [100.0, 190.0],
            },
            PointerInput::Move {
                id: 2,
                at: [200.0, 190.0],
            },
            PointerInput::Move {
                id: 1,
                at: [100.0, 170.0],
            },
        ],
        1.0,
    );
    assert!(
        together
            .iter()
            .all(|c| matches!(c, EfbCommand::PitchBy(d) if *d > 0.0))
            && !together.is_empty(),
        "{together:?}"
    );
    let mut gestures = Gestures::default();
    let spread = all(
        &mut gestures,
        &[
            PointerInput::Down {
                id: 1,
                at: [100.0, 200.0],
                tilts: false,
            },
            PointerInput::Down {
                id: 2,
                at: [200.0, 200.0],
                tilts: false,
            },
            PointerInput::Move {
                id: 2,
                at: [300.0, 200.0],
            },
        ],
        1.0,
    );
    assert_eq!(spread, [EfbCommand::ZoomBy(1.0)]);
}

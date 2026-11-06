//! The PFD synthetic-vision sky: aviation blue above the horizon at every heading and
//! altitude, divided from the ground along the banked horizon.

use std::time::Duration;

use glam::{DQuat, DVec3, EulerRot};
use sim_core::{CameraMount, SimInstant};

use super::{instrument_maps::svs_spec, sample_scenery_and_runtime, try_host_gpu};
use crate::{
    AvionicsMapSet, ConformalMapFrame, conformal_stamp_from_vehicle, symmetric_frustum,
    tests::sample_vehicle_at,
};

const W: usize = 120;
const H: usize = 90;

/// Renders the SVS of a vehicle at `height_m` with NED attitude `heading`, `pitch`, `roll`
/// (degrees); returns tight RGBA rows.
fn render(maps: &mut AvionicsMapSet, height_m: f64, [heading, pitch, roll]: [f64; 3]) -> Vec<u8> {
    let ned_body = DQuat::from_euler(
        EulerRot::ZYX,
        heading.to_radians(),
        pitch.to_radians(),
        roll.to_radians(),
    );
    let veh = sample_vehicle_at(39.0, -75.5, height_m, ned_body);
    let svs = maps.get_mut("pfd_svs").expect("svs");
    assert_eq!(svs.size(), [W as u32, H as u32]);
    let stamp = conformal_stamp_from_vehicle(
        SimInstant::START,
        &veh,
        &CameraMount::forward_nose(DVec3::ZERO),
        DVec3::ZERO,
        symmetric_frustum(1.0, W as u32, H as u32, 0.5, 250_000.0).expect("frustum"),
        0.0,
    )
    .expect("stamp");
    svs.invalidate();
    svs.render_conformal(ConformalMapFrame {
        timestamp: Duration::from_millis(16),
        stamp,
        ownship_msl_m: height_m as f32,
        runway: None,
        entities: &[],
    })
    .expect("svs frame");
    svs.read_rgba().expect("rgba")
}

fn svs_maps() -> Option<AvionicsMapSet> {
    let gpu = try_host_gpu()?;
    let (scenery, _, runtime) = sample_scenery_and_runtime(gpu);
    let mut maps = AvionicsMapSet::new(&runtime, vec![svs_spec()]).expect("svs set");
    let _supplied = maps.supply_dem_synchronously(&scenery).expect("dem");
    Some(maps)
}

fn pixel(rgba: &[u8], x: usize, y: usize) -> [u8; 4] {
    let at = (y * W + x) * 4;
    [rgba[at], rgba[at + 1], rgba[at + 2], rgba[at + 3]]
}

fn is_aviation_blue([r, _, b, a]: [u8; 4]) -> bool {
    a == 255 && b > 90 && b > r.saturating_add(40)
}

fn upper_quarter(rgba: &[u8]) -> &[u8] {
    &rgba[..W * (H / 4) * 4]
}

fn assert_upper_quarter_blue(rgba: &[u8], what: &str) {
    for (index, px) in upper_quarter(rgba).as_chunks::<4>().0.iter().enumerate() {
        assert!(
            is_aviation_blue(*px),
            "{what}: non-blue sky {px:?} at ({}, {})",
            index % W,
            index / W
        );
    }
}

#[test]
fn level_flight_shows_rich_aviation_blue_above_the_horizon() {
    let Some(mut maps) = svs_maps() else {
        return;
    };
    let rgba = render(&mut maps, 450.0, [90.0, 0.0, 0.0]);
    assert_upper_quarter_blue(&rgba, "level flight");
    // The style's `sky-color` #0c3b75, untinted by any scattering, fills the top of the frame.
    let top = pixel(&rgba, W / 2, 0);
    let sky_color = [0x0c, 0x3b, 0x75];
    assert!(
        top.iter().zip(sky_color).all(|(a, e)| a.abs_diff(e) <= 8),
        "top of frame {top:?} is the style sky {sky_color:?}"
    );
    let ground = pixel(&rgba, W / 2, H - 4);
    assert!(
        !is_aviation_blue(ground) && ground[3] == 255,
        "terrain below the horizon: {ground:?}"
    );
}

#[test]
fn the_sky_does_not_change_with_heading() {
    let Some(mut maps) = svs_maps() else {
        return;
    };
    let south_east = render(&mut maps, 450.0, [135.0, 0.0, 0.0]);
    let north_west = render(&mut maps, 450.0, [315.0, 0.0, 0.0]);
    let worst = upper_quarter(&south_east)
        .iter()
        .zip(upper_quarter(&north_west))
        .map(|(a, b)| a.abs_diff(*b))
        .max();
    assert!(worst <= Some(1), "upper sky differs by {worst:?} LSB");
    assert_upper_quarter_blue(&south_east, "heading 135");
}

#[test]
fn near_space_keeps_the_sky_blue() {
    let Some(mut maps) = svs_maps() else {
        return;
    };
    let low = render(&mut maps, 450.0, [90.0, 0.0, 0.0]);
    let high = render(&mut maps, 28_000.0, [90.0, 0.0, 0.0]);
    assert_upper_quarter_blue(&low, "450 m");
    assert_upper_quarter_blue(&high, "28 km");
}

/// Rolled 60° right, the horizon through the frame centre tilts so the left of the frame is
/// sky and the right is ground, and the split follows the banked horizon's normal.
#[test]
fn a_sixty_degree_bank_divides_sky_and_ground_along_the_banked_horizon() {
    let Some(mut maps) = svs_maps() else {
        return;
    };
    let rgba = render(&mut maps, 450.0, [90.0, 0.0, 60.0]);
    let (cx, cy) = ((W / 2) as f64, (H / 2) as f64);
    let (sin, cos) = 60_f64.to_radians().sin_cos();
    let at = |along: f64, across: f64| {
        // `across` is along the banked horizon's sky-ward normal, image y down.
        let x = cx + along * cos - across * sin;
        let y = cy + along * sin - across * cos;
        pixel(&rgba, x.round() as usize, y.round() as usize)
    };
    for along in [-20.0, 0.0, 20.0] {
        let sky = at(along, 18.0);
        let ground = at(along, -18.0);
        assert!(is_aviation_blue(sky), "sky side at {along}: {sky:?}");
        assert!(
            !is_aviation_blue(ground) && ground[3] == 255,
            "ground side at {along}: {ground:?}"
        );
    }
    let level = render(&mut maps, 450.0, [90.0, 0.0, 0.0]);
    let (left, right) = (pixel(&rgba, 10, H / 2), pixel(&rgba, W - 11, H / 2));
    assert!(
        is_aviation_blue(left),
        "left of a right bank is sky: {left:?}"
    );
    assert!(
        !is_aviation_blue(right),
        "right of a right bank is ground: {right:?}"
    );
    assert_ne!(level, rgba, "the bank changes the picture");
}

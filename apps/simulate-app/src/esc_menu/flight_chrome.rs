//! In-flight vector chrome drawn over the cockpit while the hub is closed: the scenario
//! takeover banner, the EFB's bezel, header, and buttons, and the spaceflight avionics
//! overlay (`FDAI / Navball`, 3D `Orrery`, `AerospikePlumeVisual`, DSN link, and spacecraft
//! bus telemetry).
//!
//! The EFB map itself is a compositor quad under this overlay, so nothing here fills the
//! map's rectangle: the expanded tablet's bezel is four strips around it.

mod efb_paint;

use indicate_instrument_scene::{Anchor, HAlign, LayerId, PaintMode, Rgba8, SceneWriter, VAlign};
use sim_render::{FdaiMarkerDir, VectorVertex};

use super::scene::{SceneResult, centre, paint_banner, tessellate};
use super::view::{Palette, glyph_safe};
use crate::cockpit_rig::efb_chrome::EfbChrome;
use crate::error::AppSimError;
use crate::flight::SpaceHudChrome;
use efb_paint::paint_efb;

const CHROME_BYTES: usize = 192 * 1024;

/// Draws the optional takeover `banner`, EFB `chrome`, and spaceflight `space` HUD for a
/// `surface_px` window.
pub fn tessellate_flight_chrome(
    banner: Option<&str>,
    chrome: Option<&EfbChrome>,
    space: Option<&SpaceHudChrome>,
    surface_px: [u32; 2],
) -> Result<Vec<VectorVertex>, AppSimError> {
    let [sw, sh] = surface_px.map(|v| v.max(1) as f32);
    let design = chrome.map_or([720.0 * sw / sh, 720.0], |c| c.design);
    let [dw, _] = design;
    let mut buf = vec![0_u8; CHROME_BYTES];
    let mut w = SceneWriter::new(&mut buf)?;
    w.begin_layer(LayerId::Background)?;
    if let Some(chrome) = chrome {
        paint_efb(&mut w, chrome)?;
    }
    if let Some(space) = space {
        paint_space_chrome(&mut w, space, dw)?;
    }
    if let Some(text) = banner {
        paint_banner(&mut w, text, dw)?;
    }
    w.end_layer(LayerId::Background)?;
    let len = w.finish();
    tessellate(buf.get(..len).unwrap_or_default(), design, surface_px)
}

fn paint_space_chrome(
    w: &mut SceneWriter<'_>,
    space: &SpaceHudChrome,
    design_w: f32,
) -> SceneResult {
    paint_space_fdai_widget(w, space)?;
    paint_space_orrery_widget(w, space, design_w)?;
    paint_space_bus_and_plume_widget(w, space)
}

fn paint_space_fdai_widget(w: &mut SceneWriter<'_>, space: &SpaceHudChrome) -> SceneResult {
    let (x, y, cw, ch) = (18.0, 522.0, 184.0, 180.0);
    w.fill_color(Palette::PANEL)?;
    w.stroke(Palette::EDGE, 1.5)?;
    w.rect(PaintMode::FillStroke, x, y, cw, ch)?;
    let left = Anchor {
        h: HAlign::Left,
        v: VAlign::Middle,
    };
    w.fill_color(Palette::TITLE)?;
    w.text(x + 10.0, y + 14.0, 9.5, left, "FDAI NAVBALL")?;
    w.fill_color(Palette::VALUE)?;
    w.text(x + cw - 52.0, y + 14.0, 9.0, left, space.frame_label)?;

    let (cx, cy, r) = (x + cw * 0.5, y + 100.0, 62.0);
    w.fill_color(Palette::STUDIO)?;
    w.stroke(Palette::EDGE, 1.5)?;
    w.circle(PaintMode::FillStroke, cx, cy, r)?;

    let up_b = space.fdai.q_ref_to_body * glam::DVec3::Z;
    let roll_angle = (up_b.x as f32).atan2(up_b.z as f32);
    let (hx, hy) = (roll_angle.cos() * (r - 4.0), roll_angle.sin() * (r - 4.0));
    w.stroke(Palette::HINT, 1.2)?;
    w.line(cx - hx, cy - hy, cx + hx, cy + hy)?;
    w.line(cx - 8.0, cy, cx + 8.0, cy)?;
    w.line(cx, cy - 8.0, cx, cy + 8.0)?;

    for (marker, label, ink) in [
        (space.fdai.prograde, "PRO", Palette::READOUT),
        (space.fdai.retrograde, "RET", Palette::WARN),
        (space.fdai.radial_out, "RAD", Palette::VALUE),
        (space.fdai.normal, "NRM", Palette::FOCUS),
        (space.fdai.maneuver_burn, "BRN", Palette::ATTENTION),
    ] {
        paint_fdai_marker(w, [cx, cy, r - 8.0], marker, label, ink)?;
    }
    Ok(())
}

fn paint_fdai_marker(
    w: &mut SceneWriter<'_>,
    [cx, cy, scale]: [f32; 3],
    marker: FdaiMarkerDir,
    label: &str,
    ink: Rgba8,
) -> SceneResult {
    let [dx, dy] = marker.disk_xy;
    let px = cx + dx.clamp(-0.92, 0.92) * scale;
    let py = cy - dy.clamp(-0.92, 0.92) * scale;
    w.fill_color(ink)?;
    w.stroke(ink, 1.4)?;
    let mode = if marker.front_hemisphere {
        PaintMode::FillStroke
    } else {
        PaintMode::Stroke
    };
    w.circle(mode, px, py, 4.2)?;
    w.fill_color(Palette::TITLE)?;
    w.text(px, py - 8.0, 7.5, centre(), label)
}

fn paint_space_orrery_widget(
    w: &mut SceneWriter<'_>,
    space: &SpaceHudChrome,
    design_w: f32,
) -> SceneResult {
    let (cw, ch) = (204.0, 180.0);
    let (x, y) = (design_w - cw - 18.0, 522.0);
    w.fill_color(Palette::PANEL)?;
    w.stroke(Palette::EDGE, 1.5)?;
    w.rect(PaintMode::FillStroke, x, y, cw, ch)?;
    let left = Anchor {
        h: HAlign::Left,
        v: VAlign::Middle,
    };
    w.fill_color(Palette::TITLE)?;
    w.text(x + 10.0, y + 14.0, 9.5, left, "3D ORRERY")?;
    if let Some(dsn) = space.dsn_link {
        let tag = if dsn.accessible {
            format!("DSN {:.1}S", dsn.light_time_s)
        } else {
            "DSN OCC".to_owned()
        };
        let safe = glyph_safe(&tag, 16);
        w.fill_color(if dsn.accessible {
            Palette::READOUT
        } else {
            Palette::WARN
        })?;
        w.text(x + cw - 82.0, y + 14.0, 8.5, left, &safe)?;
    }

    let (cx, cy, r) = (x + cw * 0.5, y + 100.0, 62.0);
    w.fill_color(Palette::STUDIO)?;
    w.stroke(Palette::EDGE, 1.2)?;
    w.circle(PaintMode::FillStroke, cx, cy, r)?;
    w.stroke(Palette::TRACK, 1.0)?;
    w.circle(PaintMode::Stroke, cx, cy, r * 0.5)?;

    for node in &space.orrery {
        let [nx_ndc, ny_ndc] = node.ndc_xy;
        let px = cx + nx_ndc.clamp(-0.86, 0.86) * (r - 6.0);
        let py = cy - ny_ndc.clamp(-0.86, 0.86) * (r - 6.0);
        let (label, ink, dot_r) = match node.id {
            10 => ("SUN", Palette::WARN, 4.5),
            399 => ("EAR", Palette::VALUE, 4.0),
            301 => ("MON", Palette::LABEL, 3.0),
            499 => ("MAR", Palette::ATTENTION, 3.5),
            1 => ("OWN", Palette::READOUT, 3.8),
            2 => ("L2", Palette::FOCUS, 3.5),
            _ => ("OBJ", Palette::HINT, 2.5),
        };
        w.fill_color(ink)?;
        w.stroke(ink, 1.2)?;
        w.circle(PaintMode::FillStroke, px, py, dot_r)?;
        w.fill_color(Palette::TITLE)?;
        w.text(px, py - 7.5, 7.5, centre(), label)?;
    }
    Ok(())
}

fn paint_space_bus_and_plume_widget(
    w: &mut SceneWriter<'_>,
    space: &SpaceHudChrome,
) -> SceneResult {
    let (x, y, cw, ch) = (18.0, 18.0, 268.0, 128.0);
    w.fill_color(Palette::PANEL)?;
    w.stroke(Palette::EDGE, 1.5)?;
    w.rect(PaintMode::FillStroke, x, y, cw, ch)?;
    let left = Anchor {
        h: HAlign::Left,
        v: VAlign::Middle,
    };
    w.fill_color(Palette::TITLE)?;
    w.text(x + 10.0, y + 14.0, 9.5, left, "BUS / RELATIVITY / PLUME")?;

    let bus = space.ownship_bus.or(space.jwst_bus);
    let (pwr_w, tau_us, rate_us_d) = bus.map_or((0.0, 0.0, 0.0), |b| {
        (
            b.solar_power_w,
            b.oscillator_tau_minus_tdb_s * 1.0e6,
            b.clock_rate_offset * 86_400.0 * 1.0e6,
        )
    });
    let line1 = glyph_safe(&format!("ARRAY {:5.0} W", pwr_w), 24);
    let line2 = glyph_safe(&format!("DTAU {:+7.2} US", tau_us), 24);
    let line3 = glyph_safe(&format!("RATE {:+6.1} US/D", rate_us_d), 24);
    w.fill_color(Palette::READOUT)?;
    w.text(x + 10.0, y + 38.0, 8.8, left, &line1)?;
    w.fill_color(Palette::VALUE)?;
    w.text(x + 10.0, y + 58.0, 8.8, left, &line2)?;
    w.fill_color(Palette::LABEL)?;
    w.text(x + 10.0, y + 78.0, 8.8, left, &line3)?;
    if space.epoch_clamped {
        w.fill_color(Palette::WARN)?;
        w.text(x + 10.0, y + 104.0, 8.2, left, "EPOCH CLAMPED TO DE440")?;
    } else {
        w.fill_color(Palette::HINT)?;
        w.text(x + 10.0, y + 104.0, 8.2, left, "DE440 EPHEMERIS NOMINAL")?;
    }

    paint_aerospike_plume_box(w, [x + 182.0, y + 28.0, 74.0, 88.0], space)
}

fn paint_aerospike_plume_box(
    w: &mut SceneWriter<'_>,
    [bx, by, bw, bh]: [f32; 4],
    space: &SpaceHudChrome,
) -> SceneResult {
    w.fill_color(Palette::STUDIO)?;
    w.stroke(Palette::EDGE, 1.0)?;
    w.rect(PaintMode::FillStroke, bx, by, bw, bh)?;
    let cx = bx + bw * 0.5;
    let nozzle_y = by + 18.0;
    w.fill_color(Palette::BUTTON)?;
    w.stroke(Palette::TITLE, 1.2)?;
    w.polygon(
        PaintMode::FillStroke,
        &[
            [cx - 10.0, by + 6.0],
            [cx + 10.0, by + 6.0],
            [cx + 15.0, nozzle_y],
            [cx - 15.0, nozzle_y],
        ],
    )?;
    let plume = space.plume;
    let len_px = if plume.visible {
        (plume.plume_length_m * 4.2).clamp(16.0, 56.0)
    } else {
        14.0
    };
    let half_w = (8.0 * plume.expansion_ratio.clamp(1.0, 3.5)).clamp(8.0, 28.0);
    let ink = if plume.visible {
        let [cr, cg, cb] = plume.core_emissive_rgb;
        Rgba8::rgba(
            (cr * 48.0).clamp(120.0, 255.0) as u8,
            (cg * 42.0).clamp(160.0, 255.0) as u8,
            (cb * 38.0).clamp(190.0, 255.0) as u8,
            220,
        )
    } else {
        Palette::TRACK
    };
    w.fill_color(ink)?;
    w.stroke(Palette::FOCUS, 1.2)?;
    w.polygon(
        PaintMode::FillStroke,
        &[
            [cx - 11.0, nozzle_y],
            [cx + 11.0, nozzle_y],
            [cx + half_w, nozzle_y + len_px],
            [cx, nozzle_y + len_px + 6.0],
            [cx - half_w, nozzle_y + len_px],
        ],
    )?;
    if plume.visible && plume.shock_diamond_spacing_m > 0.05 {
        w.fill_color(Palette::TITLE)?;
        w.circle(PaintMode::Fill, cx, nozzle_y + len_px * 0.45, 3.2)?;
    }
    Ok(())
}

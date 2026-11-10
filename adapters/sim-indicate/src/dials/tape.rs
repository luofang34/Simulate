//! The card a tape drum carries round it, such as a heading card behind a
//! directional gyro's window, baked once into rows that a square atlas cell
//! holds: the whole period read left to right, row after row.

use indicate_instrument_analog::TapeSense;
use indicate_instrument_descriptor::PanelDrawError;
use indicate_instrument_glyphs::FaceTable;
use indicate_instrument_scene::{Anchor, LayerId, PaintMode, SceneError, SceneWriter};

use super::DialSource;
use super::bake::{FaceBake, bake, lighting};
use crate::PanelError;

/// A baked tape card: `rows` rows, each `1 / rows` of the period.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TapeBake {
    /// The image, `rows` bands top to bottom.
    pub image: FaceBake,
    /// How many rows the period is cut into.
    pub rows: u32,
}

/// `source`'s tape card, `side` pixels wide, lettered in `faces`, laid out
/// as the drum carries it: card position `p` of period `P` sits in row
/// `floor(p R / P)`, left to right, where `p` is the value on a card rising
/// to the right and `P - v` on one rising to the left, so the lettering
/// reads upright either way. `None` when the dial has no tape.
pub fn bake_tape_strip(
    source: &DialSource,
    side: u32,
    faces: &FaceTable<'_>,
) -> Result<Option<TapeBake>, PanelError> {
    let Some(tape) = source.builtin.and_then(|b| b.tapes.first()) else {
        return Ok(None);
    };
    let [_, _, _, window_h] = tape.aperture;
    let length = tape.pitch * tape.period;
    if !(length > 0.0 && window_h > 0.0) {
        return Ok(None);
    }
    // Rows fill the cell when rows² × window height ≈ the card's length.
    let rows = ((length / window_h).sqrt().floor() as u32).max(1);
    let px_per_unit = side as f32 * rows as f32 / length;
    let row_h = (window_h * px_per_unit).round().max(1.0);
    let height = (row_h as u32 * rows).min(side);
    let span = tape.period / rows as f32;
    let slot = source.numerals_slot();
    let image = bake(side, height, faces, |mode, w| {
        let ink = mode.paint(tape.paint).under(lighting(mode));
        let ground = mode
            .paint(indicate_instrument_analog::Paint::plain(tape.drum))
            .under(lighting(mode));
        let draw = |w: &mut SceneWriter<'_>| -> Result<(), SceneError> {
            w.begin_layer(LayerId::Background)?;
            w.fill_color(ground)?;
            w.rect(PaintMode::Fill, 0.0, 0.0, side as f32, height as f32)?;
            w.end_layer(LayerId::Background)?;
            w.begin_layer(LayerId::Tapes)?;
            w.font(slot)?;
            for row in 0..rows {
                let top = row as f32 * row_h;
                let start = row as f32 * span;
                // The value's x in this row, for each of its period copies.
                let xs = |value: f32| {
                    let position = match tape.sense {
                        TapeSense::RightIncreasing => value,
                        TapeSense::LeftIncreasing => tape.period - value,
                    };
                    [-tape.period, 0.0, tape.period]
                        .map(|shift| (position + shift - start) * tape.pitch * px_per_unit)
                };
                for ticks in tape.ticks {
                    w.stroke(ink, ticks.width * px_per_unit)?;
                    let count = (tape.period / ticks.every).round() as u32;
                    for k in 0..count {
                        for x in xs(k as f32 * ticks.every) {
                            if (-row_h..side as f32 + row_h).contains(&x) {
                                w.line(x, top + ticks.top * row_h, x, top + ticks.bottom * row_h)?;
                            }
                        }
                    }
                }
                w.fill_color(ink)?;
                let size = tape.numeral_size * px_per_unit;
                for numeral in tape.numerals {
                    for x in xs(numeral.value) {
                        if (-2.0 * size..side as f32 + 2.0 * size).contains(&x) {
                            let y = top + tape.numeral_at * row_h;
                            w.text(x, y, size, Anchor::CENTER, numeral.text)?;
                        }
                    }
                }
            }
            w.end_layer(LayerId::Tapes)
        };
        draw(w).map_err(PanelDrawError::from)
    })?;
    Ok(Some(TapeBake { image, rows }))
}

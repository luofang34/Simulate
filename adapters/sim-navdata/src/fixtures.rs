//! A tile source in memory for the tests: every imagery tile distinct,
//! every elevation tile a flat Terrarium PNG, with a count of the fetches.

use std::cell::{Cell, RefCell};
use std::collections::BTreeMap;
use std::io::Cursor;

use image::{ImageFormat, Rgb, RgbImage};
use navigate_data::PlannedCamera;

use crate::{HttpAnswer, TileTransport, planned_camera};

/// The reference route: Zaragoza's closed land circuit at 2000 m above
/// mean sea level.
pub const CIRCUIT: [[f64; 3]; 5] = [
    [41.7, -1.05, 2000.0],
    [41.7, -0.8, 2000.0],
    [41.8, -0.8, 2000.0],
    [41.8, -1.0, 2000.0],
    [41.74, -1.05, 2000.0],
];

/// The camera of the desktop onboard package.
pub fn camera() -> PlannedCamera {
    planned_camera(400, 300, 67.4, 90.0)
}

/// A Terrarium PNG of 8 by 8 pixels, every pixel `height_m` above mean sea
/// level.
pub fn terrarium_png(height_m: f64) -> Vec<u8> {
    let scaled = height_m + 32_768.0;
    let pixel = Rgb([
        (scaled / 256.0).floor() as u8,
        (scaled % 256.0).floor() as u8,
        ((scaled.fract()) * 256.0).floor() as u8,
    ]);
    let mut png = Vec::new();
    RgbImage::from_pixel(8, 8, pixel)
        .write_to(&mut Cursor::new(&mut png), ImageFormat::Png)
        .unwrap();
    png
}

/// Answers every address with a tile, except the addresses told otherwise.
pub struct FakeSource {
    /// The ground height of every elevation tile, metres.
    pub ground_m: f64,
    /// Bytes of each imagery tile.
    pub imagery_bytes: usize,
    /// Statuses of the addresses that are not a tile, by a text their URL
    /// contains.
    pub statuses: BTreeMap<&'static str, u16>,
    /// Requests still to fail before the source answers: a dropped
    /// connection each.
    pub drops: Cell<usize>,
    /// Every URL asked, in order.
    pub asked: RefCell<Vec<String>>,
}

impl FakeSource {
    pub fn new(ground_m: f64) -> Self {
        Self {
            ground_m,
            imagery_bytes: 1_000,
            statuses: BTreeMap::new(),
            drops: Cell::new(0),
            asked: RefCell::new(Vec::new()),
        }
    }

    pub fn asked(&self) -> usize {
        self.asked.borrow().len()
    }
}

impl TileTransport for FakeSource {
    async fn get(&self, url: &str) -> Result<HttpAnswer, String> {
        self.asked.borrow_mut().push(url.to_owned());
        if self.drops.get() > 0 {
            self.drops.set(self.drops.get() - 1);
            return Err("connection dropped".to_owned());
        }
        if let Some((_, status)) = self.statuses.iter().find(|(part, _)| url.contains(**part)) {
            return match status {
                0 => Err("connection refused".to_owned()),
                status => Ok(HttpAnswer {
                    status: *status,
                    body: Vec::new(),
                }),
            };
        }
        let body = if url.contains("terrarium") {
            terrarium_png(self.ground_m)
        } else {
            let mut body = url.as_bytes().to_vec();
            body.resize(self.imagery_bytes, 0x5a);
            body
        };
        Ok(HttpAnswer { status: 200, body })
    }
}

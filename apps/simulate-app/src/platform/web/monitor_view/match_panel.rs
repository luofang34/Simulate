//! What the visual navigator matched: the reference view it compared the
//! camera frame against and the frame as matched, side by side, with every
//! correspondence drawn between them (green kept by the fit, red refused),
//! captioned with what the navigator made of the frame and how far its
//! offered fix lay from the truth.

use sim_link_host::{ABSENT, GreyImage, MatchShot, Monitor, PlanLine, clock_words, outcome_words};
use sim_navigate::NavMeans;
use wasm_bindgen::{Clamped, JsCast};
use web_sys::{CanvasRenderingContext2d, Document, Element, HtmlCanvasElement, ImageData};

use super::super::section::{append, write_lines};

/// Room between the two images, canvas pixels.
const GAP_PX: f64 = 12.0;
const INLIER: &str = "#3fbf6f";
const OUTLIER: &str = "#e05b4c";
/// Metres per degree of latitude, near enough for a distance of metres.
const METRES_PER_DEGREE: f64 = 111_320.0;

/// What the panel says the pictures are.
const ABOUT: &str = "What the visual navigator matched: left, the reference view it \
    rendered or stored from its onboard map; right, the camera frame as it matched it. \
    Lines join each correspondence: green kept by the fit, red refused.";

/// The match panel.
pub struct MatchPanel {
    document: Document,
    root: Element,
    canvas: HtmlCanvasElement,
    context: CanvasRenderingContext2d,
    caption: Element,
    shown: Option<(f64, f64)>,
}

impl MatchPanel {
    /// A panel at the end of `parent`, hidden until a match comes.
    pub fn under(document: &Document, parent: &Element) -> Option<Self> {
        let root = append(document, parent, "div", "match-panel")?;
        root.set_attribute("hidden", "").ok();
        if let Some(about) = append(document, &root, "p", "detail") {
            about.set_text_content(Some(ABOUT));
        }
        let canvas: HtmlCanvasElement = append(document, &root, "canvas", "match")?
            .dyn_into()
            .ok()?;
        canvas
            .set_attribute(
                "aria-label",
                "The navigator's reference view and the camera frame with their matches",
            )
            .ok();
        let context = canvas.get_context("2d").ok()??.dyn_into().ok()?;
        let caption = append(document, &root, "dl", "lines live-rows")?;
        Some(Self {
            document: document.clone(),
            root,
            canvas,
            context,
            caption,
            shown: None,
        })
    }

    /// Shows the first visual set's latest comparison, when it or the set's
    /// latest frame changed.
    pub fn draw(&mut self, monitor: &Monitor) {
        let Some(set) = monitor
            .traces()
            .iter()
            .find(|trace| trace.means.contains(&NavMeans::Visual))
            .map(|trace| trace.name.as_str())
        else {
            return;
        };
        let log = monitor.matches();
        let (Some(compared), Some(latest)) = (log.compared(set), log.latest(set)) else {
            return;
        };
        let key = (compared.at_s, latest.at_s);
        if self.shown == Some(key) {
            return;
        }
        self.paint(compared);
        write_lines(
            &self.document,
            &self.caption,
            &caption(monitor, compared, latest),
        );
        self.root.remove_attribute("hidden").ok();
        self.shown = Some(key);
    }

    /// The two images side by side at their own size, and the lines.
    fn paint(&self, shot: &MatchShot) {
        let (Some(reference), Some(frame)) = (&shot.reference, &shot.frame) else {
            return;
        };
        let offset = f64::from(reference.width) + GAP_PX;
        let width = offset + f64::from(frame.width);
        let height = f64::from(reference.height.max(frame.height));
        self.canvas.set_width(width as u32);
        self.canvas.set_height(height as u32);
        let context = &self.context;
        context.set_fill_style_str("#0b0e17");
        context.fill_rect(0.0, 0.0, width, height);
        put(context, reference, 0.0);
        put(context, frame, offset);
        context.set_line_width(1.0);
        for line in &shot.lines {
            context.set_stroke_style_str(if line.inlier { INLIER } else { OUTLIER });
            context.begin_path();
            context.move_to(line.reference_px[0], line.reference_px[1]);
            context.line_to(line.frame_px[0] + offset, line.frame_px[1]);
            context.stroke();
        }
    }
}

/// Puts `image` on `context` with its left edge at `x`.
fn put(context: &CanvasRenderingContext2d, image: &GreyImage, x: f64) {
    let rgba: Vec<u8> = image
        .pixels
        .iter()
        .flat_map(|&grey| [grey, grey, grey, 255])
        .collect();
    if let Ok(data) =
        ImageData::new_with_u8_clamped_array_and_sh(Clamped(&rgba), image.width, image.height)
    {
        context.put_image_data(&data, x, 0.0).ok();
    }
}

/// The caption of `compared`, and of `latest` when a later frame gave no
/// comparison: always the same four lines, a dash where there is nothing.
fn caption(monitor: &Monitor, compared: &MatchShot, latest: &MatchShot) -> Vec<PlanLine> {
    let line = |label: &str, text: String| PlanLine {
        label: label.to_owned(),
        text,
    };
    let source = match compared.path.as_deref() {
        Some("keyframe") => "keyframe",
        Some(_) => "rendered view",
        None => ABSENT,
    };
    let kept = compared.lines.iter().filter(|line| line.inlier).count();
    let matches = match compared.solved {
        Some(fit) => format!(
            "{kept} of {} kept · {:.1} px · {} cells",
            compared.lines.len(),
            fit.reprojection_rms_px,
            fit.spatial_support
        ),
        None => format!("{} found · no pose", compared.lines.len()),
    };
    let offered = compared.fix.map_or_else(
        || ABSENT.to_owned(),
        |fix| match truth_at(monitor, compared.at_s) {
            Some(truth) => format!(
                "{:.1} m off truth · σ {:.1} m",
                metres_between(fix.at_deg, truth),
                fix.sigma_horizontal_m
            ),
            None => format!("σ {:.1} m · no truth then", fix.sigma_horizontal_m),
        },
    );
    let since = if latest.at_s > compared.at_s {
        format!(
            "{} · {} · no images",
            clock_words(latest.at_s),
            outcome_words(&latest.outcome)
        )
    } else {
        ABSENT.to_owned()
    };
    vec![
        line(
            "Frame",
            format!(
                "{} · {source} · {}",
                clock_words(compared.at_s),
                outcome_words(&compared.outcome)
            ),
        ),
        line("Matches", matches),
        line("Offered fix", offered),
        line("Since", since),
    ]
}

/// TRUTH: the aircraft's latitude and longitude at the sample nearest
/// `at_s`, degrees.
fn truth_at(monitor: &Monitor, at_s: f64) -> Option<(f64, f64)> {
    monitor
        .truth()
        .iter()
        .min_by(|a, b| (a.0 - at_s).abs().total_cmp(&(b.0 - at_s).abs()))
        .map(|(_, at)| *at)
}

/// Horizontal metres between two latitude and longitude pairs, degrees.
fn metres_between(a: (f64, f64), b: (f64, f64)) -> f64 {
    let north = (a.0 - b.0) * METRES_PER_DEGREE;
    let east = (a.1 - b.1) * METRES_PER_DEGREE * (0.5 * (a.0 + b.0)).to_radians().cos();
    north.hypot(east)
}

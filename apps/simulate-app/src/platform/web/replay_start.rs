//! A development build's stand-in for the flight: a recorded native
//! trial's progress stream, fetched from beside the page and flown again
//! through the monitor, labelled as what it is.

use sim_link_host::{PlanLine, Replay};
use sim_navdata::TileTransport;
use sim_navdata::browser::BrowserTransport;
use tracing::{info, warn};

use super::flight_run::fly;
use super::monitor_view::MonitorView;
use super::section::Section;

/// The recorded stream, beside the page.
const RECORDING: &str = "progress-replay.ndjson";
/// The pace a replay starts at, route seconds per wall second.
const REPLAY_SPEED: f64 = 16.0;
/// What heads the status line, so no one takes the replay for the link's
/// flight.
const NOTE: &str = "Replay of a recorded native trial, not the flight of this link. ";

/// Fetches the recording and flies it in `section`.
pub async fn fly_recorded(section: Section) {
    let text = match BrowserTransport.get(RECORDING).await {
        Ok(answer) if answer.status == 200 => String::from_utf8(answer.body).ok(),
        Ok(answer) => {
            warn!(status = answer.status, "no recorded run beside the page");
            None
        }
        Err(error) => {
            warn!(%error, "no recorded run beside the page");
            None
        }
    };
    let Some(text) = text else {
        section.refuse(
            "No recorded run to replay",
            &[line(&format!(
                "this development build replays {RECORDING} from beside the page, and there is none"
            ))],
        );
        return;
    };
    let replay = match Replay::from_lines(&text) {
        Ok(replay) => replay,
        Err(error) => {
            section.refuse(
                "The recorded run cannot be read",
                &[line(&error.to_string())],
            );
            return;
        }
    };
    info!(records = replay.len(), "replaying a recorded run");
    if let Some(view) = MonitorView::inside(section, Some(REPLAY_SPEED), false) {
        fly(view, replay, NOTE).await;
    }
}

fn line(text: &str) -> PlanLine {
    PlanLine {
        label: "Reason".to_owned(),
        text: text.to_owned(),
    }
}

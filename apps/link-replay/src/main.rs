//! `link-replay` command-line entrypoint.

use std::io::Write;
use std::process::ExitCode;

fn main() -> ExitCode {
    #[cfg(not(target_arch = "wasm32"))]
    {
        tracing_subscriber::fmt()
            .with_writer(std::io::stderr)
            .with_env_filter(
                tracing_subscriber::EnvFilter::try_from_default_env()
                    .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
            )
            .init();
        match run_cli() {
            Ok(()) => ExitCode::SUCCESS,
            Err(err) => {
                drop(writeln!(std::io::stderr(), "error: {err}"));
                ExitCode::FAILURE
            }
        }
    }
    #[cfg(target_arch = "wasm32")]
    {
        drop(writeln!(
            std::io::stderr(),
            "link-replay requires a native target"
        ));
        ExitCode::FAILURE
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn run_cli() -> Result<(), link_replay::ReplayError> {
    let args = link_replay::cli::parse_args(std::env::args().skip(1))?;
    link_replay::runner::run_blocking(&args).map(|_| ())
}

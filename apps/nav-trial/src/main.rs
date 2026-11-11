//! `nav-trial`: see [`nav_trial::cli`] for the command line.

fn main() -> Result<(), nav_trial::NavTrialError> {
    // Logs go to standard error: standard output may carry the progress
    // stream.
    #[cfg(not(target_arch = "wasm32"))]
    tracing_subscriber::fmt()
        .with_writer(std::io::stderr)
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();
    let args = nav_trial::cli::parse_args(std::env::args().skip(1))?;
    nav_trial::run::run_blocking(&args).map(|_| ())
}

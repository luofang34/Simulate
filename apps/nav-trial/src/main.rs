//! `nav-trial --request <request.json> --out <result.json>`.

fn main() -> Result<(), nav_trial::NavTrialError> {
    #[cfg(not(target_arch = "wasm32"))]
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();
    let args = nav_trial::cli::parse_args(std::env::args().skip(1))?;
    nav_trial::cli::run_files_blocking(&args.request, &args.out).map(|_| ())
}

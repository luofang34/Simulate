//! `nav-trial prepare …` and `nav-trial fly --request <request.json> --out <result.json>`.

#[cfg(not(target_arch = "wasm32"))]
fn main() -> Result<(), nav_trial::NavTrialError> {
    use nav_trial::cli::{CliCommand, parse_args, run_files_blocking};
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();
    match parse_args(std::env::args().skip(1))? {
        CliCommand::Prepare {
            request,
            onboard,
            world,
            cache,
        } => {
            let prepared = nav_trial::prepare::prepare_files_blocking(
                &request,
                &onboard,
                &world,
                cache.as_deref(),
            )?;
            tracing::info!(seconds = prepared.seconds, "prepared");
        }
        CliCommand::Fly { request, out } => {
            run_files_blocking(&request, &out)?;
        }
    }
    Ok(())
}

#[cfg(target_arch = "wasm32")]
fn main() {}

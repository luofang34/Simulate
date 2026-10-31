//! Headless XIL ironbird binary entry point.

#[cfg(not(target_arch = "wasm32"))]
fn main() -> Result<(), ironbird::IronbirdError> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();
    let args = ironbird::cli::parse_args(std::env::args().skip(1))?;
    let _ = ironbird::cli::execute_cli(&args)?;
    Ok(())
}

#[cfg(target_arch = "wasm32")]
fn main() {}

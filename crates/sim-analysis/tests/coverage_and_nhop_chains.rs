//! Integration tests for geodesic H3 coverage grids and N-hop causal light-time chains.

#[cfg(test)]
#[path = "coverage_and_nhop_chains/chain_tests.rs"]
mod chain_tests;
#[cfg(test)]
#[path = "coverage_and_nhop_chains/chain_window_tests.rs"]
mod chain_window_tests;
#[cfg(test)]
#[path = "coverage_and_nhop_chains/coverage_tests.rs"]
mod coverage_tests;
#[cfg(test)]
#[path = "coverage_and_nhop_chains/fixtures.rs"]
mod fixtures;

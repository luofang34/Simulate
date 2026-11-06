#!/usr/bin/env bash
# Local CI in gate order. Every gate must pass before a push.
set -euo pipefail
cd "$(dirname "$0")/.."

gate() {
    echo "==> $*"
    "$@"
}

gate cargo fmt --check
gate cargo clippy --workspace --all-targets -- -D warnings
gate cargo clippy --workspace --target wasm32-unknown-unknown -- -D warnings
gate cargo check -p sim-scenario --no-default-features
gate cargo test --workspace --all-targets
RUSTDOCFLAGS="-D missing_docs -D rustdoc::broken_intra_doc_links" gate cargo doc --workspace --no-deps
gate cargo build --workspace --release
(cd apps/simulate-app && gate trunk build --release)
for check in scripts/check-*.sh; do
    gate "$check"
done
echo "==> all gates passed"

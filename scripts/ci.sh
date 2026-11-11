#!/usr/bin/env bash
# Local CI in gate order. Every gate must pass before a push. Every cargo step
# is `--locked`: a lock that does not match Cargo.toml is a failure, never an
# update.
set -euo pipefail
cd "$(dirname "$0")/.."

gate() {
    echo "==> $*"
    "$@"
}

gate cargo fmt --check
gate scripts/check-sibling-sources.sh
gate cargo clippy --workspace --all-targets --locked -- -D warnings
gate cargo clippy --workspace --target wasm32-unknown-unknown --locked -- -D warnings
gate cargo check -p sim-scenario --no-default-features --locked
gate cargo test --workspace --all-targets --locked
RUSTDOCFLAGS="-D missing_docs -D rustdoc::broken_intra_doc_links" gate cargo doc --workspace --no-deps --locked
gate cargo build --workspace --release --locked
(cd apps/simulate-app && gate trunk build --release --locked)
if command -v mdbook >/dev/null 2>&1; then
    gate mdbook build book
fi
for check in scripts/check-*.sh; do
    gate "$check"
done
gate scripts/use-local-siblings.test.sh
echo "==> all gates passed"

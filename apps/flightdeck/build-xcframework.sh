#!/usr/bin/env bash
# Builds the `flightdeck-ffi` Rust static library for macOS (arm64), iOS/iPadOS device,
# iOS/iPadOS Simulator, visionOS device, and visionOS Simulator, then packages
# `target/flightdeck-ffi/FlightDeckFFI.xcframework`.
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/../.." && pwd)"
ffi_dir="$repo_root/apps/flightdeck/ffi"
out_dir="$repo_root/target/flightdeck-ffi"
xcf_path="$out_dir/FlightDeckFFI.xcframework"

cleanup() {
    rm -f "$ffi_dir/Cargo.lock"
}
trap cleanup EXIT

cp "$repo_root/Cargo.lock" "$ffi_dir/Cargo.lock"

targets=(
    aarch64-apple-darwin
    aarch64-apple-ios
    aarch64-apple-ios-sim
    aarch64-apple-visionos
    aarch64-apple-visionos-sim
)

rustup target add "${targets[@]}"

for triple in "${targets[@]}"; do
    echo "==> Building flightdeck-ffi for $triple"
    cargo build \
        --manifest-path "$ffi_dir/Cargo.toml" \
        --target "$triple" \
        --target-dir "$out_dir" \
        --release
done

rm -rf "$xcf_path"
echo "==> Packaging $xcf_path"
xcodebuild -create-xcframework \
    -library "$out_dir/aarch64-apple-darwin/release/libflightdeck_ffi.a" \
    -library "$out_dir/aarch64-apple-ios/release/libflightdeck_ffi.a" \
    -library "$out_dir/aarch64-apple-ios-sim/release/libflightdeck_ffi.a" \
    -library "$out_dir/aarch64-apple-visionos/release/libflightdeck_ffi.a" \
    -library "$out_dir/aarch64-apple-visionos-sim/release/libflightdeck_ffi.a" \
    -output "$xcf_path"

echo "==> Ready: $xcf_path"

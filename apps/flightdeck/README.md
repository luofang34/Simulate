# Sokoly FlightDeck (`FlightDeckPad` & `FlightDeckXR`) — Multi-Body EFB, GCS & Synthetic Vision

`FlightDeckXR` is an Apple Vision Pro (`visionOS` 2.0) Mixed-Reality Synthetic Vision System (`SVS`) application built on top of `Simulate`, `avionics-link`, `world` / `Flightbag`, and `Indicate` / `IndicateAppleDisplay`.

## Architecture

1. **Rust Adapter (`adapters/sim-flightdeck`)**:
   - **Ownship GNSS, AHRS & ADS-B (`avionics-link` + GDL-90)**:
     - Decodes Garmin Connext framed streams (`AccessoryLink`, `TelemetryDecoder`, `TelemetryClock`) and replays captured hardware flights (`real-session.tsv` at `47N` Central Jersey Regional), preserving honest missing-sensor semantics (`IAS` and `VSI` stay `None` when absent on the bus).
     - Decodes FAA GDL-90 (`0x0A` Ownship, `0x0B` Geo Alt, `0x14` Traffic, `0x65` ForeFlight AHRS) over UDP `:4000` with verified CRC-CCITT (`0x8BB3`).
   - **3D Airspace & Traffic Surveillance (`world` / `Flightbag`)**:
     - Extrudes 3D airspace boundaries/curtains (`Class B/C/D/E`, `SUA`, `TFR`) from `PMTiles` v3 / MVT 2.1 vector tiles and the NY/NJ terminal catalog, fused with `world-notices` live operational TFR/SUA activations and `world-geomag` WMM2025 magnetic variation.
     - Fuses multi-source ADS-B traffic via `world-surveillance` (`TrafficFuser`) and computes CPA threat tiers (`OTHER`, `PROX`, `TA`, `RA`) with callsign, relative altitude (`±hundreds ft`), and vertical trend (`↑`/`↓`).
   - **Flight Instruments (`Indicate`)**:
     - Drives 3D unit-sphere Apple Vision Pro HWD symbology (`indicate-instrument-hmd` with `35°/28°` off-axis glance hysteresis), conformal 2D HUD (`indicate-instrument-hud`), PFD & HSI (`indicate-instrument-panels`), and 3D Volumetric Terrain (`indicate-instrument-volumes`).
2. **C-ABI Bridge (`apps/flightdeck/ffi`)**:
   - Compiles `libflightdeck_ffi.a` into `target/flightdeck-ffi/FlightDeckFFI.xcframework` (`aarch64-apple-visionos`, `aarch64-apple-visionos-sim`, and `aarch64-apple-darwin`).
3. **Shared Swift Package (`apps/flightdeck/swift/FlightDeckKit`)**:
   - Cross-platform (`visionOS 2.0`, `iOS 17.0`, `macOS 14.0`) controller, wire decoder (`FDKF`/`FDKG`), TCP/UDP transports, `IndicateAppleDisplay` producers, and `RealityKit` conformal 3D scene builder (`SvsRealityScene`).
   - Keeps iPadOS UI cleanly **deferred** while sharing 100% of the Rust/Swift core when an iPad SwiftUI target is added later.
4. **Apple Vision Pro App (`apps/flightdeck/App`)**:
   - **Digital Crown Virtual / Reality Blend**: Binds the hardware Digital Crown (`ProgressiveImmersionStyle(immersion: 0.10...0.85)` + `.onImmersionChange`) and cockpit rotary knob (`RotaryBlendKnob`) to `CrownBlendConfig`, strictly enforcing **`>= 15%` real-world outside passthrough** (`virtualTerrainOpacity <= 0.85`) at all times.

## Build & Verify

From the repository root:

```bash
# 1. Build multi-arch FlightDeckFFI.xcframework (macOS + visionOS + visionOS Simulator)
./apps/flightdeck/build-xcframework.sh

# 2. Run FlightDeckKit end-to-end Swift tests (FFI + Connext replay + RealityKit + Indicate)
swift test --package-path apps/flightdeck/swift/FlightDeckKit --scratch-path target/swift-build

# 3. Generate Xcode project and build FlightDeckXR for Apple Vision Pro Simulator
(cd apps/flightdeck/App && xcodegen generate)
xcodebuild \
  -project apps/flightdeck/App/FlightDeckXR.xcodeproj \
  -scheme FlightDeckXR \
  -destination 'generic/platform=visionOS Simulator' \
  -derivedDataPath target/xcode-derived \
  CODE_SIGNING_ALLOWED=NO \
  build
```

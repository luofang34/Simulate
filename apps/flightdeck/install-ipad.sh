#!/usr/bin/env bash
set -euo pipefail

DEVICE_UDID="${1:-00008132-001139CA2685001C}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
XCF_DIR="${REPO_ROOT}/target/flightdeck-ffi/FlightDeckFFI.xcframework"
RES_DIR="${REPO_ROOT}/target/ipad-bundle-resources"
APP_BUNDLE="${REPO_ROOT}/target/ipad-derived/Build/Products/Release-iphoneos/FlightDeckPad.app"
BUNDLE_ID="com.sokolysystems.visionsvs.pad"
FLIGHTBAG_DIST="/Users/fangluo/Flightbag/dist/cycles/2026-10-29"
FLIGHTBAG_NASR_ZIP="/Users/fangluo/Flightbag/data/cache/2026-10-29/28DaySubscription_Effective_2026-10-29.zip"
FLIGHTBAG_STYLE="/Users/fangluo/Flightbag/styles/airspace_vfr_sectional.json"

if [[ ! -d "${XCF_DIR}" ]]; then
  echo "==> Building FlightDeckFFI.xcframework..."
  "${REPO_ROOT}/apps/flightdeck/build-xcframework.sh"
fi

mkdir -p "${RES_DIR}"
if [[ -d "${FLIGHTBAG_DIST}" ]]; then
  for f in airspace.pmtiles vfr_sectional.pmtiles vfr_terminal.pmtiles ifr_low.pmtiles ifr_high.pmtiles cifp_procedures.bin dtpp_plates.bin; do
    if [[ ( ! -f "${RES_DIR}/${f}" || "${FLIGHTBAG_DIST}/${f}" -nt "${RES_DIR}/${f}" ) && -f "${FLIGHTBAG_DIST}/${f}" ]]; then
      echo "==> Staging offline pack ${f}..."
      cp -f "${FLIGHTBAG_DIST}/${f}" "${RES_DIR}/${f}"
    fi
  done
fi
if [[ ! -f "${RES_DIR}/nasr_waypoints.bin" && -f "${FLIGHTBAG_NASR_ZIP}" ]]; then
  echo "==> Building offline NASR waypoint index..."
  python3 "${REPO_ROOT}/apps/flightdeck/scripts/build-nasr-waypoints.py" "${FLIGHTBAG_NASR_ZIP}" "${RES_DIR}/nasr_waypoints.bin"
fi
if [[ -d "${FLIGHTBAG_DIST}" ]]; then
  [[ -f "${FLIGHTBAG_DIST}/manifest.json" ]] && cp -f "${FLIGHTBAG_DIST}/manifest.json" "${RES_DIR}/flightbag_nasr_manifest.json"
  [[ -f "${FLIGHTBAG_DIST}/cifp_faa_cifp.json" ]] && cp -f "${FLIGHTBAG_DIST}/cifp_faa_cifp.json" "${RES_DIR}/cifp_faa_cifp.json"
  [[ -f "${FLIGHTBAG_DIST}/dtpp_faa_dtpp.json" ]] && cp -f "${FLIGHTBAG_DIST}/dtpp_faa_dtpp.json" "${RES_DIR}/dtpp_faa_dtpp.json"
  [[ -f "${FLIGHTBAG_DIST}/chart_faa_vfr_sectional.json" ]] && cp -f "${FLIGHTBAG_DIST}/chart_faa_vfr_sectional.json" "${RES_DIR}/chart_faa_vfr_sectional.json"
  [[ -f "${FLIGHTBAG_DIST}/chart_faa_vfr_terminal.json" ]] && cp -f "${FLIGHTBAG_DIST}/chart_faa_vfr_terminal.json" "${RES_DIR}/chart_faa_vfr_terminal.json"
  [[ -f "${FLIGHTBAG_DIST}/chart_faa_ifr_low.json" ]] && cp -f "${FLIGHTBAG_DIST}/chart_faa_ifr_low.json" "${RES_DIR}/chart_faa_ifr_low.json"
  [[ -f "${FLIGHTBAG_DIST}/chart_faa_ifr_high.json" ]] && cp -f "${FLIGHTBAG_DIST}/chart_faa_ifr_high.json" "${RES_DIR}/chart_faa_ifr_high.json"
fi
cp -f "${FLIGHTBAG_STYLE}" "${RES_DIR}/airspace_vfr_sectional.json"
chmod 644 "${RES_DIR}"/*

echo "==> Building signed Release FlightDeckPad.app..."
(
  cd "${REPO_ROOT}/apps/flightdeck/App"
  xcodegen generate
  xcodebuild \
    -project FlightDeck.xcodeproj \
    -scheme FlightDeckPad \
    -configuration Release \
    -destination 'generic/platform=iOS' \
    -derivedDataPath "${REPO_ROOT}/target/ipad-derived" \
    -allowProvisioningUpdates \
    build
)

echo "==> Installing FlightDeckPad.app onto device ${DEVICE_UDID}..."
xcrun devicectl device install app --device "${DEVICE_UDID}" "${APP_BUNDLE}"

echo "==> Launching ${BUNDLE_ID} on device ${DEVICE_UDID}..."
xcrun devicectl --timeout 8 device process launch --terminate-existing --device "${DEVICE_UDID}" "${BUNDLE_ID}" || echo "App installed; unlock iPad screen to launch interactively."

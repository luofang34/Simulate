import Foundation
import SwiftUI
import FlightDeckKit

/// Entry point for the iPadOS EFB + `Sokoly` GCS + 3D Synthetic Vision & Multi-Body Globe app.
@main
struct FlightDeckPadApp: App {
    /// Explicitly declared reference body and vertical datum at launch (never assumed implicitly).
    @State private var controller = FlightDeckController(body: .earthWgs84Egm96)
    @State private var primaryMode: PadPrimaryViewMode = .map
    @State private var svsSubMode: SvsSubMode = .fpv
    @State private var svsLookOffset = SvsLookOffset()
    @State private var followMode: MapFollowMode = .free
    @State private var northLocked: Bool = true
    @State private var layers = PadLayerConfiguration()
    @State private var orbit: OrbitCameraState = .wholeGlobeDefault
    /// Preference for conformal HUD inside `SVS -> FPV` (never displayed in `Map` or `Chase`).
    @State private var showHudOverlay: Bool = true
    @State private var isLayersPanelExpanded: Bool = false
    @State private var layersSubmenu: LayersSubmenuPage?
    @State private var isFplSheetPresented: Bool = false
    @State private var isReceiverSheetPresented: Bool = false
    @State private var isDownloadsSheetPresented: Bool = false
    @State private var presentedPlateRequest: PresentedPlateRequest?
    @State private var bannerText: String?
    @State private var fplInitialQuery: String = ""
    @State private var didApplyLaunchOptions: Bool = false

    var body: some Scene {
        WindowGroup {
            PadCockpitView(
                controller: controller,
                primaryMode: $primaryMode,
                svsSubMode: $svsSubMode,
                svsLookOffset: $svsLookOffset,
                followMode: $followMode,
                northLocked: $northLocked,
                layers: $layers,
                orbit: $orbit,
                showHudOverlay: $showHudOverlay,
                isLayersPanelExpanded: $isLayersPanelExpanded,
                layersSubmenu: $layersSubmenu,
                isFplSheetPresented: $isFplSheetPresented,
                isReceiverSheetPresented: $isReceiverSheetPresented,
                isDownloadsSheetPresented: $isDownloadsSheetPresented,
                presentedPlateRequest: $presentedPlateRequest,
                bannerText: $bannerText,
                fplInitialQuery: fplInitialQuery
            )
            .onAppear {
                guard !didApplyLaunchOptions else { return }
                didApplyLaunchOptions = true
                applyCommandLineLaunchOptionsIfPresent()
            }
            .onOpenURL { incomingUrl in
                openTrackPreview(from: incomingUrl)
            }
        }
    }

    /// Opens an AirDrop / Open-In `.kml` or `.gpx` file directly into 6-DoF Preview mode.
    @MainActor
    private func openTrackPreview(from url: URL) {
        if controller.loadKmlFile(at: url) {
            controller.seekKml(fraction: 0.0)
            if (controller.latestFrame?.ownship.groundSpeedKt ?? 0.0) < 35.0,
               controller.kmlPlayback.durationSeconds > 120.0 {
                controller.seekKml(fraction: 0.06)
            }
            controller.setKmlPlayback(paused: false, speedMultiplier: 2.0)
            svsLookOffset.reset()
            primaryMode = .svs
            svsSubMode = .fpv
            showHudOverlay = true

            let callsign = controller.kmlPlayback.callsign.isEmpty
                ? url.deletingPathExtension().lastPathComponent
                : controller.kmlPlayback.callsign
            let route = controller.kmlPlayback.route.isEmpty
                ? ""
                : " · \(controller.kmlPlayback.route)"
            scheduleBanner(
                "PREVIEW · \(callsign)\(route) · \(controller.kmlPlayback.pointCount) pts (\(controller.celestialBody.badgeLabel))"
            )
        } else {
            scheduleBanner("Unable to preview track file: \(url.lastPathComponent)")
        }
    }

    @MainActor
    private func applyCommandLineLaunchOptionsIfPresent() {
        let args = ProcessInfo.processInfo.arguments
        guard args.count > 1 else { return }

        var idx = 1
        while idx < args.count {
            let flag = args[idx]
            switch flag {
            case "--body" where idx + 1 < args.count:
                let val = args[idx + 1].lowercased()
                idx += 1
                switch val {
                case "earth", "earth_msl":
                    controller.setCelestialBody(.earthWgs84Egm96)
                    orbit = .wholeGlobeDefault
                case "earth_hae":
                    controller.setCelestialBody(.earthWgs84Ellipsoid)
                    orbit = .wholeGlobeDefault
                case "moon":
                    controller.setCelestialBody(.moonIauSphere)
                    orbit = .lunarWholeGlobeDefault
                case "mars":
                    controller.setCelestialBody(.marsIauSpheroid)
                    orbit = .martianWholeGlobeDefault
                default:
                    break
                }
            case "--surface" where idx + 1 < args.count:
                let surfVal = args[idx + 1].lowercased()
                idx += 1
                switch surfVal {
                case "satellite", "sat", "ortho":
                    layers.baseSurface = .satellite
                case "terrain", "dem", "relief":
                    layers.baseSurface = .terrain
                default:
                    break
                }
            case "--layer" where idx + 1 < args.count:
                let layerVal = args[idx + 1].lowercased()
                idx += 1
                switch layerVal {
                case "off", "none":
                    layers.aeroChart = .none
                case "vfr", "us_vfr":
                    layers.aeroChart = .usVfr
                case "ifr_low", "ifr":
                    layers.aeroChart = .ifrLow
                case "ifr_high":
                    layers.aeroChart = .ifrHigh
                case "satellite", "sat", "ortho":
                    layers.baseSurface = .satellite
                    layers.aeroChart = .none
                case "terrain", "dem":
                    layers.baseSurface = .terrain
                    layers.aeroChart = .none
                default:
                    break
                }
            case "--chart-opacity" where idx + 1 < args.count:
                if let alpha = Double(args[idx + 1]) {
                    layers.chartOpacity = min(max(alpha, 0.35), 1.0)
                }
                idx += 1
            case "--airspace" where idx + 1 < args.count:
                let airVal = args[idx + 1].lowercased()
                idx += 1
                layers.airspace3D = (airVal != "off" && airVal != "false" && airVal != "0")
            case "--terrain" where idx + 1 < args.count:
                let terVal = args[idx + 1].lowercased()
                idx += 1
                layers.terrain3D = (terVal != "off" && terVal != "false" && terVal != "0")
            case "--preset" where idx + 1 < args.count:
                let presetId = args[idx + 1]
                idx += 1
                if let bm = NavigationBookmark.forPrimaryBody(controller.primaryBody)
                    .first(where: { $0.id == presetId }) {
                    if !bm.routeCodes.isEmpty {
                        controller.setFlightPlanCodes(bm.routeCodes)
                    }
                    orbit = bm.state
                    northLocked = abs(bm.state.bearingDeg) < 0.5
                    primaryMode = .map
                }
            case "--lat" where idx + 1 < args.count:
                if let v = Double(args[idx + 1]) { orbit.latitude = v }
                idx += 1
            case "--lon" where idx + 1 < args.count:
                if let v = Double(args[idx + 1]) { orbit.longitude = v }
                idx += 1
            case "--zoom" where idx + 1 < args.count:
                if let v = Double(args[idx + 1]) { orbit.zoom = v }
                idx += 1
            case "--pitch" where idx + 1 < args.count:
                if let v = Double(args[idx + 1]) { orbit.pitchDeg = v }
                idx += 1
            case "--bearing" where idx + 1 < args.count:
                if let v = Double(args[idx + 1]) {
                    orbit.bearingDeg = v
                    northLocked = abs(v) < 0.5
                }
                idx += 1
            case "--route" where idx + 1 < args.count:
                let codes = args[idx + 1]
                    .split(separator: ",")
                    .map { String($0).trimmingCharacters(in: .whitespaces) }
                controller.setFlightPlanCodes(codes)
                idx += 1
            case "--apply-cifp" where idx + 1 < args.count,
                 "--apply-procedure" where idx + 1 < args.count:
                // Syntax: `AIRPORT,PROC_IDENT[,ENR_OR_FEEDER_TRANS[,RWY_TRANS]]` (e.g. `N51,R22,PIYIN`).
                let parts = args[idx + 1]
                    .split(separator: ",")
                    .map { String($0).trimmingCharacters(in: .whitespaces).uppercased() }
                idx += 1
                if parts.count >= 2 {
                    let apt = parts[0]
                    let procCode = parts[1]
                    let enrTr = parts.count >= 3 && !parts[2].isEmpty ? parts[2] : nil
                    let rwyTr = parts.count >= 4 && !parts[3].isEmpty ? parts[3] : nil
                    if let proc = controller.availableCifpProcedures(forAirport: apt)
                        .first(where: { $0.procedureIdent == procCode }) {
                        controller.applyCifpProcedure(
                            proc,
                            runwayTransitionIdent: rwyTr,
                            enrouteOrFeederTransitionIdent: enrTr
                        )
                    }
                }
            case "--active-wpt" where idx + 1 < args.count:
                if let wptIdx = Int(args[idx + 1]) {
                    controller.activateDirectTo(index: wptIdx)
                }
                idx += 1
            case "--pin-plate" where idx + 1 < args.count:
                // Syntax: `AIRPORT,PDF_NAME_OR_PROC` (e.g. `N51,05075R22.PDF` or `KISP,R06`).
                let parts = args[idx + 1]
                    .split(separator: ",")
                    .map { String($0).trimmingCharacters(in: .whitespaces).uppercased() }
                idx += 1
                if parts.count >= 2 {
                    let apt = parts[0]
                    let target = parts[1]
                    let pinned: PinnedMapPlate?
                    if target.hasSuffix(".PDF") {
                        pinned = controller.pinPlateByPdfName(airportCode: apt, pdfName: target)
                    } else if let proc = controller.availableCifpProcedures(forAirport: apt)
                        .first(where: { $0.procedureIdent == target }) {
                        pinned = controller.pinPlateToMap(controller.resolvePlateDescriptor(for: proc), procedure: proc)
                    } else {
                        pinned = nil
                    }
                    if let pinned {
                        orbit = OrbitCameraState(
                            latitude: pinned.centerLatitudeDeg,
                            longitude: pinned.centerLongitudeDeg,
                            zoom: pinned.descriptor.kind == .airportDiagram ? 13.4 : 10.6,
                            pitchDeg: 0.0,
                            bearingDeg: 0.0
                        )
                        primaryMode = .map
                    }
                }
            case "--plate-invert":
                controller.pinnedPlateInverted = true
            case "--open-plate" where idx + 1 < args.count:
                let parts = args[idx + 1]
                    .split(separator: ",")
                    .map { String($0).trimmingCharacters(in: .whitespaces).uppercased() }
                idx += 1
                if parts.count >= 2 {
                    let apt = parts[0]
                    let target = parts[1]
                    let hideChrome = parts.count >= 3 && (parts[2] == "HIDE" || parts[2] == "FULL")
                    if let plate = controller.availablePlates(forAirport: apt)
                        .first(where: { ($0.pdfName ?? "").uppercased() == target }) {
                        let matchingProc = controller.availableCifpProcedures(forAirport: apt)
                            .first(where: { $0.procedureIdent == plate.cifpIdent })
                        presentedPlateRequest = PresentedPlateRequest(
                            descriptor: plate,
                            procedure: matchingProc,
                            runwayTransitionIdent: nil,
                            enrouteOrFeederTransitionIdent: nil,
                            startWithChromeHidden: hideChrome
                        )
                    }
                }
            case "--open-layers":
                isLayersPanelExpanded = true
            case "--layers-submenu" where idx + 1 < args.count:
                let subVal = args[idx + 1].lowercased()
                idx += 1
                isLayersPanelExpanded = true
                switch subVal {
                case "airspace": layersSubmenu = .airspace
                case "weather", "wx": layersSubmenu = .weather
                case "terrain", "safety", "terrain_safety": layersSubmenu = .terrainSafety
                case "traffic", "runways", "traffic_runways": layersSubmenu = .trafficRunways
                default: layersSubmenu = nil
                }
            case "--open-fpl":
                isFplSheetPresented = true
            case "--open-downloads":
                isDownloadsSheetPresented = true
            case "--clear-fpl":
                controller.clearFlightPlan()
            case "--delayed-clear-fpl":
                Task { @MainActor in
                    try? await Task.sleep(nanoseconds: 2_200_000_000)
                    controller.clearFlightPlan()
                }
            case "--test-utc" where idx + 1 < args.count:
                let utcStr = args[idx + 1]
                idx += 1
                if utcStr.lowercased() == "live" {
                    controller.dataPackStore.setReferenceUtcOverride(nil)
                } else if let parsed = AiracCycleGrid.parseIso8601Utc(utcStr) {
                    controller.dataPackStore.setReferenceUtcOverride(parsed)
                }
            case "--fpl-query" where idx + 1 < args.count:
                fplInitialQuery = args[idx + 1]
                isFplSheetPresented = true
                idx += 1
            case "--hud":
                showHudOverlay = true
            case "--no-hud":
                showHudOverlay = false
            case "--camera" where idx + 1 < args.count:
                let cam = args[idx + 1].lowercased()
                idx += 1
                switch cam {
                case "fpv", "cockpit":
                    primaryMode = .svs
                    svsSubMode = .fpv
                case "chase":
                    primaryMode = .svs
                    svsSubMode = .chase
                case "map", "globe":
                    primaryMode = .map
                default:
                    break
                }
            case "--preview-kml" where idx + 1 < args.count:
                let path = args[idx + 1]
                idx += 1
                openTrackPreview(from: URL(fileURLWithPath: path))
            case "--seek" where idx + 1 < args.count:
                if let frac = Double(args[idx + 1]) {
                    controller.seekKml(fraction: frac)
                }
                idx += 1
            case "--pause":
                controller.setKmlPlayback(paused: true, speedMultiplier: controller.kmlPlayback.speedMultiplier)
            case "--sim-off-limb-pan" where idx + 1 < args.count:
                let parts = args[idx + 1].split(separator: ",").compactMap { Double($0.trimmingCharacters(in: .whitespaces)) }
                idx += 1
                if parts.count == 2 {
                    let dx = parts[0]
                    let dy = parts[1]
                    Task { @MainActor in
                        try? await Task.sleep(nanoseconds: 1_800_000_000)
                        NotificationCenter.default.post(
                            name: .flightDeckSimulateOffLimbPan,
                            object: nil,
                            userInfo: ["dx": dx, "dy": dy]
                        )
                    }
                }
            default:
                break
            }
            idx += 1
        }
    }

    @MainActor
    private func scheduleBanner(_ text: String) {
        bannerText = text
        Task { @MainActor in
            try? await Task.sleep(nanoseconds: 5_000_000_000)
            if bannerText == text {
                bannerText = nil
            }
        }
    }
}

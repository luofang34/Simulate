import IndicateAppleDisplay
import SwiftUI
import FlightDeckKit

/// Main iPadOS viewport uniting the full-screen 3D globe/chart canvas, dead-center transparent
/// conformal HUD (`FPV` boresight only), icon-only top-left Layers button, top-right `Map | SVS`
/// controls, and bottom EFB `FPL` + `Sokoly` GCS command bar.
struct PadCockpitView: View {
    @Bindable var controller: FlightDeckController
    @Binding var primaryMode: PadPrimaryViewMode
    @Binding var svsSubMode: SvsSubMode
    @Binding var svsLookOffset: SvsLookOffset
    @Binding var followMode: MapFollowMode
    @Binding var northLocked: Bool
    @Binding var layers: PadLayerConfiguration
    @Binding var orbit: OrbitCameraState
    @Binding var showHudOverlay: Bool
    @Binding var isLayersPanelExpanded: Bool
    @Binding var layersSubmenu: LayersSubmenuPage?
    @Binding var isFplSheetPresented: Bool
    @Binding var isReceiverSheetPresented: Bool
    @Binding var isDownloadsSheetPresented: Bool
    @Binding var presentedPlateRequest: PresentedPlateRequest?
    @Binding var bannerText: String?
    var fplInitialQuery: String = ""

    @State private var effectiveZoom: Double = 1.55
    @State private var isPlateOptionsPopoverPresented: Bool = false

    /// Conformal HUD is strictly a 1st-person cockpit (`SVS -> FPV`) boresight instrument;
    /// exiting `SVS` or switching to `Chase` never leaves HUD symbology over the map.
    private var isConformalHudVisible: Bool {
        primaryMode == .svs && svsSubMode == .fpv && showHudOverlay
    }

    /// Hide the collapsed bottom dock whenever an editor/receiver/downloads/plate modal sheet is presented (`Pilotage` pattern)
    /// so the planner sheet and bottom controls never overlap.
    private var isModalSheetPresented: Bool {
        isFplSheetPresented || isReceiverSheetPresented || isDownloadsSheetPresented || presentedPlateRequest != nil
    }

    /// Computes any active offline-pack invalidation or missing-download alert for enabled layers on Earth.
    private var activePackInvalidationAlert: (message: String, isExpired: Bool)? {
        guard controller.primaryBody == .earth else { return nil }
        let store = controller.dataPackStore
        var expiredItems: [String] = []
        var missingItems: [String] = []

        switch layers.aeroChart {
        case .none:
            break
        case .usVfr:
            let secVal = store.validity(for: .faaVfrSectional)
            if secVal.isExpiredInvalid {
                expiredItems.append("US VFR Chart")
            } else if !secVal.isValidForUse {
                missingItems.append("US VFR Chart")
            }
        case .ifrLow:
            let lowVal = store.validity(for: .faaIfrLow)
            if lowVal.isExpiredInvalid {
                expiredItems.append("IFR Low Chart")
            } else if !lowVal.isValidForUse {
                missingItems.append("IFR Low Chart")
            }
        case .ifrHigh:
            let highVal = store.validity(for: .faaIfrHigh)
            if highVal.isExpiredInvalid {
                expiredItems.append("IFR High Chart")
            } else if !highVal.isValidForUse {
                missingItems.append("IFR High Chart")
            }
        }

        if layers.airspace3D {
            let airVal = store.validity(for: .faaNasrAirspace)
            if airVal.isExpiredInvalid {
                expiredItems.append("3D Airspace")
            } else if !airVal.isValidForUse {
                missingItems.append("3D Airspace")
            }
        }

        if !expiredItems.isEmpty {
            return ("\(expiredItems.joined(separator: " + ")) EXPIRED — INVALID & SUPPRESSED", true)
        }
        if !missingItems.isEmpty {
            return ("\(missingItems.joined(separator: " + ")) NOT DOWNLOADED — OFFLINE PACK REQUIRED", false)
        }
        return nil
    }

    var body: some View {
        ZStack {
            // Full-screen (`ignoresSafeArea`) container shared identically by `MaplibreMetalView`
            // and `TransparentHudOverlay` so HUD boresight (`Layout.centre`) is at exact (50%, 50%).
            GeometryReader { fullGeo in
                ZStack {
                    MaplibreMetalView(
                        controller: controller,
                        primaryMode: $primaryMode,
                        svsSubMode: $svsSubMode,
                        svsLookOffset: $svsLookOffset,
                        followMode: $followMode,
                        northLocked: $northLocked,
                        layers: $layers,
                        orbit: $orbit,
                        showHudOverlay: $showHudOverlay,
                        effectiveZoom: $effectiveZoom,
                        onCanvasInteraction: {
                            if isLayersPanelExpanded {
                                withAnimation(.easeOut(duration: 0.16)) {
                                    isLayersPanelExpanded = false
                                    layersSubmenu = nil
                                }
                            }
                        }
                    )

                    if isConformalHudVisible {
                        transparentConformalHudOverlay(fullScreenSize: fullGeo.size)
                            .allowsHitTesting(false)
                    }
                }
            }
            .ignoresSafeArea()

            // Top-leading: Icon-only Layers button + compact ForeFlight-style `Plates on Maps` chip (when active).
            HStack(alignment: .top, spacing: 8) {
                PadLayersControlCluster(
                    controller: controller,
                    layers: $layers,
                    isExpanded: $isLayersPanelExpanded,
                    activeSubmenu: $layersSubmenu,
                    effectiveZoom: effectiveZoom,
                    onSelectPrimaryBody: { selectedBody in
                        controller.setPrimaryBody(selectedBody)
                        primaryMode = .map
                        followMode = .free
                        northLocked = true
                        layersSubmenu = nil
                        switch selectedBody {
                        case .earth:
                            orbit = .wholeGlobeDefault
                        case .moon:
                            orbit = .lunarWholeGlobeDefault
                        case .mars:
                            orbit = .martianWholeGlobeDefault
                        }
                    },
                    onOpenDownloads: {
                        isDownloadsSheetPresented = true
                    }
                )

                if primaryMode == .map, let pinned = controller.pinnedMapPlate {
                    compactMapPlateChip(pinned)
                }
            }
            .floatingControlPlacement(.topLeading)

            // Top-center: Operational invalidation alert or transient event toast only (zero permanent plate banners).
            VStack(spacing: 6) {
                if let alert = activePackInvalidationAlert {
                    packInvalidationPill(message: alert.message, isExpired: alert.isExpired)
                }
                if let banner = bannerText {
                    statusToast(banner)
                }
            }
            .floatingControlPlacement(.top)

            // Top-trailing: `Map | SVS` switcher + context-specific controls.
            PadViewControlsCluster(
                controller: controller,
                primaryMode: $primaryMode,
                svsSubMode: $svsSubMode,
                svsLookOffset: $svsLookOffset,
                followMode: $followMode,
                northLocked: $northLocked,
                orbit: $orbit,
                showHudOverlay: $showHudOverlay,
                layers: layers,
                effectiveZoom: effectiveZoom
            )
            .floatingControlPlacement(.topTrailing)

            // Bottom-center: Collapsed EFB Flight Plan (`FPL`) + `Sokoly` GCS command & telemetry bar.
            // Hidden cleanly whenever the planner or receiver sheet is open (`Pilotage` MissionPlannerBar pattern).
            PadBottomFplGcsBar(
                controller: controller,
                primaryMode: $primaryMode,
                orbit: $orbit,
                isFplSheetPresented: $isFplSheetPresented,
                isReceiverSheetPresented: $isReceiverSheetPresented
            )
            .opacity(isModalSheetPresented ? 0.0 : 1.0)
            .allowsHitTesting(!isModalSheetPresented)
            .animation(.easeOut(duration: 0.16), value: isModalSheetPresented)
            .floatingControlPlacement(.bottom)
        }
        .preferredColorScheme(.dark)
        .sheet(isPresented: $isFplSheetPresented) {
            PadFplAndGcsSheet(
                controller: controller,
                primaryMode: $primaryMode,
                orbit: $orbit,
                initialDraftQuery: fplInitialQuery,
                onInspectPlate: { req in
                    isFplSheetPresented = false
                    DispatchQueue.main.asyncAfter(deadline: .now() + 0.18) {
                        presentedPlateRequest = req
                    }
                }
            )
            .presentationDetents([.medium, .large])
            .pageSheetSizingIfAvailable()
            .presentationDragIndicator(.visible)
        }
        .sheet(isPresented: $isReceiverSheetPresented) {
            PadReceiverAndDatumSheet(controller: controller)
                .presentationDetents([.medium, .large])
                .pageSheetSizingIfAvailable()
                .presentationDragIndicator(.visible)
        }
        .sheet(isPresented: $isDownloadsSheetPresented) {
            PadDataDownloadsSheet(controller: controller)
                .presentationDetents([.large])
                .pageSheetSizingIfAvailable()
                .presentationDragIndicator(.visible)
        }
        .fullScreenCover(item: $presentedPlateRequest) { req in
            PadProcedurePlateViewerSheet(
                controller: controller,
                request: req,
                onPinnedToMap: { pinned in
                    orbit = OrbitCameraState(
                        latitude: pinned.centerLatitudeDeg,
                        longitude: pinned.centerLongitudeDeg,
                        zoom: pinned.descriptor.kind == .airportDiagram ? 13.4 : 10.6,
                        pitchDeg: 0.0,
                        bearingDeg: 0.0
                    )
                    primaryMode = .map
                }
            )
        }
    }

    // MARK: - Pure Transparent Dead-Center Conformal HUD Overlay (`IndicateAppleDisplay`)

    private func transparentConformalHudOverlay(fullScreenSize: CGSize) -> some View {
        let insetX = max(fullScreenSize.width * 0.05, 16.0)
        let insetY = max(fullScreenSize.height * 0.10, 24.0)
        let hudHeight = max(fullScreenSize.height - insetY * 2.0, 1.0)
        let sceneVFovRad = 62.0 * .pi / 180.0
        let hudVFovRad = 2.0 * atan(
            tan(sceneVFovRad * 0.5) * Double(hudHeight / max(fullScreenSize.height, 1.0))
        )

        return TransparentHudOverlay(display: controller.hudDisplay)
            .frame(
                width: max(fullScreenSize.width - insetX * 2.0, 1.0),
                height: hudHeight
            )
            .position(x: fullScreenSize.width * 0.5, y: fullScreenSize.height * 0.5)
            .onAppear {
                controller.setHudFieldOfView(radians: Float(hudVFovRad))
            }
            .onChange(of: hudVFovRad) { _, updated in
                controller.setHudFieldOfView(radians: Float(updated))
            }
    }

    private func packInvalidationPill(message: String, isExpired: Bool) -> some View {
        let tint: Color = isExpired ? PadTheme.warning : PadTheme.caution
        return HStack(spacing: 8) {
            Image(systemName: isExpired ? "xmark.octagon.fill" : "arrow.down.circle.fill")
                .foregroundStyle(tint)
            Text(message)
                .font(.system(size: 11, weight: .heavy, design: .monospaced))
                .foregroundStyle(tint)
            Button {
                isDownloadsSheetPresented = true
            } label: {
                Text("Downloads")
                    .foregroundStyle(isExpired ? PadTheme.value : PadTheme.background)
            }
            .buttonStyle(.borderedProminent)
            .tint(tint)
            .controlSize(.mini)
            .font(.system(size: 10, weight: .bold, design: .monospaced))
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 7)
        .cockpitFloatingCapsule(borderTint: tint.opacity(0.75))
    }

    private func compactMapPlateChip(_ pinned: PinnedMapPlate) -> some View {
        HStack(spacing: 7) {
            Button {
                isPlateOptionsPopoverPresented.toggle()
            } label: {
                HStack(spacing: 5) {
                    Image(systemName: "doc.richtext.fill")
                        .font(.system(size: 11.5, weight: .semibold))
                        .foregroundStyle(PadTheme.commanded)
                    Text("\(pinned.descriptor.airportIdent) · \(pinned.descriptor.title)")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .foregroundStyle(PadTheme.value)
                        .lineLimit(1)
                        .frame(maxWidth: 196)
                    Image(systemName: "chevron.down")
                        .font(.system(size: 9, weight: .bold))
                        .foregroundStyle(PadTheme.label)
                }
                .padding(.vertical, 2)
            }
            .buttonStyle(.plain)
            .popover(isPresented: $isPlateOptionsPopoverPresented, arrowEdge: .top) {
                mapPlatePopoverContent(pinned)
                    .presentationCompactAdaptation(.popover)
            }

            Rectangle()
                .fill(PadTheme.separator)
                .frame(width: 1, height: 14)

            Button {
                isPlateOptionsPopoverPresented = false
                controller.clearPinnedMapPlate()
            } label: {
                Image(systemName: "xmark")
                    .font(.system(size: 10.5, weight: .bold))
                    .foregroundStyle(PadTheme.label)
                    .frame(width: 20, height: 22)
            }
            .buttonStyle(.plain)
            .accessibilityLabel("Hide Plate")
        }
        .frame(height: PadMetrics.buttonSize)
        .padding(.horizontal, 11)
        .cockpitFloatingCapsule(borderTint: PadTheme.commanded.opacity(0.55))
    }

    private func mapPlatePopoverContent(_ pinned: PinnedMapPlate) -> some View {
        let pct = Int((controller.pinnedPlateOpacity * 100.0).rounded())
        let siblingPlates = controller.availablePlates(forAirport: pinned.descriptor.airportIdent)
            .filter(\.supportsMapOverlay)

        return VStack(alignment: .leading, spacing: 10) {
            HStack {
                Text("Plate Transparency")
                    .font(.system(size: 11, weight: .semibold))
                    .foregroundStyle(PadTheme.label)
                Spacer()
                Text("\(pct)%")
                    .font(.system(size: 11, weight: .bold, design: .monospaced))
                    .foregroundStyle(PadTheme.value)
            }

            Slider(
                value: Binding(
                    get: { controller.pinnedPlateOpacity },
                    set: { controller.pinnedPlateOpacity = $0 }
                ),
                in: 0.25...1.0
            )
            .tint(PadTheme.commanded)

            Toggle(
                "Invert Plate Colors",
                isOn: Binding(
                    get: { controller.pinnedPlateInverted },
                    set: { controller.pinnedPlateInverted = $0 }
                )
            )
            .font(.system(size: 11.5, weight: .medium))
            .foregroundStyle(PadTheme.value)
            .tint(PadTheme.commanded)
            .controlSize(.small)

            HStack(spacing: 8) {
                Button {
                    isPlateOptionsPopoverPresented = false
                    presentedPlateRequest = PresentedPlateRequest(
                        descriptor: pinned.descriptor,
                        procedure: pinned.procedure,
                        runwayTransitionIdent: pinned.runwayTransitionIdent,
                        enrouteOrFeederTransitionIdent: pinned.enrouteOrFeederTransitionIdent
                    )
                } label: {
                    Label("View Fullscreen", systemImage: "arrow.up.left.and.arrow.down.right")
                        .font(.system(size: 11, weight: .bold))
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.bordered)
                .tint(PadTheme.commanded)
                .controlSize(.small)

                Button {
                    isPlateOptionsPopoverPresented = false
                    controller.clearPinnedMapPlate()
                } label: {
                    Text("Hide Plate")
                        .font(.system(size: 11, weight: .bold))
                }
                .buttonStyle(.bordered)
                .tint(PadTheme.label)
                .controlSize(.small)
            }

            if siblingPlates.count > 1 {
                Divider().overlay(PadTheme.separator)

                Text("\(pinned.descriptor.airportIdent) Plates on Map")
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                    .foregroundStyle(PadTheme.label)

                ScrollView {
                    VStack(alignment: .leading, spacing: 4) {
                        ForEach(siblingPlates) { cand in
                            let isCurrent = (cand.id == pinned.descriptor.id)
                            Button {
                                if let nextPinned = controller.pinPlateToMap(cand) {
                                    orbit.latitude = nextPinned.centerLatitudeDeg
                                    orbit.longitude = nextPinned.centerLongitudeDeg
                                    orbit.zoom = cand.kind == .airportDiagram ? 13.4 : 10.6
                                }
                            } label: {
                                HStack(spacing: 6) {
                                    Image(systemName: isCurrent ? "checkmark.circle.fill" : "circle")
                                        .font(.system(size: 11))
                                        .foregroundStyle(isCurrent ? PadTheme.commanded : PadTheme.label)
                                    Text(cand.title)
                                        .font(.system(size: 11, weight: isCurrent ? .bold : .medium, design: .monospaced))
                                        .foregroundStyle(isCurrent ? PadTheme.commanded : PadTheme.value)
                                        .lineLimit(1)
                                    Spacer(minLength: 0)
                                }
                                .padding(.vertical, 4)
                                .padding(.horizontal, 6)
                                .background(isCurrent ? PadTheme.commanded.opacity(0.14) : Color.clear, in: RoundedRectangle(cornerRadius: 6))
                            }
                            .buttonStyle(.plain)
                        }
                    }
                }
                .frame(maxHeight: 150)
            }
        }
        .padding(12)
        .frame(width: 280)
        .background(PadTheme.surface)
        .preferredColorScheme(.dark)
    }

    private func statusToast(_ message: String) -> some View {
        HStack(spacing: 8) {
            Image(systemName: "waveform.path.ecg")
                .foregroundStyle(PadTheme.active)
            Text(message)
                .font(.caption.monospaced().weight(.semibold))
                .foregroundStyle(PadTheme.value)
            Button {
                bannerText = nil
            } label: {
                Image(systemName: "xmark.circle.fill")
                    .foregroundStyle(PadTheme.label)
            }
            .buttonStyle(.plain)
        }
        .padding(.horizontal, 14)
        .padding(.vertical, 8)
        .cockpitFloatingCapsule()
    }
}

private extension View {
    @ViewBuilder
    func pageSheetSizingIfAvailable() -> some View {
        if #available(iOS 18.0, *) {
            self.presentationSizing(.page)
        } else {
            self
        }
    }
}

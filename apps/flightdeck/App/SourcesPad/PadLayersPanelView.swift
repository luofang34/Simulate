import SwiftUI
import FlightDeckKit

/// Top-leading icon-only Layers button (`square.3.layers.3d`, zero text on the button)
/// and its hierarchical Surface + Aeronautical Chart + Secondary-Submenu Overlays popover.
struct PadLayersControlCluster: View {
    @Bindable var controller: FlightDeckController
    @Binding var layers: PadLayerConfiguration
    @Binding var isExpanded: Bool
    @Binding var activeSubmenu: LayersSubmenuPage?
    let effectiveZoom: Double
    let onSelectPrimaryBody: (PrimaryCelestialBody) -> Void
    let onOpenDownloads: () -> Void

    private var hasSuppressedLayerAlert: Bool {
        guard controller.primaryBody == .earth else { return false }
        let store = controller.dataPackStore
        if layers.airspace3D, !store.validity(for: .faaNasrAirspace).isValidForUse {
            return true
        }
        switch layers.aeroChart {
        case .none:
            return false
        case .usVfr:
            return !store.validity(for: .faaVfrSectional).isValidForUse
        case .ifrLow:
            return !store.validity(for: .faaIfrLow).isValidForUse
        case .ifrHigh:
            return !store.validity(for: .faaIfrHigh).isValidForUse
        }
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Button {
                withAnimation(.spring(response: 0.26, dampingFraction: 0.86)) {
                    isExpanded.toggle()
                    if !isExpanded {
                        activeSubmenu = nil
                    }
                }
            } label: {
                ZStack(alignment: .topTrailing) {
                    Image(systemName: controller.primaryBody == .earth ? "square.3.layers.3d" : controller.primaryBody.symbolName)
                        .font(.system(size: 18, weight: .semibold))
                        .foregroundStyle(isExpanded ? PadTheme.commanded : PadTheme.value)
                        .frame(width: PadMetrics.buttonSize, height: PadMetrics.buttonSize)
                        .background {
                            Circle()
                                .fill(PadTheme.background.opacity(0.94))
                                .background(.ultraThinMaterial, in: Circle())
                        }
                        .overlay(
                            Circle()
                                .strokeBorder(
                                    isExpanded ? PadTheme.commanded : PadTheme.separator,
                                    lineWidth: 1
                                )
                        )
                        .shadow(color: .black.opacity(0.36), radius: 8, y: 3)

                    if hasSuppressedLayerAlert {
                        Circle()
                            .fill(PadTheme.warning)
                            .frame(width: 10, height: 10)
                            .offset(x: -1, y: 1)
                    } else if controller.primaryBody != .earth {
                        Circle()
                            .fill(PadTheme.caution)
                            .frame(width: 9, height: 9)
                            .offset(x: -2, y: 2)
                    }
                }
            }
            .buttonStyle(.plain)
            .accessibilityLabel("Map Layers and Reference Body")

            if isExpanded {
                PadLayersPanelCard(
                    controller: controller,
                    layers: $layers,
                    activeSubmenu: $activeSubmenu,
                    effectiveZoom: effectiveZoom,
                    onSelectPrimaryBody: onSelectPrimaryBody,
                    onOpenDownloads: {
                        isExpanded = false
                        activeSubmenu = nil
                        onOpenDownloads()
                    },
                    onClose: {
                        withAnimation(.spring(response: 0.24, dampingFraction: 0.88)) {
                            isExpanded = false
                            activeSubmenu = nil
                        }
                    }
                )
                .transition(.opacity.combined(with: .scale(scale: 0.96, anchor: .topLeading)))
            }
        }
    }
}

/// Hierarchical Layers popover separating Base Surface (`Terrain | Satellite`), Aeronautical Chart
/// (`Off | US VFR | IFR Low | IFR High`), compact header Planet menu, Offline Downloads, and `›` Secondary Submenus.
private struct PadLayersPanelCard: View {
    @Bindable var controller: FlightDeckController
    @Binding var layers: PadLayerConfiguration
    @Binding var activeSubmenu: LayersSubmenuPage?
    let effectiveZoom: Double
    let onSelectPrimaryBody: (PrimaryCelestialBody) -> Void
    let onOpenDownloads: () -> Void
    let onClose: () -> Void

    private let labelMuted = PadTheme.label
    private let iconMuted = PadTheme.label

    var body: some View {
        VStack(alignment: .leading, spacing: 0) {
            headerBar
                .padding(.horizontal, 14)
                .padding(.top, 11)
                .padding(.bottom, 9)

            Divider().overlay(PadTheme.separator)

            panelBodyContent
        }
        .frame(width: 356)
        .fixedSize(horizontal: false, vertical: true)
        .cockpitFloatingChrome(cornerRadius: 18)
    }

    private var panelBodyContent: some View {
        VStack(alignment: .leading, spacing: 12) {
            if let submenu = activeSubmenu {
                submenuContent(for: submenu)
            } else if controller.primaryBody == .earth {
                surfaceSection
                Divider().overlay(PadTheme.separator)
                aeroChartSection
                Divider().overlay(PadTheme.separator)
                overlaysRootSection
            } else {
                nonEarthNoticeSection
            }
        }
        .padding(.horizontal, 14)
        .padding(.vertical, 12)
    }

    // MARK: - Header Bar (with `< Layers` back navigation + Compact Planet Icon Menu)

    private var headerBar: some View {
        HStack(spacing: 8) {
            if let submenu = activeSubmenu {
                Button {
                    withAnimation(.easeInOut(duration: 0.18)) {
                        activeSubmenu = nil
                    }
                } label: {
                    HStack(spacing: 4) {
                        Image(systemName: "chevron.left")
                            .font(.caption.weight(.bold))
                        Text("Layers")
                            .font(.caption.weight(.semibold))
                    }
                    .foregroundStyle(PadTheme.commanded)
                }
                .buttonStyle(.plain)

                Spacer(minLength: 4)

                Text(submenu.title)
                    .font(.subheadline.weight(.bold))
                    .foregroundStyle(PadTheme.value)
                    .lineLimit(1)

                Spacer(minLength: 4)
            } else {
                Text("Layers")
                    .font(.subheadline.weight(.bold))
                    .foregroundStyle(PadTheme.value)

                Spacer()

                if controller.primaryBody == .earth {
                    downloadsHeaderButton
                }

                // Compact low-frequency Celestial Body icon menu (icon-only circle on Earth).
                planetMenuButton
            }

            Button(action: onClose) {
                Image(systemName: "xmark.circle.fill")
                    .font(.title3)
                    .foregroundStyle(iconMuted)
            }
            .buttonStyle(.plain)
            .accessibilityLabel("Close Layers Panel")
        }
    }

    private var downloadsHeaderButton: some View {
        let health = controller.dataPackStore.overallHealthSummary()
        let tint: Color = health.hasExpired ? PadTheme.warning : PadTheme.active
        return Button(action: onOpenDownloads) {
            HStack(spacing: 4) {
                Image(systemName: health.hasExpired ? "exclamationmark.arrow.triangle.2.circlepath" : "arrow.down.circle.fill")
                    .font(.system(size: 11.5, weight: .semibold))
                Text(health.hasExpired ? "EXPIRED" : "Downloads")
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
            }
            .foregroundStyle(tint)
            .padding(.horizontal, 8)
            .frame(minHeight: 28)
            .background(tint.opacity(0.16), in: Capsule())
            .overlay(Capsule().strokeBorder(tint.opacity(0.50), lineWidth: 1))
        }
        .buttonStyle(.plain)
        .accessibilityLabel("Offline Downloads and AIRAC Cycles")
    }

    private var planetMenuButton: some View {
        Menu {
            Section("Reference Celestial Body") {
                ForEach(PrimaryCelestialBody.allCases) { option in
                    Button {
                        onSelectPrimaryBody(option)
                    } label: {
                        HStack {
                            Label("\(option.rawValue) · \(option.figureSummary)", systemImage: option.symbolName)
                            if controller.primaryBody == option {
                                Image(systemName: "checkmark")
                            }
                        }
                    }
                }
            }
            if controller.primaryBody == .earth {
                Section("Earth Vertical Datum") {
                    ForEach(EarthVerticalDatum.allCases) { datum in
                        Button {
                            controller.setEarthVerticalDatum(datum)
                        } label: {
                            HStack {
                                Text(datum.title)
                                if controller.earthVerticalDatum == datum {
                                    Image(systemName: "checkmark")
                                }
                            }
                        }
                    }
                }
            }
        } label: {
            HStack(spacing: 4) {
                Image(systemName: controller.primaryBody.symbolName)
                    .font(.system(size: 13.5, weight: .semibold))
                    .foregroundStyle(controller.primaryBody == .earth ? PadTheme.value : PadTheme.caution)
                if controller.primaryBody != .earth {
                    Text(controller.primaryBody.rawValue.uppercased())
                        .font(.system(size: 9.5, weight: .heavy, design: .monospaced))
                        .foregroundStyle(PadTheme.caution)
                }
            }
            .padding(.horizontal, controller.primaryBody == .earth ? 0 : 8)
            .frame(minWidth: 28, minHeight: 28)
            .background(PadTheme.control, in: Capsule())
            .overlay(
                Capsule()
                    .strokeBorder(
                        controller.primaryBody == .earth ? PadTheme.separator : PadTheme.caution.opacity(0.65),
                        lineWidth: 1
                    )
            )
        }
        .accessibilityLabel("Select Celestial Reference Body")
    }

    // MARK: - Tier 1: Base Planet Surface (`Terrain` | `Satellite`)

    private var surfaceSection: some View {
        VStack(alignment: .leading, spacing: 7) {
            HStack {
                Text("BASE SURFACE")
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                    .foregroundStyle(labelMuted)
                Spacer()
                Text(layers.baseSurface.subtitle)
                    .font(.system(size: 9.5, weight: .medium, design: .monospaced))
                    .foregroundStyle(labelMuted)
            }

            HStack(spacing: 7) {
                ForEach(EarthBaseSurface.allCases) { surf in
                    let selected = (layers.baseSurface == surf)
                    Button {
                        layers.baseSurface = surf
                    } label: {
                        HStack(spacing: 6) {
                            Image(systemName: surf.symbolName)
                                .font(.caption.weight(.semibold))
                                .foregroundStyle(selected ? PadTheme.commanded : iconMuted)
                            Text(surf.rawValue)
                                .font(.caption.weight(.bold))
                                .foregroundStyle(selected ? PadTheme.commanded : PadTheme.value)
                        }
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 8)
                        .background(
                            selected ? PadTheme.commanded.opacity(0.18) : PadTheme.surface,
                            in: RoundedRectangle(cornerRadius: 10, style: .continuous)
                        )
                        .overlay(
                            RoundedRectangle(cornerRadius: 10, style: .continuous)
                                .strokeBorder(
                                    selected ? PadTheme.commanded : PadTheme.separator,
                                    lineWidth: 1
                                )
                        )
                    }
                    .buttonStyle(.plain)
                }
            }
        }
    }

    // MARK: - Tier 2: Aeronautical Chart Overlay (`Off` | `US VFR` | `IFR Low` | `IFR High`)

    private var aeroChartSection: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text("AERONAUTICAL CHART")
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                    .foregroundStyle(labelMuted)
                Spacer()
                Text(activeScaleDescription.text)
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                    .foregroundStyle(activeScaleDescription.tint)
            }

            LazyVGrid(columns: [GridItem(.flexible()), GridItem(.flexible())], spacing: 7) {
                ForEach(EarthAeroChart.allCases) { chart in
                    let selected = (layers.aeroChart == chart)
                    let packTag = chartPackPill(for: chart)
                    Button {
                        layers.aeroChart = chart
                    } label: {
                        VStack(alignment: .leading, spacing: 2) {
                            HStack(spacing: 5) {
                                Image(systemName: chart.symbolName)
                                    .font(.caption.weight(.semibold))
                                    .foregroundStyle(selected ? PadTheme.commanded : iconMuted)
                                Text(chart.rawValue)
                                    .font(.caption.weight(.bold))
                                    .foregroundStyle(selected ? PadTheme.commanded : PadTheme.value)
                                    .lineLimit(1)
                                Spacer(minLength: 0)
                                if let packTag {
                                    Text(packTag.text)
                                        .font(.system(size: 8, weight: .heavy, design: .monospaced))
                                        .padding(.horizontal, 4)
                                        .padding(.vertical, 1.5)
                                        .background(packTag.tint.opacity(0.20), in: Capsule())
                                        .foregroundStyle(packTag.tint)
                                }
                            }
                            Text(chart.subtitle)
                                .font(.system(size: 9.5))
                                .foregroundStyle(labelMuted)
                                .lineLimit(1)
                        }
                        .padding(.horizontal, 10)
                        .padding(.vertical, 7)
                        .background(
                            selected ? PadTheme.commanded.opacity(0.18) : PadTheme.surface,
                            in: RoundedRectangle(cornerRadius: 10, style: .continuous)
                        )
                        .overlay(
                            RoundedRectangle(cornerRadius: 10, style: .continuous)
                                .strokeBorder(
                                    selected ? PadTheme.commanded : PadTheme.separator,
                                    lineWidth: 1
                                )
                        )
                    }
                    .buttonStyle(.plain)
                }
            }

            if layers.hasActiveAeroChart {
                HStack(spacing: 8) {
                    Text("Chart Opacity")
                        .font(.system(size: 10.5, weight: .semibold))
                        .foregroundStyle(labelMuted)
                    Slider(value: $layers.chartOpacity, in: 0.35...1.0, step: 0.05)
                        .tint(PadTheme.commanded)
                    Text(String(format: "%.0f%%", layers.chartOpacity * 100.0))
                        .font(.system(size: 10.5, weight: .bold, design: .monospaced))
                        .foregroundStyle(PadTheme.value)
                        .frame(width: 36, alignment: .trailing)
                }
                .padding(.top, 2)
            }
        }
    }

    // MARK: - Tier 3: Operational Overlays with `›` Secondary Submenus

    private var overlaysRootSection: some View {
        let airValidity = controller.dataPackStore.validity(for: .faaNasrAirspace)
        let airSubtitle: String = {
            guard layers.airspace3D else { return "Off" }
            if !airValidity.isValidForUse {
                return "\(airValidity.badgeText) — Suppressed"
            }
            return layers.airspaceSummaryBadge
        }()

        return VStack(alignment: .leading, spacing: 6) {
            Text("OVERLAYS")
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .foregroundStyle(labelMuted)

            submenuEntryRow(
                page: .airspace,
                title: "Airspace",
                subtitle: airSubtitle,
                icon: "cube.transparent.fill",
                stubBadge: false,
                isOn: $layers.airspace3D
            )

            submenuEntryRow(
                page: .weather,
                title: "Weather",
                subtitle: "NEXRAD · METAR/TAF · Winds · SIGMET",
                icon: "cloud.bolt.rain.fill",
                stubBadge: true,
                isOn: $layers.weatherMasterStub
            )

            submenuEntryRow(
                page: .terrainSafety,
                title: "Terrain & Safety",
                subtitle: layers.terrain3D ? "3D DEM Active · Terminator & TFR/DOF" : "Flat Ellipsoid · Safety Overlays",
                icon: "mountain.2.circle.fill",
                stubBadge: false,
                isOn: $layers.terrain3D
            )

            submenuEntryRow(
                page: .trafficRunways,
                title: "Runways, Route & Traffic",
                subtitle: "Surveyed Thresholds · Typed FPL · ADS-B",
                icon: "point.bottomleft.forward.to.arrow.triangle.scurvepath.fill",
                stubBadge: false,
                isOn: $layers.runwaysAndRoute
            )
        }
    }

    private func submenuEntryRow(
        page: LayersSubmenuPage,
        title: String,
        subtitle: String,
        icon: String,
        stubBadge: Bool,
        isOn: Binding<Bool>
    ) -> some View {
        HStack(spacing: 8) {
            Button {
                withAnimation(.easeInOut(duration: 0.18)) {
                    activeSubmenu = page
                }
            } label: {
                HStack(spacing: 8) {
                    Image(systemName: icon)
                        .font(.caption.weight(.semibold))
                        .foregroundStyle(isOn.wrappedValue ? (stubBadge ? PadTheme.caution : PadTheme.commanded) : iconMuted)
                        .frame(width: 20)

                    VStack(alignment: .leading, spacing: 1) {
                        HStack(spacing: 5) {
                            Text(title)
                                .font(.caption.weight(.bold))
                                .foregroundStyle(PadTheme.value)
                            if stubBadge {
                                stubPill
                            }
                        }
                        Text(subtitle)
                            .font(.system(size: 9.5))
                            .foregroundStyle(labelMuted)
                            .lineLimit(1)
                    }

                    Spacer(minLength: 4)

                    Image(systemName: "chevron.right")
                        .font(.system(size: 10, weight: .bold))
                        .foregroundStyle(iconMuted)
                }
                .contentShape(Rectangle())
            }
            .buttonStyle(.plain)

            Toggle("", isOn: isOn)
                .labelsHidden()
                .toggleStyle(.switch)
                .controlSize(.mini)
                .tint(stubBadge ? PadTheme.caution : PadTheme.commanded)
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 7)
        .background(PadTheme.surface, in: RoundedRectangle(cornerRadius: 10, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: 10, style: .continuous)
                .strokeBorder(PadTheme.separator, lineWidth: 1)
        )
    }

    // MARK: - Secondary Submenu Pages

    @ViewBuilder
    private func submenuContent(for page: LayersSubmenuPage) -> some View {
        switch page {
        case .airspace:
            airspaceSubmenu
        case .weather:
            weatherSubmenu
        case .terrainSafety:
            terrainSafetySubmenu
        case .trafficRunways:
            trafficRunwaysSubmenu
        }
    }

    private var airspaceSubmenu: some View {
        VStack(alignment: .leading, spacing: 10) {
            detailToggleRow(
                title: "3D NASR Airspace Volumes",
                subtitle: "Extruded shelves, floors & ceilings",
                isOn: $layers.airspace3D
            )

            VStack(alignment: .leading, spacing: 6) {
                Text("EMPHASIS POLICY")
                    .font(.system(size: 9.5, weight: .bold, design: .monospaced))
                    .foregroundStyle(labelMuted)
                Picker("Policy", selection: $layers.airspacePolicy) {
                    ForEach(AirspacePortrayalPolicy.allCases) { pol in
                        Text(pol.rawValue).tag(pol)
                    }
                }
                .pickerStyle(.segmented)
                Text("IFR de-emphasis keeps Prohibited/Restricted/Warning at full intensity while ghosting Class B/C.")
                    .font(.system(size: 9.5))
                    .foregroundStyle(labelMuted)
            }

            Divider().overlay(Color.white.opacity(0.10))

            detailToggleRow(
                title: "Prohibited / Restricted / Warning",
                subtitle: "P-56, R-areas & offshore warning volumes",
                isOn: $layers.prohibitedRestrictedWarning
            )
            detailToggleRow(
                title: "MOA & Alert Areas",
                subtitle: "Military Operations & high-volume training",
                isOn: $layers.moaAndAlert
            )
            detailToggleRow(
                title: "700 ft Class E5 Transition",
                subtitle: "Rural magenta vignette shelves (off by default)",
                isOn: $layers.classE5Transition
            )
            detailToggleRow(
                title: "Class G Uncontrolled",
                subtitle: "Surface uncontrolled airspace (off by default)",
                isOn: $layers.classG
            )
            stubToggleRow(
                title: "Dynamic Altitude Cap Filter",
                detail: "Hide shelves > 3,000 ft above ownship",
                isOn: $layers.airspaceAltitudeCapStub
            )
        }
    }

    private var weatherSubmenu: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Planned FIS-B (978 UAT) & AviationWeather.gov overlays share this unified layer pipeline:")
                .font(.caption2)
                .foregroundStyle(labelMuted)

            stubToggleRow(
                title: "NEXRAD Composite Radar",
                detail: "CONUS + Regional reflectivity mosaic (5 min)",
                isOn: $layers.nexradRadarStub
            )
            stubToggleRow(
                title: "Flight Categories (METAR / TAF)",
                detail: "VFR / MVFR / IFR / LIFR airport color rings",
                isOn: $layers.flightCategoryMetarStub
            )
            stubToggleRow(
                title: "IR Color-Enhanced Satellite Clouds",
                detail: "GOES cloud-top temperature overlay",
                isOn: $layers.irCloudsStub
            )
            stubToggleRow(
                title: "Winds & Temperatures Aloft",
                detail: "FD/GFS wind barbs at planned cruise altitude",
                isOn: $layers.windsAloftStub
            )
            stubToggleRow(
                title: "AIRMET / SIGMET / Icing / Turbulence",
                detail: "Active convective & G-AIRMET hazard polygons",
                isOn: $layers.airmetSigmetStub
            )
        }
    }

    private var terrainSafetySubmenu: some View {
        VStack(alignment: .leading, spacing: 10) {
            detailToggleRow(
                title: "3D DEM Terrain Relief",
                subtitle: "Global Terrarium elevation mesh (on by default)",
                isOn: $layers.terrain3D
            )
            detailToggleRow(
                title: "Day / Night Solar Terminator",
                subtitle: "Dim night hemisphere on aeronautical charts",
                isOn: $layers.solarTerminator
            )

            Divider().overlay(Color.white.opacity(0.10))

            stubToggleRow(
                title: "Live FAA SWIM Active TFRs",
                detail: "Real-time NOTAM / stadium / VIP polygons",
                isOn: $layers.liveSwimTfrStub
            )
            stubToggleRow(
                title: "FAA DOF Obstacles (>200 ft AGL)",
                detail: "Lighted/unlighted towers & wind turbines",
                isOn: $layers.dofObstaclesStub
            )
            stubToggleRow(
                title: "Terrain-Aware Glide Advisor Ring",
                detail: "Best-glide reach footprint given wind & DEM",
                isOn: $layers.glideAdvisorStub
            )
        }
    }

    private var trafficRunwaysSubmenu: some View {
        VStack(alignment: .leading, spacing: 10) {
            detailToggleRow(
                title: "Master Runways & Route Overlay",
                subtitle: "Show surveyed thresholds & active flight plan",
                isOn: $layers.runwaysAndRoute
            )
            detailToggleRow(
                title: "Airport Runways & Approach Feathers",
                subtitle: "Highlight destination threshold & centerlines",
                isOn: $layers.airportRunways
            )
            detailToggleRow(
                title: "Flight Plan Legs & Waypoint Labels",
                subtitle: "Magenta active leg + clean ForeFlight identifiers",
                isOn: $layers.fplRouteOverlay
            )
            detailToggleRow(
                title: "ADS-B / TIS-B Traffic Targets",
                subtitle: "GDL-90 / Connext / CPA intruders",
                isOn: $layers.adsbTraffic
            )
            stubToggleRow(
                title: "Hide Distant / Ground Traffic",
                detail: "Filter targets > ±3,500 ft or on taxiways",
                isOn: $layers.hideDistantTrafficStub
            )
        }
    }

    private var nonEarthNoticeSection: some View {
        VStack(alignment: .leading, spacing: 6) {
            Label("\(controller.primaryBody.rawValue) Reference Active", systemImage: controller.primaryBody.symbolName)
                .font(.caption.weight(.bold))
                .foregroundStyle(PadTheme.caution)
            Text("Earth-specific FAA charts, NASR airspace, WMM2025 geomagnetic variation, and 1090ES ADS-B layers are suppressed on \(controller.primaryBody.rawValue) (\(controller.primaryBody.figureSummary)). Ephemeris starfield, selenographic/areographic graticules, and planetary SITE waypoints remain active.")
                .font(.caption2)
                .foregroundStyle(PadTheme.label)
        }
    }

    private var activeScaleDescription: (text: String, tint: Color) {
        let authority: AeroDataAuthority? = switch layers.aeroChart {
        case .none: nil
        case .usVfr: .faaVfrSectional
        case .ifrLow: .faaIfrLow
        case .ifrHigh: .faaIfrHigh
        }
        if let authority {
            switch controller.dataPackStore.validity(for: authority) {
            case .valid:
                break
            case .expiredInvalid:
                return (String(format: "z%.1f · EXPIRED INVALID", effectiveZoom), PadTheme.warning)
            case .notYetEffective:
                return (String(format: "z%.1f · NOT YET EFFECTIVE", effectiveZoom), PadTheme.caution)
            case .notDownloaded:
                return (String(format: "z%.1f · NOT DOWNLOADED", effectiveZoom), PadTheme.caution)
            }
        }
        switch layers.aeroChart {
        case .none:
            return (String(format: "z%.1f · %@", effectiveZoom, layers.baseSurface.rawValue.uppercased()), PadTheme.label)
        case .usVfr:
            let label = effectiveZoom >= 9.6
                ? String(format: "z%.1f · 1:250k TAC", effectiveZoom)
                : String(format: "z%.1f · 1:500k SEC", effectiveZoom)
            return (label, PadTheme.label)
        case .ifrLow:
            return (String(format: "z%.1f · IFR LOW", effectiveZoom), PadTheme.label)
        case .ifrHigh:
            return (String(format: "z%.1f · IFR HIGH", effectiveZoom), PadTheme.label)
        }
    }

    private func chartPackPill(for chart: EarthAeroChart) -> (text: String, tint: Color)? {
        let store = controller.dataPackStore
        let val: AeroPackValidity
        switch chart {
        case .none:
            return nil
        case .usVfr:
            val = store.validity(for: .faaVfrSectional)
        case .ifrLow:
            val = store.validity(for: .faaIfrLow)
        case .ifrHigh:
            val = store.validity(for: .faaIfrHigh)
        }
        switch val {
        case .valid(let pack, _, _):
            return (pack.airacCode, PadTheme.active)
        case .expiredInvalid:
            return ("INVALID", PadTheme.warning)
        case .notYetEffective:
            return ("PENDING", PadTheme.caution)
        case .notDownloaded:
            return ("DL REQ", PadTheme.caution)
        }
    }

    private var stubPill: some View {
        Text("STUB")
            .font(.system(size: 8, weight: .heavy, design: .monospaced))
            .padding(.horizontal, 4)
            .padding(.vertical, 1)
            .background(PadTheme.caution.opacity(0.20), in: Capsule())
            .foregroundStyle(PadTheme.caution)
    }

    private func detailToggleRow(title: String, subtitle: String, isOn: Binding<Bool>) -> some View {
        Toggle(isOn: isOn) {
            VStack(alignment: .leading, spacing: 1) {
                Text(title)
                    .font(.caption.weight(.semibold))
                    .foregroundStyle(PadTheme.value)
                Text(subtitle)
                    .font(.system(size: 9.5))
                    .foregroundStyle(labelMuted)
            }
        }
        .toggleStyle(.switch)
        .tint(PadTheme.commanded)
        .controlSize(.mini)
    }

    private func stubToggleRow(title: String, detail: String, isOn: Binding<Bool>) -> some View {
        HStack(spacing: 8) {
            VStack(alignment: .leading, spacing: 1) {
                HStack(spacing: 5) {
                    Text(title)
                        .font(.caption.weight(.semibold))
                        .foregroundStyle(PadTheme.value)
                    stubPill
                }
                Text(detail)
                    .font(.system(size: 9.5))
                    .foregroundStyle(labelMuted)
            }
            Spacer()
            Toggle("", isOn: isOn)
                .labelsHidden()
                .toggleStyle(.switch)
                .controlSize(.mini)
                .tint(PadTheme.caution)
        }
    }
}

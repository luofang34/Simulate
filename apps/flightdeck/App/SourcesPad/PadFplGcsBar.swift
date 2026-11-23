import SwiftUI
import FlightDeckKit

/// Bottom EFB Flight Plan & `Sokoly` GCS command bar (matching `Pilotage` iPadOS placement
/// and `Sokoly-App` control-lease + `AvionicsColorSystem` human-factors semantics), hidden automatically
/// when a modal planner sheet is open.
struct PadBottomFplGcsBar: View {
    @Bindable var controller: FlightDeckController
    @Binding var primaryMode: PadPrimaryViewMode
    @Binding var orbit: OrbitCameraState
    @Binding var isFplSheetPresented: Bool
    @Binding var isReceiverSheetPresented: Bool

    var body: some View {
        VStack(spacing: 8) {
            if controller.isPreviewingTrack {
                kmlScrubberStrip
            }
            mainDockRow
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 8)
        .frame(maxWidth: 1080)
        .cockpitFloatingChrome(cornerRadius: 18)
    }

    // MARK: - Main Bottom Bar Row (Receiver Link | Route Progression & Metrics | Live Telemetry)

    private var mainDockRow: some View {
        let own = controller.latestFrame?.ownship
        let summary = controller.compactFlightPlanSummary(from: own)
        let datumTag = controller.celestialBody.altitudeDatumTag
        let altText: String = {
            guard let altM = own?.altitudeMslM else { return "----" }
            if controller.primaryBody == .earth {
                return String(format: "%.0f FT", altM * 3.28084)
            } else {
                return String(format: "%.0f M", altM)
            }
        }()

        return HStack(spacing: 10) {
            Button {
                isReceiverSheetPresented = true
            } label: {
                HStack(spacing: 5) {
                    Circle()
                        .fill(linkStatusColor)
                        .frame(width: 7, height: 7)
                    Image(systemName: controller.gcsState.receiverFamily.symbolName)
                        .font(.caption.weight(.semibold))
                        .foregroundStyle(PadTheme.label)
                    Text(controller.gcsState.receiverFamily.badgeLabel)
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .foregroundStyle(PadTheme.value)
                }
                .padding(.horizontal, 10)
                .padding(.vertical, 8)
                .background(PadTheme.surface, in: Capsule())
                .overlay(Capsule().strokeBorder(PadTheme.separator, lineWidth: 1))
            }
            .buttonStyle(.plain)
            .accessibilityLabel("Avionics Receiver and Sensor Link")

            Button {
                isFplSheetPresented = true
            } label: {
                minimizedFplStripContent(summary: summary)
                    .padding(.horizontal, 12)
                    .padding(.vertical, 6)
                    .background(PadTheme.surface, in: Capsule())
                    .overlay(Capsule().strokeBorder(PadTheme.separator, lineWidth: 1))
            }
            .buttonStyle(.plain)
            .accessibilityLabel("Open Flight Plan and GCS Drawer")

            // Quiet Dark Cockpit (`FAA AC 25-11B`, `PRODUCT-UI.md §2`): all nominal measured flight
            // values render in high-contrast `PadTheme.value` (`#ECF0F3`, 16.97:1) with `PadTheme.label` headers.
            HStack(spacing: 11) {
                telemetryItem(
                    label: "GS",
                    value: own?.groundSpeedKt.map { String(format: "%.0f KT", $0) } ?? "---"
                )
                telemetryItem(
                    label: "ALT (\(datumTag))",
                    value: altText
                )
                telemetryItem(
                    label: "TRK",
                    value: (own?.groundTrackTrueDeg ?? own?.trueHeadingDeg)
                        .map { String(format: "%03.0f°", $0) } ?? "---°"
                )
            }
            .padding(.horizontal, 4)
        }
    }

    @ViewBuilder
    private func minimizedFplStripContent(summary: CompactFlightPlanSummary) -> some View {
        HStack(spacing: 8) {
            if controller.isPreviewingTrack, !controller.kmlPlayback.route.isEmpty {
                Text(controller.kmlPlayback.route)
                    .font(.system(size: 12.5, weight: .bold, design: .monospaced))
                    .foregroundStyle(PadTheme.value)
                    .lineLimit(1)
            } else if summary.hasRoute, let current = summary.currentIdent {
                HStack(spacing: 5) {
                    if let orig = summary.originIdent {
                        Text(orig)
                            .font(.system(size: 12, weight: .bold, design: .monospaced))
                            .foregroundStyle(summary.hasSequencedGapAfterOrigin ? PadTheme.label : PadTheme.value)
                        Text(summary.hasSequencedGapAfterOrigin ? "⋯" : "▸")
                            .font(.system(size: 10, weight: .bold, design: .monospaced))
                            .foregroundStyle(summary.hasSequencedGapAfterOrigin ? PadTheme.label : PadTheme.plan)
                    }

                    // Active GPS `TO` waypoint in Aviation Guidance Magenta (`#E850E8`) + redundant `▸` + capsule border.
                    HStack(spacing: 3) {
                        if summary.originIdent == nil || summary.hasSequencedGapAfterOrigin {
                            Text("▸")
                                .font(.system(size: 9.5, weight: .heavy, design: .monospaced))
                        }
                        Text(current)
                            .font(.system(size: 12, weight: .heavy, design: .monospaced))
                    }
                    .padding(.horizontal, 7)
                    .padding(.vertical, 2)
                    .background(PadTheme.plan.opacity(0.18), in: Capsule())
                    .overlay(Capsule().strokeBorder(PadTheme.plan.opacity(0.65), lineWidth: 1))
                    .foregroundStyle(PadTheme.plan)

                    if let next = summary.nextIdent {
                        Text("→")
                            .font(.system(size: 10.5, weight: .bold, design: .monospaced))
                            .foregroundStyle(PadTheme.label)
                        Text(next)
                            .font(.system(size: 12, weight: .bold, design: .monospaced))
                            .foregroundStyle(PadTheme.commanded)
                    }

                    if let dest = summary.destinationIdent {
                        Text("⋯")
                            .font(.system(size: 10.5, weight: .bold, design: .monospaced))
                            .foregroundStyle(PadTheme.label)
                        Text(dest)
                            .font(.system(size: 12, weight: .bold, design: .monospaced))
                            .foregroundStyle(PadTheme.value)
                    }

                    if let rwy = summary.runwayBadge {
                        Text("RW\(rwy)")
                            .font(.system(size: 9.5, weight: .heavy, design: .monospaced))
                            .padding(.horizontal, 5)
                            .padding(.vertical, 2)
                            .background(PadTheme.control, in: Capsule())
                            .overlay(Capsule().strokeBorder(PadTheme.commanded.opacity(0.55), lineWidth: 1))
                            .foregroundStyle(PadTheme.commanded)
                    }
                }
                .lineLimit(1)
                .minimumScaleFactor(0.85)
            } else {
                HStack(spacing: 5) {
                    Image(systemName: "point.bottomleft.forward.to.arrow.triangle.scurvepath.fill")
                        .font(.system(size: 11, weight: .semibold))
                        .foregroundStyle(PadTheme.label)
                    Text("Plan Route")
                        .font(.system(size: 12, weight: .semibold))
                        .foregroundStyle(PadTheme.label)
                }
            }

            Spacer(minLength: 4)

            if summary.hasRoute {
                HStack(spacing: 9) {
                    if let brg = summary.bearingTrueDeg {
                        fplMiniMetric(
                            label: summary.usesLiveGroundSpeed ? "BRG" : "CRS",
                            value: String(format: "%03.0f°", brg),
                            tint: PadTheme.plan
                        )
                    }
                    if let legNm = summary.legDistanceNm {
                        let distStr: String = {
                            if summary.hasRemainingLegsBeyondCurrent, let remNm = summary.remainingDistanceNm {
                                return "\(Self.formatCompactNm(legNm))/\(Self.formatCompactNm(remNm)) NM"
                            }
                            return "\(Self.formatCompactNm(legNm)) NM"
                        }()
                        fplMiniMetric(label: "DIST", value: distStr, tint: PadTheme.value)
                    }
                    if let ete = summary.eteFormatted {
                        fplMiniMetric(label: "ETE", value: ete, tint: PadTheme.value)
                    }
                    if let fuel = summary.fuelRequiredGal {
                        let fuelStr = fuel < 10.0
                            ? String(format: "%.1fG", fuel)
                            : String(format: "%.0fG", fuel)
                        fplMiniMetric(label: "FUEL", value: fuelStr, tint: PadTheme.value)
                    }
                }
            }

            Text(gcsAuthorityShortBadge)
                .font(.system(size: 9, weight: .heavy, design: .monospaced))
                .padding(.horizontal, 6)
                .padding(.vertical, 2)
                .background(gcsAuthorityBackground, in: Capsule())
                .foregroundStyle(gcsAuthorityColor)

            Image(systemName: "chevron.up")
                .font(.system(size: 9.5, weight: .bold))
                .foregroundStyle(PadTheme.label)
        }
    }

    private func fplMiniMetric(label: String, value: String, tint: Color) -> some View {
        VStack(alignment: .leading, spacing: 0) {
            Text(label)
                .font(.system(size: 7.5, weight: .bold, design: .monospaced))
                .foregroundStyle(PadTheme.label)
            Text(value)
                .font(.system(size: 10.5, weight: .bold, design: .monospaced))
                .foregroundStyle(tint)
                .lineLimit(1)
        }
    }

    private static func formatCompactNm(_ nm: Double) -> String {
        if nm < 10.0 {
            return String(format: "%.1f", nm)
        }
        return String(format: "%.0f", nm)
    }

    // MARK: - Integrated KML / GPX Preview Scrubber Row

    private var kmlScrubberStrip: some View {
        let kml = controller.kmlPlayback
        return HStack(spacing: 10) {
            Text("PREVIEW")
                .font(.system(size: 9.5, weight: .heavy, design: .monospaced))
                .padding(.horizontal, 6)
                .padding(.vertical, 2)
                .background(PadTheme.commanded.opacity(0.18), in: Capsule())
                .foregroundStyle(PadTheme.commanded)

            Button {
                controller.setKmlPlayback(
                    paused: !kml.paused,
                    speedMultiplier: max(kml.speedMultiplier, 0.5)
                )
            } label: {
                Image(systemName: kml.paused ? "play.fill" : "pause.fill")
                    .font(.caption.bold())
                    .frame(width: 26, height: 22)
            }
            .buttonStyle(.borderedProminent)
            .tint(kml.paused ? PadTheme.active : PadTheme.control)
            .controlSize(.mini)

            Slider(
                value: Binding(
                    get: { kml.progressFraction },
                    set: { frac in controller.seekKml(fraction: frac) }
                ),
                in: 0.0...1.0
            )
            .tint(PadTheme.commanded)

            Text("\(formatDuration(kml.elapsedSeconds))/\(formatDuration(kml.durationSeconds))")
                .font(.system(size: 10.5, weight: .medium, design: .monospaced))
                .foregroundStyle(PadTheme.value)
                .frame(width: 82, alignment: .trailing)

            ForEach([Float(1), Float(2), Float(5), Float(10), Float(20)], id: \.self) { spd in
                let selected = abs(kml.speedMultiplier - spd) < 0.1
                Button("\(Int(spd))×") {
                    controller.setKmlPlayback(paused: kml.paused, speedMultiplier: spd)
                }
                .buttonStyle(.bordered)
                .controlSize(.mini)
                .tint(selected ? PadTheme.commanded : PadTheme.label)
                .font(.system(size: 9.5, weight: .bold, design: .monospaced))
            }

            Button {
                controller.closeKmlPreview()
                primaryMode = .map
                orbit = .wholeGlobeDefault
            } label: {
                Image(systemName: "xmark.circle.fill")
                    .foregroundStyle(PadTheme.label)
            }
            .buttonStyle(.plain)
            .accessibilityLabel("Close Track Preview")
        }
    }

    private var linkStatusColor: Color {
        if controller.isPreviewingTrack { return PadTheme.commanded }
        switch controller.gcsState.receiverFamily {
        case .wifiGdl90: return controller.isGdl90Listening ? PadTheme.active : PadTheme.caution
        case .usbAdsBStub: return PadTheme.caution
        case .sokolyGcsTcp: return controller.isConnextConnected ? PadTheme.active : PadTheme.caution
        case .simulationBench: return PadTheme.label
        }
    }

    private var gcsAuthorityShortBadge: String {
        if controller.isPreviewingTrack { return "REPLAY" }
        if controller.gcsState.canActuate(isKmlPreview: false) {
            let armTag = controller.gcsState.armTelegraphArmed ? "ARM" : "SAFE"
            return "LEASE · \(armTag)"
        }
        return "OBS"
    }

    private var gcsAuthorityColor: Color {
        if controller.isPreviewingTrack { return PadTheme.commanded }
        if controller.gcsState.canActuate(isKmlPreview: false) {
            return controller.gcsState.armTelegraphArmed ? PadTheme.caution : PadTheme.active
        }
        return PadTheme.label
    }

    private var gcsAuthorityBackground: Color {
        if controller.isPreviewingTrack { return PadTheme.commanded.opacity(0.18) }
        if controller.gcsState.canActuate(isKmlPreview: false) {
            return (controller.gcsState.armTelegraphArmed ? PadTheme.caution : PadTheme.active).opacity(0.20)
        }
        return PadTheme.control
    }

    private func telemetryItem(label: String, value: String) -> some View {
        VStack(alignment: .leading, spacing: 1) {
            Text(label)
                .font(.system(size: 8.5, weight: .semibold, design: .monospaced))
                .foregroundStyle(PadTheme.label)
            Text(value)
                .font(.system(size: 12, weight: .bold, design: .monospaced))
                .foregroundStyle(PadTheme.value)
        }
    }

    private func formatDuration(_ sec: Double) -> String {
        let total = max(Int(sec.rounded()), 0)
        return String(format: "%02d:%02d", total / 60, total % 60)
    }
}

/// Interactive EFB Flight Plan bubble editor + Typed NavLog + Destination 3D Runway + `Sokoly` GCS sheet.
struct PadFplAndGcsSheet: View {
    @Bindable var controller: FlightDeckController
    @Binding var primaryMode: PadPrimaryViewMode
    @Binding var orbit: OrbitCameraState
    var initialDraftQuery: String = ""
    var onInspectPlate: (PresentedPlateRequest) -> Void = { _ in }
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationStack {
            ScrollView {
                VStack(alignment: .leading, spacing: 16) {
                    PadRouteFlowEditor(
                        controller: controller,
                        primaryMode: $primaryMode,
                        orbit: $orbit,
                        initialDraftQuery: initialDraftQuery,
                        onInspectPlate: onInspectPlate,
                        onPinnedPlateToMap: { pinned in
                            focusCameraOnPinnedPlate(pinned)
                            dismiss()
                        }
                    )

                    if controller.primaryBody == .earth {
                        cruisePerformanceStrip
                        PadProcedurePickerCard(
                            controller: controller,
                            onInspectPlate: onInspectPlate,
                            onPinnedPlateToMap: { pinned in
                                focusCameraOnPinnedPlate(pinned)
                                dismiss()
                            }
                        )
                    }

                    if !controller.flightPlanWaypoints.isEmpty {
                        routeNavLogSection
                    }

                    if controller.primaryBody == .earth {
                        destinationRunwaySection
                    }

                    corridorBookmarksSection
                    sokolyGcsControlSection
                }
                .padding(16)
            }
            .background(PadTheme.background)
            .navigationTitle("Flight Plan & Sokoly GCS")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .topBarTrailing) {
                    Button("Done") { dismiss() }
                        .tint(PadTheme.commanded)
                }
            }
        }
    }

    private var cruisePerformanceStrip: some View {
        HStack(spacing: 14) {
            Label("Cruise Plan", systemImage: "gauge.with.dots.needle.33percent")
                .font(.caption.weight(.bold))
                .foregroundStyle(PadTheme.label)

            Spacer()

            HStack(spacing: 6) {
                Text("KTAS")
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                    .foregroundStyle(PadTheme.label)
                Stepper(
                    String(format: "%.0f KT", controller.plannedCruiseKtas),
                    value: $controller.plannedCruiseKtas,
                    in: 45...480,
                    step: 5
                )
                .font(.system(size: 11.5, weight: .bold, design: .monospaced))
                .foregroundStyle(PadTheme.value)
                .fixedSize()
            }

            HStack(spacing: 6) {
                Text("FLOW")
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                    .foregroundStyle(PadTheme.label)
                Stepper(
                    String(format: "%.1f GPH", controller.plannedFuelFlowGph),
                    value: $controller.plannedFuelFlowGph,
                    in: 2.0...180.0,
                    step: 0.5
                )
                .font(.system(size: 11.5, weight: .bold, design: .monospaced))
                .foregroundStyle(PadTheme.value)
                .fixedSize()
            }
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 9)
        .cockpitRaisedCard(cornerRadius: 12)
    }

    private func focusCameraOnPinnedPlate(_ pinned: PinnedMapPlate) {
        orbit = OrbitCameraState(
            latitude: pinned.centerLatitudeDeg,
            longitude: pinned.centerLongitudeDeg,
            zoom: pinned.descriptor.kind == .airportDiagram ? 13.4 : 10.6,
            pitchDeg: 0.0,
            bearingDeg: 0.0
        )
        primaryMode = .map
    }

    // MARK: - Typed Waypoint NavLog Leg Table (State-Driven Color + Non-Color Geometric Badges)

    private struct PadNavLogRow: Identifiable {
        let id: UUID
        let index: Int
        let waypoint: FlightPlanWaypoint
        let isActive: Bool
        let isPast: Bool
        let leg: RouteLegMetrics?
    }

    private var routeNavLogSection: some View {
        let bodyR = controller.bodyParameters.renderRadiusMeters
        let waypoints = controller.flightPlanWaypoints
        let activeIdx = controller.activeWaypointIndex
        let clampedActiveIdx = waypoints.count >= 2
            ? min(max(activeIdx, 1), waypoints.count - 1)
            : max(activeIdx, 0)
        let rows: [PadNavLogRow] = waypoints.enumerated().map { idx, wp in
            let leg: RouteLegMetrics? = if idx > 0 {
                RouteLegMetrics.between(
                    lat1Deg: waypoints[idx - 1].latitudeDeg,
                    lon1Deg: waypoints[idx - 1].longitudeDeg,
                    lat2Deg: wp.latitudeDeg,
                    lon2Deg: wp.longitudeDeg,
                    radiusMeters: bodyR
                )
            } else {
                nil
            }
            return PadNavLogRow(
                id: wp.id,
                index: idx,
                waypoint: wp,
                isActive: idx == activeIdx,
                isPast: waypoints.count >= 2 && idx + 1 < clampedActiveIdx && idx != activeIdx,
                leg: leg
            )
        }

        return VStack(alignment: .leading, spacing: 6) {
            Text("WAYPOINT NAVLOG (COURSE · LEG DISTANCE)")
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .foregroundStyle(PadTheme.label)

            ForEach(rows) { row in
                let idx = row.index
                let wp = row.waypoint
                let isActive = row.isActive
                let isPast = row.isPast
                let leg = row.leg
                let identColor: Color = isActive ? PadTheme.plan : (isPast ? PadTheme.disabled : PadTheme.value)
                let badgeFg: Color = isActive ? PadTheme.plan : (isPast ? PadTheme.disabled : PadTheme.label)

                HStack(spacing: 8) {
                    Text(isActive ? "▸\(String(format: "%02d", idx + 1))" : String(format: "%02d", idx + 1))
                        .font(.system(size: 10, weight: .bold, design: .monospaced))
                        .foregroundStyle(isActive ? PadTheme.plan : (isPast ? PadTheme.disabled : PadTheme.label))
                        .frame(width: 26, alignment: .trailing)

                    if wp.kind == .airport {
                        Image(systemName: "airplane.circle.fill")
                            .font(.system(size: 14, weight: .semibold))
                            .foregroundStyle(isActive ? PadTheme.plan : (isPast ? PadTheme.disabled : PadTheme.value))
                            .frame(width: 52)
                    } else {
                        Text(wp.kind.badge)
                            .font(.system(size: 9.5, weight: .heavy, design: .monospaced))
                            .frame(width: 52)
                            .padding(.vertical, 3)
                            .background(PadTheme.control, in: RoundedRectangle(cornerRadius: 6, style: .continuous))
                            .foregroundStyle(badgeFg)
                    }

                    Text(wp.ident)
                        .font(.system(size: 13, weight: .bold, design: .monospaced))
                        .foregroundStyle(identColor)
                        .frame(minWidth: 54, alignment: .leading)

                    if !wp.airwayOrProcedureTag.isEmpty {
                        Text(wp.airwayOrProcedureTag)
                            .font(.system(size: 9.5, weight: .heavy, design: .monospaced))
                            .padding(.horizontal, 6)
                            .padding(.vertical, 2)
                            .background(PadTheme.control, in: Capsule())
                            .foregroundStyle(isPast ? PadTheme.disabled : PadTheme.label)
                    }

                    // Published MEA / procedure altitude constraints are nominal chart reference data —
                    // never Caution Orange/Amber (`FAA AC 25-11B §5.4`).
                    if !wp.altitudeConstraintTag.isEmpty {
                        Text(wp.altitudeConstraintTag)
                            .font(.system(size: 9.5, weight: .bold, design: .monospaced))
                            .padding(.horizontal, 6)
                            .padding(.vertical, 2)
                            .background(PadTheme.control, in: Capsule())
                            .overlay(Capsule().strokeBorder(PadTheme.separator, lineWidth: 1))
                            .foregroundStyle(isPast ? PadTheme.disabled : PadTheme.value)
                            .lineLimit(1)
                    } else {
                        Text(wp.name)
                            .font(.caption)
                            .foregroundStyle(isPast ? PadTheme.disabled : PadTheme.label)
                            .lineLimit(1)
                    }

                    Spacer(minLength: 4)

                    if let leg {
                        Text(String(format: "%03.0f°T · %.1f NM", leg.initialBearingTrueDeg, leg.distanceNauticalMiles))
                            .font(.system(size: 10.5, weight: .semibold, design: .monospaced))
                            .foregroundStyle(isActive ? PadTheme.plan : (isPast ? PadTheme.disabled : PadTheme.value))
                    } else {
                        Text("ORIGIN")
                            .font(.system(size: 9.5, weight: .bold, design: .monospaced))
                            .foregroundStyle(PadTheme.label)
                    }

                    Button(isActive ? "ACTIVE" : "DTO") {
                        controller.activateDirectTo(index: idx)
                    }
                    .buttonStyle(.bordered)
                    .controlSize(.mini)
                    .tint(isActive ? PadTheme.plan : PadTheme.label)
                    .font(.system(size: 9.5, weight: .bold, design: .monospaced))
                }
                .padding(.horizontal, 10)
                .padding(.vertical, 6)
                .background(
                    isActive ? PadTheme.plan.opacity(0.14) : PadTheme.background.opacity(0.55),
                    in: RoundedRectangle(cornerRadius: 9, style: .continuous)
                )
                .overlay(
                    RoundedRectangle(cornerRadius: 9, style: .continuous)
                        .strokeBorder(isActive ? PadTheme.plan.opacity(0.55) : PadTheme.separator.opacity(0.65), lineWidth: 1)
                )
            }
        }
        .padding(12)
        .cockpitRaisedCard(cornerRadius: 14)
    }

    @ViewBuilder
    private var destinationRunwaySection: some View {
        if let dest = controller.destinationAirportWaypoint, !dest.runways.isEmpty {
            VStack(alignment: .leading, spacing: 8) {
                HStack {
                    Text("Arrival Runway (\(dest.ident))")
                        .font(.caption.weight(.bold))
                        .foregroundStyle(PadTheme.label)
                    Spacer()
                    Button {
                        orbit = OrbitCameraState(
                            latitude: dest.latitudeDeg,
                            longitude: dest.longitudeDeg,
                            zoom: 13.6,
                            pitchDeg: 66.0,
                            bearingDeg: orbit.bearingDeg
                        )
                        primaryMode = .map
                        dismiss()
                    } label: {
                        Label("Inspect 3D Threshold", systemImage: "view.3d")
                            .font(.caption.weight(.semibold))
                    }
                    .buttonStyle(.bordered)
                    .tint(PadTheme.commanded)
                    .controlSize(.small)
                }

                ScrollView(.horizontal, showsIndicators: false) {
                    HStack(spacing: 6) {
                        ForEach(dest.runways, id: \.self) { rwy in
                            let selected = (controller.destinationRunway == rwy)
                            Button("RWY \(rwy)") {
                                controller.selectDestinationRunway(rwy)
                            }
                            .buttonStyle(.bordered)
                            .controlSize(.small)
                            .tint(selected ? PadTheme.commanded : PadTheme.label)
                            .font(.caption.monospaced().bold())
                        }
                    }
                }
            }
            .padding(12)
            .cockpitRaisedCard(cornerRadius: 14)
        }
    }

    private var corridorBookmarksSection: some View {
        let bookmarks = NavigationBookmark.forPrimaryBody(controller.primaryBody)
        return VStack(alignment: .leading, spacing: 8) {
            Text("OPERATIONAL CORRIDORS & MULTI-TYPE ROUTES")
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .foregroundStyle(PadTheme.label)

            LazyVGrid(columns: [GridItem(.flexible()), GridItem(.flexible())], spacing: 8) {
                ForEach(bookmarks) { bm in
                    Button {
                        if !bm.routeCodes.isEmpty {
                            controller.setFlightPlanCodes(bm.routeCodes)
                        }
                        orbit = bm.state
                        primaryMode = .map
                    } label: {
                        VStack(alignment: .leading, spacing: 2) {
                            Text(bm.title)
                                .font(.caption.monospaced().bold())
                                .foregroundStyle(PadTheme.value)
                                .lineLimit(1)
                            Text(bm.subtitle)
                                .font(.system(size: 9.5))
                                .foregroundStyle(PadTheme.label)
                                .lineLimit(1)
                        }
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .padding(9)
                        .background(PadTheme.control, in: RoundedRectangle(cornerRadius: 10, style: .continuous))
                        .overlay(
                            RoundedRectangle(cornerRadius: 10, style: .continuous)
                                .strokeBorder(PadTheme.separator, lineWidth: 1)
                        )
                    }
                    .buttonStyle(.plain)
                }
            }
        }
    }

    private var sokolyGcsControlSection: some View {
        let canActuate = controller.gcsState.canActuate(isKmlPreview: controller.isPreviewingTrack)
        let gateNote = controller.gcsState.gatingReason(isKmlPreview: controller.isPreviewingTrack)

        return VStack(alignment: .leading, spacing: 10) {
            HStack {
                Label("Sokoly GCS Control Plane", systemImage: "dot.radiowaves.left.and.right")
                    .font(.subheadline.weight(.bold))
                    .foregroundStyle(PadTheme.value)
                Spacer()
                Text(canActuate ? "ACTIVE LEASE" : "OBSERVING")
                    .font(.system(size: 10, weight: .heavy, design: .monospaced))
                    .padding(.horizontal, 7)
                    .padding(.vertical, 3)
                    .background(canActuate ? PadTheme.active.opacity(0.20) : PadTheme.control, in: Capsule())
                    .foregroundStyle(canActuate ? PadTheme.active : PadTheme.label)
            }

            HStack(spacing: 12) {
                Toggle(
                    "Acquire Control Lease",
                    isOn: Binding(
                        get: { controller.gcsState.hasControlLease },
                        set: { want in
                            if controller.gcsState.receiverFamily.supportsControlUplink, !controller.isPreviewingTrack {
                                controller.gcsState.hasControlLease = want
                                if !want { controller.gcsState.armTelegraphArmed = false }
                            }
                        }
                    )
                )
                .disabled(!controller.gcsState.receiverFamily.supportsControlUplink || controller.isPreviewingTrack)
                .font(.caption.weight(.semibold))
                .tint(PadTheme.active)

                Toggle("ARM Telegraph", isOn: $controller.gcsState.armTelegraphArmed)
                    .disabled(!canActuate)
                    .font(.caption.weight(.semibold))
                    .tint(PadTheme.caution)
            }

            HStack(spacing: 8) {
                ForEach(["NAV FPL", "HOLD", "ALT HOLD", "RTL"], id: \.self) { cmd in
                    let engaged = (controller.gcsState.activeGuidanceMode == cmd)
                    Button(cmd) {
                        controller.executeGcsCommand(cmd)
                    }
                    .disabled(!canActuate)
                    .buttonStyle(.borderedProminent)
                    .controlSize(.small)
                    .tint(engaged ? PadTheme.active : PadTheme.control)
                    .font(.caption.monospaced().bold())
                }
            }

            Text(gateNote ?? controller.gcsState.lastCommandNote)
                .font(.caption2.monospaced())
                .foregroundStyle(gateNote != nil ? PadTheme.caution : PadTheme.label)
        }
        .padding(12)
        .cockpitRaisedCard(cornerRadius: 14)
    }
}

/// Dedicated Avionics Receiver, USB ADS-B Stub, Sokoly TCP Link, and Earth Vertical Datum sheet.
struct PadReceiverAndDatumSheet: View {
    @Bindable var controller: FlightDeckController
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationStack {
            Form {
                Section("Avionics / ADS-B / GCS Link Family") {
                    Picker("Link Family", selection: Binding(
                        get: { controller.gcsState.receiverFamily },
                        set: { controller.selectReceiverFamily($0) }
                    )) {
                        ForEach(AvionicsReceiverFamily.allCases) { fam in
                            Label(fam.rawValue, systemImage: fam.symbolName).tag(fam)
                        }
                    }
                    .pickerStyle(.inline)
                }

                switch controller.gcsState.receiverFamily {
                case .wifiGdl90:
                    Section("Wi-Fi GDL-90 ADS-B & AHRS Receiver") {
                        HStack {
                            Text("UDP Port")
                            Spacer()
                            Text(":\(controller.gdl90Port)")
                                .font(.body.monospaced())
                                .foregroundStyle(PadTheme.value)
                        }
                        Button(controller.isGdl90Listening ? "Stop UDP :4000 Listener" : "Start UDP :4000 Listener") {
                            controller.toggleGdl90Listener()
                        }
                        .tint(controller.isGdl90Listening ? PadTheme.caution : PadTheme.active)
                        Text("Compatible with ForeFlight Sentry, Stratux, Garmin GDL 50/52, and SkyEcho 2.")
                            .font(.caption)
                            .foregroundStyle(PadTheme.label)
                    }
                case .usbAdsBStub:
                    Section("USB-C Dual-Band ADS-B (DriverKit Stub)") {
                        Label("Planned USB-C SDR / Serial Transport", systemImage: "cable.connector.horizontal")
                            .foregroundStyle(PadTheme.caution)
                        Text("Architecture reserved for external USB-C 1090ES + 978 UAT receivers via iPadOS DriverKit / ExternalAccessory. Passive receive-only surveillance semantics are enforced (`Control Lease` disabled).")
                            .font(.caption)
                            .foregroundStyle(PadTheme.label)
                    }
                case .sokolyGcsTcp:
                    Section("Sokoly GCS / Connext Avionics Link") {
                        TextField("Bridge Host / IP", text: $controller.connextHost)
                            .font(.body.monospaced())
                        Button(controller.isConnextConnected ? "Disconnect TCP" : "Connect TCP :\(controller.connextPort)") {
                            controller.toggleConnextConnection()
                        }
                        .tint(controller.isConnextConnected ? PadTheme.caution : PadTheme.active)
                        Text("Bidirectional GNSS, AHRS, 1090ES traffic, flight-plan sync, and leased GCS vehicle control.")
                            .font(.caption)
                            .foregroundStyle(PadTheme.label)
                    }
                case .simulationBench:
                    Section("Simulation & Recorded Hardware Profiles") {
                        Picker("Profile", selection: $controller.sourceMode) {
                            Text(TelemetrySourceMode.njCorridorCruise.title).tag(TelemetrySourceMode.njCorridorCruise)
                            Text(TelemetrySourceMode.highDensityTrafficCpa.title).tag(TelemetrySourceMode.highDensityTrafficCpa)
                            Text(TelemetrySourceMode.kewrApproachWithTfr.title).tag(TelemetrySourceMode.kewrApproachWithTfr)
                            Text(TelemetrySourceMode.connextReplay.title).tag(TelemetrySourceMode.connextReplay)
                        }
                    }
                }

                if controller.primaryBody == .earth {
                    Section("Earth Vertical Altitude Datum") {
                        Picker("Altitude Datum", selection: Binding(
                            get: { controller.earthVerticalDatum },
                            set: { controller.setEarthVerticalDatum($0) }
                        )) {
                            ForEach(EarthVerticalDatum.allCases) { datum in
                                VStack(alignment: .leading) {
                                    Text(datum.title)
                                    Text(datum.subtitle)
                                        .font(.caption2)
                                        .foregroundStyle(PadTheme.label)
                                }
                                .tag(datum)
                            }
                        }
                        .pickerStyle(.inline)
                    }
                }
            }
            .navigationTitle("Receivers & Datum")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .topBarTrailing) {
                    Button("Done") { dismiss() }
                        .tint(PadTheme.commanded)
                }
            }
        }
    }
}

import SwiftUI
import FlightDeckKit

/// Pilot-centric Terminal Procedures & Approach Plates selector (`Approach` · `Arrival` · `Departure` · `Airport Plates`),
/// supporting georeferenced `Plate on Map`, full-page `View Plate` (`PDFKit`), `Add to Route`, and multi-service
/// chart provider selection (`FAA AeroNav` offline + reserved `Jeppesen`, `Lido mPilot`, `EUROCONTROL`, `Navigraph` stubs).
struct PadProcedurePickerCard: View {
    private enum PickerTab: String, CaseIterable, Identifiable {
        case approach = "Approach"
        case arrival = "Arrival"
        case departure = "Departure"
        case airportPlates = "Plates"

        var id: String { rawValue }
    }

    @Bindable var controller: FlightDeckController
    let onInspectPlate: (PresentedPlateRequest) -> Void
    let onPinnedPlateToMap: (PinnedMapPlate) -> Void

    @State private var activeTab: PickerTab = .approach
    @State private var selectedProcedureId: String?
    @State private var selectedStandalonePlateId: String?
    @State private var selectedRunwayTransition: String?
    @State private var selectedEnrouteOrFeederTransition: String?
    @State private var includeMissedApproach: Bool = false
    @State private var statusBanner: String?

    var body: some View {
        let validity = controller.cifpValidity
        let isUsable = controller.isCifpUsable

        VStack(alignment: .leading, spacing: 10) {
            headerBar(validity: validity, isUsable: isUsable)

            if let stubNote = controller.chartProviders.stubFallbackNotice {
                HStack(spacing: 6) {
                    Image(systemName: "link.badge.plus")
                        .font(.caption2)
                        .foregroundStyle(PadTheme.caution)
                    Text(stubNote)
                        .font(.system(size: 10, weight: .medium))
                        .foregroundStyle(PadTheme.caution)
                }
                .padding(.horizontal, 9)
                .padding(.vertical, 6)
                .background(PadTheme.caution.opacity(0.14), in: RoundedRectangle(cornerRadius: 8, style: .continuous))
                .overlay(
                    RoundedRectangle(cornerRadius: 8, style: .continuous)
                        .strokeBorder(PadTheme.caution.opacity(0.45), lineWidth: 1)
                )
            }

            if !isUsable {
                expiredProceduresBanner(validity: validity)
            } else if departureAirportCode == nil && arrivalAirportCode == nil {
                Text("Enter a departure or destination airport above to browse Approaches, Arrivals, Departures, and Airport Diagrams.")
                    .font(.caption)
                    .foregroundStyle(PadTheme.label)
            } else {
                tabSelectorRow
                if activeTab == .airportPlates {
                    standalonePlatesBody
                } else {
                    procedureSelectionBody
                }
            }

            if let banner = statusBanner {
                HStack(spacing: 6) {
                    Image(systemName: "checkmark.circle.fill")
                        .font(.caption2)
                        .foregroundStyle(PadTheme.active)
                    Text(banner)
                        .font(.system(size: 10.5, weight: .semibold, design: .monospaced))
                        .foregroundStyle(PadTheme.active)
                }
            }
        }
        .padding(12)
        .cockpitRaisedCard(cornerRadius: 14)
        .onAppear {
            syncDefaultSelection()
        }
        .onChange(of: controller.flightPlanWaypoints.map(\.ident)) { _, _ in
            syncDefaultSelection()
        }
    }

    // MARK: - Header & Multi-Provider Chart Service Menu

    private func headerBar(validity: AeroPackValidity, isUsable: Bool) -> some View {
        let cycleColor = isUsable ? PadTheme.active : PadTheme.warning
        return HStack(spacing: 8) {
            Label("Procedures & Plates", systemImage: "doc.richtext.fill")
                .font(.subheadline.weight(.bold))
                .foregroundStyle(PadTheme.value)

            Spacer()

            chartProviderMenu

            Text(validity.shortTag)
                .font(.system(size: 9.5, weight: .heavy, design: .monospaced))
                .padding(.horizontal, 7)
                .padding(.vertical, 3)
                .background(cycleColor.opacity(0.18), in: Capsule())
                .overlay(Capsule().strokeBorder(cycleColor.opacity(0.50), lineWidth: 1))
                .foregroundStyle(cycleColor)
        }
    }

    private var chartProviderMenu: some View {
        Menu {
            Section("Primary Chart & Procedure Source") {
                ForEach(controller.chartProviders.providers) { provider in
                    let isPreferred = (controller.chartProviders.preferredProviderId == provider.id)
                    let badge: String = switch provider.status {
                    case .activeOffline(let cyc): cyc
                    case .subscriptionStub(let tag): tag
                    }
                    Button {
                        controller.chartProviders.selectPreferredProvider(provider.id)
                    } label: {
                        Label(
                            "\(provider.shortTitle) (\(badge))",
                            systemImage: isPreferred ? "checkmark.circle.fill" : "circle"
                        )
                    }
                }
            }

            Section("Simultaneous Multi-Service Subscriptions") {
                ForEach(controller.chartProviders.providers) { provider in
                    let isEnabled = controller.chartProviders.enabledProviderIds.contains(provider.id)
                    Button {
                        controller.chartProviders.toggleProviderEnabled(provider.id)
                    } label: {
                        Label(
                            "\(provider.fullName)",
                            systemImage: isEnabled ? "checkmark.square.fill" : "square"
                        )
                    }
                }
            }
        } label: {
            HStack(spacing: 4) {
                Image(systemName: "square.stack.3d.up.fill")
                    .font(.system(size: 10, weight: .semibold))
                Text(controller.chartProviders.activeSummaryLabel)
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                Image(systemName: "chevron.up.chevron.down")
                    .font(.system(size: 8.5, weight: .bold))
            }
            .padding(.horizontal, 8)
            .padding(.vertical, 4)
            .background(PadTheme.control, in: Capsule())
            .overlay(Capsule().strokeBorder(PadTheme.commanded.opacity(0.55), lineWidth: 1))
            .foregroundStyle(PadTheme.commanded)
        }
    }

    private func expiredProceduresBanner(validity: AeroPackValidity) -> some View {
        HStack(spacing: 8) {
            Image(systemName: "xmark.octagon.fill")
                .foregroundStyle(PadTheme.warning)
            VStack(alignment: .leading, spacing: 1) {
                Text("PROCEDURES & PLATES PACK INVALID (\(validity.badgeText))")
                    .font(.system(size: 10.5, weight: .heavy, design: .monospaced))
                    .foregroundStyle(PadTheme.warning)
                Text("Published Approaches, Arrivals, Departures, and Procedure Plates are suppressed until a current 28-day cycle is synced.")
                    .font(.system(size: 9.5))
                    .foregroundStyle(PadTheme.label)
            }
            Spacer(minLength: 0)
        }
        .padding(9)
        .background(PadTheme.warning.opacity(0.14), in: RoundedRectangle(cornerRadius: 10, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: 10, style: .continuous)
                .strokeBorder(PadTheme.warning.opacity(0.55), lineWidth: 1)
        )
    }

    // MARK: - Airport & Tab Resolution

    private var departureAirportCode: String? {
        controller.departureAirportWaypoint?.ident
    }

    private var arrivalAirportCode: String? {
        controller.flightPlanWaypoints.last(where: { $0.kind == .airport })?.ident
    }

    private var currentProcedureKind: CifpProcedureKind {
        switch activeTab {
        case .approach, .airportPlates: .approach
        case .arrival: .star
        case .departure: .sid
        }
    }

    private func targetAirport(for tab: PickerTab) -> String? {
        switch tab {
        case .departure:
            departureAirportCode ?? arrivalAirportCode
        case .approach, .arrival, .airportPlates:
            arrivalAirportCode ?? departureAirportCode
        }
    }

    private var activeProcedures: [CifpProcedureSummary] {
        guard let apt = targetAirport(for: activeTab) else { return [] }
        return controller.availableCifpProcedures(forAirport: apt, kind: currentProcedureKind)
    }

    private var selectedProcedure: CifpProcedureSummary? {
        let list = activeProcedures
        if let sel = selectedProcedureId,
           let match = list.first(where: { $0.id == sel }) {
            return match
        }
        return list.first
    }

    private var allRouteAirportPlates: [ProcedurePlateDescriptor] {
        var airports: [String] = []
        if let dep = departureAirportCode { airports.append(dep) }
        if let arr = arrivalAirportCode, !airports.contains(arr) { airports.append(arr) }
        return airports.flatMap { controller.availablePlates(forAirport: $0) }
    }

    private var tabSelectorRow: some View {
        ScrollView(.horizontal, showsIndicators: false) {
            HStack(spacing: 6) {
                ForEach(PickerTab.allCases) { tab in
                    let apt = targetAirport(for: tab) ?? "---"
                    let count: Int = {
                        switch tab {
                        case .approach:
                            return targetAirport(for: tab).map { controller.availableCifpProcedures(forAirport: $0, kind: .approach).count } ?? 0
                        case .arrival:
                            return targetAirport(for: tab).map { controller.availableCifpProcedures(forAirport: $0, kind: .star).count } ?? 0
                        case .departure:
                            return targetAirport(for: tab).map { controller.availableCifpProcedures(forAirport: $0, kind: .sid).count } ?? 0
                        case .airportPlates:
                            return allRouteAirportPlates.count
                        }
                    }()
                    let selected = (activeTab == tab)
                    let labelText = (tab == .airportPlates) ? "Plates" : "\(tab.rawValue) · \(apt)"

                    Button {
                        activeTab = tab
                        selectedProcedureId = nil
                        selectedRunwayTransition = nil
                        selectedEnrouteOrFeederTransition = nil
                        syncTransitionsForCurrentProcedure()
                    } label: {
                        HStack(spacing: 4) {
                            Text(labelText)
                                .font(.system(size: 10.5, weight: .bold, design: .monospaced))
                            Text("\(count)")
                                .font(.system(size: 9.5, weight: .heavy, design: .monospaced))
                                .padding(.horizontal, 5)
                                .padding(.vertical, 1)
                                .background(PadTheme.background.opacity(0.65), in: Capsule())
                        }
                        .padding(.horizontal, 10)
                        .padding(.vertical, 6)
                        .background(
                            selected ? PadTheme.commanded.opacity(0.18) : PadTheme.control,
                            in: Capsule()
                        )
                        .overlay(
                            Capsule()
                                .strokeBorder(selected ? PadTheme.commanded : PadTheme.separator, lineWidth: 1)
                        )
                        .foregroundStyle(selected ? PadTheme.commanded : PadTheme.value)
                    }
                    .buttonStyle(.plain)
                }
            }
        }
    }

    // MARK: - Procedure Selection (`Approach`, `Arrival`, `Departure`)

    @ViewBuilder
    private var procedureSelectionBody: some View {
        let list = activeProcedures
        if list.isEmpty {
            Text("No published \(activeTab.rawValue) procedures for \(targetAirport(for: activeTab) ?? "this airport").")
                .font(.caption)
                .foregroundStyle(PadTheme.label)
        } else {
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 6) {
                    ForEach(list) { proc in
                        let isSelected = (selectedProcedure?.id == proc.id)
                        Button {
                            selectedProcedureId = proc.id
                            syncTransitions(for: proc)
                        } label: {
                            Text(proc.humanTitle)
                                .font(.system(size: 11, weight: .bold, design: .monospaced))
                                .padding(.horizontal, 10)
                                .padding(.vertical, 6)
                                .background(
                                    isSelected ? PadTheme.commanded.opacity(0.18) : PadTheme.control,
                                    in: RoundedRectangle(cornerRadius: 8, style: .continuous)
                                )
                                .overlay(
                                    RoundedRectangle(cornerRadius: 8, style: .continuous)
                                        .strokeBorder(isSelected ? PadTheme.commanded : PadTheme.separator, lineWidth: 1)
                                )
                                .foregroundStyle(isSelected ? PadTheme.commanded : PadTheme.value)
                        }
                        .buttonStyle(.plain)
                    }
                }
            }

            if let proc = selectedProcedure {
                transitionAndPlateControls(for: proc)
            }
        }
    }

    @ViewBuilder
    private func transitionAndPlateControls(for proc: CifpProcedureSummary) -> some View {
        let feeders = proc.enrouteOrFeederTransitions
        let rwys = proc.runwayTransitions
        let plateDesc = controller.resolvePlateDescriptor(for: proc)

        VStack(alignment: .leading, spacing: 8) {
            if !feeders.isEmpty {
                ScrollView(.horizontal, showsIndicators: false) {
                    HStack(spacing: 6) {
                        Text(proc.kind == .approach ? "IAF / Feeder:" : "Transition:")
                            .font(.system(size: 10, weight: .semibold, design: .monospaced))
                            .foregroundStyle(PadTheme.label)

                        transitionChip(
                            title: proc.kind == .approach ? "Vectors / Final" : "Core Only",
                            isSelected: selectedEnrouteOrFeederTransition == nil
                        ) {
                            selectedEnrouteOrFeederTransition = nil
                        }

                        ForEach(feeders) { tr in
                            transitionChip(
                                title: tr.ident,
                                isSelected: selectedEnrouteOrFeederTransition == tr.ident
                            ) {
                                selectedEnrouteOrFeederTransition = tr.ident
                            }
                        }
                    }
                }
            }

            if !rwys.isEmpty {
                ScrollView(.horizontal, showsIndicators: false) {
                    HStack(spacing: 6) {
                        Text("Runway:")
                            .font(.system(size: 10, weight: .semibold, design: .monospaced))
                            .foregroundStyle(PadTheme.label)

                        ForEach(rwys) { tr in
                            transitionChip(
                                title: tr.ident,
                                isSelected: selectedRunwayTransition == tr.ident
                            ) {
                                selectedRunwayTransition = tr.ident
                            }
                        }
                    }
                }
            }

            HStack(spacing: 8) {
                if proc.kind == .approach {
                    Toggle("Missed", isOn: $includeMissedApproach)
                        .font(.system(size: 10.5, weight: .semibold, design: .monospaced))
                        .foregroundStyle(PadTheme.label)
                        .tint(PadTheme.commanded)
                        .toggleStyle(.switch)
                        .controlSize(.mini)
                }

                Spacer(minLength: 4)

                Button {
                    onInspectPlate(
                        PresentedPlateRequest(
                            descriptor: plateDesc,
                            procedure: proc,
                            runwayTransitionIdent: selectedRunwayTransition,
                            enrouteOrFeederTransitionIdent: selectedEnrouteOrFeederTransition
                        )
                    )
                } label: {
                    Label("View Plate", systemImage: "doc.text.magnifyingglass")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                }
                .buttonStyle(.bordered)
                .tint(PadTheme.commanded)
                .controlSize(.small)

                if plateDesc.supportsMapOverlay {
                    Button {
                        if let pinned = controller.pinPlateToMap(
                            plateDesc,
                            procedure: proc,
                            runwayTransitionIdent: selectedRunwayTransition,
                            enrouteOrFeederTransitionIdent: selectedEnrouteOrFeederTransition
                        ) {
                            statusBanner = "Pinned \(proc.airportIdent) \(proc.humanTitle) plate onto moving map"
                            onPinnedPlateToMap(pinned)
                        }
                    } label: {
                        Label("Plate on Map", systemImage: "map.fill")
                            .font(.system(size: 11, weight: .bold, design: .monospaced))
                            .foregroundStyle(PadTheme.background)
                    }
                    .buttonStyle(.borderedProminent)
                    .tint(PadTheme.commanded)
                    .controlSize(.small)
                }

                Button {
                    let count = controller.applyCifpProcedure(
                        proc,
                        runwayTransitionIdent: selectedRunwayTransition,
                        enrouteOrFeederTransitionIdent: selectedEnrouteOrFeederTransition,
                        includeMissedApproach: includeMissedApproach
                    )
                    let via = selectedEnrouteOrFeederTransition ?? selectedRunwayTransition ?? "Core"
                    statusBanner = "Added \(proc.airportIdent) \(proc.humanTitle) (\(via), \(count) legs)"
                } label: {
                    Label("Add to Route", systemImage: "plus.circle.fill")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .foregroundStyle(PadTheme.background)
                }
                .buttonStyle(.borderedProminent)
                .tint(PadTheme.plan)
                .controlSize(.small)
            }
        }
    }

    // MARK: - Standalone Airport Diagrams & Terminal Plates (`Plates` Tab)

    @ViewBuilder
    private var standalonePlatesBody: some View {
        let plates = allRouteAirportPlates
        if !controller.isDtppUsable {
            Text("US Terminal Procedure Plates pack is offline/expired — sync in Downloads.")
                .font(.caption)
                .foregroundStyle(PadTheme.caution)
        } else if plates.isEmpty {
            Text("No bundled FAA PDF plates for \(arrivalAirportCode ?? departureAirportCode ?? "this airport") — select an Approach tab to generate a georeferenced procedural planview.")
                .font(.caption)
                .foregroundStyle(PadTheme.label)
        } else {
            VStack(alignment: .leading, spacing: 5) {
                ForEach(plates.prefix(12)) { plate in
                    HStack(spacing: 8) {
                        Text(plate.airportIdent)
                            .font(.system(size: 11, weight: .heavy, design: .monospaced))
                            .foregroundStyle(PadTheme.value)
                            .frame(width: 44, alignment: .leading)

                        Text(plate.kind.badge)
                            .font(.system(size: 9.5, weight: .bold, design: .monospaced))
                            .padding(.horizontal, 6)
                            .padding(.vertical, 2)
                            .background(PadTheme.background, in: Capsule())
                            .foregroundStyle(PadTheme.label)

                        Text(plate.title)
                            .font(.system(size: 11.5, weight: .semibold, design: .monospaced))
                            .foregroundStyle(PadTheme.value)
                            .lineLimit(1)

                        Spacer(minLength: 4)

                        if plate.supportsMapOverlay {
                            Button("Plate on Map") {
                                if let pinned = controller.pinPlateToMap(plate) {
                                    onPinnedPlateToMap(pinned)
                                }
                            }
                            .buttonStyle(.bordered)
                            .tint(PadTheme.commanded)
                            .controlSize(.mini)
                            .font(.system(size: 9.5, weight: .bold, design: .monospaced))
                        }

                        Button {
                            onInspectPlate(
                                PresentedPlateRequest(
                                    descriptor: plate,
                                    procedure: nil,
                                    runwayTransitionIdent: nil,
                                    enrouteOrFeederTransitionIdent: nil
                                )
                            )
                        } label: {
                            Text("View")
                                .foregroundStyle(PadTheme.background)
                        }
                        .buttonStyle(.borderedProminent)
                        .tint(PadTheme.commanded)
                        .controlSize(.mini)
                        .font(.system(size: 9.5, weight: .bold, design: .monospaced))
                    }
                    .padding(.horizontal, 9)
                    .padding(.vertical, 6)
                    .background(PadTheme.control, in: RoundedRectangle(cornerRadius: 8, style: .continuous))
                    .overlay(
                        RoundedRectangle(cornerRadius: 8, style: .continuous)
                            .strokeBorder(PadTheme.separator, lineWidth: 1)
                    )
                }
            }
        }
    }

    private func transitionChip(title: String, isSelected: Bool, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            Text(title)
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .padding(.horizontal, 8)
                .padding(.vertical, 4)
                .background(
                    isSelected ? PadTheme.commanded.opacity(0.18) : PadTheme.control,
                    in: Capsule()
                )
                .overlay(
                    Capsule()
                        .strokeBorder(isSelected ? PadTheme.commanded : PadTheme.separator, lineWidth: 1)
                )
                .foregroundStyle(isSelected ? PadTheme.commanded : PadTheme.label)
        }
        .buttonStyle(.plain)
    }

    private func syncDefaultSelection() {
        if let arr = arrivalAirportCode,
           !controller.availableCifpProcedures(forAirport: arr, kind: .approach).isEmpty {
            if activeTab != .airportPlates {
                activeTab = .approach
            }
        } else if let dep = departureAirportCode,
                  !controller.availableCifpProcedures(forAirport: dep, kind: .sid).isEmpty {
            if activeTab != .airportPlates {
                activeTab = .departure
            }
        }
        syncTransitionsForCurrentProcedure()
    }

    private func syncTransitionsForCurrentProcedure() {
        if let proc = selectedProcedure {
            selectedProcedureId = proc.id
            syncTransitions(for: proc)
        }
    }

    private func syncTransitions(for proc: CifpProcedureSummary) {
        if let curEnr = selectedEnrouteOrFeederTransition,
           proc.enrouteOrFeederTransitions.contains(where: { $0.ident == curEnr }) {
            // Keep valid feeder/enroute transition.
        } else {
            selectedEnrouteOrFeederTransition = proc.enrouteOrFeederTransitions.first?.ident
        }

        if let curRwy = selectedRunwayTransition,
           proc.runwayTransitions.contains(where: { $0.ident == curRwy }) {
            // Keep valid runway transition.
        } else {
            selectedRunwayTransition = proc.runwayTransitions.first?.ident
        }
    }
}

import SwiftUI
import UIKit
import UniformTypeIdentifiers
import FlightDeckKit

/// ForeFlight & `Pilotage`-aligned Flight Plan (`FPL`) route pill editor:
/// - Quiet-Dark-Cockpit state-driven identifier pills (`#E850E8` active `▸ TO`, `#68707C` sequenced past,
///   `#262B33` / `#ECF0F3` upcoming, `#00FFFF` `2 pt` selected outline — zero rainbow hue clutter)
/// - Non-color geometric type differentiation (`✈`, `VORTAC`, `VOR`, `NDB`, `FIX`, `GPS`, `SITE`)
/// - Deterministic `PadRouteInputGeometry` + permanent sibling `PadAsciiBackspaceTextField` (`Pilotage` pattern,
///   so `Clear` or Backspace to empty never destroys a `UITextField` cell or mutates SwiftUI height mid-pass)
/// - Long-press drag-and-drop reordering (`UICollectionViewDragDelegate` / `DropDelegate`)
/// - Strict ASCII-only input (`A-Z`, `0-9`, space, `./+-` for `LAT/LON`)
struct PadRouteFlowEditor: View {
    @Bindable var controller: FlightDeckController
    @Binding var primaryMode: PadPrimaryViewMode
    @Binding var orbit: OrbitCameraState
    var initialDraftQuery: String = ""
    var onInspectPlate: ((PresentedPlateRequest) -> Void)? = nil
    var onPinnedPlateToMap: ((PinnedMapPlate) -> Void)? = nil

    @State private var draftToken: String = ""
    @State private var didSeedInitialQuery: Bool = false
    @State private var selectedBubbleIndex: Int?
    @State private var statusMessage: String?
    @State private var statusIsWarning: Bool = false

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            headerRow

            if controller.primaryBody == .earth, !controller.isEarthNavdataUsable {
                expiredNavdataBanner
            }

            PadRouteInputRepresentable(
                waypoints: controller.flightPlanWaypoints,
                activeWaypointIndex: controller.activeWaypointIndex,
                selectedBubbleIndex: $selectedBubbleIndex,
                draftToken: $draftToken,
                onReorder: { reordered in
                    controller.replaceFlightPlanWaypoints(reordered)
                    selectedBubbleIndex = nil
                    statusMessage = nil
                },
                onDeleteAt: { index in
                    guard index >= 0, index < controller.flightPlanWaypoints.count else { return }
                    let removed = controller.flightPlanWaypoints[index].ident
                    controller.removeFlightPlanWaypoint(at: index)
                    selectedBubbleIndex = nil
                    statusIsWarning = false
                    statusMessage = "Removed \(removed)"
                },
                onCommitTokens: { tokens in
                    commitAsciiTokens(tokens)
                },
                onRejectNonAscii: {
                    statusIsWarning = true
                    statusMessage = "ASCII only: enter ICAO/FAA identifiers or LAT/LON (A-Z, 0-9, ./+-)."
                }
            )
            .background(
                PadTheme.background,
                in: RoundedRectangle(cornerRadius: 12, style: .continuous)
            )
            .overlay(
                RoundedRectangle(cornerRadius: 12, style: .continuous)
                    .strokeBorder(PadTheme.separator, lineWidth: 1)
            )

            if let msg = statusMessage {
                HStack(spacing: 6) {
                    Image(systemName: statusIsWarning ? "exclamationmark.triangle.fill" : "checkmark.circle.fill")
                        .font(.caption2)
                        .foregroundStyle(statusIsWarning ? PadTheme.caution : PadTheme.active)
                    Text(msg)
                        .font(.system(size: 10.5, weight: .semibold, design: .monospaced))
                        .foregroundStyle(statusIsWarning ? PadTheme.caution : PadTheme.label)
                    Spacer(minLength: 0)
                }
            }

            if let selIdx = selectedBubbleIndex,
               selIdx >= 0,
               selIdx < controller.flightPlanWaypoints.count {
                selectedBubbleInspector(index: selIdx, waypoint: controller.flightPlanWaypoints[selIdx])
            }

            let matches = activeSearchMatches
            if !matches.isEmpty {
                autocompleteMatchesList(matches)
            }
        }
        .padding(12)
        .cockpitRaisedCard(cornerRadius: 14)
        .onAppear {
            if !didSeedInitialQuery, !initialDraftQuery.isEmpty {
                didSeedInitialQuery = true
                if let normalized = RouteAsciiInput.normalized(initialDraftQuery) {
                    draftToken = normalized.trimmingCharacters(in: .whitespaces)
                }
            }
        }
    }

    private var expiredNavdataBanner: some View {
        let validity = controller.earthNavdataValidity
        return HStack(spacing: 8) {
            Image(systemName: "xmark.octagon.fill")
                .foregroundStyle(PadTheme.warning)
            VStack(alignment: .leading, spacing: 1) {
                Text("NAVIGATION DATABASE INVALID (\(validity.badgeText))")
                    .font(.system(size: 10.5, weight: .heavy, design: .monospaced))
                    .foregroundStyle(PadTheme.warning)
                Text(validity.detailNote)
                    .font(.system(size: 9.5))
                    .foregroundStyle(PadTheme.label)
            }
            Spacer(minLength: 0)
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 8)
        .background(PadTheme.warning.opacity(0.14), in: RoundedRectangle(cornerRadius: 10, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: 10, style: .continuous)
                .strokeBorder(PadTheme.warning.opacity(0.55), lineWidth: 1)
        )
    }

    private var headerRow: some View {
        let isEmpty = controller.flightPlanWaypoints.isEmpty
        let canReverse = controller.flightPlanWaypoints.count >= 2

        return HStack(spacing: 8) {
            Label("Flight Plan Route", systemImage: "point.bottomleft.forward.to.arrow.triangle.scurvepath.fill")
                .font(.subheadline.weight(.bold))
                .foregroundStyle(PadTheme.value)

            Spacer()

            if controller.totalFlightPlanDistanceNm > 0.1 {
                Text(String(format: "%.1f NM", controller.totalFlightPlanDistanceNm))
                    .font(.caption.monospaced().bold())
                    .padding(.horizontal, 8)
                    .padding(.vertical, 3)
                    .background(PadTheme.control, in: Capsule())
                    .overlay(Capsule().strokeBorder(PadTheme.separator, lineWidth: 1))
                    .foregroundStyle(PadTheme.value)
            }

            // Permanently mounted action buttons (`.disabled` instead of conditional removal mid-tap).
            Button("Reverse") {
                selectedBubbleIndex = nil
                controller.replaceFlightPlanWaypoints(controller.flightPlanWaypoints.reversed())
            }
            .disabled(!canReverse)
            .font(.caption.weight(.semibold))
            .buttonStyle(.bordered)
            .tint(PadTheme.label)
            .controlSize(.mini)

            Button("Clear") {
                selectedBubbleIndex = nil
                draftToken = ""
                statusIsWarning = false
                statusMessage = "Route cleared"
                controller.clearFlightPlan()
            }
            .disabled(isEmpty)
            .font(.caption.weight(.semibold))
            .buttonStyle(.bordered)
            .tint(PadTheme.label)
            .controlSize(.mini)
        }
    }

    private func selectedBubbleInspector(index: Int, waypoint: FlightPlanWaypoint) -> some View {
        let isActive = (index == controller.activeWaypointIndex)
        return HStack(spacing: 8) {
            Image(systemName: waypoint.kind.symbolName)
                .font(.system(size: 14, weight: .semibold))
                .foregroundStyle(isActive ? PadTheme.plan : PadTheme.value)
                .frame(width: 22)

            if !waypoint.kind.badge.isEmpty {
                Text(waypoint.kind.badge)
                    .font(.system(size: 9.5, weight: .heavy, design: .monospaced))
                    .padding(.horizontal, 6)
                    .padding(.vertical, 3)
                    .background(PadTheme.control, in: Capsule())
                    .foregroundStyle(PadTheme.label)
            }

            if !waypoint.airwayOrProcedureTag.isEmpty {
                Text(waypoint.airwayOrProcedureTag)
                    .font(.system(size: 9.5, weight: .heavy, design: .monospaced))
                    .padding(.horizontal, 6)
                    .padding(.vertical, 3)
                    .background(PadTheme.control, in: Capsule())
                    .foregroundStyle(PadTheme.label)
            }

            VStack(alignment: .leading, spacing: 1) {
                Text(waypoint.name == waypoint.ident ? waypoint.ident : "\(waypoint.ident) · \(waypoint.name)")
                    .font(.caption.monospaced().bold())
                    .foregroundStyle(PadTheme.value)
                    .lineLimit(1)
                Text(
                    String(
                        format: "%.4f°, %.4f°%@%@",
                        waypoint.latitudeDeg,
                        waypoint.longitudeDeg,
                        waypoint.region.isEmpty ? "" : " · \(waypoint.region)",
                        waypoint.altitudeConstraintTag.isEmpty ? "" : " · \(waypoint.altitudeConstraintTag)"
                    )
                )
                .font(.system(size: 9.5, design: .monospaced))
                .foregroundStyle(PadTheme.label)
            }

            Spacer(minLength: 4)

            if waypoint.kind == .airport,
               controller.primaryBody == .earth {
                let aptPlates = controller.availablePlates(forAirport: waypoint.ident)
                if !aptPlates.isEmpty {
                    Menu {
                        Section("\(waypoint.ident) Plates on Map") {
                            ForEach(aptPlates.filter(\.supportsMapOverlay)) { plate in
                                Button {
                                    if let pinned = controller.pinPlateToMap(plate) {
                                        onPinnedPlateToMap?(pinned)
                                    }
                                } label: {
                                    Label(plate.title, systemImage: "map.fill")
                                }
                            }
                        }
                        Section("\(waypoint.ident) Fullscreen Plates") {
                            ForEach(aptPlates) { plate in
                                Button {
                                    onInspectPlate?(
                                        PresentedPlateRequest(
                                            descriptor: plate,
                                            procedure: nil,
                                            runwayTransitionIdent: nil,
                                            enrouteOrFeederTransitionIdent: nil
                                        )
                                    )
                                } label: {
                                    Label(plate.title, systemImage: "doc.richtext")
                                }
                            }
                        }
                    } label: {
                        Label("Plates", systemImage: "doc.richtext.fill")
                            .font(.system(size: 10, weight: .bold))
                    }
                    .buttonStyle(.bordered)
                    .tint(PadTheme.commanded)
                    .controlSize(.mini)
                }
            }

            Button("Direct-To") {
                controller.activateDirectTo(index: index)
                statusIsWarning = false
                statusMessage = "Direct-To \(waypoint.ident)"
            }
            .buttonStyle(.borderedProminent)
            .tint(PadTheme.plan)
            .controlSize(.mini)
            .font(.system(size: 10, weight: .bold, design: .monospaced))

            Button("Show on Map") {
                orbit.latitude = waypoint.latitudeDeg
                orbit.longitude = waypoint.longitudeDeg
                orbit.zoom = max(orbit.zoom, 9.6)
                primaryMode = .map
            }
            .buttonStyle(.bordered)
            .tint(PadTheme.commanded)
            .controlSize(.mini)
            .font(.system(size: 10, weight: .semibold))

            Button {
                selectedBubbleIndex = nil
                controller.removeFlightPlanWaypoint(at: index)
            } label: {
                Image(systemName: "trash")
            }
            .buttonStyle(.bordered)
            .tint(PadTheme.label)
            .controlSize(.mini)
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 8)
        .background(PadTheme.background, in: RoundedRectangle(cornerRadius: 10, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: 10, style: .continuous)
                .strokeBorder(PadTheme.commanded.opacity(0.45), lineWidth: 1)
        )
    }

    private var activeSearchMatches: [NavigationPoint] {
        let trimmed = draftToken.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return [] }
        let lastWord = RouteAsciiInput.words(from: trimmed).last ?? trimmed
        return controller.searchNavigationPoints(lastWord, limit: 7)
    }

    private func autocompleteMatchesList(_ matches: [NavigationPoint]) -> some View {
        let refLat = controller.flightPlanWaypoints.last?.latitudeDeg ?? controller.latestFrame?.ownship.latitudeDeg
        let refLon = controller.flightPlanWaypoints.last?.longitudeDeg ?? controller.latestFrame?.ownship.longitudeDeg
        let bodyR = controller.bodyParameters.renderRadiusMeters

        return VStack(alignment: .leading, spacing: 4) {
            ForEach(matches) { pt in
                Button {
                    appendCandidatePoint(pt)
                } label: {
                    HStack(spacing: 8) {
                        waypointLeadingIndicator(for: pt.kind)

                        Text(pt.identifier)
                            .font(.system(size: 13, weight: .bold, design: .monospaced))
                            .foregroundStyle(PadTheme.value)
                            .frame(minWidth: 54, alignment: .leading)

                        Text(pt.name)
                            .font(.caption)
                            .foregroundStyle(PadTheme.label)
                            .lineLimit(1)

                        Spacer(minLength: 4)

                        if !pt.region.isEmpty {
                            Text(pt.region)
                                .font(.system(size: 9.5, weight: .semibold, design: .monospaced))
                                .foregroundStyle(PadTheme.label)
                        }

                        if let lat0 = refLat, let lon0 = refLon {
                            let distNm = RouteLegMetrics.between(
                                lat1Deg: lat0,
                                lon1Deg: lon0,
                                lat2Deg: pt.latitudeDeg,
                                lon2Deg: pt.longitudeDeg,
                                radiusMeters: bodyR
                            ).distanceNauticalMiles
                            Text(String(format: "%.0f NM", distNm))
                                .font(.system(size: 10.5, weight: .bold, design: .monospaced))
                                .foregroundStyle(PadTheme.value)
                                .frame(width: 56, alignment: .trailing)
                        }
                    }
                    .padding(.horizontal, 9)
                    .padding(.vertical, 6)
                    .background(PadTheme.control, in: RoundedRectangle(cornerRadius: 8, style: .continuous))
                    .overlay(
                        RoundedRectangle(cornerRadius: 8, style: .continuous)
                            .strokeBorder(PadTheme.separator, lineWidth: 1)
                    )
                }
                .buttonStyle(.plain)
            }
        }
    }

    @ViewBuilder
    private func waypointLeadingIndicator(for kind: WaypointKind) -> some View {
        if kind == .airport {
            // ForeFlight-style airport icon without redundant "APT" text.
            HStack(spacing: 4) {
                Image(systemName: "airplane.circle.fill")
                    .font(.system(size: 13, weight: .semibold))
                    .foregroundStyle(PadTheme.value)
            }
            .frame(width: 52, alignment: .center)
        } else {
            Text(kind.badge)
                .font(.system(size: 9.5, weight: .heavy, design: .monospaced))
                .frame(width: 52)
                .padding(.vertical, 3)
                .background(PadTheme.background, in: RoundedRectangle(cornerRadius: 6, style: .continuous))
                .foregroundStyle(PadTheme.label)
        }
    }

    private func appendCandidatePoint(_ pt: NavigationPoint) {
        let kindSuffix = pt.kind.badge.isEmpty ? "" : " (\(pt.kind.badge))"
        if let replaceIdx = selectedBubbleIndex,
           replaceIdx >= 0,
           replaceIdx < controller.flightPlanWaypoints.count {
            var updated = controller.flightPlanWaypoints
            updated[replaceIdx] = FlightPlanWaypoint(id: updated[replaceIdx].id, point: pt)
            controller.replaceFlightPlanWaypoints(updated)
            selectedBubbleIndex = nil
            draftToken = ""
            statusIsWarning = false
            statusMessage = "Updated to \(pt.identifier)\(kindSuffix)"
        } else {
            controller.appendFlightPlanPoint(pt)
            draftToken = ""
            statusIsWarning = false
            statusMessage = "Added \(pt.identifier)\(kindSuffix)"
        }
    }

    private func commitAsciiTokens(_ tokens: [String]) {
        if controller.primaryBody == .earth, !controller.isEarthNavdataUsable {
            let coordOnly = tokens.compactMap { RouteAsciiInput.parseCoordinate($0) }
            if coordOnly.count != tokens.count {
                statusIsWarning = true
                statusMessage = "NASR database expired & invalid — download current cycle before resolving identifiers."
                return
            }
        }

        let result = controller.appendRouteTokensWithCifp(tokens, onto: controller.flightPlanWaypoints)
        controller.replaceFlightPlanWaypoints(result.updated)
        if result.unresolved.isEmpty {
            draftToken = ""
            statusIsWarning = false
            if let expanded = result.expandedNote {
                statusMessage = expanded
            } else if let last = result.updated.last {
                let suffix = last.kind.badge.isEmpty ? "" : " (\(last.kind.badge))"
                statusMessage = "Added \(last.ident)\(suffix)"
            }
        } else {
            draftToken = result.unresolved.joined(separator: " ")
            statusIsWarning = true
            if controller.primaryBody == .earth, !controller.isCifpUsable,
               result.unresolved.contains(where: { CifpProcedureDatabase.shared.lookupAirway($0) != nil }) {
                statusMessage = "Procedures & airways database expired & invalid — sync current 28d cycle."
            } else {
                statusMessage = "Unknown waypoint/airway: \(result.unresolved.joined(separator: ", "))"
            }
        }
    }
}

// MARK: - Deterministic `Pilotage` `RouteInputGeometry` + Sibling `UITextField` Architecture

private struct PadRouteInputGeometry {
    static let insetX: CGFloat = 8
    static let insetY: CGFloat = 7
    static let spacing: CGFloat = 6
    static let rowHeight: CGFloat = 30

    let chipFrames: [CGRect]
    let fieldFrame: CGRect
    let totalHeight: CGFloat

    init(
        containerWidth: CGFloat,
        chipWidths: [CGFloat],
        textWidth: CGFloat
    ) {
        let innerWidth = max((containerWidth.isFinite ? containerWidth : 420) - Self.insetX * 2, 140)
        var cursorX: CGFloat = 0
        var cursorY: CGFloat = 0
        var frames: [CGRect] = []
        frames.reserveCapacity(chipWidths.count)

        for rawW in chipWidths {
            let w = min(rawW, innerWidth)
            if cursorX > 0 && cursorX + w > innerWidth {
                cursorX = 0
                cursorY += Self.rowHeight + Self.spacing
            }
            frames.append(
                CGRect(
                    x: Self.insetX + cursorX,
                    y: Self.insetY + cursorY,
                    width: w,
                    height: Self.rowHeight
                )
            )
            cursorX += w + Self.spacing
        }

        let minFieldW = min(innerWidth, max(chipWidths.isEmpty ? innerWidth : 150, textWidth + 28))
        if cursorX > 0 && cursorX + minFieldW > innerWidth {
            cursorX = 0
            cursorY += Self.rowHeight + Self.spacing
        }
        let fieldW = max(innerWidth - cursorX, 40)
        self.chipFrames = frames
        self.fieldFrame = CGRect(
            x: Self.insetX + cursorX,
            y: Self.insetY + cursorY,
            width: fieldW,
            height: Self.rowHeight
        )
        self.totalHeight = Self.insetY * 2 + cursorY + Self.rowHeight
    }
}

private final class PadRouteChipLayout: UICollectionViewLayout {
    var frames: [CGRect] = []
    var contentSize: CGSize = .zero

    override var collectionViewContentSize: CGSize { contentSize }

    override func layoutAttributesForElements(in rect: CGRect) -> [UICollectionViewLayoutAttributes]? {
        frames.indices.compactMap { idx in
            guard frames[idx].intersects(rect) else { return nil }
            return layoutAttributesForItem(at: IndexPath(item: idx, section: 0))
        }
    }

    override func layoutAttributesForItem(at indexPath: IndexPath) -> UICollectionViewLayoutAttributes? {
        guard frames.indices.contains(indexPath.item) else { return nil }
        let attr = UICollectionViewLayoutAttributes(forCellWith: indexPath)
        attr.frame = frames[indexPath.item]
        return attr
    }
}

private final class PadRouteInputContainerView: UIView {
    let chipLayout = PadRouteChipLayout()
    lazy var collectionView: UICollectionView = {
        let cv = UICollectionView(frame: .zero, collectionViewLayout: chipLayout)
        cv.backgroundColor = .clear
        cv.isScrollEnabled = false
        cv.showsHorizontalScrollIndicator = false
        cv.showsVerticalScrollIndicator = false
        cv.keyboardDismissMode = .none
        cv.dragInteractionEnabled = true
        cv.reorderingCadence = .immediate
        return cv
    }()

    let textField = PadAsciiBackspaceTextField()
    var measureGeometry: ((CGFloat) -> PadRouteInputGeometry)?

    override init(frame: CGRect) {
        super.init(frame: frame)
        backgroundColor = .clear
        clipsToBounds = true

        textField.font = UIFont.monospacedSystemFont(ofSize: 13.5, weight: .semibold)
        textField.textColor = PadTheme.uiValue
        textField.tintColor = PadTheme.uiCommanded
        textField.keyboardType = .asciiCapable
        textField.autocapitalizationType = .allCharacters
        textField.autocorrectionType = .no
        textField.spellCheckingType = .no
        textField.smartQuotesType = .no
        textField.smartDashesType = .no
        textField.smartInsertDeleteType = .no
        textField.returnKeyType = .done

        addSubview(collectionView)
        addSubview(textField)
    }

    @available(*, unavailable)
    required init?(coder: NSCoder) {
        return nil
    }

    override func layoutSubviews() {
        super.layoutSubviews()
        guard let geom = measureGeometry?(bounds.width) else { return }
        collectionView.frame = bounds
        if chipLayout.frames != geom.chipFrames || chipLayout.contentSize != bounds.size {
            chipLayout.frames = geom.chipFrames
            chipLayout.contentSize = bounds.size
            chipLayout.invalidateLayout()
        }
        textField.frame = geom.fieldFrame
    }
}

private struct PadRouteInputRepresentable: UIViewRepresentable {
    let waypoints: [FlightPlanWaypoint]
    let activeWaypointIndex: Int
    @Binding var selectedBubbleIndex: Int?
    @Binding var draftToken: String
    let onReorder: ([FlightPlanWaypoint]) -> Void
    let onDeleteAt: (Int) -> Void
    let onCommitTokens: ([String]) -> Void
    let onRejectNonAscii: () -> Void

    func makeCoordinator() -> Coordinator {
        Coordinator(parent: self)
    }

    func makeUIView(context: Context) -> PadRouteInputContainerView {
        let container = PadRouteInputContainerView()
        let coordinator = context.coordinator

        container.collectionView.dataSource = coordinator
        container.collectionView.delegate = coordinator
        container.collectionView.dragDelegate = coordinator
        container.collectionView.dropDelegate = coordinator
        container.collectionView.register(
            PadRouteChipCell.self,
            forCellWithReuseIdentifier: PadRouteChipCell.reuseId
        )

        container.textField.delegate = coordinator
        container.textField.onDeleteWhenEmpty = { [weak coordinator] in
            coordinator?.handleEmptyBackspace()
        }
        container.textField.onRejectNonAscii = { [weak coordinator] in
            coordinator?.parent.onRejectNonAscii()
        }
        container.textField.onTextEdited = { [weak coordinator] updated in
            coordinator?.parent.draftToken = updated
        }

        container.measureGeometry = { [weak coordinator] width in
            coordinator?.computeGeometry(forWidth: width)
                ?? PadRouteInputGeometry(containerWidth: width, chipWidths: [], textWidth: 0)
        }

        let tapBg = UITapGestureRecognizer(
            target: coordinator,
            action: #selector(Coordinator.handleContainerTap(_:))
        )
        tapBg.cancelsTouchesInView = false
        container.addGestureRecognizer(tapBg)

        coordinator.containerView = container
        coordinator.applyPlaceholder(isEmpty: waypoints.isEmpty)
        return container
    }

    func updateUIView(_ container: PadRouteInputContainerView, context: Context) {
        let coordinator = context.coordinator
        let chipsChanged = coordinator.cachedWaypoints != waypoints
            || coordinator.cachedActiveIndex != activeWaypointIndex
            || coordinator.cachedSelectedIndex != selectedBubbleIndex
        coordinator.parent = self
        coordinator.applyPlaceholder(isEmpty: waypoints.isEmpty)

        if container.textField.text != draftToken {
            container.textField.text = draftToken
        }

        if chipsChanged && !container.collectionView.hasActiveDrag {
            coordinator.cachedWaypoints = waypoints
            coordinator.cachedActiveIndex = activeWaypointIndex
            coordinator.cachedSelectedIndex = selectedBubbleIndex
            container.setNeedsLayout()
            container.layoutIfNeeded()
            container.collectionView.reloadData()
        } else {
            container.setNeedsLayout()
        }
    }

    func sizeThatFits(
        _ proposal: ProposedViewSize,
        uiView: PadRouteInputContainerView,
        context: Context
    ) -> CGSize? {
        let proposedW = proposal.width ?? max(uiView.bounds.width, 460)
        let width = proposedW.isFinite && proposedW > 80 ? proposedW : 460
        let geom = context.coordinator.computeGeometry(
            forWidth: width,
            waypointsOverride: waypoints,
            activeIndexOverride: activeWaypointIndex,
            draftOverride: draftToken
        )
        let clampedH = min(max(geom.totalHeight, 44), 148)
        return CGSize(width: width, height: clampedH)
    }

    @MainActor
    final class Coordinator: NSObject,
        UICollectionViewDataSource,
        UICollectionViewDelegate,
        UICollectionViewDragDelegate,
        UICollectionViewDropDelegate,
        UITextFieldDelegate
    {
        var parent: PadRouteInputRepresentable
        weak var containerView: PadRouteInputContainerView?
        var cachedWaypoints: [FlightPlanWaypoint]
        var cachedActiveIndex: Int
        var cachedSelectedIndex: Int?

        init(parent: PadRouteInputRepresentable) {
            self.parent = parent
            self.cachedWaypoints = parent.waypoints
            self.cachedActiveIndex = parent.activeWaypointIndex
            self.cachedSelectedIndex = parent.selectedBubbleIndex
        }

        func computeGeometry(
            forWidth width: CGFloat,
            waypointsOverride: [FlightPlanWaypoint]? = nil,
            activeIndexOverride: Int? = nil,
            draftOverride: String? = nil
        ) -> PadRouteInputGeometry {
            let items = waypointsOverride ?? cachedWaypoints
            let activeIdx = activeIndexOverride ?? cachedActiveIndex
            let draft = draftOverride ?? parent.draftToken
            let chipWidths = items.enumerated().map { idx, wp in
                PadRouteChipCell.preferredWidth(for: wp, isActiveLeg: idx == activeIdx)
            }
            let textFont = UIFont.monospacedSystemFont(ofSize: 13.5, weight: .semibold)
            let textW = (draft as NSString).size(withAttributes: [.font: textFont]).width
            return PadRouteInputGeometry(
                containerWidth: width,
                chipWidths: chipWidths,
                textWidth: textW
            )
        }

        func applyPlaceholder(isEmpty: Bool) {
            guard let tf = containerView?.textField else { return }
            let placeholder = isEmpty
                ? "Enter route (e.g. KEWR SBJ N51)..."
                : "Add waypoint..."
            tf.attributedPlaceholder = NSAttributedString(
                string: placeholder,
                attributes: [
                    .foregroundColor: PadTheme.uiDisabled,
                    .font: UIFont.systemFont(ofSize: 12.5, weight: .regular),
                ]
            )
        }

        @objc func handleContainerTap(_ gr: UITapGestureRecognizer) {
            guard let container = containerView else { return }
            let ptInCv = gr.location(in: container.collectionView)
            if container.collectionView.indexPathForItem(at: ptInCv) == nil {
                parent.selectedBubbleIndex = nil
                container.textField.becomeFirstResponder()
            }
        }

        // MARK: - `UICollectionViewDataSource` & `Delegate` (`0..<cachedWaypoints.count` chips ONLY)

        func collectionView(_ collectionView: UICollectionView, numberOfItemsInSection section: Int) -> Int {
            cachedWaypoints.count
        }

        func collectionView(
            _ collectionView: UICollectionView,
            cellForItemAt indexPath: IndexPath
        ) -> UICollectionViewCell {
            guard let cell = collectionView.dequeueReusableCell(
                withReuseIdentifier: PadRouteChipCell.reuseId,
                for: indexPath
            ) as? PadRouteChipCell,
                  indexPath.item < cachedWaypoints.count else {
                return UICollectionViewCell()
            }
            let wp = cachedWaypoints[indexPath.item]
            let idx = indexPath.item
            let clampedActive = cachedWaypoints.count >= 2
                ? min(max(cachedActiveIndex, 1), cachedWaypoints.count - 1)
                : max(cachedActiveIndex, 0)
            let isPast = cachedWaypoints.count >= 2 && idx + 1 < clampedActive && idx != cachedActiveIndex
            cell.configure(
                waypoint: wp,
                isActiveLeg: idx == cachedActiveIndex,
                isPastLeg: isPast,
                isSelected: idx == cachedSelectedIndex,
                onRemove: { [weak self] in
                    self?.parent.onDeleteAt(idx)
                }
            )
            return cell
        }

        func collectionView(_ collectionView: UICollectionView, didSelectItemAt indexPath: IndexPath) {
            guard indexPath.item < cachedWaypoints.count else { return }
            if parent.selectedBubbleIndex == indexPath.item {
                parent.selectedBubbleIndex = nil
            } else {
                parent.selectedBubbleIndex = indexPath.item
            }
        }

        // MARK: - Strict ASCII `UITextFieldDelegate`

        func textField(
            _ textField: UITextField,
            shouldChangeCharactersIn range: NSRange,
            replacementString string: String
        ) -> Bool {
            let current = textField.text ?? ""
            guard let swiftRange = Range(range, in: current) else { return false }
            let proposed = current.replacingCharacters(in: swiftRange, with: string)

            guard let normalized = RouteAsciiInput.normalized(proposed) else {
                parent.onRejectNonAscii()
                return false
            }

            if normalized.contains(" ") {
                let endsWithSpace = normalized.hasSuffix(" ")
                let words = RouteAsciiInput.words(from: normalized)
                if endsWithSpace {
                    textField.text = ""
                    parent.draftToken = ""
                    if !words.isEmpty {
                        parent.onCommitTokens(words)
                    }
                } else if words.count >= 2 {
                    let committed = Array(words.dropLast())
                    let remainder = words.last ?? ""
                    textField.text = remainder
                    parent.draftToken = remainder
                    parent.onCommitTokens(committed)
                } else {
                    let trimmed = words.first ?? ""
                    textField.text = trimmed
                    parent.draftToken = trimmed
                }
                containerView?.setNeedsLayout()
                return false
            }

            textField.text = normalized
            parent.draftToken = normalized
            containerView?.setNeedsLayout()
            return false
        }

        func textFieldShouldReturn(_ textField: UITextField) -> Bool {
            let words = RouteAsciiInput.words(from: textField.text ?? "")
            if !words.isEmpty {
                textField.text = ""
                parent.draftToken = ""
                parent.onCommitTokens(words)
            } else {
                textField.resignFirstResponder()
            }
            return false
        }

        func handleEmptyBackspace() {
            if let sel = parent.selectedBubbleIndex, sel >= 0, sel < cachedWaypoints.count {
                parent.onDeleteAt(sel)
            } else if !cachedWaypoints.isEmpty {
                parent.onDeleteAt(cachedWaypoints.count - 1)
            }
        }

        // MARK: - Drag & Drop Reordering

        func collectionView(
            _ collectionView: UICollectionView,
            itemsForBeginning session: any UIDragSession,
            at indexPath: IndexPath
        ) -> [UIDragItem] {
            guard indexPath.item < cachedWaypoints.count else { return [] }
            let wp = cachedWaypoints[indexPath.item]
            let provider = NSItemProvider(object: wp.ident as NSString)
            let item = UIDragItem(itemProvider: provider)
            item.localObject = wp.id
            return [item]
        }

        func collectionView(
            _ collectionView: UICollectionView,
            dropSessionDidUpdate session: any UIDropSession,
            withDestinationIndexPath destinationIndexPath: IndexPath?
        ) -> UICollectionViewDropProposal {
            guard collectionView.hasActiveDrag, !cachedWaypoints.isEmpty else {
                return UICollectionViewDropProposal(operation: .cancel)
            }
            return UICollectionViewDropProposal(operation: .move, intent: .insertAtDestinationIndexPath)
        }

        func collectionView(
            _ collectionView: UICollectionView,
            performDropWith coordinator: any UICollectionViewDropCoordinator
        ) {
            guard let dropItem = coordinator.items.first,
                  let sourceIndexPath = dropItem.sourceIndexPath,
                  sourceIndexPath.item < cachedWaypoints.count else {
                return
            }
            let rawDest = coordinator.destinationIndexPath?.item ?? (cachedWaypoints.count - 1)
            let clampedDest = min(max(rawDest, 0), max(cachedWaypoints.count - 1, 0))
            guard clampedDest != sourceIndexPath.item else { return }

            let destIndexPath = IndexPath(item: clampedDest, section: 0)
            var reordered = cachedWaypoints
            let moved = reordered.remove(at: sourceIndexPath.item)
            reordered.insert(moved, at: clampedDest)
            cachedWaypoints = reordered

            containerView?.setNeedsLayout()
            containerView?.layoutIfNeeded()
            collectionView.performBatchUpdates {
                collectionView.moveItem(at: sourceIndexPath, to: destIndexPath)
            }
            coordinator.drop(dropItem.dragItem, toItemAt: destIndexPath)
            parent.onReorder(reordered)
        }
    }
}

/// ForeFlight-style color-coded route capsule (`[▸] IDENT [×]`) — clean identifier without `"APT"` clutter.
private final class PadRouteChipCell: UICollectionViewCell {
    static let reuseId = "PadRouteChipCell"

    private let activeIcon = UIImageView()
    private let identLabel = UILabel()
    private let removeButton = UIButton(type: .system)
    private var onRemoveHandler: (() -> Void)?

    override init(frame: CGRect) {
        super.init(frame: frame)
        contentView.layer.cornerRadius = 15
        contentView.layer.cornerCurve = .continuous
        contentView.layer.borderWidth = 1.2

        activeIcon.image = UIImage(systemName: "arrowtriangle.right.fill")
        activeIcon.tintColor = PadTheme.uiPlan
        activeIcon.contentMode = .scaleAspectFit
        activeIcon.translatesAutoresizingMaskIntoConstraints = false

        identLabel.font = UIFont.monospacedSystemFont(ofSize: 13, weight: .bold)
        identLabel.textColor = PadTheme.uiValue
        identLabel.translatesAutoresizingMaskIntoConstraints = false

        let xCfg = UIImage.SymbolConfiguration(pointSize: 9.5, weight: .bold)
        removeButton.setImage(UIImage(systemName: "xmark", withConfiguration: xCfg), for: .normal)
        removeButton.tintColor = PadTheme.uiLabel
        removeButton.translatesAutoresizingMaskIntoConstraints = false
        removeButton.addTarget(self, action: #selector(handleRemoveTap), for: .touchUpInside)

        let stack = UIStackView(arrangedSubviews: [activeIcon, identLabel, removeButton])
        stack.axis = .horizontal
        stack.alignment = .center
        stack.spacing = 4
        stack.translatesAutoresizingMaskIntoConstraints = false
        contentView.addSubview(stack)

        NSLayoutConstraint.activate([
            activeIcon.widthAnchor.constraint(equalToConstant: 8.5),
            activeIcon.heightAnchor.constraint(equalToConstant: 8.5),
            removeButton.widthAnchor.constraint(equalToConstant: 16),
            removeButton.heightAnchor.constraint(equalToConstant: 20),
            stack.leadingAnchor.constraint(equalTo: contentView.leadingAnchor, constant: 10),
            stack.trailingAnchor.constraint(equalTo: contentView.trailingAnchor, constant: -6),
            stack.centerYAnchor.constraint(equalTo: contentView.centerYAnchor),
        ])
    }

    @available(*, unavailable)
    required init?(coder: NSCoder) {
        return nil
    }

    func configure(
        waypoint: FlightPlanWaypoint,
        isActiveLeg: Bool,
        isPastLeg: Bool,
        isSelected: Bool,
        onRemove: @escaping () -> Void
    ) {
        onRemoveHandler = onRemove
        activeIcon.isHidden = !isActiveLeg
        identLabel.text = waypoint.ident

        if isSelected {
            contentView.backgroundColor = isActiveLeg
                ? PadTheme.uiPlan.withAlphaComponent(0.28)
                : PadTheme.uiControl
            contentView.layer.borderWidth = 2.0
            contentView.layer.borderColor = PadTheme.uiCommanded.cgColor
            identLabel.textColor = PadTheme.uiValue
            removeButton.tintColor = PadTheme.uiValue
        } else if isActiveLeg {
            contentView.backgroundColor = PadTheme.uiPlan.withAlphaComponent(0.22)
            contentView.layer.borderWidth = 1.5
            contentView.layer.borderColor = PadTheme.uiPlan.cgColor
            identLabel.textColor = PadTheme.uiPlan
            removeButton.tintColor = PadTheme.uiPlan.withAlphaComponent(0.85)
        } else if isPastLeg {
            contentView.backgroundColor = PadTheme.uiSurface
            contentView.layer.borderWidth = 1.0
            contentView.layer.borderColor = PadTheme.uiSeparator.cgColor
            identLabel.textColor = PadTheme.uiDisabled
            removeButton.tintColor = PadTheme.uiDisabled
        } else {
            contentView.backgroundColor = PadTheme.uiControl
            contentView.layer.borderWidth = 1.0
            contentView.layer.borderColor = PadTheme.uiSeparator.cgColor
            identLabel.textColor = PadTheme.uiValue
            removeButton.tintColor = PadTheme.uiLabel
        }
    }

    static func preferredWidth(for waypoint: FlightPlanWaypoint, isActiveLeg: Bool) -> CGSize.WidthValue {
        let identFont = UIFont.monospacedSystemFont(ofSize: 13, weight: .bold)
        let identW = (waypoint.ident as NSString).size(withAttributes: [.font: identFont]).width
        let activeW: CGFloat = isActiveLeg ? 12.5 : 0
        return max(ceil(10 + activeW + identW + 4 + 16 + 6), 58)
    }

    @objc private func handleRemoveTap() {
        onRemoveHandler?()
    }
}

private extension CGSize {
    typealias WidthValue = CGFloat
}

/// `UITextField` subclass intercepting Backspace on empty input and enforcing ASCII (`Pilotage` `RouteInputField`).
final class PadAsciiBackspaceTextField: UITextField {
    var onDeleteWhenEmpty: (() -> Void)?
    var onRejectNonAscii: (() -> Void)?
    var onTextEdited: ((String) -> Void)?

    override func deleteBackward() {
        let wasEmpty = (text ?? "").isEmpty
        super.deleteBackward()
        if wasEmpty {
            onDeleteWhenEmpty?()
        } else {
            onTextEdited?(text ?? "")
        }
    }

    override func insertText(_ inserted: String) {
        if inserted == "\n" {
            _ = delegate?.textFieldShouldReturn?(self)
            return
        }
        guard RouteAsciiInput.normalized(inserted) != nil else {
            onRejectNonAscii?()
            return
        }
        super.insertText(inserted)
    }
}

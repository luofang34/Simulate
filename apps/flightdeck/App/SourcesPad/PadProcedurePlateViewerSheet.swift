import PDFKit
import SwiftUI
import FlightDeckKit

/// Identifiable presentation payload for the full-screen `PDFKit` terminal procedure plate viewer.
struct PresentedPlateRequest: Identifiable, Equatable {
    let descriptor: ProcedurePlateDescriptor
    let procedure: CifpProcedureSummary?
    let runwayTransitionIdent: String?
    let enrouteOrFeederTransitionIdent: String?
    var startWithChromeHidden: Bool = false

    var id: String { descriptor.id }
}

/// ForeFlight-aligned full-screen `PDFKit` procedure plate & airport diagram viewer:
/// - Presented edge-to-edge (`.fullScreenCover`), never as a floating modal sheet occluding map buttons.
/// - Zero full-width navigation bars or redundant metadata banners; the chart occupies 100% of the canvas.
/// - **Single-tap anywhere on the plate** toggles minimal corner pills on/off for 100% distraction-free viewing.
struct PadProcedurePlateViewerSheet: View {
    @Bindable var controller: FlightDeckController
    let request: PresentedPlateRequest
    let onPinnedToMap: (PinnedMapPlate) -> Void
    @Environment(\.dismiss) private var dismiss

    @State private var activeRequest: PresentedPlateRequest
    @State private var isChromeVisible: Bool
    @State private var nightInvert: Bool = false

    init(
        controller: FlightDeckController,
        request: PresentedPlateRequest,
        onPinnedToMap: @escaping (PinnedMapPlate) -> Void
    ) {
        self.controller = controller
        self.request = request
        self.onPinnedToMap = onPinnedToMap
        self._activeRequest = State(initialValue: request)
        self._isChromeVisible = State(initialValue: !request.startWithChromeHidden)
    }

    var body: some View {
        let pdfBytes = controller.loadPlatePdfData(
            for: activeRequest.descriptor,
            procedure: activeRequest.procedure,
            runwayTransitionIdent: activeRequest.runwayTransitionIdent,
            enrouteOrFeederTransitionIdent: activeRequest.enrouteOrFeederTransitionIdent
        )

        ZStack {
            (nightInvert ? Color.black : Color(white: 0.94))
                .ignoresSafeArea()

            if let data = pdfBytes {
                PlatePdfKitView(
                    pdfData: data,
                    plateKey: activeRequest.id,
                    onSingleTap: {
                        withAnimation(.easeOut(duration: 0.16)) {
                            isChromeVisible.toggle()
                        }
                    }
                )
                .colorInvertIfEnabled(nightInvert)
                .ignoresSafeArea()
            } else {
                ContentUnavailableView(
                    "Plate Unavailable",
                    systemImage: "doc.text.magnifyingglass",
                    description: Text("Sync the current 28-day US Terminal Procedure Plates pack in Downloads.")
                )
                .onTapGesture {
                    withAnimation(.easeOut(duration: 0.16)) {
                        isChromeVisible.toggle()
                    }
                }
            }

            if isChromeVisible {
                topLeadingBinderPill
                    .floatingControlPlacement(.topLeading)
                    .transition(.opacity)

                topTrailingActionPill
                    .floatingControlPlacement(.topTrailing)
                    .transition(.opacity)
            }
        }
        .statusBarHidden(!isChromeVisible)
        .preferredColorScheme(nightInvert ? .dark : .light)
    }

    // MARK: - Minimal Non-Occluding Corner Pills (ForeFlight Plates Style)

    private var topLeadingBinderPill: some View {
        HStack(spacing: 8) {
            Button {
                dismiss()
            } label: {
                HStack(spacing: 4) {
                    Image(systemName: "chevron.left")
                        .font(.system(size: 12, weight: .bold))
                    Text("Map")
                        .font(.system(size: 12.5, weight: .bold))
                }
                .foregroundStyle(PadTheme.commanded)
                .padding(.vertical, 3)
            }
            .buttonStyle(.plain)

            let siblingPlates = binderPlatesForCurrentRoute
            if !siblingPlates.isEmpty {
                Rectangle()
                    .fill(PadTheme.separator)
                    .frame(width: 1, height: 15)

                Menu {
                    ForEach(binderAirportCodes, id: \.self) { apt in
                        let aptPlates = controller.availablePlates(forAirport: apt)
                        if !aptPlates.isEmpty {
                            Section(apt) {
                                ForEach(aptPlates) { plate in
                                    Button {
                                        let matchingProc = controller.availableCifpProcedures(forAirport: apt)
                                            .first(where: { $0.procedureIdent == plate.cifpIdent })
                                        activeRequest = PresentedPlateRequest(
                                            descriptor: plate,
                                            procedure: matchingProc,
                                            runwayTransitionIdent: nil,
                                            enrouteOrFeederTransitionIdent: nil
                                        )
                                    } label: {
                                        Label(
                                            plate.title,
                                            systemImage: plate.id == activeRequest.descriptor.id ? "checkmark" : plate.kind.menuSymbol
                                        )
                                    }
                                }
                            }
                        }
                    }
                } label: {
                    HStack(spacing: 4) {
                        Text(activeRequest.descriptor.airportIdent)
                            .font(.system(size: 11.5, weight: .heavy, design: .monospaced))
                            .foregroundStyle(PadTheme.value)
                        Image(systemName: "chevron.down")
                            .font(.system(size: 9.5, weight: .bold))
                            .foregroundStyle(PadTheme.label)
                    }
                    .padding(.vertical, 3)
                }
            }
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 7)
        .cockpitFloatingCapsule()
    }

    private var topTrailingActionPill: some View {
        HStack(spacing: 10) {
            Button {
                nightInvert.toggle()
            } label: {
                Image(systemName: nightInvert ? "sun.max.fill" : "moon.stars.fill")
                    .font(.system(size: 13.5, weight: .semibold))
                    .foregroundStyle(nightInvert ? PadTheme.commanded : PadTheme.value)
                    .frame(width: 26, height: 24)
            }
            .buttonStyle(.plain)
            .accessibilityLabel(nightInvert ? "Day Paper Colors" : "Night Invert Colors")

            if activeRequest.descriptor.supportsMapOverlay {
                Rectangle()
                    .fill(PadTheme.separator)
                    .frame(width: 1, height: 15)

                Button {
                    if nightInvert {
                        controller.pinnedPlateInverted = true
                    }
                    if let pinned = controller.pinPlateToMap(
                        activeRequest.descriptor,
                        procedure: activeRequest.procedure,
                        runwayTransitionIdent: activeRequest.runwayTransitionIdent,
                        enrouteOrFeederTransitionIdent: activeRequest.enrouteOrFeederTransitionIdent
                    ) {
                        onPinnedToMap(pinned)
                        dismiss()
                    }
                } label: {
                    HStack(spacing: 4) {
                        Image(systemName: "map.fill")
                            .font(.system(size: 11, weight: .semibold))
                        Text("Send to Map")
                            .font(.system(size: 11.5, weight: .bold))
                    }
                    .foregroundStyle(PadTheme.commanded)
                    .padding(.vertical, 2)
                }
                .buttonStyle(.plain)
            }

            if let proc = activeRequest.procedure {
                Rectangle()
                    .fill(PadTheme.separator)
                    .frame(width: 1, height: 15)

                Button {
                    controller.applyCifpProcedure(
                        proc,
                        runwayTransitionIdent: activeRequest.runwayTransitionIdent,
                        enrouteOrFeederTransitionIdent: activeRequest.enrouteOrFeederTransitionIdent
                    )
                    dismiss()
                } label: {
                    HStack(spacing: 4) {
                        Image(systemName: "plus.circle.fill")
                            .font(.system(size: 11.5, weight: .semibold))
                        Text("Route")
                            .font(.system(size: 11.5, weight: .bold))
                    }
                    .foregroundStyle(PadTheme.plan)
                    .padding(.vertical, 2)
                }
                .buttonStyle(.plain)
            }
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 7)
        .cockpitFloatingCapsule()
    }

    private var binderAirportCodes: [String] {
        var codes: [String] = [activeRequest.descriptor.airportIdent]
        for wp in controller.flightPlanWaypoints where wp.kind == .airport {
            if !codes.contains(wp.ident) {
                codes.append(wp.ident)
            }
        }
        return codes
    }

    private var binderPlatesForCurrentRoute: [ProcedurePlateDescriptor] {
        binderAirportCodes.flatMap { controller.availablePlates(forAirport: $0) }
    }
}

private extension ProcedurePlateKind {
    var menuSymbol: String {
        switch self {
        case .airportDiagram: "airplane.circle"
        case .approach: "scope"
        case .arrival: "arrow.down.right.circle"
        case .departure: "arrow.up.right.circle"
        }
    }
}

private final class SingleTapGestureDelegate: NSObject, UIGestureRecognizerDelegate {
    func gestureRecognizer(
        _ gestureRecognizer: UIGestureRecognizer,
        shouldRecognizeSimultaneouslyWith otherGestureRecognizer: UIGestureRecognizer
    ) -> Bool {
        true
    }

    func gestureRecognizer(
        _ gestureRecognizer: UIGestureRecognizer,
        shouldRequireFailureOf otherGestureRecognizer: UIGestureRecognizer
    ) -> Bool {
        if let otherTap = otherGestureRecognizer as? UITapGestureRecognizer,
           otherTap.numberOfTapsRequired >= 2 {
            return true
        }
        return false
    }
}

private final class AutoFitPdfView: PDFView {
    var loadedPlateKey: String = ""
    var onSingleTap: (() -> Void)?
    private var lastFittedSize: CGSize = .zero
    private var singleTapInstalled: Bool = false
    private let tapDelegate = SingleTapGestureDelegate()

    func installSingleTapIfNeeded() {
        guard !singleTapInstalled else { return }
        singleTapInstalled = true
        let tap = UITapGestureRecognizer(target: self, action: #selector(handleSingleTap(_:)))
        tap.numberOfTapsRequired = 1
        tap.numberOfTouchesRequired = 1
        tap.cancelsTouchesInView = false
        tap.delegate = tapDelegate
        addGestureRecognizer(tap)
    }

    func resetScaleFit() {
        lastFittedSize = .zero
        setNeedsLayout()
    }

    override func layoutSubviews() {
        super.layoutSubviews()
        guard bounds.width > 120, bounds.height > 120 else { return }
        if abs(bounds.width - lastFittedSize.width) > 4.0 || abs(bounds.height - lastFittedSize.height) > 4.0 {
            lastFittedSize = bounds.size
            let fit = scaleFactorForSizeToFit
            if fit > 0.05 {
                minScaleFactor = fit * 0.70
                maxScaleFactor = fit * 6.5
                scaleFactor = fit
                autoScales = true
            }
        }
    }

    @objc private func handleSingleTap(_ gesture: UITapGestureRecognizer) {
        if gesture.state == .ended {
            onSingleTap?()
        }
    }
}

private struct PlatePdfKitView: UIViewRepresentable {
    let pdfData: Data
    let plateKey: String
    let onSingleTap: () -> Void

    func makeUIView(context: Context) -> AutoFitPdfView {
        let view = AutoFitPdfView()
        view.displayMode = .singlePageContinuous
        view.displayDirection = .vertical
        view.displaysPageBreaks = false
        view.backgroundColor = .clear
        view.loadedPlateKey = plateKey
        view.onSingleTap = onSingleTap
        view.document = PDFDocument(data: pdfData)
        view.autoScales = true
        view.installSingleTapIfNeeded()
        return view
    }

    func updateUIView(_ view: AutoFitPdfView, context: Context) {
        view.onSingleTap = onSingleTap
        if view.document == nil || view.loadedPlateKey != plateKey {
            view.loadedPlateKey = plateKey
            view.document = PDFDocument(data: pdfData)
            view.resetScaleFit()
        }
    }
}

private extension View {
    @ViewBuilder
    func colorInvertIfEnabled(_ enabled: Bool) -> some View {
        if enabled {
            self.colorInvert()
        } else {
            self
        }
    }
}

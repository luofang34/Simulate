#if canImport(SwiftUI)
import CoreGraphics
import IndicateAppleDisplay
import SwiftUI

/// SwiftUI presentation of a ``PanelDisplay`` whose host `CALayer` and platform view are
/// forced completely transparent (`alpha = 0`), so conformal HUD / HWD strokes composite
/// directly over a live 3D outside scene without an opaque backing card.
public struct TransparentHudOverlay {
    private let display: PanelDisplay
    private let onOutcome: ((PanelFrameOutcome) -> Void)?

    public init(
        display: PanelDisplay,
        onOutcome: ((PanelFrameOutcome) -> Void)? = nil
    ) {
        self.display = display
        self.onOutcome = onOutcome
    }

    @MainActor
    private func makeView() -> InstrumentPanelView {
        let view = InstrumentPanelView(display: display)
        view.onOutcome = onOutcome
        Self.applyTransparency(to: view)
        return view
    }

    @MainActor
    fileprivate static func applyTransparency(to view: InstrumentPanelView) {
        #if canImport(UIKit)
        view.isOpaque = false
        view.backgroundColor = .clear
        view.isUserInteractionEnabled = false
        view.layer.isOpaque = false
        view.layer.backgroundColor = CGColor(red: 0, green: 0, blue: 0, alpha: 0)
        #elseif canImport(AppKit)
        view.layer?.isOpaque = false
        view.layer?.backgroundColor = CGColor(red: 0, green: 0, blue: 0, alpha: 0)
        #endif
    }
}

#if canImport(UIKit)
extension TransparentHudOverlay: UIViewRepresentable {
    public func makeUIView(context: Context) -> InstrumentPanelView {
        makeView()
    }

    public func updateUIView(_ view: InstrumentPanelView, context: Context) {
        view.display = display
        view.onOutcome = onOutcome
        Self.applyTransparency(to: view)
    }
}
#elseif canImport(AppKit)
extension TransparentHudOverlay: NSViewRepresentable {
    public func makeNSView(context: Context) -> InstrumentPanelView {
        makeView()
    }

    public func updateNSView(_ view: InstrumentPanelView, context: Context) {
        view.display = display
        view.onOutcome = onOutcome
        Self.applyTransparency(to: view)
    }
}
#endif
#endif

import SwiftUI
import UIKit

/// Shared layout metrics and safe-area-aware floating control placement for iPadOS.
///
/// Follows the `Pilotage` layout rule: floating controls sit over an `ignoresSafeArea()` map
/// canvas and apply `max(edgeInset, safe.<edge>)` through a single shared helper — never summing
/// fixed padding on top of system safe-area insets.
enum PadMetrics {
    static let edgeInset: CGFloat = 14
    static let buttonSize: CGFloat = 44
    static let cornerRadius: CGFloat = 14
    static let pillCornerRadius: CGFloat = 22
    static let controlSpacing: CGFloat = 8
    static let windowControlsClearance: CGFloat = 78
}

private struct FloatingControlPlacementModifier: ViewModifier {
    let alignment: Alignment

    func body(content: Content) -> some View {
        GeometryReader { proxy in
            let safe = proxy.safeAreaInsets
            let winSafe = Self.windowSafeAreaInsets
            let barHeight = Self.systemStatusBarHeight
            let isWindowed = Self.isStageManagerWindowed(width: proxy.size.width)
            let reservedTop = max(barHeight, max(safe.top, winSafe.top))
            let topPad = reservedTop > 0
                ? max(PadMetrics.edgeInset, reservedTop + 6)
                : PadMetrics.edgeInset
            let reservedBottom = max(safe.bottom, winSafe.bottom)
            let bottomPad = reservedBottom > 0
                ? max(PadMetrics.edgeInset, reservedBottom + 4)
                : PadMetrics.edgeInset
            let trailingPad = max(PadMetrics.edgeInset, max(safe.trailing, winSafe.right))
            let baseLeading = max(PadMetrics.edgeInset, max(safe.leading, winSafe.left))
            let leadingPad = (alignment == .topLeading && isWindowed)
                ? max(baseLeading, PadMetrics.windowControlsClearance)
                : baseLeading

            content
                .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: alignment)
                .padding(.top, topPad)
                .padding(.bottom, bottomPad)
                .padding(.leading, leadingPad)
                .padding(.trailing, trailingPad)
        }
        .ignoresSafeArea()
    }

    private static var activeWindowScene: UIWindowScene? {
        UIApplication.shared.connectedScenes
            .compactMap { $0 as? UIWindowScene }
            .first
    }

    private static var systemStatusBarHeight: CGFloat {
        activeWindowScene?.statusBarManager?.statusBarFrame.height ?? 0
    }

    private static var windowSafeAreaInsets: UIEdgeInsets {
        activeWindowScene?.keyWindow?.safeAreaInsets ?? .zero
    }

    private static func isStageManagerWindowed(width: CGFloat) -> Bool {
        guard let scene = activeWindowScene else {
            return false
        }
        return width + 24.0 < scene.screen.bounds.width
    }
}

extension View {
    /// Places a floating control cluster at `alignment` using `max(PadMetrics.edgeInset, safeArea)`.
    func floatingControlPlacement(_ alignment: Alignment) -> some View {
        modifier(FloatingControlPlacementModifier(alignment: alignment))
    }
}

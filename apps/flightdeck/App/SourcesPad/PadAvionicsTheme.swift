import SwiftUI
import UIKit
import FlightDeckKit

extension AvionicsRgb {
    /// SwiftUI sRGB color matching this exact `brand.toml [ui.*]` token.
    var swiftUIColor: Color {
        Color(.sRGB, red: redUnit, green: greenUnit, blue: blueUnit, opacity: 1.0)
    }

    /// UIKit sRGB color matching this exact `brand.toml [ui.*]` token.
    var uiColor: UIColor {
        UIColor(red: CGFloat(redUnit), green: CGFloat(greenUnit), blue: CGFloat(blueUnit), alpha: 1.0)
    }
}

/// Single-source-of-truth iPadOS EFB & `Sokoly` GCS human-factors theme backed by `AvionicsColorSystem`
/// (`14 CFR § 25.1322`, `FAA AC 25-11B §5.4`, `DOT/FAA/AM-06/15 & AM-07/24`, `Sokoly PRODUCT-UI.md`).
enum PadTheme {
    // MARK: - Neutral Dark Cockpit Surfaces (`[ui.neutral]`)

    /// Floating cockpit bar & modal chrome (`#0B0D11`).
    static let background = AvionicsColorSystem.background.swiftUIColor
    static let uiBackground = AvionicsColorSystem.background.uiColor

    /// Raised card, input well, & row surface (`#16191F`).
    static let surface = AvionicsColorSystem.surface.swiftUIColor
    static let uiSurface = AvionicsColorSystem.surface.uiColor

    /// Interactive button, neutral route chip, & unselected pill fill (`#262B33`).
    static let control = AvionicsColorSystem.control.swiftUIColor
    static let uiControl = AvionicsColorSystem.control.uiColor

    /// `1 pt` structural rule & border stroke (`#30363F`).
    static let separator = AvionicsColorSystem.separator.swiftUIColor
    static let uiSeparator = AvionicsColorSystem.separator.uiColor

    // MARK: - Neutral Ink (`[ui.neutral]`)

    /// Nominal measured & computed flight values (`GS`, `ALT`, `TRK`, `DIST`, `ETE`, `FUEL`, `#ECF0F3`, `16.97:1`).
    static let value = AvionicsColorSystem.value.swiftUIColor
    static let uiValue = AvionicsColorSystem.value.uiColor

    /// Field headers, units, nominal `SIM`/`OBS` status, & published `MEA`/`V3` metadata (`#8A94A0`, `6.32:1`).
    static let label = AvionicsColorSystem.label.swiftUIColor
    static let uiLabel = AvionicsColorSystem.label.uiColor

    /// Unavailable controls & sequenced (`past_leg`) route waypoints (`#68707C`).
    static let disabled = AvionicsColorSystem.disabled.swiftUIColor
    static let uiDisabled = AvionicsColorSystem.disabled.uiColor

    // MARK: - 5 Chromatic Operational State Tokens (`[ui.state]`)

    /// Operator-selected / armed target, `Next` waypoint, planned legs, selected tab/runway/plate (`#00FFFF`).
    static let commanded = AvionicsColorSystem.commanded.swiftUIColor
    static let uiCommanded = AvionicsColorSystem.commanded.uiColor

    /// Active GPS fly-to `TO` waypoint (`▸`), active route leg, active bearing, `Direct-To` (`#E850E8`).
    static let plan = AvionicsColorSystem.plan.swiftUIColor
    static let uiPlan = AvionicsColorSystem.plan.uiColor

    /// Confirmed automation mode, active GCS `LEASE`, live receiver link, `VALID` AIRAC pack (`#48CE88`).
    static let active = AvionicsColorSystem.active.swiftUIColor
    static let uiActive = AvionicsColorSystem.active.uiColor

    /// Caution / degraded / waiting link, `ARM` telegraph, `Track Up` / `ALIGN` / non-Earth notice (`#FFB020`).
    static let caution = AvionicsColorSystem.caution.swiftUIColor
    static let uiCaution = AvionicsColorSystem.caution.uiColor

    /// Warning / `EXPIRED · INVALID` navdata or chart, lost link, prohibited airspace (`#FF453A`).
    static let warning = AvionicsColorSystem.warning.swiftUIColor
    static let uiWarning = AvionicsColorSystem.warning.uiColor
}

extension View {
    /// Applies high-contrast dark cockpit floating chrome (`#0B0D11` @ `94%` over white paper charts & dark globe).
    func cockpitFloatingChrome(cornerRadius: CGFloat, borderTint: Color = PadTheme.separator) -> some View {
        self
            .background {
                RoundedRectangle(cornerRadius: cornerRadius, style: .continuous)
                    .fill(PadTheme.background.opacity(0.94))
                    .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: cornerRadius, style: .continuous))
            }
            .overlay(
                RoundedRectangle(cornerRadius: cornerRadius, style: .continuous)
                    .strokeBorder(borderTint, lineWidth: 1)
            )
            .shadow(color: .black.opacity(0.38), radius: 12, y: 5)
            .environment(\.colorScheme, .dark)
    }

    /// Applies high-contrast dark cockpit capsule chrome (`#0B0D11` @ `94%`).
    func cockpitFloatingCapsule(borderTint: Color = PadTheme.separator) -> some View {
        self
            .background {
                Capsule()
                    .fill(PadTheme.background.opacity(0.94))
                    .background(.ultraThinMaterial, in: Capsule())
            }
            .overlay(Capsule().strokeBorder(borderTint, lineWidth: 1))
            .shadow(color: .black.opacity(0.36), radius: 10, y: 4)
            .environment(\.colorScheme, .dark)
    }

    /// Applies raised cockpit card styling (`#16191F` fill + `1 pt` `#30363F` rule).
    func cockpitRaisedCard(cornerRadius: CGFloat = 14) -> some View {
        self
            .background(PadTheme.surface, in: RoundedRectangle(cornerRadius: cornerRadius, style: .continuous))
            .overlay(
                RoundedRectangle(cornerRadius: cornerRadius, style: .continuous)
                    .strokeBorder(PadTheme.separator, lineWidth: 1)
            )
    }
}

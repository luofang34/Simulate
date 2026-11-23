import CoreGraphics
import Foundation

/// Exact 8-bit sRGB color token in the unified FAA / `Sokoly` / `Indicate` avionics palette.
public struct AvionicsRgb: Sendable, Equatable, Hashable {
    /// Red channel (`0...255`).
    public let r: UInt8
    /// Green channel (`0...255`).
    public let g: UInt8
    /// Blue channel (`0...255`).
    public let b: UInt8

    /// Creates an 8-bit sRGB color token.
    public init(_ r: UInt8, _ g: UInt8, _ b: UInt8) {
        self.r = r
        self.g = g
        self.b = b
    }

    /// Normalized `[0.0, 1.0]` red component.
    public var redUnit: Double { Double(r) / 255.0 }
    /// Normalized `[0.0, 1.0]` green component.
    public var greenUnit: Double { Double(g) / 255.0 }
    /// Normalized `[0.0, 1.0]` blue component.
    public var blueUnit: Double { Double(b) / 255.0 }

    /// Uppercase `#RRGGBB` hex string for MapLibre style JSON and CSS/SVG export.
    public var hex: String {
        String(format: "#%02X%02X%02X", r, g, b)
    }

    /// CoreGraphics `CGColor` in sRGB with optional `alpha` (`0.0...1.0`).
    public func cgColor(alpha: CGFloat = 1.0) -> CGColor {
        CGColor(
            srgbRed: CGFloat(redUnit),
            green: CGFloat(greenUnit),
            blue: CGFloat(blueUnit),
            alpha: alpha
        )
    }

    /// Linearized IEC 61966-2-1 sRGB channel (`0.0...1.0`).
    public static func linearChannel(_ byte: UInt8) -> Double {
        let s = Double(byte) / 255.0
        if s <= 0.04045 {
            return s / 12.92
        }
        return pow((s + 0.055) / 1.055, 2.4)
    }

    /// Encodes a linear `[0.0, 1.0]` channel back to 8-bit sRGB.
    public static func encodeLinearChannel(_ linear: Double) -> UInt8 {
        let clamped = min(max(linear, 0.0), 1.0)
        let s: Double
        if clamped <= 0.0031308 {
            s = clamped * 12.92
        } else {
            s = 1.055 * pow(clamped, 1.0 / 2.4) - 0.055
        }
        return UInt8(min(max(Int((s * 255.0).rounded()), 0), 255))
    }

    /// WCAG 2.1 relative luminance `Y` (`0.0` black ... `1.0` D65 white).
    public var relativeLuminance: Double {
        let rl = Self.linearChannel(r)
        let gl = Self.linearChannel(g)
        let bl = Self.linearChannel(b)
        return 0.2126 * rl + 0.7152 * gl + 0.0722 * bl
    }

    /// WCAG 2.1 luminance contrast ratio (`1.0 ... 21.0`) against `other`.
    public func contrastRatio(against other: AvionicsRgb) -> Double {
        let l1 = relativeLuminance
        let l2 = other.relativeLuminance
        let hi = max(l1, l2)
        let lo = min(l1, l2)
        return (hi + 0.05) / (lo + 0.05)
    }

    /// Simulates full-severity (`1.0`) dichromacy using Machado, Oliveira & Fernandes (2009) in linear RGB.
    public func simulated(_ cvd: AvionicsCvdKind) -> AvionicsRgb {
        let lin = (Self.linearChannel(r), Self.linearChannel(g), Self.linearChannel(b))
        let m = cvd.linearMatrix
        let rOut = m.0.0 * lin.0 + m.0.1 * lin.1 + m.0.2 * lin.2
        let gOut = m.1.0 * lin.0 + m.1.1 * lin.1 + m.1.2 * lin.2
        let bOut = m.2.0 * lin.0 + m.2.1 * lin.1 + m.2.2 * lin.2
        return AvionicsRgb(
            Self.encodeLinearChannel(rOut),
            Self.encodeLinearChannel(gOut),
            Self.encodeLinearChannel(bOut)
        )
    }

    /// Converts sRGB (D65) to CIE 1976 `L*a*b*`.
    public var cieLab: (l: Double, a: Double, b: Double) {
        let rl = Self.linearChannel(r)
        let gl = Self.linearChannel(g)
        let bl = Self.linearChannel(b)
        let x = (0.4124 * rl + 0.3576 * gl + 0.1805 * bl) / 0.95047
        let y = 0.2126 * rl + 0.7152 * gl + 0.0722 * bl
        let z = (0.0193 * rl + 0.1192 * gl + 0.9505 * bl) / 1.08883

        func f(_ t: Double) -> Double {
            if t > 216.0 / 24389.0 {
                return pow(t, 1.0 / 3.0)
            }
            return (24389.0 / 27.0 * t + 16.0) / 116.0
        }

        let fx = f(x)
        let fy = f(y)
        let fz = f(z)
        return (116.0 * fy - 16.0, 500.0 * (fx - fy), 200.0 * (fy - fz))
    }

    /// Perceptual color difference `CIEDE2000` (`ΔE00`, Sharma et al. 2005).
    public func deltaE2000(to other: AvionicsRgb) -> Double {
        let (l1, a1, b1) = cieLab
        let (l2, a2, b2) = other.cieLab
        let cBar = (hypot(a1, b1) + hypot(a2, b2)) * 0.5
        let cBar7 = pow(cBar, 7.0)
        let c25_7 = pow(25.0, 7.0)
        let g = 0.5 * (1.0 - sqrt(cBar7 / (cBar7 + c25_7)))
        let a1p = a1 * (1.0 + g)
        let a2p = a2 * (1.0 + g)
        let c1p = hypot(a1p, b1)
        let c2p = hypot(a2p, b2)

        func positiveDeg(_ rad: Double) -> Double {
            let deg = rad * 180.0 / .pi
            let mod = deg.truncatingRemainder(dividingBy: 360.0)
            return mod < 0.0 ? mod + 360.0 : mod
        }

        let h1p = positiveDeg(atan2(b1, a1p))
        let h2p = positiveDeg(atan2(b2, a2p))
        let dl = l2 - l1
        let dc = c2p - c1p

        var dh = 0.0
        if c1p * c2p > 0.0 {
            dh = h2p - h1p
            if dh > 180.0 {
                dh -= 360.0
            } else if dh < -180.0 {
                dh += 360.0
            }
        }
        let dHh = 2.0 * sqrt(c1p * c2p) * sin((dh * 0.5) * .pi / 180.0)
        let lBar = (l1 + l2) * 0.5
        let cBarP = (c1p + c2p) * 0.5
        var hBar = h1p + h2p
        if c1p * c2p > 0.0 {
            if abs(h1p - h2p) > 180.0 {
                hBar = (hBar + 360.0) * 0.5
            } else {
                hBar *= 0.5
            }
        }

        let deg2rad = Double.pi / 180.0
        let t = 1.0
            - 0.17 * cos((hBar - 30.0) * deg2rad)
            + 0.24 * cos((2.0 * hBar) * deg2rad)
            + 0.32 * cos((3.0 * hBar + 6.0) * deg2rad)
            - 0.20 * cos((4.0 * hBar - 63.0) * deg2rad)
        let lShift = lBar - 50.0
        let sl = 1.0 + 0.015 * (lShift * lShift) / sqrt(20.0 + lShift * lShift)
        let sc = 1.0 + 0.045 * cBarP
        let sh = 1.0 + 0.015 * cBarP * t
        let cBarP7 = pow(cBarP, 7.0)
        let dTheta = 60.0 * exp(-pow((hBar - 275.0) / 25.0, 2.0))
        let rt = -2.0 * sqrt(cBarP7 / (cBarP7 + c25_7)) * sin(dTheta * deg2rad)

        let lTerm = dl / sl
        let cTerm = dc / sc
        let hTerm = dHh / sh
        return sqrt(lTerm * lTerm + cTerm * cTerm + hTerm * hTerm + rt * cTerm * hTerm)
    }
}

/// Color-vision deficiency classes audited for cockpit state discriminability (`DOT/FAA/AM-06/15`).
public enum AvionicsCvdKind: String, CaseIterable, Sendable {
    case protanopia
    case deuteranopia
    case tritanopia

    var linearMatrix: ((Double, Double, Double), (Double, Double, Double), (Double, Double, Double)) {
        switch self {
        case .protanopia:
            ((0.152286, 1.052583, -0.204868), (0.114503, 0.786281, 0.099216), (-0.003882, -0.048116, 1.051998))
        case .deuteranopia:
            ((0.367322, 0.860646, -0.227968), (0.280085, 0.672501, 0.047413), (-0.011820, 0.042940, 0.968881))
        case .tritanopia:
            ((1.255528, -0.076749, -0.178779), (-0.078411, 0.930809, 0.147602), (0.004733, 0.691367, 0.303900))
        }
    }
}

/// Unified FAA Human-Factors & `Sokoly` (`brand.toml [ui.*]`) / `Indicate` (`ADR-0029`) color system.
///
/// Grounded in:
/// - **14 CFR § 25.1322 / § 23.2615 & FAA AC 25-11B §5.4**: Red strictly reserved for Warning/Invalid,
///   Amber strictly reserved for Caution/Degraded, Green for Engaged/Valid mode, Magenta (`#E850E8`) for
///   Active GPS Guidance, Cyan (`#00FFFF`) for Operator-Selected/Commanded targets, and Quiet-Dark-Cockpit
///   White (`#ECF0F3`) for all nominal measured & computed flight parameters (`GS`, `ALT`, `TRK`, `DIST`, `ETE`, `FUEL`).
/// - **FAA CAMI `DOT/FAA/AM-06/15` & `DOT/FAA/AM-07/24` (Dr. Jing Xing)**: `<= 6` semantic data colors on
///   operational displays, avoiding red-pink-magenta hue confusion (`ΔE00 >= 10` across normal & CVD vision),
///   paired with redundant geometric/text coding (`▸`, badges, outlines).
/// - **FAA Volpe `DOT/FAA/TC-13/44` & `RTCA DO-257B`**: High-contrast dark casing (`#0B0D11`) for map
///   routes/labels over bright paper charts and monotonic dark surface hierarchy.
public enum AvionicsColorSystem {
    // MARK: - `[ui.neutral]` Quiet Dark Cockpit Surfaces & Ink (`brand.toml`)

    /// Floating cockpit bar, modal chrome, and map symbol/line casing (`#0B0D11`).
    public static let background = AvionicsRgb(11, 13, 17)
    /// Raised card, input well, and active table row fill (`#16191F`).
    public static let surface = AvionicsRgb(22, 25, 31)
    /// Interactive button, neutral route chip, and unselected pill fill (`#262B33`).
    public static let control = AvionicsRgb(38, 43, 51)
    /// `1 pt` structural rule and border stroke (`#30363F`).
    public static let separator = AvionicsRgb(48, 54, 63)

    /// Nominal measured & computed flight values (`GS`, `ALT`, `TRK`, `DIST`, `ETE`, `FUEL`, `#ECF0F3`, `16.97:1`).
    public static let value = AvionicsRgb(236, 240, 243)
    /// Field headers, units, nominal `SIM`/`OBS` status, and published `MEA`/`V3` metadata (`#8A94A0`, `6.32:1`).
    public static let label = AvionicsRgb(138, 148, 160)
    /// Unavailable controls and sequenced (`past_leg`) route waypoints (`#68707C`, `3.52:1` on `surface`).
    public static let disabled = AvionicsRgb(104, 112, 124)

    // MARK: - `[ui.state]` 5 Chromatic Operational State Tokens (+ `value` = 6 Semantic Data Colors)

    /// Operator-selected / armed target, `Next` waypoint, planned legs, selected runway/tab/plate (`#00FFFF`, `15.51:1`).
    public static let commanded = AvionicsRgb(0, 255, 255)
    /// Active GPS fly-to `TO` waypoint (`▸`), active route leg, active leg bearing, `Direct-To` (`#E850E8`, `6.24:1`).
    public static let plan = AvionicsRgb(232, 80, 232)
    /// Confirmed automation mode, active GCS `LEASE`, live receiver link, `VALID` AIRAC pack (`#48CE88`, `9.69:1`).
    public static let active = AvionicsRgb(72, 206, 136)
    /// Caution / degraded / waiting link, `ARM` telegraph, `Track Up` / `ALIGN` / non-Earth notice (`#FFB020`, `10.63:1`).
    public static let caution = AvionicsRgb(255, 176, 32)
    /// Warning / `EXPIRED · INVALID` navdata or chart, lost link, prohibited airspace (`#FF453A`, `5.71:1`).
    public static let warning = AvionicsRgb(255, 69, 58)

    // MARK: - `[ui.affiliation]` & `[ui.canvas]` Bounds (`brand.toml`)

    /// Friendly track symbol fill inside APP-6 / ADS-B frame (`#80E0FF`).
    public static let friend = AvionicsRgb(128, 224, 255)
    /// Hostile / Resolution-Advisory track fill (`#FF8080`).
    public static let hostile = AvionicsRgb(255, 128, 128)
    /// Neutral track fill (`#AAFFAA`).
    public static let neutralAffiliation = AvionicsRgb(170, 255, 170)
    /// Unknown / Traffic-Advisory track fill (`#FFFF80`).
    public static let unknownAffiliation = AvionicsRgb(255, 255, 128)

    /// Darkest map underlay floor (`#121417`).
    public static let canvasFloor = AvionicsRgb(18, 20, 23)
    /// Brightest uncased dark-canvas ceiling (`#3A3D42`).
    public static let canvasCeiling = AvionicsRgb(58, 61, 66)

    /// Named semantic data palette (`<= 6` hues per FAA `DOT/FAA/AM-07/24`).
    public static let semanticDataColors: [(name: String, rgb: AvionicsRgb)] = [
        ("value", value),
        ("commanded", commanded),
        ("plan", plan),
        ("active", active),
        ("caution", caution),
        ("warning", warning),
    ]

    /// Audits WCAG AA text/graphic contrast, monotonic surface luminance, `<= 6` semantic hues,
    /// and `CIEDE2000 ΔE00 >= 10.0` across normal and dichromatic vision. Returns empty array when compliant.
    public static func verifyHumanFactorsInvariants() -> [String] {
        var violations: [String] = []
        let textMin = 4.5
        let graphicMin = 3.0
        let distinctMin = 10.0

        // 1. Xing `DOT/FAA/AM-07/24` Rule of Six.
        if semanticDataColors.count > 6 {
            violations.append("semanticDataColors count \(semanticDataColors.count) > 6")
        }

        // 2. Monotonic surface luminance (`background < surface < control < separator`).
        let surfaces: [(String, AvionicsRgb)] = [
            ("background", background),
            ("surface", surface),
            ("control", control),
            ("separator", separator),
        ]
        for idx in 1..<surfaces.count {
            if surfaces[idx].1.relativeLuminance <= surfaces[idx - 1].1.relativeLuminance {
                violations.append("non-monotonic luminance: \(surfaces[idx - 1].0) -> \(surfaces[idx].0)")
            }
        }

        // 3. WCAG AA `>= 4.5:1` text contrast on `background` and `surface`, plus `value` & `label` on `control`.
        let textTokens: [(String, AvionicsRgb)] = [("label", label)] + semanticDataColors
        for (bgName, bgRgb) in [("background", background), ("surface", surface)] {
            for (fgName, fgRgb) in textTokens {
                let ratio = fgRgb.contrastRatio(against: bgRgb)
                if ratio < textMin {
                    violations.append("\(fgName) on \(bgName) contrast \(String(format: "%.2f", ratio)) < \(textMin)")
                }
            }
        }
        if value.contrastRatio(against: control) < textMin {
            violations.append("value on control contrast < \(textMin)")
        }
        if label.contrastRatio(against: control) < textMin {
            violations.append("label on control contrast < \(textMin)")
        }
        if disabled.contrastRatio(against: surface) < graphicMin {
            violations.append("disabled on surface contrast < \(graphicMin)")
        }

        // 4. Graphic contrast `>= 3.0:1` (`value >= 4.5:1`) against `canvasCeiling` (`#3A3D42`).
        if value.contrastRatio(against: canvasCeiling) < textMin {
            violations.append("value on canvasCeiling contrast < \(textMin)")
        }
        for (fgName, fgRgb) in semanticDataColors where fgName != "value" {
            let ratio = fgRgb.contrastRatio(against: canvasCeiling)
            if ratio < graphicMin {
                violations.append("\(fgName) on canvasCeiling contrast \(String(format: "%.2f", ratio)) < \(graphicMin)")
            }
        }

        // 5. Pairwise `CIEDE2000 ΔE00 >= 10.0` in normal vision and worst-case protan/deutan/tritan
        //    (with `commanded` vs `value` exempted under CVD per `PRODUCT-UI.md §2` because commanded targets
        //    always carry redundant `→` / pill / bug shape coding).
        for i in 0..<semanticDataColors.count {
            for j in (i + 1)..<semanticDataColors.count {
                let a = semanticDataColors[i]
                let b = semanticDataColors[j]
                let deNormal = a.rgb.deltaE2000(to: b.rgb)
                if deNormal < distinctMin {
                    violations.append("\(a.name) vs \(b.name) normal ΔE00 \(String(format: "%.2f", deNormal)) < \(distinctMin)")
                }
                let isShapeCuedPair = Set([a.name, b.name]) == Set(["commanded", "value"])
                if !isShapeCuedPair {
                    for cvd in AvionicsCvdKind.allCases {
                        let deCvd = a.rgb.simulated(cvd).deltaE2000(to: b.rgb.simulated(cvd))
                        if deCvd < distinctMin {
                            violations.append("\(a.name) vs \(b.name) \(cvd.rawValue) ΔE00 \(String(format: "%.2f", deCvd)) < \(distinctMin)")
                        }
                    }
                }
            }
        }

        return violations
    }
}

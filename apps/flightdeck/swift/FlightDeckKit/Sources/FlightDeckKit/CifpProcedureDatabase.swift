import Foundation

/// ARINC 424-18 terminal procedure family compiled in `FBCIFP01` (`cifp_procedures.bin`).
public enum CifpProcedureKind: UInt8, CaseIterable, Codable, Identifiable, Sendable {
    /// Standard Instrument Departure (`PD`).
    case sid = 0
    /// Standard Terminal Arrival Route (`PE`).
    case star = 1
    /// Instrument Approach Procedure (`PF`).
    case approach = 2

    public var id: UInt8 { rawValue }

    public var badge: String {
        switch self {
        case .sid: "Departure"
        case .star: "Arrival"
        case .approach: "Approach"
        }
    }

    public var sectionTitle: String {
        switch self {
        case .sid: "Departure (SID)"
        case .star: "Arrival (STAR)"
        case .approach: "Instrument Approach"
        }
    }
}

/// ARINC 424-18 altitude constraint decoded from `FBCIFP01` procedure legs.
public enum CifpAltitudeConstraint: Equatable, Sendable {
    case none
    case at(feet: Int)
    case atOrAbove(feet: Int)
    case atOrBelow(feet: Int)
    case between(upperFeet: Int, lowerFeet: Int)

    public var conciseLabel: String {
        switch self {
        case .none:
            return ""
        case .at(let ft):
            return "@\(Self.formatAltitude(ft))"
        case .atOrAbove(let ft):
            return "+\(Self.formatAltitude(ft))"
        case .atOrBelow(let ft):
            return "-\(Self.formatAltitude(ft))"
        case .between(let hi, let lo):
            return "\(Self.formatAltitude(lo))-\(Self.formatAltitude(hi))"
        }
    }

    private static func formatAltitude(_ ft: Int) -> String {
        if ft >= 18_000 && ft % 100 == 0 {
            return "FL\(ft / 100)"
        }
        return "\(ft) FT"
    }
}

/// Single named transition or common core within a CIFP procedure (`Table 4`).
public struct CifpTransitionSummary: Equatable, Identifiable, Sendable {
    /// Transition identifier (`"PIYIN"`, `"RW22B"`, `"BIGGY"`, or `""` for common core / final).
    public let ident: String
    /// Phase (`0 = Runway`, `1 = CommonCore`, `2 = EnrouteOrFeeder`).
    public let phase: UInt8
    /// ARINC 424 route type byte.
    public let routeType: UInt8
    /// Number of legs in this transition.
    public let legCount: Int
    fileprivate let firstLegIndex: Int

    public var id: String { "\(phase):\(ident.isEmpty ? "CORE" : ident)" }

    public var displayLabel: String {
        if ident.isEmpty {
            return "Final / Core"
        }
        return ident
    }
}

/// Decoded procedure leg from `Table 5` of `cifp_procedures.bin`.
public struct CifpLegSummary: Equatable, Sendable {
    public let fixIdent: String
    public let pathTerminator: String
    public let waypointKind: WaypointKind
    public let hasCoordinates: Bool
    public let latitudeDeg: Double
    public let longitudeDeg: Double
    public let altitudeConstraint: CifpAltitudeConstraint
    public let speedLimitKt: Int
    public let isMissedApproach: Bool
    public let isIaf: Bool
    public let isFaf: Bool
    public let isMapt: Bool

    public var combinedConstraintLabel: String {
        var parts: [String] = []
        let alt = altitudeConstraint.conciseLabel
        if !alt.isEmpty {
            parts.append(alt)
        }
        if speedLimitKt > 0 {
            parts.append("\(speedLimitKt) KT")
        }
        if isIaf { parts.append("IAF") }
        if isFaf { parts.append("FAF") }
        if isMapt { parts.append("MAPt") }
        if isMissedApproach { parts.append("MISSED") }
        return parts.joined(separator: " · ")
    }
}

/// Airport terminal procedure (`SID`, `STAR`, or `IAP`) from `Table 3` of `cifp_procedures.bin`.
public struct CifpProcedureSummary: Equatable, Identifiable, Sendable {
    public let airportIdent: String
    public let procedureIdent: String
    public let kind: CifpProcedureKind
    public let routeType: UInt8
    public let transitions: [CifpTransitionSummary]

    public var id: String { "\(airportIdent):\(kind.rawValue):\(procedureIdent)" }

    public var runwayTransitions: [CifpTransitionSummary] {
        transitions.filter { $0.phase == 0 }
    }

    public var commonCoreTransition: CifpTransitionSummary? {
        transitions.first { $0.phase == 1 }
    }

    public var enrouteOrFeederTransitions: [CifpTransitionSummary] {
        transitions.filter { $0.phase == 2 }
    }

    /// Pilot-facing title (`"RNAV (GPS) RWY 22"`, `"ILS RWY 22L"`, `"PORTT4 SID"`, `"LVZ5 STAR"`).
    public var humanTitle: String {
        switch kind {
        case .sid:
            return "\(procedureIdent) SID"
        case .star:
            return "\(procedureIdent) STAR"
        case .approach:
            return Self.decodeApproachTitle(procedureIdent).title
        }
    }

    /// Concise badge for waypoint NavLog rows (`"SID PORTT4"`, `"STAR LVZ5"`, `"RNAV 22"`).
    public var shortBadge: String {
        switch kind {
        case .sid:
            return "SID \(procedureIdent)"
        case .star:
            return "STAR \(procedureIdent)"
        case .approach:
            return Self.decodeApproachTitle(procedureIdent).shortBadge
        }
    }

    /// Associated landing runway designator (`"22"`, `"04R"`, `"22L"`) when encoded in the procedure ID.
    public var associatedRunwayDesignator: String? {
        guard kind == .approach else { return nil }
        return Self.decodeApproachTitle(procedureIdent).runway
    }

    private static func decodeApproachTitle(_ code: String) -> (title: String, shortBadge: String, runway: String?) {
        guard let first = code.first else {
            return (code, code, nil)
        }
        let navPrefix: (long: String, short: String)? = switch first {
        case "I": ("ILS", "ILS")
        case "L": ("LOC", "LOC")
        case "B": ("LOC BC", "LOC BC")
        case "R": ("RNAV (GPS)", "RNAV")
        case "H": ("RNAV (RNP)", "RNP")
        case "P": ("GPS", "GPS")
        case "V": ("VOR", "VOR")
        case "D": ("VOR/DME", "VOR/DME")
        case "S": ("VOR", "VOR")
        case "N": ("NDB", "NDB")
        case "Q": ("NDB/DME", "NDB")
        case "X": ("LDA", "LDA")
        case "U": ("SDF", "SDF")
        default: nil
        }
        guard let prefix = navPrefix else {
            return (code, "APP \(code)", nil)
        }

        let rest = Array(code.dropFirst())
        if rest.count >= 2, rest[0].isNumber, rest[1].isNumber {
            var rwy = String(rest[0...1])
            var cursor = 2
            if cursor < rest.count, rest[cursor] == "L" || rest[cursor] == "R" || rest[cursor] == "C" {
                rwy.append(rest[cursor])
                cursor += 1
            }
            let suffix = cursor < rest.count ? String(rest[cursor...]) : ""
            if suffix.isEmpty {
                return ("\(prefix.long) RWY \(rwy)", "\(prefix.short) \(rwy)", rwy)
            }
            return ("\(prefix.long) \(suffix) RWY \(rwy)", "\(prefix.short) \(suffix) \(rwy)", rwy)
        }

        let circling = String(rest)
        if !circling.isEmpty {
            return ("\(prefix.long)-\(circling)", "\(prefix.short)-\(circling)", nil)
        }
        return (code, "APP \(code)", nil)
    }
}

/// Enroute airway summary from `Table 1` & `Table 2` of `cifp_procedures.bin`.
public struct CifpAirwaySummary: Equatable, Sendable {
    public let routeIdent: String
    public let fixCount: Int
    fileprivate let firstFixIndex: Int
}

/// Zero-copy memory-mapped `FBCIFP01` (`cifp_procedures.bin`) airway & terminal procedure reader.
public final class CifpProcedureDatabase: Sendable {
    public static let shared = CifpProcedureDatabase()

    public let loadedFilePath: String?
    public let airacCode: String
    public let airwayCount: Int
    public let procedureCount: Int

    private let mappedData: Data?
    private let airwaysOffset: Int
    private let airwayFixesOffset: Int
    private let proceduresOffset: Int
    private let transitionsOffset: Int
    private let legsOffset: Int
    private let airwayFixCount: Int
    private let transitionCount: Int
    private let legCount: Int

    public init(fileUrl: URL? = nil) {
        let resolvedUrl = fileUrl ?? Self.locateDefaultUrl()
        if let url = resolvedUrl,
           let data = try? Data(contentsOf: url, options: .mappedIfSafe),
           data.count >= 48,
           String(data: data.subdata(in: 0..<8), encoding: .ascii) == "FBCIFP01",
           Self.readUInt32LE(data, at: 8) == 1 {
            let airac = String(data: data.subdata(in: 12..<16), encoding: .ascii) ?? ""
            let awyCnt = Int(Self.readUInt32LE(data, at: 16))
            let awyFixCnt = Int(Self.readUInt32LE(data, at: 20))
            let procCnt = Int(Self.readUInt32LE(data, at: 24))
            let trCnt = Int(Self.readUInt32LE(data, at: 28))
            let lgCnt = Int(Self.readUInt32LE(data, at: 32))

            let offAirways = 48
            let offAirwayFixes = offAirways + awyCnt * 16
            let offProcedures = offAirwayFixes + awyFixCnt * 24
            let offTransitions = offProcedures + procCnt * 24
            let offLegs = offTransitions + trCnt * 16
            let totalBytes = offLegs + lgCnt * 32

            if awyCnt >= 0, procCnt >= 0, totalBytes <= data.count {
                self.loadedFilePath = url.path
                self.airacCode = airac
                self.airwayCount = awyCnt
                self.airwayFixCount = awyFixCnt
                self.procedureCount = procCnt
                self.transitionCount = trCnt
                self.legCount = lgCnt
                self.mappedData = data
                self.airwaysOffset = offAirways
                self.airwayFixesOffset = offAirwayFixes
                self.proceduresOffset = offProcedures
                self.transitionsOffset = offTransitions
                self.legsOffset = offLegs
                return
            }
        }

        self.loadedFilePath = nil
        self.airacCode = ""
        self.airwayCount = 0
        self.airwayFixCount = 0
        self.procedureCount = 0
        self.transitionCount = 0
        self.legCount = 0
        self.mappedData = nil
        self.airwaysOffset = 48
        self.airwayFixesOffset = 48
        self.proceduresOffset = 48
        self.transitionsOffset = 48
        self.legsOffset = 48
    }

    public var isLoaded: Bool {
        mappedData != nil && (airwayCount > 0 || procedureCount > 0)
    }

    /// Returns the shared mapped instance when `packPath` matches, or maps the active cycle file at `packPath`.
    public static func reader(forPackPath packPath: String?) -> CifpProcedureDatabase {
        guard let path = packPath, !path.isEmpty else {
            return shared
        }
        if shared.loadedFilePath == path {
            return shared
        }
        return CifpProcedureDatabase(fileUrl: URL(fileURLWithPath: path))
    }

    private static func locateDefaultUrl() -> URL? {
        if let bundled = Bundle.main.url(forResource: "cifp_procedures", withExtension: "bin") {
            return bundled
        }
        let staged = URL(
            fileURLWithPath: "/Users/fangluo/FSgame-wt-vision-svs/target/ipad-bundle-resources/cifp_procedures.bin"
        )
        if FileManager.default.fileExists(atPath: staged.path) {
            return staged
        }
        let flightbagDist = URL(
            fileURLWithPath: "/Users/fangluo/Flightbag/dist/cycles/2026-10-29/cifp_procedures.bin"
        )
        if FileManager.default.fileExists(atPath: flightbagDist.path) {
            return flightbagDist
        }
        return nil
    }

    // MARK: - Enroute Airway Lookup & Expansion (`SBJ V3 MXE` / `SBJ.V3.MXE`)

    /// Binary-searches `Table 1` (`airways`) for `rawRouteIdent` (`"V3"`, `"J60"`, `"Q75"`, `"T218"`).
    public func lookupAirway(_ rawRouteIdent: String) -> CifpAirwaySummary? {
        guard let data = mappedData, airwayCount > 0 else { return nil }
        let ident = rawRouteIdent.trimmingCharacters(in: .whitespacesAndNewlines).uppercased()
        guard !ident.isEmpty, ident.utf8.count <= 8 else { return nil }
        let targetKey = Self.padIdent8(ident)

        var lo = 0
        var hi = airwayCount
        while lo < hi {
            let mid = lo + (hi - lo) / 2
            let rowOff = airwaysOffset + mid * 16
            let cmp = Self.compareIdent8(data, at: rowOff, against: targetKey)
            if cmp == 0 {
                let fixCnt = Int(Self.readUInt16LE(data, at: rowOff + 10))
                let firstIdx = Int(Self.readUInt32LE(data, at: rowOff + 12))
                return CifpAirwaySummary(
                    routeIdent: ident,
                    fixCount: fixCnt,
                    firstFixIndex: firstIdx
                )
            } else if cmp < 0 {
                lo = mid + 1
            } else {
                hi = mid
            }
        }
        return nil
    }

    /// Expands an airway segment from `entryIdent` to `exitIdent` (forward or reverse), enforcing ARINC 424
    /// airway segment continuity (`segment_break_after`).
    public func expandAirway(
        routeIdent: String,
        from entryIdent: String,
        to exitIdent: String,
        includeEntry: Bool = false
    ) -> [FlightPlanWaypoint]? {
        guard let data = mappedData,
              let airway = lookupAirway(routeIdent) else {
            return nil
        }
        let fromNorm = entryIdent.trimmingCharacters(in: .whitespacesAndNewlines).uppercased()
        let toNorm = exitIdent.trimmingCharacters(in: .whitespacesAndNewlines).uppercased()
        guard !fromNorm.isEmpty, !toNorm.isEmpty,
              airway.firstFixIndex + airway.fixCount <= airwayFixCount else {
            return nil
        }

        struct RawAwyFix {
            let ident: String
            let region: String
            let kind: WaypointKind
            let breakAfter: Bool
            let latDeg: Double
            let lonDeg: Double
            let minAltFt: Int
        }

        var fixes: [RawAwyFix] = []
        fixes.reserveCapacity(airway.fixCount)
        for idx in 0..<airway.fixCount {
            let rowOff = airwayFixesOffset + (airway.firstFixIndex + idx) * 24
            let ident = Self.readIdent8(data, at: rowOff)
            let reg = Self.readAsciiTrimmed(data, at: rowOff + 8, length: 2)
            let kindByte = data[rowOff + 10]
            let brk = data[rowOff + 11] != 0
            let latDeg = Double(Self.readInt32LE(data, at: rowOff + 12)) / 1_000_000.0
            let lonDeg = Double(Self.readInt32LE(data, at: rowOff + 16)) / 1_000_000.0
            let minAlt = Int(Self.readUInt16LE(data, at: rowOff + 20)) * 100
            fixes.append(
                RawAwyFix(
                    ident: ident,
                    region: reg,
                    kind: Self.mapCifpWaypointKind(kindByte),
                    breakAfter: brk,
                    latDeg: latDeg,
                    lonDeg: lonDeg,
                    minAltFt: minAlt
                )
            )
        }

        let startIndices = fixes.indices.filter { fixes[$0].ident == fromNorm }
        let endIndices = fixes.indices.filter { fixes[$0].ident == toNorm }
        for sIdx in startIndices {
            for eIdx in endIndices {
                let lo = min(sIdx, eIdx)
                let hi = max(sIdx, eIdx)
                if lo < hi && (lo..<hi).contains(where: { fixes[$0].breakAfter }) {
                    continue
                }
                let orderedIndices: [Int] = sIdx <= eIdx
                    ? Array(sIdx...eIdx)
                    : Array((eIdx...sIdx).reversed())
                let sliceIndices = includeEntry ? orderedIndices : Array(orderedIndices.dropFirst())
                return sliceIndices.map { i in
                    let fx = fixes[i]
                    let meaLabel = fx.minAltFt > 0 ? "MEA \(fx.minAltFt) FT" : ""
                    return FlightPlanWaypoint(
                        pointKey: "cifp-awy:\(airway.routeIdent):\(fx.ident)",
                        ident: fx.ident,
                        kind: fx.kind,
                        name: "Airway \(airway.routeIdent)",
                        region: fx.region,
                        latitudeDeg: fx.latDeg,
                        longitudeDeg: fx.lonDeg,
                        airwayOrProcedureTag: airway.routeIdent,
                        altitudeConstraintTag: meaLabel
                    )
                }
            }
        }
        return nil
    }

    // MARK: - Airport Terminal Procedures (`SID` / `STAR` / `IAP`)

    /// Binary-searches `Table 3` (`procedures`) for all procedures at `rawAirportCode` (also checking `"K" + code` for 3-letter US IDs).
    public func procedures(
        forAirport rawAirportCode: String,
        kind filterKind: CifpProcedureKind? = nil
    ) -> [CifpProcedureSummary] {
        let trimmed = rawAirportCode.trimmingCharacters(in: .whitespacesAndNewlines).uppercased()
        guard !trimmed.isEmpty else { return [] }
        let direct = proceduresExactAirport(trimmed, kind: filterKind)
        if !direct.isEmpty {
            return direct
        }
        if trimmed.count == 3 {
            return proceduresExactAirport("K" + trimmed, kind: filterKind)
        }
        if trimmed.count == 4, trimmed.hasPrefix("K") {
            return proceduresExactAirport(String(trimmed.dropFirst()), kind: filterKind)
        }
        return []
    }

    private func proceduresExactAirport(
        _ airportIdent: String,
        kind filterKind: CifpProcedureKind?
    ) -> [CifpProcedureSummary] {
        guard let data = mappedData, procedureCount > 0, airportIdent.utf8.count <= 8 else {
            return []
        }
        let targetApt = Self.padIdent8(airportIdent)

        // Lower-bound binary search on `airport_ident[8]` at `rowOff + 0`.
        var lo = 0
        var hi = procedureCount
        while lo < hi {
            let mid = lo + (hi - lo) / 2
            let rowOff = proceduresOffset + mid * 24
            if Self.compareIdent8(data, at: rowOff, against: targetApt) < 0 {
                lo = mid + 1
            } else {
                hi = mid
            }
        }

        var results: [CifpProcedureSummary] = []
        var idx = lo
        while idx < procedureCount {
            let rowOff = proceduresOffset + idx * 24
            if Self.compareIdent8(data, at: rowOff, against: targetApt) != 0 {
                break
            }
            let procIdent = Self.readIdent8(data, at: rowOff + 8)
            let kindRaw = data[rowOff + 16]
            let routeType = data[rowOff + 17]
            let trCnt = Int(Self.readUInt16LE(data, at: rowOff + 18))
            let firstTr = Int(Self.readUInt32LE(data, at: rowOff + 20))

            if let kind = CifpProcedureKind(rawValue: kindRaw),
               filterKind == nil || filterKind == kind,
               firstTr + trCnt <= transitionCount {
                var transitions: [CifpTransitionSummary] = []
                transitions.reserveCapacity(trCnt)
                for t in 0..<trCnt {
                    let trOff = transitionsOffset + (firstTr + t) * 16
                    let trIdent = Self.readIdent8(data, at: trOff)
                    let phase = data[trOff + 8]
                    let trRouteType = data[trOff + 9]
                    let lgCnt = Int(Self.readUInt16LE(data, at: trOff + 10))
                    let firstLg = Int(Self.readUInt32LE(data, at: trOff + 12))
                    transitions.append(
                        CifpTransitionSummary(
                            ident: trIdent,
                            phase: phase,
                            routeType: trRouteType,
                            legCount: lgCnt,
                            firstLegIndex: firstLg
                        )
                    )
                }
                results.append(
                    CifpProcedureSummary(
                        airportIdent: airportIdent,
                        procedureIdent: procIdent,
                        kind: kind,
                        routeType: routeType,
                        transitions: transitions
                    )
                )
            }
            idx += 1
        }
        return results
    }

    /// Reads all decoded legs of a single `CifpTransitionSummary`.
    public func legs(for transition: CifpTransitionSummary) -> [CifpLegSummary] {
        guard let data = mappedData,
              transition.firstLegIndex >= 0,
              transition.firstLegIndex + transition.legCount <= legCount else {
            return []
        }
        var out: [CifpLegSummary] = []
        out.reserveCapacity(transition.legCount)
        for i in 0..<transition.legCount {
            let off = legsOffset + (transition.firstLegIndex + i) * 32
            let fixIdent = Self.readIdent8(data, at: off)
            let pathTerm = Self.readAsciiTrimmed(data, at: off + 8, length: 2)
            let kindByte = data[off + 10]
            let altCode = data[off + 11]
            let flags = data[off + 12]
            let speedKt = Int(Self.readUInt16LE(data, at: off + 14))
            let latDeg = Double(Self.readInt32LE(data, at: off + 16)) / 1_000_000.0
            let lonDeg = Double(Self.readInt32LE(data, at: off + 20)) / 1_000_000.0
            let alt1 = Int(Self.readUInt32LE(data, at: off + 24))
            let alt2 = Int(Self.readUInt32LE(data, at: off + 28))

            let altConstraint: CifpAltitudeConstraint = switch altCode {
            case 1: .at(feet: alt1)
            case 2: .atOrAbove(feet: alt1)
            case 3: .atOrBelow(feet: alt1)
            case 4: .between(upperFeet: alt1, lowerFeet: alt2)
            default: .none
            }

            out.append(
                CifpLegSummary(
                    fixIdent: fixIdent,
                    pathTerminator: pathTerm,
                    waypointKind: Self.mapCifpWaypointKind(kindByte),
                    hasCoordinates: (flags & (1 << 0)) != 0,
                    latitudeDeg: latDeg,
                    longitudeDeg: lonDeg,
                    altitudeConstraint: altConstraint,
                    speedLimitKt: speedKt,
                    isMissedApproach: (flags & (1 << 1)) != 0,
                    isIaf: (flags & (1 << 2)) != 0,
                    isFaf: (flags & (1 << 3)) != 0,
                    isMapt: (flags & (1 << 4)) != 0
                )
            )
        }
        return out
    }

    /// Assembles ordered ground-track `FlightPlanWaypoint`s for `procedure` across optional runway & enroute/feeder
    /// transitions, deduplicating consecutive seam fixes while preserving altitude/role tags.
    public func assembleWaypoints(
        for procedure: CifpProcedureSummary,
        runwayTransitionIdent: String? = nil,
        enrouteOrFeederTransitionIdent: String? = nil,
        includeMissedApproach: Bool = false
    ) -> [FlightPlanWaypoint] {
        let rwyTr = runwayTransitionIdent.flatMap { want in
            procedure.runwayTransitions.first { $0.ident == want }
        }
        let coreTr = procedure.commonCoreTransition
        let enrTr = enrouteOrFeederTransitionIdent.flatMap { want in
            procedure.enrouteOrFeederTransitions.first { $0.ident == want }
        }

        let orderedTransitions: [CifpTransitionSummary] = switch procedure.kind {
        case .sid:
            [rwyTr, coreTr, enrTr].compactMap { $0 }
        case .star, .approach:
            [enrTr, coreTr, rwyTr].compactMap { $0 }
        }

        var assembled: [FlightPlanWaypoint] = []
        let badge = procedure.shortBadge

        for tr in orderedTransitions {
            for leg in legs(for: tr) {
                if leg.isMissedApproach && !includeMissedApproach {
                    continue
                }
                guard leg.hasCoordinates, !leg.fixIdent.isEmpty else {
                    continue
                }
                let legConstraint = leg.combinedConstraintLabel
                if let lastIdx = assembled.indices.last,
                   assembled[lastIdx].ident == leg.fixIdent {
                    // Merge constraint/role metadata onto the deduplicated seam fix.
                    if !legConstraint.isEmpty {
                        if assembled[lastIdx].altitudeConstraintTag.isEmpty {
                            assembled[lastIdx].altitudeConstraintTag = legConstraint
                        } else if !assembled[lastIdx].altitudeConstraintTag.contains(legConstraint) {
                            assembled[lastIdx].altitudeConstraintTag += " · \(legConstraint)"
                        }
                    }
                    continue
                }

                let legDesc = legConstraint.isEmpty
                    ? "\(procedure.humanTitle) (\(leg.pathTerminator))"
                    : "\(procedure.humanTitle) · \(legConstraint)"
                assembled.append(
                    FlightPlanWaypoint(
                        pointKey: "cifp:\(procedure.airportIdent):\(procedure.procedureIdent):\(leg.fixIdent)",
                        ident: leg.fixIdent,
                        kind: leg.waypointKind,
                        name: legDesc,
                        region: procedure.airportIdent,
                        latitudeDeg: leg.latitudeDeg,
                        longitudeDeg: leg.longitudeDeg,
                        airwayOrProcedureTag: badge,
                        altitudeConstraintTag: legConstraint
                    )
                )
            }
        }
        return assembled
    }

    // MARK: - Little-Endian Binary Helpers

    private static func mapCifpWaypointKind(_ code: UInt8) -> WaypointKind {
        switch code {
        case 0: .airport
        case 1: .vortac
        case 2: .ndb
        default: .fix
        }
    }

    private static func padIdent8(_ ident: String) -> [UInt8] {
        var out = [UInt8](repeating: 0, count: 8)
        for (idx, b) in ident.utf8.prefix(8).enumerated() {
            out[idx] = b
        }
        return out
    }

    private static func compareIdent8(_ data: Data, at offset: Int, against target: [UInt8]) -> Int {
        for i in 0..<8 {
            let a = data[offset + i]
            let b = target[i]
            if a != b {
                return a < b ? -1 : 1
            }
        }
        return 0
    }

    private static func readIdent8(_ data: Data, at offset: Int) -> String {
        var bytes: [UInt8] = []
        bytes.reserveCapacity(8)
        for i in 0..<8 {
            let b = data[offset + i]
            if b == 0 || b == 32 { break }
            bytes.append(b)
        }
        return String(bytes: bytes, encoding: .ascii) ?? ""
    }

    private static func readAsciiTrimmed(_ data: Data, at offset: Int, length: Int) -> String {
        var bytes: [UInt8] = []
        for i in 0..<length {
            let b = data[offset + i]
            if b != 0 && b != 32 {
                bytes.append(b)
            }
        }
        return String(bytes: bytes, encoding: .ascii) ?? ""
    }

    private static func readUInt16LE(_ data: Data, at offset: Int) -> UInt16 {
        UInt16(data[offset]) | (UInt16(data[offset + 1]) << 8)
    }

    private static func readUInt32LE(_ data: Data, at offset: Int) -> UInt32 {
        UInt32(data[offset])
            | (UInt32(data[offset + 1]) << 8)
            | (UInt32(data[offset + 2]) << 16)
            | (UInt32(data[offset + 3]) << 24)
    }

    private static func readInt32LE(_ data: Data, at offset: Int) -> Int32 {
        Int32(bitPattern: readUInt32LE(data, at: offset))
    }
}

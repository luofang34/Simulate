import Foundation

/// Pilot-facing terminal chart & procedure plate category (replacing internal `"CIFP"` / `"IAP"` jargon).
public enum ProcedurePlateKind: String, CaseIterable, Codable, Identifiable, Sendable {
    /// Georeferenced or full-page Instrument Approach plate (`ILS`, `RNAV (GPS)`, `LOC`, `VOR`).
    case approach = "Approach"
    /// Standard Terminal Arrival Route chart (`STAR`).
    case arrival = "Arrival"
    /// Standard Instrument Departure / Obstacle Departure chart (`SID` / `ODP`).
    case departure = "Departure"
    /// Surveyed Airport Diagram (`APD`).
    case airportDiagram = "Airport Diagram"

    public var id: String { rawValue }

    /// Short pilot-facing tab badge.
    public var badge: String {
        switch self {
        case .approach: "Approach"
        case .arrival: "Arrival"
        case .departure: "Departure"
        case .airportDiagram: "Airport Diagram"
        }
    }

    /// Maps FAA `d-TPP` XML `chart_code` (`APD`, `IAP`, `DP`, `STAR`) to ``ProcedurePlateKind``.
    public static func fromDtppCode(_ rawCode: String) -> ProcedurePlateKind? {
        switch rawCode.trimmingCharacters(in: .whitespacesAndNewlines).uppercased() {
        case "IAP": .approach
        case "STAR": .arrival
        case "DP", "ODP", "SID": .departure
        case "APD": .airportDiagram
        default: nil
        }
    }
}

/// Availability & subscription link state of a chart/procedure service provider.
public enum ProcedureChartProviderStatus: Equatable, Sendable {
    /// Bundled or synced offline pack is active and verified on device.
    case activeOffline(cycleBadge: String)
    /// Commercial or regional chart service slot reserved for OAuth2 / subscription linking.
    case subscriptionStub(statusBadge: String)

    /// Whether this provider currently serves live offline procedure legs and plates.
    public var isOperational: Bool {
        if case .activeOffline = self { return true }
        return false
    }
}

/// Pluggable terminal procedure & approach plate service descriptor.
///
/// Multiple chart providers may be enabled simultaneously (mirroring ForeFlight's multi-account
/// `FAA` + `Jeppesen` + regional `AIP` architecture), with deterministic priority ordering when
/// two providers cover the same aerodrome.
public struct ProcedureChartProviderDescriptor: Equatable, Identifiable, Sendable {
    public let id: String
    public let shortTitle: String
    public let fullName: String
    public let coverageSummary: String
    public let protocolFamily: String
    public let status: ProcedureChartProviderStatus

    public init(
        id: String,
        shortTitle: String,
        fullName: String,
        coverageSummary: String,
        protocolFamily: String,
        status: ProcedureChartProviderStatus
    ) {
        self.id = id
        self.shortTitle = shortTitle
        self.fullName = fullName
        self.coverageSummary = coverageSummary
        self.protocolFamily = protocolFamily
        self.status = status
    }
}

/// Registry managing multiple simultaneous procedure & terminal chart providers (`FAA AeroNav` active offline
/// + reserved subscription connectors for `Jeppesen`, `Lido mPilot`, `EUROCONTROL / Regional AIP`, and `Navigraph`).
public struct ChartProviderRegistry: Equatable, Sendable {
    public static let faaAeroNavId = "faa_aeronav"
    public static let jeppesenId = "jeppesen"
    public static let lidoId = "lido_mpilot"
    public static let eurocontrolId = "eurocontrol_aip"
    public static let navigraphId = "navigraph"

    /// Registered chart & procedure providers in priority-display order.
    public var providers: [ProcedureChartProviderDescriptor]
    /// Simultaneously enabled provider IDs (pilots may keep `FAA AeroNav` + `Jeppesen` + `EUROCONTROL` enabled together).
    public var enabledProviderIds: Set<String>
    /// Preferred primary chart source when multiple enabled providers publish plates at the same airport.
    public var preferredProviderId: String

    public init(airacBadge: String = "2611") {
        self.providers = Self.makeDefaultCatalog(airacBadge: airacBadge)
        self.enabledProviderIds = [Self.faaAeroNavId]
        self.preferredProviderId = Self.faaAeroNavId
    }

    /// Rebuilds the provider catalog with an updated active AIRAC cycle badge while preserving user selections.
    public mutating func updateActiveCycleBadge(_ badge: String) {
        providers = Self.makeDefaultCatalog(airacBadge: badge)
    }

    /// Toggles whether `providerId` is enabled in the multi-provider stack (`FAA AeroNav` stays available as baseline).
    public mutating func toggleProviderEnabled(_ providerId: String) {
        if enabledProviderIds.contains(providerId) {
            if providerId != Self.faaAeroNavId || enabledProviderIds.count > 1 {
                enabledProviderIds.remove(providerId)
            }
        } else {
            enabledProviderIds.insert(providerId)
        }
        if !enabledProviderIds.contains(preferredProviderId) {
            preferredProviderId = Self.faaAeroNavId
        }
    }

    /// Selects `providerId` as preferred primary source and ensures it is in `enabledProviderIds`.
    public mutating func selectPreferredProvider(_ providerId: String) {
        enabledProviderIds.insert(providerId)
        preferredProviderId = providerId
    }

    /// Human-readable summary of active/selected provider(s) for compact picker headers.
    public var activeSummaryLabel: String {
        guard let preferred = providers.first(where: { $0.id == preferredProviderId }) else {
            return "FAA AeroNav"
        }
        let extraCount = max(enabledProviderIds.count - 1, 0)
        if extraCount > 0 {
            return "\(preferred.shortTitle) +\(extraCount)"
        }
        return preferred.shortTitle
    }

    /// Note shown when a user prioritizes a subscription stub whose credentials are not yet linked.
    public var stubFallbackNotice: String? {
        guard let preferred = providers.first(where: { $0.id == preferredProviderId }) else {
            return nil
        }
        if case .subscriptionStub = preferred.status {
            return "\(preferred.fullName) account not linked (stub reserved) — serving verified FAA AeroNav offline procedures & plates."
        }
        return nil
    }

    private static func makeDefaultCatalog(airacBadge: String) -> [ProcedureChartProviderDescriptor] {
        [
            ProcedureChartProviderDescriptor(
                id: faaAeroNavId,
                shortTitle: "FAA AeroNav",
                fullName: "FAA AeroNav (US d-TPP & Procedures)",
                coverageSummary: "United States & Territories · Offline GeoPDF plates + published SID / STAR / Approach legs",
                protocolFamily: "Flightbag FBDTPP01 + FBCIFP01 (28-day AIRAC)",
                status: .activeOffline(cycleBadge: "AIRAC \(airacBadge)")
            ),
            ProcedureChartProviderDescriptor(
                id: jeppesenId,
                shortTitle: "Jeppesen",
                fullName: "Jeppesen FliteDeck / NavData",
                coverageSummary: "Worldwide commercial & business aviation terminal charts + tailored ARINC 424",
                protocolFamily: "Jeppesen JDM OAuth2 + Georeferenced ICharts Stub",
                status: .subscriptionStub(statusBadge: "LINK ACCOUNT · STUB")
            ),
            ProcedureChartProviderDescriptor(
                id: lidoId,
                shortTitle: "Lido mPilot",
                fullName: "Lufthansa Systems Lido mPilot",
                coverageSummary: "Worldwide airline terminal charts, EOSIDs & ARINC 815 georeferenced plates",
                protocolFamily: "Lido mPilot Airline Sync Stub",
                status: .subscriptionStub(statusBadge: "LINK ACCOUNT · STUB")
            ),
            ProcedureChartProviderDescriptor(
                id: eurocontrolId,
                shortTitle: "EUROCONTROL AIP",
                fullName: "EUROCONTROL EAD / Regional Civil eAIP",
                coverageSummary: "ECAC European & national civil AIP terminal charts (DFS, SIA, NATS, NAV CANADA)",
                protocolFamily: "AIXM 5.1 + Regional GeoPDF Pack Stub",
                status: .subscriptionStub(statusBadge: "REGIONAL PACK · STUB")
            ),
            ProcedureChartProviderDescriptor(
                id: navigraphId,
                shortTitle: "Navigraph",
                fullName: "Navigraph Simulation Charts",
                coverageSummary: "Worldwide simulation-only Jeppesen-derived terminal plates & procedures",
                protocolFamily: "Navigraph PKCE OAuth2 Simulation Stub",
                status: .subscriptionStub(statusBadge: "SIM ACCOUNT · STUB")
            ),
        ]
    }
}

/// Metadata describing an individual terminal procedure plate (official GeoPDF or procedural fallback).
public struct ProcedurePlateDescriptor: Equatable, Identifiable, Sendable {
    public let id: String
    public let providerId: String
    public let providerBadge: String
    public let airportIdent: String
    public let kind: ProcedurePlateKind
    public let title: String
    public let pdfName: String?
    public let cifpIdent: String
    public let isOfficialPdf: Bool
    public let supportsMapOverlay: Bool

    public init(
        id: String,
        providerId: String,
        providerBadge: String,
        airportIdent: String,
        kind: ProcedurePlateKind,
        title: String,
        pdfName: String?,
        cifpIdent: String,
        isOfficialPdf: Bool,
        supportsMapOverlay: Bool
    ) {
        self.id = id
        self.providerId = providerId
        self.providerBadge = providerBadge
        self.airportIdent = airportIdent
        self.kind = kind
        self.title = title
        self.pdfName = pdfName
        self.cifpIdent = cifpIdent
        self.isOfficialPdf = isOfficialPdf
        self.supportsMapOverlay = supportsMapOverlay
    }
}

/// Active georeferenced procedure plate draped onto the 2D/3D moving map via MapLibre `ImageSource`.
public struct PinnedMapPlate: Equatable, Sendable {
    public let descriptor: ProcedurePlateDescriptor
    public let pngFileUrl: URL
    /// Eight `f64` values `[nwLon, nwLat, neLon, neLat, seLon, seLat, swLon, swLat]` in `WGS-84` degrees.
    public let cornersLonLat: [Double]
    public let centerLatitudeDeg: Double
    public let centerLongitudeDeg: Double
    public let procedure: CifpProcedureSummary?
    public let runwayTransitionIdent: String?
    public let enrouteOrFeederTransitionIdent: String?

    public init(
        descriptor: ProcedurePlateDescriptor,
        pngFileUrl: URL,
        cornersLonLat: [Double],
        centerLatitudeDeg: Double,
        centerLongitudeDeg: Double,
        procedure: CifpProcedureSummary? = nil,
        runwayTransitionIdent: String? = nil,
        enrouteOrFeederTransitionIdent: String? = nil
    ) {
        self.descriptor = descriptor
        self.pngFileUrl = pngFileUrl
        self.cornersLonLat = cornersLonLat
        self.centerLatitudeDeg = centerLatitudeDeg
        self.centerLongitudeDeg = centerLongitudeDeg
        self.procedure = procedure
        self.runwayTransitionIdent = runwayTransitionIdent
        self.enrouteOrFeederTransitionIdent = enrouteOrFeederTransitionIdent
    }
}

/// Memory-mapped reader for `FBDTPP01` (`dtpp_plates.bin`) offline FAA terminal procedure plate packs.
public final class DtppPlateArchive: Sendable {
    public static let shared = DtppPlateArchive()

    private struct CatalogEntry: Decodable, Sendable {
        let airport_ident: String
        let faa_ident: String
        let icao_ident: String
        let chart_code: String
        let chart_name: String
        let pdf_name: String
        let cifp_ident: String
        let rel_offset: Int
        let length: Int
        let sha256_hex: String
    }

    public let loadedFilePath: String?
    public let isLoaded: Bool
    private let mappedData: Data?
    private let blobBaseOffset: Int
    private let entries: [CatalogEntry]

    public init(fileUrl: URL? = nil) {
        let resolvedUrl = fileUrl ?? Self.locateDefaultUrl()
        if let url = resolvedUrl,
           let data = try? Data(contentsOf: url, options: .mappedIfSafe),
           data.count > 16,
           String(data: data.subdata(in: 0..<8), encoding: .ascii) == "FBDTPP01" {
            let jsonLen = Int(Self.readUInt32LE(data, at: 8))
            let blobStart = 12 + jsonLen
            if jsonLen > 2,
               blobStart <= data.count,
               let decoded = try? JSONDecoder().decode([CatalogEntry].self, from: data.subdata(in: 12..<blobStart)) {
                self.loadedFilePath = url.path
                self.isLoaded = true
                self.mappedData = data
                self.blobBaseOffset = blobStart
                self.entries = decoded
                return
            }
        }
        self.loadedFilePath = nil
        self.isLoaded = false
        self.mappedData = nil
        self.blobBaseOffset = 0
        self.entries = []
    }

    /// Returns a reader bound to `packPath` (reusing ``shared`` when paths match).
    public static func reader(forPackPath packPath: String?) -> DtppPlateArchive {
        guard let path = packPath, !path.isEmpty else {
            return shared
        }
        if shared.loadedFilePath == path {
            return shared
        }
        return DtppPlateArchive(fileUrl: URL(fileURLWithPath: path))
    }

    /// Queries all bundled official FAA `d-TPP` plates at `airportCode` (`"N51"`, `"KEWR"`, `"EWR"`).
    public func plates(forAirport airportCode: String, kind: ProcedurePlateKind? = nil) -> [ProcedurePlateDescriptor] {
        guard isLoaded else { return [] }
        let query = airportCode.trimmingCharacters(in: .whitespacesAndNewlines).uppercased()
        guard !query.isEmpty else { return [] }
        let altStripped = (query.count == 4 && query.hasPrefix("K")) ? String(query.dropFirst()) : query
        let altPrefixed = (query.count == 3) ? "K\(query)" : query

        var results: [ProcedurePlateDescriptor] = []
        for entry in entries {
            let matchesAirport = entry.airport_ident == query
                || entry.icao_ident == query
                || entry.faa_ident == query
                || entry.faa_ident == altStripped
                || entry.icao_ident == altPrefixed
            guard matchesAirport,
                  let plateKind = ProcedurePlateKind.fromDtppCode(entry.chart_code) else {
                continue
            }
            if let filterKind = kind, plateKind != filterKind {
                continue
            }
            results.append(
                ProcedurePlateDescriptor(
                    id: "\(ChartProviderRegistry.faaAeroNavId):\(entry.airport_ident):\(entry.pdf_name)",
                    providerId: ChartProviderRegistry.faaAeroNavId,
                    providerBadge: "FAA AeroNav",
                    airportIdent: entry.airport_ident,
                    kind: plateKind,
                    title: entry.chart_name,
                    pdfName: entry.pdf_name,
                    cifpIdent: entry.cifp_ident,
                    isOfficialPdf: true,
                    supportsMapOverlay: plateKind == .approach || plateKind == .airportDiagram
                )
            )
        }
        return results
    }

    /// Finds the official FAA `d-TPP` plate matching a published procedure (`"N51"`, `"R22"` -> `"05075R22.PDF"`).
    public func matchingPlate(for procedure: CifpProcedureSummary) -> ProcedurePlateDescriptor? {
        let targetKind: ProcedurePlateKind = switch procedure.kind {
        case .approach: .approach
        case .star: .arrival
        case .sid: .departure
        }
        let candidates = plates(forAirport: procedure.airportIdent, kind: targetKind)
        guard !candidates.isEmpty else { return nil }

        let code = procedure.procedureIdent.uppercased()
        if let exact = candidates.first(where: { $0.cifpIdent == code }) {
            return exact
        }
        // Match prefix/stem for numbered SIDs/STARs (e.g. `EWR5` vs `NEWARK FIVE`) or runway suffix.
        let humanUpper = procedure.humanTitle.uppercased()
        if let titleMatch = candidates.first(where: { $0.title.uppercased() == humanUpper }) {
            return titleMatch
        }
        if let rwy = procedure.associatedRunwayDesignator {
            let rwyToken = "RWY \(rwy)"
            let navPrefix = String(humanUpper.split(separator: " ").first ?? "")
            if let rwyMatch = candidates.first(where: {
                $0.title.uppercased().contains(rwyToken) && $0.title.uppercased().contains(navPrefix)
            }) {
                return rwyMatch
            }
        }
        return nil
    }

    /// Slices raw PDF bytes for `pdfName` (`"05075R22.PDF"`, `"00285AD.PDF"`) directly from the mapped archive.
    public func pdfData(named pdfName: String) -> Data? {
        guard isLoaded, let data = mappedData else { return nil }
        let target = pdfName.trimmingCharacters(in: .whitespacesAndNewlines).uppercased()
        guard let entry = entries.first(where: { $0.pdf_name.uppercased() == target }) else {
            return nil
        }
        let start = blobBaseOffset + entry.rel_offset
        let end = start + entry.length
        guard entry.rel_offset >= 0, entry.length > 0, end <= data.count else {
            return nil
        }
        return data.subdata(in: start..<end)
    }

    private static func readUInt32LE(_ data: Data, at offset: Int) -> UInt32 {
        var val: UInt32 = 0
        for idx in 0..<4 {
            val |= UInt32(data[offset + idx]) << (idx * 8)
        }
        return val
    }

    private static func locateDefaultUrl() -> URL? {
        if let bundled = Bundle.main.url(forResource: "dtpp_plates", withExtension: "bin") {
            return bundled
        }
        let devPath = "/Users/fangluo/FSgame-wt-vision-svs/target/ipad-bundle-resources/dtpp_plates.bin"
        if FileManager.default.fileExists(atPath: devPath) {
            return URL(fileURLWithPath: devPath)
        }
        return nil
    }
}

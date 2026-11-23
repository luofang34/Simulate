import CryptoKit
import Foundation
import Observation

/// Pure ICAO AIRAC (28-day, Annex 15 §6.2 / FAA AIM 1-1-17) and FAA Aeronautical Chart (56-day)
/// UTC cycle grid anchored at `AIRAC 2601` (`2026-01-22T09:01:00Z`), matching `v99n62`'s `aerocontext-navdata`.
public enum AiracCycleGrid {
    /// `2026-01-22T09:01:00Z` — exact UTC activation instant of `AIRAC 2601`.
    public static let anchor2601UnixSeconds: TimeInterval = 1_769_072_460.0
    /// Seconds in one 28-day ICAO AIRAC cycle (`2,419,200 s`).
    public static let airac28DaySeconds: TimeInterval = 28.0 * 86_400.0
    /// Seconds in one 56-day FAA Visual / Enroute chart edition (`4,838,400 s`).
    public static let chart56DaySeconds: TimeInterval = 56.0 * 86_400.0

    /// Parses an RFC-3339 / ISO-8601 UTC timestamp (`"2026-10-29T09:01:00Z"` or `"2026-12-03T12:00:00Z"`).
    public static func parseIso8601Utc(_ text: String) -> Date? {
        let trimmed = text.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return nil }
        let formatter = ISO8601DateFormatter()
        formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
        if let exact = formatter.date(from: trimmed) {
            return exact
        }
        formatter.formatOptions = [.withInternetDateTime]
        if let standard = formatter.date(from: trimmed) {
            return standard
        }
        // Accept bare `YYYY-MM-DD` as `09:01:00Z` on that date.
        if trimmed.count == 10 {
            return formatter.date(from: "\(trimmed)T09:01:00Z")
        }
        return nil
    }

    /// Formats `date` as compact UTC (`"2026-11-26 09:01Z"`).
    public static func formatUtcShort(_ date: Date) -> String {
        let formatter = DateFormatter()
        formatter.calendar = Calendar(identifier: .gregorian)
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.timeZone = TimeZone(secondsFromGMT: 0)
        formatter.dateFormat = "yyyy-MM-dd HH:mm'Z'"
        return formatter.string(from: date)
    }

    /// Derives the 4-digit AIRAC cycle code (`"2611"`) containing `utcDate`.
    public static func airacCode(at utcDate: Date) -> String {
        let delta = utcDate.timeIntervalSince1970 - anchor2601UnixSeconds
        let cycleSteps = Int(floor(delta / airac28DaySeconds))
        let cycleStart = Date(timeIntervalSince1970: anchor2601UnixSeconds + Double(cycleSteps) * airac28DaySeconds)

        var cal = Calendar(identifier: .gregorian)
        cal.timeZone = TimeZone(secondsFromGMT: 0) ?? .gmt
        let targetYear = cal.component(.year, from: cycleStart)

        var ordinal = 1
        var cursor = cycleStart
        while true {
            let prev = cursor.addingTimeInterval(-airac28DaySeconds)
            if cal.component(.year, from: prev) != targetYear {
                break
            }
            ordinal += 1
            cursor = prev
        }
        let yy = ((targetYear % 100) + 100) % 100
        return String(format: "%02d%02d", yy, ordinal)
    }
}

/// Authority and product family managed in the offline aeronautical data store (`v99n62` + `Flightbag`).
public enum AeroDataAuthority: String, CaseIterable, Codable, Identifiable, Sendable {
    /// FAA 28-Day NASR 3D Airspace (`airspace.pmtiles`).
    case faaNasrAirspace = "faa-nasr-airspace"
    /// FAA 28-Day NASR Waypoints, VOR/VORTAC/NDB Navaids & Airports (`nasr_waypoints.bin`).
    case faaNasrWaypoints = "faa-nasr-waypoints"
    /// FAA 28-Day Instrument Procedures & Airways (`cifp_procedures.bin`).
    case faaCifp = "faa-cifp"
    /// FAA 28-Day Terminal Procedure Plates & Airport Diagrams (`dtpp_plates.bin`).
    case faaDtpp = "faa-dtpp"
    /// FAA 56-Day VFR Sectional Raster Charts (`vfr_sectional.pmtiles`, `1:500,000`).
    case faaVfrSectional = "faa-vfr-sectional"
    /// FAA 56-Day VFR Terminal Area Raster Charts (`vfr_terminal.pmtiles`, `1:250,000`).
    case faaVfrTerminal = "faa-vfr-terminal"
    /// FAA 56-Day IFR Enroute Low Altitude Charts (`ifr_low.pmtiles`).
    case faaIfrLow = "faa-ifr-low"
    /// FAA 56-Day IFR Enroute High Altitude Charts (`ifr_high.pmtiles`).
    case faaIfrHigh = "faa-ifr-high"

    public var id: String { rawValue }

    public var title: String {
        switch self {
        case .faaNasrAirspace: "3D NASR Airspace"
        case .faaNasrWaypoints: "US Waypoints & Navaids"
        case .faaCifp: "US Procedures & Airways"
        case .faaDtpp: "US Terminal Procedure Plates"
        case .faaVfrSectional: "US VFR Sectional (1:500k)"
        case .faaVfrTerminal: "US VFR Terminal TAC (1:250k)"
        case .faaIfrLow: "US IFR Enroute Low"
        case .faaIfrHigh: "US IFR Enroute High"
        }
    }

    public var cadenceBadge: String {
        switch self {
        case .faaNasrAirspace, .faaNasrWaypoints, .faaCifp, .faaDtpp:
            "28d AIRAC"
        case .faaVfrSectional, .faaVfrTerminal, .faaIfrLow, .faaIfrHigh:
            "56d FAA"
        }
    }

    public var expectedFileName: String {
        switch self {
        case .faaNasrAirspace: "airspace.pmtiles"
        case .faaNasrWaypoints: "nasr_waypoints.bin"
        case .faaCifp: "cifp_procedures.bin"
        case .faaDtpp: "dtpp_plates.bin"
        case .faaVfrSectional: "vfr_sectional.pmtiles"
        case .faaVfrTerminal: "vfr_terminal.pmtiles"
        case .faaIfrLow: "ifr_low.pmtiles"
        case .faaIfrHigh: "ifr_high.pmtiles"
        }
    }

    /// Upstream authority & architectural compiler pipeline (`Flightbag` static cycle pack + `world-notices` live amendments).
    public var provenanceSubtitle: String {
        switch self {
        case .faaNasrAirspace:
            "Flightbag NASR compiler (airspace.pmtiles) + world-notices live TFR / SUA schedules"
        case .faaNasrWaypoints:
            "Flightbag 28-day NASR APT / NAV / FIX / AWY binary index (nasr_waypoints.bin)"
        case .faaCifp:
            "Flightbag ARINC 424-18 airway & SID / STAR / Approach legs (cifp_procedures.bin) + world-notices FDC amendments"
        case .faaDtpp:
            "Flightbag FAA d-TPP GeoPDF approach plates & airport diagrams (dtpp_plates.bin)"
        case .faaVfrSectional:
            "Flightbag FAA VFR Sectional GeoTIFF harvester (vfr_sectional.pmtiles, z0..12)"
        case .faaVfrTerminal:
            "Flightbag FAA VFR Terminal TAC GeoTIFF harvester (vfr_terminal.pmtiles, z10..12)"
        case .faaIfrLow:
            "Flightbag FAA IFR_AreaLow GeoTIFF harvester (ifr_low.pmtiles, z0..11)"
        case .faaIfrHigh:
            "Flightbag FAA IFR_High GeoTIFF harvester (ifr_high.pmtiles, z0..9)"
        }
    }

    /// Maps manifests using underscore or hyphen authority slugs (`"faa_vfr_sectional"` / `"faa-vfr-sectional"`).
    public static func fromManifestSlug(_ raw: String, fileName: String = "") -> AeroDataAuthority? {
        let normalized = raw.lowercased().replacingOccurrences(of: "_", with: "-")
        switch normalized {
        case "faa-nasr-airspace":
            return .faaNasrAirspace
        case "faa-nasr-waypoints":
            return .faaNasrWaypoints
        case "faa-nasr":
            return fileName.hasSuffix(".bin") ? .faaNasrWaypoints : .faaNasrAirspace
        case "faa-cifp", "faa-cifp-arinc424", "cifp":
            return .faaCifp
        case "faa-dtpp", "dtpp":
            return .faaDtpp
        case "faa-vfr-sectional":
            return .faaVfrSectional
        case "faa-vfr-terminal":
            return .faaVfrTerminal
        case "faa-ifr-low":
            return .faaIfrLow
        case "faa-ifr-high":
            return .faaIfrHigh
        default:
            return nil
        }
    }
}

/// Verified offline pack entry installed in the bundle seed or `Application Support/AeroDataStore`.
public struct InstalledAeroPack: Codable, Equatable, Identifiable, Sendable {
    public var authority: AeroDataAuthority
    public var airacCode: String
    public var effectiveDate: String
    public var expiresDate: String
    public var effectiveFromUtc: Date
    public var effectiveUntilUtc: Date
    public var sha256Hex: String
    public var sizeBytes: Int64
    public var minZoom: Int
    public var maxZoom: Int
    public var bounds: [Double]
    public var filePath: String
    public var isBundledSeed: Bool

    public var id: String {
        "\(authority.rawValue):\(effectiveDate):\(airacCode)"
    }

    public init(
        authority: AeroDataAuthority,
        airacCode: String,
        effectiveDate: String,
        expiresDate: String,
        effectiveFromUtc: Date,
        effectiveUntilUtc: Date,
        sha256Hex: String,
        sizeBytes: Int64,
        minZoom: Int,
        maxZoom: Int,
        bounds: [Double],
        filePath: String,
        isBundledSeed: Bool
    ) {
        self.authority = authority
        self.airacCode = airacCode
        self.effectiveDate = effectiveDate
        self.expiresDate = expiresDate
        self.effectiveFromUtc = effectiveFromUtc
        self.effectiveUntilUtc = effectiveUntilUtc
        self.sha256Hex = sha256Hex
        self.sizeBytes = sizeBytes
        self.minZoom = minZoom
        self.maxZoom = maxZoom
        self.bounds = bounds
        self.filePath = filePath
        self.isBundledSeed = isBundledSeed
    }

    /// Half-open ICAO AIRAC / FAA validity interval `[effectiveFromUtc, effectiveUntilUtc)`.
    public func isEffective(at referenceUtc: Date) -> Bool {
        effectiveFromUtc <= referenceUtc && referenceUtc < effectiveUntilUtc
    }

    /// Formatted file size (`"490.1 MB"`, `"27.0 MB"`, `"2.0 MB"`).
    public var formattedSize: String {
        let mb = Double(max(sizeBytes, 0)) / (1024.0 * 1024.0)
        if mb >= 10.0 {
            return String(format: "%.1f MB", mb)
        }
        return String(format: "%.2f MB", mb)
    }

    /// Short SHA-256 provenance fingerprint (`"75e375ab…"`).
    public var shortSha256: String {
        guard sha256Hex.count >= 8 else { return sha256Hex }
        return String(sha256Hex.prefix(8))
    }
}

/// Deterministic cycle currency & offline availability classification (`v99n62` `Freshness` + strict invalidation).
public enum AeroPackValidity: Equatable, Sendable {
    /// Installed offline pack is within `[effectiveFromUtc, effectiveUntilUtc)` and verified on disk.
    case valid(pack: InstalledAeroPack, remainingSeconds: TimeInterval, hasPrefetchNext: Bool)
    /// Installed pack is past `effectiveUntilUtc` and no successor cycle has been downloaded — **STRICTLY INVALID**.
    case expiredInvalid(lastPack: InstalledAeroPack, expiredSecondsAgo: TimeInterval)
    /// Only a future prefetch cycle is present (`referenceUtc < effectiveFromUtc`) — not yet effective.
    case notYetEffective(nextPack: InstalledAeroPack, startsInSeconds: TimeInterval)
    /// Pack has not been downloaded to offline storage — cannot be rendered (online tile fallback is forbidden).
    case notDownloaded

    /// Returns the `InstalledAeroPack` **only** when `.valid`; returns `nil` when expired, future, or missing.
    public var activePack: InstalledAeroPack? {
        if case .valid(let pack, _, _) = self {
            return pack
        }
        return nil
    }

    public var isValidForUse: Bool {
        activePack != nil
    }

    public var isExpiredInvalid: Bool {
        if case .expiredInvalid = self { return true }
        return false
    }

    public var badgeText: String {
        switch self {
        case .valid(let pack, let remSec, let prefetch):
            let days = max(Int(floor(remSec / 86_400.0)), 0)
            let suffix = prefetch ? " +NEXT" : ""
            return "VALID · \(pack.airacCode) (\(days)d)\(suffix)"
        case .expiredInvalid(let lastPack, _):
            return "EXPIRED · \(lastPack.airacCode) INVALID"
        case .notYetEffective(let nextPack, _):
            return "PENDING · \(nextPack.airacCode)"
        case .notDownloaded:
            return "NOT DOWNLOADED"
        }
    }

    public var shortTag: String {
        switch self {
        case .valid(let pack, _, _):
            "CYCLE \(pack.airacCode)"
        case .expiredInvalid:
            "EXPIRED · INVALID"
        case .notYetEffective:
            "NOT EFFECTIVE"
        case .notDownloaded:
            "OFFLINE REQ"
        }
    }

    public var detailNote: String {
        switch self {
        case .valid(let pack, _, _):
            "Offline verified (\(pack.formattedSize), sha256 \(pack.shortSha256)…) · Effective \(pack.effectiveDate) → \(pack.expiresDate) 09:01Z"
        case .expiredInvalid(let lastPack, _):
            "Expired \(AiracCycleGrid.formatUtcShort(lastPack.effectiveUntilUtc)) with no updated cycle downloaded — suppressed as INVALID."
        case .notYetEffective(let nextPack, _):
            "Prefetched cycle \(nextPack.airacCode) activates \(AiracCycleGrid.formatUtcShort(nextPack.effectiveFromUtc))."
        case .notDownloaded:
            "Offline pre-download required before use. Unverified live chart streaming is disabled."
        }
    }
}

/// Immutable snapshot of currently valid offline packs passed into `EarthStyleBuilder`.
/// Excludes volatile countdown seconds so `Equatable` only changes when pack validity or cycle paths change.
public struct ResolvedAeroPackSet: Equatable, Sendable {
    public let airspace: InstalledAeroPack?
    public let waypoints: InstalledAeroPack?
    public let cifpProcedures: InstalledAeroPack?
    public let dtppPlates: InstalledAeroPack?
    public let vfrSectional: InstalledAeroPack?
    public let vfrTerminal: InstalledAeroPack?
    public let ifrLow: InstalledAeroPack?
    public let ifrHigh: InstalledAeroPack?

    public init(
        airspace: InstalledAeroPack?,
        waypoints: InstalledAeroPack?,
        cifpProcedures: InstalledAeroPack? = nil,
        dtppPlates: InstalledAeroPack? = nil,
        vfrSectional: InstalledAeroPack?,
        vfrTerminal: InstalledAeroPack?,
        ifrLow: InstalledAeroPack?,
        ifrHigh: InstalledAeroPack?
    ) {
        self.airspace = airspace
        self.waypoints = waypoints
        self.cifpProcedures = cifpProcedures
        self.dtppPlates = dtppPlates
        self.vfrSectional = vfrSectional
        self.vfrTerminal = vfrTerminal
        self.ifrLow = ifrLow
        self.ifrHigh = ifrHigh
    }
}

/// Remote manifest sync progress state (`v99n62` `sync` workflow).
public enum AeroDataSyncState: Equatable, Sendable {
    case idle
    case checkingManifest
    case downloading(authority: AeroDataAuthority, fraction: Double)
    case synced(summary: String, checkedAtUtc: Date)
    case failed(message: String)
}

/// Remote manifest item (`v99n62` `ManifestEntry` + `Flightbag` `RollingEntry` dual-schema compatible).
public struct RemoteAeroManifestEntry: Codable, Equatable, Sendable {
    public let authority: String
    public let airac: String
    public let effectiveOn: String
    public let nextEffectiveOn: String
    public let effectiveFromUtc: String?
    public let effectiveUntilUtc: String?
    public let file: String?
    public let url: String
    public let sha256: String
    public let bytes: Int64
    public let minZoom: Int?
    public let maxZoom: Int?
    public let bounds: [Double]?
    public let cid: String?

    enum CodingKeys: String, CodingKey {
        case authority
        case airac
        case airacCode = "airac_code"
        case effectiveOn = "effective_on"
        case effectiveDate = "effective_date"
        case nextEffectiveOn = "next_effective_on"
        case expiresDate = "expires_date"
        case effectiveFromUtc = "effective_from_utc"
        case effectiveUntilUtc = "effective_until_utc"
        case file
        case url
        case sha256
        case sha256Hex = "sha256_hex"
        case bytes
        case sizeBytes = "size_bytes"
        case minZoom = "min_zoom"
        case maxZoom = "max_zoom"
        case bounds
        case cid
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        self.authority = try c.decode(String.self, forKey: .authority)
        if let v = try c.decodeIfPresent(String.self, forKey: .airac) {
            self.airac = v
        } else {
            self.airac = try c.decode(String.self, forKey: .airacCode)
        }
        if let v = try c.decodeIfPresent(String.self, forKey: .effectiveOn) {
            self.effectiveOn = v
        } else {
            self.effectiveOn = try c.decode(String.self, forKey: .effectiveDate)
        }
        if let v = try c.decodeIfPresent(String.self, forKey: .nextEffectiveOn) {
            self.nextEffectiveOn = v
        } else {
            self.nextEffectiveOn = try c.decode(String.self, forKey: .expiresDate)
        }
        self.effectiveFromUtc = try c.decodeIfPresent(String.self, forKey: .effectiveFromUtc)
        self.effectiveUntilUtc = try c.decodeIfPresent(String.self, forKey: .effectiveUntilUtc)
        self.file = try c.decodeIfPresent(String.self, forKey: .file)
        self.url = try c.decode(String.self, forKey: .url)
        if let v = try c.decodeIfPresent(String.self, forKey: .sha256) {
            self.sha256 = v
        } else {
            self.sha256 = try c.decode(String.self, forKey: .sha256Hex)
        }
        if let v = try c.decodeIfPresent(Int64.self, forKey: .bytes) {
            self.bytes = v
        } else {
            self.bytes = try c.decode(Int64.self, forKey: .sizeBytes)
        }
        self.minZoom = try c.decodeIfPresent(Int.self, forKey: .minZoom)
        self.maxZoom = try c.decodeIfPresent(Int.self, forKey: .maxZoom)
        self.bounds = try c.decodeIfPresent([Double].self, forKey: .bounds)
        self.cid = try c.decodeIfPresent(String.self, forKey: .cid)
    }

    public func encode(to encoder: Encoder) throws {
        var c = encoder.container(keyedBy: CodingKeys.self)
        try c.encode(authority, forKey: .authority)
        try c.encode(airac, forKey: .airac)
        try c.encode(effectiveOn, forKey: .effectiveOn)
        try c.encode(nextEffectiveOn, forKey: .nextEffectiveOn)
        try c.encodeIfPresent(effectiveFromUtc, forKey: .effectiveFromUtc)
        try c.encodeIfPresent(effectiveUntilUtc, forKey: .effectiveUntilUtc)
        try c.encodeIfPresent(file, forKey: .file)
        try c.encode(url, forKey: .url)
        try c.encode(sha256, forKey: .sha256)
        try c.encode(bytes, forKey: .bytes)
        try c.encodeIfPresent(minZoom, forKey: .minZoom)
        try c.encodeIfPresent(maxZoom, forKey: .maxZoom)
        try c.encodeIfPresent(bounds, forKey: .bounds)
        try c.encodeIfPresent(cid, forKey: .cid)
    }
}

/// Offline aeronautical chart, 3D airspace, and NASR navdata store with SHA-256 verification,
/// `current` + `prefetch` (`upcoming`) cycle slots, pruning, and strict expiration invalidation.
@MainActor
@Observable
public final class AeroDataPackStore {
    public static let defaultPrimaryManifestUrl = URL(
        string: "https://github.com/luofang34/Flightbag/releases/download/flightbag-manifest/manifest.json"
    )
    public static let defaultFallbackManifestUrl = URL(
        string: "https://luofang34.github.io/Flightbag/manifest.json"
    )

    /// Installed offline packs across bundled seeds and downloaded cycles.
    public private(set) var installedPacks: [InstalledAeroPack] = []
    /// Authorities explicitly evicted/disabled locally by the user for testing or storage management.
    public private(set) var disabledSeedAuthorities: Set<AeroDataAuthority> = []
    /// Optional deterministic UTC clock override (`nil` = live system UTC `Date()`).
    public var referenceUtcOverride: Date?
    /// Current manifest sync status.
    public private(set) var syncState: AeroDataSyncState = .idle
    /// Monotonic counter incremented whenever pack installation, eviction, or clock override changes.
    public private(set) var storeRevision: UInt64 = 1

    @ObservationIgnored private let storeRootDirectory: URL
    @ObservationIgnored private let indexFileUrl: URL

    public init(storeDirectory: URL? = nil, resourceDirectoryOverride: URL? = nil) {
        let root: URL
        if let storeDirectory {
            root = storeDirectory
        } else {
            let appSupport = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask).first
                ?? URL(fileURLWithPath: NSTemporaryDirectory())
            root = appSupport.appendingPathComponent("AeroDataStore", isDirectory: true)
        }
        self.storeRootDirectory = root
        self.indexFileUrl = root.appendingPathComponent("store_index.json")

        var loaded: [InstalledAeroPack] = Self.discoverBundledSeedPacks(resourceDirectoryOverride: resourceDirectoryOverride)
        let persisted = Self.loadPersistedPacks(from: indexFileUrl)
        for pack in persisted where FileManager.default.fileExists(atPath: pack.filePath) {
            if !loaded.contains(where: { $0.id == pack.id && $0.filePath == pack.filePath }) {
                loaded.append(pack)
            }
        }
        self.installedPacks = loaded
    }

    /// Effective UTC reference timestamp (`referenceUtcOverride ?? Date()`).
    public var effectiveReferenceUtc: Date {
        referenceUtcOverride ?? Date()
    }

    /// Sets or clears a deterministic UTC clock override for cycle-expiration verification.
    public func setReferenceUtcOverride(_ overrideUtc: Date?) {
        referenceUtcOverride = overrideUtc
        storeRevision &+= 1
    }

    /// Evaluates strict offline availability and AIRAC / FAA cycle validity for `authority` at `utc`.
    public func validity(for authority: AeroDataAuthority, at utc: Date? = nil) -> AeroPackValidity {
        let refUtc = utc ?? effectiveReferenceUtc
        let candidates = installedPacks.filter { pack in
            guard pack.authority == authority else { return false }
            if pack.isBundledSeed && disabledSeedAuthorities.contains(authority) {
                return false
            }
            return FileManager.default.fileExists(atPath: pack.filePath)
        }

        guard !candidates.isEmpty else {
            return .notDownloaded
        }

        let activeMatches = candidates
            .filter { $0.isEffective(at: refUtc) }
            .sorted { a, b in
                if a.effectiveFromUtc != b.effectiveFromUtc {
                    return a.effectiveFromUtc > b.effectiveFromUtc
                }
                if a.isBundledSeed != b.isBundledSeed {
                    return !a.isBundledSeed
                }
                return a.filePath < b.filePath
            }

        let futureMatches = candidates
            .filter { $0.effectiveFromUtc > refUtc }
            .sorted { $0.effectiveFromUtc < $1.effectiveFromUtc }

        if let current = activeMatches.first {
            let remaining = max(current.effectiveUntilUtc.timeIntervalSince(refUtc), 0.0)
            return .valid(
                pack: current,
                remainingSeconds: remaining,
                hasPrefetchNext: !futureMatches.isEmpty
            )
        }

        let expiredMatches = candidates
            .filter { refUtc >= $0.effectiveUntilUtc }
            .sorted { $0.effectiveUntilUtc > $1.effectiveUntilUtc }

        if let latestExpired = expiredMatches.first {
            let age = max(refUtc.timeIntervalSince(latestExpired.effectiveUntilUtc), 0.0)
            return .expiredInvalid(lastPack: latestExpired, expiredSecondsAgo: age)
        }

        if let earliestFuture = futureMatches.first {
            let untilStart = max(earliestFuture.effectiveFromUtc.timeIntervalSince(refUtc), 0.0)
            return .notYetEffective(nextPack: earliestFuture, startsInSeconds: untilStart)
        }

        return .notDownloaded
    }

    /// Resolves the active valid pack set at `utc` (`nil` for any expired, pending, or missing authority).
    public func resolvedPackSet(at utc: Date? = nil) -> ResolvedAeroPackSet {
        let refUtc = utc ?? effectiveReferenceUtc
        return ResolvedAeroPackSet(
            airspace: validity(for: .faaNasrAirspace, at: refUtc).activePack,
            waypoints: validity(for: .faaNasrWaypoints, at: refUtc).activePack,
            cifpProcedures: validity(for: .faaCifp, at: refUtc).activePack,
            dtppPlates: validity(for: .faaDtpp, at: refUtc).activePack,
            vfrSectional: validity(for: .faaVfrSectional, at: refUtc).activePack,
            vfrTerminal: validity(for: .faaVfrTerminal, at: refUtc).activePack,
            ifrLow: validity(for: .faaIfrLow, at: refUtc).activePack,
            ifrHigh: validity(for: .faaIfrHigh, at: refUtc).activePack
        )
    }

    /// Worst-case health summary across all installed offline authorities (`28d` NASR/CIFP + `56d` VFR/IFR charts).
    public func overallHealthSummary(at utc: Date? = nil) -> (label: String, hasExpired: Bool) {
        let refUtc = utc ?? effectiveReferenceUtc
        var expiredCodes: [String] = []
        var activeCode: String?

        for auth in AeroDataAuthority.allCases {
            switch validity(for: auth, at: refUtc) {
            case .valid(let pack, _, _):
                if activeCode == nil || auth == .faaNasrAirspace || auth == .faaCifp {
                    activeCode = pack.airacCode
                }
            case .expiredInvalid(let lastPack, _):
                if !expiredCodes.contains(lastPack.airacCode) {
                    expiredCodes.append(lastPack.airacCode)
                }
            case .notYetEffective, .notDownloaded:
                break
            }
        }

        if !expiredCodes.isEmpty {
            return ("EXPIRED · INVALID", true)
        }
        let resolvedCode = activeCode ?? AiracCycleGrid.airacCode(at: refUtc)
        return ("AIRAC \(resolvedCode) · VALID", false)
    }

    /// Toggles whether a bundled seed pack is active or locally offloaded, or deletes a downloaded pack.
    public func removeOrOffloadPack(for authority: AeroDataAuthority) {
        let downloaded = installedPacks.filter { $0.authority == authority && !$0.isBundledSeed }
        for pack in downloaded {
            try? FileManager.default.removeItem(atPath: pack.filePath)
        }
        installedPacks.removeAll { $0.authority == authority && !$0.isBundledSeed }
        if installedPacks.contains(where: { $0.authority == authority && $0.isBundledSeed }) {
            disabledSeedAuthorities.insert(authority)
        }
        persistDownloadedPacks()
        storeRevision &+= 1
    }

    /// Restores a bundled seed pack that was previously offloaded in-session.
    public func restoreBundledSeedIfPresent(for authority: AeroDataAuthority) {
        disabledSeedAuthorities.remove(authority)
        storeRevision &+= 1
    }

    public func canRestoreBundledSeed(for authority: AeroDataAuthority) -> Bool {
        disabledSeedAuthorities.contains(authority)
            && installedPacks.contains(where: { $0.authority == authority && $0.isBundledSeed })
    }

    /// Prunes expired non-bundled cycle files from `Application Support/AeroDataStore` (`v99n62` `prune_authority`).
    @discardableResult
    public func pruneExpiredCycles(at utc: Date? = nil) -> Int {
        let refUtc = utc ?? effectiveReferenceUtc
        var removedCount = 0
        var retained: [InstalledAeroPack] = []
        for pack in installedPacks {
            if !pack.isBundledSeed && refUtc >= pack.effectiveUntilUtc {
                try? FileManager.default.removeItem(atPath: pack.filePath)
                removedCount += 1
            } else {
                retained.append(pack)
            }
        }
        if removedCount > 0 {
            installedPacks = retained
            persistDownloadedPacks()
            storeRevision &+= 1
        }
        return removedCount
    }

    /// Polls the remote Flightbag / `v99n62` manifest, downloads missing `current` + `prefetch` cycles,
    /// verifies `SHA-256` before atomic installation, and prunes expired cycles.
    public func syncFromManifest(
        primaryUrl: URL? = AeroDataPackStore.defaultPrimaryManifestUrl,
        fallbackUrl: URL? = AeroDataPackStore.defaultFallbackManifestUrl
    ) async {
        syncState = .checkingManifest
        let refUtc = effectiveReferenceUtc
        let candidateUrls = [primaryUrl, fallbackUrl].compactMap { $0 }
        guard !candidateUrls.isEmpty else {
            syncState = .failed(message: "No manifest URL configured.")
            return
        }

        var fetchedData: Data?
        var lastError: String = "Manifest unreachable"

        for url in candidateUrls {
            do {
                var req = URLRequest(url: url, cachePolicy: .reloadIgnoringLocalCacheData, timeoutInterval: 15.0)
                req.setValue("SokolyEFB/0.4 (AeroDataStore)", forHTTPHeaderField: "User-Agent")
                let (data, resp) = try await URLSession.shared.data(for: req)
                if let http = resp as? HTTPURLResponse, !(200...299).contains(http.statusCode) {
                    lastError = "HTTP \(http.statusCode) from \(url.host ?? url.absoluteString)"
                    continue
                }
                fetchedData = data
                break
            } catch {
                lastError = error.localizedDescription
            }
        }

        guard let manifestBytes = fetchedData else {
            syncState = .failed(message: "Offline / Manifest fetch failed: \(lastError)")
            return
        }

        let entries = Self.parseRemoteManifestEntries(manifestBytes)
        guard !entries.isEmpty else {
            syncState = .failed(message: "Published manifest contained no readable cycle entries.")
            return
        }

        var downloadedCount = 0
        var verifiedExistingCount = 0

        for authority in AeroDataAuthority.allCases {
            let matching = entries.filter {
                AeroDataAuthority.fromManifestSlug($0.authority, fileName: $0.file ?? $0.url) == authority
            }
            guard !matching.isEmpty else { continue }

            let wanted = Self.selectCurrentAndPrefetch(entries: matching, at: refUtc)
            for remote in wanted {
                guard let effFrom = Self.resolveEffectiveFrom(remote),
                      let effUntil = Self.resolveEffectiveUntil(remote) else {
                    continue
                }
                if hasVerifiedInstalledPack(authority: authority, effectiveDate: remote.effectiveOn, sha256Hex: remote.sha256) {
                    verifiedExistingCount += 1
                    continue
                }
                do {
                    syncState = .downloading(authority: authority, fraction: 0.05)
                    let installed = try await downloadAndVerifyPack(
                        remote: remote,
                        authority: authority,
                        effectiveFrom: effFrom,
                        effectiveUntil: effUntil
                    )
                    installedPacks.removeAll { $0.id == installed.id && !$0.isBundledSeed }
                    installedPacks.append(installed)
                    disabledSeedAuthorities.remove(authority)
                    downloadedCount += 1
                } catch {
                    syncState = .failed(message: "\(authority.title): \(error.localizedDescription)")
                    persistDownloadedPacks()
                    storeRevision &+= 1
                    return
                }
            }
        }

        let pruned = pruneExpiredCycles(at: refUtc)
        persistDownloadedPacks()
        storeRevision &+= 1
        syncState = .synced(
            summary: "Downloaded \(downloadedCount), verified \(verifiedExistingCount), pruned \(pruned).",
            checkedAtUtc: Date()
        )
    }

    // MARK: - Internal Verification, Atomic Write & Seed Discovery

    private func hasVerifiedInstalledPack(
        authority: AeroDataAuthority,
        effectiveDate: String,
        sha256Hex: String
    ) -> Bool {
        installedPacks.contains { pack in
            pack.authority == authority
                && pack.effectiveDate == effectiveDate
                && pack.sha256Hex.caseInsensitiveCompare(sha256Hex) == .orderedSame
                && FileManager.default.fileExists(atPath: pack.filePath)
        }
    }

    private func downloadAndVerifyPack(
        remote: RemoteAeroManifestEntry,
        authority: AeroDataAuthority,
        effectiveFrom: Date,
        effectiveUntil: Date
    ) async throws -> InstalledAeroPack {
        var urlsToTry: [URL] = []
        if let primary = URL(string: remote.url) {
            urlsToTry.append(primary)
        }
        if let cid = remote.cid, !cid.isEmpty,
           let gateway = URL(string: "https://ipfs.io/ipfs/\(cid)") {
            urlsToTry.append(gateway)
        }
        guard !urlsToTry.isEmpty else {
            throw NSError(domain: "AeroDataStore", code: 1, userInfo: [NSLocalizedDescriptionKey: "Invalid blob URL"])
        }

        let cycleDir = storeRootDirectory
            .appendingPathComponent(authority.rawValue, isDirectory: true)
            .appendingPathComponent(remote.effectiveOn, isDirectory: true)
        try FileManager.default.createDirectory(at: cycleDir, withIntermediateDirectories: true)

        let destUrl = cycleDir.appendingPathComponent(authority.expectedFileName)
        let tmpUrl = cycleDir.appendingPathComponent("\(authority.expectedFileName).tmp")

        var lastErr: Error?
        for candidateUrl in urlsToTry {
            do {
                let (tempDownloadedUrl, resp) = try await URLSession.shared.download(from: candidateUrl)
                if let http = resp as? HTTPURLResponse, !(200...299).contains(http.statusCode) {
                    throw NSError(
                        domain: "AeroDataStore",
                        code: http.statusCode,
                        userInfo: [NSLocalizedDescriptionKey: "HTTP \(http.statusCode)"]
                    )
                }
                syncState = .downloading(authority: authority, fraction: 0.85)
                if FileManager.default.fileExists(atPath: tmpUrl.path) {
                    try FileManager.default.removeItem(at: tmpUrl)
                }
                try FileManager.default.moveItem(at: tempDownloadedUrl, to: tmpUrl)

                let computedSha = try Self.computeFileSha256Hex(at: tmpUrl)
                guard computedSha.caseInsensitiveCompare(remote.sha256) == .orderedSame else {
                    try? FileManager.default.removeItem(at: tmpUrl)
                    throw NSError(
                        domain: "AeroDataStore",
                        code: 2,
                        userInfo: [
                            NSLocalizedDescriptionKey: "SHA-256 mismatch (expected \(remote.sha256.prefix(8))…, got \(computedSha.prefix(8))…)",
                        ]
                    )
                }

                if FileManager.default.fileExists(atPath: destUrl.path) {
                    try FileManager.default.removeItem(at: destUrl)
                }
                try FileManager.default.moveItem(at: tmpUrl, to: destUrl)

                let attrs = try? FileManager.default.attributesOfItem(atPath: destUrl.path)
                let diskSize = (attrs?[.size] as? NSNumber)?.int64Value ?? remote.bytes

                return InstalledAeroPack(
                    authority: authority,
                    airacCode: remote.airac,
                    effectiveDate: remote.effectiveOn,
                    expiresDate: remote.nextEffectiveOn,
                    effectiveFromUtc: effectiveFrom,
                    effectiveUntilUtc: effectiveUntil,
                    sha256Hex: computedSha,
                    sizeBytes: diskSize,
                    minZoom: remote.minZoom ?? 0,
                    maxZoom: remote.maxZoom ?? 12,
                    bounds: remote.bounds ?? [-180.0, -85.0, 180.0, 85.0],
                    filePath: destUrl.path,
                    isBundledSeed: false
                )
            } catch {
                lastErr = error
            }
        }
        throw lastErr ?? NSError(domain: "AeroDataStore", code: 3, userInfo: [NSLocalizedDescriptionKey: "Download failed"])
    }

    public static func computeFileSha256Hex(at url: URL) throws -> String {
        let handle = try FileHandle(forReadingFrom: url)
        defer { try? handle.close() }
        var hasher = SHA256()
        while true {
            let chunk = try handle.read(upToCount: 1_048_576) ?? Data()
            if chunk.isEmpty { break }
            hasher.update(data: chunk)
        }
        let digest = hasher.finalize()
        return digest.map { String(format: "%02x", $0) }.joined()
    }

    private func persistDownloadedPacks() {
        let nonSeed = installedPacks.filter { !$0.isBundledSeed }
        do {
            try FileManager.default.createDirectory(at: storeRootDirectory, withIntermediateDirectories: true)
            let encoder = JSONEncoder()
            encoder.dateEncodingStrategy = .iso8601
            encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
            let data = try encoder.encode(nonSeed)
            let tmp = indexFileUrl.appendingPathExtension("tmp")
            try data.write(to: tmp, options: .atomic)
            if FileManager.default.fileExists(atPath: indexFileUrl.path) {
                try FileManager.default.removeItem(at: indexFileUrl)
            }
            try FileManager.default.moveItem(at: tmp, to: indexFileUrl)
        } catch {
            // Keep in-memory state intact even if sandbox disk is read-only.
        }
    }

    private static func loadPersistedPacks(from indexUrl: URL) -> [InstalledAeroPack] {
        guard let data = try? Data(contentsOf: indexUrl) else { return [] }
        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601
        return (try? decoder.decode([InstalledAeroPack].self, from: data)) ?? []
    }

    public static func parseRemoteManifestEntries(_ data: Data) -> [RemoteAeroManifestEntry] {
        if let rootObj = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any],
           let entriesArr = rootObj["entries"],
           let entriesData = try? JSONSerialization.data(withJSONObject: entriesArr),
           let decoded = try? JSONDecoder().decode([RemoteAeroManifestEntry].self, from: entriesData) {
            return decoded
        }
        return (try? JSONDecoder().decode([RemoteAeroManifestEntry].self, from: data)) ?? []
    }

    private static func resolveEffectiveFrom(_ entry: RemoteAeroManifestEntry) -> Date? {
        if let utc = entry.effectiveFromUtc, let parsed = AiracCycleGrid.parseIso8601Utc(utc) {
            return parsed
        }
        return AiracCycleGrid.parseIso8601Utc(entry.effectiveOn)
    }

    private static func resolveEffectiveUntil(_ entry: RemoteAeroManifestEntry) -> Date? {
        if let utc = entry.effectiveUntilUtc, let parsed = AiracCycleGrid.parseIso8601Utc(utc) {
            return parsed
        }
        return AiracCycleGrid.parseIso8601Utc(entry.nextEffectiveOn)
    }

    private static func selectCurrentAndPrefetch(
        entries: [RemoteAeroManifestEntry],
        at refUtc: Date
    ) -> [RemoteAeroManifestEntry] {
        var selected: [RemoteAeroManifestEntry] = []
        let active = entries
            .filter { item in
                guard let s = resolveEffectiveFrom(item), let e = resolveEffectiveUntil(item) else { return false }
                return s <= refUtc && refUtc < e
            }
            .sorted { (resolveEffectiveFrom($0) ?? .distantPast) > (resolveEffectiveFrom($1) ?? .distantPast) }
        if let cur = active.first {
            selected.append(cur)
        }
        let upcoming = entries
            .filter { item in
                guard let s = resolveEffectiveFrom(item) else { return false }
                return s > refUtc
            }
            .sorted { (resolveEffectiveFrom($0) ?? .distantFuture) < (resolveEffectiveFrom($1) ?? .distantFuture) }
        if let next = upcoming.first {
            selected.append(next)
        }
        return selected
    }

    // MARK: - Bundled `Flightbag` Cycle Seed Loader

    private static func locateResourceUrl(
        name: String,
        ext: String,
        overrideDir: URL?
    ) -> URL? {
        if let dir = overrideDir {
            let candidate = dir.appendingPathComponent("\(name).\(ext)")
            if FileManager.default.fileExists(atPath: candidate.path) {
                return candidate
            }
        }
        if let bundled = Bundle.main.url(forResource: name, withExtension: ext) {
            return bundled
        }
        let devDir = URL(fileURLWithPath: "/Users/fangluo/FSgame-wt-vision-svs/target/ipad-bundle-resources")
        let devCandidate = devDir.appendingPathComponent("\(name).\(ext)")
        if FileManager.default.fileExists(atPath: devCandidate.path) {
            return devCandidate
        }
        return nil
    }

    private static func discoverBundledSeedPacks(resourceDirectoryOverride: URL?) -> [InstalledAeroPack] {
        var packs: [InstalledAeroPack] = []

        // 1. 28-day FAA NASR (`flightbag_nasr_manifest.json` -> `airspace.pmtiles` + `nasr_waypoints.bin`)
        if let nasrJsonUrl = locateResourceUrl(
            name: "flightbag_nasr_manifest",
            ext: "json",
            overrideDir: resourceDirectoryOverride
        ),
           let data = try? Data(contentsOf: nasrJsonUrl),
           let obj = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any],
           let airac = obj["airac_code"] as? String,
           let effDate = obj["effective_date"] as? String,
           let expDate = obj["expires_date"] as? String,
           let fromStr = obj["effective_from_utc"] as? String,
           let untilStr = obj["effective_until_utc"] as? String,
           let effFrom = AiracCycleGrid.parseIso8601Utc(fromStr),
           let effUntil = AiracCycleGrid.parseIso8601Utc(untilStr) {
            if let airDict = obj["airspace_pmtiles"] as? [String: Any],
               let pmtilesUrl = locateResourceUrl(
                   name: "airspace",
                   ext: "pmtiles",
                   overrideDir: resourceDirectoryOverride
               ) {
                let sha = (airDict["sha256_hex"] as? String) ?? ""
                let bytes = (airDict["size_bytes"] as? NSNumber)?.int64Value ?? 0
                let minZ = (airDict["min_zoom"] as? Int) ?? 2
                let maxZ = (airDict["max_zoom"] as? Int) ?? 10
                let bounds = (airDict["bounds"] as? [Double]) ?? [-180.0, -14.58, 180.0, 82.0]
                packs.append(
                    InstalledAeroPack(
                        authority: .faaNasrAirspace,
                        airacCode: airac,
                        effectiveDate: effDate,
                        expiresDate: expDate,
                        effectiveFromUtc: effFrom,
                        effectiveUntilUtc: effUntil,
                        sha256Hex: sha,
                        sizeBytes: bytes,
                        minZoom: minZ,
                        maxZoom: maxZ,
                        bounds: bounds,
                        filePath: pmtilesUrl.path,
                        isBundledSeed: true
                    )
                )
            }

            if let wptUrl = locateResourceUrl(
                name: "nasr_waypoints",
                ext: "bin",
                overrideDir: resourceDirectoryOverride
            ) {
                let attrs = try? FileManager.default.attributesOfItem(atPath: wptUrl.path)
                let size = (attrs?[.size] as? NSNumber)?.int64Value ?? 0
                let upstreamSha = ((obj["upstream_nasr"] as? [String: Any])?["sha256_hex"] as? String) ?? ""
                packs.append(
                    InstalledAeroPack(
                        authority: .faaNasrWaypoints,
                        airacCode: airac,
                        effectiveDate: effDate,
                        expiresDate: expDate,
                        effectiveFromUtc: effFrom,
                        effectiveUntilUtc: effUntil,
                        sha256Hex: upstreamSha,
                        sizeBytes: size,
                        minZoom: 0,
                        maxZoom: 16,
                        bounds: [-180.0, -14.58, 180.0, 82.0],
                        filePath: wptUrl.path,
                        isBundledSeed: true
                    )
                )
            }
        }

        // 1b. 28-day FAA CIFP (`cifp_faa_cifp.json` -> `cifp_procedures.bin`)
        if let cifpJsonUrl = locateResourceUrl(
            name: "cifp_faa_cifp",
            ext: "json",
            overrideDir: resourceDirectoryOverride
        ),
           let cifpBinUrl = locateResourceUrl(
               name: "cifp_procedures",
               ext: "bin",
               overrideDir: resourceDirectoryOverride
           ),
           let data = try? Data(contentsOf: cifpJsonUrl),
           let obj = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any],
           let airac = obj["airac_code"] as? String,
           let effDate = obj["effective_date"] as? String,
           let expDate = obj["expires_date"] as? String,
           let fromStr = obj["effective_from_utc"] as? String,
           let untilStr = obj["effective_until_utc"] as? String,
           let effFrom = AiracCycleGrid.parseIso8601Utc(fromStr),
           let effUntil = AiracCycleGrid.parseIso8601Utc(untilStr),
           let binDict = obj["cifp_binary"] as? [String: Any] {
            let sha = (binDict["sha256_hex"] as? String) ?? ""
            let bytes = (binDict["size_bytes"] as? NSNumber)?.int64Value ?? 0
            let bounds = (binDict["bounds"] as? [Double]) ?? [-180.0, -14.63, 180.0, 71.40]
            packs.append(
                InstalledAeroPack(
                    authority: .faaCifp,
                    airacCode: airac,
                    effectiveDate: effDate,
                    expiresDate: expDate,
                    effectiveFromUtc: effFrom,
                    effectiveUntilUtc: effUntil,
                    sha256Hex: sha,
                    sizeBytes: bytes,
                    minZoom: 0,
                    maxZoom: 16,
                    bounds: bounds,
                    filePath: cifpBinUrl.path,
                    isBundledSeed: true
                )
            )
        }

        // 1c. 28-day FAA d-TPP Terminal Procedure Plates (`dtpp_faa_dtpp.json` -> `dtpp_plates.bin`)
        if let dtppJsonUrl = locateResourceUrl(
            name: "dtpp_faa_dtpp",
            ext: "json",
            overrideDir: resourceDirectoryOverride
        ),
           let dtppBinUrl = locateResourceUrl(
               name: "dtpp_plates",
               ext: "bin",
               overrideDir: resourceDirectoryOverride
           ),
           let data = try? Data(contentsOf: dtppJsonUrl),
           let obj = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any],
           let airac = obj["airac_code"] as? String,
           let effDate = obj["effective_date"] as? String,
           let expDate = obj["expires_date"] as? String,
           let fromStr = obj["effective_from_utc"] as? String,
           let untilStr = obj["effective_until_utc"] as? String,
           let effFrom = AiracCycleGrid.parseIso8601Utc(fromStr),
           let effUntil = AiracCycleGrid.parseIso8601Utc(untilStr),
           let binDict = obj["dtpp_binary"] as? [String: Any] {
            let sha = (binDict["sha256_hex"] as? String) ?? ""
            let bytes = (binDict["size_bytes"] as? NSNumber)?.int64Value ?? 0
            let bounds = (binDict["bounds"] as? [Double]) ?? [-179.9, 17.5, -64.0, 71.6]
            packs.append(
                InstalledAeroPack(
                    authority: .faaDtpp,
                    airacCode: airac,
                    effectiveDate: effDate,
                    expiresDate: expDate,
                    effectiveFromUtc: effFrom,
                    effectiveUntilUtc: effUntil,
                    sha256Hex: sha,
                    sizeBytes: bytes,
                    minZoom: 0,
                    maxZoom: 16,
                    bounds: bounds,
                    filePath: dtppBinUrl.path,
                    isBundledSeed: true
                )
            )
        }

        // 2. 56-day FAA Visual & IFR Enroute Charts (`chart_faa_*.json` -> `*.pmtiles`)
        let chartSeeds: [(manifestName: String, pmtilesName: String, authority: AeroDataAuthority)] = [
            ("chart_faa_vfr_sectional", "vfr_sectional", .faaVfrSectional),
            ("chart_faa_vfr_terminal", "vfr_terminal", .faaVfrTerminal),
            ("chart_faa_ifr_low", "ifr_low", .faaIfrLow),
            ("chart_faa_ifr_high", "ifr_high", .faaIfrHigh),
        ]
        for seed in chartSeeds {
            guard let jsonUrl = locateResourceUrl(
                name: seed.manifestName,
                ext: "json",
                overrideDir: resourceDirectoryOverride
            ),
                  let pmtilesUrl = locateResourceUrl(
                      name: seed.pmtilesName,
                      ext: "pmtiles",
                      overrideDir: resourceDirectoryOverride
                  ),
                  let data = try? Data(contentsOf: jsonUrl),
                  let obj = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any],
                  let airac = obj["airac_code"] as? String,
                  let effDate = obj["effective_date"] as? String,
                  let expDate = obj["expires_date"] as? String,
                  let fromStr = obj["effective_from_utc"] as? String,
                  let untilStr = obj["effective_until_utc"] as? String,
                  let effFrom = AiracCycleGrid.parseIso8601Utc(fromStr),
                  let effUntil = AiracCycleGrid.parseIso8601Utc(untilStr),
                  let chartDict = obj["chart_pmtiles"] as? [String: Any] else {
                continue
            }
            let sha = (chartDict["sha256_hex"] as? String) ?? ""
            let bytes = (chartDict["size_bytes"] as? NSNumber)?.int64Value ?? 0
            let minZ = (chartDict["min_zoom"] as? Int) ?? 0
            let maxZ = (chartDict["max_zoom"] as? Int) ?? 12
            let bounds = (chartDict["bounds"] as? [Double]) ?? [-180.0, 16.63, -63.28, 71.97]
            packs.append(
                InstalledAeroPack(
                    authority: seed.authority,
                    airacCode: airac,
                    effectiveDate: effDate,
                    expiresDate: expDate,
                    effectiveFromUtc: effFrom,
                    effectiveUntilUtc: effUntil,
                    sha256Hex: sha,
                    sizeBytes: bytes,
                    minZoom: minZ,
                    maxZoom: maxZ,
                    bounds: bounds,
                    filePath: pmtilesUrl.path,
                    isBundledSeed: true
                )
            )
        }

        return packs
    }
}

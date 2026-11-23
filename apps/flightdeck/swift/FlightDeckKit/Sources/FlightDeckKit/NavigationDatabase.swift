import Foundation

/// Navigation point classification across Earth (`FAA NASR` / `ICAO`) and off-world reference bodies.
public enum WaypointKind: String, Codable, CaseIterable, Sendable {
    case airport = "airport"
    case vortac = "vortac"
    case vorDme = "vor_dme"
    case vor = "vor"
    case tacan = "tacan"
    case ndb = "ndb"
    case fix = "fix"
    case vfrWaypoint = "vfr"
    case coordinate = "coordinate"
    case planetarySite = "site"

    /// ForeFlight-aligned type label for search & NavLog rows (`""` for airports — never `"APT"`).
    public var badge: String {
        switch self {
        case .airport: ""
        case .vortac: "VORTAC"
        case .vorDme: "VOR/DME"
        case .vor: "VOR"
        case .tacan: "TACAN"
        case .ndb: "NDB"
        case .fix: "FIX"
        case .vfrWaypoint: "VP"
        case .coordinate: "GPS"
        case .planetarySite: "SITE"
        }
    }

    /// SF Symbol icon representing this navigation point category in ForeFlight-style lists.
    public var symbolName: String {
        switch self {
        case .airport: "airplane.circle.fill"
        case .vortac, .vorDme, .vor: "dot.radiowaves.up.forward"
        case .tacan: "antenna.radiowaves.left.and.right"
        case .ndb: "wave.3.right"
        case .fix: "diamond.fill"
        case .vfrWaypoint: "flag.fill"
        case .coordinate: "location.north.circle.fill"
        case .planetarySite: "globe"
        }
    }

    /// Descriptive category label for detail sheets and NavLog rows.
    public var title: String {
        switch self {
        case .airport: "Airport"
        case .vortac: "VORTAC"
        case .vorDme: "VOR/DME"
        case .vor: "VOR"
        case .tacan: "TACAN / DME"
        case .ndb: "NDB"
        case .fix: "RNAV / IFR Fix"
        case .vfrWaypoint: "VFR Waypoint"
        case .coordinate: "Lat/Lon Coordinate"
        case .planetarySite: "Planetary Landmark"
        }
    }

    /// Search tie-breaker rank (prominent airports and VHF navaids ahead of 5-letter fixes).
    public var prominenceRank: Int {
        switch self {
        case .airport: 0
        case .vortac: 1
        case .vorDme: 2
        case .vor: 3
        case .tacan: 4
        case .ndb: 5
        case .fix: 6
        case .vfrWaypoint: 7
        case .coordinate: 8
        case .planetarySite: 0
        }
    }

    init?(packCode: UInt8) {
        switch packCode {
        case 0: self = .airport
        case 1: self = .vortac
        case 2: self = .vorDme
        case 3: self = .vor
        case 4: self = .tacan
        case 5: self = .ndb
        case 6: self = .fix
        case 7: self = .vfrWaypoint
        default: return nil
        }
    }
}

/// Resolved navigation point from `SIMWPT01` (`FAA NASR`), `AirportDatabase`, coordinate parser, or planetary gazetteer.
public struct NavigationPoint: Equatable, Identifiable, Sendable {
    public let key: String
    public let identifier: String
    public let kind: WaypointKind
    public let name: String
    public let region: String
    public let latitudeDeg: Double
    public let longitudeDeg: Double
    public let elevationDatumM: Double
    public let runways: [String]

    public var id: String { key }

    public init(
        key: String,
        identifier: String,
        kind: WaypointKind,
        name: String,
        region: String,
        latitudeDeg: Double,
        longitudeDeg: Double,
        elevationDatumM: Double = 0.0,
        runways: [String] = []
    ) {
        self.key = key
        self.identifier = identifier
        self.kind = kind
        self.name = name
        self.region = region
        self.latitudeDeg = latitudeDeg
        self.longitudeDeg = longitudeDeg
        self.elevationDatumM = elevationDatumM
        self.runways = runways
    }
}

/// Strict ASCII-only flight-plan input sanitizer and coordinate parser (`Pilotage` `RouteIdentifierInput` rules).
public enum RouteAsciiInput {
    /// Normalizes `raw` to uppercase ASCII route tokens (`A-Z`, `0-9`, space, and `./+-` for `LAT/LON`).
    /// Returns `nil` if `raw` contains any non-ASCII or disallowed punctuation character.
    public static func normalized(_ raw: String) -> String? {
        var output = ""
        output.reserveCapacity(raw.utf8.count)
        for scalar in raw.unicodeScalars {
            switch scalar.value {
            case 65...90, 48...57, 43, 45, 46, 47:
                output.unicodeScalars.append(scalar)
            case 97...122:
                if let upper = UnicodeScalar(scalar.value - 32) {
                    output.unicodeScalars.append(upper)
                }
            case 9, 10, 13, 32:
                output.append(" ")
            default:
                return nil
            }
        }
        return output
    }

    /// Splits normalized route text into ordered identifier words, omitting `"DCT"` filler tokens.
    public static func words(from text: String) -> [String] {
        text
            .split(whereSeparator: { $0.isWhitespace })
            .map { String($0).uppercased() }
            .filter { !$0.isEmpty && $0 != "DCT" }
    }

    /// Parses decimal slash (`40.58/-74.70`, `38.85N/77.04W`) or ICAO compact (`3851N07702W`, `3851N/07702W`) coordinates.
    public static func parseCoordinate(_ rawToken: String) -> NavigationPoint? {
        let token = rawToken.trimmingCharacters(in: .whitespacesAndNewlines).uppercased()
        guard !token.isEmpty else { return nil }

        if token.contains("/") {
            let parts = token.split(separator: "/", omittingEmptySubsequences: false)
            if parts.count == 2,
               let lat = parseSignedAngle(String(parts[0]), isLatitude: true),
               let lon = parseSignedAngle(String(parts[1]), isLatitude: false) {
                return makeCoordinatePoint(lat: lat, lon: lon)
            }
        }

        // ICAO compact `DDMM[N|S]DDDMM[E|W]` (e.g. `3851N07702W`).
        if token.count == 11 {
            let chars = Array(token)
            let latHem = chars[4]
            let lonHem = chars[10]
            if (latHem == "N" || latHem == "S"),
               (lonHem == "E" || lonHem == "W"),
               let dLat = Double(String(chars[0...1])),
               let mLat = Double(String(chars[2...3])),
               let dLon = Double(String(chars[5...7])),
               let mLon = Double(String(chars[8...9])),
               mLat < 60.0, mLon < 60.0 {
                let lat = (dLat + mLat / 60.0) * (latHem == "S" ? -1.0 : 1.0)
                let lon = (dLon + mLon / 60.0) * (lonHem == "W" ? -1.0 : 1.0)
                if (-90.0...90.0).contains(lat), (-180.0...180.0).contains(lon) {
                    return makeCoordinatePoint(lat: lat, lon: lon)
                }
            }
        }
        return nil
    }

    private static func parseSignedAngle(_ part: String, isLatitude: Bool) -> Double? {
        guard !part.isEmpty else { return nil }
        var body = part
        var sign = 1.0
        if let last = body.last {
            if isLatitude && (last == "N" || last == "S") {
                sign = (last == "S") ? -1.0 : 1.0
                body.removeLast()
            } else if !isLatitude && (last == "E" || last == "W") {
                sign = (last == "W") ? -1.0 : 1.0
                body.removeLast()
            }
        }
        // Support `3851` (`DDMM`) or `07702` (`DDDMM`) when hemisphere letter was stripped and no decimal point is present.
        if !body.contains("."), !body.hasPrefix("-"), !body.hasPrefix("+") {
            if isLatitude, body.count == 4,
               let deg = Double(body.prefix(2)),
               let min = Double(body.suffix(2)),
               min < 60.0 {
                let val = (deg + min / 60.0) * sign
                return (-90.0...90.0).contains(val) ? val : nil
            }
            if !isLatitude, body.count == 5,
               let deg = Double(body.prefix(3)),
               let min = Double(body.suffix(2)),
               min < 60.0 {
                let val = (deg + min / 60.0) * sign
                return (-180.0...180.0).contains(val) ? val : nil
            }
        }
        guard let numeric = Double(body) else { return nil }
        let val = numeric * sign
        let limit = isLatitude ? 90.0 : 180.0
        return (-limit...limit).contains(val) ? val : nil
    }

    private static func makeCoordinatePoint(lat: Double, lon: Double) -> NavigationPoint {
        let ns = lat >= 0.0 ? "N" : "S"
        let ew = lon >= 0.0 ? "E" : "W"
        let ident = String(format: "%.2f%@/%.2f%@", abs(lat), ns, abs(lon), ew)
        let name = String(format: "WGS-84 %.4f°, %.4f°", lat, lon)
        return NavigationPoint(
            key: String(format: "gps:%.5f,%.5f", lat, lon),
            identifier: ident,
            kind: .coordinate,
            name: name,
            region: "GPS",
            latitudeDeg: lat,
            longitudeDeg: lon
        )
    }
}

/// Memory-mapped binary reader (`SIMWPT01`) over 85,000+ FAA NASR airports, VOR/VORTAC/NDB navaids, and RNAV/VFR fixes,
/// plus fallback fixtures for headless unit tests and planetary gazetteers for Moon and Mars.
public final class NavigationDatabaseIndex: Sendable {
    public static let shared = NavigationDatabaseIndex()

    private let mappedData: Data?
    private let recordCount: Int
    private let tableOffset: Int = 16
    private let recordStride: Int = 24
    private let poolOffset: Int

    public init(bundleUrl: URL? = nil) {
        let resolvedUrl = bundleUrl ?? Self.locatePackUrl()
        if let url = resolvedUrl,
           let data = try? Data(contentsOf: url, options: .mappedIfSafe),
           data.count >= 16,
           String(data: data.subdata(in: 0..<8), encoding: .ascii) == "SIMWPT01" {
            let version = Self.readUInt32LE(data, at: 8)
            let count = Int(Self.readUInt32LE(data, at: 12))
            let expectedPool = 16 + count * 24
            if version == 1, count > 0, expectedPool <= data.count {
                self.mappedData = data
                self.recordCount = count
                self.poolOffset = expectedPool
                return
            }
        }
        self.mappedData = nil
        self.recordCount = 0
        self.poolOffset = 16
    }

    private static func locatePackUrl() -> URL? {
        if let bundled = Bundle.main.url(forResource: "nasr_waypoints", withExtension: "bin") {
            return bundled
        }
        let devCandidate = URL(
            fileURLWithPath: "/Users/fangluo/FSgame-wt-vision-svs/target/ipad-bundle-resources/nasr_waypoints.bin"
        )
        if FileManager.default.fileExists(atPath: devCandidate.path) {
            return devCandidate
        }
        return nil
    }

    /// Searches navigation points matching `rawQuery` on `primaryBody`, ranked by exact identifier match and proximity.
    public func search(
        query rawQuery: String,
        primaryBody: PrimaryCelestialBody,
        allowEarthDatabase: Bool = true,
        nearLatDeg: Double?,
        nearLonDeg: Double?,
        radiusMeters: Double,
        airportLookup: (String) -> ResolvedAirport?,
        limit: Int = 24
    ) -> [NavigationPoint] {
        guard let normalized = RouteAsciiInput.normalized(rawQuery) else { return [] }
        let query = normalized.trimmingCharacters(in: .whitespaces)
        guard !query.isEmpty else { return [] }

        var candidates: [NavigationPoint] = []
        var seenKeys: Set<String> = []
        let appendUnique = { (pt: NavigationPoint) in
            if seenKeys.insert(pt.key).inserted {
                candidates.append(pt)
            }
        }

        if let coordPt = RouteAsciiInput.parseCoordinate(query) {
            appendUnique(coordPt)
        }

        switch primaryBody {
        case .moon:
            for pt in Self.lunarGazetteer where pt.identifier.hasPrefix(query) || pt.name.uppercased().contains(query) {
                appendUnique(pt)
            }
        case .mars:
            for pt in Self.martianGazetteer where pt.identifier.hasPrefix(query) || pt.name.uppercased().contains(query) {
                appendUnique(pt)
            }
        case .earth:
            guard allowEarthDatabase else { break }
            // 1. Check `AirportDatabase::builtin()` for exact ICAO/FAA/IATA match (includes surveyed runways & global ICAO).
            if let apt = airportLookup(query) {
                appendUnique(
                    NavigationPoint(
                        key: "apt:\(apt.code)",
                        identifier: apt.code,
                        kind: .airport,
                        name: apt.municipality.isEmpty ? apt.name : "\(apt.name) · \(apt.municipality)",
                        region: "",
                        latitudeDeg: apt.latitudeDeg,
                        longitudeDeg: apt.longitudeDeg,
                        elevationDatumM: apt.elevationDatumM,
                        runways: apt.runways
                    )
                )
            }

            // 2. Binary-search `SIMWPT01` (`nasr_waypoints.bin`) across Airports, VORs, NDBs, and Fixes.
            if mappedData != nil, recordCount > 0 {
                for pt in searchBinaryPrefix(query, maxScan: 180, airportLookup: airportLookup) {
                    appendUnique(pt)
                }
                if query.count == 3 {
                    for pt in searchBinaryPrefix("K" + query, maxScan: 12, airportLookup: airportLookup) {
                        appendUnique(pt)
                    }
                }
            } else {
                for pt in Self.fallbackEarthPoints where pt.identifier.hasPrefix(query) {
                    appendUnique(pt)
                }
            }
        }

        let sorted = candidates.sorted { a, b in
            let aExact = (a.identifier == query)
            let bExact = (b.identifier == query)
            if aExact != bExact { return aExact }

            if let refLat = nearLatDeg, let refLon = nearLonDeg {
                let dA = RouteLegMetrics.between(
                    lat1Deg: refLat,
                    lon1Deg: refLon,
                    lat2Deg: a.latitudeDeg,
                    lon2Deg: a.longitudeDeg,
                    radiusMeters: radiusMeters
                ).distanceMeters
                let dB = RouteLegMetrics.between(
                    lat1Deg: refLat,
                    lon1Deg: refLon,
                    lat2Deg: b.latitudeDeg,
                    lon2Deg: b.longitudeDeg,
                    radiusMeters: radiusMeters
                ).distanceMeters
                if aExact && bExact && abs(dA - dB) > 1_852.0 {
                    return dA < dB
                }
                if a.kind.prominenceRank != b.kind.prominenceRank && !aExact {
                    return a.kind.prominenceRank < b.kind.prominenceRank
                }
                if abs(dA - dB) > 500.0 {
                    return dA < dB
                }
            }
            if a.kind.prominenceRank != b.kind.prominenceRank {
                return a.kind.prominenceRank < b.kind.prominenceRank
            }
            return a.identifier < b.identifier
        }

        return Array(sorted.prefix(max(limit, 1)))
    }

    private func searchBinaryPrefix(
        _ prefix: String,
        maxScan: Int,
        airportLookup: (String) -> ResolvedAirport?
    ) -> [NavigationPoint] {
        guard let data = mappedData, recordCount > 0, !prefix.isEmpty else { return [] }
        let prefixBytes = Array(prefix.utf8.prefix(8))

        // Lower-bound binary search on 8-byte NUL-padded ASCII identifier.
        var lo = 0
        var hi = recordCount
        while lo < hi {
            let mid = lo + (hi - lo) / 2
            if compareIdentToPrefixKey(data: data, index: mid, key: prefixBytes) < 0 {
                lo = mid + 1
            } else {
                hi = mid
            }
        }

        var results: [NavigationPoint] = []
        let end = min(recordCount, lo + maxScan)
        var idx = lo
        while idx < end {
            guard let ident = readIdent(data: data, index: idx) else { break }
            if !ident.hasPrefix(prefix) { break }
            if let point = decodeRecord(data: data, index: idx, ident: ident, airportLookup: airportLookup) {
                results.append(point)
            }
            idx += 1
        }
        return results
    }

    private func compareIdentToPrefixKey(data: Data, index: Int, key: [UInt8]) -> Int {
        let base = tableOffset + index * recordStride
        for i in 0..<8 {
            let recByte = data[base + i]
            let keyByte: UInt8 = i < key.count ? key[i] : 0
            if recByte != keyByte {
                return Int(recByte) - Int(keyByte)
            }
        }
        return 0
    }

    private func readIdent(data: Data, index: Int) -> String? {
        let base = tableOffset + index * recordStride
        var bytes: [UInt8] = []
        bytes.reserveCapacity(8)
        for i in 0..<8 {
            let b = data[base + i]
            if b == 0 { break }
            bytes.append(b)
        }
        return String(bytes: bytes, encoding: .ascii)
    }

    private func decodeRecord(
        data: Data,
        index: Int,
        ident: String,
        airportLookup: (String) -> ResolvedAirport?
    ) -> NavigationPoint? {
        let base = tableOffset + index * recordStride
        let kindCode = data[base + 8]
        guard let kind = WaypointKind(packCode: kindCode) else { return nil }
        let nameLen = Int(data[base + 9])
        var stateBytes: [UInt8] = []
        if data[base + 10] != 0 { stateBytes.append(data[base + 10]) }
        if data[base + 11] != 0 { stateBytes.append(data[base + 11]) }
        let state = String(bytes: stateBytes, encoding: .ascii) ?? ""

        let latE6 = Self.readInt32LE(data, at: base + 12)
        let lonE6 = Self.readInt32LE(data, at: base + 16)
        let latDeg = Double(latE6) / 1_000_000.0
        let lonDeg = Double(lonE6) / 1_000_000.0
        let strOff = Int(Self.readUInt32LE(data, at: base + 20))
        let name: String
        if nameLen > 0, poolOffset + strOff + nameLen <= data.count {
            let sub = data.subdata(in: (poolOffset + strOff)..<(poolOffset + strOff + nameLen))
            name = String(data: sub, encoding: .ascii) ?? ident
        } else {
            name = ident
        }

        if kind == .airport, let apt = airportLookup(ident), apt.code == ident {
            return NavigationPoint(
                key: "apt:\(apt.code)",
                identifier: apt.code,
                kind: .airport,
                name: apt.municipality.isEmpty ? apt.name : "\(apt.name) · \(apt.municipality)",
                region: state,
                latitudeDeg: apt.latitudeDeg,
                longitudeDeg: apt.longitudeDeg,
                elevationDatumM: apt.elevationDatumM,
                runways: apt.runways
            )
        }

        return NavigationPoint(
            key: "\(kind.rawValue):\(ident):\(state):\(latE6):\(lonE6)",
            identifier: ident,
            kind: kind,
            name: name,
            region: state,
            latitudeDeg: latDeg,
            longitudeDeg: lonDeg
        )
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

    // MARK: - Headless Fallback & Planetary Gazetteers

    private static let fallbackEarthPoints: [NavigationPoint] = [
        NavigationPoint(
            key: "vor_dme:SBJ:NJ", identifier: "SBJ", kind: .vorDme,
            name: "Solberg (112.90 MHz)", region: "NJ", latitudeDeg: 40.58304, longitudeDeg: -74.74180
        ),
        NavigationPoint(
            key: "vortac:VUZ:AL", identifier: "VUZ", kind: .vortac,
            name: "Vulcan (114.40 MHz)", region: "AL", latitudeDeg: 33.67019, longitudeDeg: -86.89983
        ),
        NavigationPoint(
            key: "vortac:RMG:GA", identifier: "RMG", kind: .vortac,
            name: "Rome (115.40 MHz)", region: "GA", latitudeDeg: 34.16256, longitudeDeg: -85.11939
        ),
        NavigationPoint(
            key: "vor_dme:AML:VA", identifier: "AML", kind: .vorDme,
            name: "Armel (113.50 MHz)", region: "VA", latitudeDeg: 38.93459, longitudeDeg: -77.46670
        ),
        NavigationPoint(
            key: "fix:CUTOP:MD", identifier: "CUTOP", kind: .fix,
            name: "ZDC Center", region: "MD", latitudeDeg: 39.15583, longitudeDeg: -76.98972
        ),
        NavigationPoint(
            key: "vortac:MSL:AL", identifier: "MSL", kind: .vortac,
            name: "Muscle Shoals (116.50 MHz)", region: "AL", latitudeDeg: 34.70686, longitudeDeg: -87.49158
        ),
        NavigationPoint(
            key: "fix:CABAS:AL", identifier: "CABAS", kind: .fix,
            name: "ZME Center", region: "AL", latitudeDeg: 34.62958, longitudeDeg: -87.30364
        ),
        NavigationPoint(
            key: "fix:HABJE:AL", identifier: "HABJE", kind: .fix,
            name: "ZME Center", region: "AL", latitudeDeg: 34.48222, longitudeDeg: -87.42111
        ),
        NavigationPoint(
            key: "ndb:GTN:DC", identifier: "GTN", kind: .ndb,
            name: "Georgetown (323 kHz)", region: "DC", latitudeDeg: 38.92194, longitudeDeg: -77.07250
        ),
    ]

    private static let lunarGazetteer: [NavigationPoint] = [
        NavigationPoint(
            key: "site:APOLLO11", identifier: "APOLLO11", kind: .planetarySite,
            name: "Mare Tranquillitatis Statio", region: "MOON", latitudeDeg: 0.6741, longitudeDeg: 23.4730
        ),
        NavigationPoint(
            key: "site:TYCHO", identifier: "TYCHO", kind: .planetarySite,
            name: "Tycho Central Peak", region: "MOON", latitudeDeg: -43.31, longitudeDeg: -11.36
        ),
        NavigationPoint(
            key: "site:SHACKLETON", identifier: "SHACKLETON", kind: .planetarySite,
            name: "Shackleton South Rim", region: "MOON", latitudeDeg: -89.54, longitudeDeg: 0.0
        ),
        NavigationPoint(
            key: "site:COPERNICUS", identifier: "COPERNICUS", kind: .planetarySite,
            name: "Copernicus Crater Floor", region: "MOON", latitudeDeg: 9.62, longitudeDeg: -20.08
        ),
    ]

    private static let martianGazetteer: [NavigationPoint] = [
        NavigationPoint(
            key: "site:JEZERO", identifier: "JEZERO", kind: .planetarySite,
            name: "Jezero Delta Octavia E. Butler", region: "MARS", latitudeDeg: 18.4447, longitudeDeg: 77.4508
        ),
        NavigationPoint(
            key: "site:OLYMPUS", identifier: "OLYMPUS", kind: .planetarySite,
            name: "Olympus Mons Caldera", region: "MARS", latitudeDeg: 18.65, longitudeDeg: -133.80
        ),
        NavigationPoint(
            key: "site:GALE", identifier: "GALE", kind: .planetarySite,
            name: "Gale Crater Aeolis Mons", region: "MARS", latitudeDeg: -5.40, longitudeDeg: 137.80
        ),
        NavigationPoint(
            key: "site:VALLES", identifier: "VALLES", kind: .planetarySite,
            name: "Valles Marineris Melas Chasma", region: "MARS", latitudeDeg: -10.30, longitudeDeg: -72.80
        ),
    ]
}

import Foundation
@_exported import IndicateAppleDisplay
import simd

/// Distinct physical celestial bodies supported by `FlightDeckKit` (never duplicating Earth across vertical datums).
public enum PrimaryCelestialBody: String, CaseIterable, Identifiable, Sendable {
    /// Earth (`WGS-84` reference ellipsoid, `a = 6,378.137 km`, `g = 9.81 m/s²`).
    case earth = "Earth"
    /// The Moon (`IAU` mean sphere, `R = 1,737.400 km`, `g = 1.62 m/s²`, vacuum sky).
    case moon = "Moon"
    /// Mars (`IAU` oblate spheroid, `a = 3,396.190 km`, `b = 3,376.200 km`, `g = 3.72 m/s²`).
    case mars = "Mars"

    public var id: String { rawValue }

    /// Explicit reference figure & surface gravity summary.
    public var figureSummary: String {
        switch self {
        case .earth: "WGS-84 Ellipsoid · a=6378.137 km · g=9.81 m/s²"
        case .moon: "IAU Sphere · R=1737.400 km · g=1.62 m/s² · Vacuum"
        case .mars: "IAU Spheroid · a=3396.190 km · g=3.72 m/s²"
        }
    }

    /// SF Symbol icon representing the celestial body.
    public var symbolName: String {
        switch self {
        case .earth: "globe.americas.fill"
        case .moon: "moon.stars.fill"
        case .mars: "circle.hexagongrid.fill"
        }
    }
}

/// Vertical height datum on `WGS-84` Earth (`MSL` orthometric geoid height vs. `HAE` geometric ellipsoidal height).
public enum EarthVerticalDatum: String, CaseIterable, Identifiable, Sendable {
    /// Orthometric height above the `EGM96` geoid (`MSL` — standard FAA charts, baro & GNSS aviation altimetry).
    case egm96Msl = "MSL"
    /// Pure geometric height above the `WGS-84` reference ellipsoid (`HAE` — raw GNSS / INS sensor output).
    case wgs84Hae = "HAE"

    public var id: String { rawValue }

    public var title: String {
        switch self {
        case .egm96Msl: "MSL (EGM96 Geoid)"
        case .wgs84Hae: "HAE (WGS-84 Ellipsoid)"
        }
    }

    public var subtitle: String {
        switch self {
        case .egm96Msl: "Orthometric altitude above mean sea level"
        case .wgs84Hae: "Geometric height above WGS-84 ellipsoid"
        }
    }
}

/// Explicitly declared celestial body, reference ellipsoid/spheroid, and vertical datum wire code.
///
/// Nothing in `FlightDeckKit` assumes Earth or `WGS-84` implicitly: callers must select and pass
/// an explicit `CelestialBody` when constructing or reconfiguring an `FlightDeckController`.
public enum CelestialBody: UInt8, CaseIterable, Identifiable, Sendable {
    /// Earth on the `WGS-84` ellipsoid with orthometric heights above the `EGM96` geoid (`MSL`).
    case earthWgs84Egm96 = 0
    /// Earth on the `WGS-84` ellipsoid with pure ellipsoidal (`HAE`) heights.
    case earthWgs84Ellipsoid = 1
    /// The Moon on the `IAU` mean sphere (`R = 1,737.4 km`, `GRAIL` `g = 1.62 m/s²`, vacuum sky).
    case moonIauSphere = 2
    /// Mars on the `IAU` oblate spheroid (`a = 3,396.19 km`, `b = 3,376.20 km`, `g = 3.72 m/s²`).
    case marsIauSpheroid = 3

    public var id: UInt8 { rawValue }

    /// Resolves a distinct celestial body + Earth vertical datum into its explicit `CelestialBody`.
    public static func resolve(
        primary: PrimaryCelestialBody,
        earthVerticalDatum: EarthVerticalDatum = .egm96Msl
    ) -> CelestialBody {
        switch primary {
        case .earth:
            earthVerticalDatum == .wgs84Hae ? .earthWgs84Ellipsoid : .earthWgs84Egm96
        case .moon:
            .moonIauSphere
        case .mars:
            .marsIauSpheroid
        }
    }

    /// Primary celestial body without duplicating Earth across vertical datums.
    public var primaryBody: PrimaryCelestialBody {
        switch self {
        case .earthWgs84Egm96, .earthWgs84Ellipsoid: .earth
        case .moonIauSphere: .moon
        case .marsIauSpheroid: .mars
        }
    }

    /// Active Earth vertical datum when `primaryBody == .earth` (`.egm96Msl` fallback otherwise).
    public var earthVerticalDatum: EarthVerticalDatum {
        switch self {
        case .earthWgs84Ellipsoid: .wgs84Hae
        case .earthWgs84Egm96, .moonIauSphere, .marsIauSpheroid: .egm96Msl
        }
    }

    /// Compact body abbreviation.
    public var abbrevLabel: String {
        switch self {
        case .earthWgs84Egm96, .earthWgs84Ellipsoid: "EARTH"
        case .moonIauSphere: "MOON"
        case .marsIauSpheroid: "MARS"
        }
    }

    /// Explicit aviation/astronautics badge text declaring both body and vertical datum.
    public var badgeLabel: String {
        switch self {
        case .earthWgs84Egm96: "EARTH · WGS-84 / EGM96 MSL"
        case .earthWgs84Ellipsoid: "EARTH · WGS-84 HAE"
        case .moonIauSphere: "MOON · IAU 1737.4 KM"
        case .marsIauSpheroid: "MARS · IAU SPHEROID"
        }
    }

    /// Physical geodesy and gravity summary shown in the body selector menu.
    public var geodesySummary: String {
        switch self {
        case .earthWgs84Egm96: "a=6378.137 km · EGM96 Geoid · g=9.81 m/s²"
        case .earthWgs84Ellipsoid: "a=6378.137 km · Ellipsoidal HAE · g=9.81 m/s²"
        case .moonIauSphere: "R=1737.400 km · GRAIL J2 · g=1.62 m/s² · No Atm"
        case .marsIauSpheroid: "a=3396.190 km · IAU Spheroid · g=3.72 m/s²"
        }
    }

    /// Altitude datum tag used in English UI telemetry readouts.
    public var altitudeDatumTag: String {
        switch self {
        case .earthWgs84Egm96: "MSL"
        case .earthWgs84Ellipsoid: "HAE"
        case .moonIauSphere: "IAU"
        case .marsIauSpheroid: "IAU"
        }
    }

    /// Whether this body is `WGS-84` Earth (where `FAA NASR`, `VFR` charts, `WMM2025`, and `ADS-B` apply).
    public var isEarthWgs84: Bool {
        switch self {
        case .earthWgs84Egm96, .earthWgs84Ellipsoid: true
        case .moonIauSphere, .marsIauSpheroid: false
        }
    }
}

/// Physical ellipsoid, render-sphere bounds, and gravity parameters queried from `sim-geo`.
public struct CelestialBodyParameters: Equatable, Sendable {
    public let body: CelestialBody
    public let renderRadiusMeters: Double
    public let highestGroundMeters: Double
    public let lowestGroundMeters: Double
    public let semiMajorMeters: Double
    public let semiMinorMeters: Double
    public let surfaceGravityMps2: Double
    public let hasAtmosphere: Bool
    public let isEarthWgs84: Bool

    public init(
        body: CelestialBody,
        renderRadiusMeters: Double,
        highestGroundMeters: Double,
        lowestGroundMeters: Double,
        semiMajorMeters: Double,
        semiMinorMeters: Double,
        surfaceGravityMps2: Double,
        hasAtmosphere: Bool,
        isEarthWgs84: Bool
    ) {
        self.body = body
        self.renderRadiusMeters = renderRadiusMeters
        self.highestGroundMeters = highestGroundMeters
        self.lowestGroundMeters = lowestGroundMeters
        self.semiMajorMeters = semiMajorMeters
        self.semiMinorMeters = semiMinorMeters
        self.surfaceGravityMps2 = surfaceGravityMps2
        self.hasAtmosphere = hasAtmosphere
        self.isEarthWgs84 = isEarthWgs84
    }
}

/// Ephemeris-driven Sun, Moon, atmosphere, and starfield snapshot for the declared body.
public struct CelestialSkySnapshot: Equatable, Sendable {
    public let hasSun: Bool
    public let sunDirectionFixed: SIMD3<Double>
    public let sunAngularRadiusDeg: Double
    public let hasMoon: Bool
    public let moonDirectionFixed: SIMD3<Double>
    public let moonAngularRadiusDeg: Double
    public let nightBrightness: Float
    public let atmosphere: Bool
    public let starsUpdated: Bool
    public let starCount: UInt32
}

/// Telemetry source modes supported by the Synthetic Vision System.
public enum TelemetrySourceMode: UInt8, CaseIterable, Identifiable, Sendable {
    /// Deterministic cruise across ridges and airspace shelves.
    case njCorridorCruise = 0
    /// High-density ADS-B traffic encounter demonstrating `TA` / `RA` CPA promotion and callsigns.
    case highDensityTrafficCpa = 1
    /// Stabilized `3°` approach with HITS gates.
    case kewrApproachWithTfr = 2
    /// Recorded `avionics-link` Connext hardware session (`47N`, `N19880`, `JBU1990`, `DWI2363`, `ATN3345`, `DLH499`).
    case connextReplay = 3
    /// Live Connext TCP relay / cockpit accessory stream (`avionics-link` protocol).
    case connextLiveTcp = 4
    /// Live Wi-Fi GDL-90 UDP (`:4000`) stream (ForeFlight / Sentry / Stratux / Garmin).
    case gdl90LiveUdp = 5
    /// FlightAware / Google Earth KML (`<gx:Track>`) or GPX flight track preview.
    case kmlReplay = 6

    public var id: UInt8 { rawValue }

    public var title: String {
        switch self {
        case .njCorridorCruise: "Corridor Cruise"
        case .highDensityTrafficCpa: "Traffic CPA"
        case .kewrApproachWithTfr: "3.0° Approach + HITS"
        case .connextReplay: "Connext Capture (47N)"
        case .connextLiveTcp: "Connext Live TCP"
        case .gdl90LiveUdp: "GDL-90 UDP :4000"
        case .kmlReplay: "KML Track Preview"
        }
    }

    public var subtitle: String {
        switch self {
        case .njCorridorCruise: "Synthetic flight corridor • Terrain + Airspace"
        case .connextReplay: "Recorded Garmin Connext hardware capture"
        case .highDensityTrafficCpa: "Converging traffic • TA/RA callsigns"
        case .kewrApproachWithTfr: "3.0° HITS glideslope • Approach guidance"
        case .connextLiveTcp: "avionics-link TCP bridge (GNSS + AHRS + ADS-B + FPN)"
        case .gdl90LiveUdp: "Portable ADS-B/AHRS receiver on UDP port 4000"
        case .kmlReplay: "AirDrop / Open-In KML & GPX 6-DoF track preview"
        }
    }
}

/// Decoded KML / GPX track summary and preview playback status.
public struct TrackPlaybackState: Equatable, Sendable {
    public var hasTrack: Bool
    public var paused: Bool
    public var speedMultiplier: Float
    public var elapsedSeconds: Double
    public var durationSeconds: Double
    public var pointCount: Int
    public var callsign: String
    public var route: String
    public var title: String
    /// Polyline coordinates `[lon, lat, alt_m]` in planetographic degrees and metres above declared datum.
    public var coordinates: [SIMD3<Double>]

    public init(
        hasTrack: Bool = false,
        paused: Bool = true,
        speedMultiplier: Float = 1.0,
        elapsedSeconds: Double = 0.0,
        durationSeconds: Double = 0.0,
        pointCount: Int = 0,
        callsign: String = "",
        route: String = "",
        title: String = "",
        coordinates: [SIMD3<Double>] = []
    ) {
        self.hasTrack = hasTrack
        self.paused = paused
        self.speedMultiplier = speedMultiplier
        self.elapsedSeconds = elapsedSeconds
        self.durationSeconds = durationSeconds
        self.pointCount = pointCount
        self.callsign = callsign
        self.route = route
        self.title = title
        self.coordinates = coordinates
    }

    public var progressFraction: Double {
        guard durationSeconds > 0.0 else { return 0.0 }
        return min(max(elapsedSeconds / durationSeconds, 0.0), 1.0)
    }
}

/// Digital Crown virtual/reality opacity blend weights enforcing a minimum `15%` real-world passthrough floor.
public struct SvsBlendWeights: Equatable, Sendable {
    public static let minPassthroughFraction: Float = 0.15
    public static let maxVirtualOpacity: Float = 0.85

    public let crownTurn: Float
    public let virtualTerrainOpacity: Float
    public let passthroughFraction: Float
    public let airspaceFillOpacity: Float
    public let airspaceRimOpacity: Float
    public let symbologyOpacity: Float

    public init(
        crownTurn: Float = 0.55,
        virtualTerrainOpacity: Float = 0.55 * 0.85,
        passthroughFraction: Float = 1.0 - 0.55 * 0.85,
        airspaceFillOpacity: Float = 0.22,
        airspaceRimOpacity: Float = 0.72,
        symbologyOpacity: Float = 0.96
    ) {
        self.crownTurn = crownTurn
        self.virtualTerrainOpacity = min(max(virtualTerrainOpacity, 0), Self.maxVirtualOpacity)
        self.passthroughFraction = max(passthroughFraction, Self.minPassthroughFraction)
        self.airspaceFillOpacity = airspaceFillOpacity
        self.airspaceRimOpacity = airspaceRimOpacity
        self.symbologyOpacity = symbologyOpacity
    }
}

/// Decoded ownship GNSS, AHRS, air-data, and active flight-plan leg snapshot.
public struct OwnshipSnapshot: Equatable, Sendable {
    public let gnssValid: Bool
    public let groundVectorValid: Bool
    public let latitudeDeg: Double?
    public let longitudeDeg: Double?
    public let altitudeMslM: Double?
    public let pressureAltitudeFt: Double?
    public let groundSpeedKt: Double?
    public let indicatedAirspeedKt: Double?
    public let verticalSpeedFpm: Double?
    public let magneticHeadingDeg: Double?
    public let trueHeadingDeg: Double?
    public let groundTrackTrueDeg: Double?
    public let pitchDeg: Double?
    public let rollDeg: Double?
    public let aglFt: Double?
    public let callsign: String
    public let activeLeg: String
    public let connextRecords: UInt64
    public let gdl90Frames: UInt64
}

/// 3D airport/landing runway surface and `3°` Highway-in-the-Sky approach gates in local ENU metres.
public struct RunwayOverlay3D: Equatable, Sendable {
    public let label: String
    public let cornersEnuM: [SIMD3<Float>]
    public let centerlineEnuM: [SIMD3<Float>]
    public let glideslopeGatesEnuM: [[SIMD3<Float>]]
}

/// 3D synthetic terrain mesh in ownship-centered local East-North-Up metres.
public struct TerrainSurfaceMesh: Equatable, Sendable {
    public let positionsEnuM: [SIMD3<Float>]
    public let normalsEnu: [SIMD3<Float>]
    public let colorsRgba: [SIMD4<UInt8>]
    public let indices: [UInt32]
    public let runways: [RunwayOverlay3D]
}

/// Portrayal style of a 3D airspace volume.
public enum AirspaceClassStyle: UInt8, Equatable, Sendable {
    case classB = 0
    case classC = 1
    case classD = 2
    case classE = 3
    case suaActive = 4
    case suaInactive = 5
    case tfr = 6
}

/// Ownship airspace alert tier.
public enum AirspaceAlertLevel: UInt8, Comparable, Equatable, Sendable {
    case clear = 0
    case proximity = 1
    case insideControlled = 2
    case restrictedOrTfrPenetration = 3

    public static func < (lhs: Self, rhs: Self) -> Bool {
        lhs.rawValue < rhs.rawValue
    }
}

/// Extruded 3D airspace curtain and perimeter rims in ownship-centered local ENU metres.
public struct AirspaceVolumeMesh: Equatable, Identifiable, Sendable {
    public var id: String { airspaceKey }
    public let airspaceKey: String
    public let label: String
    public let style: AirspaceClassStyle
    public let active: Bool
    public let ownshipInside: Bool
    public let rgb: SIMD3<Float>
    public let labelAnchorEnuM: SIMD3<Float>
    public let wallPositionsEnuM: [SIMD3<Float>]
    public let wallIndices: [UInt32]
    public let rimSegmentsEnuM: [(SIMD3<Float>, SIMD3<Float>)]

    public static func == (lhs: Self, rhs: Self) -> Bool {
        lhs.airspaceKey == rhs.airspaceKey
            && lhs.label == rhs.label
            && lhs.style == rhs.style
            && lhs.active == rhs.active
            && lhs.ownshipInside == rhs.ownshipInside
            && lhs.wallPositionsEnuM.count == rhs.wallPositionsEnuM.count
    }
}

/// ADS-B traffic threat level following TCAS II / GDL-90 conventions.
public enum TrafficThreatLevel: UInt8, Comparable, Equatable, Sendable {
    case other = 0
    case proximate = 1
    case trafficAdvisory = 2
    case resolutionAdvisory = 3

    public static func < (lhs: Self, rhs: Self) -> Bool {
        lhs.rawValue < rhs.rawValue
    }

    public var badgeText: String {
        switch self {
        case .other: "OTHER"
        case .proximate: "PROX"
        case .trafficAdvisory: "TA"
        case .resolutionAdvisory: "RA"
        }
    }
}

/// Traffic vertical trend indicator.
public enum TrafficVerticalTrend: UInt8, Equatable, Sendable {
    case level = 0
    case climbing = 1
    case descending = 2

    public var arrow: String {
        switch self {
        case .level: "→"
        case .climbing: "↑"
        case .descending: "↓"
        }
    }
}

/// Fused 3D ADS-B / TIS-B traffic target with callsign placard and CPA metrics.
public struct TrafficTarget: Equatable, Identifiable, Sendable {
    public var id: UInt64 { trackId }
    public let trackId: UInt64
    public let addressHex: String
    public let callsign: String
    public let placard: String
    public let offsetEnuM: SIMD3<Float>
    public let velocityEnuMps: SIMD3<Float>
    public let trackTrueDeg: Float?
    public let slantRangeNm: Float
    public let relAltitudeHundredsFt: Int32
    public let verticalTrend: TrafficVerticalTrend
    public let threat: TrafficThreatLevel
    public let onGround: Bool
    public let active: Bool
    public let timeToCpaS: Float
    public let cpaHorizontalM: Float
}

/// `Indicate` HWD, HUD, PFD, HSI, and 3D Terrain Volume command buffers for one frame.
public struct InstrumentSceneSet: Equatable, Sendable {
    public let hwdCompactMode: Bool
    public let hwd3dStrokesEnu: [(SIMD3<Float>, SIMD3<Float>)]
    public let hwdOverlayScene: [UInt8]
    public let hudScene: [UInt8]
    public let pfdScene: [UInt8]
    public let hsiScene: [UInt8]
    public let terrainVolumeScene: [UInt8]

    public static func == (lhs: Self, rhs: Self) -> Bool {
        lhs.hwdCompactMode == rhs.hwdCompactMode
            && lhs.hwd3dStrokesEnu.count == rhs.hwd3dStrokesEnu.count
            && lhs.hwdOverlayScene == rhs.hwdOverlayScene
            && lhs.hudScene == rhs.hudScene
            && lhs.pfdScene == rhs.pfdScene
            && lhs.hsiScene == rhs.hsiScene
            && lhs.terrainVolumeScene == rhs.terrainVolumeScene
    }
}

/// Complete decoded Synthetic Vision System frame snapshot (`FDKF` v1).
public struct FlightDeckFrame: Equatable, Sendable {
    public let generation: UInt32
    public let sourceMode: TelemetrySourceMode
    public let blend: SvsBlendWeights
    public let ownship: OwnshipSnapshot
    public let terrain: TerrainSurfaceMesh
    public let airspaceAlertLevel: AirspaceAlertLevel
    public let airspaceAlertMessage: String
    public let airspaces: [AirspaceVolumeMesh]
    public let traffic: [TrafficTarget]
    public let instruments: InstrumentSceneSet
}

/// Single verified bitmap glyph from `Indicate`'s `PANEL_GLYPHS`.
public struct InstrumentGlyph: Equatable, Sendable {
    public let scalar: UInt32
    public let advance: UInt8
    public let rows: [UInt8]
}

/// Verified `Indicate` glyph pack (`FDKG` v1).
public struct InstrumentGlyphPack: Equatable, Sendable {
    public let cellWidth: UInt8
    public let cellHeight: UInt8
    public let contentHash: [UInt8]
    public let glyphs: [InstrumentGlyph]
}

/// `IndicateAppleDisplay.GlyphAtlas` backed by the verified Rust `PANEL_GLYPHS` pack.
public struct InstrumentGlyphAtlas: GlyphAtlas, Sendable {
    public let cellWidth: Int
    public let cellHeight: Int
    public let advance: Int
    public let contentHashHex: String
    private let table: [UInt32: [UInt8]]

    public init(pack: InstrumentGlyphPack) {
        self.cellWidth = Int(pack.cellWidth)
        self.cellHeight = Int(pack.cellHeight)
        self.advance = Int(pack.glyphs.first?.advance ?? 6)
        self.table = Dictionary(
            pack.glyphs.map { ($0.scalar, $0.rows) },
            uniquingKeysWith: { first, _ in first }
        )
        self.contentHashHex = pack.contentHash.map { String(format: "%02x", $0) }.joined()
    }

    public func rows(for scalar: UInt32) -> [UInt8]? {
        table[scalar]
    }
}

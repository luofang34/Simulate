import Foundation
import FlightDeckKit

/// Top-level viewport mode separating exocentric situational `Map` navigation from egocentric `SVS`.
enum PadPrimaryViewMode: String, CaseIterable, Identifiable {
    /// Situational 2D/3D EFB & GCS moving map / planetary globe.
    case map = "Map"
    /// Egocentric 6-DoF Synthetic Vision camera (`FPV` boresight or `Chase` external trail).
    case svs = "SVS"

    var id: String { rawValue }
}

/// Camera mount inside `SVS` mode (`FPV` cockpit boresight vs. `Chase` external trail).
enum SvsSubMode: String, CaseIterable, Identifiable {
    /// First-person cockpit boresight camera (`62°` vFoV, supports conformal `HUD` overlay).
    case fpv = "FPV"
    /// External trailing camera (`220 m` aft by default; supports touch orbit & pinch distance).
    case chase = "Chase"

    var id: String { rawValue }
}

/// Interactive look / trail offset for `SVS` (`Chase` orbit or `FPV` head-look when `HUD` is off).
struct SvsLookOffset: Equatable {
    var yawDeg: Double = 0.0
    var pitchDeg: Double = 0.0
    var chaseDistanceMeters: Double = 220.0

    var isNonDefault: Bool {
        abs(yawDeg) > 0.5 || abs(pitchDeg) > 0.5 || abs(chaseDistanceMeters - 220.0) > 4.0
    }

    mutating func reset() {
        yawDeg = 0.0
        pitchDeg = 0.0
        chaseDistanceMeters = 220.0
    }
}

/// Map follow and heading-orientation lock matching `Pilotage` iPadOS (`Free`, `North Up`, `Track Up`).
enum MapFollowMode: String, CaseIterable, Identifiable {
    /// Free pan/zoom/rotate (`northLocked` still holds bearing at `000°T` during 1-finger pan when set).
    case free = "Free"
    /// Centered on ownship with true north (`000°T`) locked at the top of the display.
    case northUp = "North Up"
    /// Centered on ownship with aircraft ground track / true heading locked at the top of the display.
    case trackUp = "Track Up"

    var id: String { rawValue }

    var symbolName: String {
        switch self {
        case .free: "location"
        case .northUp: "location.fill"
        case .trackUp: "location.north.line.fill"
        }
    }

    var shortTag: String {
        switch self {
        case .free: "FREE"
        case .northUp: "N-UP"
        case .trackUp: "TRK"
        }
    }

    var nextCycled: MapFollowMode {
        switch self {
        case .free: .northUp
        case .northUp: .trackUp
        case .trackUp: .free
        }
    }
}

/// Physical planet base surface on `WGS-84` Earth (orthogonal to aeronautical paper/enroute charts).
enum EarthBaseSurface: String, CaseIterable, Identifiable {
    /// Shaded topographic relief with DEM elevation hillshade and coastline/country context.
    case terrain = "Terrain"
    /// Global high-resolution satellite / aerial orthoimagery.
    case satellite = "Satellite"

    var id: String { rawValue }

    var symbolName: String {
        switch self {
        case .terrain: "mountain.2.fill"
        case .satellite: "globe.americas.fill"
        }
    }

    var subtitle: String {
        switch self {
        case .terrain: "Shaded Relief & Hillshade"
        case .satellite: "Global Orthoimagery"
        }
    }
}

/// Aeronautical chart overlay draped above `EarthBaseSurface` (`Off`, `US VFR`, `IFR Low`, `IFR High`).
enum EarthAeroChart: String, CaseIterable, Identifiable {
    /// No aeronautical raster chart — shows pure `Terrain` or `Satellite` base surface.
    case none = "Off"
    /// FAA VFR Sectional (`1:500,000`) + automatic VFR Terminal Area Chart (`1:250,000` at `z >= 9.6`).
    case usVfr = "US VFR"
    /// FAA IFR Enroute Low Altitude Charts (`< 18,000 ft MSL`, `IFR_AreaLow`).
    case ifrLow = "IFR Low"
    /// FAA IFR Enroute High Altitude Jet Route Charts (`FL180–FL600`, `IFR_High`).
    case ifrHigh = "IFR High"

    var id: String { rawValue }

    var subtitle: String {
        switch self {
        case .none: "Pure Terrain / Satellite Surface"
        case .usVfr: "Sectional 1:500k + Auto TAC 1:250k"
        case .ifrLow: "Victor / T-Routes < 18,000 ft"
        case .ifrHigh: "Jet / Q-Routes FL180+"
        }
    }

    var symbolName: String {
        switch self {
        case .none: "eye.slash"
        case .usVfr: "map.fill"
        case .ifrLow: "point.topleft.down.curvedto.point.bottomright.up"
        case .ifrHigh: "airplane.departure"
        }
    }

    var isAeronauticalChart: Bool {
        self != .none
    }
}

/// Airspace portrayal emphasis policy inside the `Airspace ›` secondary menu.
enum AirspacePortrayalPolicy: String, CaseIterable, Identifiable {
    /// Automatically matches active aeronautical chart (`VFR` full vs. `IFR` ghosted Class B/C).
    case autoChart = "Auto (By Chart)"
    /// Full VFR portrayal (`Class B/C/D/E` + Special Use Airspace).
    case vfrFull = "VFR Full"
    /// IFR portrayal (`Prohibited/Restricted/Warning` full; `Class B/C` ghosted; `Class D/E` hidden).
    case ifrDeemphasized = "IFR De-emphasized"

    var id: String { rawValue }
}

/// Drill-down secondary submenu pages inside the top-leading Layers popover.
enum LayersSubmenuPage: String, CaseIterable, Identifiable {
    case airspace = "airspace"
    case weather = "weather"
    case terrainSafety = "terrain_safety"
    case trafficRunways = "traffic_runways"

    var id: String { rawValue }

    var title: String {
        switch self {
        case .airspace: "Airspace & Special Use"
        case .weather: "Aviation Weather"
        case .terrainSafety: "Terrain, Lighting & Safety"
        case .trafficRunways: "Runways, Route & Traffic"
        }
    }
}

/// Unified map & SVS layer configuration separating Base Surface, Aeronautical Chart, and Submenu Overlays.
struct PadLayerConfiguration: Equatable {
    // MARK: - Tier 1: Physical Planet Surface (`Terrain` default)
    var baseSurface: EarthBaseSurface = .terrain

    // MARK: - Tier 2: Aeronautical Chart Overlay (`US VFR` default, `0.35...1.0` opacity)
    var aeroChart: EarthAeroChart = .usVfr
    var chartOpacity: Double = 1.0

    // MARK: - Tier 3A: Airspace Overlay & Secondary Submenu
    var airspace3D: Bool = true
    var airspacePolicy: AirspacePortrayalPolicy = .autoChart
    var prohibitedRestrictedWarning: Bool = true
    var moaAndAlert: Bool = true
    /// Rural 700 ft AGL Class E5 transition shelves (`OFF` by default).
    var classE5Transition: Bool = false
    /// Uncontrolled Class G airspace (`OFF` by default).
    var classG: Bool = false
    /// Planned altitude-cap filter stub (`e.g. hide shelves > FL180`).
    var airspaceAltitudeCapStub: Bool = false

    // MARK: - Tier 3B: Weather Secondary Submenu (Planned FIS-B & AviationWeather Feeds)
    var weatherMasterStub: Bool = false
    var nexradRadarStub: Bool = false
    var flightCategoryMetarStub: Bool = false
    var irCloudsStub: Bool = false
    var windsAloftStub: Bool = false
    var airmetSigmetStub: Bool = false

    // MARK: - Tier 3C: Terrain, Lighting & Safety Secondary Submenu
    /// 3D DEM terrain displacement (`ON` by default).
    var terrain3D: Bool = true
    /// Solar twilight / night-side darkening (`OFF` by default so night flights never dim paper charts).
    var solarTerminator: Bool = false
    var liveSwimTfrStub: Bool = false
    var dofObstaclesStub: Bool = false
    var glideAdvisorStub: Bool = false

    // MARK: - Tier 3D: Runways, Route & Traffic Secondary Submenu
    var runwaysAndRoute: Bool = true
    var airportRunways: Bool = true
    var fplRouteOverlay: Bool = true
    var adsbTraffic: Bool = true
    var hideDistantTrafficStub: Bool = false

    /// Whether an aeronautical chart is actively displayed (`aeroChart != .none`).
    var hasActiveAeroChart: Bool {
        aeroChart.isAeronauticalChart
    }

    /// Whether an opaque paper chart is covering the surface (`>= 92%` opacity), suppressing hillshade muddying.
    var isOpaquePaperChart: Bool {
        aeroChart.isAeronauticalChart && chartOpacity >= 0.92
    }

    /// Resolved IFR de-emphasis state combining `airspacePolicy` and `aeroChart`.
    var resolvedAirspaceIsIfr: Bool {
        switch airspacePolicy {
        case .autoChart:
            aeroChart == .ifrLow || aeroChart == .ifrHigh
        case .vfrFull:
            false
        case .ifrDeemphasized:
            true
        }
    }

    /// Concise summary shown on the `Airspace ›` row.
    var airspaceSummaryBadge: String {
        guard airspace3D else { return "Off" }
        if aeroChart == .ifrHigh && airspacePolicy == .autoChart {
            return "IFR High · SUA"
        }
        return resolvedAirspaceIsIfr ? "IFR · SUA + Ghost B/C" : "VFR · B/C/D/E + SUA"
    }

    /// Compact zoom/layer tag shown in the top-trailing map scale readout.
    func scaleReadoutTag(effectiveZoom: Double) -> String {
        switch aeroChart {
        case .usVfr:
            effectiveZoom >= 9.6 ? "TAC" : "SEC"
        case .ifrLow:
            "IFR-L"
        case .ifrHigh:
            "IFR-H"
        case .none:
            baseSurface == .satellite ? "SAT" : "DEM"
        }
    }
}

/// Orbital globe/map camera state supporting whole-globe deep space (`z0.5`) down to runway threshold (`z18`).
struct OrbitCameraState: Equatable {
    var latitude: Double
    var longitude: Double
    var zoom: Double
    var pitchDeg: Double
    var bearingDeg: Double

    static let wholeGlobeDefault = OrbitCameraState(
        latitude: 35.2,
        longitude: -90.0,
        zoom: 1.55,
        pitchDeg: 0.0,
        bearingDeg: 0.0
    )

    static let lunarWholeGlobeDefault = OrbitCameraState(
        latitude: 10.0,
        longitude: 12.0,
        zoom: 1.45,
        pitchDeg: 0.0,
        bearingDeg: 0.0
    )

    static let martianWholeGlobeDefault = OrbitCameraState(
        latitude: 4.0,
        longitude: -75.0,
        zoom: 1.45,
        pitchDeg: 0.0,
        bearingDeg: 0.0
    )
}

/// Curated operational corridor & benchmark camera presets (with mixed Airport, VOR/VORTAC, NDB, and Fix legs).
struct NavigationBookmark: Identifiable {
    let id: String
    let title: String
    let subtitle: String
    let routeCodes: [String]
    let state: OrbitCameraState

    static func forPrimaryBody(_ body: PrimaryCelestialBody) -> [NavigationBookmark] {
        switch body {
        case .earth: earthBookmarks
        case .moon: lunarBookmarks
        case .mars: martianBookmarks
        }
    }

    static let earthBookmarks: [NavigationBookmark] = [
        NavigationBookmark(
            id: "mem_hsv_atl",
            title: "KMEM → MSL → CABAS → KHSV → RMG → KATL",
            subtitle: "Mid-South Airport + VORTAC + RNAV Fix Corridor",
            routeCodes: ["KMEM", "MSL", "CABAS", "KHSV", "RMG", "KATL"],
            state: OrbitCameraState(latitude: 34.35, longitude: -87.25, zoom: 6.85, pitchDeg: 0.0, bearingDeg: 0.0)
        ),
        NavigationBookmark(
            id: "ewr_n51",
            title: "KEWR → SBJ → N51 Solberg",
            subtitle: "Solberg VOR/DME to RWY 22 final approach",
            routeCodes: ["KEWR", "SBJ", "N51"],
            state: OrbitCameraState(latitude: 40.5920, longitude: -74.7270, zoom: 13.8, pitchDeg: 66.0, bearingDeg: 210.0)
        ),
        NavigationBookmark(
            id: "dca_orf",
            title: "KDCA → GTN → AML → KORF",
            subtitle: "P-56A/B Prohibited + GTN NDB & AML VOR/DME",
            routeCodes: ["KDCA", "GTN", "AML", "KORF"],
            state: OrbitCameraState(latitude: 38.8520, longitude: -77.0370, zoom: 10.2, pitchDeg: 58.0, bearingDeg: 335.0)
        ),
        NavigationBookmark(
            id: "ny_tac",
            title: "KJFK → KTEB Hudson SFRA",
            subtitle: "NY 1:250k TAC + Class B 70/13 wedding-cake",
            routeCodes: ["KJFK", "KTEB"],
            state: OrbitCameraState(latitude: 40.6892, longitude: -74.0445, zoom: 10.9, pitchDeg: 60.0, bearingDeg: 18.0)
        ),
        NavigationBookmark(
            id: "sna_lax",
            title: "KSNA → SLI → KLAX",
            subtitle: "SoCal Class B/C + Seal Beach VORTAC",
            routeCodes: ["KSNA", "SLI", "KLAX"],
            state: OrbitCameraState(latitude: 33.6760, longitude: -117.8680, zoom: 10.8, pitchDeg: 56.0, bearingDeg: 320.0)
        ),
        NavigationBookmark(
            id: "cos_den",
            title: "KCOS → FQF → KDEN",
            subtitle: "Pikes Peak 3D DEM + Falcon VORTAC + A-260",
            routeCodes: ["KCOS", "FQF", "KDEN"],
            state: OrbitCameraState(latitude: 38.8800, longitude: -104.7600, zoom: 10.2, pitchDeg: 62.0, bearingDeg: 315.0)
        ),
        NavigationBookmark(
            id: "earth_globe",
            title: "Whole Globe Overview",
            subtitle: "Deep-space Johnson-V starfield & atmospheric limb",
            routeCodes: [],
            state: .wholeGlobeDefault
        ),
    ]

    static let lunarBookmarks: [NavigationBookmark] = [
        NavigationBookmark(
            id: "moon_globe",
            title: "Lunar Nearside Overview",
            subtitle: "IAU R=1,737.4 km • Vacuum starfield",
            routeCodes: [],
            state: .lunarWholeGlobeDefault
        ),
        NavigationBookmark(
            id: "tranquillitatis",
            title: "APOLLO11 → TYCHO",
            subtitle: "Mare Tranquillitatis to Southern Highlands",
            routeCodes: ["APOLLO11", "TYCHO"],
            state: OrbitCameraState(latitude: 0.674, longitude: 23.473, zoom: 5.2, pitchDeg: 42.0, bearingDeg: 285.0)
        ),
    ]

    static let martianBookmarks: [NavigationBookmark] = [
        NavigationBookmark(
            id: "mars_globe",
            title: "Mars Tharsis Hemisphere",
            subtitle: "IAU Spheroid a=3,396.19 km",
            routeCodes: [],
            state: .martianWholeGlobeDefault
        ),
        NavigationBookmark(
            id: "jezero",
            title: "JEZERO → GALE",
            subtitle: "Isidis Rim to Aeolis Mons",
            routeCodes: ["JEZERO", "GALE"],
            state: OrbitCameraState(latitude: 18.44, longitude: 77.45, zoom: 5.0, pitchDeg: 44.0, bearingDeg: 310.0)
        ),
    ]
}

import Foundation
import simd

/// Surveyed airport metadata resolved from `AirportDatabase` on `WGS-84` Earth.
public struct ResolvedAirport: Equatable, Sendable {
    public let code: String
    public let name: String
    public let municipality: String
    public let latitudeDeg: Double
    public let longitudeDeg: Double
    public let elevationDatumM: Double
    public let towered: Bool
    public let longestRunwayM: Double
    public let runways: [String]

    public init(
        code: String,
        name: String,
        municipality: String,
        latitudeDeg: Double,
        longitudeDeg: Double,
        elevationDatumM: Double,
        towered: Bool,
        longestRunwayM: Double,
        runways: [String]
    ) {
        self.code = code
        self.name = name
        self.municipality = municipality
        self.latitudeDeg = latitudeDeg
        self.longitudeDeg = longitudeDeg
        self.elevationDatumM = elevationDatumM
        self.towered = towered
        self.longestRunwayM = longestRunwayM
        self.runways = runways
    }
}

/// Ordered typed waypoint token in the EFB / GCS flight plan (`FPL`).
public struct FlightPlanWaypoint: Equatable, Identifiable, Sendable {
    public let id: UUID
    public var pointKey: String
    public var ident: String
    public var kind: WaypointKind
    public var name: String
    public var region: String
    public var latitudeDeg: Double
    public var longitudeDeg: Double
    public var altitudeDatumM: Double
    public var runways: [String]
    /// Optional airway or CIFP procedure tag (`"V3"`, `"SID PORTT4"`, `"STAR LVZ5"`, `"RNAV 22"`).
    public var airwayOrProcedureTag: String
    /// Optional CIFP / MEA altitude & fix role tag (`"+3000 FT · IAF"`, `"+1800 FT · FAF"`, `"MEA 3000"`).
    public var altitudeConstraintTag: String

    public init(
        id: UUID = UUID(),
        pointKey: String? = nil,
        ident: String,
        kind: WaypointKind = .airport,
        name: String,
        region: String = "",
        latitudeDeg: Double,
        longitudeDeg: Double,
        altitudeDatumM: Double = 0.0,
        runways: [String] = [],
        airwayOrProcedureTag: String = "",
        altitudeConstraintTag: String = ""
    ) {
        self.id = id
        self.pointKey = pointKey ?? "\(kind.rawValue):\(ident)"
        self.ident = ident
        self.kind = kind
        self.name = name
        self.region = region
        self.latitudeDeg = latitudeDeg
        self.longitudeDeg = longitudeDeg
        self.altitudeDatumM = altitudeDatumM
        self.runways = runways
        self.airwayOrProcedureTag = airwayOrProcedureTag
        self.altitudeConstraintTag = altitudeConstraintTag
    }

    public init(
        id: UUID = UUID(),
        point: NavigationPoint,
        airwayOrProcedureTag: String = "",
        altitudeConstraintTag: String = ""
    ) {
        self.id = id
        self.pointKey = point.key
        self.ident = point.identifier
        self.kind = point.kind
        self.name = point.name
        self.region = point.region
        self.latitudeDeg = point.latitudeDeg
        self.longitudeDeg = point.longitudeDeg
        self.altitudeDatumM = point.elevationDatumM
        self.runways = point.runways
        self.airwayOrProcedureTag = airwayOrProcedureTag
        self.altitudeConstraintTag = altitudeConstraintTag
    }
}

/// Great-circle leg summary between two planetographic coordinates on `radiusMeters`.
public struct RouteLegMetrics: Equatable, Sendable {
    public let distanceMeters: Double
    public let initialBearingTrueDeg: Double

    public var distanceNauticalMiles: Double {
        distanceMeters / 1_852.0
    }

    public static func between(
        lat1Deg: Double,
        lon1Deg: Double,
        lat2Deg: Double,
        lon2Deg: Double,
        radiusMeters: Double
    ) -> RouteLegMetrics {
        let phi1 = lat1Deg * .pi / 180.0
        let phi2 = lat2Deg * .pi / 180.0
        let dPhi = (lat2Deg - lat1Deg) * .pi / 180.0
        let dLam = (lon2Deg - lon1Deg) * .pi / 180.0
        let sinHalfPhi = sin(dPhi * 0.5)
        let sinHalfLam = sin(dLam * 0.5)
        let a = min(
            max(sinHalfPhi * sinHalfPhi + cos(phi1) * cos(phi2) * sinHalfLam * sinHalfLam, 0.0),
            1.0
        )
        let central = 2.0 * atan2(sqrt(a), sqrt(max(1.0 - a, 0.0)))
        let y = sin(dLam) * cos(phi2)
        let x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dLam)
        var brg = atan2(y, x) * 180.0 / .pi
        if brg < 0.0 {
            brg += 360.0
        }
        return RouteLegMetrics(
            distanceMeters: max(radiusMeters, 1.0) * central,
            initialBearingTrueDeg: brg
        )
    }
}

/// Avionics / ADS-B / GCS receiver hardware & link family (unifying EFB receivers with `Sokoly-App` GCS links).
public enum AvionicsReceiverFamily: String, CaseIterable, Identifiable, Sendable {
    /// Wi-Fi portable ADS-B / AHRS receiver streaming GDL-90 on UDP `:4000` (Sentry, Stratux, Garmin GDL).
    case wifiGdl90 = "Wi-Fi GDL-90"
    /// Direct USB-C dual-band `1090ES` / `978 UAT` SDR or serial receiver (DriverKit / ExternalAccessory stub).
    case usbAdsBStub = "USB ADS-B"
    /// Bidirectional `Sokoly` / `avionics-link` Connext TCP telemetry & GCS command link (`:5760`).
    case sokolyGcsTcp = "Sokoly GCS / Connext"
    /// Deterministic 6-DoF flight & recorded Connext capture profiles for validation and training.
    case simulationBench = "Simulation"

    public var id: String { rawValue }

    public var badgeLabel: String {
        switch self {
        case .wifiGdl90: "ADS-B :4000"
        case .usbAdsBStub: "USB SDR · STUB"
        case .sokolyGcsTcp: "GCS :5760"
        case .simulationBench: "SIM"
        }
    }

    public var symbolName: String {
        switch self {
        case .wifiGdl90: "wifi.circle.fill"
        case .usbAdsBStub: "cable.connector.horizontal"
        case .sokolyGcsTcp: "antenna.radiowaves.left.and.right.circle.fill"
        case .simulationBench: "airplane.circle.fill"
        }
    }

    /// Whether this link family carries a command uplink capable of holding a `Sokoly` GCS control lease.
    public var supportsControlUplink: Bool {
        switch self {
        case .sokolyGcsTcp, .simulationBench: true
        case .wifiGdl90, .usbAdsBStub: false
        }
    }
}

/// Operator authority state matching `Sokoly-App`'s `ControlBarChrome` (`Observing` vs. `Control Lease`, `SAFE` vs. `ARM`).
public struct GcsControlState: Equatable, Sendable {
    public var receiverFamily: AvionicsReceiverFamily
    public var hasControlLease: Bool
    public var armTelegraphArmed: Bool
    public var activeGuidanceMode: String
    public var lastCommandNote: String

    public init(
        receiverFamily: AvionicsReceiverFamily = .simulationBench,
        hasControlLease: Bool = false,
        armTelegraphArmed: Bool = false,
        activeGuidanceMode: String = "NAV",
        lastCommandNote: String = "Observing telemetry (EFB passive)"
    ) {
        self.receiverFamily = receiverFamily
        self.hasControlLease = hasControlLease
        self.armTelegraphArmed = armTelegraphArmed
        self.activeGuidanceMode = activeGuidanceMode
        self.lastCommandNote = lastCommandNote
    }

    /// Honest command gating (`Sokoly-App` `rules::can_do` pattern): passive ADS-B / KML replay never fakes actuation.
    public func canActuate(isKmlPreview: Bool) -> Bool {
        guard !isKmlPreview else { return false }
        return receiverFamily.supportsControlUplink && hasControlLease
    }

    public func gatingReason(isKmlPreview: Bool) -> String? {
        if isKmlPreview {
            return "Read-only KML track preview — close preview to issue GCS commands."
        }
        if !receiverFamily.supportsControlUplink {
            return "\(receiverFamily.rawValue) is a receive-only surveillance link — switch to Sokoly GCS TCP or Simulation."
        }
        if !hasControlLease {
            return "Observing only — acquire Control Lease before issuing vehicle commands."
        }
        return nil
    }
}

/// Pilot-focused compact navigation & performance summary displayed in the minimized bottom `FPL` bar.
public struct CompactFlightPlanSummary: Equatable, Sendable {
    public let hasRoute: Bool
    /// Departure / first waypoint identifier when distinct from `currentIdent` (`"KEWR"`).
    public let originIdent: String?
    /// Whether `originIdent` and `currentIdent` are separated by already-sequenced intermediate waypoints.
    public let hasSequencedGapAfterOrigin: Bool
    /// Active `TO` waypoint currently being navigated toward (`"PIYIN"`, `"SBJ"`, `"N51"`).
    public let currentIdent: String?
    /// Immediate next waypoint after `currentIdent` (`"DODLY"`, `"N51"`), or `nil` on the final leg.
    public let nextIdent: String?
    /// Final destination identifier when distinct from both `currentIdent` and `nextIdent` (`"N51"`).
    public let destinationIdent: String?
    /// Selected destination runway badge (`"22"`, `"22L"`), Earth only.
    public let runwayBadge: String?
    /// True course / bearing (`0..360°`) from ownship (or prior fix) to `currentIdent`.
    public let bearingTrueDeg: Double?
    /// Great-circle distance (`NM`) to `currentIdent`.
    public let legDistanceNm: Double?
    /// Total remaining route distance (`NM`) from ownship (or `origin`) through `currentIdent` to final destination.
    public let remainingDistanceNm: Double?
    /// Whether `remainingDistanceNm` spans additional legs beyond `currentIdent`.
    public let hasRemainingLegsBeyondCurrent: Bool
    /// Estimated time enroute to final destination (`"0:09"`, `"1:24"`).
    public let eteFormatted: String?
    /// Estimated fuel required to final destination in US gallons (`Earth` only; `nil` off-world).
    public let fuelRequiredGal: Double?
    /// Whether `eteFormatted` & `fuelRequiredGal` use live `GS` (`true`) or planned cruise `KTAS` (`false`).
    public let usesLiveGroundSpeed: Bool

    public init(
        hasRoute: Bool,
        originIdent: String? = nil,
        hasSequencedGapAfterOrigin: Bool = false,
        currentIdent: String? = nil,
        nextIdent: String? = nil,
        destinationIdent: String? = nil,
        runwayBadge: String? = nil,
        bearingTrueDeg: Double? = nil,
        legDistanceNm: Double? = nil,
        remainingDistanceNm: Double? = nil,
        hasRemainingLegsBeyondCurrent: Bool = false,
        eteFormatted: String? = nil,
        fuelRequiredGal: Double? = nil,
        usesLiveGroundSpeed: Bool = false
    ) {
        self.hasRoute = hasRoute
        self.originIdent = originIdent
        self.hasSequencedGapAfterOrigin = hasSequencedGapAfterOrigin
        self.currentIdent = currentIdent
        self.nextIdent = nextIdent
        self.destinationIdent = destinationIdent
        self.runwayBadge = runwayBadge
        self.bearingTrueDeg = bearingTrueDeg
        self.legDistanceNm = legDistanceNm
        self.remainingDistanceNm = remainingDistanceNm
        self.hasRemainingLegsBeyondCurrent = hasRemainingLegsBeyondCurrent
        self.eteFormatted = eteFormatted
        self.fuelRequiredGal = fuelRequiredGal
        self.usesLiveGroundSpeed = usesLiveGroundSpeed
    }
}

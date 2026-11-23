import CoreGraphics
import Foundation
import IndicateAppleDisplay
import Observation
import QuartzCore
import simd
import flightdeck_ffiFFI

/// Thread-safe scene buffer mailbox feeding `IndicateAppleDisplay` background render workers.
final class SceneMailbox: @unchecked Sendable {
    private let lock = NSLock()
    private var generation: UInt32 = 0
    private var pfd: [UInt8] = []
    private var hsi: [UInt8] = []
    private var hud: [UInt8] = []
    private var hwdOverlay: [UInt8] = []
    private var terrainVolume: [UInt8] = []

    func publish(scenes: InstrumentSceneSet, generation: UInt32) {
        lock.lock()
        self.generation = generation
        self.pfd = scenes.pfdScene
        self.hsi = scenes.hsiScene
        self.hud = scenes.hudScene
        self.hwdOverlay = scenes.hwdOverlayScene
        self.terrainVolume = scenes.terrainVolumeScene
        lock.unlock()
    }

    func pfdFrame() -> SceneFrame {
        lock.lock()
        defer { lock.unlock() }
        return SceneFrame(bytes: pfd, generation: generation)
    }

    func hsiFrame() -> SceneFrame {
        lock.lock()
        defer { lock.unlock() }
        return SceneFrame(bytes: hsi, generation: generation)
    }

    func hudFrame() -> SceneFrame {
        lock.lock()
        defer { lock.unlock() }
        return SceneFrame(bytes: hud, generation: generation)
    }

    func hwdOverlayFrame() -> SceneFrame {
        lock.lock()
        defer { lock.unlock() }
        return SceneFrame(bytes: hwdOverlay, generation: generation)
    }

    func terrainVolumeFrame() -> VolumeFrame {
        lock.lock()
        defer { lock.unlock() }
        return VolumeFrame(bytes: terrainVolume, generation: generation)
    }
}

/// Thread-safe lock wrapper around the Rust `FlightDeckSession` pointer so `PanelDisplay` background
/// workers can re-emit the conformal HUD at the exact requested `designFrame` without data races.
final class FfiSessionHandle: @unchecked Sendable {
    private let lock = NSLock()
    private let ptr: OpaquePointer?
    private var generation: UInt32 = 1

    init(ptr: OpaquePointer?) {
        self.ptr = ptr
    }

    deinit {
        if let ptr {
            flightdeck_session_free(ptr)
        }
    }

    func withSession<R>(_ body: (OpaquePointer) -> R) -> R? {
        lock.lock()
        defer { lock.unlock() }
        guard let ptr else { return nil }
        return body(ptr)
    }

    func setGeneration(_ gen: UInt32) {
        lock.lock()
        generation = max(gen, 1)
        lock.unlock()
    }

    func emitHudFrame(designFrame: CGRect, fallback: SceneFrame) -> SceneFrame {
        lock.lock()
        defer { lock.unlock() }
        guard let ptr else { return fallback }
        let w = Float(min(max(designFrame.width, 390.0), 2560.0))
        let h = Float(min(max(designFrame.height, 390.0), 1600.0))
        let buf = flightdeck_session_emit_hud(ptr, w, h)
        defer { flightdeck_byte_buffer_free(buf) }
        guard let base = buf.ptr, buf.len > 0 else { return fallback }
        let bytes = Array(UnsafeBufferPointer(start: base, count: buf.len))
        return SceneFrame(bytes: bytes, generation: max(generation, fallback.generation))
    }
}

/// Main-actor observable state & FFI session coordinator for the iPadOS EFB/GCS & Vision Pro flight-deck apps.
@MainActor
@Observable
public final class FlightDeckController {
    /// Explicitly declared celestial body and vertical datum (`WGS-84 / EGM96`, `WGS-84 HAE`, `Moon IAU`, `Mars IAU`).
    public private(set) var celestialBody: CelestialBody
    /// Physical ellipsoid, render radius, and gravity queried from `sim-geo` for `celestialBody`.
    public private(set) var bodyParameters: CelestialBodyParameters

    /// Active telemetry feed (`Scenario`, `ConnextReplay`, `ConnextLiveTcp`, `Gdl90LiveUdp`, or `KmlReplay`).
    public var sourceMode: TelemetrySourceMode = .njCorridorCruise {
        didSet {
            guard sourceMode != oldValue else { return }
            applySourceMode(sourceMode)
        }
    }

    /// Digital Crown virtual/reality blend (`0.0` = 100% real outside passthrough;
    /// `1.0` = 85% synthetic terrain + **15% real outside passthrough floor**).
    public var crownTurn: Float = 0.55 {
        didSet {
            let clamped = crownTurn.isFinite ? min(max(crownTurn, 0.0), 1.0) : 0.0
            if clamped != crownTurn {
                crownTurn = clamped
                return
            }
            _ = sessionHandle.withSession { flightdeck_session_set_crown_turn($0, clamped) }
        }
    }

    /// Currently designated ADS-B `trackId` for HUD target lock cues (`nil` = automatic TA/RA).
    public var designatedTrackId: UInt64? {
        didSet {
            let hasTrack = designatedTrackId != nil
            let trackVal = designatedTrackId ?? 0
            _ = sessionHandle.withSession {
                flightdeck_session_set_designated_track($0, hasTrack, trackVal)
            }
        }
    }

    /// Hostname or IP of an `avionics-link` Connext TCP bridge.
    public var connextHost: String = "127.0.0.1"
    /// TCP port of an `avionics-link` Connext TCP bridge.
    public var connextPort: UInt16 = 5760
    /// UDP port for portable GDL-90 ADS-B/AHRS receivers (`4000` standard).
    public var gdl90Port: UInt16 = 4000

    /// Live transport status text.
    public private(set) var transportStatus: String = "Ready"
    /// Whether the Connext TCP client is connected.
    public private(set) var isConnextConnected: Bool = false
    /// Whether the GDL-90 UDP listener is active.
    public private(set) var isGdl90Listening: Bool = false

    /// Latest decoded `FDKF` frame snapshot.
    public private(set) var latestFrame: FlightDeckFrame?
    /// Active KML / GPX preview state and track polyline.
    public private(set) var kmlPlayback: TrackPlaybackState = TrackPlaybackState()
    /// Active EFB / GCS flight plan waypoints (`FPL`).
    public var flightPlanWaypoints: [FlightPlanWaypoint] = []
    /// Index of the active Direct-To / next waypoint in `flightPlanWaypoints`.
    public var activeWaypointIndex: Int = 1
    /// Selected destination runway designator (`"22"`, `"04"`, `"18L"`).
    public var destinationRunway: String?
    /// Planned cruise true airspeed (`KTAS`) used for `ETE`/`FUEL` when live `GS` is below `25 kt`.
    public var plannedCruiseKtas: Double = 125.0
    /// Planned aircraft fuel burn (`US gal/hr`) used for `FUEL` estimates on Earth.
    public var plannedFuelFlowGph: Double = 9.5
    /// Multi-provider terminal procedure & chart service registry (`FAA AeroNav` active + `Jeppesen`/`Lido`/`EUROCONTROL`/`Navigraph` stubs).
    public var chartProviders: ChartProviderRegistry = ChartProviderRegistry()
    /// Active georeferenced procedure plate draped onto the 2D/3D moving map (`nil` when none pinned).
    public var pinnedMapPlate: PinnedMapPlate?
    /// Whether `pinnedMapPlate` is currently visible on the moving map.
    public var pinnedPlateVisible: Bool = true {
        didSet {
            guard pinnedPlateVisible != oldValue else { return }
            plateOverlayRevision &+= 1
        }
    }
    /// Opacity (`0.25...1.0`) of `pinnedMapPlate` on the moving map.
    public var pinnedPlateOpacity: Double = 0.84 {
        didSet {
            let clamped = min(max(pinnedPlateOpacity, 0.25), 1.0)
            if clamped != pinnedPlateOpacity {
                pinnedPlateOpacity = clamped
                return
            }
            guard abs(pinnedPlateOpacity - oldValue) > 0.001 else { return }
            plateOverlayRevision &+= 1
        }
    }
    /// Whether `pinnedMapPlate` renders in dark-cockpit color-inverted mode on the moving map.
    public var pinnedPlateInverted: Bool = false {
        didSet {
            guard pinnedPlateInverted != oldValue else { return }
            refreshPinnedPlateForColorMode()
        }
    }
    /// `Sokoly-App`-aligned receiver family & GCS control-lease state.
    public var gcsState: GcsControlState = GcsControlState()
    /// Offline aeronautical chart, 3D airspace, and NASR cycle store (`v99n62` + `Flightbag`).
    public let dataPackStore: AeroDataPackStore
    /// Monotonic revision bumped when typed `FPL` waypoints, active leg, or KML route polylines change.
    public private(set) var routeOverlayRevision: UInt64 = 1
    /// Monotonic revision bumped when airport runway geometry, destination runway, or celestial body changes.
    public private(set) var runwayOverlayRevision: UInt64 = 1
    /// Monotonic revision bumped when `pinnedMapPlate`, its visibility, or its opacity changes.
    public private(set) var plateOverlayRevision: UInt64 = 1

    /// Primary celestial body (`Earth`, `Moon`, `Mars`) without duplicating vertical datums.
    public var primaryBody: PrimaryCelestialBody {
        celestialBody.primaryBody
    }

    /// Active vertical height datum when `primaryBody == .earth`.
    public var earthVerticalDatum: EarthVerticalDatum {
        celestialBody.earthVerticalDatum
    }

    /// Whether an AirDrop / Open-In KML or GPX track preview is currently active.
    public var isPreviewingTrack: Bool {
        sourceMode == .kmlReplay && kmlPlayback.hasTrack
    }

    /// Verified `Indicate` bitmap font atlas shared across all 2D displays.
    public let glyphAtlas: InstrumentGlyphAtlas
    /// Persistent `IndicateAppleDisplay` Primary Flight Display (`480 x 360`).
    public let pfdDisplay: PanelDisplay
    /// Persistent `IndicateAppleDisplay` Horizontal Situation Indicator (`480 x 360`).
    public let hsiDisplay: PanelDisplay
    /// Persistent `IndicateAppleDisplay` conformal 2D HUD (pure transparent overlay, dynamic `designFrame`).
    public let hudDisplay: PanelDisplay
    /// Persistent `IndicateAppleDisplay` 2D HWD peripheral/glance overlay (`1200 x 600`, pure transparent).
    public let hwdOverlayDisplay: PanelDisplay
    /// Persistent `IndicateAppleDisplay` 3D Volumetric Terrain display (`terrain-volume`).
    public let terrainVolumeDisplay: VolumeDisplay

    @ObservationIgnored let sessionHandle: FfiSessionHandle
    @ObservationIgnored private let mailbox = SceneMailbox()
    @ObservationIgnored private var frameGeneration: UInt32 = 0
    @ObservationIgnored private var lastHudFovRad: Float = -1.0
    @ObservationIgnored var lastAirportRouteSignature: String = ""
    @ObservationIgnored nonisolated(unsafe) private var connextClient: ConnextTcpClient?
    @ObservationIgnored nonisolated(unsafe) private var gdl90Listener: Gdl90UdpListener?

    /// Creates a controller explicitly bound to `body` (no implicit Earth or `WGS-84` default).
    public init(body: CelestialBody, dataPackStore: AeroDataPackStore? = nil) {
        let sessionPtr = flightdeck_session_new_on(body.rawValue)
        let handle = FfiSessionHandle(ptr: sessionPtr)
        self.sessionHandle = handle
        self.celestialBody = body
        self.bodyParameters = Self.queryBodyParameters(handle: handle, fallback: body)
        self.dataPackStore = dataPackStore ?? AeroDataPackStore()

        let pack = Self.loadVerifiedGlyphPack()
        let atlas = InstrumentGlyphAtlas(pack: pack)
        self.glyphAtlas = atlas

        let box = self.mailbox
        let panelSize = CGSize(width: 480, height: 360)
        // Match `Indicate`'s `HUD_SCREEN_DESCRIPTOR` (`390x390` .. `2560x1600`) 1:1 so `PanelDisplay`
        // passes the exact viewport `designFrame` into `emitHudFrame` with zero aspect/offset skew.
        let hudMinSize = CGSize(width: 390, height: 390)
        let hudCanonicalSize = CGSize(width: 1280, height: 960)
        let hudMaxSize = CGSize(width: 2560, height: 1600)
        let hwdSize = CGSize(width: 1200, height: 600)

        let pfdReq = PanelRequirements(
            id: "flightdeck.pfd",
            title: "Primary Flight Display",
            criticalLayers: [],
            frameMin: panelSize,
            frameMax: panelSize,
            canonicalFrame: panelSize,
            preferredFramesPerSecond: 30
        )
        let hsiReq = PanelRequirements(
            id: "flightdeck.hsi",
            title: "Horizontal Situation Indicator",
            criticalLayers: [],
            frameMin: panelSize,
            frameMax: panelSize,
            canonicalFrame: panelSize,
            preferredFramesPerSecond: 30
        )
        let hudReq = PanelRequirements(
            id: "flightdeck.hud",
            title: "Conformal HUD",
            criticalLayers: [],
            frameMin: hudMinSize,
            frameMax: hudMaxSize,
            canonicalFrame: hudCanonicalSize,
            preferredFramesPerSecond: 60
        )
        let hwdReq = PanelRequirements(
            id: "flightdeck.hwd",
            title: "HWD Glance Symbology",
            criticalLayers: [],
            frameMin: hwdSize,
            frameMax: hwdSize,
            canonicalFrame: hwdSize,
            preferredFramesPerSecond: 60
        )
        let volReq = VolumeRequirements(
            id: "flightdeck.terrain.volume",
            title: "3D Terrain Situation Volume",
            requiredBands: [],
            extentMin: Extent3D(widthMeters: 0.20, heightMeters: 0.14, depthMeters: 0.20),
            extentMax: Extent3D(widthMeters: 1.20, heightMeters: 0.90, depthMeters: 1.20),
            canonicalExtent: Extent3D(widthMeters: 0.42, heightMeters: 0.28, depthMeters: 0.42),
            underlay: .volumetricTerrain3D,
            preferredFramesPerSecond: 30
        )

        let nowMs = CACurrentMediaTime() * 1000.0
        self.pfdDisplay = PanelDisplay(
            requirements: pfdReq,
            producer: ClosureSceneProducer { _ in box.pfdFrame() },
            atlas: atlas,
            background: CGColor(red: 0.04, green: 0.06, blue: 0.09, alpha: 0.94),
            nowMs: nowMs
        )
        self.hsiDisplay = PanelDisplay(
            requirements: hsiReq,
            producer: ClosureSceneProducer { _ in box.hsiFrame() },
            atlas: atlas,
            background: CGColor(red: 0.04, green: 0.06, blue: 0.09, alpha: 0.94),
            nowMs: nowMs
        )
        // Pure transparent background (`nil`) + dynamic `designFrame` emission so HUD boresight is dead-center.
        self.hudDisplay = PanelDisplay(
            requirements: hudReq,
            producer: ClosureSceneProducer { designFrame in
                handle.emitHudFrame(designFrame: designFrame, fallback: box.hudFrame())
            },
            atlas: atlas,
            background: nil,
            nowMs: nowMs
        )
        self.hwdOverlayDisplay = PanelDisplay(
            requirements: hwdReq,
            producer: ClosureSceneProducer { _ in box.hwdOverlayFrame() },
            atlas: atlas,
            background: nil,
            nowMs: nowMs
        )
        self.terrainVolumeDisplay = VolumeDisplay(
            requirements: volReq,
            producer: ClosureVolumeProducer { _, _ in box.terrainVolumeFrame() },
            nowMs: nowMs
        )

        let initMode = sourceMode.rawValue
        let initCrown = crownTurn
        _ = handle.withSession { session in
            flightdeck_session_set_source_mode(session, initMode)
            flightdeck_session_set_crown_turn(session, initCrown)
        }
        setupTransports()
        if body.isEarthWgs84 {
            setFlightPlanCodes(["KEWR", "N51"], destinationRunway: "22")
        }
        _ = stepFrame(nowMs: UInt64(max(nowMs, 0)))
    }

    deinit {
        connextClient?.disconnect()
        gdl90Listener?.stop()
    }

    /// Switches the primary celestial body (`Earth`, `Moon`, `Mars`), preserving Earth's active vertical datum.
    public func setPrimaryBody(_ primary: PrimaryCelestialBody) {
        let resolved = CelestialBody.resolve(primary: primary, earthVerticalDatum: earthVerticalDatum)
        setCelestialBody(resolved)
    }

    /// Switches Earth's vertical datum (`MSL` vs. `HAE`) while keeping `Earth (WGS-84)` active.
    public func setEarthVerticalDatum(_ datum: EarthVerticalDatum) {
        guard celestialBody.isEarthWgs84 else { return }
        setCelestialBody(CelestialBody.resolve(primary: .earth, earthVerticalDatum: datum))
    }

    /// Explicitly re-declares the active celestial body and vertical datum.
    public func setCelestialBody(_ body: CelestialBody) {
        guard body != celestialBody else { return }
        let wasEarth = celestialBody.isEarthWgs84
        let nowMs = UInt64(max(CACurrentMediaTime() * 1000.0, 0))
        let ok = sessionHandle.withSession { flightdeck_session_set_body($0, body.rawValue, nowMs) } ?? false
        guard ok else { return }
        celestialBody = body
        bodyParameters = Self.queryBodyParameters(handle: sessionHandle, fallback: body)
        if !body.isEarthWgs84 {
            clearPinnedMapPlate()
            clearFlightPlan()
        } else if !wasEarth && flightPlanWaypoints.isEmpty {
            setFlightPlanCodes(["KEWR", "N51"], destinationRunway: "22")
        } else {
            syncFlightPlanToSession()
        }
        routeOverlayRevision &+= 1
        runwayOverlayRevision &+= 1
        resetDisplays()
        if sourceMode == .kmlReplay {
            refreshKmlTrackSummary()
            refreshKmlStatus()
        }
        _ = stepFrame(nowMs: nowMs)
        transportStatus = "\(body.badgeLabel) • \(body.geodesySummary)"
    }

    /// Updates the conformal HUD vertical field of view (`radians`) to match the inset viewport over the 3D scene.
    public func setHudFieldOfView(radians: Float) {
        guard radians.isFinite, abs(radians - lastHudFovRad) > 0.002 else { return }
        lastHudFovRad = radians
        _ = sessionHandle.withSession { flightdeck_session_set_hud_fov($0, radians) }
    }

    /// Synchronizes the exact outside-scene camera pose (`FPV` or `Chase`; `active: false` in `Globe` orbit)
    /// and optional onboard DEM sample (`m` above datum) so HUD horizon, velocity vector, and runway align 1:1.
    public func setCameraViewPose(
        active: Bool,
        latitudeDeg: Double = 0.0,
        longitudeDeg: Double = 0.0,
        altitudeDatumM: Double = 0.0,
        trueHeadingDeg: Double = 0.0,
        pitchDeg: Double = 0.0,
        rollDeg: Double = 0.0,
        demDatumM: Double? = nil
    ) {
        let hasDem = demDatumM?.isFinite == true
        let demVal = demDatumM ?? 0.0
        _ = sessionHandle.withSession { session in
            flightdeck_session_set_camera_view(
                session,
                active,
                latitudeDeg,
                longitudeDeg,
                altitudeDatumM,
                trueHeadingDeg,
                pitchDeg,
                rollDeg,
                hasDem,
                demVal
            )
        }
    }

    /// Queries Surveyed airport runway GeoJSON (`FeatureCollection`) around `(centerLatitudeDeg, centerLongitudeDeg)`
    /// on `WGS-84` Earth (prioritizing destination/route airports; empty on non-Earth bodies).
    public func runwaysGeoJson(
        centerLatitudeDeg: Double,
        centerLongitudeDeg: Double,
        radiusKm: Double = 95.0
    ) -> String {
        sessionHandle.withSession { session in
            let buf = flightdeck_session_runways_geojson(
                session,
                centerLatitudeDeg,
                centerLongitudeDeg,
                radiusKm
            )
            defer { flightdeck_byte_buffer_free(buf) }
            guard let ptr = buf.ptr, buf.len > 0 else {
                return "{\"type\":\"FeatureCollection\",\"features\":[]}"
            }
            let data = Data(bytes: ptr, count: buf.len)
            return String(data: data, encoding: .utf8)
                ?? "{\"type\":\"FeatureCollection\",\"features\":[]}"
        } ?? "{\"type\":\"FeatureCollection\",\"features\":[]}"
    }

    /// Evaluates ephemeris-driven Sun, Moon, atmosphere, and Johnson-V starfield on `celestialBody`.
    public func evaluateSky(
        utcUnixSeconds: Double,
        fallbackLatitudeDeg: Double,
        fallbackLongitudeDeg: Double,
        fallbackAltitudeDatumM: Double
    ) -> CelestialSkySnapshot {
        guard let raw = sessionHandle.withSession({ session in
            flightdeck_session_evaluate_sky(
                session,
                utcUnixSeconds,
                fallbackLatitudeDeg,
                fallbackLongitudeDeg,
                fallbackAltitudeDatumM
            )
        }) else {
            return CelestialSkySnapshot(
                hasSun: false,
                sunDirectionFixed: SIMD3<Double>(1, 0, 0),
                sunAngularRadiusDeg: 0.266,
                hasMoon: false,
                moonDirectionFixed: SIMD3<Double>(0, 1, 0),
                moonAngularRadiusDeg: 0.0,
                nightBrightness: 0.65,
                atmosphere: celestialBody.isEarthWgs84,
                starsUpdated: false,
                starCount: 0
            )
        }
        return CelestialSkySnapshot(
            hasSun: raw.has_sun,
            sunDirectionFixed: SIMD3<Double>(raw.sun_fixed_x, raw.sun_fixed_y, raw.sun_fixed_z),
            sunAngularRadiusDeg: raw.sun_angular_radius_deg,
            hasMoon: raw.has_moon,
            moonDirectionFixed: SIMD3<Double>(raw.moon_fixed_x, raw.moon_fixed_y, raw.moon_fixed_z),
            moonAngularRadiusDeg: raw.moon_angular_radius_deg,
            nightBrightness: raw.night_brightness,
            atmosphere: raw.atmosphere,
            starsUpdated: raw.stars_updated,
            starCount: raw.star_count
        )
    }

    /// Borrows the packed `[dir_x, dir_y, dir_z, v_mag, r, g, b]` `f32` star buffer from the latest `evaluateSky`.
    public func withPackedStarBuffer<R>(_ callback: (UnsafePointer<Float>?, UInt32) -> R) -> R {
        sessionHandle.withSession { session in
            var starCount: UInt32 = 0
            let ptr = flightdeck_session_stars_f32(session, &starCount)
            return callback(ptr, starCount)
        } ?? callback(nil, 0)
    }

    /// Updates pilot headset orientation (`radians`) relative to aircraft cockpit nose.
    public func updateHeadPose(yawRad: Float, pitchRad: Float, rollRad: Float) {
        _ = sessionHandle.withSession { flightdeck_session_set_head_pose($0, yawRad, pitchRad, rollRad) }
    }

    /// Toggles designation of a traffic target on the HUD and XR display.
    public func toggleDesignatedTrack(_ trackId: UInt64) {
        if designatedTrackId == trackId {
            designatedTrackId = nil
        } else {
            designatedTrackId = trackId
        }
    }

    /// Connects or disconnects the Connext TCP bridge.
    public func toggleConnextConnection() {
        if isConnextConnected {
            connextClient?.disconnect()
            isConnextConnected = false
            transportStatus = "Connext TCP disconnected"
        } else {
            sourceMode = .connextLiveTcp
            connextClient?.connect(host: connextHost, port: connextPort)
        }
    }

    /// Starts or stops the GDL-90 UDP `:4000` listener.
    public func toggleGdl90Listener() {
        if isGdl90Listening {
            gdl90Listener?.stop()
            isGdl90Listening = false
            transportStatus = "GDL-90 UDP stopped"
        } else {
            sourceMode = .gdl90LiveUdp
            gdl90Listener?.start(port: gdl90Port)
        }
    }

    /// Ingests raw Connext bytes directly (used by TCP relay and unit tests), returning outbound writes.
    @discardableResult
    public func ingestConnextBytes(_ data: Data, nowMs: UInt64) -> [[UInt8]] {
        let writes: [[UInt8]] = sessionHandle.withSession { session in
            data.withUnsafeBytes { rawBuf in
                let ptr = rawBuf.bindMemory(to: UInt8.self).baseAddress
                let outBuf = flightdeck_session_ingest_connext(session, ptr, data.count, nowMs)
                defer { flightdeck_byte_buffer_free(outBuf) }
                guard let outPtr = outBuf.ptr, outBuf.len > 0 else { return [] }
                return decodeWriteBatch(UnsafeBufferPointer(start: outPtr, count: outBuf.len))
            }
        } ?? []
        if !writes.isEmpty {
            connextClient?.send(writes)
        }
        return writes
    }

    /// Ingests raw GDL-90 UDP bytes directly.
    public func ingestGdl90Bytes(_ data: Data, nowMs: UInt64) {
        _ = sessionHandle.withSession { session in
            data.withUnsafeBytes { rawBuf in
                let ptr = rawBuf.bindMemory(to: UInt8.self).baseAddress
                flightdeck_session_ingest_gdl90(session, ptr, data.count, nowMs)
            }
        }
    }

    /// Loads KML (`<gx:Track>`, `<LineString>`) or GPX (`<trkpt>`) flight track data into Preview mode.
    @discardableResult
    public func loadKmlData(_ data: Data, sourceName: String = "KML") -> Bool {
        guard !data.isEmpty else { return false }
        let nowMs = UInt64(max(CACurrentMediaTime() * 1000.0, 0))
        let ok: Bool = sessionHandle.withSession { session in
            data.withUnsafeBytes { rawBuf in
                let ptr = rawBuf.bindMemory(to: UInt8.self).baseAddress
                return flightdeck_session_load_kml(session, ptr, data.count, nowMs)
            }
        } ?? false
        guard ok else {
            transportStatus = "Failed to parse \(sourceName)"
            return false
        }
        sourceMode = .kmlReplay
        refreshKmlTrackSummary()
        refreshKmlStatus()
        routeOverlayRevision &+= 1
        runwayOverlayRevision &+= 1
        resetDisplays()
        _ = stepFrame(nowMs: nowMs)
        let cs = kmlPlayback.callsign.isEmpty ? sourceName : kmlPlayback.callsign
        let rt = kmlPlayback.route.isEmpty ? "" : " (\(kmlPlayback.route))"
        transportStatus = "Previewing \(cs)\(rt) • \(kmlPlayback.pointCount) pts"
        return true
    }

    /// Loads a KML or GPX file from an AirDrop or Open-In security-scoped URL into Preview mode.
    @discardableResult
    public func loadKmlFile(at url: URL) -> Bool {
        let scoped = url.startAccessingSecurityScopedResource()
        defer {
            if scoped {
                url.stopAccessingSecurityScopedResource()
            }
        }
        guard let data = try? Data(contentsOf: url) else {
            transportStatus = "Unable to read \(url.lastPathComponent)"
            return false
        }
        return loadKmlData(data, sourceName: url.lastPathComponent)
    }

    /// Exits KML / GPX Preview mode cleanly and restores default telemetry state.
    public func closeKmlPreview() {
        _ = sessionHandle.withSession { flightdeck_session_clear_kml($0) }
        kmlPlayback = TrackPlaybackState()
        sourceMode = .njCorridorCruise
        routeOverlayRevision &+= 1
        runwayOverlayRevision &+= 1
        resetDisplays()
        let nowMs = UInt64(max(CACurrentMediaTime() * 1000.0, 0))
        _ = stepFrame(nowMs: nowMs)
        transportStatus = "Ready"
    }

    /// Sets KML preview pause state and playback speed multiplier (`0.25x..=30x`).
    public func setKmlPlayback(paused: Bool, speedMultiplier: Float) {
        let nowMs = UInt64(max(CACurrentMediaTime() * 1000.0, 0))
        _ = sessionHandle.withSession {
            flightdeck_session_set_kml_playback($0, paused, speedMultiplier, nowMs)
        }
        refreshKmlStatus()
    }

    /// Seeks KML preview cursor to normalized `fraction` (`0.0...1.0`).
    public func seekKml(fraction: Double) {
        let nowMs = UInt64(max(CACurrentMediaTime() * 1000.0, 0))
        _ = sessionHandle.withSession { flightdeck_session_seek_kml($0, fraction, nowMs) }
        refreshKmlStatus()
        _ = stepFrame(nowMs: nowMs)
    }

    /// Advances the Rust `FlightDeckSession` to `nowMs`, updates all `Indicate` producers, and returns ``FlightDeckFrame``.
    @discardableResult
    public func stepFrame(nowMs: UInt64? = nil) -> FlightDeckFrame? {
        let timestampMs = nowMs ?? UInt64(max(CACurrentMediaTime() * 1000.0, 0))

        if isConnextConnected {
            let writes: [[UInt8]] = sessionHandle.withSession { session in
                let tickBuf = flightdeck_session_tick_connext(session, timestampMs)
                defer { flightdeck_byte_buffer_free(tickBuf) }
                guard let ptr = tickBuf.ptr, tickBuf.len > 0 else { return [] }
                return decodeWriteBatch(UnsafeBufferPointer(start: ptr, count: tickBuf.len))
            } ?? []
            if !writes.isEmpty {
                connextClient?.send(writes)
            }
        }

        frameGeneration &+= 1
        let gen = max(frameGeneration, 1)
        guard let decoded: FlightDeckFrame = sessionHandle.withSession({ session -> FlightDeckFrame? in
            let buf = flightdeck_session_render_frame(session, timestampMs)
            defer { flightdeck_byte_buffer_free(buf) }
            guard let ptr = buf.ptr, buf.len > 0 else { return nil }
            return try? decodeFlightDeckFrame(
                UnsafeBufferPointer(start: ptr, count: buf.len),
                generation: gen
            )
        }) ?? nil else {
            return nil
        }

        sessionHandle.setGeneration(gen)
        mailbox.publish(scenes: decoded.instruments, generation: gen)
        latestFrame = decoded
        advanceWaypointSequencingIfNeeded(ownship: decoded.ownship)
        enforcePinnedPlateValidity()
        if decoded.sourceMode == .kmlReplay {
            refreshKmlStatus()
        }
        return decoded
    }

    func bumpRouteOverlayRevision() {
        routeOverlayRevision &+= 1
    }

    func bumpRunwayOverlayRevision() {
        runwayOverlayRevision &+= 1
    }

    func bumpPlateOverlayRevision() {
        plateOverlayRevision &+= 1
    }

    private func resetDisplays() {
        let nowMs = CACurrentMediaTime() * 1000.0
        pfdDisplay.reset(nowMs: nowMs)
        hsiDisplay.reset(nowMs: nowMs)
        hudDisplay.reset(nowMs: nowMs)
        hwdOverlayDisplay.reset(nowMs: nowMs)
        terrainVolumeDisplay.reset(nowMs: nowMs)
    }

    private func refreshKmlStatus() {
        guard let st = sessionHandle.withSession({ flightdeck_session_kml_status($0) }) else { return }
        kmlPlayback.hasTrack = st.has_track
        kmlPlayback.paused = st.paused
        kmlPlayback.speedMultiplier = st.speed_multiplier
        kmlPlayback.elapsedSeconds = st.elapsed_s
        kmlPlayback.durationSeconds = st.duration_s
        kmlPlayback.pointCount = Int(st.point_count)
    }

    private func refreshKmlTrackSummary() {
        let jsonData: Data? = sessionHandle.withSession { session in
            let buf = flightdeck_session_kml_track_json(session)
            defer { flightdeck_byte_buffer_free(buf) }
            guard let ptr = buf.ptr, buf.len > 0 else { return nil }
            return Data(bytes: ptr, count: buf.len)
        } ?? nil
        guard let data = jsonData,
              let obj = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any] else {
            return
        }
        kmlPlayback.callsign = (obj["callsign"] as? String) ?? ""
        kmlPlayback.route = (obj["route"] as? String) ?? ""
        kmlPlayback.title = (obj["title"] as? String) ?? ""
        if let rawCoords = obj["coordinates"] as? [[Double]] {
            kmlPlayback.coordinates = rawCoords.compactMap { arr in
                guard arr.count >= 3 else { return nil }
                return SIMD3<Double>(arr[0], arr[1], arr[2])
            }
        }
    }

    private func applySourceMode(_ mode: TelemetrySourceMode) {
        _ = sessionHandle.withSession { flightdeck_session_set_source_mode($0, mode.rawValue) }
        routeOverlayRevision &+= 1
        runwayOverlayRevision &+= 1
        resetDisplays()
        transportStatus = mode.subtitle
    }

    private func setupTransports() {
        connextClient = ConnextTcpClient(
            onStateChange: { [weak self] connected, status in
                Task { @MainActor [weak self] in
                    self?.isConnextConnected = connected
                    self?.transportStatus = status
                }
            },
            onBytes: { [weak self] data in
                Task { @MainActor [weak self] in
                    let nowMs = UInt64(max(CACurrentMediaTime() * 1000.0, 0))
                    _ = self?.ingestConnextBytes(data, nowMs: nowMs)
                }
            }
        )
        gdl90Listener = Gdl90UdpListener(
            onStateChange: { [weak self] listening, status in
                Task { @MainActor [weak self] in
                    self?.isGdl90Listening = listening
                    self?.transportStatus = status
                }
            },
            onDatagram: { [weak self] data in
                Task { @MainActor [weak self] in
                    let nowMs = UInt64(max(CACurrentMediaTime() * 1000.0, 0))
                    self?.ingestGdl90Bytes(data, nowMs: nowMs)
                }
            }
        )
    }

    private static func queryBodyParameters(
        handle: FfiSessionHandle,
        fallback: CelestialBody
    ) -> CelestialBodyParameters {
        guard let info = handle.withSession({ flightdeck_session_body_info($0) }) else {
            return CelestialBodyParameters(
                body: fallback,
                renderRadiusMeters: fallback.isEarthWgs84 ? 6_371_008.8 : 1_737_400.0,
                highestGroundMeters: 8_849.0,
                lowestGroundMeters: -450.0,
                semiMajorMeters: fallback.isEarthWgs84 ? 6_378_137.0 : 1_737_400.0,
                semiMinorMeters: fallback.isEarthWgs84 ? 6_356_752.3 : 1_737_400.0,
                surfaceGravityMps2: fallback.isEarthWgs84 ? 9.81 : 1.62,
                hasAtmosphere: fallback.isEarthWgs84,
                isEarthWgs84: fallback.isEarthWgs84
            )
        }
        let resolvedBody = CelestialBody(rawValue: info.body_code) ?? fallback
        return CelestialBodyParameters(
            body: resolvedBody,
            renderRadiusMeters: info.render_radius_m,
            highestGroundMeters: info.highest_ground_m,
            lowestGroundMeters: info.lowest_ground_m,
            semiMajorMeters: info.semi_major_m,
            semiMinorMeters: info.semi_minor_m,
            surfaceGravityMps2: info.surface_gravity_mps2,
            hasAtmosphere: info.has_atmosphere,
            isEarthWgs84: info.is_earth_wgs84
        )
    }

    private static func loadVerifiedGlyphPack() -> InstrumentGlyphPack {
        let buf = flightdeck_verified_glyph_pack()
        defer { flightdeck_byte_buffer_free(buf) }
        guard let ptr = buf.ptr, buf.len > 0,
              let pack = try? decodeInstrumentGlyphPack(UnsafeBufferPointer(start: ptr, count: buf.len)) else {
            return InstrumentGlyphPack(cellWidth: 5, cellHeight: 7, contentHash: Array(repeating: 0, count: 32), glyphs: [])
        }
        return pack
    }
}

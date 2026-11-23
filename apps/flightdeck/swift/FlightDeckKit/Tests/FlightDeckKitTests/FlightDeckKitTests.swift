import Foundation
import IndicateAppleDisplay
import RealityKit
import Testing
@testable import FlightDeckKit

@MainActor
@Suite("FlightDeckKit End-to-End Bridge, Explicit Body, RealityKit & Indicate Suite")
struct FlightDeckKitTests {
    @Test("Digital Crown blend preserves >= 15% real-world passthrough across full range")
    func crownPassthroughFloor() {
        let controller = FlightDeckController(body: .earthWgs84Egm96)
        for raw in stride(from: Float(-0.2), through: Float(1.3), by: 0.15) {
            controller.crownTurn = raw
            let frame = controller.stepFrame(nowMs: 1_000)
            #expect(frame != nil)
            if let blend = frame?.blend {
                #expect(blend.passthroughFraction >= SvsBlendWeights.minPassthroughFraction - 1e-5)
                #expect(blend.virtualTerrainOpacity <= SvsBlendWeights.maxVirtualOpacity + 1e-5)
            }
        }
    }

    @Test("Connext real-session replay decodes 47N ownship, traffic callsigns, and honest missing IAS/VSI")
    func connextHardwareCaptureReplay() {
        let controller = FlightDeckController(body: .earthWgs84Egm96)
        controller.sourceMode = .connextReplay
        let boot = controller.stepFrame(nowMs: 0)
        #expect(boot?.ownship.gnssValid == false)

        let snap = controller.stepFrame(nowMs: 5_100)
        #expect(snap != nil)
        #expect(snap?.ownship.gnssValid == true)
        #expect(snap?.ownship.callsign == "N19880")
        #expect(snap?.ownship.indicatedAirspeedKt == nil)
        #expect(snap?.ownship.verticalSpeedFpm == nil)
        #expect(snap?.ownship.activeLeg == "47N → SBJ")

        let callsigns = Set(snap?.traffic.map(\.callsign) ?? [])
        #expect(callsigns.contains("JBU1990"))
        #expect(callsigns.contains("DWI2363"))
        #expect(callsigns.contains("ATN3345"))
        #expect(callsigns.contains("DLH499"))
    }

    @Test("Explicit lunar IAU body suppresses Earth NASR/WMM/ADS-B and evaluates vacuum starfield")
    func explicitLunarBodyNeverAssumesEarth() {
        let controller = FlightDeckController(body: .moonIauSphere)
        #expect(controller.celestialBody == .moonIauSphere)
        #expect(!controller.bodyParameters.isEarthWgs84)
        #expect(!controller.bodyParameters.hasAtmosphere)
        #expect(abs(controller.bodyParameters.renderRadiusMeters - 1_737_400.0) < 1.0)
        #expect(abs(controller.bodyParameters.surfaceGravityMps2 - 1.62) < 0.05)

        let frame = controller.stepFrame(nowMs: 2_000)
        #expect(frame != nil)
        #expect(frame?.airspaces.isEmpty == true)
        #expect(frame?.traffic.isEmpty == true)
        #expect(frame?.ownship.magneticHeadingDeg == nil)

        let sky = controller.evaluateSky(
            utcUnixSeconds: 1_788_134_400.0,
            fallbackLatitudeDeg: 0.674,
            fallbackLongitudeDeg: 23.473,
            fallbackAltitudeDatumM: 1_200.0
        )
        #expect(!sky.atmosphere)
        #expect(!sky.hasMoon)
        #expect(sky.starCount > 1_000)

        let hudOut = controller.hudDisplay.renderBlocking(pixelWidth: 960, pixelHeight: 720, nowMs: 2_100)
        #expect(!hudOut.showingFailure)
    }

    @Test("RealityKit scene materializes terrain, airspace curtains, traffic callsigns & HWD strokes, and Indicate displays paint cleanly")
    func realitySceneAndIndicatePanelsPaint() {
        let controller = FlightDeckController(body: .earthWgs84Egm96)
        controller.sourceMode = .highDensityTrafficCpa
        controller.crownTurn = 1.0

        guard let frame = controller.stepFrame(nowMs: 2_500) else {
            Issue.record("Expected non-nil FlightDeckFrame")
            return
        }
        #expect(!frame.terrain.positionsEnuM.isEmpty)
        #expect(!frame.airspaces.isEmpty)
        #expect(frame.traffic.contains { $0.callsign == "UAL842" && $0.threat >= .trafficAdvisory })
        #expect(!frame.instruments.hwd3dStrokesEnu.isEmpty)

        let scene = SvsRealityScene()
        scene.update(with: frame, designatedTrackId: frame.traffic.first?.trackId)
        #expect(!scene.rootEntity.children.isEmpty)

        let pfdOut = controller.pfdDisplay.renderBlocking(pixelWidth: 480, pixelHeight: 360, nowMs: 2_600)
        #expect(!pfdOut.showingFailure)
        #expect(pfdOut.image != nil)

        let hsiOut = controller.hsiDisplay.renderBlocking(pixelWidth: 480, pixelHeight: 360, nowMs: 2_600)
        #expect(!hsiOut.showingFailure)

        let hudOut = controller.hudDisplay.renderBlocking(pixelWidth: 960, pixelHeight: 720, nowMs: 2_600)
        #expect(!hudOut.showingFailure)

        let hwdOut = controller.hwdOverlayDisplay.renderBlocking(pixelWidth: 1200, pixelHeight: 600, nowMs: 2_600)
        #expect(!hwdOut.showingFailure)

        let volOut = controller.terrainVolumeDisplay.render(nowMs: 2_600)
        #expect(!volOut.showingFailure)
    }

    @Test("Distinct celestial bodies, vertical datum separation, AirportDatabase FPL, and GCS lease gating")
    func distinctBodiesFlightPlanAndGcsLeaseGating() {
        #expect(PrimaryCelestialBody.allCases == [.earth, .moon, .mars])
        #expect(CelestialBody.resolve(primary: .earth, earthVerticalDatum: .egm96Msl) == .earthWgs84Egm96)
        #expect(CelestialBody.resolve(primary: .earth, earthVerticalDatum: .wgs84Hae) == .earthWgs84Ellipsoid)
        #expect(CelestialBody.earthWgs84Egm96.abbrevLabel == "EARTH")
        #expect(CelestialBody.earthWgs84Ellipsoid.abbrevLabel == "EARTH")
        #expect(CelestialBody.moonIauSphere.abbrevLabel == "MOON")
        #expect(CelestialBody.marsIauSpheroid.abbrevLabel == "MARS")

        let controller = FlightDeckController(body: .earthWgs84Egm96)
        #expect(controller.flightPlanRouteSummary == "KEWR → N51")

        let hsv = controller.lookupAirport("KHSV")
        #expect(hsv?.code == "KHSV")
        #expect((hsv?.runways.isEmpty ?? true) == false)

        controller.setFlightPlanCodes(["KMEM", "KHSV", "KATL"])
        #expect(controller.flightPlanRouteSummary == "KMEM → KHSV → KATL")
        #expect(controller.totalFlightPlanDistanceNm > 250.0)
        #expect(controller.flightPlanGeoJson().contains("active_leg"))

        // Passive ADS-B links never allow GCS actuation even if toggled.
        controller.selectReceiverFamily(.wifiGdl90)
        #expect(!controller.executeGcsCommand("HOLD"))
        controller.selectReceiverFamily(.usbAdsBStub)
        #expect(!controller.executeGcsCommand("RTL"))

        // Sokoly GCS / Simulation bench requires explicit Control Lease before actuation.
        controller.selectReceiverFamily(.simulationBench)
        #expect(!controller.executeGcsCommand("HOLD"))
        controller.gcsState.hasControlLease = true
        #expect(controller.executeGcsCommand("HOLD"))

        controller.setCameraViewPose(
            active: true,
            latitudeDeg: 40.5965,
            longitudeDeg: -74.7239,
            altitudeDatumM: 122.0,
            trueHeadingDeg: 210.0,
            pitchDeg: -0.6,
            rollDeg: 0.0,
            demDatumM: 58.0
        )
        _ = controller.stepFrame(nowMs: 3_000)

        let wideHud = controller.hudDisplay.renderBlocking(pixelWidth: 2200, pixelHeight: 1480, nowMs: 3_050)
        #expect(!wideHud.showingFailure)
        #expect(wideHud.image != nil)

        let earthJson = controller.runwaysGeoJson(centerLatitudeDeg: 33.6407, centerLongitudeDeg: -84.4277, radiusKm: 40.0)
        #expect(earthJson.contains("KATL"))

        controller.setPrimaryBody(.moon)
        #expect(controller.flightPlanWaypoints.isEmpty)
        let moonJson = controller.runwaysGeoJson(centerLatitudeDeg: 0.674, centerLongitudeDeg: 23.473, radiusKm: 40.0)
        #expect(moonJson == "{\"type\":\"FeatureCollection\",\"features\":[]}")
    }

    @Test("Multi-type NASR waypoints, strict ASCII-only route sanitizer, coordinate parser, and bubble reorder")
    func multiTypeWaypointsAsciiSanitizerAndReorder() {
        // 1. Strict ASCII-only validation (`A-Z`, `0-9`, space, `./+-` for coordinates).
        #expect(RouteAsciiInput.normalized("kmem vuz habje khsv rmg katl") == "KMEM VUZ HABJE KHSV RMG KATL")
        #expect(RouteAsciiInput.normalized("40.58/-74.70 3851n07702w") == "40.58/-74.70 3851N07702W")
        #expect(RouteAsciiInput.normalized("\u{822A}\u{8DEF}\u{70B9}") == nil)
        #expect(RouteAsciiInput.normalized("KEWR ✈️ N51") == nil)
        #expect(RouteAsciiInput.words(from: "KEWR DCT SBJ N51") == ["KEWR", "SBJ", "N51"])

        // 2. Coordinate token parsing (decimal slash and ICAO compact DDMMNDDDMMW).
        let slashCoord = RouteAsciiInput.parseCoordinate("40.58/-74.70")
        #expect(slashCoord?.kind == .coordinate)
        #expect(slashCoord?.kind.badge == "GPS")
        #expect(abs((slashCoord?.latitudeDeg ?? 0.0) - 40.58) < 1e-4)
        #expect(abs((slashCoord?.longitudeDeg ?? 0.0) - (-74.70)) < 1e-4)

        let icaoCoord = RouteAsciiInput.parseCoordinate("3851N07702W")
        #expect(icaoCoord?.kind == .coordinate)
        #expect(abs((icaoCoord?.latitudeDeg ?? 0.0) - 38.85) < 1e-3)
        #expect(abs((icaoCoord?.longitudeDeg ?? 0.0) - (-77.03333)) < 1e-3)

        // 3. Multi-type resolution across Airports (ForeFlight clean ident, zero "APT" clutter), VORTACs/VORs, NDBs, Fixes, and GPS.
        let controller = FlightDeckController(body: .earthWgs84Egm96)
        controller.setFlightPlanCodes(["KMEM", "VUZ", "HABJE", "KHSV", "RMG", "KATL"])
        #expect(controller.flightPlanWaypoints.count == 6)
        #expect(controller.flightPlanWaypoints[0].ident == "KMEM")
        #expect(controller.flightPlanWaypoints[0].kind == .airport)
        #expect(controller.flightPlanWaypoints[0].kind.badge.isEmpty)
        #expect(controller.flightPlanWaypoints[0].region != "APT")
        #expect(controller.flightPlanWaypoints[1].ident == "VUZ")
        #expect(controller.flightPlanWaypoints[1].kind == .vortac)
        #expect(controller.flightPlanWaypoints[1].kind.badge == "VORTAC")
        #expect(controller.flightPlanWaypoints[2].ident == "HABJE")
        #expect(controller.flightPlanWaypoints[2].kind == .fix)
        #expect(controller.flightPlanWaypoints[2].kind.badge == "FIX")
        #expect(controller.flightPlanWaypoints[3].ident == "KHSV")
        #expect(controller.flightPlanWaypoints[3].kind == .airport)
        #expect(controller.flightPlanWaypoints[4].ident == "RMG")
        #expect(controller.flightPlanWaypoints[4].kind == .vortac)
        #expect(controller.flightPlanWaypoints[5].ident == "KATL")
        #expect(controller.flightPlanWaypoints[5].kind == .airport)
        #expect(controller.destinationAirportWaypoint?.ident == "KATL")
        #expect(controller.destinationRunway != nil)

        // GeoJSON carries ForeFlight-clean `label == ident` with zero `"APT"` suffixes.
        let geoJson = controller.flightPlanGeoJson()
        #expect(geoJson.contains("\"wp_kind\":\"vortac\""))
        #expect(geoJson.contains("\"wp_kind\":\"fix\""))
        #expect(geoJson.contains("\"label\":\"KMEM\""))
        #expect(geoJson.contains("\"label\":\"VUZ\""))
        #expect(geoJson.contains("\"label\":\"HABJE\""))
        #expect(!geoJson.contains("APT"))

        // 4. Deleting a non-airport fix bumps `routeOverlayRevision` without bumping `runwayOverlayRevision`,
        //    and `clearFlightPlan()` from 6 waypoints empties cleanly without crashing.
        let rwyRevBeforeFixRemove = controller.runwayOverlayRevision
        let routeRevBeforeFixRemove = controller.routeOverlayRevision
        controller.removeFlightPlanWaypoint(at: 2) // Remove HABJE (RNAV fix)
        #expect(controller.routeOverlayRevision > routeRevBeforeFixRemove)
        #expect(controller.runwayOverlayRevision == rwyRevBeforeFixRemove)

        // Restore 6-waypoint route, drag-reorder, then clear completely.
        controller.setFlightPlanCodes(["KMEM", "VUZ", "HABJE", "KHSV", "RMG", "KATL"])
        let activeId = controller.flightPlanWaypoints[controller.activeWaypointIndex].id
        var reordered = controller.flightPlanWaypoints
        let moved = reordered.remove(at: 2) // Move HABJE before VUZ
        reordered.insert(moved, at: 1)
        controller.replaceFlightPlanWaypoints(reordered)
        #expect(controller.flightPlanWaypoints.map(\.ident) == ["KMEM", "HABJE", "VUZ", "KHSV", "RMG", "KATL"])
        #expect(controller.flightPlanWaypoints[controller.activeWaypointIndex].id == activeId)

        controller.clearFlightPlan()
        #expect(controller.flightPlanWaypoints.isEmpty)
        #expect(controller.destinationRunway == nil)
        #expect(controller.flightPlanRouteSummary == "No Route")
        #expect(controller.flightPlanGeoJson() == "{\"type\":\"FeatureCollection\",\"features\":[]}" || controller.flightPlanGeoJson().contains("\"features\":[]"))

        // Planetary gazetteer search on Moon (`SITE`).
        controller.setPrimaryBody(.moon)
        controller.setFlightPlanCodes(["APOLLO11", "TYCHO"])
        #expect(controller.flightPlanWaypoints.map(\.ident) == ["APOLLO11", "TYCHO"])
        #expect(controller.flightPlanWaypoints.first?.kind == .planetarySite)
        #expect(controller.flightPlanWaypoints.first?.kind.badge == "SITE")
    }

    @Test("Offline AIRAC 28d & FAA 56d cycle store enforces pre-download and strict expiration invalidation")
    func offlineDataPackStoreAndStrictExpirationInvalidation() throws {
        let tempStore = FileManager.default.temporaryDirectory
            .appendingPathComponent("AeroDataStoreTest-\(UUID().uuidString)", isDirectory: true)
        defer { try? FileManager.default.removeItem(at: tempStore) }

        let store = AeroDataPackStore(storeDirectory: tempStore)
        let controller = FlightDeckController(body: .earthWgs84Egm96, dataPackStore: store)

        // 1. Inside AIRAC 2611 (`2026-11-13T12:00:00Z`): all 7 offline Flightbag packs (28d NASR/CIFP + 56d VFR/IFR) are valid!
        let mid2611 = try #require(AiracCycleGrid.parseIso8601Utc("2026-11-13T12:00:00Z"))
        #expect(AiracCycleGrid.airacCode(at: mid2611) == "2611")
        store.setReferenceUtcOverride(mid2611)

        #expect(store.validity(for: .faaNasrAirspace).isValidForUse)
        #expect(store.validity(for: .faaNasrWaypoints).isValidForUse)
        #expect(store.validity(for: .faaCifp).isValidForUse)
        #expect(AeroDataAuthority.fromManifestSlug("faa_cifp") == .faaCifp)
        #expect(AeroDataAuthority.fromManifestSlug("faa_cifp_arinc424") == .faaCifp)
        #expect(AeroDataAuthority.faaCifp.expectedFileName == "cifp_procedures.bin")
        #expect(store.validity(for: .faaVfrSectional).isValidForUse)
        #expect(store.validity(for: .faaVfrTerminal).isValidForUse)
        #expect(store.validity(for: .faaIfrLow).isValidForUse)
        #expect(store.validity(for: .faaIfrHigh).isValidForUse)
        #expect(controller.isEarthNavdataUsable)
        #expect(controller.isCifpUsable)
        #expect(!controller.searchNavigationPoints("KMEM").isEmpty)

        // Verify `Flightbag` `dist/manifest.json` (`RollingManifest`) dual-schema compatibility.
        let flightbagManifestUrl = URL(fileURLWithPath: "/Users/fangluo/Flightbag/dist/manifest.json")
        if FileManager.default.fileExists(atPath: flightbagManifestUrl.path) {
            let manifestBytes = try Data(contentsOf: flightbagManifestUrl)
            let remoteEntries = AeroDataPackStore.parseRemoteManifestEntries(manifestBytes)
            #expect(remoteEntries.count >= 6)
            let mappedAuthorities = Set(remoteEntries.compactMap {
                AeroDataAuthority.fromManifestSlug($0.authority, fileName: $0.file ?? $0.url)
            })
            #expect(mappedAuthorities.contains(.faaCifp))
            #expect(mappedAuthorities.contains(.faaNasrAirspace))
            #expect(mappedAuthorities.contains(.faaVfrSectional))
            #expect(mappedAuthorities.contains(.faaVfrTerminal))
            #expect(mappedAuthorities.contains(.faaIfrLow))
            #expect(mappedAuthorities.contains(.faaIfrHigh))
        }

        // 2. Past 28-day NASR & CIFP expiry (`2026-12-03T12:00:00Z`): Airspace, Waypoints, and CIFP become `.expiredInvalid`,
        //    while 56-day VFR & IFR charts (expiring `2026-12-24T09:01:00Z`) remain `.valid`!
        let post28d = try #require(AiracCycleGrid.parseIso8601Utc("2026-12-03T12:00:00Z"))
        store.setReferenceUtcOverride(post28d)

        #expect(store.validity(for: .faaNasrAirspace).isExpiredInvalid)
        #expect(store.validity(for: .faaNasrAirspace).activePack == nil)
        #expect(store.validity(for: .faaNasrWaypoints).isExpiredInvalid)
        #expect(store.validity(for: .faaCifp).isExpiredInvalid)
        #expect(!controller.isEarthNavdataUsable)
        #expect(!controller.isCifpUsable)
        #expect(controller.availableCifpProcedures(forAirport: "N51").isEmpty)
        // Stale NASR database identifiers are refused when expired, whereas pure mathematical LAT/LON still parses.
        #expect(controller.searchNavigationPoints("KMEM").isEmpty)
        #expect(controller.searchNavigationPoints("40.58/-74.70").count == 1)
        #expect(store.validity(for: .faaVfrSectional).isValidForUse)
        #expect(store.validity(for: .faaVfrTerminal).isValidForUse)
        #expect(store.validity(for: .faaIfrLow).isValidForUse)
        #expect(store.validity(for: .faaIfrHigh).isValidForUse)

        // 3. Past 56-day VFR/IFR chart expiry (`2027-01-02T12:00:00Z`): ALL 2611 packs are `.expiredInvalid` (`activePack == nil`).
        let post56d = try #require(AiracCycleGrid.parseIso8601Utc("2027-01-02T12:00:00Z"))
        store.setReferenceUtcOverride(post56d)

        let resolvedExpired = store.resolvedPackSet()
        #expect(resolvedExpired.airspace == nil)
        #expect(resolvedExpired.waypoints == nil)
        #expect(resolvedExpired.cifpProcedures == nil)
        #expect(resolvedExpired.vfrSectional == nil)
        #expect(resolvedExpired.vfrTerminal == nil)
        #expect(resolvedExpired.ifrLow == nil)
        #expect(resolvedExpired.ifrHigh == nil)

        // 4. Offloading and restoring a pack at a valid date immediately suppresses and restores it.
        store.setReferenceUtcOverride(mid2611)
        store.removeOrOffloadPack(for: .faaVfrSectional)
        #expect(store.validity(for: .faaVfrSectional) == .notDownloaded)
        store.restoreBundledSeedIfPresent(for: .faaVfrSectional)
        #expect(store.validity(for: .faaVfrSectional).isValidForUse)
    }

    @Test("FAA CIFP FBCIFP01 database decodes airways, SID/STAR/IAP legs, expands V3, and aligns arrival runway")
    func cifpAirwaysAndProceduresEndToEnd() throws {
        let tempStore = FileManager.default.temporaryDirectory
            .appendingPathComponent("AeroDataStoreCifpTest-\(UUID().uuidString)", isDirectory: true)
        defer { try? FileManager.default.removeItem(at: tempStore) }

        let store = AeroDataPackStore(storeDirectory: tempStore)
        let mid2611 = try #require(AiracCycleGrid.parseIso8601Utc("2026-11-13T12:00:00Z"))
        store.setReferenceUtcOverride(mid2611)
        let controller = FlightDeckController(body: .earthWgs84Egm96, dataPackStore: store)

        let db = try #require(controller.activeCifpDatabase)
        #expect(db.airacCode == "2611")
        #expect(db.airwayCount >= 1_400)
        #expect(db.procedureCount >= 14_000)

        // 1. Enroute airway expansion (`SBJ V3 MXE` and dot-chained `SBJ.V3.MXE`).
        let v3Forward = try #require(db.expandAirway(routeIdent: "V3", from: "SBJ", to: "MXE", includeEntry: true))
        #expect(v3Forward.first?.ident == "SBJ")
        #expect(v3Forward.last?.ident == "MXE")
        #expect(v3Forward.count >= 3)

        let v3Reverse = try #require(db.expandAirway(routeIdent: "V3", from: "MXE", to: "SBJ", includeEntry: true))
        #expect(v3Reverse.map(\.ident) == v3Forward.map(\.ident).reversed())

        controller.setFlightPlanCodes(["KEWR", "SBJ", "V3", "MXE", "N51"])
        #expect(controller.flightPlanWaypoints.first?.ident == "KEWR")
        #expect(controller.flightPlanWaypoints.last?.ident == "N51")
        #expect(controller.flightPlanWaypoints.count == 2 + v3Forward.count)
        #expect(controller.flightPlanWaypoints.contains { $0.airwayOrProcedureTag == "V3" })

        controller.setFlightPlanCodes(["KEWR", "SBJ.V3.MXE", "N51"])
        #expect(controller.flightPlanWaypoints.count == 2 + v3Forward.count)

        // 2. Destination approach assembly (`N51` `R22` via `PIYIN` IAF) & automatic arrival runway `22` alignment.
        controller.setFlightPlanCodes(["KEWR", "N51"], destinationRunway: "04")
        let n51Approaches = controller.availableCifpProcedures(forAirport: "N51", kind: .approach)
        let r22 = try #require(n51Approaches.first { $0.procedureIdent == "R22" })
        #expect(r22.humanTitle == "RNAV (GPS) RWY 22")
        #expect(r22.associatedRunwayDesignator == "22")
        #expect(r22.enrouteOrFeederTransitions.contains { $0.ident == "PIYIN" })

        let inserted = controller.applyCifpProcedure(r22, enrouteOrFeederTransitionIdent: "PIYIN")
        #expect(inserted >= 4)
        let idents = controller.flightPlanWaypoints.map(\.ident)
        #expect(idents == ["KEWR", "PIYIN", "DODLY", "CAVZA", "RW22", "N51"])
        // Seam fix `DODLY` preserves its `+3000 FT` final-approach constraint after transition seam deduplication!
        let dodly = try #require(controller.flightPlanWaypoints.first { $0.ident == "DODLY" })
        #expect(dodly.altitudeConstraintTag.contains("+3000 FT"))
        #expect(controller.destinationRunway == "22")

        // 3. Departure SID (`KEWR` `PORTT4` via `RW22B` -> `BIGGY`).
        let kewrSids = controller.availableCifpProcedures(forAirport: "KEWR", kind: .sid)
        let portt4 = try #require(kewrSids.first { $0.procedureIdent == "PORTT4" })
        let sidLegs = controller.applyCifpProcedure(
            portt4,
            runwayTransitionIdent: "RW22B",
            enrouteOrFeederTransitionIdent: "BIGGY"
        )
        #expect(sidLegs >= 4)
        #expect(controller.flightPlanWaypoints.first?.ident == "KEWR")
        #expect(controller.flightPlanWaypoints.contains { $0.ident == "PORTT" && $0.airwayOrProcedureTag == "SID PORTT4" })
    }

    @Test("GeoPDF terminal procedure plates, multi-provider registry, procedural fallback, and 28-day plate expiration")
    func geoPdfPlatesAndMultiProviderService() throws {
        let tempStore = FileManager.default.temporaryDirectory
            .appendingPathComponent("AeroDataStorePlatesTest-\(UUID().uuidString)", isDirectory: true)
        defer { try? FileManager.default.removeItem(at: tempStore) }

        let store = AeroDataPackStore(storeDirectory: tempStore)
        let mid2611 = try #require(AiracCycleGrid.parseIso8601Utc("2026-11-13T12:00:00Z"))
        store.setReferenceUtcOverride(mid2611)
        let controller = FlightDeckController(body: .earthWgs84Egm96, dataPackStore: store)

        // 1. Multi-provider registry supports simultaneous services (`FAA AeroNav` + `Jeppesen` + `EUROCONTROL`).
        #expect(controller.chartProviders.providers.count == 5)
        controller.chartProviders.toggleProviderEnabled(ChartProviderRegistry.jeppesenId)
        controller.chartProviders.toggleProviderEnabled(ChartProviderRegistry.eurocontrolId)
        #expect(controller.chartProviders.enabledProviderIds.count == 3)
        #expect(controller.chartProviders.activeSummaryLabel == "FAA AeroNav +2")
        controller.chartProviders.selectPreferredProvider(ChartProviderRegistry.jeppesenId)
        #expect(controller.chartProviders.stubFallbackNotice != nil)

        // 2. Official FAA GeoPDF (`N51` `05075R22.PDF`) solves ISO 32000-1 `/VP` corners around N51 (`40.58°N, -74.74°W`).
        let n51Plates = controller.availablePlates(forAirport: "N51")
        #expect(!n51Plates.isEmpty)
        let r22Plate = try #require(n51Plates.first { $0.pdfName == "05075R22.PDF" })
        #expect(r22Plate.isOfficialPdf)
        #expect(r22Plate.supportsMapOverlay)

        let pinned = try #require(controller.pinPlateToMap(r22Plate))
        #expect(pinned.cornersLonLat.count == 8)
        #expect(FileManager.default.fileExists(atPath: pinned.pngFileUrl.path))
        // NW latitude > SE latitude, NE longitude > NW longitude, and N51 (`40.5829, -74.7364`) + PIYIN (`40.753, -74.484`) sit strictly inside!
        let nwLon = pinned.cornersLonLat[0]
        let nwLat = pinned.cornersLonLat[1]
        let neLon = pinned.cornersLonLat[2]
        let seLat = pinned.cornersLonLat[5]
        #expect(nwLat > 40.74 && seLat < 40.52)
        #expect(nwLon < -74.80 && neLon > -74.45)

        // 3. Airport Diagram (`KEWR` `00285AD.PDF`) loads raw PDF bytes for `PDFKit` full-plate inspection.
        let kewrDiagrams = controller.availablePlates(forAirport: "KEWR", kind: .airportDiagram)
        let kewrAd = try #require(kewrDiagrams.first)
        let adPdf = try #require(controller.loadPlatePdfData(for: kewrAd))
        #expect(adPdf.count > 10_000)

        // 4. Procedural planview fallback (`KISP` approach not in the 13-airport PDF seed) synthesizes both PNG & PDF.
        let kispApproaches = controller.availableCifpProcedures(forAirport: "KISP", kind: .approach)
        let kispProc = try #require(kispApproaches.first)
        let synthDesc = controller.resolvePlateDescriptor(for: kispProc)
        #expect(!synthDesc.isOfficialPdf)
        #expect(synthDesc.supportsMapOverlay)
        let synthPinned = try #require(controller.pinPlateToMap(synthDesc, procedure: kispProc))
        #expect(FileManager.default.fileExists(atPath: synthPinned.pngFileUrl.path))
        let synthPdf = try #require(controller.loadPlatePdfData(for: synthDesc, procedure: kispProc))
        #expect(synthPdf.count > 1_000)

        // 5. Advancing past 28-day AIRAC expiry (`2026-12-03T12:00:00Z`) suppresses `dtpp` & clears pinned map plate!
        _ = controller.pinPlateToMap(r22Plate)
        #expect(controller.pinnedMapPlate != nil)
        let post28d = try #require(AiracCycleGrid.parseIso8601Utc("2026-12-03T12:00:00Z"))
        store.setReferenceUtcOverride(post28d)
        _ = controller.stepFrame(nowMs: 1_000)
        #expect(!controller.isDtppUsable)
        #expect(controller.pinnedMapPlate == nil)
    }

    @Test("Minimized FPL summary computes Origin, Current, Next, Destination, leg/remaining distance, ETE, and fuel")
    func compactFlightPlanSummaryMetrics() throws {
        let controller = FlightDeckController(body: .earthWgs84Egm96)
        controller.plannedCruiseKtas = 120.0
        controller.plannedFuelFlowGph = 10.0

        // Multi-waypoint route: `KEWR -> PIYIN -> DODLY -> CAVZA -> RW22 -> N51`
        controller.setFlightPlanCodes(["KEWR", "N51"])
        let r22 = try #require(controller.availableCifpProcedures(forAirport: "N51", kind: .approach).first { $0.procedureIdent == "R22" })
        _ = controller.applyCifpProcedure(r22, enrouteOrFeederTransitionIdent: "PIYIN")
        controller.activateDirectTo(index: 2) // Active `TO` = `DODLY`, `Next` = `CAVZA`, `Origin` = `KEWR`, `Dest` = `N51`

        let summary = controller.compactFlightPlanSummary(from: nil)
        #expect(summary.hasRoute)
        #expect(summary.originIdent == "KEWR")
        #expect(summary.hasSequencedGapAfterOrigin)
        #expect(summary.currentIdent == "DODLY")
        #expect(summary.nextIdent == "CAVZA")
        #expect(summary.destinationIdent == "N51")
        #expect(summary.runwayBadge == "22")
        #expect(summary.hasRemainingLegsBeyondCurrent)
        let legNm = try #require(summary.legDistanceNm)
        let remNm = try #require(summary.remainingDistanceNm)
        #expect(remNm > legNm + 3.0)
        #expect(summary.eteFormatted != nil)
        #expect((summary.fuelRequiredGal ?? 0.0) > 0.2)

        // Switching to Moon suppresses Earth-only fuel gallon readout cleanly.
        controller.setCelestialBody(.moonIauSphere)
        controller.setFlightPlanCodes(["38.85N/77.04W", "39.10N/76.80W"])
        let lunarSummary = controller.compactFlightPlanSummary(from: nil)
        #expect(lunarSummary.hasRoute)
        #expect(lunarSummary.fuelRequiredGal == nil)
    }

    @Test("FAA AC 25-11B / DOT-FAA-AM-07-24 / Sokoly avionics color system enforces WCAG AA contrast, <=6 semantic colors, and CVD ΔE00 >= 10")
    func faaHumanFactorsAvionicsColorSystemInvariants() {
        let violations = AvionicsColorSystem.verifyHumanFactorsInvariants()
        #expect(violations.isEmpty, "Human-factors color violations: \(violations)")

        // Canonical `brand.toml [ui.*]` / `Indicate` ADR-0029 parity checks.
        #expect(AvionicsColorSystem.background.hex == "#0B0D11")
        #expect(AvionicsColorSystem.surface.hex == "#16191F")
        #expect(AvionicsColorSystem.control.hex == "#262B33")
        #expect(AvionicsColorSystem.separator.hex == "#30363F")
        #expect(AvionicsColorSystem.value.hex == "#ECF0F3")
        #expect(AvionicsColorSystem.label.hex == "#8A94A0")
        #expect(AvionicsColorSystem.disabled.hex == "#68707C")
        #expect(AvionicsColorSystem.commanded.hex == "#00FFFF")
        #expect(AvionicsColorSystem.plan.hex == "#E850E8")
        #expect(AvionicsColorSystem.active.hex == "#48CE88")
        #expect(AvionicsColorSystem.caution.hex == "#FFB020")
        #expect(AvionicsColorSystem.warning.hex == "#FF453A")

        // Critical FAA CAMI (`DOT/FAA/AM-06/15`) anti-confusion guard: Guidance Magenta (`plan`) vs Warning Red (`warning`).
        let planVsWarningDe = AvionicsColorSystem.plan.deltaE2000(to: AvionicsColorSystem.warning)
        #expect(planVsWarningDe > 30.0)
        #expect(AvionicsColorSystem.value.contrastRatio(against: AvionicsColorSystem.background) > 16.5)
    }

    @Test("Surveyed FAA Airport Diagrams (KEWR 00285AD.PDF & KMMU 00931AD.PDF) georeference onto the moving map and support dark color inversion")
    func airportDiagramGeoreferencingAndInversion() throws {
        let controller = FlightDeckController(body: .earthWgs84Egm96)

        // 1. KEWR Airport Diagram (`00285AD.PDF` — solved via surveyed NASR runway threshold RANSAC).
        let kewrAd = try #require(controller.availablePlates(forAirport: "KEWR", kind: .airportDiagram).first)
        #expect(kewrAd.supportsMapOverlay)
        let pinnedKewr = try #require(controller.pinPlateToMap(kewrAd))
        #expect(abs(pinnedKewr.centerLatitudeDeg - 40.6895) < 0.03)
        #expect(abs(pinnedKewr.centerLongitudeDeg - (-74.1745)) < 0.03)
        #expect(FileManager.default.fileExists(atPath: pinnedKewr.pngFileUrl.path))

        // 2. Toggle dark-cockpit color inversion on the map plate.
        let dayUrl = pinnedKewr.pngFileUrl
        controller.pinnedPlateInverted = true
        let invUrl = try #require(controller.pinnedMapPlate?.pngFileUrl)
        #expect(invUrl != dayUrl)
        #expect(FileManager.default.fileExists(atPath: invUrl.path))

        // 3. KMMU Airport Diagram (`00931AD.PDF` — solved via printed neatline graticule ticks).
        let kmmuAd = try #require(controller.availablePlates(forAirport: "KMMU", kind: .airportDiagram).first)
        let pinnedKmmu = try #require(controller.pinPlateToMap(kmmuAd))
        #expect(abs(pinnedKmmu.centerLatitudeDeg - 40.799) < 0.03)
        #expect(abs(pinnedKmmu.centerLongitudeDeg - (-74.415)) < 0.03)
    }
}

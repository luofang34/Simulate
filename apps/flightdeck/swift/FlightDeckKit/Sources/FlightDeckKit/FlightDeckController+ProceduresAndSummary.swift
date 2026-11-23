import Foundation

extension FlightDeckController {
    // MARK: - Offline Terminal Procedure Plates (`dtpp_plates.bin`) & Multi-Provider Registry

    /// Cycle validity of the offline FAA 28-day Terminal Procedure Plates pack (`dtpp_plates.bin`).
    public var dtppValidity: AeroPackValidity {
        dataPackStore.validity(for: .faaDtpp)
    }

    /// Whether official FAA `d-TPP` terminal procedure plates are permitted (`false` when expired, offloaded, or off-world).
    public var isDtppUsable: Bool {
        guard primaryBody == .earth else { return false }
        let hasInstalledSeedOrPack = dataPackStore.installedPacks.contains { $0.authority == .faaDtpp }
        if hasInstalledSeedOrPack || dataPackStore.disabledSeedAuthorities.contains(.faaDtpp) {
            return dtppValidity.isValidForUse
        }
        return DtppPlateArchive.shared.isLoaded
    }

    /// Active memory-mapped `DtppPlateArchive` when `isDtppUsable` holds (`nil` when expired, offloaded, or off-world).
    public var activeDtppArchive: DtppPlateArchive? {
        guard isDtppUsable else { return nil }
        let archive = DtppPlateArchive.reader(forPackPath: dtppValidity.activePack?.filePath)
        return archive.isLoaded ? archive : nil
    }

    /// Queries official FAA `d-TPP` plates (`Airport Diagram`, `Approach`, `Arrival`, `Departure`) at `airportCode`.
    public func availablePlates(
        forAirport airportCode: String,
        kind: ProcedurePlateKind? = nil
    ) -> [ProcedurePlateDescriptor] {
        guard let archive = activeDtppArchive else { return [] }
        return archive.plates(forAirport: airportCode, kind: kind)
    }

    /// Resolves the best available plate for a published procedure (official FAA GeoPDF first, procedural planview fallback second).
    public func resolvePlateDescriptor(for procedure: CifpProcedureSummary) -> ProcedurePlateDescriptor {
        if let official = activeDtppArchive?.matchingPlate(for: procedure) {
            return official
        }
        let kind: ProcedurePlateKind = switch procedure.kind {
        case .approach: .approach
        case .star: .arrival
        case .sid: .departure
        }
        return ProcedurePlateDescriptor(
            id: "procedural:\(procedure.airportIdent):\(procedure.procedureIdent)",
            providerId: ChartProviderRegistry.faaAeroNavId,
            providerBadge: "Procedural Planview",
            airportIdent: procedure.airportIdent,
            kind: kind,
            title: procedure.humanTitle,
            pdfName: nil,
            cifpIdent: procedure.procedureIdent,
            isOfficialPdf: false,
            supportsMapOverlay: procedure.kind == .approach
        )
    }

    /// Loads raw PDF bytes for full-screen `PDFKit` viewing (official `d-TPP` PDF or synthesized procedural planview PDF).
    public func loadPlatePdfData(
        for descriptor: ProcedurePlateDescriptor,
        procedure: CifpProcedureSummary? = nil,
        runwayTransitionIdent: String? = nil,
        enrouteOrFeederTransitionIdent: String? = nil
    ) -> Data? {
        if let pdfName = descriptor.pdfName,
           let raw = activeDtppArchive?.pdfData(named: pdfName) {
            return raw
        }
        let resolvedProc = procedure ?? availableCifpProcedures(forAirport: descriptor.airportIdent)
            .first(where: { $0.procedureIdent == descriptor.cifpIdent })
        guard let proc = resolvedProc,
              let db = activeCifpDatabase else {
            return nil
        }
        let legs = db.assembleWaypoints(
            for: proc,
            runwayTransitionIdent: runwayTransitionIdent,
            enrouteOrFeederTransitionIdent: enrouteOrFeederTransitionIdent,
            includeMissedApproach: true
        )
        let (aptLat, aptLon) = airportCoordinates(for: proc.airportIdent, fallbackLegs: legs)
        return GeoPdfPlateRenderer.renderProceduralPdfData(
            procedure: proc,
            airportLatDeg: aptLat,
            airportLonDeg: aptLon,
            legs: legs
        )
    }

    /// Georeferences and drapes `descriptor` directly onto the 2D/3D moving map (`ImageSource` overlay).
    @discardableResult
    public func pinPlateToMap(
        _ descriptor: ProcedurePlateDescriptor,
        procedure: CifpProcedureSummary? = nil,
        runwayTransitionIdent: String? = nil,
        enrouteOrFeederTransitionIdent: String? = nil
    ) -> PinnedMapPlate? {
        guard primaryBody == .earth, descriptor.supportsMapOverlay else { return nil }

        if let pdfName = descriptor.pdfName,
           let pdfBytes = activeDtppArchive?.pdfData(named: pdfName) {
            let fallbackReg: GeoPdfRegistration? = if descriptor.kind == .airportDiagram {
                solveAirportDiagramRegistration(airportCode: descriptor.airportIdent, pdfBytes: pdfBytes)
            } else {
                nil
            }
            if let rendered = GeoPdfPlateRenderer.renderGeoPdfOverlayPng(
                pdfData: pdfBytes,
                cacheKey: "\(descriptor.airportIdent)_\(pdfName)",
                fallbackRegistration: fallbackReg,
                invertColors: pinnedPlateInverted
            ) {
                let pinned = PinnedMapPlate(
                    descriptor: descriptor,
                    pngFileUrl: rendered.pngUrl,
                    cornersLonLat: rendered.registration.cornersLonLat,
                    centerLatitudeDeg: rendered.registration.centerLatitudeDeg,
                    centerLongitudeDeg: rendered.registration.centerLongitudeDeg,
                    procedure: procedure,
                    runwayTransitionIdent: runwayTransitionIdent,
                    enrouteOrFeederTransitionIdent: enrouteOrFeederTransitionIdent
                )
                pinnedMapPlate = pinned
                pinnedPlateVisible = true
                bumpPlateOverlayRevision()
                return pinned
            }
        }

        let resolvedProc = procedure ?? availableCifpProcedures(forAirport: descriptor.airportIdent)
            .first(where: { $0.procedureIdent == descriptor.cifpIdent })
        guard let proc = resolvedProc,
              let db = activeCifpDatabase else {
            return nil
        }
        let legs = db.assembleWaypoints(
            for: proc,
            runwayTransitionIdent: runwayTransitionIdent,
            enrouteOrFeederTransitionIdent: enrouteOrFeederTransitionIdent,
            includeMissedApproach: true
        )
        let (aptLat, aptLon) = airportCoordinates(for: proc.airportIdent, fallbackLegs: legs)
        guard let rendered = GeoPdfPlateRenderer.renderProceduralApproachOverlayPng(
            procedure: proc,
            airportLatDeg: aptLat,
            airportLonDeg: aptLon,
            legs: legs,
            cacheKey: "\(proc.airportIdent)_\(proc.procedureIdent)_\(enrouteOrFeederTransitionIdent ?? "core")",
            invertColors: pinnedPlateInverted
        ) else {
            return nil
        }

        let pinned = PinnedMapPlate(
            descriptor: descriptor,
            pngFileUrl: rendered.pngUrl,
            cornersLonLat: rendered.registration.cornersLonLat,
            centerLatitudeDeg: rendered.registration.centerLatitudeDeg,
            centerLongitudeDeg: rendered.registration.centerLongitudeDeg,
            procedure: proc,
            runwayTransitionIdent: runwayTransitionIdent,
            enrouteOrFeederTransitionIdent: enrouteOrFeederTransitionIdent
        )
        pinnedMapPlate = pinned
        pinnedPlateVisible = true
        bumpPlateOverlayRevision()
        return pinned
    }

    /// Re-renders the active `pinnedMapPlate` PNG when `pinnedPlateInverted` toggles.
    func refreshPinnedPlateForColorMode() {
        guard let current = pinnedMapPlate else { return }
        _ = pinPlateToMap(
            current.descriptor,
            procedure: current.procedure,
            runwayTransitionIdent: current.runwayTransitionIdent,
            enrouteOrFeederTransitionIdent: current.enrouteOrFeederTransitionIdent
        )
    }

    private func solveAirportDiagramRegistration(
        airportCode: String,
        pdfBytes: Data
    ) -> GeoPdfRegistration? {
        let (aptLat, aptLon) = airportCoordinates(for: airportCode, fallbackLegs: [])
        let rwyEnds = surveyedRunwayEnds(forAirport: airportCode, centerLatDeg: aptLat, centerLonDeg: aptLon)
        return AirportDiagramGeoreferencer.solve(
            pdfData: pdfBytes,
            referenceLatDeg: aptLat,
            referenceLonDeg: aptLon,
            runwayEnds: rwyEnds
        )
    }

    private func surveyedRunwayEnds(
        forAirport airportCode: String,
        centerLatDeg: Double,
        centerLonDeg: Double
    ) -> [SurveyedRunwayEnd] {
        let code = airportCode.trimmingCharacters(in: .whitespacesAndNewlines).uppercased()
        let jsonStr = runwaysGeoJson(centerLatitudeDeg: centerLatDeg, centerLongitudeDeg: centerLonDeg, radiusKm: 30.0)
        guard let jsonData = jsonStr.data(using: .utf8),
              let root = try? JSONSerialization.jsonObject(with: jsonData) as? [String: Any],
              let features = root["features"] as? [[String: Any]] else {
            return []
        }

        var centerlines: [((Double, Double), (Double, Double))] = []
        var labels: [(desig: String, lon: Double, lat: Double)] = []
        let prefix = "\(code) "

        for feat in features {
            guard let props = feat["properties"] as? [String: Any],
                  let kind = props["kind"] as? String,
                  let geom = feat["geometry"] as? [String: Any] else {
                continue
            }
            if kind == "centerline",
               let coords = geom["coordinates"] as? [[Double]],
               coords.count >= 2,
               coords[0].count >= 2,
               coords[1].count >= 2 {
                centerlines.append(((coords[0][0], coords[0][1]), (coords[1][0], coords[1][1])))
            } else if kind == "label",
                      let name = props["name"] as? String,
                      name.uppercased().hasPrefix(prefix),
                      let coords = geom["coordinates"] as? [Double],
                      coords.count >= 2 {
                let desig = String(name.uppercased().split(separator: " ").last ?? "")
                if !desig.isEmpty {
                    labels.append((desig, coords[0], coords[1]))
                }
            }
        }

        let cosLat = max(cos(centerLatDeg * .pi / 180.0), 0.15)
        let mPerLon = 111_320.0 * cosLat
        let mPerLat = 111_132.0
        var ends: [SurveyedRunwayEnd] = []

        for lbl in labels {
            var bestNear: (Double, Double) = (lbl.lon, lbl.lat)
            var bestFar: (Double, Double)?
            var bestAlongDist = 1_200.0
            for (endA, endB) in centerlines {
                let vx = (endB.0 - endA.0) * mPerLon
                let vy = (endB.1 - endA.1) * mPerLat
                let vLen = hypot(vx, vy)
                guard vLen > 150.0 else { continue }
                let px = (lbl.lon - endA.0) * mPerLon
                let py = (lbl.lat - endA.1) * mPerLat
                // Cross-track distance from `lbl` (threshold) to this runway centerline must be `< 30 m`
                // so parallel runways (`04L` vs `04R`, 290 m apart with displaced thresholds) never cross-match.
                let crossM = abs(px * vy - py * vx) / vLen
                guard crossM < 30.0 else { continue }

                let dA = hypot((endA.0 - lbl.lon) * mPerLon, (endA.1 - lbl.lat) * mPerLat)
                let dB = hypot((endB.0 - lbl.lon) * mPerLon, (endB.1 - lbl.lat) * mPerLat)
                if dA <= dB, dA < bestAlongDist {
                    bestAlongDist = dA
                    bestNear = endA
                    bestFar = endB
                } else if dB < dA, dB < bestAlongDist {
                    bestAlongDist = dB
                    bestNear = endB
                    bestFar = endA
                }
            }

            var (labelLon, labelLat, outEast, outNorth) = (bestNear.0, bestNear.1, 0.0, 0.0)
            if let far = bestFar {
                let (dxM, dyM) = ((bestNear.0 - far.0) * mPerLon, (bestNear.1 - far.1) * mPerLat)
                let lenM = hypot(dxM, dyM)
                if lenM > 150.0 {
                    (outEast, outNorth) = (dxM / lenM, dyM / lenM)
                    labelLon = bestNear.0 + outEast * 105.0 / mPerLon
                    labelLat = bestNear.1 + outNorth * 105.0 / mPerLat
                }
            }
            ends.append(SurveyedRunwayEnd(
                designator: lbl.desig, longitudeDeg: bestNear.0, latitudeDeg: bestNear.1,
                labelLongitudeDeg: labelLon, labelLatitudeDeg: labelLat,
                outboundEast: outEast, outboundNorth: outNorth
            ))
        }
        return ends
    }

    /// Convenience helper pinning an official plate by `(airportCode, pdfName)` (e.g. `("N51", "05075R22.PDF")`).
    @discardableResult
    public func pinPlateByPdfName(airportCode: String, pdfName: String) -> PinnedMapPlate? {
        let target = pdfName.trimmingCharacters(in: .whitespacesAndNewlines).uppercased()
        guard let match = availablePlates(forAirport: airportCode)
            .first(where: { ($0.pdfName ?? "").uppercased() == target }) else {
            return nil
        }
        return pinPlateToMap(match)
    }

    /// Removes any active procedure plate overlay from the moving map.
    public func clearPinnedMapPlate() {
        guard pinnedMapPlate != nil else { return }
        pinnedMapPlate = nil
        bumpPlateOverlayRevision()
    }

    /// Suppresses any pinned map plate immediately if its backing 28-day AIRAC pack expires or is offloaded.
    func enforcePinnedPlateValidity() {
        guard let current = pinnedMapPlate else { return }
        if primaryBody != .earth {
            clearPinnedMapPlate()
            return
        }
        if current.descriptor.isOfficialPdf && !isDtppUsable {
            clearPinnedMapPlate()
        } else if !current.descriptor.isOfficialPdf && !isCifpUsable {
            clearPinnedMapPlate()
        }
    }

    private func airportCoordinates(
        for airportCode: String,
        fallbackLegs: [FlightPlanWaypoint]
    ) -> (Double, Double) {
        if let apt = lookupAirport(airportCode) {
            return (apt.latitudeDeg, apt.longitudeDeg)
        }
        if let pt = resolveNavigationIdentifier(airportCode) {
            return (pt.latitudeDeg, pt.longitudeDeg)
        }
        if let last = fallbackLegs.last {
            return (last.latitudeDeg, last.longitudeDeg)
        }
        return (40.5829, -74.7364)
    }

    // MARK: - Automatic Leg Sequencing & Minimized `FPL` Navigation / Time / Fuel Summary

    /// Advances `activeWaypointIndex` automatically when ownship passes within `0.85 NM` of the active fix
    /// and enters the outbound sector toward the subsequent fix.
    func advanceWaypointSequencingIfNeeded(ownship: OwnshipSnapshot) {
        guard flightPlanWaypoints.count >= 2,
              activeWaypointIndex >= 0,
              activeWaypointIndex + 1 < flightPlanWaypoints.count,
              let ownLat = ownship.latitudeDeg,
              let ownLon = ownship.longitudeDeg,
              (ownship.groundSpeedKt ?? 0.0) >= 20.0 else {
            return
        }

        let r = bodyParameters.renderRadiusMeters
        let activeWp = flightPlanWaypoints[activeWaypointIndex]
        let nextWp = flightPlanWaypoints[activeWaypointIndex + 1]
        let toActive = RouteLegMetrics.between(
            lat1Deg: ownLat,
            lon1Deg: ownLon,
            lat2Deg: activeWp.latitudeDeg,
            lon2Deg: activeWp.longitudeDeg,
            radiusMeters: r
        )
        let distNm = toActive.distanceNauticalMiles
        guard distNm <= 0.85 else { return }

        if distNm <= 0.35 {
            activeWaypointIndex += 1
            bumpRouteOverlayRevision()
            return
        }

        let outbound = RouteLegMetrics.between(
            lat1Deg: activeWp.latitudeDeg,
            lon1Deg: activeWp.longitudeDeg,
            lat2Deg: nextWp.latitudeDeg,
            lon2Deg: nextWp.longitudeDeg,
            radiusMeters: r
        )
        let fromActiveToOwn = RouteLegMetrics.between(
            lat1Deg: activeWp.latitudeDeg,
            lon1Deg: activeWp.longitudeDeg,
            lat2Deg: ownLat,
            lon2Deg: ownLon,
            radiusMeters: r
        )
        let deltaRad = (fromActiveToOwn.initialBearingTrueDeg - outbound.initialBearingTrueDeg) * .pi / 180.0
        if cos(deltaRad) >= -0.10 {
            activeWaypointIndex += 1
            bumpRouteOverlayRevision()
        }
    }

    /// Computes the high-priority pilot scan summary (`Origin`, `Current`, `Next`, `Dest`, `BRG`, `DIST`, `ETE`, `FUEL`)
    /// displayed in the minimized bottom `FPL` strip.
    public func compactFlightPlanSummary(from ownship: OwnshipSnapshot?) -> CompactFlightPlanSummary {
        guard !flightPlanWaypoints.isEmpty else {
            return CompactFlightPlanSummary(hasRoute: false)
        }

        let count = flightPlanWaypoints.count
        let clampedActive = min(max(activeWaypointIndex, 0), count - 1)
        let currentWp = flightPlanWaypoints[clampedActive]

        let originIdent: String? = (clampedActive > 0) ? flightPlanWaypoints[0].ident : nil
        let hasGapAfterOrigin = clampedActive > 1
        let nextIdent: String? = (clampedActive + 1 < count) ? flightPlanWaypoints[clampedActive + 1].ident : nil
        let destinationIdent: String? = (clampedActive + 2 < count) ? flightPlanWaypoints[count - 1].ident : nil
        let rwyBadge: String? = (primaryBody == .earth && !(destinationRunway ?? "").isEmpty) ? destinationRunway : nil

        let r = bodyParameters.renderRadiusMeters
        let legMetrics: RouteLegMetrics? = {
            if let ownLat = ownship?.latitudeDeg, let ownLon = ownship?.longitudeDeg {
                return RouteLegMetrics.between(
                    lat1Deg: ownLat,
                    lon1Deg: ownLon,
                    lat2Deg: currentWp.latitudeDeg,
                    lon2Deg: currentWp.longitudeDeg,
                    radiusMeters: r
                )
            } else if clampedActive > 0 {
                let prev = flightPlanWaypoints[clampedActive - 1]
                return RouteLegMetrics.between(
                    lat1Deg: prev.latitudeDeg,
                    lon1Deg: prev.longitudeDeg,
                    lat2Deg: currentWp.latitudeDeg,
                    lon2Deg: currentWp.longitudeDeg,
                    radiusMeters: r
                )
            }
            return nil
        }()

        var tailMeters = 0.0
        if clampedActive + 1 < count {
            for idx in (clampedActive + 1)..<count {
                let a = flightPlanWaypoints[idx - 1]
                let b = flightPlanWaypoints[idx]
                tailMeters += RouteLegMetrics.between(
                    lat1Deg: a.latitudeDeg,
                    lon1Deg: a.longitudeDeg,
                    lat2Deg: b.latitudeDeg,
                    lon2Deg: b.longitudeDeg,
                    radiusMeters: r
                ).distanceMeters
            }
        }

        let legNm = legMetrics?.distanceNauticalMiles
        let remNm: Double? = legNm.map { $0 + (tailMeters / 1_852.0) }
        let hasBeyond = (clampedActive + 1 < count) && (tailMeters > 90.0)

        let liveGs = ownship?.groundSpeedKt ?? 0.0
        let useLiveGs = liveGs >= 25.0
        let effectiveSpeedKt = useLiveGs ? liveGs : max(plannedCruiseKtas, 25.0)

        let eteFormatted: String?
        let fuelGal: Double?
        if let rem = remNm, rem >= 0.05, effectiveSpeedKt >= 10.0 {
            let hours = rem / effectiveSpeedKt
            let totalMinutes = min(max(Int((hours * 60.0).rounded()), 0), 5999)
            eteFormatted = String(format: "%d:%02d", totalMinutes / 60, totalMinutes % 60)
            if primaryBody == .earth && plannedFuelFlowGph > 0.1 {
                fuelGal = hours * plannedFuelFlowGph
            } else {
                fuelGal = nil
            }
        } else {
            eteFormatted = nil
            fuelGal = nil
        }

        return CompactFlightPlanSummary(
            hasRoute: true,
            originIdent: originIdent,
            hasSequencedGapAfterOrigin: hasGapAfterOrigin,
            currentIdent: currentWp.ident,
            nextIdent: nextIdent,
            destinationIdent: destinationIdent,
            runwayBadge: rwyBadge,
            bearingTrueDeg: legMetrics?.initialBearingTrueDeg,
            legDistanceNm: legNm,
            remainingDistanceNm: remNm,
            hasRemainingLegsBeyondCurrent: hasBeyond,
            eteFormatted: eteFormatted,
            fuelRequiredGal: fuelGal,
            usesLiveGroundSpeed: useLiveGs
        )
    }
}

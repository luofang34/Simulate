import Foundation
import flightdeck_ffiFFI

extension FlightDeckController {
    /// Resolves an airport identifier (`"KHSV"`, `"KMEM"`, `"N51"`, `"KEWR"`, `"EGLL"`, etc.) from `AirportDatabase`
    /// on `WGS-84` Earth (`nil` when unknown or when a non-Earth body is active).
    public func lookupAirport(_ code: String) -> ResolvedAirport? {
        let trimmed = code.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty, celestialBody.isEarthWgs84 else { return nil }
        let rawJson: Data? = sessionHandle.withSession { session in
            let utf8 = Array(trimmed.utf8)
            return utf8.withUnsafeBufferPointer { bufPtr in
                let out = flightdeck_session_lookup_airport_json(session, bufPtr.baseAddress, bufPtr.count)
                defer { flightdeck_byte_buffer_free(out) }
                guard let ptr = out.ptr, out.len > 2 else { return nil }
                return Data(bytes: ptr, count: out.len)
            }
        } ?? nil
        guard let data = rawJson,
              let obj = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any],
              let primary = obj["code"] as? String,
              let lat = obj["latitude"] as? Double,
              let lon = obj["longitude"] as? Double else {
            return nil
        }
        let name = (obj["name"] as? String) ?? primary
        let city = (obj["municipality"] as? String) ?? ""
        let elev = (obj["elevation_m"] as? Double) ?? 0.0
        let towered = (obj["towered"] as? Bool) ?? false
        let longest = (obj["longest_runway_m"] as? Double) ?? 0.0
        let runways = (obj["runways"] as? [String]) ?? []
        return ResolvedAirport(
            code: primary,
            name: name,
            municipality: city,
            latitudeDeg: lat,
            longitudeDeg: lon,
            elevationDatumM: elev,
            towered: towered,
            longestRunwayM: longest,
            runways: runways
        )
    }

    /// Cycle validity of the offline FAA NASR navigation database (`nasr_waypoints.bin`).
    public var earthNavdataValidity: AeroPackValidity {
        dataPackStore.validity(for: .faaNasrWaypoints)
    }

    /// Whether Earth NASR airport/navaid/fix lookups are permitted (`false` when expired or offloaded).
    public var isEarthNavdataUsable: Bool {
        let hasInstalledSeedOrPack = dataPackStore.installedPacks.contains { $0.authority == .faaNasrWaypoints }
        if hasInstalledSeedOrPack || dataPackStore.disabledSeedAuthorities.contains(.faaNasrWaypoints) {
            return earthNavdataValidity.isValidForUse
        }
        return true
    }

    /// Cycle validity of the offline FAA 28-day CIFP database (`cifp_procedures.bin`).
    public var cifpValidity: AeroPackValidity {
        dataPackStore.validity(for: .faaCifp)
    }

    /// Whether FAA CIFP airways, SIDs, STARs, and approaches are permitted (`false` when expired or offloaded).
    public var isCifpUsable: Bool {
        guard primaryBody == .earth else { return false }
        let hasInstalledSeedOrPack = dataPackStore.installedPacks.contains { $0.authority == .faaCifp }
        if hasInstalledSeedOrPack || dataPackStore.disabledSeedAuthorities.contains(.faaCifp) {
            return cifpValidity.isValidForUse
        }
        return CifpProcedureDatabase.shared.isLoaded
    }

    /// Active memory-mapped `CifpProcedureDatabase` when `isCifpUsable` holds (`nil` when expired, offloaded, or off-world).
    public var activeCifpDatabase: CifpProcedureDatabase? {
        guard isCifpUsable else { return nil }
        let db = CifpProcedureDatabase.reader(forPackPath: cifpValidity.activePack?.filePath)
        return db.isLoaded ? db : nil
    }

    /// Searches airports, VOR/VORTAC/NDB navaids, RNAV/VFR fixes, `LAT/LON` coordinates, or planetary sites.
    public func searchNavigationPoints(
        _ query: String,
        near reference: FlightPlanWaypoint? = nil,
        limit: Int = 24
    ) -> [NavigationPoint] {
        let refLat = reference?.latitudeDeg ?? flightPlanWaypoints.last?.latitudeDeg ?? latestFrame?.ownship.latitudeDeg
        let refLon = reference?.longitudeDeg ?? flightPlanWaypoints.last?.longitudeDeg ?? latestFrame?.ownship.longitudeDeg
        let allowEarthDb = isEarthNavdataUsable
        return NavigationDatabaseIndex.shared.search(
            query: query,
            primaryBody: primaryBody,
            allowEarthDatabase: allowEarthDb,
            nearLatDeg: refLat,
            nearLonDeg: refLon,
            radiusMeters: bodyParameters.renderRadiusMeters,
            airportLookup: { [weak self] code in
                guard allowEarthDb else { return nil }
                return self?.lookupAirport(code)
            },
            limit: limit
        )
    }

    /// Resolves a single route token (`"KEWR"`, `"SBJ"`, `"CUTOP"`, `"38.85N/77.04W"`, etc.) to its best match.
    public func resolveNavigationIdentifier(
        _ rawToken: String,
        near reference: FlightPlanWaypoint? = nil
    ) -> NavigationPoint? {
        let matches = searchNavigationPoints(rawToken, near: reference, limit: 8)
        guard let normalized = RouteAsciiInput.normalized(rawToken)?
            .trimmingCharacters(in: .whitespaces),
              !normalized.isEmpty else {
            return nil
        }
        if let coord = RouteAsciiInput.parseCoordinate(normalized) {
            return coord
        }
        if let exact = matches.first(where: { $0.identifier == normalized }) {
            return exact
        }
        if normalized.count == 3,
           let kExact = matches.first(where: { $0.identifier == "K" + normalized && $0.kind == .airport }) {
            return kExact
        }
        return nil
    }

    /// Replaces the entire ordered waypoint array (used by drag-reorder, backspace-delete, and inline insert/edit).
    public func replaceFlightPlanWaypoints(
        _ updated: [FlightPlanWaypoint],
        destinationRunway: String? = nil
    ) {
        let previousActiveId: UUID? = (activeWaypointIndex >= 0 && activeWaypointIndex < flightPlanWaypoints.count)
            ? flightPlanWaypoints[activeWaypointIndex].id
            : nil

        flightPlanWaypoints = updated

        if updated.isEmpty {
            activeWaypointIndex = 0
            self.destinationRunway = nil
        } else if let prevId = previousActiveId,
                  let preserved = updated.firstIndex(where: { $0.id == prevId }) {
            activeWaypointIndex = preserved
        } else {
            activeWaypointIndex = updated.count >= 2 ? 1 : 0
        }

        let destAirport = updated.last(where: { $0.kind == .airport && !$0.runways.isEmpty })
        if let explicit = destinationRunway, !explicit.isEmpty {
            self.destinationRunway = explicit
        } else if let destRwys = destAirport?.runways {
            if let cur = self.destinationRunway, destRwys.contains(cur) {
                // Keep existing valid runway selection.
            } else {
                self.destinationRunway = destRwys.first
            }
        } else {
            self.destinationRunway = nil
        }
        syncFlightPlanToSession()
    }

    /// Replaces the active flight plan with an ordered sequence of identifiers or airway tokens
    /// (`"KEWR"`, `"SBJ"`, `"V3"`, `"MXE"`, `"SBJ.V3.MXE"`, `"38.85N/77.04W"`, etc.).
    public func setFlightPlanCodes(_ codes: [String], destinationRunway: String? = nil) {
        let result = appendRouteTokensWithCifp(codes, onto: [])
        replaceFlightPlanWaypoints(result.updated, destinationRunway: destinationRunway)
    }

    /// Resolves and appends an ordered token stream onto `existing`, expanding CIFP airways in both
    /// space-separated (`SBJ V3 MXE`) and dot-separated (`SBJ.V3.MXE`) ICAO/ForeFlight syntax.
    public func appendRouteTokensWithCifp(
        _ rawTokens: [String],
        onto existing: [FlightPlanWaypoint]
    ) -> (updated: [FlightPlanWaypoint], unresolved: [String], expandedNote: String?) {
        // Normalize and split dot-chained airway expressions (`SBJ.V3.MXE` -> `["SBJ", "V3", "MXE"]`) while
        // keeping decimal coordinates (`40.58/-74.70` or `38.85N/77.04W`) intact.
        var flatTokens: [String] = []
        for raw in rawTokens {
            guard let norm = RouteAsciiInput.normalized(raw)?.trimmingCharacters(in: .whitespaces),
                  !norm.isEmpty else {
                continue
            }
            for word in RouteAsciiInput.words(from: norm) {
                if !word.contains("/"),
                   word.contains("."),
                   RouteAsciiInput.parseCoordinate(word) == nil {
                    let dotParts = word.split(separator: ".").map(String.init).filter { !$0.isEmpty && $0 != "DCT" }
                    if dotParts.count >= 3 {
                        flatTokens.append(contentsOf: dotParts)
                        continue
                    }
                }
                flatTokens.append(word)
            }
        }

        var updated = existing
        var unresolved: [String] = []
        var expandedSummaries: [String] = []
        let cifpDb = activeCifpDatabase

        var idx = 0
        while idx < flatTokens.count {
            let token = flatTokens[idx]

            // Check if `token` is a CIFP airway linking `updated.last` -> `flatTokens[idx + 1]`.
            if let db = cifpDb,
               let entryIdent = updated.last?.ident,
               idx + 1 < flatTokens.count,
               db.lookupAirway(token) != nil {
                let exitToken = flatTokens[idx + 1]
                if let expanded = db.expandAirway(routeIdent: token, from: entryIdent, to: exitToken, includeEntry: false),
                   !expanded.isEmpty {
                    let enriched = expanded.map { enrichCifpWaypoint($0) }
                    updated.append(contentsOf: enriched)
                    expandedSummaries.append("\(token) (\(enriched.count) fixes → \(exitToken))")
                    idx += 2
                    continue
                }
            }

            let ref = updated.last
            if let pt = resolveSingleWaypointToken(token, near: ref) {
                if updated.last?.ident != pt.ident {
                    updated.append(pt)
                }
            } else {
                unresolved.append(token)
            }
            idx += 1
        }

        let note = expandedSummaries.isEmpty ? nil : "Expanded " + expandedSummaries.joined(separator: ", ")
        return (updated, unresolved, note)
    }

    private func resolveSingleWaypointToken(
        _ token: String,
        near reference: FlightPlanWaypoint?
    ) -> FlightPlanWaypoint? {
        if let pt = resolveNavigationIdentifier(token, near: reference)
            ?? searchNavigationPoints(token, near: reference, limit: 1).first(where: { $0.identifier == token }) {
            return FlightPlanWaypoint(point: pt)
        }
        if isEarthNavdataUsable, let apt = lookupAirport(token) {
            return FlightPlanWaypoint(
                pointKey: "apt:\(apt.code)",
                ident: apt.code,
                kind: .airport,
                name: apt.municipality.isEmpty ? apt.name : "\(apt.name) · \(apt.municipality)",
                region: "",
                latitudeDeg: apt.latitudeDeg,
                longitudeDeg: apt.longitudeDeg,
                altitudeDatumM: apt.elevationDatumM,
                runways: apt.runways
            )
        }
        return nil
    }

    private func enrichCifpWaypoint(_ wp: FlightPlanWaypoint) -> FlightPlanWaypoint {
        guard isEarthNavdataUsable,
              let resolved = resolveNavigationIdentifier(wp.ident, near: wp) else {
            return wp
        }
        let distM = RouteLegMetrics.between(
            lat1Deg: wp.latitudeDeg,
            lon1Deg: wp.longitudeDeg,
            lat2Deg: resolved.latitudeDeg,
            lon2Deg: resolved.longitudeDeg,
            radiusMeters: bodyParameters.renderRadiusMeters
        ).distanceMeters
        guard distM <= 5_000.0 else { return wp }
        var enriched = wp
        enriched.kind = resolved.kind
        if wp.airwayOrProcedureTag.isEmpty || wp.name.hasPrefix("Airway ") {
            enriched.name = resolved.name
        }
        if enriched.region.isEmpty {
            enriched.region = resolved.region
        }
        return enriched
    }

    // MARK: - CIFP Terminal Procedures (`SID` / `STAR` / `Approach`)

    /// First airport waypoint in the active flight plan (`nil` when route has no origin airport).
    public var departureAirportWaypoint: FlightPlanWaypoint? {
        flightPlanWaypoints.first(where: { $0.kind == .airport })
    }

    /// Queries CIFP procedures (`SID`, `STAR`, or `Approach`) at `airportCode` when `isCifpUsable` holds.
    public func availableCifpProcedures(
        forAirport airportCode: String,
        kind: CifpProcedureKind? = nil
    ) -> [CifpProcedureSummary] {
        guard let db = activeCifpDatabase else { return [] }
        return db.procedures(forAirport: airportCode, kind: kind)
    }

    /// Inserts assembled CIFP `SID`, `STAR`, or `Approach` legs into the active flight plan and aligns the
    /// destination 3D runway designator when an instrument approach specifies a landing runway.
    @discardableResult
    public func applyCifpProcedure(
        _ procedure: CifpProcedureSummary,
        runwayTransitionIdent: String? = nil,
        enrouteOrFeederTransitionIdent: String? = nil,
        includeMissedApproach: Bool = false
    ) -> Int {
        guard let db = activeCifpDatabase else { return 0 }
        let rawLegs = db.assembleWaypoints(
            for: procedure,
            runwayTransitionIdent: runwayTransitionIdent,
            enrouteOrFeederTransitionIdent: enrouteOrFeederTransitionIdent,
            includeMissedApproach: includeMissedApproach
        )
        guard !rawLegs.isEmpty else { return 0 }

        var current = flightPlanWaypoints
        var explicitRwy: String?

        switch procedure.kind {
        case .sid:
            // Remove any prior SID legs.
            current.removeAll { $0.airwayOrProcedureTag.hasPrefix("SID ") }
            let insertAt: Int
            if let depIdx = current.firstIndex(where: { $0.kind == .airport }) {
                insertAt = depIdx + 1
            } else {
                if let aptPt = resolveSingleWaypointToken(procedure.airportIdent, near: nil) {
                    current.insert(aptPt, at: 0)
                    insertAt = 1
                } else {
                    insertAt = 0
                }
            }
            var toInsert = rawLegs
            if insertAt > 0, current[insertAt - 1].ident == toInsert.first?.ident {
                toInsert.removeFirst()
            }
            if insertAt < current.count, current[insertAt].ident == toInsert.last?.ident {
                current[insertAt].airwayOrProcedureTag = procedure.shortBadge
                if current[insertAt].altitudeConstraintTag.isEmpty, let lastTag = toInsert.last?.altitudeConstraintTag {
                    current[insertAt].altitudeConstraintTag = lastTag
                }
                toInsert.removeLast()
            }
            current.insert(contentsOf: toInsert, at: insertAt)

        case .star:
            current.removeAll { $0.airwayOrProcedureTag.hasPrefix("STAR ") }
            // Insert before any existing approach legs or before the destination airport.
            let insertAt: Int
            if let firstIap = current.firstIndex(where: {
                !$0.airwayOrProcedureTag.isEmpty
                    && !$0.airwayOrProcedureTag.hasPrefix("SID ")
                    && !$0.airwayOrProcedureTag.hasPrefix("STAR ")
                    && $0.pointKey.hasPrefix("cifp:")
            }) {
                insertAt = firstIap
            } else if let destIdx = current.lastIndex(where: { $0.kind == .airport }), destIdx > 0 {
                insertAt = destIdx
            } else {
                insertAt = current.count
            }
            var toInsert = rawLegs
            if insertAt > 0, current[insertAt - 1].ident == toInsert.first?.ident {
                current[insertAt - 1].airwayOrProcedureTag = procedure.shortBadge
                toInsert.removeFirst()
            }
            if insertAt < current.count, current[insertAt].ident == toInsert.last?.ident {
                toInsert.removeLast()
            }
            current.insert(contentsOf: toInsert, at: insertAt)

        case .approach:
            // Remove any prior approach legs (`cifp:` points that are neither `SID` nor `STAR`).
            current.removeAll {
                $0.pointKey.hasPrefix("cifp:")
                    && !$0.airwayOrProcedureTag.hasPrefix("SID ")
                    && !$0.airwayOrProcedureTag.hasPrefix("STAR ")
            }
            // Ensure the destination airport sits at the end of the route so 3D runway threshold guidance renders.
            let destIdx: Int
            if let existingDest = current.lastIndex(where: {
                $0.kind == .airport && ($0.ident == procedure.airportIdent || "K" + $0.ident == procedure.airportIdent)
            }) {
                destIdx = existingDest
            } else if let aptPt = resolveSingleWaypointToken(procedure.airportIdent, near: current.last) {
                current.append(aptPt)
                destIdx = current.count - 1
            } else {
                destIdx = current.count
            }

            var toInsert = rawLegs
            if destIdx > 0, current[destIdx - 1].ident == toInsert.first?.ident {
                if !toInsert[0].altitudeConstraintTag.isEmpty {
                    current[destIdx - 1].altitudeConstraintTag = toInsert[0].altitudeConstraintTag
                }
                current[destIdx - 1].airwayOrProcedureTag = procedure.shortBadge
                toInsert.removeFirst()
            }
            current.insert(contentsOf: toInsert, at: destIdx)

            // Match `associatedRunwayDesignator` (`"22"`, `"04"`, `"04R"`) against surveyed airport runways.
            if let rwyHint = procedure.associatedRunwayDesignator,
               let destApt = current.last(where: { $0.kind == .airport && !$0.runways.isEmpty }) {
                let strippedHint = rwyHint.hasPrefix("0") ? String(rwyHint.dropFirst()) : rwyHint
                if let exact = destApt.runways.first(where: { $0 == rwyHint || $0 == strippedHint }) {
                    explicitRwy = exact
                }
            }
        }

        replaceFlightPlanWaypoints(current, destinationRunway: explicitRwy)
        return rawLegs.count
    }

    /// Appends any resolved navigation point (airport, VOR, NDB, fix, or coordinate) to the active flight plan.
    @discardableResult
    public func appendFlightPlanPoint(_ point: NavigationPoint) -> Bool {
        var updated = flightPlanWaypoints
        updated.append(FlightPlanWaypoint(point: point))
        replaceFlightPlanWaypoints(updated)
        return true
    }

    /// Appends an identifier resolved against the active navigation database.
    @discardableResult
    public func appendFlightPlanAirport(_ code: String) -> Bool {
        guard let pt = resolveNavigationIdentifier(code, near: flightPlanWaypoints.last) else {
            return false
        }
        return appendFlightPlanPoint(pt)
    }

    /// Removes the waypoint at `index` and keeps `activeWaypointIndex` valid.
    public func removeFlightPlanWaypoint(at index: Int) {
        guard index >= 0, index < flightPlanWaypoints.count else { return }
        var updated = flightPlanWaypoints
        updated.remove(at: index)
        replaceFlightPlanWaypoints(updated)
    }

    /// Activates Direct-To guidance to `flightPlanWaypoints[index]`.
    public func activateDirectTo(index: Int) {
        guard index >= 0, index < flightPlanWaypoints.count else { return }
        activeWaypointIndex = index
        gcsState.activeGuidanceMode = "DTO \(flightPlanWaypoints[index].ident)"
        syncFlightPlanToSession()
    }

    /// Selects the arrival runway designator for the destination airport.
    public func selectDestinationRunway(_ designator: String?) {
        destinationRunway = designator
        syncFlightPlanToSession()
    }

    /// Clears all flight plan waypoints and session route prioritization.
    public func clearFlightPlan() {
        replaceFlightPlanWaypoints([])
    }

    /// Synchronizes airport codes along the route into `FlightDeckSession` so destination 3D runways & HUD stay aligned.
    public func syncFlightPlanToSession() {
        let airportIdents = flightPlanWaypoints
            .filter { $0.kind == .airport }
            .map(\.ident)
        let label = airportIdents.joined(separator: " ")
        let rwySignature = "\(label)|\(destinationRunway ?? "")"
        let utf8 = Array(label.utf8)
        _ = sessionHandle.withSession { session in
            utf8.withUnsafeBufferPointer { ptr in
                flightdeck_session_set_flight_plan_route(session, ptr.baseAddress, ptr.count)
            }
        }
        bumpRouteOverlayRevision()
        if rwySignature != lastAirportRouteSignature {
            lastAirportRouteSignature = rwySignature
            bumpRunwayOverlayRevision()
        }
    }

    /// Last airport waypoint in the route that carries runway geometry (`nil` if route has no airports).
    public var destinationAirportWaypoint: FlightPlanWaypoint? {
        flightPlanWaypoints.last(where: { $0.kind == .airport && !$0.runways.isEmpty })
    }

    /// Concise route string (`"KEWR → SBJ → N51"` or `"No Route"`).
    public var flightPlanRouteSummary: String {
        if isPreviewingTrack, !kmlPlayback.route.isEmpty {
            return kmlPlayback.route
        }
        guard !flightPlanWaypoints.isEmpty else { return "No Route" }
        if flightPlanWaypoints.count <= 4 {
            return flightPlanWaypoints.map(\.ident).joined(separator: " → ")
        }
        let first = flightPlanWaypoints.first?.ident ?? ""
        let last = flightPlanWaypoints.last?.ident ?? ""
        return "\(first) → ··· (\(flightPlanWaypoints.count)) → \(last)"
    }

    /// Sum of great-circle leg distances across `flightPlanWaypoints` in nautical miles.
    public var totalFlightPlanDistanceNm: Double {
        guard flightPlanWaypoints.count >= 2 else { return 0.0 }
        let r = bodyParameters.renderRadiusMeters
        var sumM = 0.0
        for idx in 1..<flightPlanWaypoints.count {
            let a = flightPlanWaypoints[idx - 1]
            let b = flightPlanWaypoints[idx]
            sumM += RouteLegMetrics.between(
                lat1Deg: a.latitudeDeg,
                lon1Deg: a.longitudeDeg,
                lat2Deg: b.latitudeDeg,
                lon2Deg: b.longitudeDeg,
                radiusMeters: r
            ).distanceMeters
        }
        return sumM / 1_852.0
    }

    /// Bearing and distance from `ownship` (or prior waypoint) to the active waypoint.
    public func activeWaypointMetrics(from ownship: OwnshipSnapshot?) -> (ident: String, metrics: RouteLegMetrics)? {
        guard !flightPlanWaypoints.isEmpty else { return nil }
        let idx = min(max(activeWaypointIndex, 0), flightPlanWaypoints.count - 1)
        let target = flightPlanWaypoints[idx]
        let fromLat: Double
        let fromLon: Double
        if let lat = ownship?.latitudeDeg, let lon = ownship?.longitudeDeg {
            fromLat = lat
            fromLon = lon
        } else if idx > 0 {
            fromLat = flightPlanWaypoints[idx - 1].latitudeDeg
            fromLon = flightPlanWaypoints[idx - 1].longitudeDeg
        } else {
            return nil
        }
        let metrics = RouteLegMetrics.between(
            lat1Deg: fromLat,
            lon1Deg: fromLon,
            lat2Deg: target.latitudeDeg,
            lon2Deg: target.longitudeDeg,
            radiusMeters: bodyParameters.renderRadiusMeters
        )
        return (target.ident, metrics)
    }

    /// Generates a GeoJSON `FeatureCollection` of active/planned route legs, typed waypoints, and KML preview polylines.
    public func flightPlanGeoJson() -> String {
        var features: [[String: Any]] = []

        if isPreviewingTrack, kmlPlayback.coordinates.count >= 2 {
            let step = max(kmlPlayback.coordinates.count / 360, 1)
            var coords: [[Double]] = []
            var idx = 0
            while idx < kmlPlayback.coordinates.count {
                let pt = kmlPlayback.coordinates[idx]
                coords.append([pt.x, pt.y])
                idx += step
            }
            if let last = kmlPlayback.coordinates.last,
               coords.last != [last.x, last.y] {
                coords.append([last.x, last.y])
            }
            features.append([
                "type": "Feature",
                "properties": ["kind": "kml_track"],
                "geometry": ["type": "LineString", "coordinates": coords],
            ])
        }

        if flightPlanWaypoints.count >= 2 {
            let activeIdx = min(max(activeWaypointIndex, 1), flightPlanWaypoints.count - 1)
            for idx in 1..<flightPlanWaypoints.count {
                let a = flightPlanWaypoints[idx - 1]
                let b = flightPlanWaypoints[idx]
                let kind: String
                if idx < activeIdx {
                    kind = "past_leg"
                } else if idx == activeIdx {
                    kind = "active_leg"
                } else {
                    kind = "planned_leg"
                }
                features.append([
                    "type": "Feature",
                    "properties": ["kind": kind],
                    "geometry": [
                        "type": "LineString",
                        "coordinates": [
                            [a.longitudeDeg, a.latitudeDeg],
                            [b.longitudeDeg, b.latitudeDeg],
                        ],
                    ],
                ])
            }
        }

        let clampedActiveIdx = flightPlanWaypoints.count >= 2
            ? min(max(activeWaypointIndex, 1), flightPlanWaypoints.count - 1)
            : max(activeWaypointIndex, 0)
        for (idx, wp) in flightPlanWaypoints.enumerated() {
            let isActive = (idx == activeWaypointIndex)
            let seqState: String = if isActive {
                "active"
            } else if flightPlanWaypoints.count >= 2 && idx + 1 < clampedActiveIdx {
                "past"
            } else {
                "upcoming"
            }
            features.append([
                "type": "Feature",
                "properties": [
                    "kind": isActive ? "active_waypoint" : "waypoint",
                    "seq_state": seqState,
                    "wp_kind": wp.kind.rawValue,
                    "badge": wp.kind.badge,
                    "ident": wp.ident,
                    "label": wp.ident,
                ],
                "geometry": [
                    "type": "Point",
                    "coordinates": [wp.longitudeDeg, wp.latitudeDeg],
                ],
            ])
        }

        let root: [String: Any] = [
            "type": "FeatureCollection",
            "features": features,
        ]
        guard let data = try? JSONSerialization.data(withJSONObject: root),
              let str = String(data: data, encoding: .utf8) else {
            return "{\"type\":\"FeatureCollection\",\"features\":[]}"
        }
        return str
    }

    /// Switches the active EFB / GCS receiver family and enforces honest control-lease rules.
    public func selectReceiverFamily(_ family: AvionicsReceiverFamily) {
        gcsState.receiverFamily = family
        if !family.supportsControlUplink {
            gcsState.hasControlLease = false
            gcsState.armTelegraphArmed = false
        }
        switch family {
        case .wifiGdl90:
            if !isGdl90Listening {
                toggleGdl90Listener()
            } else {
                sourceMode = .gdl90LiveUdp
            }
            gcsState.lastCommandNote = "Listening for GDL-90 ADS-B / AHRS on UDP :\(gdl90Port)"
        case .usbAdsBStub:
            if isGdl90Listening {
                toggleGdl90Listener()
            }
            gcsState.lastCommandNote = "USB ADS-B (1090ES/978 UAT) DriverKit transport stub — no USB SDR enumerated"
        case .sokolyGcsTcp:
            sourceMode = .connextLiveTcp
            gcsState.lastCommandNote = "Sokoly / Connext TCP link target \(connextHost):\(connextPort)"
        case .simulationBench:
            if sourceMode == .connextLiveTcp || sourceMode == .gdl90LiveUdp || sourceMode == .kmlReplay {
                sourceMode = .njCorridorCruise
            }
            gcsState.lastCommandNote = "Deterministic 6-DoF simulation bench active"
        }
    }

    /// Executes a `Sokoly-App` GCS guidance command when `gcsState.canActuate` holds.
    @discardableResult
    public func executeGcsCommand(_ command: String) -> Bool {
        guard gcsState.canActuate(isKmlPreview: isPreviewingTrack) else {
            gcsState.lastCommandNote = gcsState.gatingReason(isKmlPreview: isPreviewingTrack) ?? "Command gated"
            return false
        }
        gcsState.activeGuidanceMode = command
        gcsState.lastCommandNote = "Dispatched \(command) under active Control Lease"
        return true
    }
}

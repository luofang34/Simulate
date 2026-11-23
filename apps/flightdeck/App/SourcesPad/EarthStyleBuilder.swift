import Foundation
import FlightDeckKit

/// Deterministic MapLibre v8 style compiler separating Base Surface (`Terrain` vs `Satellite`),
/// Offline Aeronautical Chart Packs (`Off`, `US VFR`, `IFR Low`, `IFR High` with strict cycle gating),
/// granular 3D NASR Airspace portrayal, surveyed airport runways, and typed `FPL` route waypoints.
enum EarthStyleBuilder {
    static let emptyFeatureCollection = "{\"type\":\"FeatureCollection\",\"features\":[]}"

    static func hasValidAeroChart(
        config: PadLayerConfiguration,
        resolvedPacks: ResolvedAeroPackSet
    ) -> Bool {
        switch config.aeroChart {
        case .none:
            return false
        case .usVfr:
            return resolvedPacks.vfrSectional != nil || resolvedPacks.vfrTerminal != nil
        case .ifrLow:
            return resolvedPacks.ifrLow != nil
        case .ifrHigh:
            return resolvedPacks.ifrHigh != nil
        }
    }

    static let defaultPlateCornersFlat: [Double] = [
        0.0, 0.01,
        0.01, 0.01,
        0.01, 0.0,
        0.0, 0.0,
    ]

    static func buildEarthStyleJson(
        config: PadLayerConfiguration,
        resolvedPacks: ResolvedAeroPackSet,
        runwaysGeoJson: String,
        routeGeoJson: String,
        pinnedPlate: PinnedMapPlate? = nil,
        pinnedPlateVisible: Bool = true,
        pinnedPlateOpacity: Double = 0.84
    ) -> String {
        guard let styleUrl = Bundle.main.url(forResource: "airspace_vfr_sectional", withExtension: "json"),
              let text = try? String(contentsOf: styleUrl, encoding: .utf8),
              let styleData = text.data(using: .utf8),
              var root = (try? JSONSerialization.jsonObject(with: styleData)) as? [String: Any],
              var sources = root["sources"] as? [String: Any],
              let baseLayers = root["layers"] as? [[String: Any]] else {
            return fallbackEmptyStyle
        }

        configureSources(
            &sources,
            config: config,
            resolvedPacks: resolvedPacks,
            runwaysGeoJson: runwaysGeoJson,
            routeGeoJson: routeGeoJson,
            pinnedPlate: pinnedPlate
        )
        root["sources"] = sources

        if config.terrain3D {
            root["terrain"] = [
                "source": "terrainSource",
                "exaggeration": 1,
            ]
        } else {
            root.removeValue(forKey: "terrain")
        }

        let initialPlateAlpha = (pinnedPlate != nil && pinnedPlateVisible)
            ? min(max(pinnedPlateOpacity, 0.25), 1.0)
            : 0.0
        root["layers"] = buildEarthLayers(
            from: baseLayers,
            config: config,
            resolvedPacks: resolvedPacks,
            initialPlateOpacity: initialPlateAlpha
        )

        guard let outData = try? JSONSerialization.data(withJSONObject: root),
              let outStr = String(data: outData, encoding: .utf8) else {
            return text
        }
        return outStr
    }

    // MARK: - Sources (`tileSize: 128` on 256px FAA offline charts + explicit `bounds` + zero online chart fallback)

    private static func configureSources(
        _ sources: inout [String: Any],
        config: PadLayerConfiguration,
        resolvedPacks: ResolvedAeroPackSet,
        runwaysGeoJson: String,
        routeGeoJson: String,
        pinnedPlate: PinnedMapPlate?
    ) {
        // Strip template placeholders and online FAA chart URLs so only verified offline packs are used.
        sources.removeValue(forKey: "airspace")
        sources.removeValue(forKey: "vfr_sectional")
        sources.removeValue(forKey: "vfr_sectional_online")
        sources.removeValue(forKey: "vfr_terminal")
        sources.removeValue(forKey: "ifr_low")
        sources.removeValue(forKey: "ifr_high")

        let chartRendered = hasValidAeroChart(config: config, resolvedPacks: resolvedPacks)
        let opaquePaperChart = chartRendered && config.chartOpacity >= 0.92

        // Cap remote S3 DEM & Esri orthoimagery maxzoom when an opaque aeronautical chart is active so
        // 5G cellular HTTP fetches never starve local NVMe `PMTiles` (`DESCENDANT_LEVEL_DEPTH = 3`: z10 DEM drapes z12 charts!).
        if var terrainSrc = sources["terrainSource"] as? [String: Any] {
            terrainSrc["maxzoom"] = opaquePaperChart ? 10 : 13
            sources["terrainSource"] = terrainSrc
        }
        if opaquePaperChart {
            sources.removeValue(forKey: "hillshadeSource")
        } else if var hillSrc = sources["hillshadeSource"] as? [String: Any] {
            hillSrc["maxzoom"] = 11
            sources["hillshadeSource"] = hillSrc
        }
        if config.baseSurface == .terrain {
            sources.removeValue(forKey: "satellite")
        } else if opaquePaperChart, var satSrc = sources["satellite"] as? [String: Any] {
            satSrc["maxzoom"] = 8
            sources["satellite"] = satSrc
        }

        if var omtSrc = sources["openmaptiles"] as? [String: Any] {
            omtSrc["minzoom"] = 0
            omtSrc["maxzoom"] = 7
            sources["openmaptiles"] = omtSrc
        }

        if config.airspace3D, let airPack = resolvedPacks.airspace {
            sources["airspace"] = [
                "type": "vector",
                "url": "pmtiles://file://\(airPack.filePath)",
                "minzoom": airPack.minZoom,
                "maxzoom": airPack.maxZoom,
                "bounds": airPack.bounds,
            ]
        }

        switch config.aeroChart {
        case .none:
            break
        case .usVfr:
            if let secPack = resolvedPacks.vfrSectional {
                sources["vfr_sectional"] = [
                    "type": "raster",
                    "url": "pmtiles://file://\(secPack.filePath)",
                    "tileSize": 128,
                    "minzoom": secPack.minZoom,
                    "maxzoom": secPack.maxZoom,
                    "bounds": secPack.bounds,
                ]
            }
            if let tacPack = resolvedPacks.vfrTerminal {
                sources["vfr_terminal"] = [
                    "type": "raster",
                    "url": "pmtiles://file://\(tacPack.filePath)",
                    "tileSize": 128,
                    "minzoom": tacPack.minZoom,
                    "maxzoom": tacPack.maxZoom,
                    "bounds": tacPack.bounds,
                ]
            }
        case .ifrLow:
            if let ifrLowPack = resolvedPacks.ifrLow {
                sources["ifr_low"] = [
                    "type": "raster",
                    "url": "pmtiles://file://\(ifrLowPack.filePath)",
                    "tileSize": 128,
                    "minzoom": ifrLowPack.minZoom,
                    "maxzoom": ifrLowPack.maxZoom,
                    "bounds": ifrLowPack.bounds,
                ]
            }
        case .ifrHigh:
            if let ifrHighPack = resolvedPacks.ifrHigh {
                sources["ifr_high"] = [
                    "type": "raster",
                    "url": "pmtiles://file://\(ifrHighPack.filePath)",
                    "tileSize": 128,
                    "minzoom": ifrHighPack.minZoom,
                    "maxzoom": ifrHighPack.maxZoom,
                    "bounds": ifrHighPack.bounds,
                ]
            }
        }

        if config.baseSurface == .terrain {
            sources["topo_relief"] = [
                "type": "raster",
                "tiles": [
                    "https://services.arcgisonline.com/ArcGIS/rest/services/World_Physical_Map/MapServer/tile/{z}/{y}/{x}",
                ],
                "tileSize": 256,
                "minzoom": 0,
                "maxzoom": 8,
                "attribution": "US National Park Service / Esri Shaded Relief",
            ]
        }

        let effectiveRunwayJson = (config.runwaysAndRoute && config.airportRunways)
            ? runwaysGeoJson
            : emptyFeatureCollection
        let rwyObj = (effectiveRunwayJson.data(using: .utf8).flatMap { try? JSONSerialization.jsonObject(with: $0) })
            ?? ["type": "FeatureCollection", "features": []]
        sources["airport_runways"] = [
            "type": "geojson",
            "maxzoom": 16,
            "data": rwyObj,
        ]

        let effectiveRouteJson = (config.runwaysAndRoute && config.fplRouteOverlay)
            ? routeGeoJson
            : emptyFeatureCollection
        let routeObj = (effectiveRouteJson.data(using: .utf8).flatMap { try? JSONSerialization.jsonObject(with: $0) })
            ?? ["type": "FeatureCollection", "features": []]
        sources["flight_plan_route"] = [
            "type": "geojson",
            "maxzoom": 16,
            "data": routeObj,
        ]

        let plateUrl = (pinnedPlate?.pngFileUrl ?? GeoPdfPlateRenderer.placeholderTransparentPngUrl()).absoluteString
        let flat = (pinnedPlate?.cornersLonLat.count == 8) ? (pinnedPlate?.cornersLonLat ?? defaultPlateCornersFlat) : defaultPlateCornersFlat
        sources["procedure_plate"] = [
            "type": "image",
            "url": plateUrl,
            "coordinates": [
                [flat[0], flat[1]],
                [flat[2], flat[3]],
                [flat[4], flat[5]],
                [flat[6], flat[7]],
            ],
        ]
    }

    // MARK: - Layer Stack Assembly (Base Surface -> Hillshade -> Offline Aero Chart -> Georef Plate -> Airspace -> Route & Runways)

    private static func buildEarthLayers(
        from rawLayers: [[String: Any]],
        config: PadLayerConfiguration,
        resolvedPacks: ResolvedAeroPackSet,
        initialPlateOpacity: Double
    ) -> [[String: Any]] {
        var out: [[String: Any]] = [
            [
                "id": "background",
                "type": "background",
                "paint": ["background-color": "#060e1c"],
            ],
            [
                "id": "antarctic-ice-fill",
                "type": "fill",
                "source": "antarctic-ice",
                "paint": ["fill-color": "#dce6f0"],
            ],
        ]

        // 1. Physical Base Surface (`Terrain` hypsometric relief vs `Satellite` world orthoimagery).
        switch config.baseSurface {
        case .terrain:
            out.append([
                "id": "terrain-physical-surface",
                "type": "raster",
                "source": "topo_relief",
                "paint": [
                    "raster-fade-duration": 0,
                    "raster-resampling": "linear",
                ],
            ])
        case .satellite:
            out.append([
                "id": "satellite-surface",
                "type": "raster",
                "source": "satellite",
                "paint": [
                    "raster-fade-duration": 0,
                    "raster-resampling": "linear",
                ],
            ])
        }

        for layer in rawLayers where (layer["id"] as? String) == "country-boundaries" {
            out.append(layer)
        }

        let chartRendered = hasValidAeroChart(config: config, resolvedPacks: resolvedPacks)
        let opaquePaperChart = chartRendered && config.chartOpacity >= 0.92

        // 2. Hillshade: enabled when no valid chart is covering or when chart opacity < 92%.
        if !opaquePaperChart {
            out.append([
                "id": "hills",
                "type": "hillshade",
                "source": "hillshadeSource",
                "minzoom": 5,
                "paint": [
                    "hillshade-shadow-color": "#1a2430",
                    "hillshade-highlight-color": "#fffdf5",
                    "hillshade-exaggeration": config.baseSurface == .terrain ? 0.46 : 0.20,
                ],
            ])
        }

        // 3. Offline Aeronautical Chart Overlay (`US VFR`, `IFR Low`, `IFR High`) — ONLY when cycle is valid!
        out.append(contentsOf: chartDrapeLayers(
            for: config.aeroChart,
            opacity: config.chartOpacity,
            resolvedPacks: resolvedPacks
        ))

        // 3b. Georeferenced Procedure Plate Overlay (`ImageSource`, updated live via FFI without recreating the map).
        out.append([
            "id": "procedure-plate-layer",
            "type": "raster",
            "source": "procedure_plate",
            "paint": [
                "raster-opacity": initialPlateOpacity,
                "raster-fade-duration": 0,
                "raster-resampling": "linear",
            ],
        ])

        // 4. Mode-aware & policy-aware 3D NASR Airspace volumes & rims — ONLY when NASR cycle is valid!
        if config.airspace3D, resolvedPacks.airspace != nil {
            out.append(contentsOf: buildAirspaceLayers(config: config, chartRendered: chartRendered))
        }

        // 5. Typed Flight Plan (`FPL`) route legs/waypoints + surveyed airport runways (updated live via FFI).
        if config.runwaysAndRoute {
            if config.fplRouteOverlay {
                out.append(contentsOf: buildRouteStyleLayers(sourceName: "flight_plan_route"))
            }
            if config.airportRunways {
                out.append(contentsOf: buildAirportRunwayStyleLayers(onPaperChart: chartRendered))
            }
        }

        // 6. Global ocean & country overview labels (`z0..6.5`).
        for layer in rawLayers {
            let id = (layer["id"] as? String) ?? ""
            if id == "ocean-labels" || id == "country-labels" {
                out.append(layer)
            }
        }

        return out
    }

    private static func chartDrapeLayers(
        for chart: EarthAeroChart,
        opacity: Double,
        resolvedPacks: ResolvedAeroPackSet
    ) -> [[String: Any]] {
        let clampedAlpha = min(max(opacity, 0.25), 1.0)
        let groundHoldOpacity: [Any] = [
            "interpolate", ["linear"], ["zoom"],
            12.8, clampedAlpha,
            15.8, clampedAlpha * 0.90,
        ]
        switch chart {
        case .none:
            return []
        case .usVfr:
            var layers: [[String: Any]] = []
            if resolvedPacks.vfrSectional != nil {
                layers.append([
                    "id": "vfr-sectional-drape",
                    "type": "raster",
                    "source": "vfr_sectional",
                    "minzoom": 0,
                    "paint": [
                        "raster-opacity": groundHoldOpacity,
                        "raster-fade-duration": 0,
                        "raster-resampling": "linear",
                    ],
                ])
            }
            if resolvedPacks.vfrTerminal != nil {
                layers.append([
                    "id": "vfr-terminal-drape",
                    "type": "raster",
                    "source": "vfr_terminal",
                    "minzoom": 9.6,
                    "paint": [
                        "raster-opacity": groundHoldOpacity,
                        "raster-fade-duration": 0,
                        "raster-resampling": "linear",
                    ],
                ])
            }
            return layers
        case .ifrLow:
            guard resolvedPacks.ifrLow != nil else { return [] }
            return [
                [
                    "id": "ifr-low-drape",
                    "type": "raster",
                    "source": "ifr_low",
                    "minzoom": 0,
                    "paint": [
                        "raster-opacity": groundHoldOpacity,
                        "raster-fade-duration": 0,
                        "raster-resampling": "linear",
                    ],
                ],
            ]
        case .ifrHigh:
            guard resolvedPacks.ifrHigh != nil else { return [] }
            return [
                [
                    "id": "ifr-high-drape",
                    "type": "raster",
                    "source": "ifr_high",
                    "minzoom": 0,
                    "paint": [
                        "raster-opacity": groundHoldOpacity,
                        "raster-fade-duration": 0,
                        "raster-resampling": "linear",
                    ],
                ],
            ]
        }
    }

    // MARK: - Granular & Mode-Aware 3D Airspace Portrayal

    private static func buildAirspaceLayers(
        config: PadLayerConfiguration,
        chartRendered: Bool
    ) -> [[String: Any]] {
        let isIfr = config.resolvedAirspaceIsIfr
        let isIfrHigh = (config.aeroChart == .ifrHigh && config.airspacePolicy == .autoChart)

        var filterClauses: [Any] = [
            "all",
            ["!", ["get", "is_enroute_blanket"]],
        ]

        if !config.classG {
            filterClauses.append(["!=", ["get", "class"], "G"])
            filterClauses.append(["!=", ["get", "local_type"], "CLASS_G"])
        }
        if !config.prohibitedRestrictedWarning {
            filterClauses.append(["!", ["match", ["get", "class"], ["P", "R", "W"], true, false]])
        }
        if !config.moaAndAlert {
            filterClauses.append(["!", ["match", ["get", "class"], ["MOA", "A"], true, false]])
        }

        if isIfrHigh {
            filterClauses.append(["match", ["get", "class"], ["P", "R", "W"], true, false])
        } else if isIfr {
            var ifrClasses = ["B", "C", "NSA"]
            if config.prohibitedRestrictedWarning { ifrClasses.append(contentsOf: ["P", "R", "W"]) }
            if config.moaAndAlert { ifrClasses.append(contentsOf: ["MOA", "A"]) }
            if config.classG { ifrClasses.append("G") }
            filterClauses.append(["match", ["get", "class"], ifrClasses, true, false])
        } else if !config.classE5Transition {
            filterClauses.append([
                "!",
                [
                    "match",
                    ["get", "local_type"],
                    ["CLASS_E5", "CLASS_E6", "CLASS_E_OTHER"],
                    true,
                    false,
                ],
            ])
        }

        let isPaperChart = chartRendered
        let bcFillOpacity: Double = isIfr ? 0.04 : (isPaperChart ? 0.13 : 0.23)
        let dFillOpacity: Double = isPaperChart ? 0.10 : 0.19
        let eFillOpacity: Double = isPaperChart ? 0.10 : 0.17
        let moaFillOpacity: Double = isIfr ? 0.025 : (isPaperChart ? 0.03 : 0.10)
        let bcRimOpacity: Double = isIfr ? 0.24 : 0.90
        let bcRimWidth: Double = isIfr ? 1.05 : 2.15

        let volumeColor: [Any] = [
            "match", ["get", "class"],
            "B", "#0072ce",
            "C", "#9e2a7b",
            "D", "#0060b8",
            "E", "#9e2a7b",
            "G", "#475569",
            "P", "#c81d3c",
            "R", "#29489e",
            "W", "#3457b2",
            "A", "#8f3270",
            "MOA", "#8f3270",
            "NSA", "#b02376",
            "#0072ce",
        ]
        let volumeOpacity: [Any] = [
            "match", ["get", "class"],
            "B", bcFillOpacity,
            "C", bcFillOpacity,
            "D", dFillOpacity,
            "E", ["match", ["get", "local_type"], "CLASS_E5", 0.06, eFillOpacity],
            "G", 0.05,
            "P", 0.32,
            "R", 0.22,
            "W", 0.16,
            "A", moaFillOpacity,
            "MOA", moaFillOpacity,
            "NSA", 0.22,
            0.14,
        ]

        let rimColor: [Any] = [
            "match", ["get", "class"],
            "B", "#004c99",
            "C", "#881d69",
            "P", "#a80f2b",
            ["R", "W"], "#223b8a",
            ["A", "MOA", "NSA"], "#881d69",
            "#004c99",
        ]
        let rimWidth: [Any] = [
            "match", ["get", "class"],
            ["B", "C"], bcRimWidth,
            "P", 2.6,
            ["R", "W"], 2.0,
            1.45,
        ]
        let rimOpacity: [Any] = [
            "match", ["get", "class"],
            ["B", "C"], bcRimOpacity,
            "P", 0.98,
            ["R", "W", "NSA"], 0.94,
            isIfr ? 0.48 : 0.88,
        ]

        var layers: [[String: Any]] = [
            [
                "id": "airspace-rim-surface-solid",
                "type": "line",
                "source": "airspace",
                "source-layer": "airspace_boundary",
                "minzoom": 6,
                "filter": [
                    "all",
                    filterClauses,
                    ["==", ["get", "boundary_role"], "surface_floor"],
                ],
                "layout": ["line-join": "round", "line-cap": "round"],
                "paint": [
                    "line-elevation": 3.0,
                    "line-elevation-datum": "agl",
                    "line-color": rimColor,
                    "line-width": rimWidth,
                    "line-opacity": rimOpacity,
                ],
            ],
            [
                "id": "airspace-rim-floor-solid",
                "type": "line",
                "source": "airspace",
                "source-layer": "airspace_boundary",
                "minzoom": 6,
                "filter": [
                    "all",
                    filterClauses,
                    ["==", ["get", "boundary_role"], "floor"],
                ],
                "layout": ["line-join": "round", "line-cap": "round"],
                "paint": [
                    "line-elevation": ["get", "rim_m"],
                    "line-elevation-datum": ["get", "rim_datum"],
                    "line-color": rimColor,
                    "line-width": rimWidth,
                    "line-opacity": rimOpacity,
                ],
            ],
            [
                "id": "airspace-volumes-vfr",
                "type": "fill-extrusion",
                "source": "airspace",
                "source-layer": "airspace",
                "minzoom": 6,
                "filter": filterClauses,
                "paint": [
                    "fill-extrusion-translucent": true,
                    "fill-extrusion-vertical-gradient": true,
                    "fill-extrusion-part": ["get", "part"],
                    "fill-extrusion-base": ["get", "lower_m"],
                    "fill-extrusion-height": ["get", "display_upper_m"],
                    "fill-extrusion-base-datum": ["get", "lower_datum"],
                    "fill-extrusion-height-datum": ["get", "display_upper_datum"],
                    "fill-extrusion-color": volumeColor,
                    "fill-extrusion-opacity": volumeOpacity,
                ],
            ],
            [
                "id": "airspace-rim-ceiling-solid",
                "type": "line",
                "source": "airspace",
                "source-layer": "airspace_boundary",
                "minzoom": 6,
                "filter": [
                    "all",
                    filterClauses,
                    ["==", ["get", "boundary_role"], "ceiling"],
                    ["match", ["get", "class"], ["B", "C", "P", "R", "W", "A", "MOA"], true, false],
                ],
                "layout": ["line-join": "round", "line-cap": "round"],
                "paint": [
                    "line-elevation": ["get", "rim_m"],
                    "line-elevation-datum": ["get", "rim_datum"],
                    "line-color": rimColor,
                    "line-width": rimWidth,
                    "line-opacity": rimOpacity,
                ],
            ],
        ]

        if !isIfr {
            layers.append([
                "id": "airspace-rim-ceiling-dashed",
                "type": "line",
                "source": "airspace",
                "source-layer": "airspace_boundary",
                "minzoom": 6.5,
                "filter": [
                    "all",
                    filterClauses,
                    ["==", ["get", "boundary_role"], "ceiling"],
                    [
                        "any",
                        ["match", ["get", "class"], ["D", "NSA"], true, false],
                        ["match", ["get", "local_type"], ["CLASS_E2", "CLASS_E3", "CLASS_E4"], true, false],
                    ],
                ],
                "layout": ["line-join": "round", "line-cap": "butt"],
                "paint": [
                    "line-elevation": ["get", "rim_m"],
                    "line-elevation-datum": ["get", "rim_datum"],
                    "line-color": ["match", ["get", "class"], "D", "#004c99", "#881d69"],
                    "line-dasharray": [4, 3],
                    "line-width": 1.9,
                    "line-opacity": 0.90,
                ],
            ])
        } else if !isIfrHigh {
            layers.append([
                "id": "airspace-rim-nsa-ifr",
                "type": "line",
                "source": "airspace",
                "source-layer": "airspace_boundary",
                "minzoom": 6,
                "filter": [
                    "all",
                    filterClauses,
                    ["==", ["get", "boundary_role"], "ceiling"],
                    ["==", ["get", "class"], "NSA"],
                ],
                "layout": ["line-join": "round", "line-cap": "butt"],
                "paint": [
                    "line-elevation": ["get", "rim_m"],
                    "line-elevation-datum": ["get", "rim_datum"],
                    "line-color": "#9e1c68",
                    "line-dasharray": [4, 3],
                    "line-width": 2.1,
                    "line-opacity": 0.94,
                ],
            ])
        }

        return layers
    }

    // MARK: - Typed EFB / GCS Flight Plan Route & Airport Runway Style Layers

    private static func buildRouteStyleLayers(sourceName: String) -> [[String: Any]] {
        let darkCasing = AvionicsColorSystem.background.hex
        let activePlan = AvionicsColorSystem.plan.hex
        let commandedCyan = AvionicsColorSystem.commanded.hex
        let pastMuted = AvionicsColorSystem.disabled.hex
        let valueWhite = AvionicsColorSystem.value.hex
        let labelGrey = AvionicsColorSystem.label.hex

        return [
            [
                "id": "flight-plan-route-casing",
                "type": "line",
                "source": sourceName,
                "filter": ["in", ["get", "kind"], ["literal", ["past_leg", "active_leg", "planned_leg", "kml_track"]]],
                "layout": ["line-join": "round", "line-cap": "round"],
                "paint": [
                    "line-color": darkCasing,
                    "line-width": 6.6,
                    "line-opacity": 0.90,
                ],
            ],
            [
                "id": "flight-plan-route-core",
                "type": "line",
                "source": sourceName,
                "filter": ["in", ["get", "kind"], ["literal", ["past_leg", "active_leg", "planned_leg", "kml_track"]]],
                "layout": ["line-join": "round", "line-cap": "round"],
                "paint": [
                    "line-color": [
                        "match", ["get", "kind"],
                        "active_leg", activePlan,
                        "past_leg", pastMuted,
                        commandedCyan,
                    ],
                    "line-width": 3.8,
                    "line-opacity": 0.96,
                ],
            ],
            [
                "id": "flight-plan-waypoint-dot",
                "type": "circle",
                "source": sourceName,
                "filter": ["in", ["get", "kind"], ["literal", ["waypoint", "active_waypoint"]]],
                "paint": [
                    "circle-radius": [
                        "match", ["get", "seq_state"],
                        "active", 6.2,
                        "past", 4.6,
                        5.4,
                    ],
                    "circle-color": [
                        "match", ["get", "seq_state"],
                        "active", activePlan,
                        "past", pastMuted,
                        commandedCyan,
                    ],
                    "circle-stroke-width": 2.2,
                    "circle-stroke-color": darkCasing,
                    "circle-pitch-scale": "viewport",
                    "circle-pitch-alignment": "viewport",
                ],
            ],
            [
                "id": "flight-plan-waypoint-label",
                "type": "symbol",
                "source": sourceName,
                "filter": ["in", ["get", "kind"], ["literal", ["waypoint", "active_waypoint"]]],
                "layout": [
                    "text-field": ["get", "label"],
                    "text-font": ["Noto Sans Bold"],
                    "text-size": 11.5,
                    "text-offset": [0, -1.25],
                    "text-pitch-alignment": "viewport",
                ],
                "paint": [
                    "text-color": [
                        "match", ["get", "seq_state"],
                        "active", activePlan,
                        "past", labelGrey,
                        valueWhite,
                    ],
                    "text-halo-color": darkCasing,
                    "text-halo-width": 2.4,
                ],
            ],
        ]
    }

    private static func buildAirportRunwayStyleLayers(onPaperChart: Bool) -> [[String: Any]] {
        let darkCasing = AvionicsColorSystem.background.hex
        let surfaceDark = AvionicsColorSystem.surface.hex
        let controlDark = AvionicsColorSystem.control.hex
        let commandedCyan = AvionicsColorSystem.commanded.hex
        let valueWhite = AvionicsColorSystem.value.hex
        let labelGrey = AvionicsColorSystem.label.hex

        let stripOpacity: Any = onPaperChart
            ? ["case", ["==", ["get", "dest"], 1], 0.90, 0.70]
            : ["case", ["==", ["get", "dest"], 1], 0.30, 0.14]
        let stripColor: Any = onPaperChart
            ? ["case", ["==", ["get", "dest"], 1], surfaceDark, controlDark]
            : surfaceDark

        return [
            [
                "id": "airport-runway-strip",
                "type": "fill",
                "source": "airport_runways",
                "minzoom": 9.5,
                "filter": ["==", ["get", "kind"], "strip"],
                "paint": ["fill-color": stripColor, "fill-opacity": stripOpacity],
            ],
            [
                "id": "airport-runway-lead",
                "type": "line",
                "source": "airport_runways",
                "minzoom": 9.5,
                "filter": ["==", ["get", "kind"], "lead"],
                "paint": [
                    "line-color": ["case", ["==", ["get", "dest"], 1], commandedCyan, labelGrey],
                    "line-dasharray": [4, 3],
                    "line-width": ["case", ["==", ["get", "dest"], 1], 2.2, 1.2],
                    "line-opacity": 0.88,
                ],
            ],
            [
                "id": "airport-runway-outline",
                "type": "line",
                "source": "airport_runways",
                "minzoom": 9.5,
                "filter": ["==", ["get", "kind"], "outline"],
                "paint": [
                    "line-color": ["case", ["==", ["get", "dest"], 1], commandedCyan, valueWhite],
                    "line-width": ["case", ["==", ["get", "dest"], 1], 2.6, 1.4],
                    "line-opacity": 0.95,
                ],
            ],
            [
                "id": "airport-runway-centerline",
                "type": "line",
                "source": "airport_runways",
                "minzoom": 11.0,
                "filter": ["==", ["get", "kind"], "centerline"],
                "paint": [
                    "line-color": valueWhite,
                    "line-dasharray": [3, 3],
                    "line-width": 1.6,
                    "line-opacity": 0.92,
                ],
            ],
            [
                "id": "airport-runway-threshold",
                "type": "line",
                "source": "airport_runways",
                "minzoom": 10.5,
                "filter": ["==", ["get", "kind"], "threshold"],
                "paint": [
                    "line-color": ["case", ["==", ["get", "dest"], 1], commandedCyan, valueWhite],
                    "line-width": 3.4,
                    "line-opacity": 0.98,
                ],
            ],
            [
                "id": "airport-runway-label",
                "type": "symbol",
                "source": "airport_runways",
                "minzoom": 9.5,
                "filter": ["==", ["get", "kind"], "label"],
                "layout": [
                    "text-field": ["get", "name"],
                    "text-font": ["Noto Sans Bold"],
                    "text-size": ["case", ["==", ["get", "dest"], 1], 13, 11],
                    "text-offset": [0, 1.35],
                    "text-padding": 14,
                    "text-pitch-alignment": "viewport",
                ],
                "paint": [
                    "text-color": ["case", ["==", ["get", "dest"], 1], commandedCyan, valueWhite],
                    "text-halo-color": darkCasing,
                    "text-halo-width": 2.2,
                ],
            ],
        ]
    }

    // MARK: - Explicit Non-Earth Planetary Style (`Moon IAU`, `Mars IAU`)

    static func buildPlanetaryStyleJson(for body: CelestialBody, routeGeoJson: String = "") -> String {
        let isMoon = (body == .moonIauSphere)
        let regolithHex = isMoon ? "#787d86" : "#a85d3c"
        let basinHex = isMoon ? "#4a4f58" : "#7c3f28"
        let graticuleHex = isMoon ? "#c8d2e6" : "#f2c6a8"

        var features: [[String: Any]] = []
        for lat in stride(from: -60, through: 60, by: 30) {
            let coords = stride(from: -180, through: 180, by: 5).map { [Double($0), Double(lat)] }
            features.append([
                "type": "Feature",
                "properties": ["kind": lat == 0 ? "equator" : "grid"],
                "geometry": ["type": "LineString", "coordinates": coords],
            ])
        }
        for lon in stride(from: -150, through: 180, by: 30) {
            let coords = stride(from: -85, through: 85, by: 5).map { [Double(lon), Double($0)] }
            features.append([
                "type": "Feature",
                "properties": ["kind": lon == 0 ? "prime" : "grid"],
                "geometry": ["type": "LineString", "coordinates": coords],
            ])
        }

        if isMoon {
            let maria: [(String, Double, Double, Double, Double)] = [
                ("Mare Imbrium", -15.6, 32.8, 18.0, 14.0),
                ("Oceanus Procellarum", -56.0, 18.4, 24.0, 28.0),
                ("Mare Serenitatis", 17.5, 28.0, 11.0, 10.0),
                ("Mare Tranquillitatis", 31.4, 8.5, 13.0, 9.5),
                ("Mare Crisium", 59.1, 17.0, 8.5, 7.0),
                ("Mare Fecunditatis", 53.7, -7.8, 10.0, 9.5),
                ("Mare Nubium", -16.6, -21.3, 11.0, 9.0),
            ]
            for (name, lonC, latC, rx, ry) in maria {
                let ring = stride(from: 0, through: 360, by: 15).map { deg -> [Double] in
                    let rad = Double(deg) * .pi / 180.0
                    return [lonC + rx * cos(rad), latC + ry * sin(rad)]
                }
                features.append([
                    "type": "Feature",
                    "properties": ["kind": "basin", "name": name],
                    "geometry": ["type": "Polygon", "coordinates": [ring]],
                ])
            }
            let landmarks: [(String, Double, Double)] = [
                ("Mare Tranquillitatis · Apollo 11", 23.473, 0.674),
                ("Mare Imbrium", -15.6, 32.8),
                ("Oceanus Procellarum", -54.0, 18.4),
                ("Mare Serenitatis", 17.5, 28.0),
                ("Tycho Crater", -11.36, -43.31),
                ("Copernicus Crater", -20.08, 9.62),
                ("Mare Crisium", 59.1, 17.0),
            ]
            for (name, lon, lat) in landmarks {
                features.append([
                    "type": "Feature",
                    "properties": ["kind": "label", "name": name],
                    "geometry": ["type": "Point", "coordinates": [lon, lat]],
                ])
            }
        } else {
            let landmarks: [(String, Double, Double)] = [
                ("Olympus Mons", -133.8, 18.65),
                ("Valles Marineris", -59.2, -13.9),
                ("Jezero Crater", 77.45, 18.44),
                ("Hellas Planitia", 70.0, -42.4),
                ("Gale Crater", 137.8, -5.4),
            ]
            for (name, lon, lat) in landmarks {
                features.append([
                    "type": "Feature",
                    "properties": ["kind": "label", "name": name],
                    "geometry": ["type": "Point", "coordinates": [lon, lat]],
                ])
            }
        }

        let routeObj = (routeGeoJson.data(using: .utf8).flatMap { try? JSONSerialization.jsonObject(with: $0) })
            ?? ["type": "FeatureCollection", "features": []]
        let sources: [String: Any] = [
            "planetary_features": [
                "type": "geojson",
                "data": ["type": "FeatureCollection", "features": features],
            ],
            "flight_plan_route": [
                "type": "geojson",
                "maxzoom": 16,
                "data": routeObj,
            ],
        ]

        var layers: [[String: Any]] = [
            [
                "id": "regolith-base",
                "type": "background",
                "paint": ["background-color": regolithHex],
            ],
            [
                "id": "planetary-basins",
                "type": "fill",
                "source": "planetary_features",
                "filter": ["==", ["get", "kind"], "basin"],
                "paint": ["fill-color": basinHex, "fill-opacity": 0.72],
            ],
            [
                "id": "planetary-graticule",
                "type": "line",
                "source": "planetary_features",
                "filter": ["in", ["get", "kind"], ["literal", ["grid", "equator", "prime"]]],
                "paint": [
                    "line-color": graticuleHex,
                    "line-opacity": ["match", ["get", "kind"], ["equator", "prime"], 0.55, 0.26],
                    "line-width": ["match", ["get", "kind"], ["equator", "prime"], 1.4, 0.8],
                ],
            ],
            [
                "id": "planetary-labels",
                "type": "symbol",
                "source": "planetary_features",
                "filter": ["==", ["get", "kind"], "label"],
                "layout": [
                    "text-field": ["get", "name"],
                    "text-font": ["Noto Sans Bold"],
                    "text-size": 13,
                    "text-max-width": 10,
                    "text-pitch-alignment": "viewport",
                ],
                "paint": [
                    "text-color": "#ffffff",
                    "text-halo-color": "#06080f",
                    "text-halo-width": 1.8,
                ],
            ],
        ]
        layers.append(contentsOf: buildRouteStyleLayers(sourceName: "flight_plan_route"))

        let root: [String: Any] = [
            "version": 8,
            "name": body.badgeLabel,
            "projection": ["type": "vertical-perspective"],
            "glyphs": "https://tiles.openfreemap.org/fonts/{fontstack}/{range}.pbf",
            "sources": sources,
            "layers": layers,
            "center": [0.0, 0.0],
            "zoom": 1.1,
            "pitch": 0,
            "bearing": 0,
        ]

        guard let data = try? JSONSerialization.data(withJSONObject: root),
              let str = String(data: data, encoding: .utf8) else {
            return fallbackEmptyStyle
        }
        return str
    }

    private static let fallbackEmptyStyle =
        "{\"version\":8,\"projection\":{\"type\":\"vertical-perspective\"},\"sources\":{},\"layers\":[]}"
}

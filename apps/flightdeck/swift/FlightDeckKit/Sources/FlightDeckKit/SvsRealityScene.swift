import CoreGraphics
import CoreText
import Foundation
import RealityKit
import simd

#if canImport(UIKit)
import UIKit
#elseif canImport(AppKit)
import AppKit
#endif

/// Component attached to interactive 3D ADS-B traffic entities so spatial tap designates a target.
public struct SvsTrafficHitComponent: Component, Equatable, Sendable {
    public var trackId: UInt64
    public var callsign: String

    public init(trackId: UInt64, callsign: String) {
        self.trackId = trackId
        self.callsign = callsign
    }
}

/// Builds and updates the conformal Mixed-Reality `RealityKit` scene for Apple Vision Pro:
/// - **Synthetic Terrain + Runways + 3° HITS Glideslope Gates** (`opacity <= 0.85` -> `>= 15%` passthrough)
/// - **3D Controlled / SUA / TFR Airspace Curtains, Rims & Floating Labels**
/// - **3D ADS-B Traffic Diamonds, Velocity Leaders & Callsign Placards** (`JBU1990 +07↓ 3.8NM`)
/// - **Collimated 3D Unit-Sphere HWD Symbology** (`indicate-instrument-hmd`)
@MainActor
public final class SvsRealityScene {
    private static var didRegisterComponents = false

    /// Root entity added to `RealityView`.
    public let rootEntity = Entity()

    /// Conformal optical collimation scale (`1 m ENU -> 0.04 m` optical shell):
    /// preserves every true angular bearing, horizon dip, glideslope, and traffic line-of-sight
    /// from `(0, 0, 0)` while keeping `25 NM` (`46 km` -> `1,850 m`) inside the headset far clip plane.
    public var collimationScale: Float = 0.04

    /// Eye-level offset (`metres`) relative to `ImmersiveSpace` origin (`~1.25 m` seated pilot eye height).
    public var cockpitEyeHeightM: Float = 1.25

    private let worldAlignedRoot = Entity()
    private let terrainGroup = Entity()
    private let runwayGroup = Entity()
    private let airspaceGroup = Entity()
    private let trafficGroup = Entity()
    private let hwdSphereGroup = Entity()

    private let unitBox: MeshResource
    private let unitSphere: MeshResource
    private let unitPlane: MeshResource

    private var placardMaterialCache: [String: UnlitMaterial] = [:]
    private var lastGeometryRefreshGeneration: UInt32 = 0
    private var lastTerrainOpacity: Float = -1.0
    private var lastAirspaceFillOpacity: Float = -1.0

    public init() {
        Self.registerComponentsIfNeeded()
        self.unitBox = .generateBox(size: 1.0)
        self.unitSphere = .generateSphere(radius: 1.0)
        self.unitPlane = .generatePlane(width: 1.0, height: 1.0)

        rootEntity.name = "svs.root"
        worldAlignedRoot.name = "svs.worldAligned"
        terrainGroup.name = "svs.terrain"
        runwayGroup.name = "svs.runways"
        airspaceGroup.name = "svs.airspaces"
        trafficGroup.name = "svs.traffic"
        hwdSphereGroup.name = "svs.hwdSphere"

        worldAlignedRoot.addChild(terrainGroup)
        worldAlignedRoot.addChild(runwayGroup)
        worldAlignedRoot.addChild(airspaceGroup)
        worldAlignedRoot.addChild(trafficGroup)
        worldAlignedRoot.addChild(hwdSphereGroup)
        rootEntity.addChild(worldAlignedRoot)
    }

    public static func registerComponentsIfNeeded() {
        guard !didRegisterComponents else { return }
        SvsTrafficHitComponent.registerComponent()
        #if os(visionOS)
        BillboardComponent.registerComponent()
        #endif
        didRegisterComponents = true
    }

    /// Resolves a tapped `Entity` (or ancestor) to an ``SvsTrafficHitComponent``.
    public static func resolveTrafficHit(from entity: Entity?) -> SvsTrafficHitComponent? {
        var cur = entity
        while let e = cur {
            if let hit = e.components[SvsTrafficHitComponent.self] {
                return hit
            }
            cur = e.parent
        }
        return nil
    }

    /// Updates the 3D XR scene from `frame`, strictly enforcing `virtualTerrainOpacity <= 0.85`.
    public func update(with frame: FlightDeckFrame, designatedTrackId: UInt64?) {
        let headingDeg = Float(frame.ownship.trueHeadingDeg ?? frame.ownship.groundTrackTrueDeg ?? 0.0)
        let headingRad = headingDeg * (.pi / 180.0)

        // Rotate world-aligned ENU (`east -> +X, up -> +Y, north -> -Z`) by `+headingRad` around `+Y`
        // so ownship's nose always points straight along `-Z` out the forward cockpit windshield.
        rootEntity.position = SIMD3<Float>(0, cockpitEyeHeightM, 0)
        worldAlignedRoot.orientation = simd_quatf(angle: headingRad, axis: SIMD3<Float>(0, 1, 0))

        // Enforce the non-negotiable >= 15% real-world passthrough safety floor.
        let safeTerrainOpacity = min(
            max(frame.blend.virtualTerrainOpacity, 0.0),
            SvsBlendWeights.maxVirtualOpacity
        )
        terrainGroup.isEnabled = safeTerrainOpacity > 0.005

        let usesEntityOpacity: Bool
        if #available(macOS 15.0, iOS 18.0, visionOS 2.0, *) {
            terrainGroup.components.set(OpacityComponent(opacity: safeTerrainOpacity))
            usesEntityOpacity = true
        } else {
            usesEntityOpacity = false
        }

        let terrainOpacityDelta = abs(safeTerrainOpacity - lastTerrainOpacity)
        let airspaceOpacityDelta = abs(frame.blend.airspaceFillOpacity - lastAirspaceFillOpacity)
        let blendChanged = (!usesEntityOpacity && terrainOpacityDelta > 0.005)
            || (lastTerrainOpacity <= 0.005 && safeTerrainOpacity > 0.005)
            || airspaceOpacityDelta > 0.01

        // Rebuild heavy terrain/airspace meshes at ~10 Hz (every 3rd 30 Hz frame or on mode/blend jump),
        // while updating HWD 3D strokes and ADS-B traffic every single frame.
        let shouldRebuildHeavy = lastGeometryRefreshGeneration == 0
            || frame.generation &- lastGeometryRefreshGeneration >= 3
            || blendChanged
        if shouldRebuildHeavy {
            lastGeometryRefreshGeneration = frame.generation
            lastTerrainOpacity = safeTerrainOpacity
            lastAirspaceFillOpacity = frame.blend.airspaceFillOpacity
            rebuildTerrain(
                frame.terrain,
                opacity: usesEntityOpacity ? (safeTerrainOpacity > 0.005 ? 1.0 : 0.0) : safeTerrainOpacity
            )
            rebuildRunways(frame.terrain.runways, symbologyOpacity: frame.blend.symbologyOpacity)
            rebuildAirspaces(
                frame.airspaces,
                fillOpacity: frame.blend.airspaceFillOpacity,
                rimOpacity: frame.blend.airspaceRimOpacity
            )
        }

        rebuildTraffic(
            frame.traffic,
            designatedTrackId: designatedTrackId,
            opacity: frame.blend.symbologyOpacity
        )
        rebuildHwdUnitSphere(
            frame.instruments.hwd3dStrokesEnu,
            opacity: frame.blend.symbologyOpacity
        )
    }

    /// Converts local ENU metres `(east, north, up)` to collimated `RealityKit` coordinates `(east, up, -north) * scale`.
    public func enuToScene(_ enu: SIMD3<Float>) -> SIMD3<Float> {
        SIMD3<Float>(enu.x, enu.z, -enu.y) * collimationScale
    }

    private func rebuildTerrain(_ terrain: TerrainSurfaceMesh, opacity: Float) {
        terrainGroup.children.removeAll()
        guard opacity > 0.005,
              terrain.positionsEnuM.count >= 4,
              terrain.indices.count >= 3 else {
            return
        }
        let vCount = terrain.positionsEnuM.count
        let side = Int(Float(vCount).squareRoot().rounded())
        let positions = terrain.positionsEnuM.map(enuToScene)
        let normals = terrain.normalsEnu.map { SIMD3<Float>($0.x, $0.z, -$0.y) }

        var uvs: [SIMD2<Float>] = []
        uvs.reserveCapacity(vCount)
        if side * side == vCount, side > 1 {
            let denom = Float(side - 1)
            for idx in 0..<vCount {
                let r = Float(idx / side)
                let c = Float(idx % side)
                // CoreGraphics row 0 is top; RealityKit UV v=1 is top, so flip v to align exact grid vertices.
                uvs.append(SIMD2<Float>(c / denom, 1.0 - (r / denom)))
            }
        } else {
            uvs = Array(repeating: .zero, count: vCount)
        }

        var desc = MeshDescriptor(name: "svs.terrain.patch")
        desc.positions = MeshBuffers.Positions(positions)
        if normals.count == vCount {
            desc.normals = MeshBuffers.Normals(normals)
        }
        desc.textureCoordinates = MeshBuffers.TextureCoordinates(uvs)
        desc.primitives = .triangles(terrain.indices)
        guard let mesh = try? MeshResource.generate(from: [desc]) else { return }

        let material = makeTerrainMaterial(colors: terrain.colorsRgba, side: side, opacity: opacity)
        let entity = ModelEntity(mesh: mesh, materials: [material])
        entity.name = "svs.terrain.mesh"
        terrainGroup.addChild(entity)
    }

    private func rebuildRunways(_ runways: [RunwayOverlay3D], symbologyOpacity: Float) {
        runwayGroup.children.removeAll()
        let cyan = SIMD4<Float>(0.15, 0.96, 0.92, symbologyOpacity)
        let green = SIMD4<Float>(0.18, 1.0, 0.48, symbologyOpacity * 0.90)

        for rwy in runways where rwy.cornersEnuM.count == 4 {
            let pts = rwy.cornersEnuM.map(enuToScene)
            for i in 0..<4 {
                let next = (i + 1) % 4
                if let edge = makeBarSegment(from: pts[i], to: pts[next], thickness: 0.08, rgba: cyan) {
                    runwayGroup.addChild(edge)
                }
            }
            if rwy.centerlineEnuM.count == 2 {
                let c0 = enuToScene(rwy.centerlineEnuM[0])
                let c1 = enuToScene(rwy.centerlineEnuM[1])
                if let cl = makeBarSegment(from: c0, to: c1, thickness: 0.05, rgba: cyan) {
                    runwayGroup.addChild(cl)
                }
            }
            for gate in rwy.glideslopeGatesEnuM where gate.count == 4 {
                let gPts = gate.map(enuToScene)
                for (a, b) in [(0, 1), (1, 2), (2, 3), (3, 0)] {
                    if let bar = makeBarSegment(from: gPts[a], to: gPts[b], thickness: 0.06, rgba: green) {
                        runwayGroup.addChild(bar)
                    }
                }
            }
            let rwyLabelPos = pts[0] + SIMD3<Float>(0, 1.8, 0)
            runwayGroup.addChild(
                makePlacardEntity(
                    text: rwy.label,
                    position: rwyLabelPos,
                    heightMeters: max(simd_length(rwyLabelPos) * 0.022, 0.35),
                    rgb: SIMD3<Float>(0.20, 0.98, 0.92)
                )
            )
        }
    }

    private func rebuildAirspaces(
        _ volumes: [AirspaceVolumeMesh],
        fillOpacity: Float,
        rimOpacity: Float
    ) {
        airspaceGroup.children.removeAll()
        guard fillOpacity > 0.01 || rimOpacity > 0.01 else { return }

        for vol in volumes {
            let wallAlpha = vol.ownshipInside ? min(fillOpacity * 1.35, 0.50) : fillOpacity
            if !vol.wallPositionsEnuM.isEmpty, !vol.wallIndices.isEmpty {
                var desc = MeshDescriptor(name: "svs.airspace.\(vol.airspaceKey)")
                desc.positions = MeshBuffers.Positions(vol.wallPositionsEnuM.map(enuToScene))
                desc.primitives = .triangles(vol.wallIndices)
                if let mesh = try? MeshResource.generate(from: [desc]) {
                    let mat = makeSolidMaterial(
                        rgb: vol.rgb,
                        alpha: vol.active ? wallAlpha : wallAlpha * 0.45
                    )
                    let wallEntity = ModelEntity(mesh: mesh, materials: [mat])
                    wallEntity.name = "svs.airspace.wall.\(vol.airspaceKey)"
                    airspaceGroup.addChild(wallEntity)
                }
            }

            let rimRgba = SIMD4<Float>(
                vol.rgb.x,
                vol.rgb.y,
                vol.rgb.z,
                vol.active ? rimOpacity : rimOpacity * 0.5
            )
            // Subsample dense rim segments slightly to keep XR draw-call count crisp.
            let strideStep = max(vol.rimSegmentsEnuM.count / 40, 1)
            for idx in Swift.stride(from: 0, to: vol.rimSegmentsEnuM.count, by: strideStep) {
                let (aEnu, bEnu) = vol.rimSegmentsEnuM[idx]
                let a = enuToScene(aEnu)
                let b = enuToScene(bEnu)
                let dist = max(simd_length(0.5 * (a + b)), 4.0)
                if let bar = makeBarSegment(
                    from: a,
                    to: b,
                    thickness: min(max(dist * 0.0022, 0.05), 0.90),
                    rgba: rimRgba
                ) {
                    airspaceGroup.addChild(bar)
                }
            }

            let anchorPos = enuToScene(vol.labelAnchorEnuM)
            let anchorDist = max(simd_length(anchorPos), 6.0)
            let labelHeight = min(max(anchorDist * 0.024, 0.45), 14.0)
            let placard = makePlacardEntity(
                text: vol.label,
                position: anchorPos + SIMD3<Float>(0, labelHeight * 0.8, 0),
                heightMeters: labelHeight,
                rgb: vol.rgb
            )
            placard.name = "svs.airspace.label.\(vol.airspaceKey)"
            airspaceGroup.addChild(placard)
        }
    }

    private func rebuildTraffic(
        _ targets: [TrafficTarget],
        designatedTrackId: UInt64?,
        opacity: Float
    ) {
        trafficGroup.children.removeAll()
        for target in targets.prefix(24) {
            let pos = enuToScene(target.offsetEnuM)
            let dist = max(simd_length(pos), 3.0)
            // Angularly consistent marker size (~0.9 deg visual subtense) so distant 8 NM traffic
            // and close 0.8 NM traffic are both immediately legible with their callsigns!
            let markerRadius = min(max(dist * 0.014, 0.14), 8.0)
            let rgb = threatRgb(target.threat)

            let container = Entity()
            container.name = "svs.traffic.\(target.callsign)"
            container.position = pos
            container.components.set(
                SvsTrafficHitComponent(trackId: target.trackId, callsign: target.callsign)
            )
            #if os(visionOS)
            container.components.set(InputTargetComponent())
            #endif
            container.components.set(
                CollisionComponent(shapes: [.generateSphere(radius: markerRadius * 2.2)])
            )

            let symbolMesh: MeshResource = (target.threat == .resolutionAdvisory) ? unitBox : unitSphere
            let symbol = ModelEntity(
                mesh: symbolMesh,
                materials: [makeSolidMaterial(rgb: rgb, alpha: opacity)]
            )
            symbol.scale = SIMD3<Float>(repeating: markerRadius * 1.5)
            if target.threat != .resolutionAdvisory {
                symbol.orientation = simd_quatf(
                    angle: .pi * 0.25,
                    axis: simd_normalize(SIMD3<Float>(1, 0, 1))
                )
            }
            container.addChild(symbol)

            if designatedTrackId == target.trackId {
                let halo = ModelEntity(
                    mesh: unitBox,
                    materials: [makeSolidMaterial(rgb: SIMD3<Float>(0.25, 1.0, 0.55), alpha: 0.45)]
                )
                halo.scale = SIMD3<Float>(repeating: markerRadius * 2.5)
                container.addChild(halo)
            }

            // 25-second projected velocity leader line in scene coordinates
            let velScene = SIMD3<Float>(
                target.velocityEnuMps.x,
                target.velocityEnuMps.z,
                -target.velocityEnuMps.y
            ) * (collimationScale * 25.0)
            if simd_length(velScene) > markerRadius * 0.5,
               let leader = makeBarSegment(
                   from: .zero,
                   to: velScene,
                   thickness: markerRadius * 0.22,
                   rgba: SIMD4<Float>(rgb.x, rgb.y, rgb.z, opacity * 0.85)
               ) {
                container.addChild(leader)
            }

            let placardHeight = min(max(dist * 0.022, 0.24), 12.0)
            let placard = makePlacardEntity(
                text: target.placard,
                position: SIMD3<Float>(0, markerRadius * 1.9 + placardHeight * 0.55, 0),
                heightMeters: placardHeight,
                rgb: rgb
            )
            container.addChild(placard)
            trafficGroup.addChild(container)
        }
    }

    private func rebuildHwdUnitSphere(
        _ strokesEnu: [(SIMD3<Float>, SIMD3<Float>)],
        opacity: Float
    ) {
        hwdSphereGroup.children.removeAll()
        guard !strokesEnu.isEmpty else { return }
        // Render `indicate-instrument-hmd` unit-sphere strokes on a 2.4 m collimated HUD dome around pilot eye.
        let radius: Float = 2.4
        let hudGreen = SIMD4<Float>(0.18, 1.0, 0.46, opacity)
        for (aEnu, bEnu) in strokesEnu {
            let p0 = SIMD3<Float>(aEnu.x, aEnu.z, -aEnu.y) * radius
            let p1 = SIMD3<Float>(bEnu.x, bEnu.z, -bEnu.y) * radius
            if let stroke = makeBarSegment(from: p0, to: p1, thickness: 0.0045, rgba: hudGreen) {
                hwdSphereGroup.addChild(stroke)
            }
        }
    }

    private func makeBarSegment(
        from a: SIMD3<Float>,
        to b: SIMD3<Float>,
        thickness: Float,
        rgba: SIMD4<Float>
    ) -> ModelEntity? {
        let delta = b - a
        let len = simd_length(delta)
        guard len > 1e-4, len.isFinite else { return nil }
        let dir = delta / len
        let rot = simd_quatf(from: SIMD3<Float>(0, 1, 0), to: dir)
        let mat = makeSolidMaterial(rgb: SIMD3<Float>(rgba.x, rgba.y, rgba.z), alpha: rgba.w)
        let bar = ModelEntity(mesh: unitBox, materials: [mat])
        bar.transform = Transform(
            scale: SIMD3<Float>(thickness, len, thickness),
            rotation: rot,
            translation: 0.5 * (a + b)
        )
        return bar
    }

    private func makePlacardEntity(
        text: String,
        position: SIMD3<Float>,
        heightMeters: Float,
        rgb: SIMD3<Float>
    ) -> Entity {
        let holder = Entity()
        holder.position = position
        #if os(visionOS)
        holder.components.set(BillboardComponent())
        #endif

        let glyphCount = max(text.unicodeScalars.count, 4)
        let widthMeters = max(heightMeters * Float(glyphCount) * 0.58, heightMeters * 1.8)
        let key = "\(text)|\(Int(rgb.x * 255))|\(Int(rgb.y * 255))|\(Int(rgb.z * 255))"
        let mat: UnlitMaterial
        if let cached = placardMaterialCache[key] {
            mat = cached
        } else {
            let built = Self.buildTextPlacardMaterial(text: text, rgb: rgb)
            if placardMaterialCache.count > 256 {
                placardMaterialCache.removeAll(keepingCapacity: true)
            }
            placardMaterialCache[key] = built
            mat = built
        }

        let card = ModelEntity(mesh: unitPlane, materials: [mat])
        card.transform = Transform(
            scale: SIMD3<Float>(widthMeters, 1.0, heightMeters),
            rotation: simd_quatf(angle: 0.5 * .pi, axis: SIMD3<Float>(1, 0, 0)),
            translation: .zero
        )
        holder.addChild(card)
        return holder
    }

    private func makeTerrainMaterial(
        colors: [SIMD4<UInt8>],
        side: Int,
        opacity: Float
    ) -> UnlitMaterial {
        if side >= 2, colors.count == side * side,
           let cgImage = Self.makeGridCgImage(colors: colors, width: side, height: side),
           let tex = try? TextureResource.generate(from: cgImage, options: .init(semantic: .color)) {
            var mat = UnlitMaterial()
            mat.color = .init(tint: .white, texture: .init(tex))
            if opacity < 0.999 {
                mat.blending = .transparent(opacity: .init(floatLiteral: opacity))
            }
            return mat
        }
        return makeSolidMaterial(rgb: SIMD3<Float>(0.18, 0.45, 0.28), alpha: opacity)
    }

    private func makeSolidMaterial(rgb: SIMD3<Float>, alpha: Float) -> UnlitMaterial {
        let clampedAlpha = min(max(alpha, 0.02), 1.0)
        #if canImport(UIKit)
        let color = UIColor(
            red: CGFloat(rgb.x),
            green: CGFloat(rgb.y),
            blue: CGFloat(rgb.z),
            alpha: 1.0
        )
        #elseif canImport(AppKit)
        let color = NSColor(
            srgbRed: CGFloat(rgb.x),
            green: CGFloat(rgb.y),
            blue: CGFloat(rgb.z),
            alpha: 1.0
        )
        #endif
        var mat = UnlitMaterial(color: color)
        if clampedAlpha < 0.999 {
            mat.blending = .transparent(opacity: .init(floatLiteral: clampedAlpha))
        }
        return mat
    }

    private func threatRgb(_ threat: TrafficThreatLevel) -> SIMD3<Float> {
        switch threat {
        case .other: SIMD3<Float>(0.25, 0.88, 0.98)
        case .proximate: SIMD3<Float>(0.45, 0.96, 1.0)
        case .trafficAdvisory: SIMD3<Float>(1.0, 0.76, 0.12)
        case .resolutionAdvisory: SIMD3<Float>(1.0, 0.22, 0.20)
        }
    }

    private static func makeGridCgImage(
        colors: [SIMD4<UInt8>],
        width: Int,
        height: Int
    ) -> CGImage? {
        var raw = [UInt8]()
        raw.reserveCapacity(width * height * 4)
        for c in colors {
            raw.append(c.x)
            raw.append(c.y)
            raw.append(c.z)
            raw.append(255)
        }
        guard let provider = CGDataProvider(data: Data(raw) as CFData) else { return nil }
        return CGImage(
            width: width,
            height: height,
            bitsPerComponent: 8,
            bitsPerPixel: 32,
            bytesPerRow: width * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.noneSkipLast.rawValue),
            provider: provider,
            decode: nil,
            shouldInterpolate: true,
            intent: .defaultIntent
        )
    }

    private static func buildTextPlacardMaterial(text: String, rgb: SIMD3<Float>) -> UnlitMaterial {
        let font = CTFontCreateWithName("Menlo-Bold" as CFString, 40, nil)
        let fg = CGColor(
            srgbRed: CGFloat(rgb.x),
            green: CGFloat(rgb.y),
            blue: CGFloat(rgb.z),
            alpha: 1.0
        )
        let attr = NSAttributedString(
            string: text,
            attributes: [.font: font, .foregroundColor: fg]
        )
        let line = CTLineCreateWithAttributedString(attr)
        let bounds = CTLineGetBoundsWithOptions(line, .useOpticalBounds)
        let w = max(Int(ceil(bounds.width)) + 28, 64)
        let h = max(Int(ceil(bounds.height)) + 20, 44)

        guard let ctx = CGContext(
            data: nil,
            width: w,
            height: h,
            bitsPerComponent: 8,
            bytesPerRow: w * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else {
            return UnlitMaterial()
        }

        // Translucent dark cockpit pill background + colored border for daylight outdoor legibility.
        let pillRect = CGRect(x: 2, y: 2, width: CGFloat(w - 4), height: CGFloat(h - 4))
        let pillPath = CGPath(roundedRect: pillRect, cornerWidth: 10, cornerHeight: 10, transform: nil)
        ctx.setFillColor(CGColor(srgbRed: 0.03, green: 0.05, blue: 0.09, alpha: 0.80))
        ctx.addPath(pillPath)
        ctx.fillPath()

        ctx.setStrokeColor(fg)
        ctx.setLineWidth(2.5)
        ctx.addPath(pillPath)
        ctx.strokePath()

        ctx.textPosition = CGPoint(x: 14 - bounds.origin.x, y: 10 - bounds.origin.y)
        CTLineDraw(line, ctx)

        guard let cgImage = ctx.makeImage(),
              let tex = try? TextureResource.generate(from: cgImage, options: .init(semantic: .color)) else {
            return UnlitMaterial()
        }
        var mat = UnlitMaterial()
        mat.color = .init(tint: .white, texture: .init(tex))
        mat.blending = .transparent(opacity: .init(floatLiteral: 0.96))
        return mat
    }
}

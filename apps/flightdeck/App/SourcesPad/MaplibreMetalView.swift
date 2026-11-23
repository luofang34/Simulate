import Foundation
import Metal
import MetalKit
import QuartzCore
import SwiftUI
import UIKit
import FlightDeckKit
import simd

extension Notification.Name {
    /// Triggers a deterministic multi-step off-limb empty-space pan on the live Metal globe view.
    static let flightDeckSimulateOffLimbPan = Notification.Name("FlightDeckSimulateOffLimbPan")
}

/// Full-screen 60 Hz Metal globe, terrain, chart, and 3D airspace viewport backed by `libmaplibre_visionos.a`.
struct MaplibreMetalView: UIViewRepresentable {
    let controller: FlightDeckController
    @Binding var primaryMode: PadPrimaryViewMode
    @Binding var svsSubMode: SvsSubMode
    @Binding var svsLookOffset: SvsLookOffset
    @Binding var followMode: MapFollowMode
    @Binding var northLocked: Bool
    @Binding var layers: PadLayerConfiguration
    @Binding var orbit: OrbitCameraState
    @Binding var showHudOverlay: Bool
    @Binding var effectiveZoom: Double
    let onCanvasInteraction: () -> Void

    func makeCoordinator() -> Coordinator {
        Coordinator(parent: self)
    }

    func makeUIView(context: Context) -> MTKView {
        let view = MTKView(frame: CGRect(x: 0, y: 0, width: 2048, height: 1536))
        view.colorPixelFormat = .bgra8Unorm
        view.framebufferOnly = false
        view.autoResizeDrawable = false
        view.drawableSize = CGSize(width: 2048, height: 1536)
        view.preferredFramesPerSecond = 60
        view.isPaused = false
        view.enableSetNeedsDisplay = false
        view.isOpaque = true
        view.clearColor = MTLClearColor(red: 0.0, green: 0.0, blue: 0.0, alpha: 1.0)
        context.coordinator.attach(to: view)
        return view
    }

    func updateUIView(_ uiView: MTKView, context: Context) {
        context.coordinator.parent = self
    }

    @MainActor
    final class Coordinator: NSObject, MTKViewDelegate, UIGestureRecognizerDelegate {
        var parent: MaplibreMetalView
        private var mapPtr: OpaquePointer?
        private var configuredBody: CelestialBody?
        private var configuredLayers: PadLayerConfiguration?
        private var configuredPackSet: ResolvedAeroPackSet?
        private var configuredRouteRevision: UInt64 = 0
        private var configuredRunwayRevision: UInt64 = 0
        private var configuredPlateRevision: UInt64 = 0
        private var lastCheckedStoreRevision: UInt64 = 0
        private var lastPackCheckMediaTime: CFTimeInterval = -100.0
        private var cachedPackSet = ResolvedAeroPackSet(
            airspace: nil,
            waypoints: nil,
            vfrSectional: nil,
            vfrTerminal: nil,
            ifrLow: nil,
            ifrHigh: nil
        )
        private var commandQueue: MTLCommandQueue?
        private var mapPixelWidth: UInt32 = 0
        private var mapPixelHeight: UInt32 = 0
        private var startMediaTime: CFTimeInterval = CACurrentMediaTime()
        private var lastReportedZoom: Double = -1.0
        private var lastSkySyncMediaTime: CFTimeInterval = -100.0
        private var lastSkyTerminatorState: Bool?
        private var lastSkyHasAeroChart: Bool?
        private var needsInitialStarUpload: Bool = true

        // 1:1 Ground-anchor pan & pinch gesture state (`Map` mode)
        private var hasPanAnchor: Bool = false
        private var panAnchorLat: Double = 0.0
        private var panAnchorLon: Double = 0.0
        private var panAnchorElevM: Double = 0.0
        private var lastPanLocation: CGPoint = .zero

        private var pinchStartZoom: Double = 1.55
        private var pinchStartChaseDistance: Double = 220.0
        private var hasPinchAnchor: Bool = false
        private var pinchAnchorLat: Double = 0.0
        private var pinchAnchorLon: Double = 0.0
        private var pinchAnchorElevM: Double = 0.0

        private var lastTiltTranslation: CGPoint = .zero
        private var rotationStartBearing: Double = 0.0
        private var rotationUnlockedDeadzone: Bool = false

        init(parent: MaplibreMetalView) {
            self.parent = parent
            super.init()
            NotificationCenter.default.addObserver(
                self,
                selector: #selector(handleSimulatedOffLimbPan(_:)),
                name: .flightDeckSimulateOffLimbPan,
                object: nil
            )
        }

        nonisolated deinit {}

        @objc private func handleSimulatedOffLimbPan(_ note: Notification) {
            let dx = (note.userInfo?["dx"] as? Double) ?? 0.0
            let dy = (note.userInfo?["dy"] as? Double) ?? 0.0
            simulateOffLimbPan(dx: dx, dy: dy)
        }

        func attach(to view: MTKView) {
            view.delegate = self
            let (pxW, pxH) = Self.targetDrawableDimensions(for: view)
            ensureMapConfigured(width: pxW, height: pxH, for: view)

            let pan = UIPanGestureRecognizer(target: self, action: #selector(handleSinglePan(_:)))
            pan.minimumNumberOfTouches = 1
            pan.maximumNumberOfTouches = 1
            pan.delegate = self
            view.addGestureRecognizer(pan)

            let tiltPan = UIPanGestureRecognizer(target: self, action: #selector(handleTwoFingerTilt(_:)))
            tiltPan.minimumNumberOfTouches = 2
            tiltPan.maximumNumberOfTouches = 2
            tiltPan.delegate = self
            view.addGestureRecognizer(tiltPan)

            let pinch = UIPinchGestureRecognizer(target: self, action: #selector(handlePinch(_:)))
            pinch.delegate = self
            view.addGestureRecognizer(pinch)

            let rotate = UIRotationGestureRecognizer(target: self, action: #selector(handleRotation(_:)))
            rotate.delegate = self
            view.addGestureRecognizer(rotate)
        }

        func gestureRecognizer(
            _ gestureRecognizer: UIGestureRecognizer,
            shouldRecognizeSimultaneouslyWith otherGestureRecognizer: UIGestureRecognizer
        ) -> Bool {
            let aMulti = gestureRecognizer is UIPinchGestureRecognizer || gestureRecognizer is UIRotationGestureRecognizer
            let bMulti = otherGestureRecognizer is UIPinchGestureRecognizer || otherGestureRecognizer is UIRotationGestureRecognizer
            return aMulti && bMulti
        }

        private static func maxAllowedPitch(forZoom zoom: Double) -> Double {
            if zoom <= 2.4 {
                return 0.0
            }
            let ramp = min(max((zoom - 2.4) / 3.0, 0.0), 1.0)
            return ramp * 82.0
        }

        // MARK: - 1:1 Ground-Anchor Finger Pan (`Map`) & Camera Look/Orbit (`SVS`)

        @objc private func handleSinglePan(_ gr: UIPanGestureRecognizer) {
            guard let view = gr.view else { return }
            let loc = gr.location(in: view)

            if parent.primaryMode == .svs {
                switch gr.state {
                case .began:
                    parent.onCanvasInteraction()
                    lastPanLocation = loc
                case .changed:
                    // Keep FPV + HUD strictly slaved to aircraft boresight so conformal symbology stays 1:1.
                    guard parent.svsSubMode == .chase || !parent.showHudOverlay else { return }
                    let dx = Double(loc.x - lastPanLocation.x)
                    let dy = Double(loc.y - lastPanLocation.y)
                    lastPanLocation = loc
                    parent.svsLookOffset.yawDeg = min(max(parent.svsLookOffset.yawDeg + dx * 0.22, -170.0), 170.0)
                    parent.svsLookOffset.pitchDeg = min(max(parent.svsLookOffset.pitchDeg - dy * 0.18, -45.0), 35.0)
                default:
                    break
                }
                return
            }

            switch gr.state {
            case .began:
                parent.onCanvasInteraction()
                if parent.followMode == .northUp {
                    parent.northLocked = true
                }
                parent.followMode = .free
                lastPanLocation = loc

                if let mapPtr {
                    var lat = 0.0
                    var lon = 0.0
                    var elev = 0.0
                    hasPanAnchor = maplibre_visionos_screen_to_ground(
                        mapPtr,
                        Double(loc.x),
                        Double(loc.y),
                        &lat,
                        &lon,
                        &elev
                    )
                    panAnchorLat = lat
                    panAnchorLon = lon
                    panAnchorElevM = elev
                } else {
                    hasPanAnchor = false
                }

            case .changed:
                guard let mapPtr else { return }
                let clampedPitch = min(max(parent.orbit.pitchDeg, 0.0), Self.maxAllowedPitch(forZoom: parent.orbit.zoom))
                let holdBearing = parent.northLocked ? 0.0 : parent.orbit.bearingDeg
                var outLat = parent.orbit.latitude
                var outLon = parent.orbit.longitude
                var outBearing = holdBearing

                var handledByAnchor = false
                if hasPanAnchor {
                    if maplibre_visionos_move_anchor_to_screen(
                        mapPtr,
                        panAnchorLat,
                        panAnchorLon,
                        panAnchorElevM,
                        Double(loc.x),
                        Double(loc.y),
                        parent.orbit.zoom,
                        clampedPitch,
                        holdBearing,
                        parent.northLocked,
                        &outLat,
                        &outLon,
                        &outBearing
                    ) {
                        parent.orbit.latitude = outLat
                        parent.orbit.longitude = outLon
                        parent.orbit.bearingDeg = parent.northLocked ? 0.0 : Self.normalizeDegrees(outBearing)
                        handledByAnchor = true
                    } else {
                        // Finger crossed off the planetary limb into space (or anchor went past horizon):
                        // release surface anchor and continue immediately via center-anchored hyperbolic globe pan.
                        hasPanAnchor = false
                    }
                }

                if !handledByAnchor {
                    performOrbitDragStep(mapPtr: mapPtr, from: lastPanLocation, to: loc)
                    // If the finger subsequently enters onto the visible planetary disk, lock 1:1 to that surface point.
                    var lat = 0.0
                    var lon = 0.0
                    var elev = 0.0
                    if maplibre_visionos_screen_to_ground(
                        mapPtr,
                        Double(loc.x),
                        Double(loc.y),
                        &lat,
                        &lon,
                        &elev
                    ) {
                        hasPanAnchor = true
                        panAnchorLat = lat
                        panAnchorLon = lon
                        panAnchorElevM = elev
                    }
                }
                lastPanLocation = loc

            default:
                hasPanAnchor = false
            }
        }

        private func performOrbitDragStep(mapPtr: OpaquePointer, from: CGPoint, to: CGPoint) {
            let clampedPitch = min(max(parent.orbit.pitchDeg, 0.0), Self.maxAllowedPitch(forZoom: parent.orbit.zoom))
            let holdBearing = parent.northLocked ? 0.0 : parent.orbit.bearingDeg
            var outLat = parent.orbit.latitude
            var outLon = parent.orbit.longitude
            var outBearing = holdBearing
            if maplibre_visionos_drag_orbit(
                mapPtr,
                Double(from.x),
                Double(from.y),
                Double(to.x),
                Double(to.y),
                parent.orbit.zoom,
                clampedPitch,
                holdBearing,
                parent.northLocked,
                &outLat,
                &outLon,
                &outBearing
            ) {
                parent.orbit.latitude = outLat
                parent.orbit.longitude = outLon
                parent.orbit.bearingDeg = parent.northLocked ? 0.0 : Self.normalizeDegrees(outBearing)
            }
        }

        private func simulateOffLimbPan(dx: Double, dy: Double) {
            guard let mapPtr else { return }
            parent.followMode = .free
            let start = CGPoint(x: 56.0, y: 240.0)
            let steps = 20
            var prev = start
            for step in 1...steps {
                let frac = Double(step) / Double(steps)
                let cur = CGPoint(x: start.x + dx * frac, y: start.y + dy * frac)
                performOrbitDragStep(mapPtr: mapPtr, from: prev, to: cur)
                prev = cur
            }
        }

        // MARK: - Anchor-Aware Pinch Zoom + Two-Finger Tilt & Rotate

        @objc private func handlePinch(_ gr: UIPinchGestureRecognizer) {
            guard let view = gr.view else { return }
            let mid = gr.location(in: view)

            if parent.primaryMode == .svs {
                switch gr.state {
                case .began:
                    parent.onCanvasInteraction()
                    pinchStartChaseDistance = parent.svsLookOffset.chaseDistanceMeters
                case .changed:
                    guard parent.svsSubMode == .chase else { return }
                    let scale = max(Double(gr.scale), 0.1)
                    parent.svsLookOffset.chaseDistanceMeters = min(
                        max(pinchStartChaseDistance / scale, 80.0),
                        900.0
                    )
                default:
                    break
                }
                return
            }

            switch gr.state {
            case .began:
                parent.onCanvasInteraction()
                pinchStartZoom = parent.orbit.zoom
                if parent.followMode == .free, let mapPtr {
                    var lat = 0.0
                    var lon = 0.0
                    var elev = 0.0
                    hasPinchAnchor = maplibre_visionos_screen_to_ground(
                        mapPtr,
                        Double(mid.x),
                        Double(mid.y),
                        &lat,
                        &lon,
                        &elev
                    )
                    pinchAnchorLat = lat
                    pinchAnchorLon = lon
                    pinchAnchorElevM = elev
                } else {
                    hasPinchAnchor = false
                }

            case .changed:
                let scale = max(Double(gr.scale), 0.02)
                let nextZoom = min(max(pinchStartZoom + log2(scale), 0.5), 18.0)
                let maxPitch = Self.maxAllowedPitch(forZoom: nextZoom)
                let nextPitch = min(parent.orbit.pitchDeg, maxPitch)
                let holdBearing = parent.northLocked ? 0.0 : parent.orbit.bearingDeg

                if parent.followMode == .free, hasPinchAnchor, let mapPtr {
                    var outLat = parent.orbit.latitude
                    var outLon = parent.orbit.longitude
                    var outBearing = holdBearing
                    if maplibre_visionos_move_anchor_to_screen(
                        mapPtr,
                        pinchAnchorLat,
                        pinchAnchorLon,
                        pinchAnchorElevM,
                        Double(mid.x),
                        Double(mid.y),
                        nextZoom,
                        nextPitch,
                        holdBearing,
                        parent.northLocked,
                        &outLat,
                        &outLon,
                        &outBearing
                    ) {
                        parent.orbit.latitude = outLat
                        parent.orbit.longitude = outLon
                    }
                }
                parent.orbit.zoom = nextZoom
                parent.orbit.pitchDeg = nextPitch

            default:
                hasPinchAnchor = false
            }
        }

        @objc private func handleTwoFingerTilt(_ gr: UIPanGestureRecognizer) {
            guard let view = gr.view else { return }
            guard parent.primaryMode == .map else { return }
            switch gr.state {
            case .began:
                parent.onCanvasInteraction()
                lastTiltTranslation = gr.translation(in: view)
            case .changed:
                let cur = gr.translation(in: view)
                let dy = Double(cur.y - lastTiltTranslation.y)
                lastTiltTranslation = cur
                let maxPitch = Self.maxAllowedPitch(forZoom: parent.orbit.zoom)
                parent.orbit.pitchDeg = min(max(parent.orbit.pitchDeg - dy * 0.24, 0.0), maxPitch)
            default:
                break
            }
        }

        @objc private func handleRotation(_ gr: UIRotationGestureRecognizer) {
            guard parent.primaryMode == .map else { return }
            switch gr.state {
            case .began:
                parent.onCanvasInteraction()
                rotationStartBearing = parent.orbit.bearingDeg
                rotationUnlockedDeadzone = !parent.northLocked && parent.followMode == .free
            case .changed:
                if !rotationUnlockedDeadzone {
                    guard abs(gr.rotation) > 0.07 else { return }
                    rotationUnlockedDeadzone = true
                    parent.northLocked = false
                    if parent.followMode != .free {
                        parent.followMode = .free
                    }
                }
                let deltaDeg = -Double(gr.rotation) * 180.0 / .pi
                parent.orbit.bearingDeg = Self.normalizeDegrees(rotationStartBearing + deltaDeg)
            default:
                break
            }
        }

        private static func normalizeDegrees(_ raw: Double) -> Double {
            var v = raw.truncatingRemainder(dividingBy: 360.0)
            if v < 0.0 { v += 360.0 }
            return v
        }

        // MARK: - 60 Hz Metal Render Loop

        func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

        func draw(in view: MTKView) {
            let (targetW, targetH) = Self.targetDrawableDimensions(for: view)
            let desiredDrawable = CGSize(width: CGFloat(targetW), height: CGFloat(targetH))
            if abs(view.drawableSize.width - desiredDrawable.width) > 1.0
                || abs(view.drawableSize.height - desiredDrawable.height) > 1.0 {
                view.drawableSize = desiredDrawable
            }
            ensureMapConfigured(width: targetW, height: targetH, for: view)

            let nowMedia = CACurrentMediaTime()
            let nowMs = UInt64(max(nowMedia * 1000.0, 0))
            let frame = parent.controller.stepFrame(nowMs: nowMs)

            guard let mapPtr else { return }

            let hasAero = parent.controller.celestialBody.isEarthWgs84
                && EarthStyleBuilder.hasValidAeroChart(config: parent.layers, resolvedPacks: cachedPackSet)
            let skyConfigChanged = (lastSkyTerminatorState != parent.layers.solarTerminator)
                || (lastSkyHasAeroChart != hasAero)
            if nowMedia - lastSkySyncMediaTime >= 1.0 || needsInitialStarUpload || skyConfigChanged {
                syncCelestialSkyAndStarfield(mapPtr: mapPtr)
                lastSkySyncMediaTime = nowMedia
                lastSkyTerminatorState = parent.layers.solarTerminator
                lastSkyHasAeroChart = hasAero
            }

            let elapsedSec = max(nowMedia - startMediaTime, 0.0)
            let rawTexPtr: UnsafeRawPointer?
            switch parent.primaryMode {
            case .map:
                parent.controller.setCameraViewPose(active: false)
                var orb = parent.orbit
                if parent.followMode != .free,
                   let own = frame?.ownship,
                   let lat = own.latitudeDeg,
                   let lon = own.longitudeDeg {
                    orb.latitude = lat
                    orb.longitude = lon
                    orb.bearingDeg = (parent.followMode == .northUp)
                        ? 0.0
                        : Self.normalizeDegrees(own.trueHeadingDeg ?? own.groundTrackTrueDeg ?? orb.bearingDeg)
                    if abs(orb.latitude - parent.orbit.latitude) > 1e-5
                        || abs(orb.longitude - parent.orbit.longitude) > 1e-5
                        || abs(orb.bearingDeg - parent.orbit.bearingDeg) > 0.05 {
                        let synced = orb
                        Task { @MainActor [weak self] in
                            self?.parent.orbit = synced
                        }
                    }
                } else if parent.northLocked {
                    orb.bearingDeg = 0.0
                }

                let clampedPitch = min(max(orb.pitchDeg, 0.0), Self.maxAllowedPitch(forZoom: orb.zoom))
                rawTexPtr = maplibre_visionos_render_orbit(
                    mapPtr,
                    orb.latitude,
                    orb.longitude,
                    orb.zoom,
                    clampedPitch,
                    orb.bearingDeg,
                    0.0,
                    elapsedSec
                )

            case .svs:
                rawTexPtr = renderSixDofFrame(
                    mapPtr: mapPtr,
                    ownship: frame?.ownship,
                    chaseMode: parent.svsSubMode == .chase,
                    lookOffset: parent.svsLookOffset,
                    hudActive: parent.svsSubMode == .fpv && parent.showHudOverlay,
                    aspect: Double(max(view.drawableSize.width, 1)) / Double(max(view.drawableSize.height, 1)),
                    nowMs: nowMs,
                    elapsedSec: elapsedSec
                )
            }

            let z = maplibre_visionos_effective_zoom(mapPtr)
            if z.isFinite, abs(z - lastReportedZoom) > 0.02 {
                lastReportedZoom = z
                Task { @MainActor [weak self] in
                    self?.parent.effectiveZoom = z
                }
            }

            guard let rawTexPtr,
                  let srcTex = Unmanaged<AnyObject>.fromOpaque(rawTexPtr).takeUnretainedValue() as? MTLTexture,
                  let commandQueue,
                  let drawable = view.currentDrawable,
                  let cmdBuf = commandQueue.makeCommandBuffer(),
                  let blit = cmdBuf.makeBlitCommandEncoder() else {
                return
            }

            let dstTex = drawable.texture
            let copyW = min(srcTex.width, dstTex.width)
            let copyH = min(srcTex.height, dstTex.height)
            if copyW > 0, copyH > 0 {
                blit.copy(
                    from: srcTex,
                    sourceSlice: 0,
                    sourceLevel: 0,
                    sourceOrigin: MTLOrigin(x: 0, y: 0, z: 0),
                    sourceSize: MTLSize(width: copyW, height: copyH, depth: 1),
                    to: dstTex,
                    destinationSlice: 0,
                    destinationLevel: 0,
                    destinationOrigin: MTLOrigin(x: 0, y: 0, z: 0)
                )
            }
            blit.endEncoding()
            cmdBuf.present(drawable)
            cmdBuf.commit()
        }

        // MARK: - Ephemeris Sky, Daylight Chart Luminance, and 6-DoF Pose Sync

        private func syncCelestialSkyAndStarfield(mapPtr: OpaquePointer) {
            let utcUnixSec = Date().timeIntervalSince1970
            let sky = parent.controller.evaluateSky(
                utcUnixSeconds: utcUnixSec,
                fallbackLatitudeDeg: parent.orbit.latitude,
                fallbackLongitudeDeg: parent.orbit.longitude,
                fallbackAltitudeDatumM: 0.0
            )
            let sunArr: [Double] = [
                sky.sunDirectionFixed.x,
                sky.sunDirectionFixed.y,
                sky.sunDirectionFixed.z,
            ]
            let moonArr: [Double] = [
                sky.moonDirectionFixed.x,
                sky.moonDirectionFixed.y,
                sky.moonDirectionFixed.z,
            ]
            let useHostSunTerminator: Bool = {
                guard parent.controller.celestialBody.isEarthWgs84 else {
                    return sky.hasSun
                }
                if parent.layers.solarTerminator {
                    return sky.hasSun
                }
                return !parent.layers.hasActiveAeroChart && sky.hasSun
            }()
            let nightBright: Float = {
                guard parent.controller.celestialBody.isEarthWgs84 else {
                    return max(sky.nightBrightness, 0.28)
                }
                return max(sky.nightBrightness, 0.55)
            }()

            sunArr.withUnsafeBufferPointer { sunBuf in
                moonArr.withUnsafeBufferPointer { moonBuf in
                    maplibre_visionos_set_sky(
                        mapPtr,
                        useHostSunTerminator,
                        sunBuf.baseAddress,
                        sky.sunAngularRadiusDeg,
                        sky.hasMoon,
                        moonBuf.baseAddress,
                        sky.moonAngularRadiusDeg,
                        nightBright,
                        sky.atmosphere
                    )
                }
            }

            if sky.starsUpdated || needsInitialStarUpload {
                parent.controller.withPackedStarBuffer { ptr, count in
                    maplibre_visionos_set_stars(mapPtr, ptr, count)
                }
                needsInitialStarUpload = false
            }
        }

        private func renderSixDofFrame(
            mapPtr: OpaquePointer,
            ownship: OwnshipSnapshot?,
            chaseMode: Bool,
            lookOffset: SvsLookOffset,
            hudActive: Bool,
            aspect: Double,
            nowMs: UInt64,
            elapsedSec: Double
        ) -> UnsafeRawPointer? {
            let ownLat = ownship?.latitudeDeg ?? parent.orbit.latitude
            let ownLon = ownship?.longitudeDeg ?? parent.orbit.longitude
            let rawAltM = max(ownship?.altitudeMslM ?? 280.0, 12.0)
            let baseHeadingDeg = ownship?.trueHeadingDeg ?? ownship?.groundTrackTrueDeg ?? parent.orbit.bearingDeg

            let rawOwnDem = maplibre_visionos_terrain_elevation(mapPtr, ownLat, ownLon)
            let ownDemM: Double? = (rawOwnDem.isFinite && rawOwnDem > -500.0 && rawOwnDem < 9_000.0)
                ? Double(rawOwnDem)
                : nil

            let eyeLat: Double
            let eyeLon: Double
            let eyeAltM: Double
            let viewHeadingDeg: Double
            let pitchDeg: Double
            let rollDeg: Double

            if chaseMode {
                let renderR = max(parent.controller.bodyParameters.renderRadiusMeters, 1_000.0)
                let azDeg = Self.normalizeDegrees(baseHeadingDeg + lookOffset.yawDeg)
                let azRad = azDeg * .pi / 180.0
                let distM = min(max(lookOffset.chaseDistanceMeters, 80.0), 900.0)
                let dNorthM = -cos(azRad) * distM
                let dEastM = -sin(azRad) * distM
                eyeLat = ownLat + (dNorthM / renderR) * (180.0 / .pi)
                eyeLon = ownLon + (dEastM / (renderR * max(cos(ownLat * .pi / 180.0), 0.15))) * (180.0 / .pi)
                let rawChaseDem = maplibre_visionos_terrain_elevation(mapPtr, eyeLat, eyeLon)
                let chaseFloor = (rawChaseDem.isFinite && rawChaseDem > -500.0)
                    ? Double(rawChaseDem) + 12.0
                    : 15.0
                eyeAltM = max(rawAltM + 65.0 - lookOffset.pitchDeg * 2.2, chaseFloor)
                viewHeadingDeg = azDeg
                pitchDeg = min(max(-10.0 + lookOffset.pitchDeg, -55.0), 25.0)
                rollDeg = (ownship?.rollDeg ?? 0.0) * 0.30
            } else {
                eyeLat = ownLat
                eyeLon = ownLon
                let fpvFloor = (ownDemM ?? 0.0) + 2.2
                eyeAltM = max(rawAltM, fpvFloor)
                viewHeadingDeg = hudActive
                    ? baseHeadingDeg
                    : Self.normalizeDegrees(baseHeadingDeg + lookOffset.yawDeg)
                pitchDeg = hudActive
                    ? (ownship?.pitchDeg ?? 0.0)
                    : min(max((ownship?.pitchDeg ?? 0.0) + lookOffset.pitchDeg, -65.0), 65.0)
                rollDeg = ownship?.rollDeg ?? 0.0
            }

            parent.controller.setCameraViewPose(
                active: true,
                latitudeDeg: eyeLat,
                longitudeDeg: eyeLon,
                altitudeDatumM: eyeAltM,
                trueHeadingDeg: viewHeadingDeg,
                pitchDeg: pitchDeg,
                rollDeg: rollDeg,
                demDatumM: ownDemM
            )
            _ = parent.controller.stepFrame(nowMs: nowMs)

            let identityMat: [Float] = [
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1,
            ]
            let hRad = Float(viewHeadingDeg * .pi / 180.0)
            let base = simd_float3x3(
                SIMD3<Float>(1, 0, 0),
                SIMD3<Float>(0, 0, 1),
                SIMD3<Float>(0, -1, 0)
            )
            let yawRot = simd_float3x3(simd_quatf(angle: -hRad, axis: SIMD3<Float>(0, 0, 1)))
            let pitchRot = simd_float3x3(simd_quatf(angle: Float(pitchDeg * .pi / 180.0), axis: SIMD3<Float>(1, 0, 0)))
            let rollRot = simd_float3x3(simd_quatf(angle: Float(-rollDeg * .pi / 180.0), axis: SIMD3<Float>(0, 0, 1)))
            let rot = yawRot * base * pitchRot * rollRot

            let eyeMat: [Float] = [
                rot.columns.0.x, rot.columns.0.y, rot.columns.0.z, 0,
                rot.columns.1.x, rot.columns.1.y, rot.columns.1.z, 0,
                rot.columns.2.x, rot.columns.2.y, rot.columns.2.z, 0,
                0, 0, 0, 1,
            ]
            let vFovRad = (chaseMode ? 60.0 : 62.0) * Double.pi / 180.0
            let vTan = Float(tan(vFovRad * 0.5))
            let hTan = vTan * Float(max(aspect, 0.5))
            let tangents: [Float] = [hTan, hTan, vTan, vTan]

            return identityMat.withUnsafeBufferPointer { worldFromSceneBuf in
                eyeMat.withUnsafeBufferPointer { worldFromEyeBuf in
                    tangents.withUnsafeBufferPointer { tanBuf in
                        var placement = MaplibreVisionOSPlacement(
                            anchor_latitude: eyeLat,
                            anchor_longitude: eyeLon,
                            anchor_altitude_meters: eyeAltM,
                            world_from_scene: worldFromSceneBuf.baseAddress
                        )
                        var eye = MaplibreVisionOSEye(
                            world_from_eye: worldFromEyeBuf.baseAddress,
                            tangents: tanBuf.baseAddress,
                            near: 1.5,
                            far: 180_000.0,
                            color_texture: nil,
                            depth_texture: nil
                        )
                        return maplibre_visionos_render_frame(
                            mapPtr,
                            &placement,
                            nil,
                            &eye,
                            1,
                            0.0,
                            elapsedSec
                        )
                    }
                }
            }
        }

        // MARK: - Native 1:1 Retina Drawable & Map Lifecycle

        private static func targetDrawableDimensions(for view: MTKView) -> (UInt32, UInt32) {
            let bounds = view.bounds.size
            guard bounds.width > 64, bounds.height > 64 else {
                return (2420, 1668)
            }
            let rawScale = view.window?.screen.scale ?? view.traitCollection.displayScale
            let scale = min(max(rawScale > 0 ? rawScale : 2.0, 1.0), 2.0)
            let rawW = bounds.width * scale
            let rawH = bounds.height * scale
            let maxEdge: CGFloat = 2752.0
            let factor = min(maxEdge / max(rawW, rawH), 1.0)
            let w = UInt32(max((rawW * factor).rounded(), 320))
            let h = UInt32(max((rawH * factor).rounded(), 240))
            return (w, h)
        }

        private func currentResolvedPacks() -> ResolvedAeroPackSet {
            let nowMedia = CACurrentMediaTime()
            let storeRev = parent.controller.dataPackStore.storeRevision
            if storeRev != lastCheckedStoreRevision || (nowMedia - lastPackCheckMediaTime) >= 1.0 {
                lastCheckedStoreRevision = storeRev
                lastPackCheckMediaTime = nowMedia
                cachedPackSet = parent.controller.dataPackStore.resolvedPackSet()
            }
            return cachedPackSet
        }

        private func updateGeoJsonSourceInPlace(
            mapPtr: OpaquePointer,
            sourceName: String,
            geoJson: String
        ) {
            sourceName.withCString { nameCStr in
                geoJson.withCString { jsonCStr in
                    _ = maplibre_visionos_set_geojson_source(mapPtr, nameCStr, jsonCStr)
                }
            }
        }

        private func syncPlateOverlayInPlace(mapPtr: OpaquePointer) {
            guard parent.controller.celestialBody.isEarthWgs84 else { return }
            let pinned = parent.controller.pinnedMapPlate
            let urlStr = (pinned?.pngFileUrl ?? GeoPdfPlateRenderer.placeholderTransparentPngUrl()).absoluteString
            let flat = (pinned?.cornersLonLat.count == 8)
                ? (pinned?.cornersLonLat ?? EarthStyleBuilder.defaultPlateCornersFlat)
                : EarthStyleBuilder.defaultPlateCornersFlat
            let alpha = (pinned != nil && parent.controller.pinnedPlateVisible)
                ? min(max(parent.controller.pinnedPlateOpacity, 0.25), 1.0)
                : 0.0

            "procedure_plate".withCString { srcCStr in
                urlStr.withCString { urlCStr in
                    flat.withUnsafeBufferPointer { cornerBuf in
                        _ = maplibre_visionos_set_image_source(mapPtr, srcCStr, urlCStr, cornerBuf.baseAddress)
                    }
                }
            }
            "procedure-plate-layer".withCString { layerCStr in
                _ = maplibre_visionos_set_layer_opacity(mapPtr, layerCStr, alpha)
            }
        }

        private func ensureMapConfigured(width: UInt32, height: UInt32, for view: MTKView) {
            let desiredBody = parent.controller.celestialBody
            let desiredLayers = parent.layers
            let desiredPacks: ResolvedAeroPackSet? = desiredBody.isEarthWgs84 ? currentResolvedPacks() : nil
            let desiredRouteRev = parent.controller.routeOverlayRevision
            let desiredRunwayRev = parent.controller.runwayOverlayRevision
            let desiredPlateRev = parent.controller.plateOverlayRevision
            let sizeChanged = abs(Int(width) - Int(mapPixelWidth)) > 16 || abs(Int(height) - Int(mapPixelHeight)) > 16

            if let existing = mapPtr,
               configuredBody == desiredBody,
               configuredLayers == desiredLayers,
               configuredPackSet == desiredPacks,
               !sizeChanged {
                // Live in-place GeoJSON & ImageSource updates (<0.2 ms) — NEVER destroy or reload map tiles!
                if configuredRouteRevision != desiredRouteRev {
                    let routeGeoJson = (desiredLayers.runwaysAndRoute && desiredLayers.fplRouteOverlay)
                        ? parent.controller.flightPlanGeoJson()
                        : EarthStyleBuilder.emptyFeatureCollection
                    updateGeoJsonSourceInPlace(mapPtr: existing, sourceName: "flight_plan_route", geoJson: routeGeoJson)
                    configuredRouteRevision = desiredRouteRev
                }
                if configuredRunwayRevision != desiredRunwayRev {
                    if desiredBody.isEarthWgs84 {
                        let centerLat = parent.controller.latestFrame?.ownship.latitudeDeg ?? parent.orbit.latitude
                        let centerLon = parent.controller.latestFrame?.ownship.longitudeDeg ?? parent.orbit.longitude
                        let runwaysGeoJson = (desiredLayers.runwaysAndRoute && desiredLayers.airportRunways)
                            ? parent.controller.runwaysGeoJson(
                                centerLatitudeDeg: centerLat,
                                centerLongitudeDeg: centerLon,
                                radiusKm: 130.0
                            )
                            : EarthStyleBuilder.emptyFeatureCollection
                        updateGeoJsonSourceInPlace(mapPtr: existing, sourceName: "airport_runways", geoJson: runwaysGeoJson)
                    }
                    configuredRunwayRevision = desiredRunwayRev
                }
                if configuredPlateRevision != desiredPlateRev {
                    syncPlateOverlayInPlace(mapPtr: existing)
                    configuredPlateRevision = desiredPlateRev
                }
                return
            }

            if let existing = mapPtr {
                if let commandQueue, let fence = commandQueue.makeCommandBuffer() {
                    fence.commit()
                    fence.waitUntilCompleted()
                }
                maplibre_visionos_destroy(existing)
                mapPtr = nil
            }

            let centerLat = parent.controller.latestFrame?.ownship.latitudeDeg ?? parent.orbit.latitude
            let centerLon = parent.controller.latestFrame?.ownship.longitudeDeg ?? parent.orbit.longitude
            let runwaysGeoJson = (desiredBody.isEarthWgs84 && desiredLayers.runwaysAndRoute && desiredLayers.airportRunways)
                ? parent.controller.runwaysGeoJson(
                    centerLatitudeDeg: centerLat,
                    centerLongitudeDeg: centerLon,
                    radiusKm: 130.0
                )
                : EarthStyleBuilder.emptyFeatureCollection
            let routeGeoJson = (desiredLayers.runwaysAndRoute && desiredLayers.fplRouteOverlay)
                ? parent.controller.flightPlanGeoJson()
                : EarthStyleBuilder.emptyFeatureCollection

            let styleJson = desiredBody.isEarthWgs84
                ? EarthStyleBuilder.buildEarthStyleJson(
                    config: desiredLayers,
                    resolvedPacks: desiredPacks ?? currentResolvedPacks(),
                    runwaysGeoJson: runwaysGeoJson,
                    routeGeoJson: routeGeoJson,
                    pinnedPlate: parent.controller.pinnedMapPlate,
                    pinnedPlateVisible: parent.controller.pinnedPlateVisible,
                    pinnedPlateOpacity: parent.controller.pinnedPlateOpacity
                )
                : EarthStyleBuilder.buildPlanetaryStyleJson(
                    for: desiredBody,
                    routeGeoJson: routeGeoJson
                )

            let cacheUrl = FileManager.default.urls(for: .cachesDirectory, in: .userDomainMask).first
            let cachePath = cacheUrl?.appendingPathComponent("maplibre-pad-cache-\(desiredBody.rawValue)").path
                ?? NSTemporaryDirectory()
            try? FileManager.default.createDirectory(atPath: cachePath, withIntermediateDirectories: true)

            let created: OpaquePointer? = styleJson.withCString { styleCStr in
                cachePath.withCString { cacheCStr in
                    maplibre_visionos_create(styleCStr, width, height, cacheCStr)
                }
            }
            guard let created else { return }

            maplibre_visionos_set_opaque_environment(created, false)
            let boundsW = Double(max(view.bounds.width, 320.0))
            let effectiveRatio = min(max(Double(width) / boundsW, 1.0), 3.0)
            maplibre_visionos_set_pixel_ratio(created, effectiveRatio)

            let bodyParams = parent.controller.bodyParameters
            maplibre_visionos_set_body(
                created,
                bodyParams.renderRadiusMeters,
                bodyParams.highestGroundMeters,
                bodyParams.lowestGroundMeters
            )

            self.mapPtr = created
            self.configuredBody = desiredBody
            self.configuredLayers = desiredLayers
            self.configuredPackSet = desiredPacks
            self.configuredRouteRevision = desiredRouteRev
            self.configuredRunwayRevision = desiredRunwayRev
            self.configuredPlateRevision = desiredPlateRev
            self.mapPixelWidth = width
            self.mapPixelHeight = height
            self.needsInitialStarUpload = true
            self.lastSkySyncMediaTime = -100.0

            if let rawQueue = maplibre_visionos_command_queue(created),
               let mtlQueue = Unmanaged<AnyObject>.fromOpaque(rawQueue).takeUnretainedValue() as? MTLCommandQueue {
                self.commandQueue = mtlQueue
                view.device = mtlQueue.device
                view.drawableSize = CGSize(width: Int(width), height: Int(height))
            }
            syncCelestialSkyAndStarfield(mapPtr: created)
        }
    }
}

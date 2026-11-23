import ARKit
import QuartzCore
import RealityKit
import SwiftUI
import FlightDeckKit
import simd

/// Mixed-Reality `ImmersiveSpace` overlaying conformal 3D Synthetic Vision terrain (`<= 85%` opacity,
/// `>= 15%` real outside passthrough), 3D airspace curtains, ADS-B traffic callsigns, and collimated HWD strokes.
///
/// On Apple Vision Pro (`visionOS` 2.0), turning the physical Digital Crown adjusts
/// `ProgressiveImmersionStyle(immersion: 0.10...0.85)` — hard-capped at `85%` so at least `15%`
/// real-world cockpit/outside passthrough is preserved at all times — and feeds `.onImmersionChange`
/// directly into `FlightDeckController.crownTurn`.
struct SvsImmersiveView: View {
    private static let minCrownImmersion: Double = 0.10
    private static let maxCrownImmersion: Double = Double(SvsBlendWeights.maxVirtualOpacity)

    @Bindable var controller: FlightDeckController
    @Binding var isImmersiveOpen: Bool

    @State private var sceneBuilder = SvsRealityScene()
    @State private var arSession = ARKitSession()
    @State private var worldTracking = WorldTrackingProvider()

    var body: some View {
        TimelineView(.animation(minimumInterval: 1.0 / 30.0)) { timeline in
            let _ = timeline.date
            RealityView { content in
                content.add(sceneBuilder.rootEntity)
                refreshImmersiveFrame()
            } update: { _ in
                refreshImmersiveFrame()
            }
            .gesture(
                SpatialTapGesture()
                    .targetedToAnyEntity()
                    .onEnded { value in
                        if let hit = SvsRealityScene.resolveTrafficHit(from: value.entity) {
                            controller.toggleDesignatedTrack(hit.trackId)
                        }
                    }
            )
        }
        .onImmersionChange { _, newContext in
            if let amount = newContext.amount {
                let span = max(Self.maxCrownImmersion - Self.minCrownImmersion, 0.01)
                let normalized = min(max((amount - Self.minCrownImmersion) / span, 0.0), 1.0)
                controller.crownTurn = Float(normalized)
            }
        }
        .task {
            isImmersiveOpen = true
            if WorldTrackingProvider.isSupported {
                try? await arSession.run([worldTracking])
            }
        }
        .onDisappear {
            arSession.stop()
            isImmersiveOpen = false
        }
    }

    @MainActor
    private func refreshImmersiveFrame() {
        let nowSec = CACurrentMediaTime()
        if worldTracking.state == .running,
           let anchor = worldTracking.queryDeviceAnchor(atTimestamp: nowSec) {
            let euler = Self.extractHeadEulerRadians(from: anchor.originFromAnchorTransform)
            controller.updateHeadPose(yawRad: euler.yaw, pitchRad: euler.pitch, rollRad: euler.roll)
        }
        if let frame = controller.stepFrame(nowMs: UInt64(max(nowSec * 1000.0, 0))) {
            sceneBuilder.update(with: frame, designatedTrackId: controller.designatedTrackId)
        }
    }

    /// Extracts cockpit-relative `(yaw, pitch, roll)` in radians from `originFromAnchorTransform`
    /// (`visionOS` camera convention: `+X` right, `+Y` up, `-Z` forward).
    private static func extractHeadEulerRadians(
        from m: simd_float4x4
    ) -> (yaw: Float, pitch: Float, roll: Float) {
        let forward = simd_normalize(SIMD3<Float>(-m.columns.2.x, -m.columns.2.y, -m.columns.2.z))
        let right = simd_normalize(SIMD3<Float>(m.columns.0.x, m.columns.0.y, m.columns.0.z))
        let yaw = atan2(forward.x, -forward.z)
        let pitch = asin(min(max(forward.y, -1.0), 1.0))
        let roll = atan2(-right.y, m.columns.1.y)
        return (yaw, pitch, roll)
    }
}

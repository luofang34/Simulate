import SwiftUI
import FlightDeckKit

/// Top-trailing view-mode switcher (`Map | SVS`) and context-specific perspective / orientation stack.
///
/// Design rationale:
/// - `Map` is the exocentric EFB/GCS situational globe (`Free` / `North Up` / `Track Up` follow lock,
///   dynamic `CompassRose`, and `2D/3D` tilt). Conformal boresight HUD never appears in `Map`.
/// - `SVS` is the egocentric vehicle viewport (`FPV` 1st-person cockpit camera vs. `Chase` 3rd-person
///   trail camera). Conformal `HUD` is strictly a 1st-person boresight instrument (`FPV` only);
///   `Chase` (and `FPV` with `HUD` off) supports touch look/orbit with 1-tap `Recenter`.
struct PadViewControlsCluster: View {
    @Bindable var controller: FlightDeckController
    @Binding var primaryMode: PadPrimaryViewMode
    @Binding var svsSubMode: SvsSubMode
    @Binding var svsLookOffset: SvsLookOffset
    @Binding var followMode: MapFollowMode
    @Binding var northLocked: Bool
    @Binding var orbit: OrbitCameraState
    @Binding var showHudOverlay: Bool
    let layers: PadLayerConfiguration
    let effectiveZoom: Double

    var body: some View {
        VStack(alignment: .trailing, spacing: 10) {
            primaryModeCapsule

            if primaryMode == .map {
                mapNavigationPill
            } else {
                svsPerspectivePill
            }
        }
    }

    // MARK: - Top `Map | SVS` Switcher

    private var primaryModeCapsule: some View {
        HStack(spacing: 3) {
            ForEach(PadPrimaryViewMode.allCases) { mode in
                let active = (primaryMode == mode)
                Button {
                    withAnimation(.easeInOut(duration: 0.18)) {
                        switchPrimaryMode(to: mode)
                    }
                } label: {
                    HStack(spacing: 5) {
                        Image(systemName: mode == .map ? "map" : "viewfinder")
                            .font(.caption.weight(.semibold))
                            .foregroundStyle(active ? PadTheme.commanded : PadTheme.label)
                        Text(mode.rawValue)
                            .font(.caption.weight(.bold))
                            .foregroundStyle(active ? PadTheme.value : PadTheme.label)
                    }
                    .padding(.horizontal, 12)
                    .padding(.vertical, 7)
                    .background(
                        active ? PadTheme.commanded.opacity(0.18) : Color.clear,
                        in: Capsule()
                    )
                    .overlay(
                        Capsule()
                            .strokeBorder(active ? PadTheme.commanded.opacity(0.65) : Color.clear, lineWidth: 1)
                    )
                }
                .buttonStyle(.plain)
            }
        }
        .padding(3)
        .cockpitFloatingCapsule()
    }

    // MARK: - `Map` Controls (`FollowMode` Free / North Up / Track Up + `CompassRose` + `2D/3D`)

    private var mapNavigationPill: some View {
        let is3DPitched = orbit.pitchDeg > 8.0

        return VStack(spacing: 2) {
            Button {
                cycleFollowMode()
            } label: {
                VStack(spacing: 1) {
                    Image(systemName: followMode.symbolName)
                        .font(.system(size: 15, weight: .semibold))
                    Text(followMode.shortTag)
                        .font(.system(size: 8, weight: .heavy, design: .monospaced))
                }
                .foregroundStyle(followTint)
                .frame(width: PadMetrics.buttonSize, height: PadMetrics.buttonSize)
            }
            .buttonStyle(.plain)
            .accessibilityLabel("Map Follow and Orientation Lock")

            pillDivider

            Button {
                withAnimation(.spring(response: 0.25, dampingFraction: 0.85)) {
                    orbit.bearingDeg = 0.0
                    northLocked = true
                    if followMode == .trackUp {
                        followMode = .northUp
                    }
                }
            } label: {
                CompassRoseIcon(bearingDeg: orbit.bearingDeg, northLocked: northLocked || followMode == .northUp)
                    .frame(width: PadMetrics.buttonSize, height: PadMetrics.buttonSize)
            }
            .buttonStyle(.plain)
            .accessibilityLabel("Reset Bearing to True North")

            pillDivider

            Button {
                withAnimation(.spring(response: 0.28, dampingFraction: 0.86)) {
                    if is3DPitched {
                        orbit.pitchDeg = 0.0
                    } else {
                        orbit.zoom = max(orbit.zoom, 6.5)
                        orbit.pitchDeg = 58.0
                    }
                }
            } label: {
                Text(is3DPitched ? "3D" : "2D")
                    .font(.system(size: 13, weight: .heavy, design: .monospaced))
                    .foregroundStyle(is3DPitched ? PadTheme.commanded : PadTheme.value)
                    .frame(width: PadMetrics.buttonSize, height: 38)
            }
            .buttonStyle(.plain)
            .accessibilityLabel("Toggle 2D or 3D Map Tilt")
        }
        .padding(3)
        .cockpitFloatingChrome(cornerRadius: PadMetrics.pillCornerRadius)
    }

    // MARK: - `SVS` Controls (`FPV | Chase` + `HUD` in `FPV` + `Recenter` when offset)

    private var svsPerspectivePill: some View {
        let canLookAround = (svsSubMode == .chase) || !showHudOverlay

        return VStack(spacing: 2) {
            ForEach(SvsSubMode.allCases) { sub in
                let selected = (svsSubMode == sub)
                Button {
                    withAnimation(.easeInOut(duration: 0.16)) {
                        svsSubMode = sub
                        if sub == .fpv && showHudOverlay {
                            svsLookOffset.reset()
                        }
                    }
                } label: {
                    VStack(spacing: 1) {
                        Image(systemName: sub == .fpv ? "airplane.circle" : "camera.metering.center.weighted")
                            .font(.system(size: 12, weight: .semibold))
                        Text(sub.rawValue)
                            .font(.system(size: 10, weight: .bold, design: .monospaced))
                    }
                    .foregroundStyle(selected ? PadTheme.commanded : PadTheme.label)
                    .frame(width: 54, height: 40)
                    .background(
                        selected ? PadTheme.commanded.opacity(0.18) : Color.clear,
                        in: RoundedRectangle(cornerRadius: 14, style: .continuous)
                    )
                }
                .buttonStyle(.plain)
            }

            if svsSubMode == .fpv {
                pillDivider

                Button {
                    showHudOverlay.toggle()
                    if showHudOverlay {
                        svsLookOffset.reset()
                    }
                } label: {
                    VStack(spacing: 1) {
                        Image(systemName: showHudOverlay ? "scope" : "eye.slash")
                            .font(.system(size: 11, weight: .semibold))
                        Text("HUD")
                            .font(.system(size: 10, weight: .heavy, design: .monospaced))
                    }
                    .foregroundStyle(showHudOverlay ? PadTheme.active : PadTheme.label)
                    .frame(width: 54, height: 38)
                    .background(
                        showHudOverlay ? PadTheme.active.opacity(0.18) : Color.clear,
                        in: RoundedRectangle(cornerRadius: 14, style: .continuous)
                    )
                }
                .buttonStyle(.plain)
                .accessibilityLabel("Toggle Conformal Boresight HUD")
            }

            if canLookAround, svsLookOffset.isNonDefault {
                pillDivider

                Button {
                    withAnimation(.spring(response: 0.24, dampingFraction: 0.86)) {
                        svsLookOffset.reset()
                    }
                } label: {
                    VStack(spacing: 1) {
                        Image(systemName: "dot.scope")
                            .font(.system(size: 12, weight: .semibold))
                        Text("ALIGN")
                            .font(.system(size: 8.5, weight: .heavy, design: .monospaced))
                    }
                    .foregroundStyle(PadTheme.caution)
                    .frame(width: 54, height: 36)
                }
                .buttonStyle(.plain)
                .accessibilityLabel("Recenter Camera Boresight")
            }
        }
        .padding(4)
        .cockpitFloatingChrome(cornerRadius: 18)
    }

    private var pillDivider: some View {
        Rectangle()
            .fill(PadTheme.separator)
            .frame(width: 28, height: 1)
    }

    private var followTint: Color {
        switch followMode {
        case .free: PadTheme.value
        case .northUp: PadTheme.commanded
        case .trackUp: PadTheme.caution
        }
    }

    private func switchPrimaryMode(to next: PadPrimaryViewMode) {
        guard next != primaryMode else { return }
        if next == .svs {
            svsLookOffset.reset()
            primaryMode = .svs
        } else {
            if let own = controller.latestFrame?.ownship,
               let lat = own.latitudeDeg,
               let lon = own.longitudeDeg {
                orbit.latitude = lat
                orbit.longitude = lon
                orbit.zoom = max(orbit.zoom, 9.2)
                orbit.pitchDeg = 0.0
                orbit.bearingDeg = northLocked ? 0.0 : (own.trueHeadingDeg ?? own.groundTrackTrueDeg ?? 0.0)
            }
            primaryMode = .map
        }
    }

    private func cycleFollowMode() {
        let next = followMode.nextCycled
        followMode = next
        switch next {
        case .northUp:
            northLocked = true
            orbit.bearingDeg = 0.0
            centerOrbitOnOwnshipIfValid()
        case .trackUp:
            northLocked = false
            if let own = controller.latestFrame?.ownship {
                orbit.bearingDeg = own.trueHeadingDeg ?? own.groundTrackTrueDeg ?? orbit.bearingDeg
            }
            centerOrbitOnOwnshipIfValid()
        case .free:
            break
        }
    }

    private func centerOrbitOnOwnshipIfValid() {
        guard let own = controller.latestFrame?.ownship,
              let lat = own.latitudeDeg,
              let lon = own.longitudeDeg else {
            return
        }
        orbit.latitude = lat
        orbit.longitude = lon
        if orbit.zoom < 6.5 {
            orbit.zoom = 9.0
        }
    }
}

/// Vector compass needle rotating dynamically with `-bearingDeg` (`red` north tip, `N` badge when locked).
private struct CompassRoseIcon: View {
    let bearingDeg: Double
    let northLocked: Bool

    var body: some View {
        ZStack {
            Circle()
                .strokeBorder(northLocked ? PadTheme.commanded.opacity(0.55) : PadTheme.separator, lineWidth: 1)
                .frame(width: 30, height: 30)

            VStack(spacing: 0) {
                TrianglePointer()
                    .fill(PadTheme.warning)
                    .frame(width: 7, height: 10)
                TrianglePointer()
                    .fill(PadTheme.label)
                    .frame(width: 7, height: 10)
                    .rotationEffect(.degrees(180))
            }
            .rotationEffect(.degrees(-bearingDeg))

            if abs(bearingDeg.truncatingRemainder(dividingBy: 360.0)) < 1.0, northLocked {
                Text("N")
                    .font(.system(size: 7, weight: .heavy, design: .monospaced))
                    .foregroundStyle(PadTheme.commanded)
                    .offset(y: -11)
            }
        }
    }
}

private struct TrianglePointer: Shape {
    func path(in rect: CGRect) -> Path {
        var path = Path()
        path.move(to: CGPoint(x: rect.midX, y: rect.minY))
        path.addLine(to: CGPoint(x: rect.maxX, y: rect.maxY))
        path.addLine(to: CGPoint(x: rect.minX, y: rect.maxY))
        path.closeSubpath()
        return path
    }
}

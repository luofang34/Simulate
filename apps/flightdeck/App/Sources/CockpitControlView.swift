import IndicateAppleDisplay
import QuartzCore
import SwiftUI
import FlightDeckKit

/// Primary cockpit control & instrument flight-deck window for Apple Vision Pro (`visionOS`).
struct CockpitControlView: View {
    @Bindable var controller: FlightDeckController
    @Binding var isImmersiveOpen: Bool
    @Binding var useProgressiveCrownStyle: Bool

    @Environment(\.openImmersiveSpace) private var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace
    @Environment(\.openWindow) private var openWindow

    @State private var crownScalar: Double = 0.55

    var body: some View {
        VStack(spacing: 14) {
            headerBar
            alertBannerIfActive
            HStack(alignment: .top, spacing: 14) {
                leftControlsColumn
                    .frame(width: 390)
                instrumentsCenterColumn
                rightSituationalColumn
                    .frame(width: 350)
            }
        }
        .padding(20)
        .onChange(of: crownScalar) { _, newValue in
            controller.crownTurn = Float(newValue)
        }
        .onChange(of: controller.crownTurn) { _, newTurn in
            let asDouble = Double(newTurn)
            if abs(crownScalar - asDouble) > 0.005 {
                crownScalar = asDouble
            }
        }
        .task {
            crownScalar = Double(controller.crownTurn)
            while !Task.isCancelled {
                if !isImmersiveOpen {
                    _ = controller.stepFrame(
                        nowMs: UInt64(max(CACurrentMediaTime() * 1000.0, 0))
                    )
                }
                try? await Task.sleep(nanoseconds: 33_333_333)
            }
        }
    }

    private var headerBar: some View {
        HStack(spacing: 16) {
            VStack(alignment: .leading, spacing: 2) {
                Text("SOKOLY FLIGHT DECK • \(controller.celestialBody.badgeLabel)")
                    .font(.headline.monospaced())
                Text(controller.transportStatus)
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
            Spacer()
            Menu {
                Section("Celestial Body") {
                    ForEach(PrimaryCelestialBody.allCases) { primary in
                        Button {
                            controller.setPrimaryBody(primary)
                        } label: {
                            Label(
                                "\(primary.rawValue) — \(primary.figureSummary)",
                                systemImage: primary == controller.primaryBody ? "checkmark.circle.fill" : primary.symbolName
                            )
                        }
                    }
                }
                if controller.primaryBody == .earth {
                    Section("Earth Vertical Datum") {
                        ForEach(EarthVerticalDatum.allCases) { datum in
                            Button {
                                controller.setEarthVerticalDatum(datum)
                            } label: {
                                Label(
                                    datum.title,
                                    systemImage: datum == controller.earthVerticalDatum ? "checkmark.circle.fill" : "arrow.up.and.down.circle"
                                )
                            }
                        }
                    }
                }
            } label: {
                Label(controller.celestialBody.badgeLabel, systemImage: "globe")
                    .font(.caption.monospaced())
            }
            Button {
                openWindow(id: "terrain-volume")
            } label: {
                Label("3D Terrain Volume", systemImage: "cube.transparent")
            }
            Button {
                Task {
                    if isImmersiveOpen {
                        await dismissImmersiveSpace()
                        isImmersiveOpen = false
                    } else {
                        let res = await openImmersiveSpace(id: "svs-mixed")
                        if case .opened = res {
                            isImmersiveOpen = true
                        }
                    }
                }
            } label: {
                Label(
                    isImmersiveOpen ? "Exit XR SVS" : "Enter Mixed-Reality SVS",
                    systemImage: isImmersiveOpen ? "xmark.circle.fill" : "visionpro"
                )
            }
            .buttonStyle(.borderedProminent)
            .tint(isImmersiveOpen ? .orange : .cyan)
        }
    }

    @ViewBuilder
    private var alertBannerIfActive: some View {
        if let frame = controller.latestFrame,
           frame.airspaceAlertLevel != .clear,
           !frame.airspaceAlertMessage.isEmpty {
            HStack {
                Image(systemName: "exclamationmark.triangle.fill")
                Text(frame.airspaceAlertMessage)
                    .font(.subheadline.weight(.bold).monospaced())
                Spacer()
                Text(alertBadge(frame.airspaceAlertLevel))
                    .font(.caption.monospaced().bold())
            }
            .padding(.horizontal, 14)
            .padding(.vertical, 8)
            .background(alertTint(frame.airspaceAlertLevel).opacity(0.28))
            .clipShape(RoundedRectangle(cornerRadius: 10))
        }
    }

    private var leftControlsColumn: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 14) {
                crownBlendCard
                sourceSelectorCard
                ownshipReadoutCard
            }
        }
    }

    private var crownBlendCard: some View {
        let blend = controller.latestFrame?.blend ?? SvsBlendWeights(crownTurn: controller.crownTurn)
        let virtPct = Int((blend.virtualTerrainOpacity * 100.0).rounded())
        let realPct = Int((blend.passthroughFraction * 100.0).rounded())

        return VStack(alignment: .leading, spacing: 10) {
            HStack {
                Label("Digital Crown SVS / Reality Knob", systemImage: "dial.medium.fill")
                    .font(.subheadline.bold())
                Spacer()
            }
            HStack(spacing: 14) {
                RotaryBlendKnob(value: $crownScalar)
                    .frame(width: 78, height: 78)
                VStack(alignment: .leading, spacing: 6) {
                    Text("Virtual SVS: \(virtPct)% (≤85%)")
                        .font(.callout.monospaced().bold())
                        .foregroundStyle(.cyan)
                    Text("Outside Real: \(realPct)% (≥15%)")
                        .font(.callout.monospaced().bold())
                        .foregroundStyle(.green)
                    Slider(value: $crownScalar, in: 0.0...1.0)
                }
            }
            HStack(spacing: 8) {
                Button("VFR Clear (0%)") { crownScalar = 0.0 }
                Button("Haze (47%)") { crownScalar = 0.55 }
                Button("Max SVS (85%)") { crownScalar = 1.0 }
            }
            .buttonStyle(.bordered)
            .controlSize(.small)
            Toggle("Bind Hardware Digital Crown (Progressive 10–85%)", isOn: $useProgressiveCrownStyle)
                .font(.caption)
            Text("Safety floor guarantees ≥15% real outside passthrough even at full Digital Crown rotation.")
                .font(.caption2)
                .foregroundStyle(.secondary)
        }
        .padding(14)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 14))
    }

    private var sourceSelectorCard: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Telemetry Link (`avionics-link` / GDL-90 / Sim)")
                .font(.subheadline.bold())
            Picker("Telemetry Source", selection: $controller.sourceMode) {
                ForEach(TelemetrySourceMode.allCases) { mode in
                    Text(mode.title).tag(mode)
                }
            }
            .pickerStyle(.menu)

            if controller.sourceMode == .connextLiveTcp {
                HStack(spacing: 8) {
                    TextField("Host", text: $controller.connextHost)
                        .textFieldStyle(.roundedBorder)
                    Button(controller.isConnextConnected ? "Disconnect" : "Connect :\(controller.connextPort)") {
                        controller.toggleConnextConnection()
                    }
                    .buttonStyle(.borderedProminent)
                }
            } else if controller.sourceMode == .gdl90LiveUdp {
                HStack {
                    Text("UDP Port :\(controller.gdl90Port)")
                        .font(.caption.monospaced())
                    Spacer()
                    Button(controller.isGdl90Listening ? "Stop Listener" : "Listen UDP :4000") {
                        controller.toggleGdl90Listener()
                    }
                    .buttonStyle(.borderedProminent)
                }
            }
        }
        .padding(14)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 14))
    }

    private var ownshipReadoutCard: some View {
        let own = controller.latestFrame?.ownship
        return VStack(alignment: .leading, spacing: 6) {
            HStack {
                Text("Ownship GNSS & AHRS")
                    .font(.subheadline.bold())
                Spacer()
                Text(own?.callsign.isEmpty == false ? (own?.callsign ?? "") : "OWNSHIP")
                    .font(.caption.monospaced().bold())
                    .foregroundStyle(.cyan)
            }
            Grid(alignment: .leading, horizontalSpacing: 12, verticalSpacing: 4) {
                GridRow {
                    metricCell("GNSS", own?.gnssValid == true ? formatLatLon(own) : "NO FIX")
                    metricCell("P-ALT", formatOpt(own?.pressureAltitudeFt, suffix: " FT", digits: 0))
                }
                GridRow {
                    metricCell("GS", formatOpt(own?.groundSpeedKt, suffix: " KT", digits: 0))
                    metricCell("IAS", formatOpt(own?.indicatedAirspeedKt, suffix: " KT", digits: 0, missing: "NO SENSOR"))
                }
                GridRow {
                    metricCell("VSI", formatOpt(own?.verticalSpeedFpm, suffix: " FPM", digits: 0, missing: "NO SENSOR"))
                    metricCell("AGL", formatOpt(own?.aglFt, suffix: " FT", digits: 0))
                }
                GridRow {
                    metricCell("HDG/TRK", formatOpt(own?.trueHeadingDeg ?? own?.groundTrackTrueDeg, suffix: "°T", digits: 0))
                    metricCell("ATT", formatAttitude(own))
                }
            }
            if let leg = own?.activeLeg, !leg.isEmpty {
                Text("Active Leg: \(leg)")
                    .font(.caption.monospaced())
                    .foregroundStyle(.green)
            }
        }
        .padding(14)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 14))
    }

    private var instrumentsCenterColumn: some View {
        let compact = controller.latestFrame?.instruments.hwdCompactMode ?? false
        return VStack(spacing: 12) {
            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    Text("Indicate Conformal HUD (`indicate-instrument-hud`)")
                        .font(.caption.bold())
                    Spacer()
                    Text(compact ? "HWD: COMPACT GLANCE (>35°)" : "HWD: FULL CONFORMAL")
                        .font(.caption2.monospaced().bold())
                        .foregroundStyle(compact ? .orange : .green)
                }
                TransparentHudOverlay(display: controller.hudDisplay)
                    .frame(minHeight: 260)
            }

            VStack(alignment: .leading, spacing: 4) {
                Text("Vision Pro HWD Peripheral Symbology (`indicate-instrument-hmd`)")
                    .font(.caption.bold())
                TransparentHudOverlay(display: controller.hwdOverlayDisplay)
                    .frame(height: 120)
            }

            HStack(spacing: 12) {
                VStack(alignment: .leading, spacing: 4) {
                    Text("PFD")
                        .font(.caption.bold())
                    InstrumentPanel(display: controller.pfdDisplay)
                        .aspectRatio(4.0 / 3.0, contentMode: .fit)
                        .clipShape(RoundedRectangle(cornerRadius: 10))
                }
                VStack(alignment: .leading, spacing: 4) {
                    Text("HSI")
                        .font(.caption.bold())
                    InstrumentPanel(display: controller.hsiDisplay)
                        .aspectRatio(4.0 / 3.0, contentMode: .fit)
                        .clipShape(RoundedRectangle(cornerRadius: 10))
                }
            }
        }
    }

    private var rightSituationalColumn: some View {
        let traffic = controller.latestFrame?.traffic ?? []
        let airspaces = controller.latestFrame?.airspaces ?? []

        return ScrollView {
            VStack(alignment: .leading, spacing: 14) {
                VStack(alignment: .leading, spacing: 8) {
                    HStack {
                        Text("ADS-B Traffic & Callsigns (\(traffic.count))")
                            .font(.subheadline.bold())
                        Spacer()
                        Text("Tap to Lock")
                            .font(.caption2)
                            .foregroundStyle(.secondary)
                    }
                    if traffic.isEmpty {
                        Text("No active ADS-B targets in range")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    } else {
                        ForEach(traffic.prefix(10)) { t in
                            trafficRow(t)
                        }
                    }
                }
                .padding(14)
                .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 14))

                VStack(alignment: .leading, spacing: 8) {
                    Text("3D Airspace Volumes (\(airspaces.count))")
                        .font(.subheadline.bold())
                    ForEach(airspaces.prefix(8)) { vol in
                        HStack(spacing: 8) {
                            Circle()
                                .fill(Color(
                                    red: Double(vol.rgb.x),
                                    green: Double(vol.rgb.y),
                                    blue: Double(vol.rgb.z)
                                ))
                                .frame(width: 10, height: 10)
                            Text(vol.label)
                                .font(.caption2.monospaced())
                                .lineLimit(1)
                            Spacer()
                            if vol.ownshipInside {
                                Text("INSIDE")
                                    .font(.caption2.monospaced().bold())
                                    .foregroundStyle(.yellow)
                            }
                        }
                    }
                }
                .padding(14)
                .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 14))
            }
        }
    }

    private func trafficRow(_ t: TrafficTarget) -> some View {
        let isDesignated = controller.designatedTrackId == t.trackId
        let relSign = t.relAltitudeHundredsFt >= 0 ? "+" : ""
        return Button {
            controller.toggleDesignatedTrack(t.trackId)
        } label: {
            HStack(spacing: 8) {
                Text(t.threat.badgeText)
                    .font(.caption2.monospaced().bold())
                    .padding(.horizontal, 5)
                    .padding(.vertical, 2)
                    .background(threatSwiftColor(t.threat).opacity(0.25))
                    .foregroundStyle(threatSwiftColor(t.threat))
                    .clipShape(RoundedRectangle(cornerRadius: 4))
                Text(t.callsign)
                    .font(.callout.monospaced().bold())
                Spacer()
                Text("\(relSign)\(String(format: "%02d", t.relAltitudeHundredsFt))\(t.verticalTrend.arrow)")
                    .font(.caption.monospaced())
                Text(String(format: "%.1fNM", t.slantRangeNm))
                    .font(.caption.monospaced())
                    .foregroundStyle(.secondary)
            }
            .padding(.horizontal, 8)
            .padding(.vertical, 5)
            .background(isDesignated ? Color.green.opacity(0.22) : Color.clear)
            .clipShape(RoundedRectangle(cornerRadius: 8))
        }
        .buttonStyle(.plain)
    }

    private func metricCell(_ title: String, _ value: String) -> some View {
        VStack(alignment: .leading, spacing: 1) {
            Text(title)
                .font(.caption2)
                .foregroundStyle(.secondary)
            Text(value)
                .font(.caption.monospaced().bold())
        }
    }

    private func formatOpt(
        _ val: Double?,
        suffix: String,
        digits: Int,
        missing: String = "---"
    ) -> String {
        guard let val, val.isFinite else { return missing }
        return String(format: "%.\(digits)f%@", val, suffix)
    }

    private func formatLatLon(_ own: OwnshipSnapshot?) -> String {
        guard let lat = own?.latitudeDeg, let lon = own?.longitudeDeg else { return "NO FIX" }
        return String(format: "%.3f, %.3f", lat, lon)
    }

    private func formatAttitude(_ own: OwnshipSnapshot?) -> String {
        guard let p = own?.pitchDeg, let r = own?.rollDeg else { return "---" }
        return String(format: "P%+.1f° R%+.1f°", p, r)
    }

    private func alertBadge(_ level: AirspaceAlertLevel) -> String {
        switch level {
        case .clear: "CLEAR"
        case .proximity: "PROXIMITY"
        case .insideControlled: "CONTROLLED"
        case .restrictedOrTfrPenetration: "RESTRICTED / TFR"
        }
    }

    private func alertTint(_ level: AirspaceAlertLevel) -> Color {
        switch level {
        case .clear: .green
        case .proximity: .yellow
        case .insideControlled: .cyan
        case .restrictedOrTfrPenetration: .red
        }
    }

    private func threatSwiftColor(_ threat: TrafficThreatLevel) -> Color {
        switch threat {
        case .other, .proximate: .cyan
        case .trafficAdvisory: .orange
        case .resolutionAdvisory: .red
        }
    }
}

/// Interactive cockpit rotary knob for dialing virtual SVS vs. real-world passthrough (`>= 15%` outside floor).
private struct RotaryBlendKnob: View {
    @Binding var value: Double
    @State private var dragBase: Double?

    var body: some View {
        let clamped = min(max(value, 0.0), 1.0)
        let angleDeg = -135.0 + clamped * 270.0
        ZStack {
            Circle()
                .trim(from: 0.0, to: 0.75 * 0.85)
                .stroke(Color.white.opacity(0.18), style: StrokeStyle(lineWidth: 6, lineCap: .round))
                .rotationEffect(.degrees(135))
            Circle()
                .trim(from: 0.75 * 0.85, to: 0.75)
                .stroke(Color.green.opacity(0.75), style: StrokeStyle(lineWidth: 6, lineCap: .round))
                .rotationEffect(.degrees(135))
            Circle()
                .trim(from: 0.0, to: 0.75 * 0.85 * clamped)
                .stroke(Color.cyan, style: StrokeStyle(lineWidth: 6, lineCap: .round))
                .rotationEffect(.degrees(135))
            Circle()
                .fill(.ultraThinMaterial)
                .padding(10)
            Capsule()
                .fill(Color.cyan)
                .frame(width: 4, height: 18)
                .offset(y: -14)
                .rotationEffect(.degrees(angleDeg))
        }
        .contentShape(Circle())
        .gesture(
            DragGesture(minimumDistance: 0)
                .onChanged { drag in
                    if dragBase == nil {
                        dragBase = value
                    }
                    let base = dragBase ?? value
                    let delta = (-drag.translation.height + drag.translation.width) / 140.0
                    value = min(max(base + delta, 0.0), 1.0)
                }
                .onEnded { _ in
                    dragBase = nil
                }
        )
        .accessibilityLabel("SVS Virtual to Reality Rotary Knob")
        .accessibilityValue("\(Int((clamped * 85.0).rounded())) percent virtual, \(100 - Int((clamped * 85.0).rounded())) percent real")
    }
}

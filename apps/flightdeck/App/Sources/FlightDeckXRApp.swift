import IndicateAppleDisplay
import SwiftUI
import FlightDeckKit

@main
struct FlightDeckXRApp: App {
    private static let crownProgressiveStyle = ProgressiveImmersionStyle(
        immersion: 0.10...Double(SvsBlendWeights.maxVirtualOpacity),
        initialAmount: 0.55
    )

    @State private var controller = FlightDeckController(body: .earthWgs84Egm96)
    @State private var isImmersiveOpen = false
    @State private var useProgressiveCrownStyle = true
    @State private var immersionStyle: ImmersionStyle = Self.crownProgressiveStyle

    var body: some Scene {
        WindowGroup("Sokoly Flight Deck", id: "cockpit-deck") {
            CockpitControlView(
                controller: controller,
                isImmersiveOpen: $isImmersiveOpen,
                useProgressiveCrownStyle: $useProgressiveCrownStyle
            )
            .onChange(of: useProgressiveCrownStyle) { _, progressive in
                immersionStyle = progressive ? Self.crownProgressiveStyle : .mixed
            }
        }
        .defaultSize(width: 1320, height: 880)

        WindowGroup("3D Terrain Situation Volume", id: "terrain-volume") {
            InstrumentVolume(display: controller.terrainVolumeDisplay)
        }
        .windowStyle(.volumetric)
        .defaultSize(width: 0.46, height: 0.32, depth: 0.46, in: .meters)

        ImmersiveSpace(id: "svs-mixed") {
            SvsImmersiveView(
                controller: controller,
                isImmersiveOpen: $isImmersiveOpen
            )
        }
        .immersionStyle(
            selection: $immersionStyle,
            in: .mixed,
            Self.crownProgressiveStyle
        )
    }
}

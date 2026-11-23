// swift-tools-version: 6.1

import PackageDescription

let package = Package(
    name: "FlightDeckKit",
    platforms: [
        .visionOS(.v2),
        .iOS(.v17),
        .macOS(.v14),
    ],
    products: [
        .library(name: "FlightDeckKit", targets: ["FlightDeckKit"]),
    ],
    dependencies: [
        .package(
            url: "https://github.com/luofang34/IndicateAppleDisplay.git",
            revision: "e3f3207c00230f383899386e97e6e0840ba9b9c8"
        ),
    ],
    targets: [
        .binaryTarget(
            name: "FlightDeckFFI",
            path: "../../../../target/flightdeck-ffi/FlightDeckFFI.xcframework"
        ),
        .target(
            name: "flightdeck_ffiFFI",
            dependencies: ["FlightDeckFFI"]
        ),
        .target(
            name: "FlightDeckKit",
            dependencies: [
                "flightdeck_ffiFFI",
                .product(name: "IndicateAppleDisplay", package: "IndicateAppleDisplay"),
            ]
        ),
        .testTarget(
            name: "FlightDeckKitTests",
            dependencies: [
                "FlightDeckKit",
                .product(name: "IndicateAppleDisplay", package: "IndicateAppleDisplay"),
            ]
        ),
    ]
)

import Foundation
import simd

/// Errors raised while decoding `FDKF` or `FDKG` binary wire buffers.
public enum FlightDeckWireError: Error, Equatable, Sendable {
    case invalidMagic
    case unsupportedVersion(UInt32)
    case truncatedBuffer
    case invalidUtf8
}

/// Decodes a complete `FDKF` v1 binary snapshot into an ``FlightDeckFrame``.
public func decodeFlightDeckFrame(
    _ bytes: UnsafeBufferPointer<UInt8>,
    generation: UInt32
) throws -> FlightDeckFrame {
    var r = WireReader(bytes: bytes)
    guard try r.readBytes(count: 4) == [0x46, 0x44, 0x4B, 0x46] else {
        throw FlightDeckWireError.invalidMagic
    }
    let version = try r.readU32()
    guard version == 1 else {
        throw FlightDeckWireError.unsupportedVersion(version)
    }
    let modeRaw = try r.readU8()
    let gnssValid = (try r.readU8()) != 0
    let groundVectorValid = (try r.readU8()) != 0
    let hwdCompactMode = (try r.readU8()) != 0

    let blend = SvsBlendWeights(
        crownTurn: try r.readF32(),
        virtualTerrainOpacity: try r.readF32(),
        passthroughFraction: try r.readF32(),
        airspaceFillOpacity: try r.readF32(),
        airspaceRimOpacity: try r.readF32(),
        symbologyOpacity: try r.readF32()
    )

    let ownship = try decodeOwnship(&r, gnssValid: gnssValid, groundVectorValid: groundVectorValid)
    let terrain = try decodeTerrain(&r)
    let (alertLevel, alertMessage, airspaces) = try decodeAirspaces(&r)
    let traffic = try decodeTraffic(&r)
    let instruments = try decodeInstruments(&r, hwdCompactMode: hwdCompactMode)

    return FlightDeckFrame(
        generation: generation,
        sourceMode: TelemetrySourceMode(rawValue: modeRaw) ?? .njCorridorCruise,
        blend: blend,
        ownship: ownship,
        terrain: terrain,
        airspaceAlertLevel: alertLevel,
        airspaceAlertMessage: alertMessage,
        airspaces: airspaces,
        traffic: traffic,
        instruments: instruments
    )
}

/// Decodes an `FDKG` v1 binary glyph pack into ``InstrumentGlyphPack``.
public func decodeInstrumentGlyphPack(_ bytes: UnsafeBufferPointer<UInt8>) throws -> InstrumentGlyphPack {
    var r = WireReader(bytes: bytes)
    guard try r.readBytes(count: 4) == [0x46, 0x44, 0x4B, 0x47] else {
        throw FlightDeckWireError.invalidMagic
    }
    let version = try r.readU32()
    guard version == 1 else {
        throw FlightDeckWireError.unsupportedVersion(version)
    }
    let cellWidth = try r.readU8()
    let cellHeight = try r.readU8()
    _ = try r.readBytes(count: 2)
    let hash = try r.readBytes(count: 32)
    let glyphCount = Int(try r.readU32())
    var glyphs: [InstrumentGlyph] = []
    glyphs.reserveCapacity(glyphCount)
    for _ in 0..<glyphCount {
        let scalar = try r.readU32()
        let advance = try r.readU8()
        let rows = try r.readLengthPrefixedBytes()
        glyphs.append(InstrumentGlyph(scalar: scalar, advance: advance, rows: rows))
    }
    return InstrumentGlyphPack(
        cellWidth: cellWidth,
        cellHeight: cellHeight,
        contentHash: hash,
        glyphs: glyphs
    )
}

/// Decodes framed outbound Connext transport write batches (`u32 count` + `[u32 len, bytes]...`).
public func decodeWriteBatch(_ bytes: UnsafeBufferPointer<UInt8>) -> [[UInt8]] {
    guard !bytes.isEmpty else { return [] }
    var r = WireReader(bytes: bytes)
    guard let count = try? r.readU32() else { return [] }
    var writes: [[UInt8]] = []
    writes.reserveCapacity(Int(count))
    for _ in 0..<count {
        guard let packet = try? r.readLengthPrefixedBytes() else { break }
        writes.append(packet)
    }
    return writes
}

private func decodeOwnship(
    _ r: inout WireReader,
    gnssValid: Bool,
    groundVectorValid: Bool
) throws -> OwnshipSnapshot {
    let lat = try r.readOptF64()
    let lon = try r.readOptF64()
    let altM = try r.readOptF64()
    let paFt = try r.readOptF64()
    let gsKt = try r.readOptF64()
    let iasKt = try r.readOptF64()
    let vsFpm = try r.readOptF64()
    let magHdg = try r.readOptF64()
    let trueHdg = try r.readOptF64()
    let trkDeg = try r.readOptF64()
    let pitch = try r.readOptF64()
    let roll = try r.readOptF64()
    let aglFt = try r.readOptF64()
    let callsign = try r.readString()
    let activeLeg = try r.readString()
    let connextRecords = try r.readU64()
    let gdl90Frames = try r.readU64()

    return OwnshipSnapshot(
        gnssValid: gnssValid,
        groundVectorValid: groundVectorValid,
        latitudeDeg: lat,
        longitudeDeg: lon,
        altitudeMslM: altM,
        pressureAltitudeFt: paFt,
        groundSpeedKt: gsKt,
        indicatedAirspeedKt: iasKt,
        verticalSpeedFpm: vsFpm,
        magneticHeadingDeg: magHdg,
        trueHeadingDeg: trueHdg,
        groundTrackTrueDeg: trkDeg,
        pitchDeg: pitch,
        rollDeg: roll,
        aglFt: aglFt,
        callsign: callsign,
        activeLeg: activeLeg,
        connextRecords: connextRecords,
        gdl90Frames: gdl90Frames
    )
}

private func decodeTerrain(_ r: inout WireReader) throws -> TerrainSurfaceMesh {
    let vCount = Int(try r.readU32())
    var positions: [SIMD3<Float>] = []
    positions.reserveCapacity(vCount)
    for _ in 0..<vCount {
        positions.append(try r.readVec3())
    }
    var normals: [SIMD3<Float>] = []
    normals.reserveCapacity(vCount)
    for _ in 0..<vCount {
        normals.append(try r.readVec3())
    }
    var colors: [SIMD4<UInt8>] = []
    colors.reserveCapacity(vCount)
    for _ in 0..<vCount {
        let c = try r.readBytes(count: 4)
        colors.append(SIMD4<UInt8>(c[0], c[1], c[2], c[3]))
    }
    let idxCount = Int(try r.readU32())
    var indices: [UInt32] = []
    indices.reserveCapacity(idxCount)
    for _ in 0..<idxCount {
        indices.append(try r.readU32())
    }
    let rwyCount = Int(try r.readU32())
    var runways: [RunwayOverlay3D] = []
    runways.reserveCapacity(rwyCount)
    for _ in 0..<rwyCount {
        let label = try r.readString()
        let corners = [try r.readVec3(), try r.readVec3(), try r.readVec3(), try r.readVec3()]
        let centerline = [try r.readVec3(), try r.readVec3()]
        let gateCount = Int(try r.readU32())
        var gates: [[SIMD3<Float>]] = []
        gates.reserveCapacity(gateCount)
        for _ in 0..<gateCount {
            gates.append([try r.readVec3(), try r.readVec3(), try r.readVec3(), try r.readVec3()])
        }
        runways.append(
            RunwayOverlay3D(
                label: label,
                cornersEnuM: corners,
                centerlineEnuM: centerline,
                glideslopeGatesEnuM: gates
            )
        )
    }
    return TerrainSurfaceMesh(
        positionsEnuM: positions,
        normalsEnu: normals,
        colorsRgba: colors,
        indices: indices,
        runways: runways
    )
}

private func decodeAirspaces(
    _ r: inout WireReader
) throws -> (AirspaceAlertLevel, String, [AirspaceVolumeMesh]) {
    let alertLevel = AirspaceAlertLevel(rawValue: try r.readU8()) ?? .clear
    let alertMessage = try r.readString()
    let count = Int(try r.readU32())
    var volumes: [AirspaceVolumeMesh] = []
    volumes.reserveCapacity(count)
    for _ in 0..<count {
        let key = try r.readString()
        let label = try r.readString()
        let style = AirspaceClassStyle(rawValue: try r.readU8()) ?? .classE
        let active = (try r.readU8()) != 0
        let inside = (try r.readU8()) != 0
        let rgb = try r.readVec3()
        let anchor = try r.readVec3()
        let wpCount = Int(try r.readU32())
        var wallPos: [SIMD3<Float>] = []
        wallPos.reserveCapacity(wpCount)
        for _ in 0..<wpCount {
            wallPos.append(try r.readVec3())
        }
        let wiCount = Int(try r.readU32())
        var wallIdx: [UInt32] = []
        wallIdx.reserveCapacity(wiCount)
        for _ in 0..<wiCount {
            wallIdx.append(try r.readU32())
        }
        let rimCount = Int(try r.readU32())
        var rims: [(SIMD3<Float>, SIMD3<Float>)] = []
        rims.reserveCapacity(rimCount)
        for _ in 0..<rimCount {
            rims.append((try r.readVec3(), try r.readVec3()))
        }
        volumes.append(
            AirspaceVolumeMesh(
                airspaceKey: key,
                label: label,
                style: style,
                active: active,
                ownshipInside: inside,
                rgb: rgb,
                labelAnchorEnuM: anchor,
                wallPositionsEnuM: wallPos,
                wallIndices: wallIdx,
                rimSegmentsEnuM: rims
            )
        )
    }
    return (alertLevel, alertMessage, volumes)
}

private func decodeTraffic(_ r: inout WireReader) throws -> [TrafficTarget] {
    let count = Int(try r.readU32())
    var out: [TrafficTarget] = []
    out.reserveCapacity(count)
    for _ in 0..<count {
        let trackId = try r.readU64()
        let hex = try r.readString()
        let callsign = try r.readString()
        let placard = try r.readString()
        let pos = try r.readVec3()
        let vel = try r.readVec3()
        let trkRaw = try r.readF32()
        let rangeNm = try r.readF32()
        let relAlt = try r.readI32()
        let trend = TrafficVerticalTrend(rawValue: try r.readU8()) ?? .level
        let threat = TrafficThreatLevel(rawValue: try r.readU8()) ?? .other
        let onGround = (try r.readU8()) != 0
        let active = (try r.readU8()) != 0
        let tCpa = try r.readF32()
        let dCpa = try r.readF32()
        out.append(
            TrafficTarget(
                trackId: trackId,
                addressHex: hex,
                callsign: callsign,
                placard: placard,
                offsetEnuM: pos,
                velocityEnuMps: vel,
                trackTrueDeg: trkRaw.isFinite ? trkRaw : nil,
                slantRangeNm: rangeNm,
                relAltitudeHundredsFt: relAlt,
                verticalTrend: trend,
                threat: threat,
                onGround: onGround,
                active: active,
                timeToCpaS: tCpa,
                cpaHorizontalM: dCpa
            )
        )
    }
    return out
}

private func decodeInstruments(
    _ r: inout WireReader,
    hwdCompactMode: Bool
) throws -> InstrumentSceneSet {
    let strokeCount = Int(try r.readU32())
    var strokes: [(SIMD3<Float>, SIMD3<Float>)] = []
    strokes.reserveCapacity(strokeCount)
    for _ in 0..<strokeCount {
        strokes.append((try r.readVec3(), try r.readVec3()))
    }
    let hwdOverlay = try r.readLengthPrefixedBytes()
    let hud = try r.readLengthPrefixedBytes()
    let pfd = try r.readLengthPrefixedBytes()
    let hsi = try r.readLengthPrefixedBytes()
    let vol = try r.readLengthPrefixedBytes()
    return InstrumentSceneSet(
        hwdCompactMode: hwdCompactMode,
        hwd3dStrokesEnu: strokes,
        hwdOverlayScene: hwdOverlay,
        hudScene: hud,
        pfdScene: pfd,
        hsiScene: hsi,
        terrainVolumeScene: vol
    )
}

private struct WireReader {
    let bytes: UnsafeBufferPointer<UInt8>
    var cursor: Int = 0

    mutating func readU8() throws -> UInt8 {
        guard cursor < bytes.count else { throw FlightDeckWireError.truncatedBuffer }
        let b = bytes[cursor]
        cursor += 1
        return b
    }

    mutating func readBytes(count: Int) throws -> [UInt8] {
        guard count >= 0, cursor + count <= bytes.count else {
            throw FlightDeckWireError.truncatedBuffer
        }
        let slice = Array(bytes[cursor..<(cursor + count)])
        cursor += count
        return slice
    }

    mutating func readU32() throws -> UInt32 {
        let b = try readBytes(count: 4)
        return UInt32(b[0])
            | (UInt32(b[1]) << 8)
            | (UInt32(b[2]) << 16)
            | (UInt32(b[3]) << 24)
    }

    mutating func readI32() throws -> Int32 {
        Int32(bitPattern: try readU32())
    }

    mutating func readU64() throws -> UInt64 {
        let lo = UInt64(try readU32())
        let hi = UInt64(try readU32())
        return lo | (hi << 32)
    }

    mutating func readF32() throws -> Float {
        Float(bitPattern: try readU32())
    }

    mutating func readOptF64() throws -> Double? {
        let val = Double(bitPattern: try readU64())
        return val.isFinite ? val : nil
    }

    mutating func readVec3() throws -> SIMD3<Float> {
        SIMD3<Float>(try readF32(), try readF32(), try readF32())
    }

    mutating func readLengthPrefixedBytes() throws -> [UInt8] {
        let len = Int(try readU32())
        return try readBytes(count: len)
    }

    mutating func readString() throws -> String {
        let raw = try readLengthPrefixedBytes()
        guard let str = String(bytes: raw, encoding: .utf8) else {
            throw FlightDeckWireError.invalidUtf8
        }
        return str
    }
}

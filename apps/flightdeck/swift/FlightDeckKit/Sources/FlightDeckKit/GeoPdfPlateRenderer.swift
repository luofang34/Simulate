import CoreGraphics
import CoreText
import Foundation
import ImageIO
import UniformTypeIdentifiers

/// Georeferenced `[NW, NE, SE, SW]` `WGS-84` corner solution extracted from an ISO 32000-1 GeoPDF `/VP` dictionary.
public struct GeoPdfRegistration: Equatable, Sendable {
    /// Eight `f64` coordinates `[nwLon, nwLat, neLon, neLat, seLon, seLat, swLon, swLat]` in degrees.
    public let cornersLonLat: [Double]
    public let centerLatitudeDeg: Double
    public let centerLongitudeDeg: Double
    /// Georeferenced planview `/BBox` in PDF user-space points (`nil` for procedurally synthesized canvases).
    public let viewportBoxPdf: CGRect?

    public init(
        cornersLonLat: [Double],
        centerLatitudeDeg: Double,
        centerLongitudeDeg: Double,
        viewportBoxPdf: CGRect? = nil
    ) {
        self.cornersLonLat = cornersLonLat
        self.centerLatitudeDeg = centerLatitudeDeg
        self.centerLongitudeDeg = centerLongitudeDeg
        self.viewportBoxPdf = viewportBoxPdf
    }
}

/// CoreGraphics ISO 32000-1 GeoPDF `/VP` corner solver, rasterizer, and procedural approach plate synthesizer.
public enum GeoPdfPlateRenderer {
    /// Transparent 1x1 PNG seeded at map initialization before a procedure plate is pinned.
    public static func placeholderTransparentPngUrl() -> URL {
        let dir = plateCacheDirectory()
        let url = dir.appendingPathComponent("empty_plate_1x1.png")
        if FileManager.default.fileExists(atPath: url.path) {
            return url
        }
        let colorSpace = CGColorSpaceCreateDeviceRGB()
        if let ctx = CGContext(
            data: nil,
            width: 1,
            height: 1,
            bitsPerComponent: 8,
            bytesPerRow: 4,
            space: colorSpace,
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ),
           let cgImg = ctx.makeImage() {
            _ = writePng(cgImage: cgImg, to: url)
        }
        return url
    }

    /// Extracts ISO 32000-1 `/VP` (`/BBox`, `/Measure` -> `/LPTS`, `/GPTS`) affine corner registration from page 1.
    public static func extractGeoRegistration(from pdfData: Data) -> GeoPdfRegistration? {
        guard let provider = CGDataProvider(data: pdfData as CFData),
              let doc = CGPDFDocument(provider),
              let page = doc.page(at: 1),
              let pageDict = page.dictionary else {
            return nil
        }

        let media = page.getBoxRect(.mediaBox)
        guard media.width > 10.0, media.height > 10.0 else { return nil }

        var vpArrayRef: CGPDFArrayRef?
        guard CGPDFDictionaryGetArray(pageDict, "VP", &vpArrayRef),
              let vpArray = vpArrayRef else {
            return nil
        }

        let vpCount = CGPDFArrayGetCount(vpArray)
        for idx in 0..<vpCount {
            if let reg = solveViewportDictionary(in: vpArray, index: idx, mediaBox: media) {
                return reg
            }
        }
        return nil
    }

    /// Rasterizes a georeferenced FAA `d-TPP` PDF page to a cached high-DPI PNG and returns its `[NW, NE, SE, SW]` corners.
    public static func renderGeoPdfOverlayPng(
        pdfData: Data,
        cacheKey: String,
        maxEdgePixels: Int = 1600,
        fallbackRegistration: GeoPdfRegistration? = nil,
        invertColors: Bool = false
    ) -> (pngUrl: URL, registration: GeoPdfRegistration)? {
        guard let reg = extractGeoRegistration(from: pdfData) ?? fallbackRegistration,
              let provider = CGDataProvider(data: pdfData as CFData),
              let doc = CGPDFDocument(provider),
              let page = doc.page(at: 1) else {
            return nil
        }

        let safeName = sanitizeFileName(cacheKey)
        let modeTag = invertColors ? "inv" : "day"
        let outUrl = plateCacheDirectory().appendingPathComponent("geopdf_vp_\(safeName)_\(maxEdgePixels)_\(modeTag).png")
        if FileManager.default.fileExists(atPath: outUrl.path) {
            return (outUrl, reg)
        }

        let media = page.getBoxRect(.mediaBox)
        let cropRect = reg.viewportBoxPdf ?? media
        let scale = Double(maxEdgePixels) / Double(max(cropRect.width, cropRect.height))
        let pxW = max(Int((Double(cropRect.width) * scale).rounded()), 64)
        let pxH = max(Int((Double(cropRect.height) * scale).rounded()), 64)

        let colorSpace = CGColorSpaceCreateDeviceRGB()
        guard let ctx = CGContext(
            data: nil,
            width: pxW,
            height: pxH,
            bitsPerComponent: 8,
            bytesPerRow: pxW * 4,
            space: colorSpace,
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else {
            return nil
        }

        let fullRect = CGRect(x: 0, y: 0, width: pxW, height: pxH)
        ctx.setFillColor(red: 0.99, green: 0.99, blue: 0.97, alpha: 1.0)
        ctx.fill(fullRect)
        ctx.interpolationQuality = .high
        ctx.saveGState()
        ctx.scaleBy(x: CGFloat(scale), y: CGFloat(scale))
        ctx.translateBy(x: -cropRect.minX, y: -cropRect.minY)
        ctx.drawPDFPage(page)
        ctx.restoreGState()

        if invertColors {
            ctx.saveGState()
            ctx.setBlendMode(.difference)
            ctx.setFillColor(red: 0.94, green: 0.95, blue: 0.96, alpha: 1.0)
            ctx.fill(fullRect)
            ctx.restoreGState()
            ctx.setStrokeColor(red: 0.0, green: 1.0, blue: 1.0, alpha: 0.65)
        } else {
            ctx.setStrokeColor(red: 0.14, green: 0.56, blue: 0.78, alpha: 0.85)
        }
        ctx.setLineWidth(3.0)
        ctx.stroke(CGRect(x: 1.5, y: 1.5, width: CGFloat(pxW) - 3.0, height: CGFloat(pxH) - 3.0))

        guard let cgImg = ctx.makeImage(), writePng(cgImage: cgImg, to: outUrl) else {
            return nil
        }
        return (outUrl, reg)
    }

    /// Synthesizes a north-up georeferenced approach diagram (`14 NM x 14 NM`) for airports without a bundled GeoPDF.
    public static func renderProceduralApproachOverlayPng(
        procedure: CifpProcedureSummary,
        airportLatDeg: Double,
        airportLonDeg: Double,
        legs: [FlightPlanWaypoint],
        cacheKey: String,
        invertColors: Bool = false
    ) -> (pngUrl: URL, registration: GeoPdfRegistration)? {
        let halfSpanNm = 7.5
        let dLat = halfSpanNm / 60.0
        let cosLat = max(cos(airportLatDeg * .pi / 180.0), 0.15)
        let dLon = halfSpanNm / (60.0 * cosLat)

        let minLat = airportLatDeg - dLat
        let maxLat = airportLatDeg + dLat
        let minLon = airportLonDeg - dLon
        let maxLon = airportLonDeg + dLon
        let corners: [Double] = [
            minLon, maxLat,
            maxLon, maxLat,
            maxLon, minLat,
            minLon, minLat,
        ]
        let reg = GeoPdfRegistration(
            cornersLonLat: corners,
            centerLatitudeDeg: airportLatDeg,
            centerLongitudeDeg: airportLonDeg
        )

        let safeName = sanitizeFileName(cacheKey)
        let modeTag = invertColors ? "inv" : "day"
        let outUrl = plateCacheDirectory().appendingPathComponent("synth_\(safeName)_\(modeTag).png")
        if FileManager.default.fileExists(atPath: outUrl.path) {
            return (outUrl, reg)
        }

        let sizePx = 1200
        let colorSpace = CGColorSpaceCreateDeviceRGB()
        guard let ctx = CGContext(
            data: nil,
            width: sizePx,
            height: sizePx,
            bitsPerComponent: 8,
            bytesPerRow: sizePx * 4,
            space: colorSpace,
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else {
            return nil
        }

        drawProceduralApproachCanvas(
            in: ctx,
            size: CGFloat(sizePx),
            procedure: procedure,
            airportLatDeg: airportLatDeg,
            airportLonDeg: airportLonDeg,
            minLat: minLat,
            maxLat: maxLat,
            minLon: minLon,
            maxLon: maxLon,
            legs: legs
        )

        if invertColors {
            ctx.saveGState()
            ctx.setBlendMode(.difference)
            ctx.setFillColor(red: 0.94, green: 0.95, blue: 0.96, alpha: 1.0)
            ctx.fill(CGRect(x: 0, y: 0, width: sizePx, height: sizePx))
            ctx.restoreGState()
        }

        guard let cgImg = ctx.makeImage(), writePng(cgImage: cgImg, to: outUrl) else {
            return nil
        }
        return (outUrl, reg)
    }

    /// Synthesizes a single-page PDF document for `PDFKit` full-screen inspection when no bundled FAA PDF is present.
    public static func renderProceduralPdfData(
        procedure: CifpProcedureSummary,
        airportLatDeg: Double,
        airportLonDeg: Double,
        legs: [FlightPlanWaypoint]
    ) -> Data? {
        let halfSpanNm = 7.5
        let dLat = halfSpanNm / 60.0
        let cosLat = max(cos(airportLatDeg * .pi / 180.0), 0.15)
        let dLon = halfSpanNm / (60.0 * cosLat)

        let mutableData = NSMutableData()
        var mediaBox = CGRect(x: 0, y: 0, width: 612, height: 612)
        guard let consumer = CGDataConsumer(data: mutableData as CFMutableData),
              let pdfCtx = CGContext(consumer: consumer, mediaBox: &mediaBox, nil) else {
            return nil
        }
        pdfCtx.beginPDFPage(nil)
        drawProceduralApproachCanvas(
            in: pdfCtx,
            size: 612.0,
            procedure: procedure,
            airportLatDeg: airportLatDeg,
            airportLonDeg: airportLonDeg,
            minLat: airportLatDeg - dLat,
            maxLat: airportLatDeg + dLat,
            minLon: airportLonDeg - dLon,
            maxLon: airportLonDeg + dLon,
            legs: legs
        )
        pdfCtx.endPDFPage()
        pdfCtx.closePDF()
        return mutableData as Data
    }

    // MARK: - Private GeoPDF `/VP` Affine Solver & Drawing Helpers

    private static func solveViewportDictionary(
        in vpArray: CGPDFArrayRef,
        index: Int,
        mediaBox: CGRect
    ) -> GeoPdfRegistration? {
        var vpDictRef: CGPDFDictionaryRef?
        var measDictRef: CGPDFDictionaryRef?
        guard CGPDFArrayGetDictionary(vpArray, index, &vpDictRef),
              let vpDict = vpDictRef,
              CGPDFDictionaryGetDictionary(vpDict, "Measure", &measDictRef),
              let measDict = measDictRef else {
            return nil
        }

        let bbox = readRealArray(from: vpDict, key: "BBox")
        let lpts = readRealArray(from: measDict, key: "LPTS")
        let gpts = readRealArray(from: measDict, key: "GPTS")
        let pairCount = min(lpts.count / 2, gpts.count / 2)
        guard bbox.count == 4, pairCount >= 3 else { return nil }

        let xMin = min(bbox[0], bbox[2])
        let xMax = max(bbox[0], bbox[2])
        let yMin = min(bbox[1], bbox[3])
        let yMax = max(bbox[1], bbox[3])
        guard (xMax - xMin) > 10.0, (yMax - yMin) > 10.0 else { return nil }

        var samples: [(x: Double, y: Double, lon: Double, lat: Double)] = []
        for k in 0..<pairCount {
            let u = lpts[k * 2]
            let v = lpts[k * 2 + 1]
            let lat = gpts[k * 2]
            let lon = gpts[k * 2 + 1]
            guard abs(lat) <= 90.0, abs(lon) <= 180.0 else { return nil }
            let px = xMin + u * (xMax - xMin)
            let py = yMin + v * (yMax - yMin)
            samples.append((px, py, lon, lat))
        }

        guard let lonCoeffs = fitAffine2D(samples: samples.map { ($0.x, $0.y, $0.lon) }),
              let latCoeffs = fitAffine2D(samples: samples.map { ($0.x, $0.y, $0.lat) }) else {
            return nil
        }

        let eval = { (x: Double, y: Double) -> (lon: Double, lat: Double) in
            let lon = lonCoeffs.0 + lonCoeffs.1 * x + lonCoeffs.2 * y
            let lat = latCoeffs.0 + latCoeffs.1 * x + latCoeffs.2 * y
            return (lon, lat)
        }

        let vpRect = CGRect(x: xMin, y: yMin, width: xMax - xMin, height: yMax - yMin)
        let nw = eval(xMin, yMax)
        let ne = eval(xMax, yMax)
        let se = eval(xMax, yMin)
        let sw = eval(xMin, yMin)
        let ctr = eval(0.5 * (xMin + xMax), 0.5 * (yMin + yMax))

        let corners = [nw.lon, nw.lat, ne.lon, ne.lat, se.lon, se.lat, sw.lon, sw.lat]
        guard corners.allSatisfy({ $0.isFinite }) else { return nil }
        return GeoPdfRegistration(
            cornersLonLat: corners,
            centerLatitudeDeg: ctr.lat,
            centerLongitudeDeg: ctr.lon,
            viewportBoxPdf: vpRect
        )
    }

    private static func fitAffine2D(samples: [(Double, Double, Double)]) -> (Double, Double, Double)? {
        var ata = Array(repeating: Array(repeating: 0.0, count: 3), count: 3)
        var atb = Array(repeating: 0.0, count: 3)
        for (x, y, z) in samples {
            let row = [1.0, x, y]
            for r in 0..<3 {
                atb[r] += row[r] * z
                for c in 0..<3 {
                    ata[r][c] += row[r] * row[c]
                }
            }
        }
        // Solve 3x3 linear system with partial pivoting.
        for col in 0..<3 {
            var pivot = col
            for row in (col + 1)..<3 where abs(ata[row][col]) > abs(ata[pivot][col]) {
                pivot = row
            }
            if abs(ata[pivot][col]) < 1e-12 {
                return nil
            }
            if pivot != col {
                ata.swapAt(col, pivot)
                atb.swapAt(col, pivot)
            }
            let diag = ata[col][col]
            for c in col..<3 { ata[col][c] /= diag }
            atb[col] /= diag
            for row in 0..<3 where row != col {
                let factor = ata[row][col]
                for c in col..<3 {
                    ata[row][c] -= factor * ata[col][c]
                }
                atb[row] -= factor * atb[col]
            }
        }
        return (atb[0], atb[1], atb[2])
    }

    private static func readRealArray(from dict: CGPDFDictionaryRef, key: String) -> [Double] {
        var arrRef: CGPDFArrayRef?
        guard CGPDFDictionaryGetArray(dict, key, &arrRef), let arr = arrRef else {
            return []
        }
        let count = CGPDFArrayGetCount(arr)
        var out: [Double] = []
        out.reserveCapacity(count)
        for idx in 0..<count {
            var val: CGPDFReal = 0
            if CGPDFArrayGetNumber(arr, idx, &val) {
                out.append(Double(val))
            }
        }
        return out
    }

    private static func drawProceduralApproachCanvas(
        in ctx: CGContext,
        size: CGFloat,
        procedure: CifpProcedureSummary,
        airportLatDeg: Double,
        airportLonDeg: Double,
        minLat: Double,
        maxLat: Double,
        minLon: Double,
        maxLon: Double,
        legs: [FlightPlanWaypoint]
    ) {
        let project = { (lat: Double, lon: Double) -> CGPoint in
            let nx = (lon - minLon) / max(maxLon - minLon, 1e-6)
            let ny = (lat - minLat) / max(maxLat - minLat, 1e-6)
            return CGPoint(x: CGFloat(nx) * size, y: CGFloat(ny) * size)
        }

        ctx.setFillColor(red: 0.96, green: 0.97, blue: 0.95, alpha: 0.94)
        ctx.fill(CGRect(x: 0, y: 0, width: size, height: size))

        ctx.setStrokeColor(red: 0.14, green: 0.22, blue: 0.30, alpha: 0.85)
        ctx.setLineWidth(size * 0.004)
        ctx.stroke(CGRect(x: size * 0.012, y: size * 0.012, width: size * 0.976, height: size * 0.976))

        let aptPt = project(airportLatDeg, airportLonDeg)
        let pxPerNm = size / 15.0
        ctx.setStrokeColor(red: 0.25, green: 0.38, blue: 0.48, alpha: 0.28)
        ctx.setLineWidth(max(size * 0.0018, 1.0))
        for ringNm in [5.0, 10.0] {
            let r = CGFloat(ringNm) * pxPerNm
            ctx.strokeEllipse(in: CGRect(x: aptPt.x - r, y: aptPt.y - r, width: r * 2, height: r * 2))
        }

        if legs.count >= 2 {
            ctx.setStrokeColor(red: 0.08, green: 0.12, blue: 0.18, alpha: 0.95)
            ctx.setLineWidth(max(size * 0.0048, 2.2))
            for idx in 1..<legs.count {
                let p0 = project(legs[idx - 1].latitudeDeg, legs[idx - 1].longitudeDeg)
                let p1 = project(legs[idx].latitudeDeg, legs[idx].longitudeDeg)
                ctx.move(to: p0)
                ctx.addLine(to: p1)
                ctx.strokePath()
            }
        }

        for wp in legs {
            let pt = project(wp.latitudeDeg, wp.longitudeDeg)
            let tri = max(size * 0.011, 5.5)
            ctx.setFillColor(red: 0.10, green: 0.14, blue: 0.22, alpha: 1.0)
            ctx.beginPath()
            ctx.move(to: CGPoint(x: pt.x, y: pt.y + tri))
            ctx.addLine(to: CGPoint(x: pt.x + tri * 0.88, y: pt.y - tri * 0.75))
            ctx.addLine(to: CGPoint(x: pt.x - tri * 0.88, y: pt.y - tri * 0.75))
            ctx.closePath()
            ctx.fillPath()

            let label = wp.altitudeConstraintTag.isEmpty
                ? wp.ident
                : "\(wp.ident) (\(wp.altitudeConstraintTag))"
            drawCenteredLabel(
                in: ctx,
                text: label,
                at: CGPoint(x: pt.x, y: pt.y + tri + size * 0.014),
                fontSize: max(size * 0.018, 10.0),
                bold: true
            )
        }

        ctx.setFillColor(red: 0.05, green: 0.35, blue: 0.65, alpha: 1.0)
        let aptR = max(size * 0.010, 5.0)
        ctx.fillEllipse(in: CGRect(x: aptPt.x - aptR, y: aptPt.y - aptR, width: aptR * 2, height: aptR * 2))

        drawCenteredLabel(
            in: ctx,
            text: "\(procedure.airportIdent) · \(procedure.humanTitle)",
            at: CGPoint(x: size * 0.5, y: size * 0.945),
            fontSize: max(size * 0.025, 13.0),
            bold: true
        )
        drawCenteredLabel(
            in: ctx,
            text: "PROCEDURAL PLANVIEW · NORTH UP · 5 / 10 NM RINGS · NOT FOR REAL FLIGHT",
            at: CGPoint(x: size * 0.5, y: size * 0.035),
            fontSize: max(size * 0.014, 8.5),
            bold: false
        )
    }

    private static func drawCenteredLabel(
        in ctx: CGContext,
        text: String,
        at center: CGPoint,
        fontSize: CGFloat,
        bold: Bool
    ) {
        let fontName = bold ? "Menlo-Bold" : "Menlo"
        let ctFont = CTFontCreateWithName(fontName as CFString, fontSize, nil)
        let colorSpace = CGColorSpaceCreateDeviceRGB()
        let fg = CGColor(colorSpace: colorSpace, components: [0.06, 0.10, 0.16, 1.0])
        var attrs: [NSAttributedString.Key: Any] = [
            NSAttributedString.Key(kCTFontAttributeName as String): ctFont,
        ]
        if let fg {
            attrs[NSAttributedString.Key(kCTForegroundColorAttributeName as String)] = fg
        }
        let attrStr = NSAttributedString(string: text, attributes: attrs)
        let line = CTLineCreateWithAttributedString(attrStr)
        let bounds = CTLineGetBoundsWithOptions(line, .useOpticalBounds)
        let padX = fontSize * 0.32
        let padY = fontSize * 0.18
        let bgRect = CGRect(
            x: center.x - bounds.width * 0.5 - padX,
            y: center.y - bounds.height * 0.5 - padY,
            width: bounds.width + padX * 2.0,
            height: bounds.height + padY * 2.0
        )
        ctx.setFillColor(red: 1.0, green: 1.0, blue: 0.98, alpha: 0.88)
        ctx.fill(bgRect)
        ctx.textPosition = CGPoint(
            x: center.x - bounds.width * 0.5,
            y: center.y - bounds.height * 0.35
        )
        CTLineDraw(line, ctx)
    }

    private static func writePng(cgImage: CGImage, to url: URL) -> Bool {
        guard let dest = CGImageDestinationCreateWithURL(
            url as CFURL,
            UTType.png.identifier as CFString,
            1,
            nil
        ) else {
            return false
        }
        CGImageDestinationAddImage(dest, cgImage, nil)
        return CGImageDestinationFinalize(dest)
    }

    private static func plateCacheDirectory() -> URL {
        let dir = FileManager.default.temporaryDirectory
            .appendingPathComponent("FlightDeckProcedurePlates", isDirectory: true)
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }

    private static func sanitizeFileName(_ raw: String) -> String {
        let allowed = CharacterSet.alphanumerics.union(CharacterSet(charactersIn: "-_"))
        return String(raw.unicodeScalars.map { allowed.contains($0) ? Character($0) : "_" })
    }
}

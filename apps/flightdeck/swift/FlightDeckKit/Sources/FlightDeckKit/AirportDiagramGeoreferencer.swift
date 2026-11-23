import CoreGraphics
import Foundation
import PDFKit

/// Surveyed runway threshold endpoint used to solve 2D similarity georeferencing for FAA `d-TPP` Airport Diagrams (`APD`).
public struct SurveyedRunwayEnd: Equatable, Sendable {
    /// Normalized runway designator without leading `"RW"` (`"4L"`, `"04L"`, `"22R"`, `"11"`, `"29"`).
    public let designator: String
    /// Surveyed pavement-end longitude (`degrees`, `WGS-84`).
    public let longitudeDeg: Double
    /// Surveyed pavement-end latitude (`degrees`, `WGS-84`).
    public let latitudeDeg: Double
    /// Outbound runway label anchor (~`105 m` off the pavement end where FAA prints the runway number on `APD` charts).
    public let labelLongitudeDeg: Double
    /// Outbound runway label anchor latitude (`degrees`, `WGS-84`).
    public let labelLatitudeDeg: Double
    /// Unit East component pointing outbound from the threshold along the extended runway centerline (`0` if unknown).
    public let outboundEast: Double
    /// Unit North component pointing outbound from the threshold along the extended runway centerline (`0` if unknown).
    public let outboundNorth: Double

    /// Creates a surveyed runway endpoint record.
    public init(
        designator: String,
        longitudeDeg: Double,
        latitudeDeg: Double,
        labelLongitudeDeg: Double,
        labelLatitudeDeg: Double,
        outboundEast: Double = 0.0,
        outboundNorth: Double = 0.0
    ) {
        self.designator = designator
        self.longitudeDeg = longitudeDeg
        self.latitudeDeg = latitudeDeg
        self.labelLongitudeDeg = labelLongitudeDeg
        self.labelLatitudeDeg = labelLatitudeDeg
        self.outboundEast = outboundEast
        self.outboundNorth = outboundNorth
    }
}

/// Solves verified `WGS-84` corner coordinates (`[NW, NE, SE, SW]`) for official FAA `d-TPP` Airport Diagrams (`APD`)
/// using either printed neatline graticule ticks (`DD°MM.M'N/W`) or surveyed runway threshold designator RANSAC.
public enum AirportDiagramGeoreferencer {
    /// Solves `GeoPdfRegistration` for page 1 of `pdfData`, returning `nil` if the diagram is marked `"NOT TO SCALE"`
    /// or lacks sufficient geometric control points (`< 55 m` RMS residual required).
    public static func solve(
        pdfData: Data,
        referenceLatDeg: Double,
        referenceLonDeg: Double,
        runwayEnds: [SurveyedRunwayEnd]
    ) -> GeoPdfRegistration? {
        if let vp = GeoPdfPlateRenderer.extractGeoRegistration(from: pdfData) {
            return vp
        }
        guard let doc = PDFDocument(data: pdfData),
              let page = doc.page(at: 0) else {
            return nil
        }

        let media = page.bounds(for: .mediaBox)
        guard media.width > 100.0, media.height > 100.0 else { return nil }

        let pageText = (page.string ?? "").uppercased()
        if pageText.contains("NOT TO SCALE") {
            return nil
        }

        if let tickReg = solveFromGraticuleTicks(doc: doc, page: page, media: media) {
            return tickReg
        }

        if runwayEnds.count >= 3 {
            return solveFromRunwayThresholds(
                doc: doc,
                page: page,
                media: media,
                refLatDeg: referenceLatDeg,
                refLonDeg: referenceLonDeg,
                runwayEnds: runwayEnds
            )
        }
        return nil
    }

    // MARK: - Method 1: Printed Graticule Tick Solver (`DD°MM(.M)'N/S/E/W`)

    private static func solveFromGraticuleTicks(
        doc: PDFDocument,
        page: PDFPage,
        media: CGRect
    ) -> GeoPdfRegistration? {
        guard let rawText = page.string,
              let regex = try? NSRegularExpression(pattern: #"(\d{1,3})°(\d{1,2}(?:\.\d+)?)[\x27\u2032]?\s*([NSEW])"#) else {
            return nil
        }

        let ns = rawText as NSString
        let matches = regex.matches(in: rawText, range: NSRange(location: 0, length: ns.length))
        guard matches.count >= 4 else { return nil }

        var latTicks: [(y: Double, lat: Double)] = []
        var lonTicks: [(x: Double, lon: Double)] = []

        for m in matches {
            let full = ns.substring(with: m.range)
            let degStr = ns.substring(with: m.range(at: 1))
            let minStr = ns.substring(with: m.range(at: 2))
            let hemi = ns.substring(with: m.range(at: 3))
            guard let deg = Double(degStr), let mins = Double(minStr) else { continue }
            let signed = ((hemi == "S" || hemi == "W") ? -1.0 : 1.0) * (deg + mins / 60.0)

            for sel in doc.findString(full, withOptions: .literal) {
                let b = sel.bounds(for: page)
                guard b.width > 1.0, b.height > 1.0 else { continue }
                if hemi == "N" || hemi == "S" {
                    latTicks.append((Double(b.midY), signed))
                } else {
                    lonTicks.append((Double(b.midX), signed))
                }
            }
        }

        guard let latLine = fitLinear1D(latTicks),
              let lonLine = fitLinear1D(lonTicks),
              latLine.slope > 1e-6,
              lonLine.slope > 1e-6 else {
            return nil
        }

        // Check isotropic scale consistency (`meters per PDF point` in X vs Y within 12%).
        let midLat = latLine.intercept + latLine.slope * Double(media.midY)
        let cosLat = max(cos(midLat * .pi / 180.0), 0.15)
        let mPerPtY = latLine.slope * 111_132.0
        let mPerPtX = lonLine.slope * 111_320.0 * cosLat
        let ratio = mPerPtX / max(mPerPtY, 1e-6)
        guard ratio > 0.85, ratio < 1.18, mPerPtY > 1.5, mPerPtY < 45.0 else {
            return nil
        }

        let lonAt = { (x: Double) -> Double in lonLine.intercept + lonLine.slope * x }
        let latAt = { (y: Double) -> Double in latLine.intercept + latLine.slope * y }

        let x0 = Double(media.minX)
        let x1 = Double(media.maxX)
        let y0 = Double(media.minY)
        let y1 = Double(media.maxY)
        let corners = [
            lonAt(x0), latAt(y1),
            lonAt(x1), latAt(y1),
            lonAt(x1), latAt(y0),
            lonAt(x0), latAt(y0),
        ]
        return GeoPdfRegistration(
            cornersLonLat: corners,
            centerLatitudeDeg: latAt(0.5 * (y0 + y1)),
            centerLongitudeDeg: lonAt(0.5 * (x0 + x1)),
            viewportBoxPdf: media
        )
    }

    private static func fitLinear1D(_ samples: [(Double, Double)]) -> (intercept: Double, slope: Double)? {
        // Deduplicate nearly identical coordinate values to require >= 2 distinct parallels/meridians.
        var byValue: [Int: [Double]] = [:]
        for (pos, deg) in samples {
            let bucket = Int((deg * 6_000.0).rounded())
            byValue[bucket, default: []].append(pos)
        }
        guard byValue.count >= 2 else { return nil }

        var pts: [(x: Double, y: Double)] = []
        for (bucket, positions) in byValue {
            let sorted = positions.sorted()
            let med = sorted[sorted.count / 2]
            pts.append((med, Double(bucket) / 6_000.0))
        }

        let n = Double(pts.count)
        let meanX = pts.reduce(0.0) { $0 + $1.x } / n
        let meanY = pts.reduce(0.0) { $0 + $1.y } / n
        var num = 0.0
        var den = 0.0
        for p in pts {
            num += (p.x - meanX) * (p.y - meanY)
            den += (p.x - meanX) * (p.x - meanX)
        }
        guard den > 200.0 else { return nil }
        let slope = num / den
        let intercept = meanY - slope * meanX
        return (intercept, slope)
    }

    // MARK: - Method 2: Surveyed Runway Designator 2D Similarity RANSAC

    private struct LocalTarget {
        let designator: String
        let eastM: Double
        let northM: Double
        let threshEastM: Double
        let threshNorthM: Double
        let outEast: Double
        let outNorth: Double

        var hasRay: Bool { hypot(outEast, outNorth) > 0.5 }
    }

    private static func solveFromRunwayThresholds(
        doc: PDFDocument,
        page: PDFPage,
        media: CGRect,
        refLatDeg: Double,
        refLonDeg: Double,
        runwayEnds: [SurveyedRunwayEnd]
    ) -> GeoPdfRegistration? {
        let cosLat = max(cos(refLatDeg * .pi / 180.0), 0.15)
        let mPerDegLat = 111_132.0
        let mPerDegLon = 111_320.0 * cosLat

        // Collect PDF bounding-box centers for each runway designator (`"4L"` / `"04L"`, `"22R"`, `"11"`, `"29"`).
        // Ignore the header notes box at the top 10% (`y > 0.90 * height`) where runway lighting notes live.
        let diagramRect = CGRect(
            x: media.minX + 12.0,
            y: media.minY + 18.0,
            width: media.width - 24.0,
            height: media.height * 0.86
        )

        var candidatesByEnd: [(target: LocalTarget, pdfPts: [CGPoint])] = []
        for end in runwayEnds {
            let stripped = stripLeadingZero(end.designator)
            let searchTokens = stripped == end.designator ? [end.designator] : [stripped, end.designator]
            var pts: [CGPoint] = []
            for tok in searchTokens {
                for sel in doc.findString(tok, withOptions: .literal) {
                    let matched = (sel.string ?? "").trimmingCharacters(in: .whitespacesAndNewlines).uppercased()
                    guard matched == tok else { continue }
                    // Verify token isn't embedded inside a longer number like `"11000"` or `"290"`:
                    let b = sel.bounds(for: page)
                    guard diagramRect.contains(CGPoint(x: b.midX, y: b.midY)),
                          b.width >= 3.5, b.width <= 26.0,
                          b.height >= 3.5, b.height <= 22.0 else {
                        continue
                    }
                    let pt = CGPoint(x: b.midX, y: b.midY)
                    if !pts.contains(where: { hypot($0.x - pt.x, $0.y - pt.y) < 6.0 }) {
                        pts.append(pt)
                    }
                }
            }
            if !pts.isEmpty {
                let east = (end.labelLongitudeDeg - refLonDeg) * mPerDegLon
                let north = (end.labelLatitudeDeg - refLatDeg) * mPerDegLat
                let tEast = (end.longitudeDeg - refLonDeg) * mPerDegLon
                let tNorth = (end.latitudeDeg - refLatDeg) * mPerDegLat
                candidatesByEnd.append((
                    LocalTarget(
                        designator: end.designator, eastM: east, northM: north,
                        threshEastM: tEast, threshNorthM: tNorth,
                        outEast: end.outboundEast, outNorth: end.outboundNorth
                    ),
                    pts
                ))
            }
        }

        guard candidatesByEnd.count >= 3 else { return nil }

        // Pass 1 (preferred): True-North-Up isotropic scale + translation using exact cross-track centerline rays
        // (invariant to `ALSF-2` / `MALSR` approach light ladders shifting runway numbers outbound along centerline).
        // Pass 2 (fallback): general 2D similarity (`>= 4` inliers & `< 28 m` RMS).
        let solvedSim = solveRansac(candidatesByEnd: candidatesByEnd, northUpOnly: true, maxRmsM: 55.0, minInliers: 3)
            ?? solveRansac(candidatesByEnd: candidatesByEnd, northUpOnly: false, maxRmsM: 28.0, minInliers: 4)
        guard let refined = solvedSim else {
            return nil
        }

        let toLonLat = { (x: Double, y: Double) -> (lon: Double, lat: Double) in
            let en = refined.eval(x, y)
            return (refLonDeg + en.east / mPerDegLon, refLatDeg + en.north / mPerDegLat)
        }

        let (x0, x1, y0, y1) = (Double(media.minX), Double(media.maxX), Double(media.minY), Double(media.maxY))
        let (nw, ne, se, sw) = (toLonLat(x0, y1), toLonLat(x1, y1), toLonLat(x1, y0), toLonLat(x0, y0))
        let ctr = toLonLat(0.5 * (x0 + x1), 0.5 * (y0 + y1))

        return GeoPdfRegistration(
            cornersLonLat: [nw.lon, nw.lat, ne.lon, ne.lat, se.lon, se.lat, sw.lon, sw.lat],
            centerLatitudeDeg: ctr.lat,
            centerLongitudeDeg: ctr.lon,
            viewportBoxPdf: media
        )
    }

    private static func candidateErrorM(sim: Similarity2D, pdfPt: CGPoint, target: LocalTarget, northUp: Bool) -> Double? {
        let pred = sim.eval(Double(pdfPt.x), Double(pdfPt.y))
        if northUp && target.hasRay {
            let de = pred.east - target.threshEastM
            let dn = pred.north - target.threshNorthM
            let cross = abs(-target.outNorth * de + target.outEast * dn)
            let along = target.outEast * de + target.outNorth * dn
            guard cross <= 32.0, along >= -35.0, along <= 1_050.0 else { return nil }
            return hypot(cross, 0.03 * (along - 220.0))
        }
        let err = hypot(pred.east - target.eastM, pred.north - target.northM)
        return err <= 65.0 ? err : nil
    }

    private static func hypothesesForPair(
        pA: CGPoint, tA: LocalTarget, pB: CGPoint, tB: LocalTarget, baseScale: Double, northUpOnly: Bool
    ) -> [Similarity2D] {
        if !northUpOnly {
            return fitSimilarity2D(pairs: [(pA, tA), (pB, tB)]).map { [$0] } ?? []
        }
        var out: [Similarity2D] = []
        if let direct = fitNorthUp2D(pairs: [(pA, tA), (pB, tB)]) { out.append(direct) }
        if tA.hasRay && tB.hasRay {
            let det = (-tA.outNorth) * tB.outEast - tA.outEast * (-tB.outNorth)
            if abs(det) > 0.22 {
                let (xA, yA, xB, yB) = (Double(pA.x), Double(pA.y), Double(pB.x), Double(pB.y))
                for mult in [0.92, 1.04, 1.18, 1.34, 1.50] {
                    let s = baseScale * mult
                    guard s >= 2.2, s <= 24.0 else { continue }
                    let r1 = (-tA.outNorth * tA.threshEastM + tA.outEast * tA.threshNorthM) - s * (-tA.outNorth * xA + tA.outEast * yA)
                    let r2 = (-tB.outNorth * tB.threshEastM + tB.outEast * tB.threshNorthM) - s * (-tB.outNorth * xB + tB.outEast * yB)
                    let tx = (r1 * tB.outEast - tA.outEast * r2) / det
                    let ty = ((-tA.outNorth) * r2 - r1 * (-tB.outNorth)) / det
                    out.append(Similarity2D(a: s, b: 0.0, tx: tx, ty: ty))
                }
            }
        }
        return out
    }

    private static func solveRansac(
        candidatesByEnd: [(target: LocalTarget, pdfPts: [CGPoint])],
        northUpOnly: Bool,
        maxRmsM: Double,
        minInliers: Int
    ) -> Similarity2D? {
        var bestInliers: [(pdf: CGPoint, target: LocalTarget)] = []
        var bestRmsM = Double.infinity

        for i in 0..<candidatesByEnd.count {
            for j in (i + 1)..<candidatesByEnd.count {
                let (endA, endB) = (candidatesByEnd[i], candidatesByEnd[j])
                let worldDist = hypot(endB.target.eastM - endA.target.eastM, endB.target.northM - endA.target.northM)
                guard worldDist >= 550.0 else { continue }
                for pA in endA.pdfPts {
                    for pB in endB.pdfPts {
                        let pdfDist = Double(hypot(pB.x - pA.x, pB.y - pA.y))
                        guard pdfDist >= 45.0 else { continue }
                        let scale = worldDist / pdfDist
                        guard scale >= 1.8, scale <= 24.0 else { continue }
                        for sim in hypothesesForPair(pA: pA, tA: endA.target, pB: pB, tB: endB.target, baseScale: scale, northUpOnly: northUpOnly) {
                            var inliers: [(pdf: CGPoint, target: LocalTarget)] = []
                            var sqErrSum = 0.0
                            for item in candidatesByEnd {
                                var (bestPt, bestErr): (CGPoint?, Double) = (nil, .infinity)
                                for candPt in item.pdfPts {
                                    if let err = candidateErrorM(sim: sim, pdfPt: candPt, target: item.target, northUp: northUpOnly), err < bestErr {
                                        (bestPt, bestErr) = (candPt, err)
                                    }
                                }
                                if let pt = bestPt, bestErr.isFinite {
                                    inliers.append((pt, item.target))
                                    sqErrSum += bestErr * bestErr
                                }
                            }
                            if inliers.count >= minInliers {
                                let rms = sqrt(sqErrSum / Double(inliers.count))
                                if inliers.count > bestInliers.count || (inliers.count == bestInliers.count && rms < bestRmsM) {
                                    (bestInliers, bestRmsM) = (inliers, rms)
                                }
                            }
                        }
                    }
                }
            }
        }

        guard bestInliers.count >= minInliers else { return nil }
        let refinedOpt = northUpOnly ? fitNorthUp2D(pairs: bestInliers) : fitSimilarity2D(pairs: bestInliers)
        guard let refined = refinedOpt else { return nil }

        var refinedSqErr = 0.0
        for pair in bestInliers {
            guard let err = candidateErrorM(sim: refined, pdfPt: pair.pdf, target: pair.target, northUp: northUpOnly) else {
                refinedSqErr += 10_000.0
                continue
            }
            refinedSqErr += err * err
        }
        let refinedRms = sqrt(refinedSqErr / Double(bestInliers.count))
        return refinedRms <= maxRmsM ? refined : nil
    }

    private struct Similarity2D {
        let a: Double
        let b: Double
        let tx: Double
        let ty: Double

        func eval(_ x: Double, _ y: Double) -> (east: Double, north: Double) {
            (a * x - b * y + tx, b * x + a * y + ty)
        }
    }

    private static func fitNorthUp2D(pairs: [(pdf: CGPoint, target: LocalTarget)]) -> Similarity2D? {
        guard pairs.count >= 2 else { return nil }
        if pairs.allSatisfy({ $0.target.hasRay }) {
            var m = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
            var v = [0.0, 0.0, 0.0]
            let addEq = { (r0: Double, r1: Double, r2: Double, rhs: Double, w: Double) in
                let r = [r0, r1, r2]
                for i in 0..<3 {
                    v[i] += w * r[i] * rhs
                    for j in 0..<3 { m[i][j] += w * r[i] * r[j] }
                }
            }
            for p in pairs {
                let (x, y) = (Double(p.pdf.x), Double(p.pdf.y))
                let t = p.target
                addEq(-t.outNorth * x + t.outEast * y, -t.outNorth, t.outEast, -t.outNorth * t.threshEastM + t.outEast * t.threshNorthM, 1.0)
                addEq(t.outEast * x + t.outNorth * y, t.outEast, t.outNorth, t.outEast * t.threshEastM + t.outNorth * t.threshNorthM + 220.0, 0.002)
            }
            if let sol = solveLinear3x3(m, v), sol.0 > 2.0, sol.0 < 25.0 {
                return Similarity2D(a: sol.0, b: 0.0, tx: sol.1, ty: sol.2)
            }
        }
        let n = Double(pairs.count)
        let mx = pairs.reduce(0.0) { $0 + Double($1.pdf.x) } / n
        let my = pairs.reduce(0.0) { $0 + Double($1.pdf.y) } / n
        let me = pairs.reduce(0.0) { $0 + $1.target.eastM } / n
        let mn = pairs.reduce(0.0) { $0 + $1.target.northM } / n
        var (denom, numS) = (0.0, 0.0)
        for p in pairs {
            let (dx, dy) = (Double(p.pdf.x) - mx, Double(p.pdf.y) - my)
            denom += dx * dx + dy * dy
            numS += dx * (p.target.eastM - me) + dy * (p.target.northM - mn)
        }
        guard denom > 100.0 else { return nil }
        let s = numS / denom
        guard s > 2.0, s < 25.0 else { return nil }
        return Similarity2D(a: s, b: 0.0, tx: me - s * mx, ty: mn - s * my)
    }

    private static func solveLinear3x3(_ aIn: [[Double]], _ bIn: [Double]) -> (Double, Double, Double)? {
        var a = aIn
        var b = bIn
        for col in 0..<3 {
            var piv = col
            for r in (col + 1)..<3 where abs(a[r][col]) > abs(a[piv][col]) { piv = r }
            guard abs(a[piv][col]) > 1e-7 else { return nil }
            if piv != col { a.swapAt(col, piv); b.swapAt(col, piv) }
            for r in (col + 1)..<3 {
                let f = a[r][col] / a[col][col]
                for c in col..<3 { a[r][c] -= f * a[col][c] }
                b[r] -= f * b[col]
            }
        }
        let x2 = b[2] / a[2][2]
        let x1 = (b[1] - a[1][2] * x2) / a[1][1]
        let x0 = (b[0] - a[0][1] * x1 - a[0][2] * x2) / a[0][0]
        return (x0, x1, x2)
    }

    private static func fitSimilarity2D(pairs: [(pdf: CGPoint, target: LocalTarget)]) -> Similarity2D? {
        guard pairs.count >= 2 else { return nil }
        let n = Double(pairs.count)
        let mx = pairs.reduce(0.0) { $0 + Double($1.pdf.x) } / n
        let my = pairs.reduce(0.0) { $0 + Double($1.pdf.y) } / n
        let me = pairs.reduce(0.0) { $0 + $1.target.eastM } / n
        let mn = pairs.reduce(0.0) { $0 + $1.target.northM } / n
        var (denom, numA, numB) = (0.0, 0.0, 0.0)
        for p in pairs {
            let (dx, dy) = (Double(p.pdf.x) - mx, Double(p.pdf.y) - my)
            let (de, dn) = (p.target.eastM - me, p.target.northM - mn)
            denom += dx * dx + dy * dy
            numA += dx * de + dy * dn
            numB += dx * dn - dy * de
        }
        guard denom > 100.0 else { return nil }
        let (a, b) = (numA / denom, numB / denom)
        return Similarity2D(a: a, b: b, tx: me - (a * mx - b * my), ty: mn - (b * mx + a * my))
    }

    private static func stripLeadingZero(_ rwy: String) -> String {
        let upper = rwy.trimmingCharacters(in: .whitespacesAndNewlines).uppercased()
        if upper.count >= 2, upper.hasPrefix("0"), Array(upper)[1].isNumber {
            return String(upper.dropFirst())
        }
        return upper
    }
}

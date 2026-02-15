//
//  PointCloud.swift
//  DepthViz
//
//  Created by 장민석 on 8/28/24.
//  Copyright © 2024 Apple. All rights reserved.
//

import SceneKit
import simd

class PointCloud: NSObject {
    
    var n: Int = 0
    var pointCloud: [PointCloudVertex] = []
    
    override init() {
        super.init()
    }
    
    // 파일 확장자에 따라 적절한 포맷으로 로드
    func load(file: String, scale: Float = 4.0) {
        self.n = 0
        self.pointCloud = []

        let ext = (file as NSString).pathExtension.lowercased()

        switch ext {
        case "xyz":
            loadXYZ(file: file, scale: scale)
        case "las":
            loadLAS(file: file, scale: scale)
        case "ply":
            loadPLY(file: file, scale: scale)
        default:
            // 확장자 불명 — PLY ASCII로 시도
            loadPLY(file: file, scale: scale)
        }

        print("Point cloud data loaded: \(n) points (\(ext))")
    }

    // MARK: - PLY (ASCII + Binary)

    private func loadPLY(file: String, scale: Float) {
        guard let rawData = FileManager.default.contents(atPath: file) else {
            print("❌ PLY 파일 읽기 실패: \(file)")
            return
        }

        // 헤더를 ASCII로 파싱하여 포인트 수 및 포맷 판별
        var vertexCount = 0
        var isBinary = false
        var headerEndOffset = 0

        // end_header\n 위치 찾기
        let newline = UInt8(0x0A) // \n
        var lineStart = 0
        var headerLines: [String] = []

        rawData.withUnsafeBytes { (ptr: UnsafeRawBufferPointer) in
            guard let base = ptr.baseAddress else { return }
            let count = ptr.count
            for i in 0..<count {
                if base.load(fromByteOffset: i, as: UInt8.self) == newline {
                    let lineData = Data(bytes: base + lineStart, count: i - lineStart)
                    if let line = String(data: lineData, encoding: .ascii) {
                        headerLines.append(line.trimmingCharacters(in: CharacterSet(charactersIn: "\r")))
                    }
                    lineStart = i + 1
                    if headerLines.last == "end_header" {
                        headerEndOffset = i + 1
                        break
                    }
                }
            }
        }

        for line in headerLines {
            if line.hasPrefix("element vertex ") {
                let parts = line.components(separatedBy: " ")
                if parts.count >= 3, let count = Int(parts[2]) {
                    vertexCount = count
                }
            }
            if line.contains("binary_little_endian") || line.contains("binary_big_endian") {
                isBinary = true
            }
        }

        guard vertexCount > 0 else {
            print("❌ PLY 헤더에서 vertex count를 찾을 수 없음")
            return
        }

        n = vertexCount

        if isBinary {
            loadPLYBinary(rawData: rawData, headerEndOffset: headerEndOffset, vertexCount: vertexCount, scale: scale)
        } else {
            loadPLYAscii(rawData: rawData, headerEndOffset: headerEndOffset, vertexCount: vertexCount, scale: scale)
        }
    }

    private func loadPLYAscii(rawData: Data, headerEndOffset: Int, vertexCount: Int, scale: Float) {
        guard headerEndOffset < rawData.count else { return }
        let bodyData = rawData.subdata(in: headerEndOffset..<rawData.count)
        guard let bodyString = String(data: bodyData, encoding: .utf8) ?? String(data: bodyData, encoding: .ascii) else { return }

        let lines = bodyString.components(separatedBy: "\n")
        pointCloud.reserveCapacity(vertexCount)

        for line in lines {
            guard !line.isEmpty else { continue }
            let elements = line.components(separatedBy: " ")
            guard elements.count >= 6,
                  let x = Float(elements[0]),
                  let y = Float(elements[1]),
                  let z = Float(elements[2]),
                  let r = Float(elements[3]),
                  let g = Float(elements[4]),
                  let b = Float(elements[5]) else { continue }
            pointCloud.append(PointCloudVertex(
                x: x * scale, y: y * scale, z: z * scale,
                r: r / 255.0, g: g / 255.0, b: b / 255.0
            ))
            if pointCloud.count >= vertexCount { break }
        }
    }

    private func loadPLYBinary(rawData: Data, headerEndOffset: Int, vertexCount: Int, scale: Float) {
        // 포맷: float x, float y, float z, uchar r, uchar g, uchar b = 15 bytes per point
        let bytesPerPoint = 15
        let expectedSize = headerEndOffset + vertexCount * bytesPerPoint
        guard rawData.count >= expectedSize else {
            print("❌ PLY Binary 데이터 크기 부족: \(rawData.count) < \(expectedSize)")
            return
        }

        pointCloud.reserveCapacity(vertexCount)

        rawData.withUnsafeBytes { (ptr: UnsafeRawBufferPointer) in
            guard let base = ptr.baseAddress else { return }
            var offset = headerEndOffset
            for _ in 0..<vertexCount {
                var x: Float = 0, y: Float = 0, z: Float = 0
                memcpy(&x, base + offset, 4); offset += 4
                memcpy(&y, base + offset, 4); offset += 4
                memcpy(&z, base + offset, 4); offset += 4
                let r = (base + offset).load(as: UInt8.self); offset += 1
                let g = (base + offset).load(as: UInt8.self); offset += 1
                let b = (base + offset).load(as: UInt8.self); offset += 1
                pointCloud.append(PointCloudVertex(
                    x: x * scale, y: y * scale, z: z * scale,
                    r: Float(r) / 255.0, g: Float(g) / 255.0, b: Float(b) / 255.0
                ))
            }
        }
    }

    // MARK: - XYZ (텍스트: x y z r g b)

    private func loadXYZ(file: String, scale: Float) {
        guard let data = try? String(contentsOfFile: file, encoding: .utf8) else {
            print("❌ XYZ 파일 읽기 실패: \(file)")
            return
        }

        let lines = data.components(separatedBy: "\n")
        pointCloud.reserveCapacity(lines.count)

        for line in lines {
            guard !line.isEmpty else { continue }
            let elements = line.components(separatedBy: " ")
            guard elements.count >= 6,
                  let x = Float(elements[0]),
                  let y = Float(elements[1]),
                  let z = Float(elements[2]),
                  let r = Float(elements[3]),
                  let g = Float(elements[4]),
                  let b = Float(elements[5]) else { continue }
            pointCloud.append(PointCloudVertex(
                x: x * scale, y: y * scale, z: z * scale,
                r: r / 255.0, g: g / 255.0, b: b / 255.0
            ))
        }

        n = pointCloud.count
    }

    // MARK: - LAS (바이너리, Format 2 with RGB)

    private func loadLAS(file: String, scale: Float) {
        guard let rawData = FileManager.default.contents(atPath: file) else {
            print("❌ LAS 파일 읽기 실패: \(file)")
            return
        }
        guard rawData.count >= 227 else {
            print("❌ LAS 헤더 크기 부족: \(rawData.count)")
            return
        }

        rawData.withUnsafeBytes { (ptr: UnsafeRawBufferPointer) in
            guard let base = ptr.baseAddress else { return }

            // 시그니처 확인
            let sig = String(bytes: [base.load(fromByteOffset: 0, as: UInt8.self),
                                     base.load(fromByteOffset: 1, as: UInt8.self),
                                     base.load(fromByteOffset: 2, as: UInt8.self),
                                     base.load(fromByteOffset: 3, as: UInt8.self)], encoding: .ascii)
            guard sig == "LASF" else {
                print("❌ LAS 시그니처 불일치: \(sig ?? "nil")")
                return
            }

            // 헤더 파싱
            var offsetToPoints: UInt32 = 0
            memcpy(&offsetToPoints, base + 96, 4)

            var pointFormat: UInt8 = 0
            memcpy(&pointFormat, base + 104, 1)

            var pointRecordLength: UInt16 = 0
            memcpy(&pointRecordLength, base + 105, 2)

            var numPoints: UInt32 = 0
            memcpy(&numPoints, base + 107, 4)

            // 스케일 팩터 & 오프셋
            var xScale: Double = 0, yScale: Double = 0, zScale: Double = 0
            memcpy(&xScale, base + 131, 8)
            memcpy(&yScale, base + 139, 8)
            memcpy(&zScale, base + 147, 8)

            var xOffset: Double = 0, yOffset: Double = 0, zOffset: Double = 0
            memcpy(&xOffset, base + 155, 8)
            memcpy(&yOffset, base + 163, 8)
            memcpy(&zOffset, base + 171, 8)

            let recordLen = Int(pointRecordLength)
            let pointCount = Int(numPoints)
            let dataStart = Int(offsetToPoints)

            guard rawData.count >= dataStart + pointCount * recordLen else {
                print("❌ LAS 데이터 크기 부족")
                return
            }

            // Format 2: X(4) Y(4) Z(4) Intensity(2) Flags(1) Class(1) ScanAngle(1) UserData(1) SourceID(2) R(2) G(2) B(2) = 26 bytes
            let hasRGB = (pointFormat == 2 || pointFormat == 3) && recordLen >= 26

            pointCloud.reserveCapacity(pointCount)

            for i in 0..<pointCount {
                let recordOffset = dataStart + i * recordLen

                var xI: Int32 = 0, yI: Int32 = 0, zI: Int32 = 0
                memcpy(&xI, base + recordOffset, 4)
                memcpy(&yI, base + recordOffset + 4, 4)
                memcpy(&zI, base + recordOffset + 8, 4)

                let x = Float(Double(xI) * xScale + xOffset) * scale
                let y = Float(Double(yI) * yScale + yOffset) * scale
                let z = Float(Double(zI) * zScale + zOffset) * scale

                var r: Float = 0.8, g: Float = 0.8, b: Float = 0.8
                if hasRGB {
                    var r16: UInt16 = 0, g16: UInt16 = 0, b16: UInt16 = 0
                    memcpy(&r16, base + recordOffset + 20, 2)
                    memcpy(&g16, base + recordOffset + 22, 2)
                    memcpy(&b16, base + recordOffset + 24, 2)
                    r = Float(r16) / 65535.0
                    g = Float(g16) / 65535.0
                    b = Float(b16) / 65535.0
                }

                pointCloud.append(PointCloudVertex(x: x, y: y, z: z, r: r, g: g, b: b))
            }

            n = pointCloud.count
        }
    }
    
    /// Recenter all points relative to a reference transform (e.g. startCameraTransform).
    func recenter(using referenceTransform: simd_float4x4) {
        let inv = referenceTransform.inverse
        pointCloud = pointCloud.map { v in
            let pos = SIMD4<Float>(v.x, v.y, v.z, 1.0)
            let recentered = inv * pos
            return PointCloudVertex(x: recentered.x, y: recentered.y, z: recentered.z,
                                    r: v.r, g: v.g, b: v.b)
        }
    }

    /// Remove points farther than maxDistance from the origin.
    func clampDistance(maxDistance: Float) {
        pointCloud = pointCloud.filter { v in
            let d = sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            return d <= maxDistance
        }
    }

    /// Extended getNode supporting color mode and point size.
    func getNode(useColor: Bool = true, colorMode: PointColorMode, pointSize: CGFloat) -> SCNNode {
        let vertices: [PointCloudVertex]
        switch colorMode {
        case .rgb:
            vertices = useColor ? pointCloud : pointCloud.map {
                PointCloudVertex(x: $0.x, y: $0.y, z: $0.z, r: 1, g: 1, b: 1)
            }
        case .intensity:
            vertices = pointCloud.map { v in
                let gray = 0.299 * v.r + 0.587 * v.g + 0.114 * v.b
                return PointCloudVertex(x: v.x, y: v.y, z: v.z, r: gray, g: gray, b: gray)
            }
        case .height:
            let ys = pointCloud.map { $0.y }
            let minY = ys.min() ?? 0
            let maxY = ys.max() ?? 1
            let range = max(maxY - minY, 0.001)
            vertices = pointCloud.map { v in
                let t = (v.y - minY) / range
                return PointCloudVertex(x: v.x, y: v.y, z: v.z,
                                        r: t, g: 0.3, b: 1.0 - t)
            }
        case .mono:
            vertices = pointCloud.map {
                PointCloudVertex(x: $0.x, y: $0.y, z: $0.z, r: 0.8, g: 0.8, b: 0.8)
            }
        }
        let node = buildNode(points: vertices)
        if let geometry = node.geometry, let element = geometry.elements.first {
            element.pointSize = pointSize
            element.minimumPointScreenSpaceRadius = pointSize
            element.maximumPointScreenSpaceRadius = pointSize * 4
        }
        return node
    }

    // getNode에서 노드의 스케일 조정
    func getNode(useColor: Bool = true, scaleFactor: Float = 4.0) -> SCNNode {
        let vertices = pointCloud.map { (v: PointCloudVertex) -> PointCloudVertex in
            return useColor ? PointCloudVertex(x: v.x, y: v.y, z: v.z, r: v.r, g: v.g, b: v.b)
                : PointCloudVertex(x: v.x, y: v.y, z: v.z, r: 1.0, g: 1.0, b: 1.0)
        }
        
        let node = buildNode(points: vertices)
        
        // 포인트 클라우드 노드의 크기 조절
        node.scale = SCNVector3(x: scaleFactor, y: scaleFactor, z: scaleFactor)
        
        return node
    }
    
    private func buildNode(points: [PointCloudVertex]) -> SCNNode {
        let vertexData = Data(
            bytes: points,
            count: MemoryLayout<PointCloudVertex>.size * points.count
        )
        let positionSource = SCNGeometrySource(
            data: vertexData,
            semantic: .vertex,
            vectorCount: points.count,
            usesFloatComponents: true,
            componentsPerVector: 3,
            bytesPerComponent: MemoryLayout<Float>.size,
            dataOffset: 0,
            dataStride: MemoryLayout<PointCloudVertex>.size
        )
        let colorSource = SCNGeometrySource(
            data: vertexData,
            semantic: .color,
            vectorCount: points.count,
            usesFloatComponents: true,
            componentsPerVector: 3,
            bytesPerComponent: MemoryLayout<Float>.size,
            dataOffset: MemoryLayout<Float>.size * 3,
            dataStride: MemoryLayout<PointCloudVertex>.size
        )
        let elements = SCNGeometryElement(
            data: nil,
            primitiveType: .point,
            primitiveCount: points.count,
            bytesPerIndex: MemoryLayout<Int>.size
        )
        
        let pointsGeometry = SCNGeometry(sources: [positionSource, colorSource], elements: [elements])
        
        let material = SCNMaterial()
        material.isDoubleSided = true
        pointsGeometry.materials = [material]
        
        return SCNNode(geometry: pointsGeometry)
    }
}

struct PointCloudVertex {
    var x, y, z: Float
    var r, g, b: Float
}

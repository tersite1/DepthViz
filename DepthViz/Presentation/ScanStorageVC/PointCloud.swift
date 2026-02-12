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
    
    // 포인트 클라우드를 불러오는 함수에 scale 인자 추가
    func load(file: String, scale: Float = 4.0) {
        self.n = 0
        
        do {
            let data = try String(contentsOfFile: file, encoding: .ascii)
            var lines = data.components(separatedBy: "\n")
            
            // Read header
            while !lines.isEmpty {
                let line = lines.removeFirst()
                if line.hasPrefix("element vertex ") {
                    n = Int(line.components(separatedBy: " ")[2])!
                    continue
                }
                if line.hasPrefix("end_header") {
                    break
                }
            }
            
            // 포인트에 스케일을 적용하여 좌표를 조정
            pointCloud = lines.filter { !$0.isEmpty }.map { line -> PointCloudVertex in
                let elements = line.components(separatedBy: " ")
                return PointCloudVertex(
                    x: Float(elements[0])! * scale,  // 스케일 적용
                    y: Float(elements[1])! * scale,  // 스케일 적용
                    z: Float(elements[2])! * scale,  // 스케일 적용
                    r: Float(elements[3])! / 255.0,
                    g: Float(elements[4])! / 255.0,
                    b: Float(elements[5])! / 255.0
                )
            }
            
            print("Point cloud data loaded: \(n) points")
        } catch {
            print(error)
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

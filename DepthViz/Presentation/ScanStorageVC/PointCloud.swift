//
//  PointCloud.swift
//  DepthViz
//
//  Created by 장민석 on 8/28/24.
//  Copyright © 2024 Apple. All rights reserved.
//

import SceneKit

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

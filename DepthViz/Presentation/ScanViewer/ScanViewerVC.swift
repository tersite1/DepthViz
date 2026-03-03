//
//  ScanViewerVC.swift
//  DepthViz
//
//  저장된 PLY 파일을 포인트 클라우드로 렌더링 (ScanPreviewVC와 동일한 방식)
//

import UIKit
import SceneKit

class ScanViewerVC: UIViewController, SCNSceneRendererDelegate {

    static let identifier = "ScanViewerVC"

    var fileURL: URL?

    // SCNView를 코드로 생성 (스토리보드에 없으므로)
    private var sceneView: SCNView!
    private var pointCloudNode: SCNNode?

    // 로딩 인디케이터
    private let loadingIndicator: UIActivityIndicatorView = {
        let indicator = UIActivityIndicatorView(style: .large)
        indicator.color = .white
        indicator.hidesWhenStopped = true
        indicator.translatesAutoresizingMaskIntoConstraints = false
        return indicator
    }()

    // 높이(Y축) 클리핑 슬라이더
    private let heightClipSlider: UISlider = {
        let slider = UISlider()
        slider.minimumValue = 0
        slider.maximumValue = 1
        slider.value = 1
        slider.minimumTrackTintColor = UIColor(white: 0.4, alpha: 0.6)
        slider.maximumTrackTintColor = .white
        slider.thumbTintColor = .white
        slider.translatesAutoresizingMaskIntoConstraints = false
        return slider
    }()

    private let heightClipTopIcon: UIImageView = {
        let config = UIImage.SymbolConfiguration(pointSize: 14, weight: .medium)
        let iv = UIImageView(image: UIImage(systemName: "building.2.fill", withConfiguration: config))
        iv.tintColor = UIColor(white: 0.7, alpha: 1)
        iv.translatesAutoresizingMaskIntoConstraints = false
        return iv
    }()

    private let heightClipBottomIcon: UIImageView = {
        let config = UIImage.SymbolConfiguration(pointSize: 14, weight: .medium)
        let iv = UIImageView(image: UIImage(systemName: "scissors", withConfiguration: config))
        iv.tintColor = UIColor(white: 0.7, alpha: 1)
        iv.translatesAutoresizingMaskIntoConstraints = false
        return iv
    }()

    // 포인트 크기 슬라이더
    private let pointSizeSlider: UISlider = {
        let slider = UISlider()
        slider.minimumValue = 1.0
        slider.maximumValue = 15.0
        slider.value = 3.0
        slider.minimumTrackTintColor = .white
        slider.maximumTrackTintColor = UIColor(white: 0.4, alpha: 0.6)
        slider.thumbTintColor = .white
        slider.translatesAutoresizingMaskIntoConstraints = false
        return slider
    }()

    private let pointSizeSmallDot: UIImageView = {
        let config = UIImage.SymbolConfiguration(pointSize: 6, weight: .regular)
        let iv = UIImageView(image: UIImage(systemName: "circle.fill", withConfiguration: config))
        iv.tintColor = UIColor(white: 0.7, alpha: 1)
        iv.translatesAutoresizingMaskIntoConstraints = false
        return iv
    }()

    private let pointSizeLargeDot: UIImageView = {
        let config = UIImage.SymbolConfiguration(pointSize: 14, weight: .regular)
        let iv = UIImageView(image: UIImage(systemName: "circle.fill", withConfiguration: config))
        iv.tintColor = UIColor(white: 0.7, alpha: 1)
        iv.translatesAutoresizingMaskIntoConstraints = false
        return iv
    }()

    private var clipYMin: Float = 0
    private var clipYMax: Float = 1

    // MARK: - Lifecycle

    override func viewDidLoad() {
        super.viewDidLoad()

        view.backgroundColor = .black

        // SCNView를 코드로 생성하여 전체 화면에 배치
        sceneView = SCNView()
        sceneView.translatesAutoresizingMaskIntoConstraints = false
        sceneView.backgroundColor = .black
        sceneView.allowsCameraControl = true
        sceneView.defaultCameraController.interactionMode = .orbitTurntable
        sceneView.defaultCameraController.inertiaEnabled = true
        sceneView.antialiasingMode = .multisampling4X
        sceneView.scene = SCNScene()
        sceneView.delegate = self
        view.addSubview(sceneView)

        NSLayoutConstraint.activate([
            sceneView.topAnchor.constraint(equalTo: view.topAnchor),
            sceneView.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            sceneView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            sceneView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
        ])

        // 로딩 인디케이터
        view.addSubview(loadingIndicator)
        NSLayoutConstraint.activate([
            loadingIndicator.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            loadingIndicator.centerYAnchor.constraint(equalTo: view.centerYAnchor),
        ])

        setupSliders()

        if let fileURL = fileURL {
            loadingIndicator.startAnimating()
            loadPointCloudFromFile(fileURL: fileURL)
        }
    }

    // MARK: - Slider Setup

    private func setupSliders() {
        // 높이 클리핑 슬라이더 (우측 세로)
        view.addSubview(heightClipTopIcon)
        view.addSubview(heightClipSlider)
        view.addSubview(heightClipBottomIcon)
        heightClipSlider.transform = CGAffineTransform(rotationAngle: -.pi / 2)
        heightClipSlider.addTarget(self, action: #selector(heightClipChanged(_:)), for: .valueChanged)

        // 포인트 크기 슬라이더 (하단 가로)
        view.addSubview(pointSizeSmallDot)
        view.addSubview(pointSizeSlider)
        view.addSubview(pointSizeLargeDot)
        pointSizeSlider.addTarget(self, action: #selector(pointSizeChanged(_:)), for: .valueChanged)

        NSLayoutConstraint.activate([
            // 높이 클리핑 (우측)
            heightClipTopIcon.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -14),
            heightClipTopIcon.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 60),

            heightClipSlider.centerXAnchor.constraint(equalTo: heightClipTopIcon.centerXAnchor),
            heightClipSlider.topAnchor.constraint(equalTo: heightClipTopIcon.bottomAnchor, constant: 80),
            heightClipSlider.widthAnchor.constraint(equalToConstant: 200),

            heightClipBottomIcon.centerXAnchor.constraint(equalTo: heightClipTopIcon.centerXAnchor),
            heightClipBottomIcon.topAnchor.constraint(equalTo: heightClipSlider.bottomAnchor, constant: 80),

            // 포인트 크기 (하단)
            pointSizeSmallDot.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 20),
            pointSizeSmallDot.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -20),

            pointSizeSlider.leadingAnchor.constraint(equalTo: pointSizeSmallDot.trailingAnchor, constant: 8),
            pointSizeSlider.trailingAnchor.constraint(equalTo: pointSizeLargeDot.leadingAnchor, constant: -8),
            pointSizeSlider.centerYAnchor.constraint(equalTo: pointSizeSmallDot.centerYAnchor),

            pointSizeLargeDot.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -20),
            pointSizeLargeDot.centerYAnchor.constraint(equalTo: pointSizeSmallDot.centerYAnchor),
        ])
    }

    // MARK: - PLY Point Cloud Loading

    private func loadPointCloudFromFile(fileURL: URL) {
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else { return }

            let ext = fileURL.pathExtension.lowercased()
            let points: [(pos: SIMD3<Float>, col: SIMD3<Float>)]

            switch ext {
            case "ply":
                points = self.parsePLY(url: fileURL)
            case "xyz":
                points = self.parseXYZ(url: fileURL)
            default:
                DispatchQueue.main.async {
                    self.loadingIndicator.stopAnimating()
                    self.loadAsScene(fileURL: fileURL)
                }
                return
            }

            guard !points.isEmpty else {
                print("⚠️ 파일에서 포인트를 읽지 못함: \(fileURL.lastPathComponent)")
                DispatchQueue.main.async { self.loadingIndicator.stopAnimating() }
                return
            }

            let geometry = self.buildPointCloudGeometry(points: points)

            DispatchQueue.main.async {
                self.loadingIndicator.stopAnimating()
                self.addPointCloudToScene(geometry: geometry)
            }
        }
    }

    /// PLY (ASCII + Binary Little Endian) 파서
    private func parsePLY(url: URL) -> [(pos: SIMD3<Float>, col: SIMD3<Float>)] {
        guard let data = try? Data(contentsOf: url) else { return [] }

        guard let headerEndRange = data.range(of: Data("end_header\n".utf8)) else { return [] }
        let headerData = data[data.startIndex..<headerEndRange.lowerBound]
        guard let headerStr = String(data: headerData, encoding: .utf8) else { return [] }

        let headerLines = headerStr.components(separatedBy: "\n")
        var vertexCount = 0
        var isBinary = false
        var properties: [String] = []

        for line in headerLines {
            let parts = line.trimmingCharacters(in: .whitespaces).components(separatedBy: " ")
            if parts.first == "element" && parts.count >= 3 && parts[1] == "vertex" {
                vertexCount = Int(parts[2]) ?? 0
            } else if parts.first == "format" {
                isBinary = line.contains("binary_little_endian")
            } else if parts.first == "property" && parts.count >= 3 {
                properties.append(parts.last ?? "")
            }
        }

        guard vertexCount > 0 else { return [] }

        let xIdx = properties.firstIndex(of: "x")
        let yIdx = properties.firstIndex(of: "y")
        let zIdx = properties.firstIndex(of: "z")
        let rIdx = properties.firstIndex(of: "red")
        let gIdx = properties.firstIndex(of: "green")
        let bIdx = properties.firstIndex(of: "blue")

        guard let xi = xIdx, let yi = yIdx, let zi = zIdx else { return [] }
        let hasColor = rIdx != nil && gIdx != nil && bIdx != nil

        let bodyStart = headerEndRange.upperBound
        var result: [(pos: SIMD3<Float>, col: SIMD3<Float>)] = []
        result.reserveCapacity(vertexCount)

        if isBinary {
            var propSizes: [Int] = []
            var propOffsets: [Int] = []
            var offset = 0
            for line in headerLines {
                let parts = line.trimmingCharacters(in: .whitespaces).components(separatedBy: " ")
                guard parts.first == "property", parts.count >= 3 else { continue }
                let type = parts[1]
                let size: Int
                switch type {
                case "float", "float32": size = 4
                case "double", "float64": size = 8
                case "uchar", "uint8": size = 1
                case "short", "int16": size = 2
                case "int", "int32", "uint": size = 4
                default: size = 4
                }
                propSizes.append(size)
                propOffsets.append(offset)
                offset += size
            }
            let vertexStride = offset

            guard vertexStride > 0 else { return [] }

            data.withUnsafeBytes { rawBuffer in
                let base = rawBuffer.baseAddress!.advanced(by: bodyStart)
                for i in 0..<vertexCount {
                    let vBase = base.advanced(by: i * vertexStride)

                    guard xi < propOffsets.count, yi < propOffsets.count, zi < propOffsets.count else { break }

                    let x = vBase.advanced(by: propOffsets[xi]).assumingMemoryBound(to: Float.self).pointee
                    let y = vBase.advanced(by: propOffsets[yi]).assumingMemoryBound(to: Float.self).pointee
                    let z = vBase.advanced(by: propOffsets[zi]).assumingMemoryBound(to: Float.self).pointee

                    var r: Float = 0.7, g: Float = 0.7, b: Float = 0.7
                    if hasColor, let ri = rIdx, let gi = gIdx, let bi = bIdx,
                       ri < propOffsets.count, gi < propOffsets.count, bi < propOffsets.count {
                        if propSizes[ri] == 1 {
                            r = Float(vBase.advanced(by: propOffsets[ri]).assumingMemoryBound(to: UInt8.self).pointee) / 255.0
                            g = Float(vBase.advanced(by: propOffsets[gi]).assumingMemoryBound(to: UInt8.self).pointee) / 255.0
                            b = Float(vBase.advanced(by: propOffsets[bi]).assumingMemoryBound(to: UInt8.self).pointee) / 255.0
                        } else {
                            r = vBase.advanced(by: propOffsets[ri]).assumingMemoryBound(to: Float.self).pointee
                            g = vBase.advanced(by: propOffsets[gi]).assumingMemoryBound(to: Float.self).pointee
                            b = vBase.advanced(by: propOffsets[bi]).assumingMemoryBound(to: Float.self).pointee
                        }
                    }

                    result.append((pos: SIMD3<Float>(x, y, z), col: SIMD3<Float>(r, g, b)))
                }
            }
        } else {
            let bodyData = data[bodyStart...]
            guard let bodyStr = String(data: bodyData, encoding: .utf8) else { return [] }
            let lines = bodyStr.components(separatedBy: "\n")

            for i in 0..<min(vertexCount, lines.count) {
                let parts = lines[i].trimmingCharacters(in: .whitespaces).components(separatedBy: " ")
                guard parts.count > max(xi, yi, zi) else { continue }

                guard let x = Float(parts[xi]), let y = Float(parts[yi]), let z = Float(parts[zi]) else { continue }

                var r: Float = 0.7, g: Float = 0.7, b: Float = 0.7
                if hasColor, let ri = rIdx, let gi = gIdx, let bi = bIdx, parts.count > max(ri, gi, bi) {
                    if let rv = Float(parts[ri]), let gv = Float(parts[gi]), let bv = Float(parts[bi]) {
                        r = rv > 1 ? rv / 255.0 : rv
                        g = gv > 1 ? gv / 255.0 : gv
                        b = bv > 1 ? bv / 255.0 : bv
                    }
                }

                result.append((pos: SIMD3<Float>(x, y, z), col: SIMD3<Float>(r, g, b)))
            }
        }

        print("📂 PLY 로드: \(result.count)/\(vertexCount) 포인트 (\(isBinary ? "binary" : "ascii"))")
        return result
    }

    /// XYZ 파서 (x y z r g b)
    private func parseXYZ(url: URL) -> [(pos: SIMD3<Float>, col: SIMD3<Float>)] {
        guard let content = try? String(contentsOf: url, encoding: .utf8) else { return [] }
        var result: [(pos: SIMD3<Float>, col: SIMD3<Float>)] = []
        let lines = content.components(separatedBy: "\n")
        result.reserveCapacity(lines.count)

        for line in lines {
            let parts = line.trimmingCharacters(in: .whitespaces).components(separatedBy: " ").filter { !$0.isEmpty }
            guard parts.count >= 3,
                  let x = Float(parts[0]), let y = Float(parts[1]), let z = Float(parts[2]) else { continue }

            var r: Float = 0.7, g: Float = 0.7, b: Float = 0.7
            if parts.count >= 6, let rv = Float(parts[3]), let gv = Float(parts[4]), let bv = Float(parts[5]) {
                r = rv > 1 ? rv / 255.0 : rv
                g = gv > 1 ? gv / 255.0 : gv
                b = bv > 1 ? bv / 255.0 : bv
            }

            result.append((pos: SIMD3<Float>(x, y, z), col: SIMD3<Float>(r, g, b)))
        }

        print("📂 XYZ 로드: \(result.count) 포인트")
        return result
    }

    /// 포인트 클라우드 → SCNGeometry (ScanPreviewVC와 동일 방식)
    private func buildPointCloudGeometry(points: [(pos: SIMD3<Float>, col: SIMD3<Float>)]) -> SCNGeometry {
        let count = points.count
        guard count > 0 else { return SCNGeometry(sources: [], elements: []) }

        let maxDisplay = 2_000_000
        let stride = count > maxDisplay ? (count + maxDisplay - 1) / maxDisplay : 1
        let displayCount = (count + stride - 1) / stride

        let positionData = UnsafeMutableBufferPointer<SIMD3<Float>>.allocate(capacity: displayCount)
        let colorData = UnsafeMutableBufferPointer<SIMD4<Float>>.allocate(capacity: displayCount)
        defer {
            positionData.deallocate()
            colorData.deallocate()
        }

        var outIdx = 0
        var idx = 0
        while idx < count {
            let pt = points[idx]
            positionData[outIdx] = pt.pos
            colorData[outIdx] = SIMD4<Float>(pt.col.x, pt.col.y, pt.col.z, 1.0)
            outIdx += 1
            idx += stride
        }

        guard outIdx > 0, let posBase = positionData.baseAddress, let colBase = colorData.baseAddress else {
            return SCNGeometry(sources: [], elements: [])
        }

        let posBytes = outIdx * MemoryLayout<SIMD3<Float>>.stride
        let posData = Data(bytes: posBase, count: posBytes)
        let posSource = SCNGeometrySource(
            data: posData, semantic: .vertex, vectorCount: outIdx,
            usesFloatComponents: true, componentsPerVector: 3,
            bytesPerComponent: MemoryLayout<Float>.size,
            dataOffset: 0, dataStride: MemoryLayout<SIMD3<Float>>.stride
        )

        let colBytes = outIdx * MemoryLayout<SIMD4<Float>>.stride
        let colData = Data(bytes: colBase, count: colBytes)
        let colorSource = SCNGeometrySource(
            data: colData, semantic: .color, vectorCount: outIdx,
            usesFloatComponents: true, componentsPerVector: 4,
            bytesPerComponent: MemoryLayout<Float>.size,
            dataOffset: 0, dataStride: MemoryLayout<SIMD4<Float>>.stride
        )

        let element = SCNGeometryElement(
            data: nil, primitiveType: .point, primitiveCount: outIdx,
            bytesPerIndex: MemoryLayout<UInt32>.size
        )
        element.pointSize = 3.0
        element.minimumPointScreenSpaceRadius = 1.0
        element.maximumPointScreenSpaceRadius = 5.0

        let geometry = SCNGeometry(sources: [posSource, colorSource], elements: [element])

        let material = SCNMaterial()
        material.lightingModel = .constant
        material.isDoubleSided = true

        let geoModifier = """
        #pragma arguments
        float clipMaxY;

        #pragma body
        if (_geometry.position.y > clipMaxY) {
            _geometry.position.xyz = float3(0.0, 0.0, -1.0e8);
        }
        """
        material.shaderModifiers = [.geometry: geoModifier]
        material.setValue(NSNumber(value: Float(999)), forKey: "clipMaxY")

        geometry.materials = [material]
        return geometry
    }

    private func addPointCloudToScene(geometry: SCNGeometry) {
        guard let scene = sceneView.scene else { return }

        let node = SCNNode(geometry: geometry)
        scene.rootNode.addChildNode(node)
        pointCloudNode = node

        // 바운딩 박스 기반 카메라 배치
        let (minBound, maxBound) = node.boundingBox
        let center = SCNVector3(
            (minBound.x + maxBound.x) / 2,
            (minBound.y + maxBound.y) / 2,
            (minBound.z + maxBound.z) / 2
        )
        let size = SCNVector3(
            maxBound.x - minBound.x,
            maxBound.y - minBound.y,
            maxBound.z - minBound.z
        )
        let maxDimension = max(size.x, max(size.y, size.z))

        let cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        cameraNode.camera?.zNear = 0.01
        cameraNode.camera?.zFar = Double(maxDimension) * 10
        cameraNode.camera?.fieldOfView = 60
        cameraNode.position = SCNVector3(center.x, center.y + maxDimension * 0.5, center.z + maxDimension * 2.5)
        cameraNode.look(at: center, up: SCNVector3(0, 1, 0), localFront: SCNVector3(0, 0, -1))
        scene.rootNode.addChildNode(cameraNode)

        sceneView.pointOfView = cameraNode
        sceneView.defaultCameraController.target = center
        sceneView.defaultCameraController.worldUp = SCNVector3(0, 1, 0)

        // 높이 클리핑 슬라이더 범위 설정
        clipYMin = minBound.y
        clipYMax = maxBound.y
        let yRange = clipYMax - clipYMin
        heightClipSlider.minimumValue = clipYMin
        heightClipSlider.maximumValue = clipYMax + yRange * 0.05
        heightClipSlider.value = heightClipSlider.maximumValue
        if let mat = node.geometry?.materials.first {
            mat.setValue(NSNumber(value: heightClipSlider.maximumValue), forKey: "clipMaxY")
        }
    }

    /// 지원하지 않는 포맷 fallback
    private func loadAsScene(fileURL: URL) {
        guard let scene = try? SCNScene(url: fileURL, options: nil) else {
            print("Failed to load 3D model from URL: \(fileURL)")
            return
        }
        sceneView.scene = scene
    }

    // MARK: - Slider Actions

    @objc private func heightClipChanged(_ slider: UISlider) {
        guard let mat = pointCloudNode?.geometry?.materials.first else { return }
        mat.setValue(NSNumber(value: slider.value), forKey: "clipMaxY")
    }

    @objc private func pointSizeChanged(_ slider: UISlider) {
        guard let geometry = pointCloudNode?.geometry,
              let element = geometry.elements.first else { return }
        let size = CGFloat(slider.value)
        element.pointSize = size
        element.minimumPointScreenSpaceRadius = max(size * 0.3, 0.5)
        element.maximumPointScreenSpaceRadius = size * 2.0
    }
}

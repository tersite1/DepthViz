//
//  ScanPreviewVC.swift
//  DepthViz
//
//  스캔 완료 후 미리보기 화면 (SceneKit 3D 뷰어 + 저장/삭제 기능)
//

import UIKit
import SceneKit
import CoreLocation
import simd
import SwiftUI
#if canImport(GoogleMobileAds)
import GoogleMobileAds
#endif

protocol ScanPreviewDelegate: AnyObject {
    func scanPreviewDidSave(_ preview: ScanPreviewVC, scanData: ScanData)
    func scanPreviewDidDelete(_ preview: ScanPreviewVC)
}

class ScanPreviewVC: UIViewController {

    var scanData: ScanData?
    var renderer: Renderer?
    var currentMarker: LocationMarker?
    weak var delegate: ScanPreviewDelegate?

    private var scnView: SCNView!
    private var pointCloudNode: SCNNode?

    private var currentFileName: String = ""
    private static var scanCounter: Int = {
        return UserDefaults.standard.integer(forKey: "scan_counter")
    }()

    /// 프리뷰용 최대 표시 포인트 수 (SceneKit 성능 한계)
    private let maxDisplayPoints = 2_000_000

    /// Pre-export: 프리뷰 로드 시 백그라운드에서 미리 내보내기
    private var preExportedData: Data?
    private var preExportFormat: FileFormat?

    /// 배너 광고
    #if canImport(GoogleMobileAds)
    private var bannerView: GADBannerView?
    #endif
    private var saveButtonBottomConstraint: NSLayoutConstraint?

    // MARK: - UI Elements

    private let backButton: UIButton = {
        let button = UIButton(type: .custom)
        let config = UIImage.SymbolConfiguration(pointSize: 18, weight: .semibold)
        button.setImage(UIImage(systemName: "chevron.left", withConfiguration: config), for: .normal)
        button.tintColor = .white
        button.backgroundColor = UIColor(white: 0.15, alpha: 0.7)
        button.layer.cornerRadius = 20
        button.clipsToBounds = true
        button.translatesAutoresizingMaskIntoConstraints = false
        return button
    }()

    private let editButton: UIButton = {
        let button = UIButton(type: .custom)
        let config = UIImage.SymbolConfiguration(pointSize: 20, weight: .regular)
        button.setImage(UIImage(systemName: "pencil", withConfiguration: config), for: .normal)
        button.tintColor = .white
        button.backgroundColor = UIColor(white: 0.15, alpha: 0.7)
        button.layer.cornerRadius = 22
        button.clipsToBounds = true
        button.translatesAutoresizingMaskIntoConstraints = false
        return button
    }()

    private let shareButton: UIButton = {
        let button = UIButton(type: .custom)
        let config = UIImage.SymbolConfiguration(pointSize: 20, weight: .regular)
        button.setImage(UIImage(systemName: "square.and.arrow.up", withConfiguration: config), for: .normal)
        button.tintColor = .white
        button.backgroundColor = UIColor(white: 0.15, alpha: 0.7)
        button.layer.cornerRadius = 22
        button.clipsToBounds = true
        button.translatesAutoresizingMaskIntoConstraints = false
        return button
    }()

    private let saveButton: UIButton = {
        let button = UIButton(type: .custom)
        button.setTitle("Save Scan", for: .normal)
        button.titleLabel?.font = .systemFont(ofSize: 17, weight: .semibold)
        button.backgroundColor = UIColor(white: 0.2, alpha: 0.8)
        button.setTitleColor(.white, for: .normal)
        button.layer.cornerRadius = 25
        button.clipsToBounds = true
        button.translatesAutoresizingMaskIntoConstraints = false
        return button
    }()

    private let infoLabel: UILabel = {
        let label = UILabel()
        label.font = .systemFont(ofSize: 14, weight: .medium)
        label.textColor = .white
        label.textAlignment = .center
        label.translatesAutoresizingMaskIntoConstraints = false
        return label
    }()

    private let loadingIndicator: UIActivityIndicatorView = {
        let indicator = UIActivityIndicatorView(style: .large)
        indicator.color = .white
        indicator.hidesWhenStopped = true
        indicator.translatesAutoresizingMaskIntoConstraints = false
        return indicator
    }()

    // 높이(Y축) 클리핑 슬라이더 — 천장 제거용
    private let heightClipSlider: UISlider = {
        let slider = UISlider()
        slider.minimumValue = 0
        slider.maximumValue = 1
        slider.value = 1 // 기본값: 전부 표시
        slider.minimumTrackTintColor = UIColor(white: 0.4, alpha: 0.6)
        slider.maximumTrackTintColor = .white
        slider.thumbTintColor = .white
        slider.translatesAutoresizingMaskIntoConstraints = false
        return slider
    }()

    /// 높이 슬라이더 아이콘 (위: 전체, 아래: 자르기)
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

    /// Y축 범위 (바운딩 박스에서 계산)
    private var clipYMin: Float = 0
    private var clipYMax: Float = 1

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

    // MARK: - Lifecycle

    override func viewDidLoad() {
        super.viewDidLoad()

        generateDefaultFileName()
        setupSceneView()
        setupButtons()
        setupGestures()

        if let renderer = renderer {
            loadPointCloud(from: renderer)
            startPreExport(renderer: renderer)
        }

        // 10회 이상 비프리미엄 → 하단 배너 광고
        if ScanCountManager.shared.shouldShowBannerAd {
            setupBannerAd()
        }
    }

    // MARK: - Banner Ad

    private func setupBannerAd() {
        #if canImport(GoogleMobileAds)
        let banner = GADBannerView(adSize: GADAdSizeBanner)
        #if DEBUG
        banner.adUnitID = "ca-app-pub-3940256099942544/2934735716"
        #else
        banner.adUnitID = "ca-app-pub-2516597008794244/6421361743"
        #endif
        banner.rootViewController = self
        banner.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(banner)

        // 배너를 맨 하단에 배치
        NSLayoutConstraint.activate([
            banner.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            banner.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor),
            banner.widthAnchor.constraint(equalToConstant: GADAdSizeBanner.size.width),
            banner.heightAnchor.constraint(equalToConstant: GADAdSizeBanner.size.height)
        ])

        // saveButton + 슬라이더를 배너 위로 올림
        saveButtonBottomConstraint?.isActive = false
        saveButtonBottomConstraint = saveButton.bottomAnchor.constraint(equalTo: banner.topAnchor, constant: -8)
        saveButtonBottomConstraint?.isActive = true

        banner.load(GADRequest())
        self.bannerView = banner
        #endif
    }

    // MARK: - Pre-Export (백그라운드에서 미리 내보내기)

    private func startPreExport(renderer: Renderer) {
        let format = ScanSettings.shared.fileFormat
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            let startTime = CFAbsoluteTimeGetCurrent()
            let data = PointCloudExporter.export(renderer: renderer, format: format)
            let elapsed = CFAbsoluteTimeGetCurrent() - startTime
            print("⚡ Pre-export 완료: \(String(format: "%.2f", elapsed))초")
            DispatchQueue.main.async {
                self?.preExportedData = data
                self?.preExportFormat = format
            }
        }
    }

    // MARK: - SceneKit Setup

    private func setupSceneView() {
        scnView = SCNView(frame: view.bounds)
        scnView.translatesAutoresizingMaskIntoConstraints = false
        scnView.backgroundColor = .black
        scnView.allowsCameraControl = true
        scnView.defaultCameraController.interactionMode = .orbitTurntable
        scnView.defaultCameraController.inertiaEnabled = true
        scnView.antialiasingMode = .multisampling4X
        scnView.scene = SCNScene()

        view.addSubview(scnView)
        view.sendSubviewToBack(scnView)
        NSLayoutConstraint.activate([
            scnView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            scnView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            scnView.topAnchor.constraint(equalTo: view.topAnchor),
            scnView.bottomAnchor.constraint(equalTo: view.bottomAnchor)
        ])

        // 로딩 인디케이터
        view.addSubview(loadingIndicator)
        NSLayoutConstraint.activate([
            loadingIndicator.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            loadingIndicator.centerYAnchor.constraint(equalTo: view.centerYAnchor)
        ])
    }

    // MARK: - Point Cloud Loading

    private func loadPointCloud(from renderer: Renderer) {
        // 버퍼 읽기 전에 녹화 중지 보장
        renderer.isRecording = false

        let totalPoints = renderer.currentPointCount
        guard totalPoints > 0 else {
            infoLabel.text = "No points"
            return
        }

        loadingIndicator.startAnimating()
        infoLabel.text = "Loading..."

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else { return }
            let geometry = self.buildPointCloudGeometry(from: renderer, totalPoints: totalPoints)

            DispatchQueue.main.async {
                self.loadingIndicator.stopAnimating()
                self.addPointCloudToScene(geometry: geometry)
                self.updateInfoLabel(with: totalPoints)
            }
        }
    }

    private func buildPointCloudGeometry(from renderer: Renderer, totalPoints: Int) -> SCNGeometry {
        guard totalPoints > 0 else {
            return SCNGeometry(sources: [], elements: [])
        }

        // 다운샘플링: maxDisplayPoints 초과 시 stride 적용
        let stride: Int
        if totalPoints > maxDisplayPoints {
            stride = (totalPoints + maxDisplayPoints - 1) / maxDisplayPoints
        } else {
            stride = 1
        }
        let displayCount = (totalPoints + stride - 1) / stride

        // ARKit 월드 좌표를 그대로 사용 (Y = 중력 위쪽 = SceneKit Y축)
        // startCameraTransform.inverse로 회전하면 중력 방향과 SceneKit Y축이 어긋나서
        // orbitTurntable 스와이프 방향이 뒤바뀜
        // 대신 스캔 시작 위치만 빼서 원점 근처로 이동 (float 정밀도 보존)
        let originOffset: SIMD3<Float>
        if let startTransform = renderer.getStartCameraTransform() {
            originOffset = SIMD3<Float>(startTransform.columns.3.x, startTransform.columns.3.y, startTransform.columns.3.z)
        } else {
            originOffset = .zero
        }

        // Pre-allocate buffers
        let positionData = UnsafeMutableBufferPointer<SIMD3<Float>>.allocate(capacity: displayCount)
        let colorData = UnsafeMutableBufferPointer<SIMD4<Float>>.allocate(capacity: displayCount)
        defer {
            positionData.deallocate()
            colorData.deallocate()
        }

        var actualCount = 0
        renderer.particlesBuffer.withUnsafeBufferPointer { buffer in
            let bufferCount = min(totalPoints, buffer.count)
            var outIdx = 0
            var i = 0
            while i < bufferCount {
                let particle = buffer[i]
                i += stride

                let pos = particle.position - originOffset
                positionData[outIdx] = pos
                colorData[outIdx] = SIMD4<Float>(particle.color.x, particle.color.y, particle.color.z, 1.0)
                outIdx += 1
            }
            actualCount = outIdx
        }

        guard actualCount > 0, let posBase = positionData.baseAddress, let colBase = colorData.baseAddress else {
            return SCNGeometry(sources: [], elements: [])
        }

        // SCNGeometrySource — positions
        let posBytes = actualCount * MemoryLayout<SIMD3<Float>>.stride
        let posData = Data(bytes: posBase, count: posBytes)
        let posSource = SCNGeometrySource(
            data: posData,
            semantic: .vertex,
            vectorCount: actualCount,
            usesFloatComponents: true,
            componentsPerVector: 3,
            bytesPerComponent: MemoryLayout<Float>.size,
            dataOffset: 0,
            dataStride: MemoryLayout<SIMD3<Float>>.stride
        )

        // SCNGeometrySource — colors
        let colBytes = actualCount * MemoryLayout<SIMD4<Float>>.stride
        let colData = Data(bytes: colBase, count: colBytes)
        let colorSource = SCNGeometrySource(
            data: colData,
            semantic: .color,
            vectorCount: actualCount,
            usesFloatComponents: true,
            componentsPerVector: 4,
            bytesPerComponent: MemoryLayout<Float>.size,
            dataOffset: 0,
            dataStride: MemoryLayout<SIMD4<Float>>.stride
        )

        // SCNGeometryElement — point primitives (nil data = 순차 인덱싱)
        let element = SCNGeometryElement(
            data: nil,
            primitiveType: .point,
            primitiveCount: actualCount,
            bytesPerIndex: MemoryLayout<UInt32>.size
        )
        element.pointSize = 3.0
        element.minimumPointScreenSpaceRadius = 1.0
        element.maximumPointScreenSpaceRadius = 5.0

        let geometry = SCNGeometry(sources: [posSource, colorSource], elements: [element])

        // Material: 조명 영향 없이 vertex color 그대로 표시
        let material = SCNMaterial()
        material.lightingModel = .constant
        material.isDoubleSided = true

        // Y축(높이) 클리핑: geometry modifier에서 clipMaxY 초과 포인트를 화면 밖으로 이동
        // Metal SceneKit은 #pragma varyings 미지원 → geometry modifier만 사용
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
        guard let scene = scnView.scene else { return }

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

        // 카메라: 스캔 시작 위치(원점)에서 포인트클라우드 중심을 바라봄
        let cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        cameraNode.camera?.zNear = 0.01
        cameraNode.camera?.zFar = Double(maxDimension) * 10
        cameraNode.camera?.fieldOfView = 60

        // 원점 = 스캔 시작 위치. 원점에서 약간 뒤로 빠져서 전체를 봄
        let dx = center.x, dy = center.y, dz = center.z
        let distToCenter = sqrt(dx*dx + dy*dy + dz*dz)
        if distToCenter > 0.01 {
            let dir = SIMD3<Float>(dx, dy, dz) / distToCenter
            // 원점에서 중심 반대 방향으로 후퇴
            let pullBack = maxDimension * 2.5
            cameraNode.position = SCNVector3(
                -dir.x * pullBack,
                -dir.y * pullBack + maxDimension * 0.3,
                -dir.z * pullBack
            )
        } else {
            cameraNode.position = SCNVector3(0, maxDimension * 0.5, maxDimension * 3.0)
        }
        cameraNode.look(at: center, up: SCNVector3(0, 1, 0), localFront: SCNVector3(0, 0, -1))
        scene.rootNode.addChildNode(cameraNode)

        scnView.pointOfView = cameraNode

        // orbitTurntable: Y축(중력 방향) 기준 회전 보장
        scnView.defaultCameraController.target = center
        scnView.defaultCameraController.worldUp = SCNVector3(0, 1, 0)

        // 높이 클리핑 슬라이더 범위 설정
        clipYMin = minBound.y
        clipYMax = maxBound.y
        let yRange = clipYMax - clipYMin
        heightClipSlider.minimumValue = clipYMin
        heightClipSlider.maximumValue = clipYMax + yRange * 0.05  // 약간의 여유
        heightClipSlider.value = heightClipSlider.maximumValue  // 기본: 전부 표시
        // 셰이더 초기값 설정
        if let mat = node.geometry?.materials.first {
            mat.setValue(NSNumber(value: heightClipSlider.maximumValue), forKey: "clipMaxY")
        }

        // 트라젝토리 시각화 (프리미엄)
        addTrajectoryLine()
    }

    // MARK: - Trajectory Visualization (Premium)

    /// 형광 보라색
    private static let trajectoryColor = UIColor(red: 0.75, green: 0.0, blue: 1.0, alpha: 1.0)
    /// 튜브 반지름 (두꺼운 선)
    private static let tubeRadius: CGFloat = 0.006

    private func addTrajectoryLine() {
        guard PremiumManager.shared.isPremium,
              PremiumManager.shared.showOdometry,
              let points = scanData?.trajectoryPoints, points.count > 1 else { return }

        // startCameraTransform 기준으로 원점 이동 (포인트 클라우드와 동일)
        let originOffset: SIMD3<Float>
        if let startTransform = scanData?.startCameraTransform {
            originOffset = SIMD3<Float>(startTransform.columns.3.x, startTransform.columns.3.y, startTransform.columns.3.z)
        } else {
            originOffset = .zero
        }

        // 다운샘플: 최대 1000 세그먼트 (성능 보장)
        let maxSegments = 1000
        let stride = max(1, points.count / maxSegments)
        var transformed: [SCNVector3] = []
        for i in Swift.stride(from: 0, to: points.count, by: stride) {
            let pt = points[i] - originOffset
            transformed.append(SCNVector3(pt.x, pt.y, pt.z))
        }
        guard transformed.count > 1 else { return }

        // 형광 보라색 머티리얼 (emission으로 글로우 효과)
        let material = SCNMaterial()
        material.lightingModel = .constant
        material.diffuse.contents = Self.trajectoryColor
        material.emission.contents = Self.trajectoryColor
        material.isDoubleSided = true

        // 컨테이너 노드
        let containerNode = SCNNode()

        for i in 0..<(transformed.count - 1) {
            let a = transformed[i]
            let b = transformed[i + 1]

            let dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z
            let length = sqrt(dx * dx + dy * dy + dz * dz)
            guard length > 0.0005 else { continue }  // 너무 짧은 세그먼트 스킵

            let cylinder = SCNCylinder(radius: Self.tubeRadius, height: CGFloat(length))
            cylinder.radialSegmentCount = 6  // 가벼운 실린더
            cylinder.materials = [material]

            let node = SCNNode(geometry: cylinder)

            // 중점 배치
            node.position = SCNVector3(
                (a.x + b.x) / 2,
                (a.y + b.y) / 2,
                (a.z + b.z) / 2
            )

            // Y축 실린더를 a→b 방향으로 회전
            let dir = SCNVector3(dx, dy, dz)
            let up = SCNVector3(0, 1, 0)
            let cross = SCNVector3(
                up.y * dir.z - up.z * dir.y,
                up.z * dir.x - up.x * dir.z,
                up.x * dir.y - up.y * dir.x
            )
            let crossLen = sqrt(cross.x * cross.x + cross.y * cross.y + cross.z * cross.z)
            let dot = up.x * dir.x + up.y * dir.y + up.z * dir.z

            if crossLen > 1e-6 {
                let angle = atan2(crossLen, dot)
                let axis = SCNVector3(cross.x / crossLen, cross.y / crossLen, cross.z / crossLen)
                node.rotation = SCNVector4(axis.x, axis.y, axis.z, angle)
            } else if dot < 0 {
                // 반대 방향 (180도 회전)
                node.rotation = SCNVector4(1, 0, 0, Float.pi)
            }

            containerNode.addChildNode(node)
        }

        // 시작/끝 지점에 큰 구체 마커
        let startSphere = SCNSphere(radius: Self.tubeRadius * 1.5)
        startSphere.materials = [material]
        let startNode = SCNNode(geometry: startSphere)
        startNode.position = transformed.first!
        containerNode.addChildNode(startNode)

        let endSphere = SCNSphere(radius: Self.tubeRadius * 1.5)
        endSphere.materials = [material]
        let endNode = SCNNode(geometry: endSphere)
        endNode.position = transformed.last!
        containerNode.addChildNode(endNode)

        scnView.scene?.rootNode.addChildNode(containerNode)
        print("🟣 트라젝토리 시각화: \(transformed.count - 1) 세그먼트 (튜브 r=\(Self.tubeRadius))")
    }

    private func updateInfoLabel(with pointCount: Int) {
        let formatter = NumberFormatter()
        formatter.numberStyle = .decimal
        let pointsString = formatter.string(from: NSNumber(value: pointCount)) ?? "\(pointCount)"
        infoLabel.text = "\(pointsString) Points"
    }


    // MARK: - UI Setup

    private func generateDefaultFileName() {
        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyyMMdd_HHmmss"
        let dateString = dateFormatter.string(from: Date())
        ScanPreviewVC.scanCounter += 1
        UserDefaults.standard.set(ScanPreviewVC.scanCounter, forKey: "scan_counter")
        currentFileName = "\(dateString)_Pointcloud_\(ScanPreviewVC.scanCounter)"
    }

    private func setupGestures() {
        // 화면 왼쪽 가장자리에서만 뒤로가기 (orbit 회전과 충돌 방지)
        let edgePan = UIScreenEdgePanGestureRecognizer(target: self, action: #selector(handleEdgePan))
        edgePan.edges = .left
        view.addGestureRecognizer(edgePan)
    }

    @objc private func handleEdgePan(_ gesture: UIScreenEdgePanGestureRecognizer) {
        if gesture.state == .ended {
            backButtonTapped()
        }
    }

    private func setupButtons() {
        view.addSubview(backButton)
        backButton.addTarget(self, action: #selector(backButtonTapped), for: .touchUpInside)

        view.addSubview(editButton)
        editButton.addTarget(self, action: #selector(editButtonTapped), for: .touchUpInside)

        view.addSubview(shareButton)
        shareButton.addTarget(self, action: #selector(shareButtonTapped), for: .touchUpInside)

        view.addSubview(infoLabel)
        view.addSubview(saveButton)
        saveButton.addTarget(self, action: #selector(saveButtonTapped), for: .touchUpInside)

        // 높이 클리핑 슬라이더 (우측 세로 배치)
        view.addSubview(heightClipTopIcon)
        view.addSubview(heightClipSlider)
        view.addSubview(heightClipBottomIcon)
        heightClipSlider.transform = CGAffineTransform(rotationAngle: -.pi / 2)
        heightClipSlider.addTarget(self, action: #selector(heightClipChanged(_:)), for: .valueChanged)

        // 포인트 크기 슬라이더 (하단 가로 배치: ● ——— ⬤)
        view.addSubview(pointSizeSmallDot)
        view.addSubview(pointSizeSlider)
        view.addSubview(pointSizeLargeDot)
        pointSizeSlider.addTarget(self, action: #selector(pointSizeChanged(_:)), for: .valueChanged)

        NSLayoutConstraint.activate([
            backButton.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 20),
            backButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 20),
            backButton.heightAnchor.constraint(equalToConstant: 44),
            backButton.widthAnchor.constraint(equalToConstant: 44),

            shareButton.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -20),
            shareButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 20),
            shareButton.widthAnchor.constraint(equalToConstant: 44),
            shareButton.heightAnchor.constraint(equalToConstant: 44),

            editButton.trailingAnchor.constraint(equalTo: shareButton.leadingAnchor, constant: -12),
            editButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 20),
            editButton.widthAnchor.constraint(equalToConstant: 44),
            editButton.heightAnchor.constraint(equalToConstant: 44),

            // 높이 클리핑 슬라이더 (우측 세로) — 회전된 슬라이더이므로 width=height
            heightClipTopIcon.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -14),
            heightClipTopIcon.topAnchor.constraint(equalTo: shareButton.bottomAnchor, constant: 24),

            heightClipSlider.centerXAnchor.constraint(equalTo: heightClipTopIcon.centerXAnchor),
            heightClipSlider.topAnchor.constraint(equalTo: heightClipTopIcon.bottomAnchor, constant: 80),
            heightClipSlider.widthAnchor.constraint(equalToConstant: 200),  // 회전 후 높이가 됨

            heightClipBottomIcon.centerXAnchor.constraint(equalTo: heightClipTopIcon.centerXAnchor),
            heightClipBottomIcon.topAnchor.constraint(equalTo: heightClipSlider.bottomAnchor, constant: 80),

            // 포인트 크기 슬라이더 — Save 버튼 옆, 하단 가로
            pointSizeSmallDot.leadingAnchor.constraint(equalTo: saveButton.trailingAnchor, constant: 16),
            pointSizeSmallDot.centerYAnchor.constraint(equalTo: saveButton.centerYAnchor),

            pointSizeSlider.leadingAnchor.constraint(equalTo: pointSizeSmallDot.trailingAnchor, constant: 8),
            pointSizeSlider.trailingAnchor.constraint(equalTo: pointSizeLargeDot.leadingAnchor, constant: -8),
            pointSizeSlider.centerYAnchor.constraint(equalTo: saveButton.centerYAnchor),

            pointSizeLargeDot.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -20),
            pointSizeLargeDot.centerYAnchor.constraint(equalTo: saveButton.centerYAnchor),

            infoLabel.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 20),
            infoLabel.bottomAnchor.constraint(equalTo: saveButton.topAnchor, constant: -12),

            saveButton.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 20),
            saveButton.widthAnchor.constraint(equalToConstant: 140),
            saveButton.heightAnchor.constraint(equalToConstant: 50)
        ])

        // saveButton 하단 제약 (배너 광고 있으면 위로 올림)
        let bottomC = saveButton.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -20)
        bottomC.isActive = true
        saveButtonBottomConstraint = bottomC
    }

    // MARK: - Height Clip Control

    @objc private func heightClipChanged(_ slider: UISlider) {
        guard let mat = pointCloudNode?.geometry?.materials.first else { return }
        mat.setValue(NSNumber(value: slider.value), forKey: "clipMaxY")
    }

    // MARK: - Point Size Control

    @objc private func pointSizeChanged(_ slider: UISlider) {
        guard let geometry = pointCloudNode?.geometry,
              let element = geometry.elements.first else { return }
        let size = CGFloat(slider.value)
        element.pointSize = size
        element.minimumPointScreenSpaceRadius = max(size * 0.3, 0.5)
        element.maximumPointScreenSpaceRadius = size * 2.0
    }

    // MARK: - Actions

    @objc func backButtonTapped() {
        // SLAM 엔진 정지
        SLAMService.sharedInstance().stop()

        // 렌더러 정리
        renderer?.clearParticles()
        renderer = nil

        // 스캔 데이터 정리
        if let scanData = scanData {
            let tempURL = FileManager.default.temporaryDirectory.appendingPathComponent(scanData.fileName)
            try? FileManager.default.removeItem(at: tempURL)
        }
        scanData = nil

        if delegate != nil {
            // delegate가 dismiss + 팝업 처리
            delegate?.scanPreviewDidDelete(self)
        } else {
            // fallback: delegate 없으면 자체 dismiss
            dismiss(animated: true)
        }
    }

    @objc func editButtonTapped() {
        let alert = UIAlertController(title: "Rename File", message: "Enter a new file name", preferredStyle: .alert)
        alert.addTextField { [weak self] textField in
            textField.text = self?.currentFileName
            textField.placeholder = "File Name"
            textField.clearButtonMode = .whileEditing
        }
        alert.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        alert.addAction(UIAlertAction(title: "OK", style: .default) { [weak self] _ in
            guard let self = self,
                  let newName = alert.textFields?.first?.text,
                  !newName.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty else {
                return
            }
            self.currentFileName = newName.trimmingCharacters(in: .whitespacesAndNewlines)
            let feedback = UINotificationFeedbackGenerator()
            feedback.notificationOccurred(.success)
        })
        present(alert, animated: true)
    }

    @objc func shareButtonTapped() {
        guard let renderer = renderer else { return }
        let format = ScanSettings.shared.fileFormat
        let fileExtension = format.fileExtension
        let alert = UIAlertController(title: "파일 생성 중...", message: "잠시만 기다려주세요", preferredStyle: .alert)
        present(alert, animated: true)
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else { return }
            let fileData = PointCloudExporter.export(renderer: renderer, format: format) ?? Data()
            DispatchQueue.main.async {
                alert.dismiss(animated: true) {
                    let tempURL = FileManager.default.temporaryDirectory.appendingPathComponent("\(self.currentFileName).\(fileExtension)")
                    do {
                        try fileData.write(to: tempURL)
                        let activityVC = UIActivityViewController(activityItems: [tempURL], applicationActivities: nil)
                        if let popover = activityVC.popoverPresentationController {
                            popover.sourceView = self.shareButton
                            popover.sourceRect = self.shareButton.bounds
                        }
                        self.present(activityVC, animated: true)
                    } catch {
                        print("Share failed: \(error)")
                    }
                }
            }
        }
    }

    @objc func saveButtonTapped() {
        guard let scanData = scanData, let renderer = renderer else { return }

        // 비프리미엄 + 저장 조건 충족 → 보상형 광고 시청 후 저장
        if ScanCountManager.shared.shouldShowRewardedAd {
            InterstitialAdManager.shared.showAd(from: self) { [weak self] in
                self?.showProjectSelection(scanData: scanData, renderer: renderer)
            }
        } else {
            showProjectSelection(scanData: scanData, renderer: renderer)
        }
    }

    // MARK: - Project Selection & Save Flow

    private func showProjectSelection(scanData: ScanData, renderer: Renderer) {
        let pointCount = renderer.currentPointCount
        let markerName = currentMarker?.name

        let selectionView = ProjectSelectionView(
            defaultFileName: currentFileName,
            pointCount: pointCount,
            markerProjectName: markerName,
            onSave: { [weak self] project, fileName in
                guard let self = self else { return }
                // 시트에서 변경된 최신 format 읽기
                let format = ScanSettings.shared.fileFormat
                let fileExtension = format.fileExtension
                // 시트 닫기 → 저장 실행
                self.presentedViewController?.dismiss(animated: true) {
                    self.executeSave(
                        scanData: scanData,
                        renderer: renderer,
                        project: project,
                        fileName: fileName,
                        format: format,
                        fileExtension: fileExtension
                    )
                }
            },
            onCancel: { [weak self] in
                self?.presentedViewController?.dismiss(animated: true)
            }
        )

        let hostingController = UIHostingController(rootView: selectionView)
        if let sheet = hostingController.sheetPresentationController {
            sheet.detents = [.large()]
            sheet.prefersGrabberVisible = true
        }
        present(hostingController, animated: true)
    }

    private func executeSave(scanData: ScanData, renderer: Renderer, project: String, fileName: String, format: FileFormat, fileExtension: String) {
        // 로딩 오버레이 표시
        let loadingOverlay = showLoadingOverlay()

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else { return }

            // Pre-export 캐시가 있으면 사용, 없으면 새로 생성
            let fileData: Data
            if let cached = self.preExportedData, self.preExportFormat == format {
                fileData = cached
                print("⚡ Pre-export 캐시 사용")
            } else {
                fileData = PointCloudExporter.export(renderer: renderer, format: format) ?? Data()
                print("📝 새로 Export 실행")
            }

            // ScanData 업데이트
            scanData.rename(to: fileName + ".\(fileExtension)")
            scanData.lidarData = fileData
            scanData.fileSize = fileData.fileSize
            let rendererPoints = renderer.currentPointCount
            if rendererPoints > 0 {
                scanData.points = rendererPoints
            }
            scanData.project = project

            // 마커 프로젝트 자동 생성
            if let marker = self.currentMarker {
                if ProjectManager.shared.getProject(byName: marker.name) == nil {
                    let _ = ProjectManager.shared.createProject(name: marker.name, location: marker)
                    print("📁 프로젝트 자동 생성: \(marker.name)")
                }
            }

            // ScanStorage에 저장
            let success = ScanStorage.shared.save(scanData)

            // 프리미엄: IMU + Trajectory CSV 저장 (같은 경로)
            if PremiumManager.shared.isPremium, let renderer = self.renderer {
                let baseName = (fileName as NSString).deletingPathExtension
                self.saveCSVData(baseName: baseName, renderer: renderer)
            }

            DispatchQueue.main.async {
                loadingOverlay.removeFromSuperview()

                if success {
                    ScanCountManager.shared.incrementSaveCount()
                    UINotificationFeedbackGenerator().notificationOccurred(.success)
                    self.delegate?.scanPreviewDidSave(self, scanData: scanData)
                } else {
                    print("📊 [SaveFlow] ⚠️ 저장 실패!")
                    let alert = UIAlertController(title: "Save Failed", message: "Failed to save data.", preferredStyle: .alert)
                    alert.addAction(UIAlertAction(title: "OK", style: .default))
                    self.present(alert, animated: true)
                }
            }
        }
    }

    // MARK: - CSV Export (Premium)

    private func saveCSVData(baseName: String, renderer: Renderer) {
        let exportDir = ScanStorage.shared.exportRoot

        // IMU CSV
        if PremiumManager.shared.showIMUData {
            let imuLog = renderer.getIMULog()
            if !imuLog.isEmpty {
                var csv = "timestamp,roll,pitch,yaw,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n"
                for e in imuLog {
                    csv += String(format: "%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                                  e.timestamp, e.roll, e.pitch, e.yaw,
                                  e.ax, e.ay, e.az, e.gx, e.gy, e.gz)
                }
                let url = exportDir.appendingPathComponent("\(baseName)_imu.csv")
                try? csv.write(to: url, atomically: true, encoding: .utf8)
                print("📊 IMU CSV 저장: \(imuLog.count)개 → 파일 앱/DepthViz/\(url.lastPathComponent)")
            }
        }

        // Trajectory CSV
        if PremiumManager.shared.showOdometry {
            let trajLog = renderer.getTrajectoryLog()
            if !trajLog.isEmpty {
                var csv = "timestamp,x,y,z\n"
                for e in trajLog {
                    csv += String(format: "%.4f,%.6f,%.6f,%.6f\n",
                                  e.timestamp, e.x, e.y, e.z)
                }
                let url = exportDir.appendingPathComponent("\(baseName)_trajectory.csv")
                try? csv.write(to: url, atomically: true, encoding: .utf8)
                print("🗺️ Trajectory CSV 저장: \(trajLog.count)개 → 파일 앱/DepthViz/\(url.lastPathComponent)")
            }
        }
    }

    private func showLoadingOverlay() -> UIView {
        let overlay = UIView(frame: view.bounds)
        overlay.backgroundColor = UIColor.black.withAlphaComponent(0.6)
        overlay.autoresizingMask = [.flexibleWidth, .flexibleHeight]

        let container = UIView()
        container.backgroundColor = UIColor(white: 0.15, alpha: 0.95)
        container.layer.cornerRadius = 16
        container.translatesAutoresizingMaskIntoConstraints = false

        let spinner = UIActivityIndicatorView(style: .large)
        spinner.color = .white
        spinner.startAnimating()
        spinner.translatesAutoresizingMaskIntoConstraints = false

        let label = UILabel()
        label.text = NSLocalizedString("saving", comment: "")
        label.textColor = .white
        label.font = .systemFont(ofSize: 16, weight: .medium)
        label.translatesAutoresizingMaskIntoConstraints = false

        container.addSubview(spinner)
        container.addSubview(label)
        overlay.addSubview(container)
        view.addSubview(overlay)

        NSLayoutConstraint.activate([
            container.centerXAnchor.constraint(equalTo: overlay.centerXAnchor),
            container.centerYAnchor.constraint(equalTo: overlay.centerYAnchor),
            container.widthAnchor.constraint(equalToConstant: 160),
            container.heightAnchor.constraint(equalToConstant: 120),
            spinner.centerXAnchor.constraint(equalTo: container.centerXAnchor),
            spinner.centerYAnchor.constraint(equalTo: container.centerYAnchor, constant: -12),
            label.centerXAnchor.constraint(equalTo: container.centerXAnchor),
            label.topAnchor.constraint(equalTo: spinner.bottomAnchor, constant: 12)
        ])

        return overlay
    }
}

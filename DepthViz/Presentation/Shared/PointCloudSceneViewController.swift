//
//  PointCloudSceneViewController.swift
//  DepthViz
//
//  공통 SceneKit 포인트클라우드 뷰어.
//

import UIKit
import SceneKit
import ARKit
import Combine
import simd

enum PointColorMode {
    case rgb
    case intensity
    case height
    case mono
}

extension Notification.Name {
    static let premiumTrajectoryUpdated = Notification.Name("premiumTrajectoryUpdated")
}

final class PointCloudSceneViewController: UIViewController {
    // 애플 기본 카메라 컨트롤을 사용하도록 통합 (커스텀 제스처 대신)
    private let useAppleCameraControls = true

    private struct CameraControlState {
        var yaw: Float = 0            // 좌우 회전 (라디안)
        var pitch: Float = 0.35       // 상하 회전 (라디안)
        var distance: Float = 3.0     // 타깃까지 거리
        var target: SIMD3<Float> = .zero
    }

    // MARK: - Public configuration
    var renderer: Renderer? {
        didSet {
            trackedTrajectoryPoints = PremiumManager.shared.currentTrajectoryPoints()
            reloadContentIfReady()
        }
    }
    var scanData: ScanData? {
        didSet {
            attachScanDataObserver()
            trackedTrajectoryPoints = scanData?.trajectoryPoints ?? []
            reloadContentIfReady()
        }
    }
    var filePath: String? {
        didSet {
            if let path = filePath {
                loadTrajectoryFromFileIfAvailable(at: path)
            } else {
                trackedTrajectoryPoints = []
            }
            reloadContentIfReady()
        }
    }
    var onClose: (() -> Void)?
    var onPointCloudRendered: ((Int) -> Void)?

    // MARK: - Private properties
    private let sceneView = SCNView()
    private let showCloseButton: Bool
    private lazy var closeButton: UIButton = {
        let button = UIButton(type: .system)
        button.setImage(UIImage(systemName: "xmark.circle.fill"), for: .normal)
        button.tintColor = .white
        button.backgroundColor = UIColor.black.withAlphaComponent(0.55)
        button.layer.cornerRadius = 22
        button.translatesAutoresizingMaskIntoConstraints = false
        button.addTarget(self, action: #selector(closeButtonTapped), for: .touchUpInside)
        return button
    }()

    private let orbitNode = SCNNode()
    private let cameraNode = SCNNode()
    private let modelContainerNode = SCNNode()
    private let trajectoryNode = SCNNode()
    private var pointCloudNode: SCNNode?
    private var cameraState = CameraControlState()
    private let distanceRange: ClosedRange<Float> = 0.2...120
    private var pinchStartDistance: Float = 0.0
    private var cancellables = Set<AnyCancellable>()
    private var translationPanStartModelPosition = simd_float3.zero
    private var rotationGestureStartWidthAngle: Float = 0.0
    private var trackedTrajectoryPoints: [SIMD3<Float>] = [] {
        didSet { updateTrajectoryNode() }
    }
    private lazy var rotationPanGesture: UIPanGestureRecognizer = {
        let gesture = UIPanGestureRecognizer(target: self, action: #selector(handleRotationPan(_:)))
        gesture.minimumNumberOfTouches = 1
        gesture.maximumNumberOfTouches = 1
        gesture.delegate = self
        return gesture
    }()
    private lazy var translationPanGesture: UIPanGestureRecognizer = {
        let gesture = UIPanGestureRecognizer(target: self, action: #selector(handleTwoFingerPan(_:)))
        gesture.minimumNumberOfTouches = 2
        gesture.maximumNumberOfTouches = 2
        gesture.delegate = self
        gesture.cancelsTouchesInView = false
        return gesture
    }()
    private lazy var pinchGestureRecognizer: UIPinchGestureRecognizer = {
        let gesture = UIPinchGestureRecognizer(target: self, action: #selector(handlePinch(_:)))
        gesture.delegate = self
        return gesture
    }()
    private lazy var doubleTapGestureRecognizer: UITapGestureRecognizer = {
        let gesture = UITapGestureRecognizer(target: self, action: #selector(handleDoubleTap(_:)))
        gesture.numberOfTapsRequired = 2
        gesture.delegate = self
        return gesture
    }()
    private lazy var rotationGestureRecognizer: UIRotationGestureRecognizer = {
        let gesture = UIRotationGestureRecognizer(target: self, action: #selector(handleRotationGesture(_:)))
        gesture.delegate = self
        return gesture
    }()
    private var gestureHintTimer: Timer?
    private var currentPointCloud: PointCloud?
    private var colorMode: PointColorMode = .rgb
    private var pointSize: CGFloat = 2.0
    private lazy var controlsStack: UIStackView = {
        let control = UISegmentedControl(items: ["RGB", "Intensity", "Height", "Mono"])
        control.selectedSegmentIndex = 0
        control.addTarget(self, action: #selector(handleColorModeChanged(_:)), for: .valueChanged)
        
        let slider = UISlider()
        slider.minimumValue = 1.0
        slider.maximumValue = 8.0
        slider.value = Float(pointSize)
        slider.addTarget(self, action: #selector(handlePointSizeChanged(_:)), for: .valueChanged)
        
        let label = UILabel()
        label.text = "Point Size"
        label.font = .systemFont(ofSize: 12, weight: .medium)
        label.textColor = .white
        
        let sliderStack = UIStackView(arrangedSubviews: [label, slider])
        sliderStack.axis = .vertical
        sliderStack.spacing = 4
        
        let stack = UIStackView(arrangedSubviews: [control, sliderStack])
        stack.axis = .vertical
        stack.spacing = 8
        stack.translatesAutoresizingMaskIntoConstraints = false
        stack.layoutMargins = UIEdgeInsets(top: 8, left: 10, bottom: 8, right: 10)
        stack.isLayoutMarginsRelativeArrangement = true
        
        let blurEffect = UIBlurEffect(style: .systemThinMaterialDark)
        let blurView = UIVisualEffectView(effect: blurEffect)
        blurView.frame = stack.bounds
        blurView.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        blurView.layer.cornerRadius = 12
        blurView.clipsToBounds = true
        stack.insertSubview(blurView, at: 0)
        
        stack.layer.cornerRadius = 12
        stack.clipsToBounds = true
        return stack
    }()
    private lazy var gestureHintView: UIView = {
        let label = UILabel()
        label.text = "한 손가락: 회전 • 두 손가락: 이동 • 핀치: 확대/축소 • 더블탭: 리셋"
        label.textColor = .white
        label.font = .systemFont(ofSize: 13, weight: .semibold)
        label.numberOfLines = 0
        label.textAlignment = .center
        label.translatesAutoresizingMaskIntoConstraints = false

        let container = UIVisualEffectView(effect: UIBlurEffect(style: .systemUltraThinMaterialDark))
        container.translatesAutoresizingMaskIntoConstraints = false
        container.layer.cornerRadius = 12
        container.clipsToBounds = true
        container.contentView.addSubview(label)
        NSLayoutConstraint.activate([
            label.leadingAnchor.constraint(equalTo: container.contentView.leadingAnchor, constant: 12),
            label.trailingAnchor.constraint(equalTo: container.contentView.trailingAnchor, constant: -12),
            label.topAnchor.constraint(equalTo: container.contentView.topAnchor, constant: 8),
            label.bottomAnchor.constraint(equalTo: container.contentView.bottomAnchor, constant: -8)
        ])
        return container
    }()

    // MARK: - Init
    init(showCloseButton: Bool = false) {
        self.showCloseButton = showCloseButton
        super.init(nibName: nil, bundle: nil)
    }

    required init?(coder: NSCoder) {
        self.showCloseButton = false
        super.init(coder: coder)
    }

    // MARK: - Lifecycle
    override func viewDidLoad() {
        super.viewDidLoad()
        setupSceneView()
        setupSceneGraph()
        setupGestures()
        setupControls()
        setupGestureHint()
        if showCloseButton {
            setupCloseButton()
        }
        NotificationCenter.default.addObserver(self,
                                               selector: #selector(handleTrajectoryUpdate(_:)),
                                               name: .premiumTrajectoryUpdated,
                                               object: nil)
        reloadContentIfReady()
    }

    deinit {
        NotificationCenter.default.removeObserver(self)
        cancellables.removeAll()
    }

    // MARK: - Scene setup
    private func setupSceneView() {
        sceneView.translatesAutoresizingMaskIntoConstraints = false
        sceneView.scene = SCNScene()
        sceneView.backgroundColor = .black  // 어두운 배경
        sceneView.autoenablesDefaultLighting = true    // 기본 라이팅 활성화
        sceneView.allowsCameraControl = useAppleCameraControls
        sceneView.isMultipleTouchEnabled = true
        if useAppleCameraControls {
            sceneView.defaultCameraController.inertiaEnabled = true
            sceneView.defaultCameraController.inertiaFriction = 0.5 
            
            sceneView.defaultCameraController.interactionMode = .orbitArcball
            
            sceneView.defaultCameraController.pointOfView = cameraNode
        }
        view.addSubview(sceneView)

        NSLayoutConstraint.activate([
            sceneView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            sceneView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            sceneView.topAnchor.constraint(equalTo: view.topAnchor),
            sceneView.bottomAnchor.constraint(equalTo: view.bottomAnchor)
        ])
    }

    private func setupSceneGraph() {
        guard let scene = sceneView.scene else { return }
        scene.lightingEnvironment.intensity = 0

        modelContainerNode.name = "modelContainer"
        orbitNode.name = "orbitNode"
        cameraNode.camera = SCNCamera()
        cameraNode.camera?.wantsHDR = false
        cameraNode.camera?.automaticallyAdjustsZRange = false
        cameraNode.camera?.zNear = 0.001
        cameraNode.camera?.zFar = 20000
        orbitNode.addChildNode(cameraNode)

        scene.rootNode.addChildNode(modelContainerNode)
        scene.rootNode.addChildNode(orbitNode)
        trajectoryNode.name = "trajectoryNode"
        trajectoryNode.opacity = 0.85
        modelContainerNode.addChildNode(trajectoryNode)
        sceneView.pointOfView = cameraNode
        if useAppleCameraControls {
            sceneView.defaultCameraController.pointOfView = cameraNode
        }
    }

    private func setupGestures() {
        guard useAppleCameraControls == false else { return }
        sceneView.addGestureRecognizer(rotationPanGesture)
        sceneView.addGestureRecognizer(translationPanGesture)
        sceneView.addGestureRecognizer(pinchGestureRecognizer)
        sceneView.addGestureRecognizer(rotationGestureRecognizer)
        sceneView.addGestureRecognizer(doubleTapGestureRecognizer)
    }

    private func setupControls() {
        view.addSubview(controlsStack)
        NSLayoutConstraint.activate([
            controlsStack.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 12),
            controlsStack.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -12),
            controlsStack.widthAnchor.constraint(lessThanOrEqualToConstant: 220)
        ])
    }

    private func setupCloseButton() {
        view.addSubview(closeButton)
        NSLayoutConstraint.activate([
            closeButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 12),
            closeButton.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -12),
            closeButton.widthAnchor.constraint(equalToConstant: 44),
            closeButton.heightAnchor.constraint(equalToConstant: 44)
        ])
    }

    private func setupGestureHint() {
        guard useAppleCameraControls == false else { return }
        view.addSubview(gestureHintView)
        NSLayoutConstraint.activate([
            gestureHintView.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            gestureHintView.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 12)
        ])

        gestureHintView.alpha = 0
        UIView.animate(withDuration: 0.25) { [weak self] in
            self?.gestureHintView.alpha = 1
        }

        gestureHintTimer?.invalidate()
        gestureHintTimer = Timer.scheduledTimer(withTimeInterval: 4.0, repeats: false) { [weak self] _ in
            UIView.animate(withDuration: 0.3) {
                self?.gestureHintView.alpha = 0
            }
        }
    }

    // MARK: - Gesture handling
    @objc private func handleRotationPan(_ gesture: UIPanGestureRecognizer) {
        guard gesture.numberOfTouches == 1 else { return }

        let translation = gesture.translation(in: sceneView)
        gesture.setTranslation(.zero, in: sceneView)

        let yawDelta = Float(translation.x) * 0.007
        let pitchDelta = Float(translation.y) * 0.007
        cameraState.yaw = normalizeAngle(cameraState.yaw + yawDelta)
        cameraState.pitch = clampPitch(cameraState.pitch + pitchDelta)
        applyCameraState()
    }

    @objc private func handlePinch(_ gesture: UIPinchGestureRecognizer) {
        switch gesture.state {
        case .began:
            pinchStartDistance = cameraState.distance
        case .changed:
            let scale = Float(gesture.scale)
            let newDistance = clampDistance(pinchStartDistance / max(scale, 0.001))
            cameraState.distance = newDistance
            applyCameraState()
        default:
            break
        }
    }

    @objc private func handleTwoFingerPan(_ gesture: UIPanGestureRecognizer) {
        guard gesture.numberOfTouches == 2 else { return }
        
        switch gesture.state {
        case .began:
            translationPanStartModelPosition = modelContainerNode.simdPosition
        case .changed:
            let translation = gesture.translation(in: sceneView)
            let distanceScale = max(0.0025, cameraState.distance * 0.0025)
            let horizontal = cameraNode.simdWorldRight * Float(translation.x) * distanceScale
            let vertical = cameraNode.simdWorldUp * Float(-translation.y) * distanceScale
            modelContainerNode.simdPosition = translationPanStartModelPosition + horizontal + vertical
            applyCameraState()
        case .ended, .cancelled, .failed:
            translationPanStartModelPosition = modelContainerNode.simdPosition
        default:
            break
        }
    }
    
    @objc private func handleRotationGesture(_ gesture: UIRotationGestureRecognizer) {
        guard gesture.numberOfTouches >= 2 else { return }
        
        switch gesture.state {
        case .began:
            rotationGestureStartWidthAngle = cameraState.yaw
        case .changed, .ended:
            let delta = Float(gesture.rotation)
            cameraState.yaw = normalizeAngle(rotationGestureStartWidthAngle - delta)
            applyCameraState()
        default:
            break
        }
    }

    @objc private func handleDoubleTap(_ gesture: UITapGestureRecognizer) {
        guard gesture.state == .ended else { return }
        reframeToCurrentPointCloud(animated: true)
    }

    // MARK: - Loading helpers
    private func reloadContentIfReady() {
        guard isViewLoaded else { return }

        if let renderer = renderer, renderer.currentPointCount > 0 {
            loadFromRenderer(renderer)
            return
        }

        if let scanData = scanData, scanData.lidarData.isEmpty == false {
            loadFromScanData(scanData)
            return
        }

        if let path = filePath, FileManager.default.fileExists(atPath: path) {
            loadFromFile(path)
            return
        }
    }

    private func loadFromRenderer(_ renderer: Renderer) {
        let pointCount = renderer.currentPointCount
        guard pointCount > 0 else {
            print("⚠️ Renderer 포인트가 없어 저장된 데이터를 확인합니다.")
            if let scanData = scanData {
                loadFromScanData(scanData)
            }
            return
        }

        let pointCloud = PointCloud()
        var filteredVertices: [PointCloudVertex] = []
        filteredVertices.reserveCapacity(pointCount)
        // 렌더러에서 이미 confidenceThreshold를 적용했으므로 추가 필터를 느슨하게 유지
        let minimumConfidence: Float = 0  // 렌더러에서 적용한 임계값 외에는 추가 필터 없음
        for index in 0..<pointCount {
            let particle = renderer.particlesBuffer[index]
            if particle.confidence < minimumConfidence { continue }
            // 유효한 포인트만 추가
            let pos = particle.position
            if pos.x.isFinite && pos.y.isFinite && pos.z.isFinite {
                filteredVertices.append(PointCloudVertex(
                    x: pos.x,
                    y: pos.y,
                    z: pos.z,
                    r: particle.color.x,
                    g: particle.color.y,
                    b: particle.color.z
                ))
            }
        }
        pointCloud.pointCloud = filteredVertices
        
        // recenter 다시 사용 - 포인트 클라우드를 시작 위치 기준으로 정렬하여 중복 제거
        if let referenceTransform = renderer.getStartCameraTransform() {
            pointCloud.recenter(using: referenceTransform)
            recenterTrajectory(using: referenceTransform)
        }
        
        // 렌더링 시 희석 문제를 막기 위해 클러스터링 제거

        let maxDistance = ScanSettings.shared.distanceLimit.distanceValue
        if maxDistance.isFinite {
            pointCloud.clampDistance(maxDistance: maxDistance)
        }

        render(pointCloud: pointCloud, reportedCount: pointCloud.pointCloud.count)
    }

    private func currentFrameStartTransform() -> simd_float4x4? {
        guard let renderer = renderer else { return nil }
        return renderer.arSession.currentFrame?.camera.transform
    }

    private func loadFromScanData(_ scanData: ScanData) {
        guard scanData.lidarData.isEmpty == false else {
            print("⚠️ ScanData에 lidarData가 없습니다.")
            return
        }
        let tempURL = FileManager.default.temporaryDirectory.appendingPathComponent("preview_temp.ply")
        do {
            try scanData.lidarData.write(to: tempURL, options: [])
            loadFromFile(tempURL.path, reportedCount: scanData.points, startTransform: scanData.startCameraTransform)
            if let ref = scanData.startCameraTransform {
                recenterTrajectory(using: ref)
            }
        } catch {
            print("❌ ScanData 로드 실패: \(error)")
        }
    }

    private func loadFromFile(_ path: String, reportedCount: Int? = nil, startTransform: simd_float4x4? = nil) {
        let pointCloud = PointCloud()
        pointCloud.load(file: path)
        // recenter 다시 사용 - 포인트 클라우드를 시작 위치 기준으로 정렬
        let referenceTransform = startTransform ?? loadStartTransformFromDisk(for: path)
        if let transform = referenceTransform {
            pointCloud.recenter(using: transform)
            recenterTrajectory(using: transform)
        }
        // 렌더링 시 희석 문제를 막기 위해 클러스터링 제거
        let maxDistance = ScanSettings.shared.distanceLimit.distanceValue
        if maxDistance.isFinite {
            pointCloud.clampDistance(maxDistance: maxDistance)
        }

        guard pointCloud.pointCloud.isEmpty == false else {
            print("❌ 파일에서 포인트를 불러오지 못했습니다.")
            return
        }
        render(pointCloud: pointCloud, reportedCount: pointCloud.pointCloud.count)
    }

    private func loadStartTransformFromDisk(for path: String) -> simd_float4x4? {
        let primaryURL = URL(fileURLWithPath: path + ".startTransform.json")
        if let transform = ScanStorage.decodeTransform(from: primaryURL) {
            return transform
        }
        let fallbackURL = URL(fileURLWithPath: path).deletingPathExtension().appendingPathExtension("startTransform.json")
        return ScanStorage.decodeTransform(from: fallbackURL)
    }

    private func render(pointCloud: PointCloud, reportedCount: Int) {
        guard reportedCount > 0 else {
            pointCloudNode?.removeFromParentNode()
            pointCloudNode = nil
            onPointCloudRendered?(0)
            return
        }
        pointCloudNode?.removeFromParentNode()

        currentPointCloud = pointCloud
        let node = pointCloud.getNode(useColor: true,
                                      colorMode: colorMode,
                                      pointSize: pointSize)
        pointCloudNode = node
        modelContainerNode.addChildNode(node)
        synchronizeTrajectoryNodeTransform()
        updateTrajectoryNode()

        reframeToNode(node, animated: false)
        onPointCloudRendered?(reportedCount)
    }

    private func reframeToNode(_ node: SCNNode, animated: Bool) {
        let boundingBox = node.boundingBox
        let scale = node.scale

        let center = SCNVector3(
            ((boundingBox.min.x + boundingBox.max.x) * 0.5) * scale.x,
            ((boundingBox.min.y + boundingBox.max.y) * 0.5) * scale.y,
            ((boundingBox.min.z + boundingBox.max.z) * 0.5) * scale.z
        )

        let size = SCNVector3(
            (boundingBox.max.x - boundingBox.min.x) * scale.x,
            (boundingBox.max.y - boundingBox.min.y) * scale.y,
            (boundingBox.max.z - boundingBox.min.z) * scale.z
        )

        let maxDimension = max(size.x, max(size.y, size.z))
        let diagonal = sqrtf(size.x * size.x + size.y * size.y + size.z * size.z)
        // 핸드폰을 들고 멀리서 바라보는 뷰를 가정하여 넉넉히 뒤로 물림
        let suggestedDistance = max(maxDimension * 3.0, max(diagonal * 1.5, 1.2))

        modelContainerNode.position = SCNVector3Zero
        orbitNode.position = center
        cameraState.target = SIMD3<Float>(Float(center.x), Float(center.y), Float(center.z))
        cameraState.distance = clampDistance(suggestedDistance)
        cameraState.yaw = 0
        cameraState.pitch = 0.6  // 더 높은 시점으로 내려다보는 각도

        if useAppleCameraControls {
            sceneView.defaultCameraController.target = center
            cameraNode.position = SCNVector3(center.x, center.y, center.z + suggestedDistance)
            sceneView.defaultCameraController.pointOfView = cameraNode
            return
        }

        applyCameraState(animated: animated)
    }

    // MARK: - ScanData observation
    private func attachScanDataObserver() {
        cancellables.removeAll()
        guard let scanData = scanData else { return }
        scanData.$lidarData
            .dropFirst()
            .receive(on: RunLoop.main)
            .sink { [weak self] data in
                guard let self = self, data.isEmpty == false else { return }
                print("🔁 ScanData lidarData 변경 감지: \(data.count) bytes")
                self.loadFromScanData(scanData)
            }
            .store(in: &cancellables)
        scanData.$trajectoryPoints
            .receive(on: RunLoop.main)
            .sink { [weak self] points in
                self?.trackedTrajectoryPoints = points
            }
            .store(in: &cancellables)
        trackedTrajectoryPoints = scanData.trajectoryPoints
    }

    // MARK: - Actions
    @objc private func closeButtonTapped() {
        onClose?()
    }
    
    private func updateCameraLookTarget() {
        let target = orbitNode.worldPosition
        let worldUp = SCNVector3(0, 1, 0)
        cameraNode.look(at: target, up: worldUp, localFront: SCNVector3(0, 0, -1))
    }

    private func applyCameraState(animated: Bool = false) {
        // 애플 기본 카메라 컨트롤을 사용할 때는 기본 컨트롤러 타깃만 갱신
        if useAppleCameraControls {
            let target = SCNVector3(cameraState.target)
            sceneView.defaultCameraController.target = target
            cameraNode.position = SCNVector3(target.x, target.y, target.z + cameraState.distance)
            sceneView.defaultCameraController.pointOfView = cameraNode
            return
        }

        let applyBlock = {
            self.orbitNode.position = SCNVector3(self.cameraState.target)
            self.orbitNode.eulerAngles = SCNVector3(self.cameraState.pitch,
                                                    self.cameraState.yaw,
                                                    0)
            self.cameraNode.position = SCNVector3(0, 0, self.cameraState.distance)
            self.updateCameraLookTarget()
        }
        if animated {
            SCNTransaction.begin()
            SCNTransaction.animationDuration = 0.18
            applyBlock()
            SCNTransaction.commit()
        } else {
            applyBlock()
        }
    }

    private func reframeToCurrentPointCloud(animated: Bool) {
        guard let node = pointCloudNode else { return }
        reframeToNode(node, animated: animated)
    }

    private func clampPitch(_ value: Float) -> Float {
        return max(-1.2, min(1.2, value))
    }

    private func clampDistance(_ value: Float) -> Float {
        return min(distanceRange.upperBound, max(distanceRange.lowerBound, value))
    }

    private func normalizeAngle(_ angle: Float) -> Float {
        var result = angle
        let twoPi = Float.pi * 2
        if result > twoPi || result < -twoPi {
            result.formTruncatingRemainder(dividingBy: twoPi)
        }
        return result
    }

    private func rebuildPointNode() {
        guard let pointCloud = currentPointCloud else { return }
        pointCloudNode?.removeFromParentNode()
        let node = pointCloud.getNode(useColor: true,
                                      colorMode: colorMode,
                                      pointSize: pointSize)
        pointCloudNode = node
        modelContainerNode.addChildNode(node)
        synchronizeTrajectoryNodeTransform()
        updateTrajectoryNode()
    }

    @objc private func handleColorModeChanged(_ sender: UISegmentedControl) {
        switch sender.selectedSegmentIndex {
        case 0: colorMode = .rgb
        case 1: colorMode = .intensity
        case 2: colorMode = .height
        default: colorMode = .mono
        }
        rebuildPointNode()
    }

    @objc private func handlePointSizeChanged(_ sender: UISlider) {
        pointSize = CGFloat(sender.value)
        rebuildPointNode()
    }

    private func synchronizeTrajectoryNodeTransform() {
        guard let pointNode = pointCloudNode else { return }
        trajectoryNode.transform = pointNode.transform
        trajectoryNode.pivot = pointNode.pivot
        trajectoryNode.scale = pointNode.scale
    }

    private func updateTrajectoryNode() {
        guard trackedTrajectoryPoints.count > 1 else {
            trajectoryNode.geometry = nil
            trajectoryNode.childNodes.forEach { $0.removeFromParentNode() }
            trajectoryNode.isHidden = true
            return
        }
        trajectoryNode.geometry = nil
        trajectoryNode.childNodes.forEach { $0.removeFromParentNode() }

        let neonPurple = UIColor(red: 0.75, green: 0.2, blue: 1.0, alpha: 1.0)
        let radius: CGFloat = 0.001  // 더 얇게
        var segmentCount = 0

        for i in 0..<(trackedTrajectoryPoints.count - 1) {
            let p0 = trackedTrajectoryPoints[i]
            let p1 = trackedTrajectoryPoints[i + 1]
            let start = SCNVector3(p0.x, p0.y, p0.z)
            let end = SCNVector3(p1.x, p1.y, p1.z)
            let length = CGFloat(simd_length(p1 - p0))
            guard length > 0 else { continue }

            let cylinder = SCNCylinder(radius: radius, height: length)
            let mat = SCNMaterial()
            mat.diffuse.contents = neonPurple
            mat.emission.contents = neonPurple
            mat.lightingModel = .constant
            cylinder.firstMaterial = mat

            let node = SCNNode(geometry: cylinder)
            node.position = SCNVector3(
                (start.x + end.x) * 0.5,
                (start.y + end.y) * 0.5,
                (start.z + end.z) * 0.5
            )
            node.look(at: end, up: SCNVector3(0,1,0), localFront: SCNVector3(0,1,0))
            node.eulerAngles.x += .pi / 2  // cylinder의 축을 방향에 맞춤
            trajectoryNode.addChildNode(node)
            segmentCount += 1
        }

        // 튜브가 하나도 안 만들어졌다면 fallback으로 라인 지오메트리 표시
        if segmentCount == 0 {
            let vertices = trackedTrajectoryPoints.map { SCNVector3($0.x, $0.y, $0.z) }
            var indices = [Int32]()
            indices.reserveCapacity((vertices.count - 1) * 2)
            for i in 0..<(vertices.count - 1) {
                indices.append(Int32(i))
                indices.append(Int32(i + 1))
            }
            let vertexSource = SCNGeometrySource(vertices: vertices)
            let indexData = Data(bytes: indices, count: indices.count * MemoryLayout<Int32>.size)
            let element = SCNGeometryElement(data: indexData,
                                             primitiveType: .line,
                                             primitiveCount: vertices.count - 1,
                                             bytesPerIndex: MemoryLayout<Int32>.size)
            let geometry = SCNGeometry(sources: [vertexSource], elements: [element])
            let material = SCNMaterial()
            material.diffuse.contents = neonPurple
            material.emission.contents = neonPurple
            material.isDoubleSided = true
            material.lightingModel = .constant
            geometry.firstMaterial = material
            geometry.setValue(1.0, forKey: "lineWidth")
            trajectoryNode.geometry = geometry
        }
        trajectoryNode.isHidden = false
        synchronizeTrajectoryNodeTransform()
    }

    private func recenterTrajectory(using startTransform: simd_float4x4) {
        let inv = startTransform.inverse
        trackedTrajectoryPoints = trackedTrajectoryPoints.map { point in
            let v = SIMD4<Float>(point.x, point.y, point.z, 1.0)
            let t = inv * v
            return SIMD3<Float>(t.x, t.y, t.z)
        }
    }

    private func loadTrajectoryFromFileIfAvailable(at path: String) {
        let primaryURL = URL(fileURLWithPath: path + ".trajectory.json")
        let fallbackURL = URL(fileURLWithPath: path).deletingPathExtension().appendingPathExtension("trajectory.json")
        let candidates = [primaryURL, fallbackURL]
        for url in candidates {
            if FileManager.default.fileExists(atPath: url.path) {
                if let data = try? Data(contentsOf: url),
                   let json = try? JSONSerialization.jsonObject(with: data, options: []) as? [[Double]] {
                    let points = json.compactMap { values -> SIMD3<Float>? in
                        guard values.count == 3 else { return nil }
                        return SIMD3<Float>(Float(values[0]), Float(values[1]), Float(values[2]))
                    }
                    trackedTrajectoryPoints = points
                    return
                }
            }
        }
        trackedTrajectoryPoints = []
    }
    
    @objc private func handleTrajectoryUpdate(_ notification: Notification) {
        guard renderer != nil,
              let rawPoints = notification.userInfo?["points"] as? [[Float]] else { return }
        let mapped = rawPoints.compactMap { values -> SIMD3<Float>? in
            guard values.count == 3 else { return nil }
            return SIMD3<Float>(values[0], values[1], values[2])
        }
        trackedTrajectoryPoints = mapped
    }

    /// 프리뷰 해제 시 호출하여 SceneKit/옵저버를 정리
    func teardown() {
        NotificationCenter.default.removeObserver(self)
        gestureHintTimer?.invalidate()
        gestureHintTimer = nil
        sceneView.gestureRecognizers?.forEach { sceneView.removeGestureRecognizer($0) }
        pointCloudNode?.removeFromParentNode()
        pointCloudNode = nil
        trajectoryNode.geometry = nil
        sceneView.scene = nil
        renderer = nil
        scanData = nil
    }
}

// MARK: - Gesture delegate
extension PointCloudSceneViewController: UIGestureRecognizerDelegate {
    func gestureRecognizer(_ gestureRecognizer: UIGestureRecognizer, shouldRecognizeSimultaneouslyWith otherGestureRecognizer: UIGestureRecognizer) -> Bool {
        let combos: [(UIGestureRecognizer, UIGestureRecognizer)] = [
            (translationPanGesture, pinchGestureRecognizer),
            (pinchGestureRecognizer, translationPanGesture),
            (rotationGestureRecognizer, pinchGestureRecognizer),
            (pinchGestureRecognizer, rotationGestureRecognizer),
            (rotationGestureRecognizer, translationPanGesture),
            (translationPanGesture, rotationGestureRecognizer)
        ]
        return combos.contains { $0.0 === gestureRecognizer && $0.1 === otherGestureRecognizer }
    }
}

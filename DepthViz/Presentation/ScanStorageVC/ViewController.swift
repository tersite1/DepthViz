// 포인트클라우드 뷰어 — 저장된 파일 보기용

import UIKit
import SceneKit
#if canImport(GoogleMobileAds)
import GoogleMobileAds
#endif

class ViewController: UIViewController {

    var sceneView: SCNView!
    var pointCloud: PointCloud?
    var pointCloudNode: SCNNode?

    private let showAds = ScanCountManager.shared.shouldShowInterstitialAd
    /// 배너 높이 (광고 있을 때 50pt, 없으면 0)
    private var bannerHeight: CGFloat { showAds ? 50 : 0 }

    // MARK: - UI Elements

    private let backButton: UIButton = {
        let button = UIButton(type: .custom)
        let config = UIImage.SymbolConfiguration(pointSize: 18, weight: .semibold)
        button.setImage(UIImage(systemName: "xmark", withConfiguration: config), for: .normal)
        button.tintColor = .white
        button.backgroundColor = UIColor(white: 0.15, alpha: 0.7)
        button.layer.cornerRadius = 20
        button.clipsToBounds = true
        button.translatesAutoresizingMaskIntoConstraints = false
        return button
    }()

    private let infoLabel: UILabel = {
        let label = UILabel()
        label.font = .systemFont(ofSize: 14, weight: .medium)
        label.textColor = .white
        label.textAlignment = .left
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

    // 포인트 크기 슬라이더
    private let pointSizeSlider: UISlider = {
        let slider = UISlider()
        slider.minimumValue = 1.0
        slider.maximumValue = 15.0
        slider.value = 1.5
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
        view.backgroundColor = .black
        setupSceneView()
        setupUI()
        if showAds { setupBannerAd() }
    }

    // MARK: - Setup

    func setupSceneView() {
        sceneView = SCNView(frame: view.bounds)
        sceneView.translatesAutoresizingMaskIntoConstraints = false
        sceneView.scene = SCNScene()
        sceneView.backgroundColor = .black
        sceneView.allowsCameraControl = true
        sceneView.defaultCameraController.interactionMode = .orbitTurntable
        sceneView.defaultCameraController.inertiaEnabled = true
        sceneView.antialiasingMode = .multisampling4X
        view.addSubview(sceneView)

        NSLayoutConstraint.activate([
            sceneView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            sceneView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            sceneView.topAnchor.constraint(equalTo: view.topAnchor),
            sceneView.bottomAnchor.constraint(equalTo: view.bottomAnchor)
        ])

        // 로딩 인디케이터
        view.addSubview(loadingIndicator)
        NSLayoutConstraint.activate([
            loadingIndicator.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            loadingIndicator.centerYAnchor.constraint(equalTo: view.centerYAnchor)
        ])
    }

    private func setupUI() {
        // 닫기 버튼
        view.addSubview(backButton)
        backButton.addTarget(self, action: #selector(closeButtonTapped), for: .touchUpInside)

        // 포인트 수 라벨
        view.addSubview(infoLabel)

        // 포인트 크기 슬라이더
        view.addSubview(pointSizeSmallDot)
        view.addSubview(pointSizeSlider)
        view.addSubview(pointSizeLargeDot)
        pointSizeSlider.addTarget(self, action: #selector(pointSizeChanged(_:)), for: .valueChanged)

        // 배너 광고가 있으면 UI를 배너 위로 올림
        let bottomOffset: CGFloat = showAds ? -(bannerHeight + 12) : -24

        NSLayoutConstraint.activate([
            // 닫기 버튼 (좌상단)
            backButton.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 20),
            backButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 8),
            backButton.widthAnchor.constraint(equalToConstant: 40),
            backButton.heightAnchor.constraint(equalToConstant: 40),

            // 포인트 수 (좌하단 — 배너 위)
            infoLabel.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 20),
            infoLabel.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: bottomOffset),

            // 포인트 크기 슬라이더 (하단 가로: ● ——— ⬤)
            pointSizeSmallDot.leadingAnchor.constraint(equalTo: infoLabel.trailingAnchor, constant: 20),
            pointSizeSmallDot.centerYAnchor.constraint(equalTo: infoLabel.centerYAnchor),

            pointSizeSlider.leadingAnchor.constraint(equalTo: pointSizeSmallDot.trailingAnchor, constant: 8),
            pointSizeSlider.trailingAnchor.constraint(equalTo: pointSizeLargeDot.leadingAnchor, constant: -8),
            pointSizeSlider.centerYAnchor.constraint(equalTo: infoLabel.centerYAnchor),

            pointSizeLargeDot.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -20),
            pointSizeLargeDot.centerYAnchor.constraint(equalTo: infoLabel.centerYAnchor)
        ])
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

        NSLayoutConstraint.activate([
            banner.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            banner.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor),
            banner.widthAnchor.constraint(equalToConstant: GADAdSizeBanner.size.width),
            banner.heightAnchor.constraint(equalToConstant: GADAdSizeBanner.size.height)
        ])

        banner.load(GADRequest())
        #endif
    }

    // MARK: - Load Point Cloud

    func READFILE(from filePath: String) {
        loadingIndicator.startAnimating()
        infoLabel.text = "Loading..."

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            let pc = PointCloud()
            pc.load(file: filePath, scale: 1.0)

            DispatchQueue.main.async {
                guard let self = self else { return }
                self.pointCloud = pc
                self.loadingIndicator.stopAnimating()

                guard pc.pointCloud.count > 0 else {
                    self.infoLabel.text = "No points"
                    return
                }

                let node = pc.getNode(useColor: true, colorMode: .rgb, pointSize: 1.5)
                self.sceneView.scene?.rootNode.addChildNode(node)
                self.pointCloudNode = node
                self.setupCamera(for: node)
                self.updateInfoLabel()
            }
        }
    }

    // MARK: - Camera Setup

    private func setupCamera(for node: SCNNode) {
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
        guard maxDimension > 0 else { return }

        let cameraDistance = Float(maxDimension) * 3.5

        let cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        cameraNode.camera?.zNear = 0.01
        cameraNode.camera?.zFar = Double(maxDimension) * 10
        cameraNode.camera?.fieldOfView = 60

        let dx = center.x
        let dy = center.y
        let dz = center.z
        let distToCenter = sqrt(dx * dx + dy * dy + dz * dz)
        if distToCenter > 0.01 {
            let scale = cameraDistance / distToCenter
            cameraNode.position = SCNVector3(
                center.x - dx * scale,
                center.y - dy * scale + cameraDistance * 0.1,
                center.z - dz * scale
            )
        } else {
            cameraNode.position = SCNVector3(center.x, center.y + cameraDistance * 0.2, center.z + cameraDistance)
        }
        cameraNode.look(at: center, up: SCNVector3(0, 1, 0), localFront: SCNVector3(0, 0, -1))
        sceneView.scene?.rootNode.addChildNode(cameraNode)

        sceneView.pointOfView = cameraNode
        sceneView.defaultCameraController.target = center
        sceneView.defaultCameraController.worldUp = SCNVector3(0, 1, 0)
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

    // MARK: - Info Label

    private func updateInfoLabel() {
        let count = pointCloud?.pointCloud.count ?? 0
        let formatter = NumberFormatter()
        formatter.numberStyle = .decimal
        let pointsString = formatter.string(from: NSNumber(value: count)) ?? "\(count)"
        infoLabel.text = "\(pointsString) Points"
    }

    // MARK: - Actions

    @objc func closeButtonTapped() {
        dismiss(animated: true)
    }
}

//
//  MainVC.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//

import UIKit
import Metal
import MetalKit
import ARKit
import Combine
import CoreLocation
import CoreMotion
import SwiftUI
import MapKit
#if canImport(GoogleMobileAds)
import GoogleMobileAds
#endif
import AVFoundation

/// 메인화면의 UI 및 UX 담당
final class MainVC: UIViewController, ARSessionDelegate, CLLocationManagerDelegate, ScanPreviewDelegate {
    /// 기록측정 및 종료 버튼
    private let recordingButton = RecordingButton()
    /// 현재 동작상태 표시 텍스트
    private let statusLabel = StatusIndicatorLabel()
    /// 최적화/로딩 시 스피너
    private let processingSpinner: UIActivityIndicatorView = {
        let spinner = UIActivityIndicatorView(style: .large)
        spinner.color = .white
        spinner.hidesWhenStopped = true
        spinner.translatesAutoresizingMaskIntoConstraints = false
        return spinner
    }()
    /// 현재 측정중인 Point Cloud 개수 표시 뷰
    private let pointCloudCountView = PointCloudCountView()
    /// 측정이력창 표시 버튼
    private let scansButton = ScansButton()
    /// 마커 표시 버튼
    private let markerButton = MarkerButton()
    /// 현재 선택된 마커 (스캔 시 프로젝트 자동 생성용)
    private var currentMarker: LocationMarker?
    /// 설정 버튼
    private let settingsButton: UIButton = {
        let button = UIButton(type: .system)
        button.setImage(UIImage(systemName: "gearshape.fill"), for: .normal)
        button.tintColor = .white
        button.backgroundColor = UIColor.systemGray.withAlphaComponent(0.8)
        button.layer.cornerRadius = 25
        button.translatesAutoresizingMaskIntoConstraints = false
        return button
    }()
    /// Point Cloud 표시를 위한 Session
    private let session = ARSession()
    /// gps 측정을 위한 객체
    private var locationManager = CLLocationManager()
    /// IMU 측정을 위한 객체 (DV-SLAM ESKF용 고주파 IMU 데이터)
    private let motionManager = CMMotionManager()
    /// 메인화면과 관련된 로직담당 객체
    private var viewModel: MainVM?
    private var cancellables: Set<AnyCancellable> = []
    
    /// AR 가이드 레이블
    private let arGuideLabel: UILabel = {
        let label = UILabel()
        label.text = "Move iPhone to Start"
        label.font = .systemFont(ofSize: 24, weight: .bold)
        label.textColor = .white
        label.textAlignment = .center
        label.backgroundColor = UIColor.black.withAlphaComponent(0.7)
        label.layer.cornerRadius = 12
        label.clipsToBounds = true
        label.translatesAutoresizingMaskIntoConstraints = false
        label.alpha = 1  // 처음부터 표시
        return label
    }()
    
    /// IMU 오버레이 (우측 상단) — 프리미엄 기능
    private let imuOverlayStack: UIStackView = {
        let label1 = UILabel()
        let label2 = UILabel()
        let label3 = UILabel()
        for l in [label1, label2, label3] {
            l.font = .monospacedSystemFont(ofSize: 11, weight: .medium)
            l.textColor = .white
            l.textAlignment = .right
        }
        let stack = UIStackView(arrangedSubviews: [label1, label2, label3])
        stack.axis = .vertical
        stack.spacing = 2
        stack.translatesAutoresizingMaskIntoConstraints = false
        stack.alpha = 0
        return stack
    }()

    /// 카메라 미리보기 PiP (프리미엄 + 동영상 페어)
    private var cameraPiPView: UIView?
    private var cameraPiPLayer: AVCaptureVideoPreviewLayer?
    private var videoWriter: AVAssetWriter?
    private var videoWriterInput: AVAssetWriterInput?
    private var videoAdaptor: AVAssetWriterInputPixelBufferAdaptor?
    private var videoStartTime: CMTime?
    private var videoOutputURL: URL?

    /// 마지막 카메라 움직임 시간
    private var lastMovementTime: Date?
    /// 움직임 감지 타이머
    private var movementCheckTimer: Timer?
    /// 이전 카메라 위치
    private var previousCameraTransform: simd_float4x4?
    
    /// MTKView를 프로그래밍 방식으로 생성
    override func loadView() {
        print("🔧 loadView 시작 - MTKView 생성")
        let mtkView = MTKView(frame: UIScreen.main.bounds)
        mtkView.backgroundColor = .clear
        mtkView.isPaused = false  // 자동 렌더링 활성화
        mtkView.enableSetNeedsDisplay = false  // CADisplayLink 기반 자동 렌더링 사용
        mtkView.preferredFramesPerSecond = 60  // 60 FPS로 렌더링
        self.view = mtkView
        print("✅ MTKView 생성 완료 (자동 렌더링 활성화)")
    }
    
    /// MainVC 최초 접근시 configure
    override func viewDidLoad() {
        super.viewDidLoad()
        print("🚀🚀🚀 MainVC viewDidLoad 시작 🚀🚀🚀")

        self.configureUI()
        print("✅ configureUI 완료")
        self.configureViewModel()
        print("✅ configureViewModel 완료 - viewModel: \(self.viewModel != nil ? "존재" : "nil")")
        self.checkLidarSensor()
        print("✅ checkLidarSensor 완료 - mode: \(String(describing: self.viewModel?.mode))")
        self.configureLocationManager()
        print("✅ configureLocationManager 완료")
        self.bindViewModel()
        print("✅ bindViewModel 완료")
        
        // 위치 기반 스캔 시작 노티피케이션 등록
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(handleScanAtLocation(_:)),
            name: NSNotification.Name("StartScanAtLocation"),
            object: nil
        )
        
        // 마커 프로젝트 표시 노티피케이션 등록
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(handleShowMarkerProject(_:)),
            name: NSNotification.Name("ShowMarkerProject"),
            object: nil
        )
        
        // 보상형 전면 광고 미리 로드 (20회 이상 미구매 대비, DEBUG에서는 스킵)
        #if !DEBUG
        Task { await InterstitialAdManager.shared.loadAd() }
        #endif

        print("🎉 MainVC viewDidLoad 완료 🎉")
    }

    deinit {
        NotificationCenter.default.removeObserver(self)
    }
    
    /// 특정 위치에서 스캔 시작
    @objc private func handleScanAtLocation(_ notification: Notification) {
        guard let marker = notification.object as? LocationMarker else {
            print("⚠️ 마커 정보를 받지 못했습니다")
            return
        }
        
        print("🎯 \(marker.name)에서 스캔 시작")
        
        // 마커 정보 저장
        self.currentMarker = marker
        
        // 스캔 시작 (ready 상태에서만 가능)
        if viewModel?.mode == .ready {
            recordingButton.sendActions(for: UIControl.Event.touchUpInside)
            
            // 사용자에게 알림 표시
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) { [weak self] in
                let alert = UIAlertController(
                    title: "📍 \(marker.name)",
                    message: "Scanning started at this location.\nData will be saved to '\(marker.name)' project.",
                    preferredStyle: .alert
                )
                alert.addAction(UIAlertAction(title: "OK", style: .default))
                self?.present(alert, animated: true)
            }
        } else {
            print("⚠️ 현재 스캔을 시작할 수 없는 상태입니다: \(String(describing: viewModel?.mode))")
        }
    }
    
    /// 마커 프로젝트 표시
    @objc private func handleShowMarkerProject(_ notification: Notification) {
        guard let marker = notification.object as? LocationMarker else {
            print("⚠️ 마커 정보를 받지 못했습니다")
            return
        }
        
        print("📊 \(marker.name) 프로젝트 표시")
        
        // ScanStorageVC 화면으로 이동
        let scanStorageVC = ScanStorageVC()
        self.navigationController?.pushViewController(scanStorageVC, animated: true)
    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        // viewWillAppear에서 이미 AR 세션 + MTKView 초기화 완료
        // 중복 호출 제거 (AR 세션 3중 리셋 방지)
    }

    /// 보상 알림 표시 (제거됨 - 광고 기능 비활성화)
    private func showRewardAlert_removed(amount: Int) {
        let alert = UIAlertController(
            title: "🎉 Reward Earned!",
            message: "Ad viewing complete! You earned \(amount) points.",
            preferredStyle: .alert
        )
        alert.addAction(UIAlertAction(title: "OK", style: .default))
        self.present(alert, animated: true)
    }
    
     /// MainVC 화면 진입시 필요한 설정들
     override func viewWillAppear(_ animated: Bool) {
         super.viewWillAppear(animated)

         // LiDAR 없는 기기: ARSession 시작하지 않음
         guard ARWorldTrackingConfiguration.supportsFrameSemantics([.sceneDepth, .smoothedSceneDepth]) else {
             self.viewModel?.cantRecording()
             self.navigationController?.setNavigationBarHidden(true, animated: true)
             return
         }

         // 녹화 중이면 초기화하지 않음
         guard viewModel?.mode != .recording else {
             self.navigationController?.setNavigationBarHidden(true, animated: true)
             startMovementCheckTimer()
             return
         }

         // 1️⃣ MTKView 즉시 정지 (clearParticles 중 draw() 방지)
         if let mtkView = self.view as? MTKView {
             mtkView.isPaused = true
         }

         // 2️⃣ SLAM 엔진 완전 정지
         SLAMService.sharedInstance().stop()

         // 3️⃣ 렌더러 완전 정리 (commandQueue 재생성 포함 — GPU 에러 복구)
         self.viewModel?.resetRenderer()

         // 4️⃣ renderDestination을 MainVC의 MTKView로 복원
         if let mtkView = self.view as? MTKView {
             self.viewModel?.renderer.renderDestination = mtkView
             self.viewModel?.renderer.drawRectResized(size: mtkView.bounds.size)
         }

         // 5️⃣ 뷰 모델 상태 초기화
         self.viewModel?.changeMode(to: .ready)

         // 6️⃣ UI 초기화
         self.navigationController?.setNavigationBarHidden(true, animated: true)
         self.pointCloudCountView.updateCount(to: 0)
         self.pointCloudCountView.alpha = 1

         // 7️⃣ ARSession delegate 복원 + 재시작
         self.session.delegate = self
         self.configureARWorldTracking()

         // 8️⃣ MTKView 재활성화 (ARSession 재시작 후)
         if let mtkView = self.view as? MTKView {
             mtkView.isPaused = false
             mtkView.enableSetNeedsDisplay = false
         }

         // 9️⃣ 타이머 시작
         startMovementCheckTimer()

         // 배너 광고는 프리뷰 화면(ScanPreviewVC)에서만 표시
     }
    
    /// 다른화면으로 전환시 ARSession 일시정지한다
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        // MTKView 즉시 정지 (다른 화면에서 Metal 충돌 방지)
        if let mtkView = self.view as? MTKView {
            mtkView.isPaused = true
        }
        // 녹화 중이면 AR session 유지 (pause하지 않음)
        if self.viewModel?.mode != .recording {
            self.session.pause()
        }
        // 타이머 정지
        stopMovementCheckTimer()
    }
    
    /// 앱이 메모리경고를 수신받는 경우
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // 현재까지 측정중인 파일을 앱 내부로 임시저장 후 앱을 종료한다
        self.viewModel?.terminateRecording()
    }
    
}

// MARK: HomeBar & StatusBar Hidden
extension MainVC {
    // Auto-hide the home indicator to maximize immersion in AR experiences.
    override var prefersHomeIndicatorAutoHidden: Bool {
        return true
    }
    
    // Hide the status bar to maximize immersion in AR experiences.
    override var prefersStatusBarHidden: Bool {
        return true
    }
}

// MARK: - AR 관련 함수들
extension MainVC {
    /// LiDAR 센서 사용가능여부 확인 함수
    private func checkLidarSensor() {
        if !ARWorldTrackingConfiguration.supportsFrameSemantics([.sceneDepth, .smoothedSceneDepth]) {
            self.viewModel?.cantRecording()
        }
    }
    
    /// AR로 표시하기 위한 MetalKitView를 설정하는 부분 및 viewModel 설정
    private func configureViewModel() {
        print("🔧 configureViewModel 시작")
        print("🔧 현재 view 타입: \(type(of: self.view))")
        
        // Metal 디바이스 생성
        guard let device = MTLCreateSystemDefaultDevice() else {
            print("❌ Metal is not supported on this device")
            return
        }
        print("✅ Metal 디바이스 생성 완료")
        
        // Metal Object를 표시하기 위한 MetalKitView로 설정
        guard let mtkView = self.view as? MTKView else {
            print("❌❌❌ CRITICAL: view를 MTKView로 캐스팅 실패!")
            print("❌ Storyboard에서 MainVC의 view 클래스가 MTKView로 설정되어 있는지 확인하세요!")
            return
        }
        
        print("✅ MTKView 캐스팅 성공")
        
        // MetalKitView에 표시하기 위한 Metal 디바이스 설정
        mtkView.device = device
        mtkView.backgroundColor = .black  // 검은 배경 (카메라 피드가 로드되기 전)
        mtkView.clearColor = MTLClearColor(red: 0.0, green: 0.0, blue: 0.0, alpha: 1.0)  // Metal clear color
        // MetalKitView의 depth 크기 설정
        mtkView.depthStencilPixelFormat = .depth32Float
        // 논리적 좌표공간(logical coordinate)(단위: points)과 장치 좌표공간(device coordinate)(단위: pixels)간의 스케일링 값
        // 1로 설정한 경우 실제좌표계와 MTKView에서 표시되는 좌표계와 동일하게 설정한다 (실제를 그대로 아이폰에서 표시하는 경우)
        mtkView.contentScaleFactor = 1
        // MetalKitView의 내용을 업데이트하고자 하는 경우 호출하기 위한 delegate 설정
        mtkView.delegate = self
        print("✅ MTKView 설정 완료 (clearColor: 검은색, delegate 설정됨)")
        
        // Configure the ViewModel, Renderer to draw to the view
        self.viewModel = MainVM(session: self.session, device: device, view: mtkView)
        self.viewModel?.renderer.delegate = self
        print("✅ MainVM 생성 완료: \(self.viewModel != nil)")
        
        // MTKView 강제 활성화 (카메라 피드 표시를 위해)
        mtkView.isPaused = false
        mtkView.enableSetNeedsDisplay = true
        mtkView.preferredFramesPerSecond = 60  // 60fps 강제 설정
        mtkView.colorPixelFormat = .bgra8Unorm  // 픽셀 포맷 명시적 설정
        
        // 강제로 렌더링 시작
        mtkView.setNeedsDisplay()
        
        print("✅ MTKView 활성화: isPaused=\(mtkView.isPaused), enableSetNeedsDisplay=\(mtkView.enableSetNeedsDisplay), fps=\(mtkView.preferredFramesPerSecond)")
        print("🔄 MTKView setNeedsDisplay() 호출됨")
    }
    
    /// LiDAR 측정을 위한 ARSession 활성화 및 Configure 설정 부분
    private func configureARWorldTracking() {
        print("📷 configureARWorldTracking 시작")
        // LiDAR 없는 기기에서 sceneDepth 요청 시 크래시 방지
        guard ARWorldTrackingConfiguration.supportsFrameSemantics([.sceneDepth, .smoothedSceneDepth]) else {
            print("⚠️ sceneDepth 미지원 기기 - ARSession 시작 안 함")
            self.viewModel?.cantRecording()
            return
        }
        guard self.viewModel?.mode != .cantRecord else {
            print("⚠️ cantRecord 모드 - ARSession 시작 안 함")
            return
        }
        
        // Create a world-tracking configuration, and enable the scene depth frame-semantic.
        // 디바이스(iPhone)의 움직임을 추적하기 위한 Configuration 값 (움직이는대로 그대로 AR로 표시하기 위함)
        let configuration = ARWorldTrackingConfiguration()
        // 카메라를 통해 보이는 실제 객체까지의 거리, 여러 프레임의 평균 거리값을 제공하도록 설정
        configuration.frameSemantics = [.sceneDepth, .smoothedSceneDepth]

        // Set the session delegate
        self.session.delegate = self
        
        print("📷 ARSession.run() 호출 중...")
        // Run the view's session
        self.session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        print("✅ ARSession.run() 호출 완료")

        // AR 코칭 오버레이 (첫 화면 3D 가이드)
        setupCoachingOverlay()

        // 사용자 설정 적용 (ARSession 시작 후)
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.15) { [weak self] in
            self?.viewModel?.renderer.applySettings()
            print("✅ 사용자 설정 적용 완료")
        }

        // The screen shouldn't dim during AR experiences.
        UIApplication.shared.isIdleTimerDisabled = true
    }

    private func setupCoachingOverlay() {
        // 이미 추가되어 있으면 스킵
        if view.subviews.contains(where: { $0 is ARCoachingOverlayView }) { return }

        let coachingOverlay = ARCoachingOverlayView()
        coachingOverlay.session = self.session
        coachingOverlay.goal = .tracking
        coachingOverlay.activatesAutomatically = true
        coachingOverlay.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(coachingOverlay)

        NSLayoutConstraint.activate([
            coachingOverlay.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            coachingOverlay.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            coachingOverlay.topAnchor.constraint(equalTo: view.topAnchor),
            coachingOverlay.bottomAnchor.constraint(equalTo: view.bottomAnchor)
        ])
    }
}

// MARK: AR 표출을 위한 MTKView Delegate
extension MainVC: MTKViewDelegate {
    // Called whenever view changes orientation or layout is changed
    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        self.viewModel?.rendererResize(to: size)
    }
    
    func draw(in view: MTKView) {
        self.viewModel?.rendererDraw()
    }
    
}

// MARK: RenderDestinationProvider
protocol RenderDestinationProvider {
    var currentRenderPassDescriptor: MTLRenderPassDescriptor? { get }
    var currentDrawable: CAMetalDrawable? { get }
    var colorPixelFormat: MTLPixelFormat { get set }
    var depthStencilPixelFormat: MTLPixelFormat { get set }
    var sampleCount: Int { get set }
}

extension MTKView: RenderDestinationProvider { }

// MARK: - MarkerButton
final class MarkerButton: UIButton {
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.backgroundColor = .systemOrange
        self.setTitle("MARKER", for: .normal)
        self.titleLabel?.font = UIFont.systemFont(ofSize: 14, weight: .semibold)
        self.titleLabel?.textColor = .white
        self.layer.cornerRadius = 12
        self.layer.cornerCurve = .continuous
        
        NSLayoutConstraint.activate([
            self.heightAnchor.constraint(equalToConstant: 36),
            self.widthAnchor.constraint(equalToConstant: 85)
        ])
    }
}

// LocationMarker is defined in Domain/Entity/LocationMarker.swift
// MarkerStorage is defined in Domain/Model/MarkerStorage.swift

// MARK: - Camera PiP + Video Recording (Premium)
extension MainVC {
    func showCameraPiP() {
        guard PremiumManager.shared.isPremium && PremiumManager.shared.saveVideo else { return }
        guard cameraPiPView == nil else { return }

        let pipSize = CGSize(width: 120, height: 160)
        let pip = UIView(frame: CGRect(
            x: 16,
            y: view.safeAreaLayoutGuide.layoutFrame.maxY - pipSize.height - 80,
            width: pipSize.width,
            height: pipSize.height
        ))
        pip.backgroundColor = .black
        pip.layer.cornerRadius = 10
        pip.clipsToBounds = true
        pip.layer.borderWidth = 1.5
        pip.layer.borderColor = UIColor.white.withAlphaComponent(0.3).cgColor
        pip.alpha = 0
        view.addSubview(pip)

        // "REC" 인디케이터
        let recDot = UIView(frame: CGRect(x: 8, y: 8, width: 8, height: 8))
        recDot.backgroundColor = .red
        recDot.layer.cornerRadius = 4
        pip.addSubview(recDot)

        let recLabel = UILabel(frame: CGRect(x: 20, y: 4, width: 40, height: 16))
        recLabel.text = "REC"
        recLabel.font = .systemFont(ofSize: 10, weight: .bold)
        recLabel.textColor = .red
        pip.addSubview(recLabel)

        cameraPiPView = pip
        UIView.animate(withDuration: 0.3) { pip.alpha = 1 }
    }

    func hideCameraPiP() {
        guard let pip = cameraPiPView else { return }
        UIView.animate(withDuration: 0.3, animations: { pip.alpha = 0 }) { _ in
            pip.removeFromSuperview()
        }
        cameraPiPView = nil
        cameraPiPLayer = nil
    }

    /// ARFrame에서 카메라 이미지를 PiP에 표시 + 동영상 프레임 기록
    func updateCameraPiP(with frame: ARFrame) {
        guard PremiumManager.shared.isPremium && PremiumManager.shared.saveVideo else { return }

        // PiP 업데이트 (30fps 간격, 3프레임마다)
        let pixelBuffer = frame.capturedImage

        // PiP 이미지 업데이트 (10fps)
        if let pip = cameraPiPView, imuPiPFrameCount % 6 == 0 {
            let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
            let context = CIContext()
            if let cgImage = context.createCGImage(ciImage, from: ciImage.extent) {
                let uiImage = UIImage(cgImage: cgImage, scale: 1, orientation: .right)
                DispatchQueue.main.async {
                    if let existing = pip.subviews.compactMap({ $0 as? UIImageView }).first {
                        existing.image = uiImage
                    } else {
                        let iv = UIImageView(frame: pip.bounds)
                        iv.contentMode = .scaleAspectFill
                        iv.clipsToBounds = true
                        iv.image = uiImage
                        pip.insertSubview(iv, at: 0)
                    }
                }
            }
        }
        imuPiPFrameCount += 1

        // 동영상 프레임 기록
        writeVideoFrame(pixelBuffer: pixelBuffer, time: frame.timestamp)
    }

    private var imuPiPFrameCount: Int {
        get { objc_getAssociatedObject(self, &pipFrameCountKey) as? Int ?? 0 }
        set { objc_setAssociatedObject(self, &pipFrameCountKey, newValue, .OBJC_ASSOCIATION_RETAIN_NONATOMIC) }
    }

    func startVideoRecording() {
        guard PremiumManager.shared.isPremium && PremiumManager.shared.saveVideo else { return }

        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyy_MMdd_HHmmss"
        let timestamp = dateFormatter.string(from: Date())
        let fileName = "\(timestamp)_Scan.mp4"
        let url = ScanStorage.shared.exportRoot.appendingPathComponent(fileName)

        // 이전 파일 정리
        try? FileManager.default.removeItem(at: url)

        do {
            let writer = try AVAssetWriter(outputURL: url, fileType: .mp4)
            let settings: [String: Any] = [
                AVVideoCodecKey: AVVideoCodecType.h264,
                AVVideoWidthKey: 1920,
                AVVideoHeightKey: 1440,
                AVVideoCompressionPropertiesKey: [
                    AVVideoAverageBitRateKey: 6_000_000
                ]
            ]
            let input = AVAssetWriterInput(mediaType: .video, outputSettings: settings)
            input.expectsMediaDataInRealTime = true
            input.transform = CGAffineTransform(rotationAngle: .pi / 2) // Portrait

            let adaptor = AVAssetWriterInputPixelBufferAdaptor(
                assetWriterInput: input,
                sourcePixelBufferAttributes: [
                    kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA
                ]
            )

            writer.add(input)
            writer.startWriting()

            self.videoWriter = writer
            self.videoWriterInput = input
            self.videoAdaptor = adaptor
            self.videoStartTime = nil
            self.videoOutputURL = url
            self.imuPiPFrameCount = 0

            print("🎥 동영상 녹화 시작: \(fileName)")
        } catch {
            print("❌ 동영상 녹화 초기화 실패: \(error)")
        }
    }

    func writeVideoFrame(pixelBuffer: CVPixelBuffer, time: TimeInterval) {
        guard let writer = videoWriter, writer.status == .writing,
              let input = videoWriterInput, input.isReadyForMoreMediaData else { return }

        let cmTime = CMTime(seconds: time, preferredTimescale: 600)

        if videoStartTime == nil {
            videoStartTime = cmTime
            writer.startSession(atSourceTime: .zero)
        }

        let presentationTime = CMTimeSubtract(cmTime, videoStartTime!)

        // 15fps로 제한 (매 4번째 프레임만 기록, 60fps ARSession 기준)
        guard imuPiPFrameCount % 4 == 0 else { return }

        videoAdaptor?.append(pixelBuffer, withPresentationTime: presentationTime)
    }

    func stopVideoRecording() {
        guard let writer = videoWriter, writer.status == .writing else {
            videoWriter = nil
            return
        }

        videoWriterInput?.markAsFinished()
        writer.finishWriting { [weak self] in
            if writer.status == .completed {
                print("✅ 동영상 저장 완료: \(self?.videoOutputURL?.lastPathComponent ?? "")")
            } else {
                print("❌ 동영상 저장 실패: \(writer.error?.localizedDescription ?? "")")
            }
            self?.videoWriter = nil
            self?.videoWriterInput = nil
            self?.videoAdaptor = nil
            self?.videoStartTime = nil
        }
    }
}

private var pipFrameCountKey: UInt8 = 0

// MARK: - Configure
extension MainVC {
    /// MainVC 표시할 UI 설정
    private func configureUI() {
        print("🔧 configureUI 시작")
        // recordingButton
        self.recordingButton.addTarget(self, action: #selector(tapRecordingButton), for: .touchUpInside)
        print("🔧 녹화 버튼 액션 연결 완료: tapRecordingButton")
        self.view.addSubview(self.recordingButton)
        NSLayoutConstraint.activate([
            self.recordingButton.centerXAnchor.constraint(equalTo: self.view.centerXAnchor),
            self.recordingButton.bottomAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.bottomAnchor, constant: -12)
        ])
        
        // statusLabel
        self.view.addSubview(self.statusLabel)
        NSLayoutConstraint.activate([
            self.statusLabel.centerXAnchor.constraint(equalTo: self.view.centerXAnchor),
            self.statusLabel.topAnchor.constraint(equalTo: self.recordingButton.topAnchor, constant: -60)
        ])

        // 최적화/로딩 스피너 (statusLabel 바로 위)
        self.view.addSubview(self.processingSpinner)
        NSLayoutConstraint.activate([
            self.processingSpinner.centerXAnchor.constraint(equalTo: self.view.centerXAnchor),
            self.processingSpinner.bottomAnchor.constraint(equalTo: self.statusLabel.topAnchor, constant: -12)
        ])
        
        // pointCloudCountView
        self.view.addSubview(self.pointCloudCountView)
        NSLayoutConstraint.activate([
            self.pointCloudCountView.centerXAnchor.constraint(equalTo: self.view.centerXAnchor),
            self.pointCloudCountView.topAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.topAnchor, constant: 12)
        ])
        
        // scansButton (DATA 버튼 - 왼쪽)
        self.scansButton.addAction(UIAction(handler: { [weak self] _ in
            self?.moveToScanStorageVC()
        }), for: .touchUpInside)
        self.view.addSubview(self.scansButton)
        NSLayoutConstraint.activate([
            self.scansButton.leadingAnchor.constraint(equalTo: self.view.leadingAnchor, constant: 32),
            self.scansButton.centerYAnchor.constraint(equalTo: self.recordingButton.centerYAnchor)
        ])
        
        // markerButton (MARKER 버튼 - 오른쪽)
        self.markerButton.addAction(UIAction(handler: { [weak self] _ in
            self?.showMapView()
        }), for: .touchUpInside)
        self.view.addSubview(self.markerButton)
        NSLayoutConstraint.activate([
            self.markerButton.trailingAnchor.constraint(equalTo: self.view.trailingAnchor, constant: -32),
            self.markerButton.centerYAnchor.constraint(equalTo: self.recordingButton.centerYAnchor)
        ])
        
        // settingsButton (설정 버튼 - 좌측 상단)
        self.settingsButton.addTarget(self, action: #selector(tapSettingsButton), for: .touchUpInside)
        self.view.addSubview(self.settingsButton)
        NSLayoutConstraint.activate([
            self.settingsButton.leadingAnchor.constraint(equalTo: self.view.leadingAnchor, constant: 20),
            self.settingsButton.topAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.topAnchor, constant: 20),
            self.settingsButton.widthAnchor.constraint(equalToConstant: 50),
            self.settingsButton.heightAnchor.constraint(equalToConstant: 50)
        ])
        
        // IMU 오버레이 (우측 상단 — 프리미엄)
        self.view.addSubview(self.imuOverlayStack)
        NSLayoutConstraint.activate([
            self.imuOverlayStack.trailingAnchor.constraint(equalTo: self.view.trailingAnchor, constant: -16),
            self.imuOverlayStack.topAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.topAnchor, constant: 80)
        ])

        // arGuideLabel (AR 가이드 - 중앙)
        self.view.addSubview(self.arGuideLabel)
        NSLayoutConstraint.activate([
            self.arGuideLabel.centerXAnchor.constraint(equalTo: self.view.centerXAnchor),
            self.arGuideLabel.centerYAnchor.constraint(equalTo: self.view.centerYAnchor),
            self.arGuideLabel.widthAnchor.constraint(greaterThanOrEqualToConstant: 280),
            self.arGuideLabel.heightAnchor.constraint(equalToConstant: 60)
        ])
}
     
    
    /// gps 값 수신을 위한 설정 함수
    private func configureLocationManager() {
        guard self.viewModel?.mode != .cantRecord else { return }
        self.locationManager.delegate = self
        self.locationManager.requestWhenInUseAuthorization()
    }
}

// MARK: INPUT (Binding Data)
extension MainVC {
    
    func OresetScanningState() {
        resetToInitialState()
    }
    /// viewModel 에서 값 변화를 수신하기 위한 함수
    private func bindViewModel() {
        self.bindMode()
        self.bindPointCount()
        self.bindLidarData()
        self.bindUploadSuccess()
        self.bindUploadProgress()
        self.bindSaveToStorageSuccess()

        self.bindScanData()
        self.bindSaveScanDataSuccess()
        self.bindIMUData()
    }
    
    /// viewModel 의 mode 값 변화를 수신하기 위한 함수
    private func bindMode() {
        self.viewModel?.$mode
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] mode in
                switch mode {
                case .ready:
                    self?.locationManager.stopUpdatingLocation()
                    self?.stopIMUUpdates()
                    self?.recordingButton.changeStatus(to: .ready)
                    self?.scansButton.fadeIn()
                    self?.scansButton.isUserInteractionEnabled = true
                    self?.markerButton.fadeIn()
                    self?.markerButton.isUserInteractionEnabled = true
                    self?.settingsButton.isEnabled = true
                    self?.settingsButton.alpha = 1.0
                    self?.imuOverlayStack.alpha = 0
                    self?.viewModel?.rendererDraw()
                case .recording:
                    self?.locationManager.startUpdatingLocation()
                    self?.startIMUUpdates()
                    self?.recordingButton.changeStatus(to: .recording)
                    self?.scansButton.fadeOut()
                    self?.scansButton.isUserInteractionEnabled = false
                    self?.markerButton.fadeOut()
                    self?.markerButton.isUserInteractionEnabled = false
                    self?.settingsButton.isEnabled = false
                    self?.settingsButton.alpha = 0.4
                    // 프리미엄: 카메라 PiP + 동영상 녹화 시작
                    self?.showCameraPiP()
                    self?.startVideoRecording()
                case .loading:
                    self?.locationManager.stopUpdatingLocation()
                    self?.stopIMUUpdates()
                    self?.recordingButton.changeStatus(to: .loading)
                    self?.scansButton.disappear()
                    self?.markerButton.disappear()
                    self?.settingsButton.isEnabled = false
                    self?.settingsButton.alpha = 0.4
                    self?.hideCameraPiP()
                    self?.stopVideoRecording()
                case .cantRecord:
                    self?.locationManager.stopUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .cantRecording)
                    self?.scansButton.fadeIn()
                    self?.markerButton.fadeIn()
                case .recordingTerminate:
                    // DV-SLAM 최적화 진행 중 — loading과 동일한 UI
                    self?.locationManager.stopUpdatingLocation()
                    self?.stopIMUUpdates()
                    self?.recordingButton.changeStatus(to: .loading)
                    self?.scansButton.disappear()
                    self?.markerButton.disappear()
                    self?.settingsButton.isEnabled = false
                    self?.settingsButton.alpha = 0.4
                    self?.imuOverlayStack.alpha = 0
                    self?.hideCameraPiP()
                    self?.stopVideoRecording()
                case .cantGetGPS:
                    self?.locationManager.stopUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .cantRecording)
                    self?.scansButton.fadeIn()
                    self?.markerButton.fadeIn()
                default:
                    self?.locationManager.stopUpdatingLocation()
                }
                
                self?.changeStatusLabel(to: mode)
            })
            .store(in: &self.cancellables)
    }
    
    /// viewModel 의 pointCount 값 변화를 수신하여 표시하기 위한 함수
    private func bindPointCount() {
        self.viewModel?.$pointCount
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] count in
                self?.pointCloudCountView.updateCount(to: count)
            })
            .store(in: &self.cancellables)
    }
    
    /// viewModel 의 lidarData 값 변화를 수신하여 SelectLocationVC 로 전달하기 위한 함수
    private func bindLidarData() {
        self.viewModel?.$lidarData
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] _ in
                guard self?.viewModel?.mode == .loading else { return }
                
            })
            .store(in: &self.cancellables)
    }
    
    private func bindUploadSuccess() {
        self.viewModel?.$uploadSuccess
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] success in
                guard success else { return }
                
                self?.showAlert(title: "Upload Success", text: "You can see the record history in the SCANS page")
                self?.viewModel?.changeMode(to: .ready)
            })
            .store(in: &self.cancellables)
    }
    
    private func bindUploadProgress() {
        self.viewModel?.$uploadProgress
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] progress in
                guard progress != 0 else { return }
                self?.statusLabel.uploadProgress(to: progress)
            })
            .store(in: &self.cancellables)
    }
    
 
    
    private func bindSaveToStorageSuccess() {
        self.viewModel?.$saveToStorageSuccess
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] success in
                guard let success = success else { return }
                
                if success == false {
                    self?.showAlert(title: "Save Failed", text: "Failed to save temporary data")
                }
            })
            .store(in: &self.cancellables)
    }
    
    private func bindScanData() {
        self.viewModel?.$scanData
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] scanData in
                guard let scanData = scanData else { return }
                
                // 스캔 프리뷰 화면 표시
                self?.showScanPreview(with: scanData)
            })
            .store(in: &self.cancellables)
    }
    
    /// 스캔 프리뷰 화면 표시
    private func showScanPreview(with scanData: ScanData) {
        // 스캔 횟수 증가
        ScanCountManager.shared.increment()
        print("📊 [ScanFlow] showScanPreview — scanCount=\(ScanCountManager.shared.currentCount)")

        // 프리뷰 전환 시 상태 UI 숨기기
        self.statusLabel.changeText(to: .removed)
        self.recordingButton.changeStatus(to: .loading)
        self.pointCloudCountView.alpha = 0

        let previewVC = ScanPreviewVC()
        previewVC.scanData = scanData
        previewVC.renderer = self.viewModel?.renderer
        previewVC.delegate = self
        previewVC.currentMarker = self.currentMarker
        previewVC.modalPresentationStyle = UIModalPresentationStyle.fullScreen
        self.present(previewVC, animated: true) {
            print("✅ 스캔 프리뷰 표시 완료 (초고속 모드)")
            self.currentMarker = nil
        }
    }
    
    private func bindSaveScanDataSuccess() {
        self.viewModel?.$saveScanDataSuccess
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] success in
                guard let success = success else { return }

                if success == false {
                    self?.showAlert(title: "Save Failed", text: "Failed to save scan data")
                } else {
                    self?.showAlert(title: "Save Successful", text: "Scan data saved successfully")
                    self?.viewModel?.changeMode(to: .ready)
                    self?.configureARWorldTracking()
                }
            })
            .store(in: &self.cancellables)
    }

    private func bindIMUData() {
        self.viewModel?.renderer.$imuDisplayData
            .receive(on: DispatchQueue.main)
            .sink { [weak self] data in
                self?.updateIMUOverlay(data)
            }
            .store(in: &self.cancellables)
    }

    private func updateIMUOverlay(_ data: IMUDisplayData?) {
        let showIMU = PremiumManager.shared.isPremium
            && PremiumManager.shared.showIMUData
            && viewModel?.mode == .recording

        guard showIMU, let data = data else {
            imuOverlayStack.alpha = 0
            return
        }

        imuOverlayStack.alpha = 1
        if let l0 = imuOverlayStack.arrangedSubviews[0] as? UILabel {
            l0.text = String(format: "R:%.1f° P:%.1f° Y:%.1f°", data.roll, data.pitch, data.yaw)
        }
        if let l1 = imuOverlayStack.arrangedSubviews[1] as? UILabel {
            l1.text = String(format: "Acc: %.2f  %.2f  %.2f g", data.ax, data.ay, data.az)
        }
        if let l2 = imuOverlayStack.arrangedSubviews[2] as? UILabel {
            l2.text = String(format: "Gyro: %.2f  %.2f  %.2f", data.gx, data.gy, data.gz)
        }
    }
}

// MARK: Action & Logic
extension MainVC {
    func resetViewModel() {
        resetToInitialState()
    }
    
    /// 녹화 버튼 디바운스용 타임스탬프
    private static var lastRecordingButtonTap: Date = .distantPast

    /// RecordingButton Tab 액션
    @objc private func tapRecordingButton(_ sender: UIButton) {
        // Debounce: 0.8초 이내 재탭 무시
        let now = Date()
        guard now.timeIntervalSince(Self.lastRecordingButtonTap) > 0.8 else {
            print("⏳ 녹화 버튼 디바운스 — 무시됨")
            return
        }
        Self.lastRecordingButtonTap = now

        print("🔴 녹화 버튼 탭됨 - 현재 모드: \(String(describing: self.viewModel?.mode))")
        guard let currentMode = self.viewModel?.mode else { return }
        if currentMode != .ready && currentMode != .recording {
            print("⚠️ 예상치 못한 모드 감지: \(currentMode). 초기 상태로 강제 리셋")
            resetToInitialState()
            return
        }

        // 녹화 시작 시 MTKView 활성화 + 햅틱 피드백
        if currentMode == .ready {
            UIImpactFeedbackGenerator(style: .light).impactOccurred()
            print("🎬 녹화 시작 - MTKView 활성화")
            if let mtkView = self.view as? MTKView {
                mtkView.isPaused = false
                mtkView.setNeedsDisplay()
                print("✅ MTKView 활성화 완료: isPaused=\(mtkView.isPaused)")
            }
        } else if currentMode == .recording {
            UIImpactFeedbackGenerator(style: .rigid).impactOccurred()
        }

        self.viewModel?.changeMode()
    }
    
    /// 설정 버튼 Tab 액션
    @objc private func tapSettingsButton(_ sender: UIButton) {
        print("⚙️ Settings button tapped - Showing Algorithm Selection")
        
        let settingsView = AlgorithmSelectionView()
        let hostingController = UIHostingController(rootView: settingsView)
        hostingController.modalPresentationStyle = .overFullScreen
        hostingController.view.backgroundColor = .clear
        
        self.present(hostingController, animated: true)
    }
    
    /// ScansButton Tab 액션
    private func moveToScanStorageVC() {
        let scanStorageVC = ScanStorageVC()
        self.navigationController?.pushViewController(scanStorageVC, animated: true)
    }
    
    /// MarkerButton Tab 액션 - 지도 화면 표시
    private func showMapView() {
        print("🗺️ 지도 화면 표시")
        let mapVC = MapViewController()
        let navController = UINavigationController(rootViewController: mapVC)
        navController.modalPresentationStyle = .fullScreen
        present(navController, animated: true)
    }
    
    /// mode값에 따라 현재 동작상태 표시내용 설정 함수
    private func changeStatusLabel(to mode: MainVM.Mode) {
        switch mode {
        case .ready:
            self.statusLabel.changeText(to: .readyForRecording)
            self.processingSpinner.stopAnimating()
        case .recording:
            self.statusLabel.changeText(to: .recording)
            self.processingSpinner.stopAnimating()
        case .recordingTerminate:
            if ScanSettings.shared.algorithm == .depthViz {
                self.statusLabel.changeText(to: .optimizing)
            } else {
                self.statusLabel.changeText(to: .loading)
            }
            self.processingSpinner.startAnimating()
        case .loading:
            self.statusLabel.changeText(to: .loading)
            self.processingSpinner.startAnimating()
        case .uploading:
            self.statusLabel.changeText(to: .uploading)
            self.processingSpinner.startAnimating()
        case .uploadingTerminate:
            self.statusLabel.changeText(to: .loading)
            self.processingSpinner.startAnimating()
        case .cantGetGPS:
            self.statusLabel.changeText(to: .needGPS)
            self.processingSpinner.stopAnimating()
        case .cantRecord:
            self.statusLabel.changeText(to: .cantRecord)
            self.processingSpinner.stopAnimating()
        }
    }
    
    // 프로젝트 선택 + 파일명 입력 + 저장은 ScanPreviewVC에서 처리됨

    
    private func resetToInitialState(restartSession: Bool = true) {
        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            print("🔄 메인 화면 초기화 - restartSession: \(restartSession)")
            self.viewModel?.resetRenderer()
            self.viewModel?.changeMode(to: .ready)
            self.pointCloudCountView.updateCount(to: 0)
            self.recordingButton.changeStatus(to: .ready)
            self.statusLabel.changeText(to: .readyForRecording)
            self.scansButton.fadeIn()
            self.markerButton.fadeIn()
            self.locationManager.stopUpdatingLocation()
            guard restartSession else { return }
            self.session.pause()
            self.configureARWorldTracking()
        }
    }
    
    private func resetScanningState() {
        resetToInitialState()
    }

    // 팝업을 표시하는 공통 메서드
    private func showAlert(title: String, message: String, onDismiss: (() -> Void)? = nil) {
        let alert = UIAlertController(title: title, message: message, preferredStyle: .alert)
        let okAction = UIAlertAction(title: "확인", style: .default) { _ in
            onDismiss?()
        }
        alert.addAction(okAction)
        self.present(alert, animated: true, completion: nil)
    }
}


// MARK: - IMU Updates for DV-SLAM
extension MainVC {
    /// Start high-frequency IMU updates for DV-SLAM ESKF
    func startIMUUpdates() {
        guard motionManager.isDeviceMotionAvailable else {
            print("⚠️ Device motion not available")
            return
        }

        // 100Hz IMU update rate for ESKF
        motionManager.deviceMotionUpdateInterval = 0.01

        motionManager.startDeviceMotionUpdates(to: .main) { [weak self] motion, error in
            guard let motion = motion, error == nil else { return }
            guard self?.viewModel?.mode == .recording else { return }

            SLAMService.sharedInstance().processIMUData(motion)
        }

        print("✅ IMU updates started (100Hz)")
    }

    /// Stop IMU updates
    func stopIMUUpdates() {
        if motionManager.isDeviceMotionActive {
            motionManager.stopDeviceMotionUpdates()
            print("✅ IMU updates stopped")
        }
    }
}


// MARK: - ARSession Delegate
extension MainVC {
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        // ARFrame 메모리 누수 방지: 필요한 데이터만 추출하고 즉시 해제
        let currentTransform = frame.camera.transform
        
        if let previousTransform = previousCameraTransform {
            // 움직임 계산
            let positionDelta = distance_squared(currentTransform.columns.3, previousTransform.columns.3)
            let rotationDelta = dot(currentTransform.columns.2, previousTransform.columns.2)
            
            // 움직임 임계값 (매우 작은 값으로 민감하게 감지)
            let hasMovement = positionDelta > 0.0001 || rotationDelta < 0.9999
            
            if hasMovement {
                lastMovementTime = Date()
                hideGuideLabel()
            }
        } else {
            // 첫 프레임
            lastMovementTime = Date()
        }

        previousCameraTransform = currentTransform

        // Feed ARFrame to DV-SLAM engine during recording
        if viewModel?.mode == .recording {
            SLAMService.sharedInstance().processARFrame(frame)
            // 프리미엄: 카메라 PiP + 동영상 프레임 기록
            updateCameraPiP(with: frame)
        }
    }
    
    // ARSession 에러 핸들링
    func session(_ session: ARSession, didFailWithError error: Error) {
        print("❌ ARSession 에러: \(error.localizedDescription)")
        
        if let arError = error as? ARError {
            switch arError.errorCode {
            case 102:  // Camera access
                print("⚠️ 카메라 권한이 없습니다. Settings에서 권한을 허용하세요.")
            case 104:  // World tracking failed
                print("⚠️ World tracking 실패 - ARSession 재시작 시도")
                DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) { [weak self] in
                    self?.configureARWorldTracking()
                }
            default:
                print("⚠️ ARError: \(arError.errorCode)")
            }
        }
    }
    
    // ARSession 중단 시 자동 재시작
    func sessionWasInterrupted(_ session: ARSession) {
        print("⚠️ ARSession 중단됨")
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        print("✅ ARSession 중단 종료 - 재시작")
        configureARWorldTracking()
    }
    
    // 카메라 트래킹 상태 모니터링
    func session(_ session: ARSession, cameraDidChangeTrackingState camera: ARCamera) {
        switch camera.trackingState {
        case .notAvailable:
            print("📷 카메라 트래킹: 사용 불가")
        case .limited(let reason):
            switch reason {
            case .initializing:
                print("📷 카메라 트래킹: 초기화 중...")
            case .relocalizing:
                print("📷 카메라 트래킹: 재위치 파악 중...")
            case .excessiveMotion:
                print("⚠️ 카메라 트래킹: 움직임이 너무 빠름")
            case .insufficientFeatures:
                print("⚠️ 카메라 트래킹: 특징점 부족")
            @unknown default:
                print("⚠️ 카메라 트래킹: 알 수 없는 제한")
            }
        case .normal:
            print("✅ 카메라 트래킹: 정상")
        }
    }
    
    private func startMovementCheckTimer() {
        lastMovementTime = Date()
        movementCheckTimer?.invalidate()
        
        movementCheckTimer = Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in
                guard let self = self else { return }
                if let lastTime = self.lastMovementTime {
                    let timeSinceLastMovement = Date().timeIntervalSince(lastTime)
                    if self.viewModel?.renderer.isRecording == true {
                        if timeSinceLastMovement > 2.0 {
                            self.hideGuideLabel()
                        }
                    } else {
                        if timeSinceLastMovement > 1.0 {
                            self.showGuideLabel(text: "Move iPhone to Start")
                        }
                    }
                }
            }
        }
    }
    
    private func stopMovementCheckTimer() {
        movementCheckTimer?.invalidate()
        movementCheckTimer = nil
    }
    
    private func showGuideLabel(text: String) {
        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            
            if self.arGuideLabel.alpha < 1 {
                self.arGuideLabel.text = text
                UIView.animate(withDuration: 0.3) {
                    self.arGuideLabel.alpha = 1
                }
            } else if self.arGuideLabel.text != text {
                // 텍스트만 변경
                self.arGuideLabel.text = text
            }
        }
    }
    
    private func hideGuideLabel() {
        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            
            if self.arGuideLabel.alpha > 0 {
                UIView.animate(withDuration: 0.3) {
                    self.arGuideLabel.alpha = 0
                }
            }
        }
    }
}

// MapViewController, MarkerAnnotation, MarkerDetailView, AddMarkerView
// are defined in their dedicated files under Presentation/Map/

// MARK: - ScanPreviewDelegate & TaskDelegate are below
// (MapViewController was previously duplicated here — removed to avoid compiler errors)

/* REMOVED: MapViewController, MarkerAnnotation, findSuperview, MarkerDetailView, AddMarkerView
   These types are defined in Presentation/Map/ folder.
   private var mapView: MKMapView!
    private var markers: [LocationMarker] = []
    private var selectedMarker: LocationMarker?
    private var temporaryAnnotation: MKPointAnnotation?
    private var poiSearch: MKLocalSearch?
    private let geocoder = CLGeocoder()
    private var lastSuggestedName: String?
    private var lastSuggestedDescription: String?
    private var lastSuggestedAddress: String?

    override func viewDidLoad() {
        super.viewDidLoad()
        setupNavigationBar()
        setupMapView()
        loadMarkers()
        addMarkers()
        setupGestures()
    }

    private func loadMarkers() {
        markers = MarkerStorage.shared.getAllMarkers()
        print("✅ 마커 로드 완료: \(markers.count)개")
    }

    private func setupNavigationBar() {
        title = "Scan Locations"
        navigationController?.navigationBar.prefersLargeTitles = false

        let closeButton = UIBarButtonItem(barButtonSystemItem: .close, target: self, action: #selector(closeButtonTapped))
        navigationItem.leftBarButtonItem = closeButton

        let addButton = UIBarButtonItem(barButtonSystemItem: .add, target: self, action: #selector(addMarkerButtonTapped))
        navigationItem.rightBarButtonItem = addButton
    }

    private func setupMapView() {
        mapView = MKMapView(frame: view.bounds)
        mapView.delegate = self
        mapView.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        mapView.showsUserLocation = true
        mapView.showsCompass = true
        if #available(iOS 13.0, *) {
            mapView.pointOfInterestFilter = .includingAll
        }
        view.addSubview(mapView)

        let seoulCenter = CLLocationCoordinate2D(latitude: 37.5665, longitude: 126.9780)
        let region = MKCoordinateRegion(center: seoulCenter, span: MKCoordinateSpan(latitudeDelta: 0.1, longitudeDelta: 0.1))
        mapView.setRegion(region, animated: false)
    }

    private func setupGestures() {
        let longPress = UILongPressGestureRecognizer(target: self, action: #selector(handleLongPress(_:)))
        longPress.minimumPressDuration = 0.5
        mapView.addGestureRecognizer(longPress)

        let tapGesture = UITapGestureRecognizer(target: self, action: #selector(handleMapTap(_:)))
        tapGesture.numberOfTouchesRequired = 1
        tapGesture.numberOfTapsRequired = 1
        tapGesture.cancelsTouchesInView = false
        tapGesture.require(toFail: longPress)
        tapGesture.delegate = self
        mapView.addGestureRecognizer(tapGesture)
    }

    @objc private func handleLongPress(_ gesture: UILongPressGestureRecognizer) {
        guard gesture.state == .began else { return }
        let touchPoint = gesture.location(in: mapView)
        let coordinate = mapView.convert(touchPoint, toCoordinateFrom: mapView)
        placeTemporaryAnnotation(at: coordinate, feedbackStyle: .medium)
    }

    @objc private func handleMapTap(_ gesture: UITapGestureRecognizer) {
        guard gesture.state == .ended else { return }
        let touchPoint = gesture.location(in: mapView)

        if let hitView = mapView.hitTest(touchPoint, with: nil) {
            if hitView is UIControl { return }
            if let annotationView = (hitView as? MKAnnotationView) ?? hitView.findSuperview(of: MKAnnotationView.self) {
                if let markerAnnotation = annotationView.annotation as? MarkerAnnotation {
                    mapView.selectAnnotation(markerAnnotation, animated: true)
                    return
                }
                if annotationView.annotation === temporaryAnnotation {
                    mapView.selectAnnotation(annotationView.annotation!, animated: true)
                    return
                }
                if let annotation = annotationView.annotation {
                    placeTemporaryAnnotation(at: annotation.coordinate, feedbackStyle: .light)
                    return
                }
            }
        }

        let coordinate = mapView.convert(touchPoint, toCoordinateFrom: mapView)
        placeTemporaryAnnotation(at: coordinate, feedbackStyle: .light)
    }

    private func addMarkers() {
        for marker in markers {
            let annotation = MarkerAnnotation(marker: marker)
            mapView.addAnnotation(annotation)
        }
    }

    @objc private func addMarkerButtonTapped() {
        let addMarkerView = AddMarkerView { [weak self] marker in
            self?.addNewMarker(marker)
        }
        let hostingController = UIHostingController(rootView: addMarkerView)
        present(hostingController, animated: true)
    }

    private func addNewMarker(_ marker: LocationMarker) {
        MarkerStorage.shared.addMarker(marker)
        markers.append(marker)

        let annotation = MarkerAnnotation(marker: marker)
        mapView.addAnnotation(annotation)

        let region = MKCoordinateRegion(center: marker.coordinate, span: MKCoordinateSpan(latitudeDelta: 0.05, longitudeDelta: 0.05))
        mapView.setRegion(region, animated: true)
        print("✅ 새 마커 추가: \(marker.name)")
    }

    @objc private func closeButtonTapped() {
        clearTemporarySelection()
        dismiss(animated: true)
    }

    private func placeTemporaryAnnotation(at coordinate: CLLocationCoordinate2D, feedbackStyle: UIImpactFeedbackGenerator.FeedbackStyle) {
        if let tempAnnotation = temporaryAnnotation {
            mapView.removeAnnotation(tempAnnotation)
        }
        poiSearch?.cancel()
        poiSearch = nil
        geocoder.cancelGeocode()
        lastSuggestedName = nil
        lastSuggestedDescription = nil
        lastSuggestedAddress = nil

        let annotation = MKPointAnnotation()
        annotation.coordinate = coordinate
        annotation.title = "Add Here"
        annotation.subtitle = "Tap \"Add Here\" to save this spot"

        temporaryAnnotation = annotation
        mapView.addAnnotation(annotation)
        mapView.selectAnnotation(annotation, animated: true)

        UIImpactFeedbackGenerator(style: feedbackStyle).impactOccurred()
        print("📍 임시 마커 생성: \(coordinate.latitude), \(coordinate.longitude)")
        fetchPlaceSuggestion(near: coordinate)
    }

    private func fetchPlaceSuggestion(near coordinate: CLLocationCoordinate2D) {
        if #available(iOS 13.0, *) {
            let request = MKLocalPointsOfInterestRequest(center: coordinate, radius: 200)
            let search = MKLocalSearch(request: request)
            poiSearch = search
            search.start { [weak self] response, _ in
                guard let self = self else { return }
                defer { self.poiSearch = nil }
                guard let currentAnnotation = self.temporaryAnnotation, self.distance(currentAnnotation.coordinate, coordinate) < 250 else { return }
                if let items = response?.mapItems,
                   let nearest = items.min(by: { self.distance($0.placemark.coordinate, coordinate) < self.distance($1.placemark.coordinate, coordinate) }) {
                    let subtitle = nearest.placemark.title ?? nearest.placemark.subtitle
                    let categoryDescription = nearest.pointOfInterestCategory?.rawValue.replacingOccurrences(of: "MKPointOfInterestCategory", with: "")
                    self.applySuggestion(name: nearest.name, description: categoryDescription, address: subtitle)
                    self.updateTemporaryAnnotation(title: nearest.name, subtitle: subtitle)
                    return
                }
                self.reverseGeocode(coordinate: coordinate)
            }
        } else {
            reverseGeocode(coordinate: coordinate)
        }
    }

    private func updateTemporaryAnnotation(title: String?, subtitle: String?) {
        DispatchQueue.main.async { [weak self] in
            guard let self = self, let annotation = self.temporaryAnnotation else { return }
            if let title = title, !title.isEmpty {
                annotation.title = title
            }
            if let subtitle = subtitle, !subtitle.isEmpty {
                annotation.subtitle = subtitle
            }
            self.mapView.selectAnnotation(annotation, animated: true)
        }
    }

    private func applySuggestion(name: String?, description: String?, address: String?) {
        lastSuggestedName = name
        lastSuggestedAddress = address
        if let description = description, !description.isEmpty {
            let cleaned = description.replacingOccurrences(of: "_", with: " ").trimmingCharacters(in: .whitespacesAndNewlines)
            lastSuggestedDescription = cleaned.capitalized
        } else {
            lastSuggestedDescription = address
        }
    }

    private func reverseGeocode(coordinate: CLLocationCoordinate2D) {
        let location = CLLocation(latitude: coordinate.latitude, longitude: coordinate.longitude)
        geocoder.reverseGeocodeLocation(location) { [weak self] placemarks, _ in
            guard let self = self else { return }
            guard let currentAnnotation = self.temporaryAnnotation, self.distance(currentAnnotation.coordinate, coordinate) < 250 else { return }
            guard let placemark = placemarks?.first else { return }
            let name = placemark.name ?? placemark.areasOfInterest?.first
            let address = self.compactAddress(for: placemark)
            self.applySuggestion(name: name, description: placemark.locality ?? placemark.subLocality, address: address)
            self.updateTemporaryAnnotation(title: name ?? "Add Here", subtitle: address ?? "Tap \"Add Here\" to save this spot")
        }
    }

    private func clearTemporarySelection() {
        if let tempAnnotation = temporaryAnnotation {
            mapView.removeAnnotation(tempAnnotation)
        }
        temporaryAnnotation = nil
        poiSearch?.cancel()
        poiSearch = nil
        geocoder.cancelGeocode()
        lastSuggestedName = nil
        lastSuggestedDescription = nil
        lastSuggestedAddress = nil
    }

    private func showAddMarkerDialog(at coordinate: CLLocationCoordinate2D) {
        let alert = UIAlertController(title: "마커 추가", message: "마커 정보를 입력하세요", preferredStyle: .alert)

        alert.addTextField { [weak self] textField in
            textField.placeholder = "마커 이름 (예: 서울역)"
            textField.autocapitalizationType = .words
            if let suggested = self?.lastSuggestedName {
                textField.text = suggested
            }
        }

        alert.addTextField { [weak self] textField in
            textField.placeholder = "설명 (선택사항)"
            textField.autocapitalizationType = .sentences
            if let description = self?.lastSuggestedDescription {
                textField.text = description
            }
        }

        alert.addAction(UIAlertAction(title: "취소", style: .cancel) { [weak self] _ in
            self?.clearTemporarySelection()
        })

        alert.addAction(UIAlertAction(title: "추가", style: .default) { [weak self, weak alert] _ in
            guard let self = self else { return }
            guard let rawName = alert?.textFields?[0].text?.trimmingCharacters(in: .whitespacesAndNewlines), !rawName.isEmpty else {
                self.clearTemporarySelection()
                return
            }

            let descriptionInput = alert?.textFields?[1].text?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
            let description = descriptionInput.isEmpty ? (self.lastSuggestedDescription ?? "User-added marker") : descriptionInput
            let address = self.lastSuggestedAddress

            let newMarker = LocationMarker(
                name: rawName,
                description: description,
                latitude: coordinate.latitude,
                longitude: coordinate.longitude,
                address: address,
                placeId: nil,
                imageUrls: []
            )

            self.clearTemporarySelection()
            self.addNewMarker(newMarker)
            UINotificationFeedbackGenerator().notificationOccurred(.success)
        })

        present(alert, animated: true)
    }

    private func showMarkerDetail(marker: LocationMarker) {
        let detailView = MarkerDetailView(
            marker: marker,
            onScanHere: { [weak self] in self?.handleScanHere(marker: marker) },
            onViewData: { [weak self] in self?.handleViewData(marker: marker) }
        )

        let hostingController = UIHostingController(rootView: detailView)
        hostingController.modalPresentationStyle = .pageSheet

        if let sheet = hostingController.sheetPresentationController {
            sheet.detents = [.medium()]
            sheet.prefersGrabberVisible = true
        }
        present(hostingController, animated: true)
    }

    private func handleScanHere(marker: LocationMarker) {
        print("🎯 Scan Here: \(marker.name)")
        dismiss(animated: true) { [weak self] in
            self?.dismiss(animated: true) {
                NotificationCenter.default.post(name: NSNotification.Name("StartScanAtLocation"), object: marker)
            }
        }
    }

    private func handleViewData(marker: LocationMarker) {
        print("📊 View Data: \(marker.name)")
        dismiss(animated: true) { [weak self] in
            self?.dismiss(animated: true) {
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.3) {
                    NotificationCenter.default.post(name: NSNotification.Name("ShowMarkerProject"), object: marker)
                }
            }
        }
    }

    func gestureRecognizer(_ gestureRecognizer: UIGestureRecognizer, shouldRecognizeSimultaneouslyWith otherGestureRecognizer: UIGestureRecognizer) -> Bool {
        return true
    }

    private func distance(_ lhs: CLLocationCoordinate2D, _ rhs: CLLocationCoordinate2D) -> CLLocationDistance {
        let lhsLocation = CLLocation(latitude: lhs.latitude, longitude: lhs.longitude)
        let rhsLocation = CLLocation(latitude: rhs.latitude, longitude: rhs.longitude)
        return lhsLocation.distance(from: rhsLocation)
    }

    private func compactAddress(for placemark: CLPlacemark) -> String? {
        var components: [String] = []
        if let name = placemark.name { components.append(name) }
        if let thoroughfare = placemark.thoroughfare { components.append(thoroughfare) }
        if let locality = placemark.locality { components.append(locality) }
        if let administrativeArea = placemark.administrativeArea { components.append(administrativeArea) }
        if let country = placemark.country { components.append(country) }
        if components.isEmpty { return nil }
        return components.joined(separator: ", ")
    }
}

extension MapViewController: MKMapViewDelegate {
    func mapView(_ mapView: MKMapView, viewFor annotation: MKAnnotation) -> MKAnnotationView? {
        if let tempAnnotation = annotation as? MKPointAnnotation, tempAnnotation === temporaryAnnotation {
            let identifier = "TempAnnotation"
            var annotationView = mapView.dequeueReusableAnnotationView(withIdentifier: identifier) as? MKMarkerAnnotationView
            if annotationView == nil {
                annotationView = MKMarkerAnnotationView(annotation: annotation, reuseIdentifier: identifier)
                annotationView?.canShowCallout = true
                annotationView?.markerTintColor = .systemGreen
                let addButton: UIButton
                if #available(iOS 15.0, *) {
                    var config = UIButton.Configuration.plain()
                    config.baseForegroundColor = .systemBlue
                    var title = AttributedString("Add Here")
                    title.font = .systemFont(ofSize: 14, weight: .semibold)
                    config.attributedTitle = title
                    config.contentInsets = NSDirectionalEdgeInsets(top: 4, leading: 10, bottom: 4, trailing: 10)
                    addButton = UIButton(configuration: config)
                } else {
                    addButton = UIButton(type: .system)
                    addButton.setTitle("Add Here", for: .normal)
                    addButton.setTitleColor(.systemBlue, for: .normal)
                    addButton.titleLabel?.font = .systemFont(ofSize: 14, weight: .semibold)
                    addButton.contentEdgeInsets = UIEdgeInsets(top: 4, left: 10, bottom: 4, right: 10)
                }
                addButton.sizeToFit()
                annotationView?.rightCalloutAccessoryView = addButton
            } else {
                annotationView?.annotation = annotation
            }
            return annotationView
        }

        guard let markerAnnotation = annotation as? MarkerAnnotation else { return nil }

        let identifier = "MarkerAnnotation"
        var annotationView = mapView.dequeueReusableAnnotationView(withIdentifier: identifier) as? MKMarkerAnnotationView
        if annotationView == nil {
            annotationView = MKMarkerAnnotationView(annotation: annotation, reuseIdentifier: identifier)
            annotationView?.canShowCallout = true
            annotationView?.markerTintColor = .systemOrange
            let infoButton = UIButton(type: .detailDisclosure)
            annotationView?.rightCalloutAccessoryView = infoButton
        } else {
            annotationView?.annotation = annotation
        }
        annotationView?.glyphText = String(markerAnnotation.marker.name.prefix(1))
        return annotationView
    }

    func mapView(_ mapView: MKMapView, annotationView view: MKAnnotationView, calloutAccessoryControlTapped control: UIControl) {
        if let tempAnnotation = view.annotation as? MKPointAnnotation, tempAnnotation === temporaryAnnotation {
            showAddMarkerDialog(at: tempAnnotation.coordinate)
            return
        }

        guard let markerAnnotation = view.annotation as? MarkerAnnotation else { return }
        selectedMarker = markerAnnotation.marker
        showMarkerDetail(marker: markerAnnotation.marker)
    }
}

class MarkerAnnotation: NSObject, MKAnnotation {
    let marker: LocationMarker
    var coordinate: CLLocationCoordinate2D { marker.coordinate }
    var title: String? { marker.name }
    var subtitle: String? { marker.description }

    init(marker: LocationMarker) {
        self.marker = marker
        super.init()
    }
}

private extension UIView {
    func findSuperview<T: UIView>(of type: T.Type) -> T? {
        var current: UIView? = self
        while let candidate = current {
            if let match = candidate as? T {
                return match
            }
            current = candidate.superview
        }
        return nil
    }
}

// MARK: - SwiftUI Views
import SwiftUI

struct MarkerDetailView: View {
    let marker: LocationMarker
    let onScanHere: () -> Void
    let onViewData: () -> Void
    
    @Environment(\.dismiss) private var dismiss
    
    var body: some View {
        ScrollView {
            VStack(spacing: 0) {
                // 헤더
                VStack(alignment: .leading, spacing: 8) {
                    HStack {
                        Image(systemName: "mappin.circle.fill")
                            .font(.system(size: 40))
                            .foregroundColor(.orange)
                        
                        VStack(alignment: .leading, spacing: 4) {
                            Text(marker.name).font(.title2).fontWeight(.bold)
                            Text(marker.description).font(.subheadline).foregroundColor(.secondary)
                        }
                        Spacer()
                    }
                    
                    if let address = marker.address {
                        HStack(spacing: 6) {
                            Image(systemName: "location.fill").font(.caption).foregroundColor(.secondary)
                            Text(address).font(.caption).foregroundColor(.secondary)
                        }.padding(.top, 4)
                    }
                }.padding(20)
                
                Divider()
                
                // 이미지 갤러리 (Google Maps 이미지 2장)
                if !marker.imageUrls.isEmpty {
                    VStack(alignment: .leading, spacing: 12) {
                        Text("Photos").font(.headline).padding(.horizontal, 20).padding(.top, 16)
                        
                        ScrollView(.horizontal, showsIndicators: false) {
                            HStack(spacing: 12) {
                                ForEach(Array(marker.imageUrls.prefix(2).enumerated()), id: \.offset) { index, urlString in
                                    AsyncImage(url: URL(string: urlString)) { phase in
                                        switch phase {
                                        case .empty:
                                            Rectangle()
                                                .fill(Color.gray.opacity(0.2))
                                                .frame(width: 280, height: 180)
                                                .overlay(ProgressView())
                                        case .success(let image):
                                            image
                                                .resizable()
                                                .aspectRatio(contentMode: .fill)
                                                .frame(width: 280, height: 180)
                                                .clipped()
                                                .cornerRadius(12)
                                        case .failure:
                                            Rectangle()
                                                .fill(Color.gray.opacity(0.2))
                                                .frame(width: 280, height: 180)
                                                .overlay(
                                                    VStack {
                                                        Image(systemName: "photo")
                                                            .font(.largeTitle)
                                                            .foregroundColor(.gray)
                                                        Text("Image not available")
                                                            .font(.caption)
                                                            .foregroundColor(.gray)
                                                    }
                                                )
                                                .cornerRadius(12)
                                        @unknown default:
                                            EmptyView()
                                        }
                                    }
                                }
                            }
                            .padding(.horizontal, 20)
                        }
                        .padding(.bottom, 16)
                    }
                    
                    Divider()
                }
                
                // 좌표 정보
                HStack(spacing: 20) {
                    VStack(alignment: .leading, spacing: 4) {
                        Text("Latitude").font(.caption).foregroundColor(.secondary)
                        Text(String(format: "%.6f", marker.latitude)).font(.system(.body, design: .monospaced)).fontWeight(.medium)
                    }
                    Divider().frame(height: 30)
                    VStack(alignment: .leading, spacing: 4) {
                        Text("Longitude").font(.caption).foregroundColor(.secondary)
                        Text(String(format: "%.6f", marker.longitude)).font(.system(.body, design: .monospaced)).fontWeight(.medium)
                    }
                    Spacer()
                }.padding(.horizontal, 20).padding(.vertical, 16)
                
                Divider()
                
                // 버튼
                VStack(spacing: 12) {
                    Button(action: onScanHere) {
                        HStack {
                            Image(systemName: "camera.fill").font(.system(size: 18, weight: .semibold))
                            Text("Scan Here").font(.system(size: 17, weight: .semibold))
                        }
                        .foregroundColor(.white)
                        .frame(maxWidth: .infinity).frame(height: 54)
                        .background(LinearGradient(gradient: Gradient(colors: [Color.blue, Color.blue.opacity(0.8)]), startPoint: .leading, endPoint: .trailing))
                        .cornerRadius(12)
                    }
                    
                    Button(action: onViewData) {
                        HStack {
                            Image(systemName: "folder.fill").font(.system(size: 18, weight: .semibold))
                            Text("View Data").font(.system(size: 17, weight: .semibold))
                        }
                        .foregroundColor(.white)
                        .frame(maxWidth: .infinity).frame(height: 54)
                        .background(LinearGradient(gradient: Gradient(colors: [Color.orange, Color.orange.opacity(0.8)]), startPoint: .leading, endPoint: .trailing))
                        .cornerRadius(12)
                    }
                }.padding(20)
            }
        }
        .background(Color(UIColor.systemBackground))
    }
}

struct AddMarkerView: View {
    @Environment(\.dismiss) private var dismiss
    @State private var name = ""
    @State private var description = ""
    @State private var latitude = ""
    @State private var longitude = ""
    @State private var address = ""
    @State private var showAlert = false
    @State private var alertMessage = ""
    
    let onSave: (LocationMarker) -> Void
    
    var body: some View {
        NavigationView {
            Form {
                Section(header: Text("Basic Information")) {
                    TextField("Name", text: $name).autocapitalization(.words)
                    TextField("Description", text: $description).autocapitalization(.sentences)
                }
                
                Section(header: Text("Location")) {
                    HStack {
                        Text("Latitude").frame(width: 80, alignment: .leading)
                        TextField("37.5665", text: $latitude).keyboardType(.decimalPad)
                    }
                    HStack {
                        Text("Longitude").frame(width: 80, alignment: .leading)
                        TextField("126.9780", text: $longitude).keyboardType(.decimalPad)
                    }
                    TextField("Address (Optional)", text: $address).autocapitalization(.words)
                }
            }
            .navigationTitle("Add Marker")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .navigationBarLeading) {
                    Button("Cancel") { dismiss() }
                }
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Save") { saveMarker() }
                        .fontWeight(.semibold)
                        .disabled(!isValid)
                }
            }
            .alert("Error", isPresented: $showAlert) {
                Button("OK", role: .cancel) { }
            } message: {
                Text(alertMessage)
            }
        }
    }
    
    private var isValid: Bool {
        !name.isEmpty && !description.isEmpty && !latitude.isEmpty && !longitude.isEmpty &&
        Double(latitude) != nil && Double(longitude) != nil
    }
    
    private func saveMarker() {
        guard let lat = Double(latitude), let lon = Double(longitude) else {
            alertMessage = "Invalid coordinates."
            showAlert = true
            return
        }
        
        guard lat >= -90 && lat <= 90 else {
            alertMessage = "Latitude must be between -90 and 90."
            showAlert = true
            return
        }
        
        guard lon >= -180 && lon <= 180 else {
            alertMessage = "Longitude must be between -180 and 180."
            showAlert = true
            return
        }
        
        let marker = LocationMarker(name: name, description: description, latitude: lat, longitude: lon, address: address.isEmpty ? nil : address)
        onSave(marker)
        dismiss()
    }
}
*/

// MARK: - ScanPreviewDelegate
extension MainVC {
    func scanPreviewDidSave(_ preview: ScanPreviewVC, scanData: ScanData) {
        print("📊 [SaveFlow] scanPreviewDidSave 호출됨 — scanCount=\(ScanCountManager.shared.currentCount)")
        preview.dismiss(animated: true) { [weak self] in
            guard let self = self else { return }
            self.showPremiumPopupIfNeeded()
        }
    }

    /// 프리미엄 팝업 표시 (저장/삭제 공통)
    private func showPremiumPopupIfNeeded() {
        // 녹화 중이면 팝업 표시하지 않음
        guard viewModel?.mode == .ready else {
            print("📊 [Popup] 스킵 — 현재 모드: \(String(describing: viewModel?.mode))")
            return
        }

        #if DEBUG
        let showPopup = ScanCountManager.shared.shouldShowPremiumPrompt
        #else
        let showPopup = ScanCountManager.shared.shouldShowPremiumPrompt
            && !ScanCountManager.shared.shouldShowInterstitialAd
        #endif

        print("📊 [Popup] showPopup=\(showPopup), count=\(ScanCountManager.shared.currentCount)")

        if showPopup {
            ScanCountManager.shared.markPromptShown()
            let popup = UIHostingController(rootView: PremiumVideoPopup())
            popup.modalPresentationStyle = .overCurrentContext
            popup.modalTransitionStyle = .crossDissolve
            popup.view.backgroundColor = .clear
            popup.view.isOpaque = false
            self.present(popup, animated: true)
        }
    }

    func scanPreviewDidDelete(_ preview: ScanPreviewVC) {
        print("📊 [DeleteFlow] scanPreviewDidDelete 호출됨 — scanCount=\(ScanCountManager.shared.currentCount)")
        preview.dismiss(animated: true) { [weak self] in
            guard let self = self else { return }
            self.showPremiumPopupIfNeeded()
        }
    }
}

// MARK: - TaskDelegate
extension MainVC: TaskDelegate {
    func sharePLY(file: Any) {
        let activityVC = UIActivityViewController(activityItems: [file], applicationActivities: nil)
        present(activityVC, animated: true)
    }

    func startMakingPlyFile() {
        print("📦 Export 시작...")
        recordingButton.changeStatus(to: .loading)
        settingsButton.isEnabled = false
    }

    func finishMakingPlyFile() {
        print("✅ Export 완료")
        settingsButton.isEnabled = true
    }

    func startUploadingData() {
        print("📤 업로드 시작...")
    }

    func showShareOrUpload(stringData: String, fileName: String) {
        // Legacy string export path — handled by Combine binding now
    }
}

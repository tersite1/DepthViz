//
//  MainVC.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//


//메인화면 다루기 담당
import UIKit
import Metal
import MetalKit
import ARKit
import Combine
import SwiftUI

/// 메인화면의 UI 및 UX 담당
final class MainVC: UIViewController, ARSessionDelegate, CLLocationManagerDelegate {
    /// 기록측정 및 종료 버튼
    private let recordingButton = RecordingButton()
    /// 현재 동작상태 표시 텍스트
    private let statusLabel = StatusIndicatorLabel()
    /// 현재 측정중인 Point Cloud 개수 표시 뷰
    private let pointCloudCountView = PointCloudCountView()
    /// 측정이력창 표시 버튼
    private let scansButton = ScansButton()
    /// cpm 표시 버튼
    private let cpmButton = CPMButton()
    /// Point Cloud 표시를 위한 Session
    private let session = ARSession()
    /// gps 측정을 위한 객체
    private var locationManager = CLLocationManager()
    /// 메인화면과 관련된 로직담당 객체
    private var viewModel: MainVM?
    private var cancellables: Set<AnyCancellable> = []
    
    /// MainVC 최초 접근시 configure
    override func viewDidLoad() {
        super.viewDidLoad()
        self.configureUI()
        self.configureViewModel()
        self.checkLidarSensor()
        self.configureLocationManager()
        self.bindViewModel()
        
        // AdMob 배너 광고 추가 (SDK가 설치된 경우에만)
        #if canImport(GoogleMobileAds)
        self.setupAdMobBanner()
        #endif
    }
    
    #if canImport(GoogleMobileAds)
    /// AdMob 배너 광고 설정
    private func setupAdMobBanner() {
        // AdMobManager를 통해 배너 광고 추가
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) { [weak self] in
            guard let self = self else { return }
            AdMobManager.shared.addBannerToViewController(self, at: .bottom)
        }
    }
    #endif
    
    /// MainVC 화면 진입시 필요한 설정들
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        // NavigationBar를 표시되지 않도록 설정한다
        self.navigationController?.setNavigationBarHidden(true, animated: true)
        // ARSession을 활성화한다
        self.configureARWorldTracking()
    }
    
    /// 다른화면으로 전환시 ARSession 일시정지한다
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        self.session.pause()
    }
    
    /// 앱이 메모리경고를 수신받는 경우
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // 현재까지 측정중인 파일을 앱 내부로 임시저장 후 앱을 종료한다
        self.viewModel?.terminateRecording()
    }
    
    // CPM 계산 뷰로 이동하는 함수 추가
    // MainVC.swift 안에 있는 moveToCPMCalculation 함수
    func moveToCPMCalculation() {
        let cpmView = MContentView()  // SwiftUI View
        
        // UIHostingController 생성과 함께 modelContainer 전달
        let cpmViewController = UIHostingController(rootView: cpmView
            .modelContainer(for: [Activity.self]))  // CPM View에 modelContainer 전달
        
        self.navigationController?.pushViewController(cpmViewController, animated: true)
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
        // Metal 디바이스 생성
        guard let device = MTLCreateSystemDefaultDevice() else {
            print("Metal is not supported on this device")
            return
        }
        
        // Metal Object를 표시하기 위한 MetalKitView로 설정
        if let view = view as? MTKView {
            // MetalKitView에 표시하기 위한 Metal 디바이스 설정
            view.device = device
            
            view.backgroundColor = UIColor.clear
            // MetalKitView의 depth 크기 설정
            view.depthStencilPixelFormat = .depth32Float
            // 논리적 좌표공간(logical coordinate)(단위: points)과 장치 좌표공간(device coordinate)(단위: pixels)간의 스케일링 값
            // 1로 설정한 경우 실제좌표계와 MTKView에서 표시되는 좌표계와 동일하게 설정한다 (실제를 그대로 아이폰에서 표시하는 경우)
            view.contentScaleFactor = 1
            // MetalKitView의 내용을 업데이트하고자 하는 경우 호출하기 위한 delegate 설정
            view.delegate = self
            
            // Configure the ViewModel, Renderer to draw to the view
            self.viewModel = MainVM(session: self.session, device: device, view: view)
        }
    }
    
    /// LiDAR 측정을 위한 ARSession 활성화 및 Configure 설정 부분
    private func configureARWorldTracking() {
        guard self.viewModel?.mode != .cantRecord else { return }
        // Create a world-tracking configuration, and enable the scene depth frame-semantic.
        // 디바이스(iPhone)의 움직임을 추적하기 위한 Configuration 값 (움직이는대로 그대로 AR로 표시하기 위함)
        let configuration = ARWorldTrackingConfiguration()
        // 카메라를 통해 보이는 실제 객체까지의 거리, 여러 프레임의 평균 거리값을 제공하도록 설정
        configuration.frameSemantics = [.sceneDepth, .smoothedSceneDepth]

        // Run the view's session
        self.session.run(configuration)
        
        // The screen shouldn't dim during AR experiences.
        UIApplication.shared.isIdleTimerDisabled = true
    }
}

// MARK: AR 표출을 위한 MTKView Delegate
extension MainVC: MTKViewDelegate {
    // Called whenever view changes orientation or layout is changed
    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        self.viewModel?.rendererResize(to: size)
    }
    
    // MetalKitView가 업데이트되어야 할 때 불린다. 이때 새롭게 다시 그린다.
    func draw(in view: MTKView) {
        guard self.viewModel?.mode != .ready else {
            return
        }
        
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

// MARK: - Configure
extension MainVC {
    /// MainVC 표시할 UI 설정
    private func configureUI() {
        // recordingButton
        self.recordingButton.addTarget(self, action: #selector(tapRecordingButton), for: .touchUpInside)
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
        
        // pointCloudCountView
        self.view.addSubview(self.pointCloudCountView)
        NSLayoutConstraint.activate([
            self.pointCloudCountView.centerXAnchor.constraint(equalTo: self.view.centerXAnchor),
            self.pointCloudCountView.topAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.topAnchor, constant: 12)
        ])
        
        // scansButton
        self.scansButton.addAction(UIAction(handler: { [weak self] _ in
            self?.moveToScanStorageVC()
        }), for: .touchUpInside)
        self.view.addSubview(self.scansButton)
        NSLayoutConstraint.activate([
            self.scansButton.leadingAnchor.constraint(equalTo: self.view.leadingAnchor, constant: 32),
            self.scansButton.centerYAnchor.constraint(equalTo: self.recordingButton.centerYAnchor)
        ])
        
        // cpmButton (New button added to the right of scansButton)
        self.cpmButton.addAction(UIAction(handler: { [weak self] _ in
                    self?.moveToCPMCalculation()
                }), for: .touchUpInside)
        self.view.addSubview(self.cpmButton)
        NSLayoutConstraint.activate([
            // CPM Button Constraints
            self.cpmButton.leadingAnchor.constraint(equalTo: self.scansButton.trailingAnchor, constant: 212),
            self.cpmButton.centerYAnchor.constraint(equalTo: self.recordingButton.centerYAnchor)
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
            // ViewModel 상태 초기화
        self.viewModel?.resetRenderer()
        self.viewModel?.changeMode(to: .ready)
            
            // ARSession 재구성
        self.configureARWorldTracking()
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
    }
    
    /// viewModel 의 mode 값 변화를 수신하기 위한 함수
    private func bindMode() {
        self.viewModel?.$mode
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] mode in
                switch mode {
                case .ready:
                    self?.locationManager.stopUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .ready)
                    self?.scansButton.fadeIn()
                    self?.viewModel?.rendererDraw()
                case .recording:
                    self?.locationManager.startUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .recording)
                    self?.scansButton.fadeOut()
                    self?.cpmButton.disappear()
                case .loading:
                    self?.locationManager.stopUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .loading)
                    self?.scansButton.disappear()
                    /// cpm 표시 버튼
                    self?.cpmButton.disappear()
                case .cantRecord:
                    self?.locationManager.stopUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .cantRecording)
                    self?.scansButton.fadeIn()
                    self?.cpmButton.disappear()
                case .cantGetGPS:
                    self?.locationManager.stopUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .cantRecording)
                    self?.scansButton.fadeIn()
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
            .sink(receiveValue: { [weak self] lidarData in
                guard self?.viewModel?.mode == .loading,
                      let lidarData = lidarData else { return }
                
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
                    self?.showAlert(title: "임시데이터 저장 실패", text: "")
                }
            })
            .store(in: &self.cancellables)
    }
    
    private func bindScanData() {
        self.viewModel?.$scanData
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] scanData in
                guard let scanData = scanData else { return }
                
                // 프로젝트 선택 알림 표시
                self?.showProjectSelectionAlert(for: scanData)
            })
            .store(in: &self.cancellables)
    }
    
    private func bindSaveScanDataSuccess() {
        self.viewModel?.$saveScanDataSuccess
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] success in
                guard let success = success else { return }
                
                if success == false {
                    self?.showAlert(title: "ScanData 저장 실패", text: "")
                } else {
                    self?.showAlert(title: "ScanData 저장 성공", text: "")
                    self?.viewModel?.changeMode(to: .ready)
                    self?.configureARWorldTracking()
                }
            })
            .store(in: &self.cancellables)
    }
}

// MARK: Action & Logic
extension MainVC {
    func resetViewModel() {
        // ARSession 및 ViewModel 초기화
        self.session.pause()
        self.viewModel?.resetRenderer()
        self.configureARWorldTracking() // ARSession을 다시 구성하여 초기 상태로 돌아가도록 설정합니다.
        self.viewModel?.changeMode(to: .ready) // 초기 모드로 설정합니다.

    func OresetScanningState() {
            // ViewModel 상태 초기화
        self.viewModel?.resetRenderer()
        self.viewModel?.changeMode(to: .ready)
            
            // ARSession 재구성
        self.configureARWorldTracking()
        }
    
}

    
    /// RecordingButton Tab 액션
    @objc private func tapRecordingButton(_ sender: UIButton) {
        self.viewModel?.changeMode()
    }
    
    /// ScansButton Tab 액션
    private func moveToScanStorageVC() {
        let scanStorageVC = ScanStorageVC()
        self.navigationController?.pushViewController(scanStorageVC, animated: true)
    }
    
    /// mode값에 따라 현재 동작상태 표시내용 설정 함수
    private func changeStatusLabel(to mode: MainVM.Mode) {
        switch mode {
        case .ready:
            self.statusLabel.changeText(to: .readyForRecording)
        case .recording:
            self.statusLabel.changeText(to: .recording)
        case .recordingTerminate:
            self.statusLabel.changeText(to: .loading)
        case .loading:
            self.statusLabel.changeText(to: .loading)
        case .uploading:
            self.statusLabel.changeText(to: .uploading)
        case .uploadingTerminate:
            self.statusLabel.changeText(to: .loading)
        case .cantGetGPS:
            self.statusLabel.changeText(to: .needGPS)
        case .cantRecord:
            self.statusLabel.changeText(to: .cantRecord)
        }
    }
    
    private func showProjectSelectionAlert(for scanData: ScanData) {
        let alert = UIAlertController(title: "프로젝트 선택", message: "스캔 데이터를 저장할 프로젝트를 선택하세요.", preferredStyle: .alert)
        
        let projects = ScanStorage.shared.getProjects()
        
        for project in projects {
            alert.addAction(UIAlertAction(title: project, style: .default, handler: { _ in
                self.promptForFileNameAndSave(scanData: scanData, toProject: project)
            }))
        }
        
        alert.addAction(UIAlertAction(title: "새 프로젝트 추가", style: .default, handler: { [weak self] _ in
            // 새 프로젝트 이름을 입력받는 별도의 Alert 창을 띄운다
            self?.showNewProjectNameAlert(scanData: scanData)
        }))
        
        alert.addAction(UIAlertAction(title: "취소", style: .cancel, handler: { [weak self] _ in
            self?.viewModel?.resetRenderer()
            self?.viewModel?.changeMode(to: .ready)
            self?.configureARWorldTracking()
        }))
        
        self.present(alert, animated: true)
    }
    
    private func showNewProjectNameAlert(scanData: ScanData) {
        let nameAlert = UIAlertController(title: "새 프로젝트 이름", message: "새 프로젝트 이름을 입력하세요.", preferredStyle: .alert)
        
        nameAlert.addTextField { textField in
            textField.placeholder = "프로젝트 이름"
        }
        
        let createAction = UIAlertAction(title: "생성", style: .default) { [weak self] _ in
            guard let newProjectName = nameAlert.textFields?.first?.text, !newProjectName.isEmpty else {
                // 이름이 입력되지 않은 경우 아무 작업도 하지 않음
                return
            }
            // 프로젝트를 생성하고 데이터를 저장
            ScanStorage.shared.addProject(newProjectName)
            self?.promptForFileNameAndSave(scanData: scanData, toProject: newProjectName)
        }
        
        let cancelAction = UIAlertAction(title: "취소", style: .cancel, handler: { [weak self] _ in
            self?.viewModel?.resetRenderer()
            self?.viewModel?.changeMode(to: .ready)
            self?.configureARWorldTracking()
        })
        
        nameAlert.addAction(createAction)
        nameAlert.addAction(cancelAction)
        
        self.present(nameAlert, animated: true)
    }
    
    
    private func promptForFileNameAndSave(scanData: ScanData, toProject project: String) {
        let nameAlert = UIAlertController(title: "파일 이름 입력", message: "저장할 파일 이름을 입력하세요.", preferredStyle: .alert)
        
        nameAlert.addTextField { textField in
            textField.placeholder = "파일 이름"
        }
        
        let saveAction = UIAlertAction(title: "저장", style: .default) { [weak self] _ in
            guard let fileName = nameAlert.textFields?.first?.text, !fileName.isEmpty else {
                return
            }
            
            // ScanData의 fileName 속성 업데이트
            let updatedScanData = ScanData(
                date: scanData.date,
                fileName: fileName + ".ply",
                lidarData: scanData.lidarData,
                fileSize: scanData.fileSize,
                points: scanData.points,
                project: project
            )
            
            let success = ScanStorage.shared.save(updatedScanData)
            if success {
                self?.showAlert(title: "데이터 저장 성공", message: "데이터가 \(project) 프로젝트에 성공적으로 저장되었습니다.")
            } else {
                self?.showAlert(title: "데이터 저장 실패", message: "데이터 저장에 실패했습니다.")
            }
            
            self?.viewModel?.changeMode(to: .ready)
            self?.resetScanningState()
        }
        
        let cancelAction = UIAlertAction(title: "취소", style: .cancel) { [weak self] _ in
            self?.viewModel?.resetRenderer()
            self?.viewModel?.changeMode(to: .ready)
            self?.configureARWorldTracking()
        }
        
        nameAlert.addAction(saveAction)
        nameAlert.addAction(cancelAction)
        
        self.present(nameAlert, animated: true)
    }

    
    private func resetScanningState() {
        // ViewModel 상태 초기화
        self.viewModel?.resetRenderer()
        self.viewModel?.changeMode(to: .ready)
        
        // ARSession 재구성
        self.configureARWorldTracking()
    }

    // 팝업을 표시하는 공통 메서드
    private func showAlert(title: String, message: String) {
        let alert = UIAlertController(title: title, message: message, preferredStyle: .alert)
        let okAction = UIAlertAction(title: "확인", style: .default) { [weak self] _ in
            // 팝업 확인 후 초기 화면으로 돌아가기 위한 처리
            self?.configureARWorldTracking()
        }
        alert.addAction(okAction)
        self.present(alert, animated: true, completion: nil)
    }
}

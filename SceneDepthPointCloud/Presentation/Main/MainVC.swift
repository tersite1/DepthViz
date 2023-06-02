/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Main view controller for the AR experience.
*/

import UIKit
import Metal
import MetalKit
import ARKit
import Combine

/// 메인화면의 UI 및 UX 담당
final class MainVC: UIViewController, ARSessionDelegate {
    /// 기록측정 및 종료 버튼
    private let recordingButton = RecordingButton()
    /// 현재 동작상태 표시 텍스트
    private let statusLabel = StatusIndicatorLabel()
    /// 현재 측정중인 Point Cloud 개수 표시 뷰
    private let pointCloudCountView = PointCloudCountView()
    /// 측정이력창 표시 버튼
    private let scansButton = ScansButton()
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
    }
    
    /// MainVC 화면 진입시 NavigationBar를 표시되지 않도록 설정한다
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.navigationController?.setNavigationBarHidden(true, animated: true)
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
            self?.moveToScansVC()
        }), for: .touchUpInside)
        self.view.addSubview(self.scansButton)
        NSLayoutConstraint.activate([
            self.scansButton.leadingAnchor.constraint(equalTo: self.view.leadingAnchor, constant: 32),
            self.scansButton.centerYAnchor.constraint(equalTo: self.recordingButton.centerYAnchor)
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
    /// viewModel 에서 값 변화를 수신하기 위한 함수
    private func bindViewModel() {
        self.bindMode()
        self.bindPointCount()
        self.bindLidarData()
        self.bindUploadSuccess()
        self.bindUploadProgress()
        self.bindProcessWithStorageData()
        self.bindSaveToStorageSuccess()
    }
    
    /// viewModel 의 mode 값 변화를 수신하기 위한 함수
    private func bindMode() {
        self.viewModel?.$mode
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] mode in
                switch mode {
                case .ready:
                    self?.session.pause()
                    self?.locationManager.stopUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .ready)
                    self?.scansButton.fadeIn()
                    self?.viewModel?.rendererDraw()
                case .recording:
                    self?.configureARWorldTracking()
                    self?.locationManager.startUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .recording)
                    self?.scansButton.fadeOut()
                case .loading:
                    self?.locationManager.stopUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .loading)
                    self?.scansButton.disappear()
                case .cantRecord:
                    self?.locationManager.stopUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .cantRecording)
                    self?.scansButton.fadeIn()
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
                      let lidarData = lidarData,
                      let locationData = self?.viewModel?.currentLocation else { return }
                
                self?.popupSelectLocationVC(lidarData, locationData)
            })
            .store(in: &self.cancellables)
    }
    
    private func bindUploadSuccess() {
        self.viewModel?.$uploadSuccess
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] success in
                guard success else { return }
                
                self?.showAlert(title: "Upload Success", text: "You can see the record historys in the SCANS page")
                self?.viewModel?.changeMode()
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
    
    private func bindProcessWithStorageData() {
        self.viewModel?.$processWithStorageData
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] isStorage in
                if isStorage {
                    self?.showAlertWithStorageStart()
                }
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
                } else {
                    self?.showAlertAndTerminate()
                }
            })
            .store(in: &self.cancellables)
    }
}

// MARK: Action & Logic
extension MainVC {
    /// RecordingButton Tab 액션
    @objc private func tapRecordingButton(_ sender: UIButton) {
        self.viewModel?.changeMode()
    }
    
    /// ScansButton Tab 액션
    private func moveToScansVC() {
        let scansVC = UIStoryboard(name: "Main", bundle: nil).instantiateViewController(withIdentifier: ScansVC.identifier)
        self.navigationController?.pushViewController(scansVC, animated: true)
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
}

// MARK: 위치정보 활성화표출 및 위치정보 수신 부분
extension MainVC: CLLocationManagerDelegate {
    /// 앱이 위치 관리자를 생성할 때와 권한 부여 상태가 변경될 때 delegate 에게 알립니다.
    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        switch manager.authorizationStatus {
        case .authorizedWhenInUse:
            // Location services are available.
            self.checkLidarSensor()
            if self.viewModel?.mode != .cantRecord {
                self.viewModel?.readyForRecording()
            }
            break
            
        case .restricted, .denied:
            // Location services currently unavailable.
            self.showGPSWarning()
            self.viewModel?.cantGetGPS()
            break
            
        case .notDetermined:
            // Authorization not determined yet.
            manager.requestWhenInUseAuthorization()
            self.viewModel?.cantGetGPS()
            break
            
        default:
            break
        }
    }
    
    /// 새 위치데이터를 수신받은 경우 delegate 에게 전달합니다.
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        // 가장 최근의 위치 데이터는 마지막 값이므로 마지막값을 사용하여 viewModel 로 전달
        guard let location = locations.last else { return }
        self.viewModel?.appendLocation(location)
    }
}

// MARK: Present
extension MainVC {
    /// SelectLocationVC 를 Modal 형식으로 띄우는 함수
    private func popupSelectLocationVC(_ lidarData: LiDARData, _ locationData: LocationData) {
        let storyboard: UIStoryboard = UIStoryboard(name: "Main", bundle: nil)
        
        if let vc = storyboard.instantiateViewController(identifier: SelectLocationVC.identifier) as? SelectLocationVC {
            let AddressRepository = AddressRepository()
            let buildingRepository = BuildingInMapRepository()
            let viewModel = SelectLocationVM(lidarData: lidarData, locationData: locationData, addressRepository: AddressRepository, buildingRepository: buildingRepository)
            // delegate 로 MainVC 전달
            vc.configureDelegate(self)
            // viewModel 로 측정된 liDarData, locationData 전달
            vc.configureViewModel(viewModel)
            // model 이 제스처로 닫히지 않도록 설정
            vc.isModalInPresentation = true
            
            self.present(vc, animated: true)
        }
    }
    
    /// 앱의 임시데이터로 복구할지 여부를 띄우는 함수
    private func showAlertWithStorageStart() {
        let alert = UIAlertController(title: "임시저장된 데이터가 존재합니다. 복구하시겠습니까?", message: "복구하지 않는 경우 해당데이터는 제거됩니다.", preferredStyle: .alert)
        let remove = UIAlertAction(title: "제거", style: .destructive) { [weak self] _ in
            self?.viewModel?.resetStorageAndStart()
        }
        let load = UIAlertAction(title: "복구", style: .default) { [weak self] _ in
            self?.viewModel?.loadFromStorage()
        }
        
        alert.addAction(remove)
        alert.addAction(load)
        
        self.present(alert, animated: true)
    }
    
    /// 업로드 실패 및 메모리 부족으로 인해 임시데이터가 저장된 후 앱 종료메세지를 띄우는 함수
    private func showAlertAndTerminate() {
        self.statusLabel.changeText(to: .removed)
        let title = self.viewModel?.mode == .recordingTerminate ? "Low Memory Warning" : "Upload Failed (\(self.viewModel?.networkErrorMessage ?? ""))"
        let alert = UIAlertController(title: title, message: "앱을 재 실행 하시기 바랍니다.", preferredStyle: .alert)
        let ok = UIAlertAction(title: "확인", style: .default) { _ in
            UIApplication.shared.perform(#selector(NSXPCConnection.suspend))
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
                exit(0)
            }
        }
        
        alert.addAction(ok)
        self.present(alert, animated: true)
    }
}



// MARK: - SelectLocationVC 에서 MainVC 의 함수 일부를 위임받는 부분
extension MainVC: SelectLocationDelegate {
    /// 측정된 데이터 서버 업로드 취소 함수
    func uploadCancel() {
        self.viewModel?.uploadCancel()
        self.showAlert(title: "Upload Canceled", text: "")
        self.viewModel?.changeMode()
    }
    
    /// 업로드 데이터들을 수신받아 업로드를 실행하는 함수
    func uploadMeasuredData(location: LocationData, buildingInfo: BuildingOfMapInfo, floor: Int, lidarName: String) {
        self.viewModel?.changeMode()
        self.viewModel?.uploadMeasuredData(location: location, buildingInfo: buildingInfo, floor: floor, lidarName: lidarName)
    }
}

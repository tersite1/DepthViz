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

final class MainVC: UIViewController, ARSessionDelegate {
    /// 기록측정 및 종료 버튼
    private let recordingButton = RecordingButton()
    /// 현재 동작상태 표시 텍스트
    private let statusLabel = StatusIndicatorLabel()
    /// 현재 측정중인 Point Cloud 개수 표시 뷰
    private let pointCloudCountView = PointCloudCountView()
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
        self.configureLocationManager()
        self.bindViewModel()
    }
    
    /// MainVC 화면 진입시 configure
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.configureARWorldTracking()
    }
}

// MARK: Configure
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
    }
    
    /// View를 나타내기 위한 데이터 처리담당 설정 함수
    private func configureViewModel() {
        guard let device = MTLCreateSystemDefaultDevice() else {
            print("Metal is not supported on this device")
            return
        }
        
        // Set the view to use the default device
        if let view = view as? MTKView {
            view.device = device
            
            view.backgroundColor = UIColor.clear
            // we need this to enable depth test
            view.depthStencilPixelFormat = .depth32Float
            view.contentScaleFactor = 1
            view.delegate = self
            
            // Configure the ViewModel, Renderer to draw to the view
            self.viewModel = MainVM(session: self.session, device: device, view: view)
        }
    }
    
    /// gps 값 수신을 위한 설정 함수
    private func configureLocationManager() {
        self.locationManager.delegate = self
    }
    
    /// session 설정 및 화면꺼짐방지
    private func configureARWorldTracking() {
        // Create a world-tracking configuration, and
        // enable the scene depth frame-semantic.
        let configuration = ARWorldTrackingConfiguration()
        configuration.frameSemantics = [.sceneDepth, .smoothedSceneDepth]

        // Run the view's session
        self.session.run(configuration)
        
        // The screen shouldn't dim during AR experiences.
        UIApplication.shared.isIdleTimerDisabled = true
    }
    
    /// 기록측정 불가 상태의 UI 표시 함수
    private func configureCantRecording() {
        // MARK: 기록측정불가 UI 구현 필요
        print("cant Recording")
    }
    
    private func configureCantGetGPS() {
        // MARK: GPS 수신불가 UI 구현 필요
        print("cant get gps")
    }
}

// MARK: INPUT
extension MainVC {
    /// viewModel 에서 값 변화를 수신하기 위한 함수
    private func bindViewModel() {
        self.bindMode()
        self.bindPointCount()
        self.bindLidarData()
        self.bindCurrentLocation()
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
                case .recording:
                    self?.locationManager.startUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .recording)
                case .loading:
                    self?.locationManager.stopUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .loading)
                case .cantRecord:
                    self?.locationManager.stopUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .cantRecording)
                    self?.configureCantRecording()
                case .cantGetGPS:
                    self?.locationManager.stopUpdatingLocation()
                    self?.recordingButton.changeStatus(to: .cantRecording)
                    self?.configureCantGetGPS()
                default:
                    self?.locationManager.stopUpdatingLocation()
                    return
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
                guard let lidarData = lidarData else { return }
                // MARK: Make SelectLocationVM, Show SelectLocationVC
                dump(lidarData)
            })
            .store(in: &self.cancellables)
    }
    
    /// viewModel 의 currentLocation 값 변화를 수신하여 SelectLocationVC 로 전달하기 위한 함수
    func bindCurrentLocation() {
        self.viewModel?.$currentLocation
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] location in
                guard let location = location else { return }
                // MARK: Make SelectLocationVM, Show SelectLocationVC
                dump(location)
            })
            .store(in: &self.cancellables)
    }
}

// MARK: Action
extension MainVC {
    /// RecordingButton Tab 액션
    @objc private func tapRecordingButton(_ sender: UIButton) {
        self.viewModel?.changeMode()
    }
}

extension MainVC {
    private func changeStatusLabel(to mode: MainVM.Mode) {
        switch mode {
        case .ready:
            self.statusLabel.changeText(to: .readyForRecording)
        case .recording:
            self.statusLabel.changeText(to: .recording)
        case .loading:
            self.statusLabel.changeText(to: .loading)
        case .uploading:
            self.statusLabel.changeText(to: .uploading)
        case .cantRecord, .cantGetGPS:
            self.statusLabel.changeText(to: .removed)
        }
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

// MARK: MTKViewDelegate
extension MainVC: MTKViewDelegate {
    // Called whenever view changes orientation or layout is changed
    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        self.viewModel?.rendererResize(to: size)
    }
    
    // Called whenever the view needs to render
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

extension MTKView: RenderDestinationProvider {
    
}

// MARK: SAVE PLY
extension MainVC {
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        
        self.memoryAlert()
        self.viewModel?.terminateRecording()
    }
    
    private func memoryAlert() {
        let alert = UIAlertController(title: "Low Memory Warning", message: "The recording has been paused. Do not quit the app until all files have been saved.", preferredStyle: .alert)
        alert.addAction(UIAlertAction(title: NSLocalizedString("OK", comment: "Default action"), style: .default, handler: { _ in
            NSLog("The \"OK\" alert occured.")
        }))
        self.present(alert, animated: true, completion: nil)
    }
}

// update textlabel on tasks start/finish
extension MainVC: TaskDelegate {
    func showUploadResult(result: NetworkResult) {
        DispatchQueue.main.async { [weak self] in
            self?.statusLabel.changeText(to: .removed)
            self?.recordingButton.changeStatus(to: .ready)
            switch result.status {
            case .SUCCESS:
                self?.showAlert(title: "Upload Success", text: "You can see the record historys in the SCANS page")
            case .ERROR:
                self?.showAlert(title: "Upload Fail", text: "\(result.status.rawValue)")
            }
            
        }
    }
    
    func sharePLY(file: Any) {
        let activityViewController = UIActivityViewController(activityItems: [file], applicationActivities: nil)
        
        if UIDevice.current.userInterfaceIdiom == .pad {
            activityViewController.popoverPresentationController?.sourceView = self.view
            activityViewController.popoverPresentationController?.sourceRect = self.recordingButton.frame
        }
        
        DispatchQueue.main.async {
            self.present(activityViewController, animated: true) { [weak self] in
                self?.showUploadResult(result: NetworkResult(data: nil, status: .SUCCESS))
            }
        }
    }
    
    func startMakingPlyFile() {
        DispatchQueue.main.async { [weak self] in
            self?.statusLabel.changeText(to: .loading)
        }
    }
    
    func finishMakingPlyFile() {
        print("finish making ply file")
    }
    
    func startUploadingData() {
        DispatchQueue.main.async { [weak self] in
            self?.statusLabel.changeText(to: .uploading)
        }
    }
    
    func showShareOrUpload(stringData: String, fileName: String) {
        DispatchQueue.main.async { [weak self] in
            let alert = UIAlertController(title: "Share OR Upload", message: "AirDrop 공유, 또는 서버 업로드 선택", preferredStyle: .alert)
            let share = UIAlertAction(title: "Share", style: .default) { [weak self] _ in
                DispatchQueue.global().async { [weak self] in
                    if let file = shareFile(content: stringData, filename: fileName, folder: "") {
                        self?.sharePLY(file: file)
                    }
                }
            }
            let upload = UIAlertAction(title: "Upload", style: .default) { [weak self] _ in
                if let fileData = stringData.data(using: .utf8) {
                    let apiService = MainApiService()
                    apiService.uploadPlyData(fileName: fileName, fileData: fileData) { [weak self] result in
                        self?.showUploadResult(result: result)
                    }
                }
            }
            alert.addAction(share)
            alert.addAction(upload)
            self?.present(alert, animated: true)
        }
    }
}

// MARK: Location-related properties and delegate methods.
extension MainVC: CLLocationManagerDelegate {
    /// 앱이 위치 관리자를 생성할 때와 권한 부여 상태가 변경될 때 delegate 에게 알립니다.
    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        switch manager.authorizationStatus {
        case .authorizedWhenInUse:
            // Location services are available.
            self.viewModel?.readyForRecording()
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

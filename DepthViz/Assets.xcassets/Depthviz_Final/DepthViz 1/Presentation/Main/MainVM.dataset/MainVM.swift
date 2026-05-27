import Foundation
import ARKit
import MetalKit
import Combine

/// 메인화면의 View 를 나타내기 위한 데이터 처리 담당
final class MainVM {
    enum Mode {
        case cantRecord // LiDAR 스캔이 불가능한 경우
        case cantGetGPS // 위치정보를 받을 수 없는 경우
        case ready // LiDAR 측정 전 상태
        case recording // LiDAR 측정중 상태
        case loading // LiDAR 측정종료 및 데이터생성중 상태
        case uploading // 데이터 업로드중 상태
        case recordingTerminate
        case uploadingTerminate
    }
    
    // MARK: OUTPUT
    @Published private(set) var mode: Mode = .ready
    @Published private(set) var pointCount: Int = 0
    @Published private(set) var lidarData: LiDARData?
    @Published private(set) var uploadSuccess: Bool = false {
        didSet {
            if uploadSuccess {
                UserDefaults.standard.setValue(false, forKey: "lidarData")
            }
        }
    }
    @Published private(set) var uploadProgress: Double = 0
    @Published private(set) var processWithStorageData: Bool = false
    @Published private(set) var saveToStorageSuccess: Bool?
    private(set) var networkErrorMessage: String = ""
    @Published private(set) var scanData: ScanData?
    @Published private(set) var saveScanDataSuccess: Bool?

    private let renderer: Renderer
    private var cancellables: Set<AnyCancellable> = []

    init(session: ARSession, device: MTLDevice, view: MTKView) {
        self.renderer = Renderer(session: session, metalDevice: device, renderDestination: view)
        self.renderer.drawRectResized(size: view.bounds.size)
        self.bindRenderer()

        if UserDefaults.standard.bool(forKey: "lidarData") == true {
            self.mode = .loading
            self.processWithStorageData = true
        } else {
            self.mode = .ready
        }
    }
}

// MARK: INPUT
extension MainVM {
    func changeMode() {
        switch self.mode {
        case .ready:
            self.startRecording()
            self.mode = .recording
        case .recording:
            self.stopRecording()
            self.mode = .loading
        case .loading:
            self.mode = .uploading
        case .uploading:
            self.resetRenderer()
            self.mode = .ready
        default:
            return
        }
    }
    
    func rendererResize(to size: CGSize) {
        self.renderer.drawRectResized(size: size)
    }

    func rendererDraw() {
        self.renderer.draw()
    }
    
    func terminateRecording() {
        self.mode = .recordingTerminate
        self.stopRecording()
    }
    
    func readyForRecording() {
        self.mode = .ready
    }
    
    func cantRecording() {
        self.mode = .cantRecord
    }
    
    func cantGetGPS() {
        self.mode = .cantGetGPS
    }
    
    func appendLocation(_ location: CLLocation) {
    }
    
    func uploadCancel() {
        self.resetRenderer()
        self.mode = .uploading
    }
    
    func saveScanData(fileName: String? = nil) {
        if var scanData = self.scanData {
            if let fileName = fileName {
                scanData.rename(to: fileName)
            }

            // ScanData -> ScanInfo 변환
            let scanInfo = scanData.info  // ScanData에서 이미 ScanInfo로 변환할 수 있도록 구현됨
            let success = ScanStorage.shared.save(scanInfo, projectName: "YourProjectName", date: Date())
            self.saveScanDataSuccess = success

            // mode 변경
            self.resetRenderer()
            self.mode = .ready
            self.scanData = nil
        }
    }
}

// MARK: - Renderer 관련 메서드
extension MainVM {
    private func bindRenderer() {
        self.renderer.$currentPointCount
            .receive(on: DispatchQueue.global())
            .sink { [weak self] count in
                self?.pointCount = count
            }
            .store(in: &self.cancellables)
        
        self.renderer.$lidarRawStringData
            .receive(on: DispatchQueue.global())
            .sink { [weak self] rawStringData in
                guard let rawStringData = rawStringData,
                      let pointCount = self?.renderer.currentPointCount else { return }

                self?.scanData = ScanData(rawStringData: rawStringData, pointCount: pointCount)
            }
            .store(in: &self.cancellables)
    }

    private func startRecording() {
        renderer.isRecording = true
    }

    private func stopRecording() {
        renderer.isRecording = false
        renderer.savePointCloud()
    }

    private func resetRenderer() {
        renderer.clearParticles()
    }
}

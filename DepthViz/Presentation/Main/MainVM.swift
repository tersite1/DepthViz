import Foundation
import ARKit
import MetalKit
import Combine

final class MainVM {
    enum Mode {
        case cantRecord
        case cantGetGPS
        case ready
        case recording
        case loading
        case uploading
        case recordingTerminate
        case uploadingTerminate
    }

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

    let renderer: Renderer
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

extension MainVM {
    func changeMode() {
        switch self.mode {
        case .ready:
            self.startRecording()
            self.changeMode(to: .recording)
        case .recording:
            self.stopRecording()
            // DV-SLAM: "최적화 수행 중..." 표시, ARKit: "처리 중..." 표시
            if ScanSettings.shared.algorithm == .depthViz {
                self.changeMode(to: .recordingTerminate)
            } else {
                self.changeMode(to: .loading)
            }
        case .loading:
            self.changeMode(to: .uploading)
        case .uploading:
            self.resetRenderer()
            self.changeMode(to: .ready)
        default:
            return
        }
    }

    func changeMode(to newMode: Mode) {
        self.mode = newMode
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
        self.changeMode(to: .uploading)
    }

    func resetRenderer() {
        self.renderer.clearParticles()
        self.renderer.isRecording = false
        self.scanData = nil
    }

    func saveScanData(fileName: String? = nil) {
        if var scanData = self.scanData {
            if let fileName = fileName {
                scanData.rename(to: fileName)
            }

            let success = ScanStorage.shared.save(scanData)
            self.saveScanDataSuccess = success

            self.resetRenderer()
            self.changeMode(to: .ready)
            self.scanData = nil
        }
    }
}

extension MainVM {
    private func bindRenderer() {
        self.renderer.$currentPointCount
            .receive(on: DispatchQueue.main)
            .sink { [weak self] count in
                self?.pointCount = count
            }
            .store(in: &self.cancellables)

        // Handle binary Data from PointCloudExporter (new fast path)
        self.renderer.$lidarRawData
            .receive(on: DispatchQueue.main)
            .sink { [weak self] rawData in
                guard let self = self,
                      let rawData = rawData else { return }

                let pointCount = self.renderer.currentPointCount
                let bcf = ByteCountFormatter()
                bcf.allowedUnits = [.useMB]
                bcf.countStyle = .file
                let fileSize = bcf.string(fromByteCount: Int64(rawData.count))

                let dateFormatter = DateFormatter()
                dateFormatter.dateFormat = "yyyy_MMdd_HHmmss"
                let timestamp = dateFormatter.string(from: Date())
                let ext = ScanSettings.shared.fileFormat.fileExtension

                let scan = ScanData(
                    date: Date(),
                    fileName: "\(timestamp)_Scan.\(ext)",
                    lidarData: rawData,
                    fileSize: fileSize,
                    points: pointCount,
                    project: "Default"
                )
                scan.startCameraTransform = self.renderer.getStartCameraTransform()
                scan.trajectoryPoints = self.renderer.getTrajectoryPoints()
                self.scanData = scan
            }
            .store(in: &self.cancellables)

        // Legacy string path (fallback for ASCII formats)
        self.renderer.$lidarRawStringData
            .receive(on: DispatchQueue.main)
            .sink { [weak self] rawStringData in
                guard let self = self,
                      let rawStringData = rawStringData else { return }

                let pointCount = self.renderer.currentPointCount
                let lidarData = LiDARData(rawStringData: rawStringData, pointCount: pointCount)

                let dateFormatter = DateFormatter()
                dateFormatter.dateFormat = "yyyy_MMdd_HHmmss"
                let timestamp = dateFormatter.string(from: Date())

                let scan = ScanData(
                    date: Date(),
                    fileName: "\(timestamp)_Scan.ply",
                    lidarData: lidarData.lidarData,
                    fileSize: lidarData.lidarFileSize,
                    points: lidarData.pointCount,
                    project: "Default"
                )
                scan.startCameraTransform = self.renderer.getStartCameraTransform()
                scan.trajectoryPoints = self.renderer.getTrajectoryPoints()
                self.scanData = scan
            }
            .store(in: &self.cancellables)
    }

    private func startRecording() {
        // 녹화 직전 최신 설정 반영 (confidence, 알고리즘, 그리드 포인트 수)
        self.renderer.applySettings()
        self.renderer.isRecording = true

        let algorithm = ScanSettings.shared.algorithm
        if algorithm == .depthViz {
            // DV-SLAM: SLAM 엔진 + IMU 시작 (LIO = LiDAR-Inertial Odometry)
            SLAMService.sharedInstance().reloadSettings()
            SLAMService.sharedInstance().delegate = self.renderer
            SLAMService.sharedInstance().start()
            self.renderer.startIMUForSLAM()
            print("🔬 DV-SLAM 엔진 + IMU(100Hz) 시작")
        } else {
            // ARKit 모드: 프리미엄 IMU 표시/CSV 로깅을 위해 IMU는 시작
            if PremiumManager.shared.isPremium {
                self.renderer.startIMUForSLAM()
                print("📱 ARKit 모드 + 프리미엄 IMU(100Hz) 시작")
            } else {
                print("📱 ARKit 모드 (SLAM/IMU 비활성)")
            }
        }
    }

    private func stopRecording() {
        self.renderer.isRecording = false

        let algorithm = ScanSettings.shared.algorithm
        // IMU는 DV-SLAM, ARKit+프리미엄 둘 다 시작하므로 항상 정지
        self.renderer.stopIMUForSLAM()
        if algorithm == .depthViz {
            SLAMService.sharedInstance().stop()
        }

        // 후처리 최적화 → 내보내기 (두 모드 공통)
        self.renderer.optimizeAndExport(useSLAM: algorithm == .depthViz)
    }
}

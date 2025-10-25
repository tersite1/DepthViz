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

extension MainVM {
    func changeMode() {
        switch self.mode {
        case .ready:
            self.startRecording()
            self.changeMode(to: .recording)
        case .recording:
            self.stopRecording()
            self.changeMode(to: .loading)
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
            .receive(on: DispatchQueue.global())
            .sink { [weak self] count in
                self?.pointCount = count
            }
            .store(in: &self.cancellables)
        
        self.renderer.$lidarRawStringData
            .receive(on: DispatchQueue.global())
            .sink { [weak self] rawStringData in
                guard let self = self,
                      let rawStringData = rawStringData else { return }
                
                let pointCount = self.renderer.currentPointCount // 옵셔널 바인딩 제거
                let lidarData = LiDARData(rawStringData: rawStringData, pointCount: pointCount)
                
                self.scanData = ScanData(
                    date: Date(),
                    fileName: "ExampleFileName",
                    lidarData: lidarData.lidarData,
                    fileSize: lidarData.lidarFileSize,
                    points: lidarData.pointCount,
                    project: "ExampleProject"
                )
            }
            .store(in: &self.cancellables)
    }

    private func startRecording() {
        self.renderer.isRecording = true
    }

    private func stopRecording() {
        self.renderer.isRecording = false
        self.renderer.savePointCloud()
    }
}
